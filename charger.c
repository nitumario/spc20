/*
 * charger.c — Charge Controller Implementation (Pipeline Step 7)
 * =========================================================================
 *
 * Responsibilities per tick (50 ms):
 *   1. Decide whether the charger should run at all. It runs only when
 *      energy_mode has activated the region (EM_CHARGE_ONLY or
 *      EM_CHARGE_AND_LOAD) AND no charge-blocking fault is latched.
 *   2. From CHG_INACTIVE, self-activate into PRECHARGE (if V_bat < 3000
 *      mV) or CC (otherwise) — this is the "activated" transition from
 *      charger_states.csv.
 *   3. Evaluate in-state transition guards (V_bat thresholds, taper
 *      completion, precharge timeout).
 *   4. Regulate ctx->pwm:
 *        - Panel-safety first: if V_panel < PANEL_SAFETY_MV, back off by
 *          PANEL_BACKOFF_STEP and skip the rest of regulation (a hard
 *          emergency floor below the regulation band).
 *        - If MPPT is TRACKING, skip regulation (MPPT owns PWM). With
 *          CHARGER_INPUT_VREG=1 this NEVER fires: the setpoint-P&O MPPT
 *          owns only the vreg setpoint, and this inner loop keeps PWM
 *          ownership at all times (it is the muscle that realises each
 *          MPPT probe).
 *        - Bulk (PRECHARGE/CC): cc_regulate. Under CHARGER_INPUT_VREG it
 *          holds V_panel at ctx->mppt.vreg_setpoint_mv (clamped by
 *          allowed_chg); otherwise it's the legacy fixed-current CC loop.
 *        - CV: cv_regulate steps pwm by CV_PWM_STEP toward BAT_CV_VOLTAGE_MV
 *          using CV_DEADBAND_MV as a stability band.
 *
 * PWM continuity
 * --------------
 * Per the README/charger_states note, "PWM is not reset on state entry,
 * it is saved from previous state." On first activation, pwm was reset
 * to PWM_MIN_DUTY by energy_mode's deactivate_charger_region() — the
 * first few ticks ramp it down gradually. That's intentional: a slow
 * start-up limits inrush current.
 *
 * Sign convention
 * ---------------
 * - Lower pwm value  = higher duty cycle = more buck output current.
 * - Higher pwm value = lower duty cycle  = less current (pwm=399 → off).
 * - To INCREASE charge current (pull V_panel DOWN): pwm -= step.
 * - To DECREASE charge current (let V_panel RISE):  pwm += step.
 *
 * This is dispositive (do NOT be misled by pwm↔current correlations read
 * off a bouncing/limit-cycling log — those are sampling artifacts and can
 * appear with either slope): PWM_MIN_DUTY = 399 is the OFF state (boot and
 * deactivate park pwm there; at IDLE pwm=399, Ichg=0), so turning the buck
 * ON necessarily moves pwm DOWN. Confirmed by the clean IDLE→CHG step
 * (pwm 399→46, Ichg 0→144) and the descending output_voltages_buck_mV LUT.
 *
 * Units
 * -----
 * Voltages in mV, currents in mA, times in ms.
 */

#include "charger.h"
#include "fault_mgr.h"
#include "SPCBoardAPI.h"

/* Any of these latched faults should block the charger from running.
 * (Undervolt is excluded: by the time V_bat is that low, energy_mode
 *  is already in SAFE_MODE which deactivates the charger.) */
#define CHG_FAULT_BLOCK_MASK   \
    (FAULT_OVERTEMP            \
   | FAULT_BAT_OVERVOLT        \
   | FAULT_OVERCURRENT_CHG     \
   | FAULT_PRECHARGE_TIMEOUT   \
   | FAULT_TEMP_CHARGE_BLOCK)

/* =========================================================================
 * PWM helpers
 * ========================================================================= */

static inline uint16_t pwm_clamp(int32_t p)
{
    if (p < PWM_MAX_DUTY) return PWM_MAX_DUTY;   /* 1   — highest duty */
    if (p > PWM_MIN_DUTY) return PWM_MIN_DUTY;   /* 399 — effectively off */
    return (uint16_t)p;
}

static inline void pwm_step(system_ctx_t *ctx, int32_t delta)
{
    ctx->pwm = pwm_clamp((int32_t)ctx->pwm + delta);
}

/* =========================================================================
 * STATE ENTRY ACTIONS
 * =========================================================================
 *
 * No PWM changes here — the previous PWM value is intentionally retained
 * so the buck can smoothly hand off between PRECHARGE → CC → CV.
 */

static void enter_precharge(system_ctx_t *ctx)
{
    ctx->charger.state = CHG_PRECHARGE;
    ctx->charger.precharge_start_ms = time_now();

    /* Taper timing is only meaningful in CV. Reset defensively. */
    ctx->charger.bat_full_timing    = false;
    ctx->charger.bat_full_signaled  = false;
}

static void enter_cc(system_ctx_t *ctx)
{
    ctx->charger.state = CHG_CC;
    ctx->charger.precharge_start_ms = 0;

    ctx->charger.bat_full_timing    = false;
    ctx->charger.bat_full_signaled  = false;
}

static void enter_cv(system_ctx_t *ctx)
{
    ctx->charger.state = CHG_CV;

    /* Start taper watch fresh on CV entry. */
    ctx->charger.bat_full_timing    = false;
    ctx->charger.bat_full_signaled  = false;
}

/* =========================================================================
 * SHARED REGULATION HELPERS
 * =========================================================================
 */

/*
 * panel_safety_backoff — if V_panel has collapsed below PANEL_SAFETY_MV
 * we are drawing too much current from the panel. Quickly reduce current
 * (pwm += several, toward the PWM_MIN_DUTY=399 off state) and skip
 * regulation. Returns true if backoff fired.
 *
 * This is a one-sided EMERGENCY floor *below* the cc_regulate setpoint
 * band (PANEL_SAFETY_MV 4800 < band low edge 5300), so with the loop
 * parked on the panel's power plateau it never fires.
 *
 * Under CHARGER_INPUT_VREG it is PACED to PANEL_VREG_INTERVAL_MS via the
 * shared regulation timer. V_panel lags ~320 ms through the ADC moving
 * average, so the original fire-every-tick version kept stepping ~6 ticks
 * past the true recovery point (+30 counts ≈ 1.3 A of demand removed on
 * the ~65 mΩ path) — parking the buck rail far BELOW V_bat, i.e. deep in
 * the reverse-pump zone. The May bench log shows the result: Ichg
 * −1010 mA with V_panel pushed to 14.1 V, ABOVE open-circuit, climbing
 * toward the TPS564247's ~17 V input ceiling. One +5 step per settle
 * window (~215 mA of demand cut) is still the strongest actor in the
 * loop and clears a collapse in 1–2 intervals without the overshoot.
 * While a collapse reading persists between paced steps we still return
 * true, suppressing the regulator (never step toward MORE current on a
 * collapsed-panel reading).
 *
 * Runs BEFORE the MPPT check so the panel is protected even during MPPT
 * TRACKING perturbations (when CHARGER_INPUT_VREG=0, where it keeps the
 * legacy every-tick behaviour).
 */
static bool panel_safety_backoff(system_ctx_t *ctx)
{
    if (ctx->meas.panel_voltage >= PANEL_SAFETY_MV)
        return false;

#if CHARGER_INPUT_VREG
    uint32_t now = time_now();
    if ((now - ctx->charger.cc_last_downstep_ms) < PANEL_VREG_INTERVAL_MS)
        return true;   /* collapse reading: hold PWM, wait out the filter */
    ctx->charger.cc_last_downstep_ms = now;
#endif

    pwm_step(ctx, +PANEL_BACKOFF_STEP);  /* reduce current → V_panel recovers */
    return true;
}

/*
 * mppt_owns_pwm — the charger yields PWM control while MPPT is actively
 * perturbing. Transition checks above still run; only the regulator
 * step is skipped.
 *
 * Under CHARGER_INPUT_VREG this is ALWAYS false: the setpoint-P&O MPPT
 * perturbs ctx->mppt.vreg_setpoint_mv and relies on cc_regulate to
 * realise each probe — yielding here during TRACKING would freeze the
 * plant at whatever pwm the dwell started with and the tracker would
 * observe nothing. Only the legacy PWM-perturbing tracker takes PWM.
 */
static inline bool mppt_owns_pwm(const system_ctx_t *ctx)
{
#if CHARGER_INPUT_VREG
    (void)ctx;
    return false;
#else
    return (ctx->mppt.state == MPPT_TRACKING);
#endif
}

/*
 * cc_regulate — bulk-charge regulator. Called from both PRECHARGE and CC
 * states (their regulation is identical; the "precharge" nature is
 * enforced by power_budget clamping allowed_chg to ≤200 mA below 3 V).
 *
 * Two implementations, selected by CHARGER_INPUT_VREG (hw_config.h):
 */
#if CHARGER_INPUT_VREG
/*
 * INPUT-VOLTAGE-REGULATED bulk charging ("constant-voltage MPPT").
 *
 * Hold V_panel at ctx->mppt.vreg_setpoint_mv (the panel's MPP voltage,
 * owned and hill-climbed by the outer MPPT loop in mppt.c). The current
 * drawn is then whatever the panel delivers at that voltage, which IS
 * the MPP current — no fixed current target to chase, so the loop
 * cannot command more than the panel can give and walk it off the
 * I-V knee.
 *
 * Priority:
 *   1. Battery-intake clamp (hard): if chg_current exceeds allowed_chg,
 *      reduce current regardless of V_panel. allowed_chg already encodes
 *      the precharge trickle / CC-zone / CV-taper limits (power_budget).
 *   2. Panel-voltage loop: otherwise steer V_panel toward the setpoint.
 *
 * PWM sign (the structural truth, NOT the limit-cycle correlation that
 * misled an earlier attempt): pwm = PWM_MIN_DUTY (399) is the off/idle
 * state, so turning the buck ON and drawing current means pwm goes DOWN.
 *   - To DRAW MORE current / pull V_panel DOWN: pwm -= step.
 *   - To DRAW LESS current / let V_panel RISE:  pwm += step.
 * (Confirmed by the clean IDLE→CHG transition pwm 399→46 / Ichg 0→144,
 *  and the descending output_voltages_buck_mV calibration LUT.)
 *
 * LOOP PACING — critical. The panel ADCs are 64-sample moving averages
 * (~320 ms group delay). Stepping every 50 ms tick gives the loop ~6
 * ticks of dead time, so it overshoots the setpoint by ~6 steps before
 * the feedback arrives — and on the steep panel I-V curve that overshoot
 * swings V_panel from ~8 V clear down past the knee to ~3.4 V and back,
 * a violent limit cycle that (in EITHER sign) collapses the panel and
 * clears has_sun. So we take at most ONE step per PANEL_VREG_INTERVAL_MS
 * (≥ the ADC settle time): each step's effect is observed before the
 * next, and the loop converges monotonically to the setpoint with no
 * overshoot. Negative feedback on a monotonic plant (more current →
 * lower V_panel), stable from either side of the MPP.
 *
 * On a stiff source V_panel never falls to the setpoint, so the loop just
 * keeps drawing more until clamp 1 (allowed_chg) catches it — i.e. it
 * degrades to classic current-limited CC with no special-casing.
 */
static void cc_regulate(system_ctx_t *ctx)
{
    /* Pace to the ADC settle time — see the dead-time note above. The
     * timer (cc_last_downstep_ms) is armed on charger activation, so the
     * first step lands one interval after bring-up, by which point the
     * lagged V_panel reading reflects the pre-positioned operating point
     * rather than the stale open-circuit voltage from the IDLE phase. */
    uint32_t now = time_now();
    if ((now - ctx->charger.cc_last_downstep_ms) < PANEL_VREG_INTERVAL_MS)
        return;
    ctx->charger.cc_last_downstep_ms = now;

    int32_t i_chg    = (int32_t)ctx->meas.chg_current;
    int32_t i_limit  = (int32_t)ctx->allowed_chg;
    int32_t v_panel  = (int32_t)ctx->meas.panel_voltage;

    /* Clamp 0: reverse-current escape. Negative chg_current means the
     * buck rail is parked BELOW V_bat and the sync FETs are pumping
     * battery charge into the panel — which can push V_panel above
     * open-circuit toward the TPS564247 input ceiling. The V_panel loop
     * below does walk out of this state on its own (panel reads high →
     * pwm DOWN), but only at PANEL_VREG_STEP per interval — seconds of
     * sustained reverse current after a backoff overshoot. Escape at the
     * backoff rate instead. Same direction the V loop would pick, so the
     * two can never fight. */
    if (i_chg < -(int32_t)CHG_REVERSE_CURRENT_MA) {
        pwm_step(ctx, -PANEL_BACKOFF_STEP);
        return;
    }

    /* Clamp 1: never exceed the battery's allowed intake. Over-current
     * always wins → reduce current → pwm UP (toward off). */
    if (i_chg > i_limit + CC_DEADBAND_MA) {
        pwm_step(ctx, +PANEL_VREG_STEP);
        return;
    }

    /* Clamp 2: regulate the panel to the MPP setpoint. The target is the
     * LIVE per-panel value owned by the outer MPPT loop (seeded from
     * 0.76·Voc at activation, then hill-climbed — see mppt.c), NOT the
     * static PANEL_VREG_SETPOINT_MV, which is only its cold-boot seed. */
    int32_t v_sp = (int32_t)ctx->mppt.vreg_setpoint_mv;
    if (v_panel < v_sp - (int32_t)PANEL_VREG_DEADBAND_MV) {
        /* Sagging below MPP → drawing too much → DRAW LESS (pwm UP) → V recovers. */
        pwm_step(ctx, +PANEL_VREG_STEP);
    } else if (v_panel > v_sp + (int32_t)PANEL_VREG_DEADBAND_MV) {
        /* Above MPP with current-headroom (clamp 1 didn't fire) → DRAW MORE (pwm DOWN) → V falls. */
        pwm_step(ctx, -PANEL_VREG_STEP);
    }
    /* Within the deadband around the MPP → stable, hold PWM. */
}
#else
/*
 * Legacy fixed-current CC: bang-bang on chg_current toward allowed_chg.
 * Open-loop unstable on a soft PV source (see CHARGER_INPUT_VREG notes);
 * retained for stiff-source bring-up / regression comparison.
 */
static void cc_regulate(system_ctx_t *ctx)
{
    /* err > 0 means we are OVER target (too much current). */
    int32_t target = (int32_t)ctx->allowed_chg;
    int32_t actual = (int32_t)ctx->meas.chg_current;
    int32_t err    = actual - target;

    if (err < -CC_DEADBAND_MA) {
        /* Under-current: push harder. Lower pwm = higher duty.
         *
         * Rate-limit DOWN-steps to CC_DOWNSTEP_INTERVAL_MS. The
         * chg_current ADC has ~320 ms group delay from its 64-sample
         * moving average; stepping every 50 ms walks the buck 6+
         * PWM counts past the regulation point before the filter
         * catches up. With a current-limited PV panel that's masked
         * by V_panel collapse → panel_safety_backoff. With a stiff
         * source (bench PSU), it overshoots FAULT_OVERCURRENT_CHG_MA
         * before any feedback arrives. */
        uint32_t now = time_now();
        if ((now - ctx->charger.cc_last_downstep_ms) >= CC_DOWNSTEP_INTERVAL_MS) {
            pwm_step(ctx, -CC_PWM_STEP);
            ctx->charger.cc_last_downstep_ms = now;
        }
    } else if (err > CC_DEADBAND_MA) {
        /* Over-current: back off. Higher pwm = lower duty.
         * Up-steps are NOT throttled — protective reaction stays fast. */
        pwm_step(ctx, +CC_PWM_STEP);
    }
    /* Within deadband: stable, no change. */
}
#endif /* CHARGER_INPUT_VREG */

/*
 * cv_regulate — regulate on V_bat around [BAT_CV_VOLTAGE_MV,
 * BAT_CV_VOLTAGE_MV + CV_DEADBAND_MV] (3650 … 3655 mV). Sign is correct
 * per the file-header convention (lower pwm = more current): V_bat below
 * target → pwm -= step (more current pulls it up); above → pwm += step.
 */
static void cv_regulate(system_ctx_t *ctx)
{
    uint16_t v_bat = ctx->meas.bat_voltage;

    if (v_bat < BAT_CV_VOLTAGE_MV) {
        /* Below target — need more current to pull voltage up. */
        pwm_step(ctx, -CV_PWM_STEP);
    } else if (v_bat > (BAT_CV_VOLTAGE_MV + CV_DEADBAND_MV)) {
        /* Above target — reduce current, voltage will drift down. */
        pwm_step(ctx, +CV_PWM_STEP);
    }
    /* Within 3650..3655: stable. */
}

/*
 * cv_taper_track — update taper timer and set ctx->bat_full when the
 * charger has been pushing < BAT_CV_TAPER_MA (200 mA) continuously for
 * BAT_FULL_HOLD_MS (30 s) while in CV.
 *
 * If the current rises back above taper at any point, the timer
 * resets — we require an uninterrupted 30 s window.
 */
static void cv_taper_track(system_ctx_t *ctx)
{
    charger_ctx_t *c = &ctx->charger;
    uint32_t now = time_now();

    bool under_taper = (ctx->meas.chg_current < (int16_t)BAT_CV_TAPER_MA);

    if (under_taper) {
        if (!c->bat_full_timing) {
            c->bat_full_timing    = true;
            c->bat_full_timer_start = now;
        } else if (!c->bat_full_signaled &&
                   (now - c->bat_full_timer_start) >= BAT_FULL_HOLD_MS) {
            c->bat_full_signaled = true;
            ctx->bat_full        = true;  /* energy_mode reads this flag */
        }
    } else {
        /* Current climbed back above taper — reset the window. */
        c->bat_full_timing   = false;
        c->bat_full_signaled = false;
    }
}

/* =========================================================================
 * PER-STATE TICK LOGIC
 * ========================================================================= */

static void tick_precharge(system_ctx_t *ctx)
{
    charger_ctx_t *c = &ctx->charger;

    /* P1: V_bat climbed above 3 V → move to CC (full rate). */
    if (ctx->meas.bat_voltage >= BAT_PRECHARGE_MV) {
        enter_cc(ctx);
        /* Fall through to CC regulation this tick for continuity. */
    }
    /* P2: 15-minute precharge timeout → raise fault and park.
     *     fault_mgr will take the protective action (buck off, charge
     *     switch off). The charger stops regulating on the next tick
     *     because CHG_FAULT_BLOCK_MASK will match. */
    else if ((time_now() - c->precharge_start_ms) >= BAT_PRECHARGE_TIMEOUT_MS) {
        fault_raise(ctx, FAULT_PRECHARGE_TIMEOUT);
        ctx->pwm = PWM_MIN_DUTY;    /* belt-and-braces: force buck off */
        return;
    }

    /* Panel safety + MPPT gate, then regulate. */
    if (panel_safety_backoff(ctx)) return;
    if (mppt_owns_pwm(ctx))        return;

    cc_regulate(ctx);
}

static void tick_cc(system_ctx_t *ctx)
{
    /* P1: V_bat ≥ 3.65 V → CV. */
    if (ctx->meas.bat_voltage >= BAT_CV_VOLTAGE_MV) {
        enter_cv(ctx);
        /* Fall through to CV regulation on the same tick. */
        if (panel_safety_backoff(ctx)) return;
        if (mppt_owns_pwm(ctx)) {
            cv_taper_track(ctx);   /* transition guard still runs */
            return;
        }
        cv_regulate(ctx);
        cv_taper_track(ctx);
        return;
    }

    if (panel_safety_backoff(ctx)) return;
    if (mppt_owns_pwm(ctx))        return;

    cc_regulate(ctx);
}

static void tick_cv(system_ctx_t *ctx)
{
    /* Taper timer must keep running even during MPPT tracking — it
     * observes current regardless of who owns PWM. Panel safety backoff
     * still pre-empts regulation, and will cause current to drop, which
     * would (correctly) trigger taper. */
    if (panel_safety_backoff(ctx)) {
        cv_taper_track(ctx);
        return;
    }

    if (mppt_owns_pwm(ctx)) {
        cv_taper_track(ctx);
        return;
    }

    cv_regulate(ctx);
    cv_taper_track(ctx);
}

/* =========================================================================
 * PUBLIC: pipeline step 7 entry
 * ========================================================================= */
void charger_update(system_ctx_t *ctx)
{
    charger_ctx_t *c = &ctx->charger;

    /* ── Gate: is the region eligible to run at all? ── */
    bool em_charging = (ctx->energy_mode == EM_CHARGE_ONLY) ||
                       (ctx->energy_mode == EM_CHARGE_AND_LOAD);
    bool fault_block = (ctx->fault.code & CHG_FAULT_BLOCK_MASK) != 0;

    if (!em_charging || fault_block) {
        /* energy_mode should have already deactivated on its transition,
         * but a fault might be newly latched this tick. Defensively
         * park PWM and mark INACTIVE — do not touch the GPIO switches,
         * fault_mgr/energy_mode own those. */
        if (c->state != CHG_INACTIVE) {
            c->state = CHG_INACTIVE;
            c->precharge_start_ms = 0;
            c->bat_full_timing    = false;
            c->bat_full_signaled  = false;
            ctx->pwm = PWM_MIN_DUTY;
        }
        return;
    }

    /* ── Self-activation from INACTIVE ──
     *
     * energy_mode has enabled buck + charge switch and left state at
     * INACTIVE. Choose PRECHARGE or CC based on V_bat, then fall
     * through to run the first regulation tick in the new state. */
    if (c->state == CHG_INACTIVE) {
        c->active_start_ms = time_now();
        c->cc_last_downstep_ms = time_now();   /* arm the descent throttle */
        if (ctx->meas.bat_voltage < BAT_PRECHARGE_MV) {
            enter_precharge(ctx);
        } else {
            enter_cc(ctx);
        }
    }

    /* ── Per-state tick ── */
    switch (c->state) {
        case CHG_PRECHARGE: tick_precharge(ctx); break;
        case CHG_CC:        tick_cc(ctx);        break;
        case CHG_CV:        tick_cv(ctx);        break;

        /* Should not happen — handled by the gate above. */
        case CHG_INACTIVE:
        default:
            break;
    }
}
