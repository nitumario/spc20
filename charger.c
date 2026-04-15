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
 *        - Panel-safety first: if V_panel < 10 V, back off by
 *          PANEL_BACKOFF_STEP and skip the rest of regulation.
 *        - If MPPT is TRACKING, skip regulation (MPPT owns PWM).
 *        - Otherwise step pwm by CC_PWM_STEP / CV_PWM_STEP toward the
 *          target, using CC_DEADBAND_MA / CV_DEADBAND_MV as a stability
 *          band.
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
 * - To INCREASE charge current: pwm -= step.
 * - To DECREASE charge current: pwm += step.
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
 * we are drawing too much current from the panel. Quickly reduce duty
 * (pwm += several) and skip regulation. Returns true if backoff fired.
 *
 * This runs BEFORE the MPPT check so we protect the panel even during
 * MPPT TRACKING perturbations that might have pushed it over the edge.
 */
static bool panel_safety_backoff(system_ctx_t *ctx)
{
    if (ctx->meas.panel_voltage < PANEL_SAFETY_MV) {
        pwm_step(ctx, PANEL_BACKOFF_STEP);  /* reduce current aggressively */
        return true;
    }
    return false;
}

/*
 * mppt_owns_pwm — the charger yields PWM control while MPPT is actively
 * perturbing. Transition checks above still run; only the regulator
 * step is skipped.
 */
static inline bool mppt_owns_pwm(const system_ctx_t *ctx)
{
    return (ctx->mppt.state == MPPT_TRACKING);
}

/*
 * cc_regulate — bang-bang regulator with deadband on ctx->meas.chg_current
 * toward ctx->allowed_chg. Called from both PRECHARGE and CC states
 * because their regulation is identical — the "precharge" nature is
 * enforced by power_budget clamping allowed_chg to ≤200 mA below 3 V.
 */
static void cc_regulate(system_ctx_t *ctx)
{
    /* err > 0 means we are OVER target (too much current). */
    int32_t target = (int32_t)ctx->allowed_chg;
    int32_t actual = (int32_t)ctx->meas.chg_current;
    int32_t err    = actual - target;

    if (err < -CC_DEADBAND_MA) {
        /* Under-current: push harder. Lower pwm = higher duty. */
        pwm_step(ctx, -CC_PWM_STEP);
    } else if (err > CC_DEADBAND_MA) {
        /* Over-current: back off. Higher pwm = lower duty. */
        pwm_step(ctx, +CC_PWM_STEP);
    }
    /* Within deadband: stable, no change. */
}

/*
 * cv_regulate — regulate on V_bat around [BAT_CV_VOLTAGE_MV,
 * BAT_CV_VOLTAGE_MV + CV_DEADBAND_MV] (3650 … 3655 mV).
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
