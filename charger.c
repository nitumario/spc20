/*
 * charger.c — Charge Controller Implementation (Pipeline Step 7)
 * =========================================================================
 *
 * Responsibilities per tick (50 ms):
 *   1. Decide whether the charger should run at all. It runs only when
 *      energy_mode has activated the region (EM_CHARGE_ONLY or
 *      EM_CHARGE_AND_LOAD) AND no charge-blocking fault is latched.
 *   2. Hold CHG_BUCK_SETTLE until instantaneous VCHG is above V_bat, then
 *      close Q49 and enter PRECHARGE, CC, or CV from the battery voltage.
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
 * PWM is retained between PRECHARGE, CC, and CV. On first activation,
 * energy_mode pre-positions it from the voltage LUT while Q49 is open;
 * CHG_BUCK_SETTLE verifies the resulting rail before connecting the cell.
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
#include "energy_mode.h"   /* safe_mode_rescue_active() — SAFE_MODE rescue gate */
#include "SPCBoardAPI.h"

/* Any of these latched faults should block the charger from running.
 * (Undervolt is excluded on purpose: while it is latched the charger is
 *  normally idle because energy_mode is in SAFE_MODE — but SAFE_MODE may run
 *  the supervised rescue trickle, which needs the charger to run WITH undervolt
 *  latched. Adding undervolt here would block that rescue. The rescue gate
 *  itself, safe_mode_rescue_active(), only fires when undervolt is the SOLE
 *  latched fault, so no charge-blocking fault in this mask is ever bypassed.) */
#define CHG_FAULT_BLOCK_MASK   \
    (FAULT_OVERTEMP            \
   | FAULT_BAT_OVERVOLT        \
   | FAULT_OVERCURRENT_CHG     \
   | FAULT_REVERSE_PUMP        \
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

/*
 * pwm_draw_more — step toward higher current, but not past the learned
 * ceiling (mppt.cliff_pwm_min, raised by every collapse — see
 * learn_cliff_pwm in mppt.c and CHG_CLIFF_PWM_MARGIN in hw_config.h).
 *
 * Only the regulator's two "draw more" branches go through here. The
 * reverse-current escape, panel_safety_backoff and the CHG_BUCK_SETTLE
 * acquisition ramp keep using pwm_step directly: the ceiling exists to bound
 * current on a panel that already fell over once, and blocking an escape or
 * an interlock ramp with it would turn a current limit into a lockup (a
 * fenced SETL ramp is exactly the "stranded in SETL forever" failure the
 * ramp was added to fix).
 *
 * Already below the ceiling — a resume that entered under it, or a relax
 * that moved it up — is left alone rather than snapped up: the fence stops
 * the loop from walking DOWN into the knee, it is not a setpoint.
 */
static inline void pwm_draw_more(system_ctx_t *ctx, int32_t step)
{
    uint16_t floor_pwm = ctx->mppt.cliff_pwm_min;
    if (floor_pwm != 0 && ctx->pwm <= floor_pwm)
        return;

    uint16_t next = pwm_clamp((int32_t)ctx->pwm - step);
    if (floor_pwm != 0 && next < floor_pwm)
        next = floor_pwm;
    ctx->pwm = next;
}

/* Is the battery physically tied to the buck rail right now? Both foreground
 * guards are meaningless outside these states: in CHG_BUCK_SETTLE Q49 is still
 * open (and VCHG is legitimately below V_bat while the rail ramps), and in
 * CHG_INACTIVE there is nothing to protect. */
static inline bool charger_connected(const system_ctx_t *ctx)
{
    charger_state_t state = ctx->charger.state;
    return (state == CHG_PRECHARGE) ||
           (state == CHG_CC) ||
           (state == CHG_CV);
}

/*
 * charger_input_present — can the source still hold the buck rail above the
 * battery? Instantaneous conversions only; see CHG_INPUT_LOST_MARGIN_MV in
 * hw_config.h for why this is judged on V_panel rather than VCHG.
 */
static inline bool charger_input_present(void)
{
    return (uint32_t)get_input_voltage_now() >
           ((uint32_t)get_battery_voltage_now() + CHG_INPUT_LOST_MARGIN_MV);
}

/*
 * charger_input_stand_down — the input went away under a running charger.
 *
 * This is an ordinary event (PSU switched off, panel unplugged, connector
 * glitch), NOT a fault: nothing has misbehaved, the energy source simply
 * left. Isolate the hardware here in the foreground, but leave the FSM
 * transition and the log line to the next 50 ms tick — see the
 * input_lost_pending comment in system_types.h.
 *
 * Q49 opens FIRST: with the switch open there is no path from the cell to the
 * collapsing rail at all. The trip point sits just above the V_panel ≈ V_bat
 * crossover where back-feeding begins (it cannot sit higher without colliding
 * with panel_safety_backoff's sag regime — see CHG_INPUT_LOST_MARGIN_MV), so
 * this pre-empts the reversal when it can and otherwise bounds it to a single
 * 10 ms sample.
 */
static void charger_input_stand_down(system_ctx_t *ctx)
{
    charger_ctx_t *c = &ctx->charger;
    /* charger_fast_guard reaches here without a snapshot; take one so the
     * log gets a trace for every stand-down, whichever guard called it. */
    if (!c->input_trace_pending) {
        adc_panel_trace_mv(c->input_trace_mv);
        c->input_trace_pwm_from = ctx->pwm;
        c->input_trace_pending  = true;
    }
    c->input_trace_result = INPUT_TRACE_STOOD_DOWN;

    disable_charge_switch();
    disable_input_buck();
    ctx->pwm = PWM_MIN_DUTY;
    c->input_trace_pwm_to  = ctx->pwm;
    c->input_lost_pending  = true;
    c->input_lost_count    = 0;   /* next connection starts clean */
}

/*
 * charger_input_guard — input collapse: back off first, stand down last.
 *
 * Runs from the free-running super loop, so it sees the latest raw conversion
 * within ~10 ms of it landing. A reading under V_bat + CHG_INPUT_LOST_MARGIN_MV
 * used to mean "source removed" and got an immediate stand-down. Every such
 * reading the bench has ever captured (18 INTRACE dumps, 10.09.26) was a
 * LIVE panel pushed over its knee — pinned at V_bat + ~190 mV pushing Isc
 * through the buck in dropout, back at Voc 400 ms after the stand-down. That
 * state is forward current, not reverse pumping, and it recovers the moment
 * the draw drops under Isc. So:
 *
 *   sample 1  snapshot the raw trace, cut pwm by CHG_INPUT_RECOVER_STEP
 *   sample 2  still under → cut again
 *   sample 3  still under → charger_input_stand_down (a removed source)
 *   any       back over the margin → RECOVERED: keep charging where we are,
 *             count it (input_dip_events; MPPT learns the level from that)
 *
 * The backoff is the one place other than apply_pwm that writes the timer,
 * through the same clamp; it is a reduction only (pwm up), so it can never
 * command more current than the pipeline already had.
 *
 * ⚠️ The counter is gated on adc_sample_seq(), NOT on calls. This function is
 * invoked thousands of times per second against a value that is refreshed
 * every TICK_ADC_MS (10 ms) from SysTick, so a plain `if (++count >= N)`
 * reaches N within microseconds off a single stale sample.
 *
 * Exposure on a genuine removal moves from "usually pre-empted at the
 * crossover" to "caught by charger_fast_guard on the first reverse-current
 * sample" — the path v0.26 already handles as a clean stand-down. See
 * CHG_INPUT_LOST_SAMPLES.
 */
void charger_input_guard(system_ctx_t *ctx)
{
    charger_ctx_t *c = &ctx->charger;

    if (!charger_connected(ctx) || c->input_lost_pending) {
        c->input_lost_count = 0;
        return;
    }

    if (charger_input_present()) {
        if (c->input_lost_count != 0) {
            /* Came back inside the window: a live source that had been pushed
             * over its knee. Stay connected at the backed-off point —
             * cc_regulate walks it back down, MPPT raises its floor. */
            c->input_lost_count = 0;
            if (c->input_dip_events < UINT16_MAX)
                c->input_dip_events++;
            if (c->input_trace_result == INPUT_TRACE_PENDING)
                c->input_trace_result = INPUT_TRACE_RECOVERED;
        }
        return;
    }

    /* Under the margin. Count it once per conversion. */
    uint32_t seq = adc_sample_seq();
    if (seq == c->input_lost_seq)
        return;
    c->input_lost_seq = seq;

    if (c->input_lost_count == 0) {
        /* First sight of it: snapshot before anything moves. Taking the
         * copy here rather than at print time keeps the descent near the
         * young end of the window instead of 50 ms of newer samples having
         * pushed it out. */
        adc_panel_trace_mv(c->input_trace_mv);
        c->input_trace_pwm_from = ctx->pwm;
        c->input_trace_result   = INPUT_TRACE_PENDING;
        c->input_trace_pending  = true;
    }
    c->input_lost_count++;
    c->input_rescue_ms = time_now();   /* opens the fast-guard blank window */

    if (c->input_lost_count >= CHG_INPUT_LOST_SAMPLES) {
        charger_input_stand_down(ctx);
        return;
    }

    /* Cut the draw and give the source one conversion to come back.
     *
     * ⚠️ This backoff MUST be allowed to command a target below V_bat, and
     * v0.34 learned that the hard way by forbidding it. Once the input has
     * collapsed the buck is in dropout — high side on, V_panel shorted to the
     * cell — and in dropout the FB target has NO authority over duty. The
     * only thing that breaks dropout is commanding a target the output is
     * already ABOVE, i.e. below V_bat. Clamping the backoff at the zero-draw
     * point (v0.34) capped it at 9 counts, and every collapse that v0.33 had
     * rescued at 20 counts became a stand-down instead: bench 10.09.26,
     * `INTRACE … RECOVERED pwm:95->115` became `PENDING pwm:97->106` followed
     * by `STANDDOWN pwm:106->399`.
     *
     * So "relieve the panel" and "briefly sink" are the same command on this
     * hardware. That is a property of the FCCM part, not a bug to tune away:
     * the reverse current it implies is bounded to the guard's 3 conversions
     * (~30 ms) and charger_fast_guard yields to us for that window rather than
     * latching (CHG_REVERSE_BLANK_MS). */
    ctx->pwm = pwm_clamp((int32_t)ctx->pwm + (int32_t)CHG_INPUT_RECOVER_STEP);
    set_buck_pwm(ctx->pwm);
    c->input_trace_pwm_to = ctx->pwm;
}

void charger_fast_guard(system_ctx_t *ctx)
{
    charger_ctx_t *c = &ctx->charger;

    /* A stand-down already isolated the path this tick; nothing left to judge
     * (and chg_current reads 0 with Q49 open anyway). */
    if (!charger_connected(ctx) || c->input_lost_pending) {
        c->reverse_count = 0;
        return;
    }

    /* chg_current (R440) is NET cell current: I_cell = I_buck − I_load. An
     * activation under load connects Q49 with the buck at ~zero delivery, so
     * I_cell legitimately reads −I_load until cc_regulate walks the delivery
     * up — judging chg alone here latched this fault on every lamps-on
     * activation (bench 2026-07-29, panel replug). Genuine reverse pumping
     * means the BUCK branch is negative, so judge I_buck = I_cell + I_dsg.
     * The dsg sensor's ~45 mA zero-offset only biases this check away from
     * tripping, well inside the threshold. */
    int32_t i_buck_now = (int32_t)get_charge_current_now() +
                         (int32_t)get_discharge_current_now();

    if (i_buck_now >= -(int32_t)CHG_REVERSE_CURRENT_MA) {
        c->reverse_count = 0;
    } else {
        /* Classify before latching. Reverse current with the input already
         * DEAD is a disconnect whose voltage signature charger_input_guard
         * narrowly lost the race to — the input node can fall past the
         * crossover inside one 10 ms conversion interval. Stand down cleanly;
         * latching there is what made every source removal look like a charge
         * over-current (bench 2026-07-30).
         *
         * Reverse current with the input still LIVE is the real failure: the
         * cell is being pushed back into a working panel, which can drive
         * V_panel above open circuit toward the TPS564247's input ceiling
         * (May bench trace: −1010 mA at V_panel 14.1 V on a 13 V panel). That
         * latches. FAULT_REVERSE_PUMP's action opens Q49 before disabling the
         * buck; energy_mode's fault-clear re-arm subsequently restarts from
         * CHG_BUCK_SETTLE, never directly connected. */
        if (!charger_input_present()) {
            charger_input_stand_down(ctx);
            return;
        }

        /* Live input, reverse current — but is this the pathology, or the
         * tail of a collapse charger_input_guard is still rescuing?
         *
         * Within CHG_REVERSE_BLANK_MS of the guard's last action the two are
         * indistinguishable by sign alone: the panel recovers to full voltage
         * in a few milliseconds while the buck is still carrying the backed-off
         * target, so "reverse current with a live input" is exactly what a
         * successful rescue looks like on its way back. v0.32 read it as the
         * pathology and latched 0x0100 on all four collapses of the 10.09.26
         * bench session — every one of them next to its own INTRACE RECOVERED.
         *
         * So inside the window, correct the cause first: lift the target back
         * to the zero-draw point (which cannot sink) and give it one
         * conversion. Still reversing on the next one means it was not the
         * rescue — stand down cleanly rather than latch, the same call v0.26
         * made for a dead input. Exposure stays bounded at one 10 ms sample. */
        bool rescuing = (c->input_rescue_ms != 0) &&
                        ((time_now() - c->input_rescue_ms) < CHG_REVERSE_BLANK_MS);

        if (rescuing) {
            /* Yield: charger_input_guard owns this event and resolves it
             * within CHG_INPUT_LOST_SAMPLES conversions (~30 ms) either way.
             * Do NOT try to correct the target here — the backoff commanding
             * below V_bat is what breaks dropout, so "helping" by lifting it
             * back cancels the rescue (v0.34). Do not latch either: reverse
             * current IS the expected signature while the guard works. */
            return;
        }

        /* Same yield for the connection transient. The buck hands off from
         * CHG_BUCK_SETTLE delivering ~nothing, so for the first conversions
         * after Q49 closes the net cell current sits AT zero — the one place
         * where a single raw sample straddles a -100 mA threshold on noise
         * alone. The reverse-pump condition itself is excluded by
         * construction here: SETL does not connect until instantaneous VCHG
         * is above the cell. This is what made 467 of the 498 charge sessions
         * in the 10.09.26 bench log die inside 300 ms. */
        if ((time_now() - c->connect_ms) < CHG_CONNECT_BLANK_MS)
            return;

        /* Debounce the latch on DISTINCT conversions, exactly as
         * charger_input_guard debounces its own teardown — this runs at
         * super-loop rate against a value that only moves every TICK_ADC_MS,
         * so counting calls would reach the threshold off one stale sample
         * within microseconds.
         *
         * Real reverse pumping is a sustained state (-1010 mA on the May
         * trace) and clears CHG_REVERSE_SAMPLES without difficulty; a noise
         * sample about zero does not. Latching is expensive enough
         * (FAULT_RECOVER_WAIT_MS, 10 s of no charging) to be worth ~30 ms of
         * confirmation. */
        uint32_t seq = adc_sample_seq();
        if (seq == c->reverse_seq)
            return;
        c->reverse_seq = seq;

        if (++c->reverse_count < CHG_REVERSE_SAMPLES)
            return;

        fault_raise(ctx, FAULT_REVERSE_PUMP);
        ctx->pwm = PWM_MIN_DUTY;
        c->reverse_count = 0;
    }
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
 * Priority (highest first; matches the "Clamp N" labels in the body):
 *   0. Reverse-current escape (hard): if chg_current is strongly negative
 *      the sync FETs are back-pumping battery charge into the panel —
 *      lift the rail fast (pwm DOWN by the backoff step).
 *   1. Battery-intake clamp (hard): if chg_current exceeds allowed_chg,
 *      reduce current regardless of V_panel. allowed_chg already encodes
 *      the precharge trickle / CC-zone / CV-taper limits (power_budget).
 *   2. Panel-voltage loop: otherwise steer V_panel toward the setpoint.
 *   3. In-band re-acquire: if within the deadband but delivering almost
 *      nothing (a panel_safety_backoff overshoot parked the rail near
 *      open circuit), draw more so the operating point walks back onto
 *      the panel instead of letting has_sun time out.
 *
 * (panel_safety_backoff, the emergency V_panel floor, runs BEFORE this
 * function in the state handlers — it is not one of the clamps here.)
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
     * MPPT_SP_FRACTION_PCT·Voc on a fresh activation, then hill-climbed —
     * see mppt.c), NOT the static PANEL_VREG_SETPOINT_MV, which is only its
     * cold-boot seed. */
    int32_t v_sp = (int32_t)ctx->mppt.vreg_setpoint_mv;
    if (v_panel < v_sp - (int32_t)PANEL_VREG_DEADBAND_MV) {
        /* Sagging below MPP → drawing too much → DRAW LESS (pwm UP) → V recovers. */
        pwm_step(ctx, +PANEL_VREG_STEP);
    } else if (v_panel > v_sp + (int32_t)PANEL_VREG_DEADBAND_MV) {
        /* Above MPP with current-headroom (clamp 1 didn't fire) → DRAW MORE (pwm DOWN) → V falls.
         * Fenced by the learned PWM ceiling: this branch is what walked the
         * panel off its knee in the first two teardowns of the 10.09.26
         * session (V_panel 1.5 V above the band, one step per interval,
         * straight into the collapse). */
        pwm_draw_more(ctx, PANEL_VREG_STEP);
    } else if (i_chg < (int32_t)LOAD_REACQUIRE_MA) {
        /* Clamp 3: in-band but delivering ~nothing → RE-ACQUIRE (pwm DOWN).
         * This is the tail of a panel_safety_backoff overshoot: the emergency
         * backoff snapped the rail from a collapse straight up to near open
         * circuit, which sits INSIDE the wide deadband, so both branches above
         * hold and the buck idles unloaded (chg_current ~10 mA). Left alone,
         * P_panel stays below the dusk floor and has_sun eventually gives up —
         * the charge-on/off teardown. Instead keep drawing more, one paced
         * step, to walk the operating point back onto the panel. Self-limiting:
         * stops the instant current returns above LOAD_REACQUIRE_MA. On a dead
         * panel at dusk it loads the panel down until V_panel collapses and the
         * voltage-path has_sun clear fires. cc_regulate only runs in
         * PRECHARGE/CC (CV uses cv_regulate), so there is no CV interaction, and
         * clamps 0/1 already returned for reverse / over-current, so this can
         * only fire in the genuine near-open-circuit dead zone. Fenced too —
         * re-acquiring is still drawing more, and a re-acquire that walks
         * back to the count that just collapsed re-collapses. */
        pwm_draw_more(ctx, PANEL_VREG_STEP);
    }
    /* Within the deadband and delivering current → stable, hold PWM. */
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
        /* Below target — need more current to pull voltage up. Fenced by the
         * learned PWM ceiling like CC's draw-more branches: the knee does not
         * care which state asked. It can only bind if CV wants more current
         * than the count that already collapsed the panel, in which case the
         * unfenced alternative is another teardown, not a faster taper. */
        pwm_draw_more(ctx, CV_PWM_STEP);
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

static void tick_buck_settle(system_ctx_t *ctx)
{
    charger_ctx_t *c = &ctx->charger;
    uint32_t now = time_now();

    if ((now - c->active_start_ms) < CHG_BUCK_SETTLE_MS)
        return;

    /* Use the latest ADC conversion, not the 64-sample moving average: the
     * averaged VCHG still contains the buck-off history. Q49 remains open for
     * as long as necessary, so a failed or weak buck start cannot connect a
     * below-Vbat rail and reverse-pump the input. Re-check instantaneous Vbat
     * as well as the activation snapshot so battery movement during a long
     * acquisition cannot invalidate the interlock. */
    uint16_t live_ready_mv =
        get_battery_voltage_now() + CHG_BUCK_READY_MARGIN_MV;
    uint16_t ready_mv = (live_ready_mv > c->activation_ready_mv)
                      ? live_ready_mv
                      : c->activation_ready_mv;
    if (get_charge_voltage_now() < ready_mv) {
        /*
         * The calibrated LUT is only the acquisition starting point. Bench
         * traces show its unloaded result can be about 300 mV low, which used
         * to strand the charger in SETL forever. Walk toward more buck output
         * under raw VCHG feedback while Q49 is still open. The small paced step
         * bounds overshoot at the eventual connection; a failed rail simply
         * saturates at PWM_MAX_DUTY without ever exposing the battery.
         */
        if (ctx->meas.panel_voltage >= PANEL_SAFETY_MV &&
            (now - c->cc_last_downstep_ms) >=
                CHG_BUCK_SETTLE_RAMP_MS) {
            c->cc_last_downstep_ms = now;
            pwm_step(ctx, -CHG_BUCK_SETTLE_PWM_STEP);
        }
        return;
    }

    enable_charge_switch();
    c->cc_last_downstep_ms = now;

    /* Q49 just closed, so cell current starts existing now — at ~0, because
     * the acquisition ramp stopped as soon as VCHG cleared the cell and
     * cc_regulate has yet to walk any delivery up. Open the window in which
     * charger_fast_guard yields rather than reading noise about zero as
     * reverse pumping, and clear any debounce carried in from the last
     * session. See CHG_CONNECT_BLANK_MS. */
    c->connect_ms     = now;
    c->reverse_count  = 0;

    /* Re-assert the learned ceiling at the handoff. The acquisition ramp above
     * is deliberately unfenced — fencing it is how a current limit becomes a
     * "stranded in SETL forever" lockup — but it steps every 50 ms while the
     * rail is merely still SLEWING, so it routinely walks 6-16 counts past
     * wherever it started. Bench 10.09.26 (v0.33): resume entered at the
     * ceiling (113), SETL ramped straight through it to 107, and CC then held
     * 107 for 156 s until the panel fell over there again — pwm_draw_more
     * cannot pull it back, since below the fence it is a no-op by design.
     * Q49 has just closed and this only ever REDUCES current, so it is safe
     * here in a way it is not inside the ramp. */
    if (ctx->mppt.cliff_pwm_min != 0 && ctx->pwm < ctx->mppt.cliff_pwm_min) {
        ctx->pwm = pwm_clamp((int32_t)ctx->mppt.cliff_pwm_min);
        set_buck_pwm(ctx->pwm);
    }

    if (ctx->meas.bat_voltage < BAT_PRECHARGE_MV)
        enter_precharge(ctx);
    else if (ctx->meas.bat_voltage < BAT_CV_VOLTAGE_MV)
        enter_cc(ctx);
    else
        enter_cv(ctx);

    /* Do not regulate on the switch-close tick. The current sample still
     * describes the open-switch interval. */
}

static void tick_precharge(system_ctx_t *ctx)
{
    charger_ctx_t *c = &ctx->charger;

    /* P1: V_bat climbed above 3 V → move to CC (full rate). */
    if (ctx->meas.bat_voltage >= BAT_PRECHARGE_MV) {
        enter_cc(ctx);
        /* Fall through to CC regulation this tick for continuity. */
    }
    /* P2: precharge timeout → raise fault and park.
     *     fault_mgr will take the protective action (buck off, charge
     *     switch off). The charger stops regulating on the next tick
     *     because CHG_FAULT_BLOCK_MASK will match.
     *
     *     A cell that hit BAT_UNDERVOLT this boot enters PRECHARGE from a much
     *     deeper start (down to BAT_RESCUE_MIN_MV) than a normal precharge, so
     *     it gets the longer BAT_RESCUE_TIMEOUT_MS window before being declared
     *     damaged. Either way the escalation is the same terminal
     *     FAULT_PRECHARGE_TIMEOUT (user-assisted).
     *
     *     Keyed on fault.history, not fault.code: the SAFE_MODE wake probe
     *     clears the BAT_UNDERVOLT latch as soon as it validates the cell is
     *     really present (energy_mode.c, fault_clear), so by the time the
     *     rescued cell reaches PRECHARGE the live code is already 0. Reading
     *     fault.code there handed a just-rescued battery the short 15 min
     *     window — only ~50 mAh at BAT_PRECHARGE_MAX_MA — and would latch
     *     PRECHARGE_TIMEOUT on a cell that was recovering normally (bench log
     *     2026-07-29: probe rescued to 2.65 V, then precharged on the short
     *     timeout). history is sticky for the boot, which is the intent: once
     *     this cell has been that deep, every precharge until reset is a
     *     deep-discharge recovery. */
    else {
        uint32_t timeout = (ctx->fault.history & FAULT_BAT_UNDERVOLT)
                         ? BAT_RESCUE_TIMEOUT_MS
                         : BAT_PRECHARGE_TIMEOUT_MS;
        if ((time_now() - c->precharge_start_ms) >= timeout) {
            fault_raise(ctx, FAULT_PRECHARGE_TIMEOUT);
            ctx->pwm = PWM_MIN_DUTY;    /* belt-and-braces: force buck off */
            return;
        }
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

    /* ── Gate: is the region eligible to run at all? ──
     *
     * Runs in the two charging energy modes, and additionally during a
     * supervised undervolt rescue in SAFE_MODE (loads stay shed; power_budget
     * keeps the intake at precharge rate). safe_mode_rescue_active() requires
     * undervolt to be the sole latched fault, so when it is true no
     * CHG_FAULT_BLOCK_MASK bit can be set — the two conditions never conflict. */
    bool rescue      = safe_mode_rescue_active(ctx);
    bool em_charging = (ctx->energy_mode == EM_CHARGE_ONLY) ||
                       (ctx->energy_mode == EM_CHARGE_AND_LOAD);
    bool fault_block = (ctx->fault.code & CHG_FAULT_BLOCK_MASK) != 0;

    if ((!em_charging && !rescue) || fault_block) {
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

    /* energy_mode owns activation and always stages it through BUCK_SETTLE.
     * An eligible-but-INACTIVE state therefore means no entry action has
     * armed the hardware yet; do not bypass the reverse-pump interlock. */
    if (c->state == CHG_INACTIVE) {
        return;
    }

    /* ── Per-state tick ── */
    switch (c->state) {
        case CHG_BUCK_SETTLE: tick_buck_settle(ctx); break;
        case CHG_PRECHARGE:   tick_precharge(ctx);   break;
        case CHG_CC:          tick_cc(ctx);          break;
        case CHG_CV:          tick_cv(ctx);          break;

        /* Should not happen — handled by the gate above. */
        case CHG_INACTIVE:
        default:
            break;
    }
}
