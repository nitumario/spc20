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

    /* Every caller of this helper either reduces current or is an interlock
     * the fence may not block, so reaching here proves the loop is not being
     * refused. Clearing here covers all of them — the backoffs, the intake
     * clamp, the reverse escape and the acquisition ramp — without each
     * having to remember to. */
    ctx->charger.draw_blocked_ms = 0;
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
    if (floor_pwm != 0 && ctx->pwm <= floor_pwm) {
        /* Refused by the fence. Stamp the FIRST refusal and leave it
         * standing: an unbroken CHG_CLIFF_PROBE_MS of being refused is the
         * fast release's evidence (mppt.c) that the fence — not the panel —
         * is what is capping the draw. Any grant, backoff or interlock step
         * clears it, so only a continuously starved loop accumulates. */
        if (ctx->charger.draw_blocked_ms == 0)
            ctx->charger.draw_blocked_ms = time_now();
        return;
    }

    uint16_t next = pwm_clamp((int32_t)ctx->pwm - step);
    if (floor_pwm != 0 && next < floor_pwm)
        next = floor_pwm;
    ctx->pwm = next;
    ctx->charger.draw_blocked_ms = 0;
}

/*
 * zero_draw_bound_pwm (v0.41) — the highest count a demand-shedding action
 * may command from `from_pwm`, where the buck was delivering `ichg_from`
 * into the cell and `idsg_from` into the load (I_buck = I_cell + I_load):
 * the zero-delivery point plus CHG_ZERO_DRAW_MARGIN, but never less than
 * `min_advance` counts of movement so the action is not a no-op.
 *
 * Uses the live charger.zero_draw_pwm estimate when there is one (refreshed
 * every settled delivering tick in charger_update, so it follows V_bat) and
 * derives it from the event's own snapshot otherwise. See
 * CHG_BUCK_MA_PER_COUNT in hw_config.h for the physics and the bench case.
 */
static uint16_t zero_draw_bound_pwm(const system_ctx_t *ctx, uint16_t from_pwm,
                                    int16_t ichg_from, uint16_t idsg_from,
                                    uint16_t min_advance)
{
    uint32_t zero;
    if (ctx->charger.zero_draw_pwm != 0) {
        zero = ctx->charger.zero_draw_pwm;
    } else {
        int32_t i_buck = (int32_t)ichg_from + (int32_t)idsg_from;
        if (i_buck < 0) i_buck = 0;
        zero = (uint32_t)from_pwm +
               ((uint32_t)i_buck + CHG_BUCK_MA_PER_COUNT - 1U) /
                   CHG_BUCK_MA_PER_COUNT;
    }
    uint32_t bound = zero + CHG_ZERO_DRAW_MARGIN;
    if (bound < (uint32_t)from_pwm + min_advance)
        bound = (uint32_t)from_pwm + min_advance;
    if (bound > PWM_MIN_DUTY)
        bound = PWM_MIN_DUTY;
    return (uint16_t)bound;
}

/* Which count is "the draw that fell over"? The droop guard runs first in the
 * super loop and may already have moved the PWM on this same conversion, so a
 * guard that sights the event second must take the droop's snapshot, not the
 * live count — otherwise it records the rescue as the cause (v0.38's
 * lesson, one layer up). */
static inline uint16_t guard_event_origin_pwm(const system_ctx_t *ctx)
{
    return (ctx->charger.droop_count != 0) ? ctx->charger.droop_pwm_from
                                           : ctx->pwm;
}
static inline int16_t guard_event_origin_ichg(const system_ctx_t *ctx)
{
    return (ctx->charger.droop_count != 0) ? ctx->charger.droop_ichg_from
                                           : ctx->meas.chg_current;
}
static inline uint16_t guard_event_origin_vpanel(const system_ctx_t *ctx)
{
    return (ctx->charger.droop_count != 0) ? ctx->charger.droop_vpanel_from
                                           : ctx->meas.panel_voltage;
}
static inline bool guard_event_origin_settled(const system_ctx_t *ctx)
{
    return (ctx->charger.droop_count != 0)
             ? ctx->charger.droop_settled
             : (ctx->charger.settled_ticks >= CHG_PWM_SETTLED_TICKS);
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
     * log gets a trace for every stand-down, whichever guard called it.
     *
     * ⚠ Gate on input_lost_count, NOT on input_trace_pending alone.
     * input_trace_pending is a LOGGING flag: log_input_trace clears it on
     * every 50 ms pipeline tick (before its own LOG_MODE_OFF return, so this
     * happens in every build). The guard's descent spans up to three 10 ms
     * conversions, so a tick boundary lands between its first sighting and
     * this stand-down about half the time — and re-snapshotting here then
     * records input_trace_pwm_from AFTER the guard's own two
     * CHG_INPUT_RECOVER_STEP backoffs. learn_cliff_pwm reads that field as
     * "the draw that fell over" and fences 2*CHG_INPUT_RECOVER_STEP counts
     * too high, permanently.
     *
     * Bench 14.09.26, serial_20260914_085920.log: the panel peaked at 5.0 W
     * (pwm 59, 12.49 V, 400 mA) and fell over at pwm 58. INTRACE printed
     * `PENDING pwm:58->78` @ 24832 ms; the stand-down landed @ 24883 ms, one
     * tick later, and overwrote pwm_from with 78. The fence came out at 84
     * instead of 64 and pinned the next 211 s at pwm 84 / 13.45 V / ~2.5 W —
     * half the available power, on a panel whose MPP the tracker had already
     * measured.
     *
     * input_lost_count is the honest test: charger_input_guard leaves it
     * non-zero for the whole event (it resets it at the END of this
     * function), and charger_fast_guard — the path that genuinely has no
     * snapshot — always arrives with it at zero. */
    if (!c->input_trace_pending)
        adc_panel_trace_mv(c->input_trace_mv);  /* old window already printed */
    if (c->input_lost_count == 0) {             /* no guard snapshot: this is it */
        c->input_trace_pwm_from    = guard_event_origin_pwm(ctx);
        c->input_trace_ichg_from   = guard_event_origin_ichg(ctx);
        c->input_trace_vpanel_from = guard_event_origin_vpanel(ctx);
        c->input_trace_settled     = guard_event_origin_settled(ctx);
    }
    c->input_trace_pending = true;              /* always re-arm the log line */
    c->input_trace_result = INPUT_TRACE_STOOD_DOWN;

    disable_charge_switch();
    disable_input_buck();
    ctx->pwm = PWM_MIN_DUTY;
    c->input_trace_pwm_to  = ctx->pwm;
    c->input_lost_pending  = true;
    c->input_lost_count    = 0;   /* next connection starts clean */
    c->vloop_measure_armed = false;
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
             * over its knee. Stay connected; MPPT raises its floor.
             *
             * v0.41: and put the rail back NOW. The backoff below had to
             * command a target under the cell to break dropout (see its
             * note), and it did — the input is back — but the buck is still
             * carrying that target: 10-20 counts under the zero-delivery
             * point is -450..-900 mA of reverse current, and the only thing
             * that used to lift it was cc_regulate's reverse escape at 5
             * counts per 400 ms. Every RECOVERED trace of the 10.09.26 v0.33
             * session was followed by FAULT_REVERSE_PUMP for exactly that
             * reason (six of its eight latches). Restore to the zero-delivery
             * point plus a margin — no draw on the panel that just fell over,
             * no reverse into it — and let the paced loop walk down from
             * there under the fence the event is about to set. The FB target
             * has authority again the moment the input is back over V_bat,
             * so this takes one conversion. */
            c->input_lost_count = 0;
            if (c->input_dip_events < UINT16_MAX)
                c->input_dip_events++;
            if (c->input_trace_result == INPUT_TRACE_PENDING)
                c->input_trace_result = INPUT_TRACE_RECOVERED;

            uint16_t restore = zero_draw_bound_pwm(ctx, c->input_trace_pwm_from,
                                                   c->input_trace_ichg_from,
                                                   ctx->meas.dsg_current,
                                                   CHG_CLIFF_PWM_MARGIN);
            if (restore < ctx->pwm) {
                ctx->pwm = restore;
                set_buck_pwm(ctx->pwm);
            }
            c->input_trace_pwm_to  = ctx->pwm;
            c->input_rescue_ms     = time_now();  /* fast guard yields from here */
            c->vloop_measure_armed = false;
            c->draw_blocked_ms     = 0;
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
        c->input_trace_pwm_from    = guard_event_origin_pwm(ctx);
        c->input_trace_ichg_from   = guard_event_origin_ichg(ctx);
        c->input_trace_vpanel_from = guard_event_origin_vpanel(ctx);
        c->input_trace_settled     = guard_event_origin_settled(ctx);
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

    /* Whatever the voltage loop had armed, this interval is no longer a
     * measurement of the plant: the PWM moved 10 counts underneath it and
     * V_panel is mid-collapse. A rescue that happens to recover with the
     * signs agreeing (rail snaps from V_bat back to Voc while the guard
     * steps the draw off) reads as ~470 mV/count — inside the sanity
     * clamps, and enough to widen the band right back to where this whole
     * change started. Drop it instead. */
    c->vloop_measure_armed = false;
}

/*
 * panel_droop_trip_mv — the raw V_panel level below which the panel is
 * falling rather than regulating. 0 = the guard cannot be armed here.
 *
 * The gap is measured in PWM COUNTS whenever the voltage loop has a gain to
 * measure them with (CHG_DROOP_TRIP_COUNTS), because that is the only unit
 * in which "further than the regulator could have moved it" means anything.
 * CHG_DROOP_TRIP_PCT is the fallback for the first seconds of a session,
 * before any gain exists — deliberately coarse, so it cannot false-trip
 * while the plant is still unknown.
 *
 * Returns 0 (disarmed) when the resulting trip would sit under
 * PANEL_SAFETY_MV. On a steep plant the count criterion puts it there by
 * itself, which is the correct outcome: one PWM count is a quarter of the
 * 6.5 V array, a droop there cannot be told from a regulation step, and the
 * emergency backoff already owns everything below that line.
 */
static uint16_t panel_droop_trip_mv(const system_ctx_t *ctx)
{
    const charger_ctx_t *c = &ctx->charger;
    if (c->panel_op_mv < PANEL_MIN_MV)
        return 0;

    uint32_t op  = (uint32_t)c->panel_op_mv;
    uint32_t gap;

    if (c->plant_mv_per_count != 0) {
        gap = (uint32_t)c->plant_mv_per_count * CHG_DROOP_TRIP_COUNTS;
        if (gap < CHG_DROOP_MIN_GAP_MV)
            gap = CHG_DROOP_MIN_GAP_MV;
    } else {
        gap = (op * (100U - CHG_DROOP_TRIP_PCT)) / 100U;
    }

    if (gap >= op)
        return 0;
    uint32_t trip = op - gap;

    if (trip < PANEL_SAFETY_MV)
        return 0;                       /* panel_safety_backoff's territory */
    return (uint16_t)trip;
}

/*
 * panel_op_track — maintain the droop reference.
 *
 * Rises are taken immediately: a panel coming back up is never a collapse,
 * and an operating point that has genuinely moved up must arm the guard at
 * the new level straight away. Falls are rate-limited to
 * CHG_DROOP_REF_FALL_MV per PANEL_VREG_INTERVAL_MS, which comfortably
 * follows both the acquisition walk (13.2 V down to 11.9 V over ~6 s on the
 * 14.09 array, ~215 mV/s) and any real irradiance ramp, while being ~25x
 * too slow to follow a collapse (4 V in 20 ms). That asymmetry IS the
 * guard: a reference that could chase the operating point into the knee
 * would disarm itself exactly when it is needed.
 *
 * Cleared whenever the charger is not connected, so a re-activation arms
 * from its own first settled reading rather than from the last session's.
 */
static void panel_op_track(system_ctx_t *ctx)
{
    charger_ctx_t *c = &ctx->charger;

    if (!charger_connected(ctx)) {
        c->panel_op_mv = 0;
        c->droop_count = 0;
        return;
    }

    uint16_t v = ctx->meas.panel_voltage;

    if (v > c->panel_op_mv) {              /* attack: immediate */
        c->panel_op_mv = v;
        c->panel_op_ms = time_now();
        return;
    }

    uint32_t now = time_now();
    if ((now - c->panel_op_ms) < PANEL_VREG_INTERVAL_MS)
        return;
    c->panel_op_ms = now;

    uint16_t fall = (uint16_t)(c->panel_op_mv - v);
    if (fall > CHG_DROOP_REF_FALL_MV)
        fall = CHG_DROOP_REF_FALL_MV;
    c->panel_op_mv -= fall;
}

/*
 * charger_panel_droop_guard — arrest the fall while a backoff still works.
 *
 * The layer that was missing until v0.40. charger_input_guard below judges
 * the raw conversion against the BATTERY (V_bat + CHG_INPUT_LOST_MARGIN_MV
 * = 3.77 V); this one judges it against where the regulator is actually
 * parked (c->panel_op_mv), at CHG_DROOP_TRIP_PCT of it. On a 12.7 V
 * operating point that is 11.2 V, and the raw traces (hw_config.h,
 * CHG_DROOP_TRIP_PCT) show the panel passing through there one to four
 * conversions before it reaches the battery.
 *
 * Why the earlier trip matters more than the earlier sample: at 11.2 V the
 * buck is nowhere near dropout, so raising pwm removes demand directly.
 * Below V_bat it is in dropout, the FB target has no authority over duty,
 * and the only thing that breaks it is commanding a rail under the cell —
 * the whole reason charger_input_guard's backoff is allowed to do something
 * that looks so wrong (see its v0.34 note). Catching the fall one layer up
 * turns a 2.5 s stand-down plus a 6-10 s re-acquisition into a ~10 ms notch
 * in the draw, and it hands mppt.c a knee sighting that cost nothing.
 *
 * Ordering: runs BEFORE charger_input_guard in the super loop. If it
 * arrests the fall, the input guard never sees a sub-margin sample and
 * nothing else happens. If it does not — a genuine removal, or a collapse
 * with no precursor at all (two of the seven traced) — the input guard is
 * underneath it, unchanged, and this guard's own backoff has already moved
 * the draw in the direction that guard wants anyway.
 *
 * This function NEVER stands down and never reduces pwm. It is bounded by
 * CHG_DROOP_MAX_STEPS precisely so that a removed source ends up parked
 * rather than walked to PWM_MIN_DUTY here: "the source is gone" is the
 * input guard's call to make, and it needs the rail state to make it.
 *
 * ⚠️ Gated on adc_sample_seq(), NOT on calls — same reason as every other
 * foreground guard: the super loop runs at kHz against a value that changes
 * every TICK_ADC_MS, so a per-call counter would spend its whole budget on
 * one stale conversion in microseconds.
 */
void charger_panel_droop_guard(system_ctx_t *ctx)
{
    charger_ctx_t *c = &ctx->charger;

    if (!charger_connected(ctx) || c->input_lost_pending) {
        c->droop_count = 0;
        return;
    }

    /* No settled operating point yet (activation, or a session that has not
     * completed one PANEL_VREG_INTERVAL_MS), or a plant on which a droop
     * cannot be told from a regulation step: nothing to judge against. */
    uint16_t trip = panel_droop_trip_mv(ctx);
    if (trip == 0) {
        c->droop_count = 0;
        return;
    }

    uint32_t seq = adc_sample_seq();
    if (seq == c->droop_seq)
        return;                       /* already judged this conversion */
    c->droop_seq = seq;

    if (get_input_voltage_now() >= trip) {
        c->droop_count = 0;           /* holding above the trip: event over */
        return;
    }

    /* Under the trip. Is the PANEL falling, or did the buck just step on its
     * own target? A panel fall cannot push more current into the cell than
     * the average — the buck's output is bounded by what the input supplies
     * — so a raw I_buck far above the average on this same conversion is the
     * input capacitor discharging into a transient of the buck's own making
     * (v0.42, CHG_DROOP_SPIKE_MA: 40 of 48 sightings on 14.09.26 v0.41 were
     * this, at 1.3-1.6 A raw against 0.1-0.5 A average). Nothing to shed
     * there, and nothing for the tracker to learn. */
    int32_t i_raw = (int32_t)get_charge_current_now() +
                    (int32_t)get_discharge_current_now();
    int32_t i_avg = (int32_t)ctx->meas.chg_current +
                    (int32_t)ctx->meas.dsg_current;
    bool    spike = (i_raw > i_avg + (int32_t)CHG_DROOP_SPIKE_MA);

    /* Snapshot the draw that could not be held BEFORE the first backoff
     * moves it — mppt.c reads this as the knee, and reading it after the
     * fact is exactly the bug v0.39 fixed in the input guard's own trace (it
     * fenced 2 backoffs too high, permanently). */
    if (c->droop_count == 0) {
        c->droop_pwm_from    = ctx->pwm;
        c->droop_ichg_from   = ctx->meas.chg_current;
        c->droop_vpanel_from = ctx->meas.panel_voltage;
        c->droop_settled     = (c->settled_ticks >= CHG_PWM_SETTLED_TICKS);

        /* v0.41: this event may shed demand down to zero delivery and no
         * further. The panel is still volts above the cell here, the buck is
         * out of dropout, and past zero the FB target is not "less demand" —
         * it is a rail under the cell and the sync FETs pumping into the
         * panel (14.09.26: 103 -> 111 on a 110 zero-draw, 0x0100 latched
         * 160 ms later). See CHG_ZERO_DRAW_MARGIN. */
        c->droop_limit_pwm = zero_draw_bound_pwm(ctx, ctx->pwm,
                                                 ctx->meas.chg_current,
                                                 ctx->meas.dsg_current,
                                                 0U);

        /* v0.41: post-mortem for main.c — the raw window and the raw buck
         * current at the sighting. Every sighting is traced, glitches
         * included; they are what the log needs to see. */
        adc_panel_trace_mv(c->droop_trace_mv);
        c->droop_trace_ibuck   = (int16_t)i_raw;
        c->droop_trace_trip_mv = trip;
        c->droop_trace_pwm_to  = ctx->pwm;
        c->droop_trace_pending = true;
        c->droop_kind          = spike ? DROOP_KIND_GLITCH : DROOP_KIND_FALL;
        c->droop_counted       = false;
        if (spike && c->droop_glitch_events < UINT16_MAX)
            c->droop_glitch_events++;
    }

    if (c->droop_count >= CHG_DROOP_MAX_STEPS)
        return;                       /* parked: the input guard's call now */
    c->droop_count++;

    if (spike)
        return;                       /* the buck's transient, not the panel */

    /* A FALL conversion — possibly after a glitch sighting, if the transient
     * tipped a marginal panel over; from here it is a real event. */
    c->droop_kind = DROOP_KIND_FALL;

    /* v0.43: shed to the bound in ONE conversion. A fall at the knee under
     * fading light means the panel's available power has just dropped below
     * the demand; no partial notch can hold it, only removing the demand can
     * (the constant-power load line has no stable point left of the new
     * MPP). Bench 14.09.26 13:03 (v0.42): nine FALL sightings, CHG_DROOP_STEP
     * (4) per conversion, and seven of them still reached the input guard —
     * the panel goes from 10.8 V to the cell in one or two conversions, and
     * four counts is ~180 mA against a power deficit of hundreds. Shedding
     * everything costs a re-acquisition (~3 s via HOLD's kicked re-probe);
     * the alternative was a rescue with the rail under the cell or a
     * stand-down. The bound is still zero delivery + CHG_ZERO_DRAW_MARGIN:
     * never a rail under the cell from here. */
    uint16_t next = c->droop_limit_pwm;
    bool stepped = (next > ctx->pwm);   /* false once at zero delivery */

    if (stepped) {
        ctx->pwm = next;
        set_buck_pwm(ctx->pwm);
        /* v0.41: a rescue, exactly as the input guard's is. Open the window
         * in which charger_fast_guard reads the rail's transient as this
         * rescue's tail rather than as reverse pumping (CHG_REVERSE_BLANK_MS). */
        c->input_rescue_ms = time_now();
    }
    c->droop_trace_pwm_to = ctx->pwm;

    /* One event, one sighting for the tracker — counted on the first FALL
     * conversion, not on each of them, so a four-conversion fall does not
     * ratchet the fence four times. Counted even when there was nothing
     * left to shed (pwm:X->X in the DROOP line): the tracker's delivering
     * gate rejects it, and the log gets to see it. */
    if (!c->droop_counted) {
        c->droop_counted = true;
        if (c->droop_events < UINT16_MAX) c->droop_events++;
    }
    if (!stepped)
        return;

    /* Whatever the voltage loop had armed is not a plant measurement any
     * more: the PWM moved underneath it and V_panel is mid-fall. Same
     * reasoning as charger_input_guard's backoff. */
    c->vloop_measure_armed = false;

    /* The draw just went DOWN, so the fence is not what is holding the loop
     * back — clear the starvation stamp for the same reason pwm_step does. */
    c->draw_blocked_ms = 0;
}

/*
 * charger_fast_guard — reverse current into a live input.
 *
 * v0.41 rewrote the decision. The raw single-conversion charge-current sense
 * is ~240 mA RMS (hw_config.h, CHG_REVERSE_FAST_MA), so "three raw samples
 * under -100 mA" was a noise process that latched within a second whenever
 * the delivery sat near zero — which is where every droop backoff, every
 * rescue and every Q49 close puts it. All six 0x0100 latches of the v0.40
 * bench session were that (dips:0 — the input never reached the battery).
 *
 * Two paths now, both blanked for CHG_REVERSE_BLANK_MS after any guard
 * rescue and for CHG_CONNECT_BLANK_MS after Q49 closes:
 *   FAST  — raw I_buck under -CHG_REVERSE_FAST_MA (~2σ) on CHG_REVERSE_SAMPLES
 *           distinct conversions. Catches a May-class -1 A reversal in
 *           ~30 ms; at zero true current it false-trips about once an hour.
 *   AVG   — the 64-sample averaged I_buck under -CHG_REVERSE_CURRENT_MA for
 *           CHG_REVERSE_LATCH_MS with the input live. cc_regulate's reverse
 *           escape acts on the same signal first (-PANEL_BACKOFF_STEP per
 *           interval); a reversal it has not cleared after 2-3 steps is
 *           sustained and real.
 * The dead-input branch is unchanged (v0.26): reverse current with the input
 * already under V_bat is a disconnect charger_input_guard lost the race to,
 * and it stands down without latching.
 *
 * chg_current (R440) is NET cell current: I_cell = I_buck − I_load. An
 * activation under load connects Q49 with the buck at ~zero delivery, so
 * I_cell legitimately reads −I_load until cc_regulate walks the delivery up
 * — judging chg alone here latched this fault on every lamps-on activation
 * (bench 2026-07-29). Genuine reverse pumping means the BUCK branch is
 * negative, so judge I_buck = I_cell + I_dsg on both paths.
 */
void charger_fast_guard(system_ctx_t *ctx)
{
    charger_ctx_t *c = &ctx->charger;

    /* A stand-down already isolated the path this tick; nothing left to judge
     * (and chg_current reads 0 with Q49 open anyway). */
    if (!charger_connected(ctx) || c->input_lost_pending) {
        c->reverse_count        = 0;
        c->reverse_avg_since_ms = 0;
        return;
    }

    uint32_t now        = time_now();
    int32_t  i_buck_now = (int32_t)get_charge_current_now() +
                          (int32_t)get_discharge_current_now();
    int32_t  i_buck_avg = (int32_t)ctx->meas.chg_current +
                          (int32_t)ctx->meas.dsg_current;

    /* Reverse current with the input already DEAD is a disconnect whose
     * voltage signature charger_input_guard narrowly lost the race to — the
     * input node can fall past the crossover inside one 10 ms conversion
     * interval. Stand down cleanly; latching there is what made every source
     * removal look like a charge over-current (bench 2026-07-30). A single
     * raw sample is enough here because the cost of being wrong is
     * CHG_INPUT_REARM_MS, not a 10 s fault. */
    if (i_buck_now < -(int32_t)CHG_REVERSE_CURRENT_MA &&
        !charger_input_present()) {
        charger_input_stand_down(ctx);
        return;
    }

    /* Inside a rescue window the sign of the buck current says nothing: a
     * rescued collapse ends with the panel back at full voltage while the
     * buck still carries the backed-off target, and the droop guard's notch
     * is the same transient one layer up (v0.32 latched 0x0100 beside every
     * INTRACE RECOVERED of the 10.09.26 session). Same for the first
     * conversions after Q49 closes, where the delivery is zero by
     * construction (467 of 498 sessions on 10.09.26 died there). Yield, and
     * start both debounces afresh afterwards. */
    bool rescuing = (c->input_rescue_ms != 0) &&
                    ((now - c->input_rescue_ms) < CHG_REVERSE_BLANK_MS);
    if (rescuing || (now - c->connect_ms) < CHG_CONNECT_BLANK_MS) {
        c->reverse_count        = 0;
        c->reverse_avg_since_ms = 0;
        return;
    }

    uint8_t kind = 0;

    /* FAST path: a large raw reversal on distinct conversions. Gated on
     * adc_sample_seq() — this runs at super-loop rate against a value that
     * only moves every TICK_ADC_MS, so counting calls would reach the
     * threshold off one stale sample within microseconds. */
    uint32_t seq = adc_sample_seq();
    if (seq != c->reverse_seq) {
        c->reverse_seq = seq;
        if (i_buck_now < -(int32_t)CHG_REVERSE_FAST_MA) {
            if (c->reverse_count < CHG_REVERSE_SAMPLES)
                c->reverse_trace_ibuck[c->reverse_count] = (int16_t)i_buck_now;
            if (++c->reverse_count >= CHG_REVERSE_SAMPLES)
                kind = 1;
        } else {
            c->reverse_count = 0;
        }
    }

    /* AVG path: a moderate reversal the regulator's escape has not cleared. */
    if (kind == 0) {
        if (i_buck_avg < -(int32_t)CHG_REVERSE_CURRENT_MA) {
            if (c->reverse_avg_since_ms == 0)
                c->reverse_avg_since_ms = (now != 0) ? now : 1U;
            else if ((now - c->reverse_avg_since_ms) >= CHG_REVERSE_LATCH_MS)
                kind = 2;
        } else {
            c->reverse_avg_since_ms = 0;
        }
    }

    if (kind == 0)
        return;

    /* Latch. FAULT_REVERSE_PUMP's action opens Q49 before disabling the buck;
     * energy_mode's fault-clear re-arm subsequently restarts from
     * CHG_BUCK_SETTLE, never directly connected. Keep the evidence for the
     * log line (main.c log_reverse_trace). */
    c->reverse_trace_kind    = kind;
    c->reverse_trace_avg     = (int16_t)i_buck_avg;
    c->reverse_trace_pwm     = ctx->pwm;
    c->reverse_trace_vpanel  = get_input_voltage_now();
    if (kind == 2) {
        c->reverse_trace_ibuck[0] = (int16_t)i_buck_now;
        for (uint8_t i = 1; i < CHG_REVERSE_SAMPLES; i++)
            c->reverse_trace_ibuck[i] = 0;
    }
    c->reverse_trace_pending = true;

    fault_raise(ctx, FAULT_REVERSE_PUMP);
    ctx->pwm = PWM_MIN_DUTY;
    c->reverse_count        = 0;
    c->reverse_avg_since_ms = 0;
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
    ctx->charger.vloop_measure_armed = false;  /* not a plant measurement */
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
/* =========================================================================
 * Adaptive regulation band
 * ========================================================================= */

/*
 * charger_vreg_deadband_mv — half-width of the V_panel regulation band.
 *
 * The band's job is to guarantee a reachable operating point inside it:
 * one PWM count must move V_panel by LESS than the band, or the loop hops
 * clean across it every interval and whipsaws the panel over its knee.
 * That is a statement about PWM counts, and PANEL_VREG_DEADBAND_MV is only
 * what it evaluated to on the panel it was tuned against — see the adaptive
 * deadband note in hw_config.h, and bench 14.09.26 for the 2.5 W it costs
 * on a panel with a 20x smaller gain.
 *
 * So: derive it from the gain the loop has actually measured, fall back to
 * the static value until there is one, and never exceed it. Shared with
 * mppt.c, which sizes the FOCV seed, the setpoint clamps and the dip
 * classifier off the same band.
 */
uint16_t charger_vreg_deadband_mv(const system_ctx_t *ctx)
{
    uint32_t gain = ctx->charger.plant_mv_per_count;
    if (gain == 0)
        return PANEL_VREG_DEADBAND_MV;      /* nothing measured yet */

    uint32_t db = gain * PANEL_VREG_DEADBAND_COUNTS;
    if (db < PANEL_VREG_DEADBAND_MIN_MV) db = PANEL_VREG_DEADBAND_MIN_MV;
    if (db > PANEL_VREG_DEADBAND_MV)     db = PANEL_VREG_DEADBAND_MV;
    return (uint16_t)db;
}

#if CHARGER_INPUT_VREG
/*
 * vloop_observe_gain — fold the last paced step into the gain estimate.
 *
 * Called once per PANEL_VREG_INTERVAL_MS at the top of cc_regulate, before
 * anything moves the PWM, so ctx->meas.panel_voltage is the settled answer
 * to the step taken one interval ago (the interval is >= the 64-sample ADC
 * group delay by construction — that is why it exists).
 *
 * Only steps the voltage loop took alone are measurable, and only if the
 * panel moved the way the plant says it must (pwm DOWN → draw more → V
 * falls). A sample that disagrees is not the plant: it is an irradiance
 * step, a collapse tail, or the guard having moved the PWM underneath us.
 * Throw those away rather than filtering them — a single wrong-signed
 * sample dragged through the IIR is a wrong band for seconds.
 */
static void vloop_observe_gain(system_ctx_t *ctx)
{
    charger_ctx_t *c = &ctx->charger;

    if (!c->vloop_measure_armed)
        return;
    c->vloop_measure_armed = false;

    int32_t d_pwm = (int32_t)c->vloop_prev_pwm - (int32_t)ctx->pwm;
    int32_t d_v   = (int32_t)c->vloop_prev_vp_mv -
                    (int32_t)ctx->meas.panel_voltage;
    if (d_pwm == 0)
        return;

    /* pwm DOWN (d_pwm > 0) must lower V_panel (d_v > 0), and vice versa:
     * the ratio is positive on a real plant reading, either direction. */
    if ((d_pwm > 0) != (d_v > 0))
        return;

    if (d_pwm < 0) { d_pwm = -d_pwm; d_v = -d_v; }

    uint32_t sample = (uint32_t)d_v / (uint32_t)d_pwm;
    if (sample < PANEL_GAIN_MIN_MV_PER_COUNT ||
        sample > PANEL_GAIN_MAX_MV_PER_COUNT)
        return;                              /* not a plant measurement */

    c->vloop_probe_steps = 0;                       /* this source responds */

    /* ASYMMETRIC on purpose. The gain is not one number: measured per count
     * on the 14.09.26 panel it is 16-55 mV/count across the flat stretch
     * (pwm 62-105) and 138-192 mV/count at the knee (pwm 59-61) — an order
     * of magnitude, on one curve, and the steep end is the end that
     * collapses. Underestimating it is the dangerous direction: the step law
     * divides by it, so a stale flat-region estimate carried into the knee
     * asks for the maximum step exactly where the plant moves furthest per
     * count. So believe a STEEPER reading immediately and filter only the
     * flatter ones. The band widens with it, which damps the approach to the
     * knee by itself. */
    if (c->plant_mv_per_count == 0 || sample > c->plant_mv_per_count) {
        c->plant_mv_per_count = (uint16_t)sample;
    } else {
        uint32_t old = c->plant_mv_per_count;
        c->plant_mv_per_count = (uint16_t)
            (old - ((old - sample) >> PANEL_GAIN_IIR_SHIFT));
    }
}

/*
 * vloop_step_counts — how many counts to move for this error.
 *
 * One count per interval is the dead-time rule near the setpoint, and it
 * stays one until the gain is known. With a gain, a long traverse (Voc down
 * to the seed: 38-70 counts on the 14.09.26 panel, 15-28 s at one count per
 * 400 ms, the whole of it with the tracker dwelling on a moving plant) can
 * be sized to the error instead. Still closed-loop: the step never exceeds
 * what the remaining error justifies, and PANEL_VREG_STEP_MAX keeps a
 * mis-estimated gain from leaping the knee in one interval.
 */
static int32_t vloop_step_counts(const system_ctx_t *ctx, int32_t err_mv)
{
    uint32_t gain = ctx->charger.plant_mv_per_count;
    if (gain == 0)
        return PANEL_VREG_STEP;

    if (err_mv < 0) err_mv = -err_mv;
    int32_t n = err_mv / (int32_t)gain;
    if (n < PANEL_VREG_STEP)     n = PANEL_VREG_STEP;
    if (n > PANEL_VREG_STEP_MAX) n = PANEL_VREG_STEP_MAX;
    return n;
}

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

    /* The step taken one interval ago has now settled through the ADC
     * average: read the plant's gain off it before anything moves again. */
    vloop_observe_gain(ctx);

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
        return;   /* not the voltage loop's step: no gain sample */
    }

    /* Clamp 1: never exceed the battery's allowed intake. Over-current
     * always wins → reduce current → pwm UP (toward off). */
    if (i_chg > i_limit + CC_DEADBAND_MA) {
        pwm_step(ctx, +PANEL_VREG_STEP);
        return;   /* current-limited, not voltage-limited: no gain sample */
    }

    /* Clamp 1b (v0.41): the fence is TWO-SIDED. mppt.c moves cliff_pwm_min
     * both ways — down to probe, back up to reject a probe or to fence above
     * a knee it just learned — and pwm_draw_more only ever refuses to go
     * under it; nothing brought the rail back UP when the fence rose. Bench
     * 14.09.26 v0.40: 33.8 % of all fenced CC samples had the actual pwm
     * under the fence, one stretch for 121 s, and a "knee" was learned from
     * two dwells measured at the same physical count (173.8-177.6 s: fence
     * 106 -> 103 -> 106 -> 105 -> HOLD, pwm 103 throughout). Realise a raised
     * fence here, at the loop's own pace and step bound. It is a reduction,
     * so it is always safe; it is not the voltage loop's step, so it leaves
     * no gain sample. */
    uint16_t fence = ctx->mppt.cliff_pwm_min;
    if (fence != 0 && ctx->pwm < fence) {
        int32_t up = (int32_t)fence - (int32_t)ctx->pwm;
        if (up > PANEL_VREG_STEP_MAX) up = PANEL_VREG_STEP_MAX;
        pwm_step(ctx, +up);
        return;
    }

    /* Clamp 2: regulate the panel to the MPP setpoint. The target is the
     * LIVE per-panel value owned by the outer MPPT loop (seeded from
     * MPPT_SP_FRACTION_PCT·Voc on a fresh activation, then hill-climbed —
     * see mppt.c), NOT the static PANEL_VREG_SETPOINT_MV, which is only its
     * cold-boot seed. */
    int32_t v_sp = (int32_t)ctx->mppt.vreg_setpoint_mv;
    int32_t band = (int32_t)charger_vreg_deadband_mv(ctx);
    int32_t err  = v_panel - v_sp;

    /* Arm the gain measurement: from here on the only thing that moves the
     * PWM this interval is the voltage loop itself, so whatever V_panel has
     * done by the next call is this step's answer. Recorded BEFORE the step
     * (vloop_observe_gain differences against ctx->pwm as it stands then). */
    uint16_t pwm_before = ctx->pwm;

    if (err < -band) {
        /* Sagging below MPP → drawing too much → DRAW LESS (pwm UP) → V recovers. */
        pwm_step(ctx, +vloop_step_counts(ctx, err));
        ctx->charger.last_backoff_ms = now;   /* v0.44: the light is falling */
    } else if (err > band) {
        /* Above MPP with current-headroom (clamp 1 didn't fire) → DRAW MORE (pwm DOWN) → V falls.
         * Fenced by the learned PWM ceiling: this branch is what walked the
         * panel off its knee in the first two teardowns of the 10.09.26
         * session (V_panel 1.5 V above the band, one step per interval,
         * straight into the collapse).
         *
         * It is also the branch the 14.09.26 log needed and never got: the
         * band was 1200 mV wide against a 1036 mV error, so a converged,
         * correct setpoint sat uncommanded for 211 s at half power. The band
         * is now sized to the measured plant — see charger_vreg_deadband_mv. */
        pwm_draw_more(ctx, vloop_step_counts(ctx, err));
    } else if (ctx->charger.plant_mv_per_count == 0 &&
               ctx->charger.vloop_probe_steps < CHG_VLOOP_PROBE_MAX &&
               (err > (int32_t)PANEL_VREG_DEADBAND_MIN_MV ||
                err < -(int32_t)PANEL_VREG_DEADBAND_MIN_MV)) {
        /* Learning probe: in-band by the WIDE fallback band only because the
         * gain that would narrow it has never been measured — and it never
         * will be while the loop holds still. Step toward the setpoint: the
         * right direction regardless, and the excitation that breaks the
         * circle. See CHG_VLOOP_PROBE_MAX. */
        ctx->charger.vloop_probe_steps++;
        if (err > 0) pwm_draw_more(ctx, PANEL_VREG_STEP);
        else         pwm_step(ctx, +PANEL_VREG_STEP);
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

    /* A step of at least one count, taken by the voltage loop alone, is a
     * clean gain sample one interval from now. Holding is not (no
     * excitation), and neither is a step the fence swallowed. */
    if (ctx->pwm != pwm_before) {
        ctx->charger.vloop_prev_pwm      = pwm_before;
        ctx->charger.vloop_prev_vp_mv    = (uint16_t)v_panel;
        ctx->charger.vloop_measure_armed = true;
    }
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

    if (v_bat > (BAT_CV_VOLTAGE_MV + CV_DEADBAND_MV)) {
        /* Above target — reduce current, voltage will drift down. */
        pwm_step(ctx, +CV_PWM_STEP);
        return;
    }

    /* v0.41: the fence is two-sided here too (see cc_regulate clamp 1b). A
     * fence raised while CV was already under it — a droop in CV — would
     * otherwise be held under indefinitely by a V_bat loop that is content
     * where it is. Same pace as CV's own step. */
    uint16_t fence = ctx->mppt.cliff_pwm_min;
    if (fence != 0 && ctx->pwm < fence) {
        pwm_step(ctx, +CV_PWM_STEP);
        return;
    }

    if (v_bat < BAT_CV_VOLTAGE_MV) {
        /* Below target — need more current to pull voltage up. Fenced by the
         * learned PWM ceiling like CC's draw-more branches: the knee does not
         * care which state asked. It can only bind if CV wants more current
         * than the count that already collapsed the panel, in which case the
         * unfenced alternative is another teardown, not a faster taper. */
        pwm_draw_more(ctx, CV_PWM_STEP);
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
         *
         * v0.41: the walk lands ON the learned fence rather than through it,
         * and if the rail still has not cleared the cell after sitting on the
         * fence for CHG_PWM_SETTLED_TICKS/2 (the slew has had ~300 ms), the
         * fence sits at or above the zero-delivery count and is provably
         * wrong: discard it, knee and all, and finish the acquisition
         * unfenced. That replaces v0.40's snap-to-fence at the Q49 close,
         * which could command a rail the interlock had never validated
         * (14.09.26: SETL cleared the cell at 108, snapped to a fence of 110
         * — the zero-draw count — and latched 0x0100 500 ms later). The
         * fence may not block this ramp; it may only be found wrong by it.
         */
        if (ctx->meas.panel_voltage >= PANEL_SAFETY_MV &&
            (now - c->cc_last_downstep_ms) >= CHG_BUCK_SETTLE_RAMP_MS) {
            c->cc_last_downstep_ms = now;

            int32_t  step  = CHG_BUCK_SETTLE_PWM_STEP;
            uint16_t fence = ctx->mppt.cliff_pwm_min;
            if (fence != 0) {
                if (ctx->pwm > fence) {
                    if (step > (int32_t)(ctx->pwm - fence))
                        step = (int32_t)(ctx->pwm - fence);
                } else if (c->settled_ticks >= (CHG_PWM_SETTLED_TICKS / 2U)) {
                    ctx->mppt.cliff_pwm_min  = 0;
                    ctx->mppt.knee_pwm       = 0;
                    ctx->mppt.knee_learned_ms = 0;
                    if (ctx->mppt.fence_dropped < UINT16_MAX)
                        ctx->mppt.fence_dropped++;
                } else {
                    step = 0;             /* on the fence: let the rail slew */
                }
            }
            if (step > 0)
                pwm_step(ctx, -step);
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
    c->connect_ms           = now;
    c->reverse_count        = 0;
    c->reverse_avg_since_ms = 0;

    /* The ramp above lands on the fence rather than through it (v0.41), so
     * there is nothing to re-assert here; if it does close under the fence
     * (a fence raised while SETL was already past it), cc_regulate's
     * two-sided fence rule walks the rail back up at the paced rate. This is
     * the one path that lands PWM on the fence without going through
     * pwm_draw_more, so the dwell it would otherwise inherit was served by a
     * previous session. The loop starts fresh here. */
    c->draw_blocked_ms = 0;

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

    /* Refresh the fast droop guard's reference before anything else moves:
     * it is the only thing standing between a knee excursion and a 2.5 s
     * stand-down, and it must describe THIS tick's operating point. */
    panel_op_track(ctx);

    /* ── Counter-evidence for the cliff fence ──
     *
     * Record the highest count this session has been SEEN delivering at, so
     * learn_cliff_pwm can never fence at or above a draw the panel actually
     * sustained (hw_config.h, CHG_DELIVERING_MIN_MA). Gated on the count
     * having been held CHG_PWM_SETTLED_TICKS: chg_current is a 64-sample
     * average, so a reading taken less than 650 ms after a step still
     * describes the previous operating point.
     *
     * Runs before the per-state tick so the dwell is measured against the
     * PWM that produced this tick's measurement, not the one about to be
     * written. */
    if (ctx->pwm == c->settled_pwm) {
        if (c->settled_ticks < CHG_PWM_SETTLED_TICKS)
            c->settled_ticks++;
    } else {
        c->settled_pwm   = ctx->pwm;
        c->settled_ticks = 0;
    }

    if (charger_connected(ctx) &&
        c->settled_ticks >= CHG_PWM_SETTLED_TICKS &&
        ctx->meas.chg_current >= (int16_t)CHG_DELIVERING_MIN_MA) {
        if (ctx->pwm > c->delivering_pwm)
            c->delivering_pwm = ctx->pwm;

        /* v0.41: and where zero delivery is, from the same settled reading —
         * pwm + I_buck / CHG_BUCK_MA_PER_COUNT (I_buck = I_cell + I_load).
         * Refreshed on every delivering tick so it follows V_bat; the last
         * value stands while the buck idles. Bounds the droop guard's
         * backoff and places the input guard's post-rescue restore. */
        int32_t  i_buck = (int32_t)ctx->meas.chg_current +
                          (int32_t)ctx->meas.dsg_current;
        uint32_t zero   = (uint32_t)ctx->pwm +
                          ((uint32_t)i_buck + CHG_BUCK_MA_PER_COUNT - 1U) /
                              CHG_BUCK_MA_PER_COUNT;
        if (zero > PWM_MIN_DUTY) zero = PWM_MIN_DUTY;
        c->zero_draw_pwm = (uint16_t)zero;
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
