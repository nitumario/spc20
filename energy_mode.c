/*
 * energy_mode.c — Energy Mode FSM 
 * ===================================================
 *
 * Implements the transition table from docs/transition_table.csv.
 *
 * Execution model:
 *   Called once per TICK_MAIN_MS (50 ms) from the main loop, after
 *   measurements, flags, and power_budget have run.
 *
 *   1. Evaluate transition guards in priority order for the current state
 *   2. If a transition fires, apply exit actions for old state
 *   3. Apply entry actions for new state (set hardware enables)
 *   4. If no transition fires, stay — no action needed
 *
 * Guard inputs (all read-only, set by earlier pipeline steps):
 *   ctx->flag_has_sun.value   — debounced, from flags_update()
 *   ctx->flag_bat_low.value   — debounced, from flags_update()
 *   ctx->has_load              — hysteresis, from flags_update()
 *   ctx->bat_full              — set by charger when CV taper completes
 *   ctx->meas.bat_voltage      — raw mV, used only for SAFE_MODE recovery
 *                                (intentionally not debounced — see CHANGELOG)
 *
 * Hardware outputs:
 *   CHARGER_EN  — enable_charge_switch() / disable_charge_switch()
 *   BATTERY_EN  — enable_battery_switch() / disable_battery_switch()
 *   OUTPUT_EN   — enable_output_switch() / disable_output_switch()
 *   USB_EN      — enable_usb_boost() / disable_usb_boost()
 *   BUCK_DIS    — disable_input_buck() / enable_input_buck()
 *
 * Charger/MPPT region control:
 *   When energy_mode deactivates the charger, it sets:
 *     ctx->charger.state = CHG_INACTIVE
 *     ctx->pwm = PWM_MIN_DUTY
 *     disable_input_buck()   (assert BUCK_DIS)
 *   This matches the charger's "deactivated → INACTIVE" exit action
 *   from charger_states.csv.
 *
 *   Deactivating the charger does NOT reset mppt_limit_ma: the learned
 *   panel capability is PRESERVED so a brief EM bounce can't release
 *   allowed_chg back to BUCK_MAX and re-overload the panel. MPPT itself
 *   transitions to DISABLED on its next tick (mppt_update sees !charging),
 *   which also preserves the limit. (Under CHARGER_INPUT_VREG=1 the limit
 *   is BUCK_MAX at all times anyway — the panel is protected by the
 *   voltage loop, not a current budget.) See deactivate_charger_region()
 *   and the mppt_limit_ma comment in system_types.h.
 */

#include "energy_mode.h"
#include "SPCBoardAPI.h"

/* ── Shorthand for flag reads ── */
#define HAS_SUN    (ctx->flag_has_sun.value)
#define BAT_LOW    (ctx->flag_bat_low.value)
#define HAS_LOAD   (ctx->has_load)
#define BAT_FULL   (ctx->bat_full)
#define V_BAT      (ctx->meas.bat_voltage)

/* =========================================================================
 * ENTRY / EXIT ACTIONS
 * =========================================================================
 *
 * Each state has a fixed hardware configuration. On entry, we set all
 * enables to the correct state. This is idempotent — calling enter_idle()
 * when already in IDLE is harmless (GPIO writes are unconditional).
 *
 * Deactivating the charger region is an exit action that fires whenever
 * we leave a charging state (CHARGE_ONLY or CHARGE_AND_LOAD) for a
 * non-charging state. It resets the charger and MPPT to safe defaults.
 */

/*
 * deactivate_charger_region — safe shutdown of charger + MPPT
 *
 * Called when transitioning OUT of a charging state. Matches the
 * "deactivated → INACTIVE" exit action in charger_states.csv:
 *   pwm = PWM_MIN_DUTY, assert BUCK_DIS, reset charger state.
 *
 * Also resets MPPT to DISABLED so it doesn't hold a stale mppt_limit.
 */
static void deactivate_charger_region(system_ctx_t *ctx)
{
    /* Buck off — stop pushing current immediately */
    ctx->pwm = PWM_MIN_DUTY;
    disable_input_buck();

    /* Charger → INACTIVE, clear timing state */
    ctx->charger.state = CHG_INACTIVE;
    ctx->charger.precharge_start_ms = 0;
    ctx->charger.active_start_ms = 0;
    ctx->charger.cc_last_downstep_ms = 0;
    ctx->charger.bat_full_timing = false;
    ctx->charger.bat_full_signaled = false;

    /* MPPT: state transitions are handled by mppt_update() seeing
     * !charging on its next tick. We DO NOT reset mppt_limit_ma here.
     * The learned panel capability must survive across brief EM
     * bounces — otherwise the next reactivation releases allowed_chg
     * to BUCK_MAX, the buck overloads the panel, V_panel collapses
     * below PANEL_MIN_CLEAR_MV, has_sun clears, EM goes back to IDLE,
     * and we oscillate. See mppt_limit_ma comment in system_types.h. */

    /* bat_full is a system-level flag consumed by energy_mode.
     * Clear it when charger is deactivated — a new charge cycle
     * must re-detect taper completion. */
    ctx->bat_full = false;
}

/*
 * activate_charger_region — prepare charger for operation
 *
 * Called when transitioning INTO a charging state. The charger will
 * determine its own initial state (PRECHARGE or CC) based on V_bat
 * on its next tick — we just enable the hardware paths.
 *
 * Only called if charger is currently INACTIVE (avoids resetting
 * a charger that's already running, e.g., CHARGE_ONLY → CHARGE_AND_LOAD).
 */
static void activate_charger_region(system_ctx_t *ctx)
{
    if (ctx->charger.state != CHG_INACTIVE)
        return;  /* already active — don't reset mid-charge */

    /*
     * Pre-position PWM to the LUT entry that produces ≈ V_bat +
     * CHG_ACTIVATION_HEADROOM_MV at the buck rail, so the converter
     * enters conduction with positive forward bias across Q49 on the
     * very first tick. Without the headroom the buck would target
     * VCHG = V_bat exactly, the TPS564247 sync FETs would reverse-pump
     * inductor current into V_panel, and CC_DOWNSTEP_INTERVAL_MS would
     * prevent CC from escaping before reverse current accumulates.
     * See CHG_ACTIVATION_HEADROOM_MV in hw_config.h for the full
     * physics rationale.
     *
     * Without any pre-position at all, ctx->pwm is held at PWM_MIN_DUTY
     * (399, ~0.25% duty) by deactivate_charger_region(), CC walks it
     * down one count per CC_DOWNSTEP_INTERVAL_MS (≈40 s from 399 to a
     * conducting duty), the MPPT settle gate expires mid-walk, and
     * MPPT engages on a non-conducting buck — parking mppt_limit_ma
     * at a phantom ~12 mA that zeros allowed_chg and deadlocks CC.
     *
     * set_charging_voltage() also writes the timer register as a
     * side-effect, but that's harmless: apply_pwm() at step 8 will
     * re-commit ctx->pwm this tick anyway.
     */
    ctx->pwm = set_charging_voltage(ctx->meas.bat_voltage + CHG_ACTIVATION_HEADROOM_MV);

    enable_input_buck();
    enable_charge_switch();

    /* Charger will self-determine PRECHARGE vs CC on its next tick
     * based on ctx->meas.bat_voltage. We leave state as INACTIVE
     * and let charger_update() handle the activated transition. */
}

/* ── Per-state entry actions ── */

/* Reconcile the shared LED boost rail (TPS61088) with lamp state: hold it up
 * iff at least one lamp is lit, shed it when every lamp is off. The rail powers
 * *only* the LED lamp strings, so with all lamps off there is nothing for it to
 * power — and a lamp commanded "off" still sits at the ~15 mA compare-99 current
 * floor (a faint glow, see set_led_current()), so the ONLY way to make "off"
 * fully dark is to drop this rail, exactly as sleep does. Safe to gate here:
 * wake-on-load is sensed on the 3VOUT branch (R432 / has_load), which is fed by
 * the output switch + USB boost and is independent of this rail, so shedding it
 * loses no load detection. fault_mgr and SAFE_MODE still shed the rail directly;
 * this only governs the lit/dark choice while the rail is otherwise allowed up.
 * main()'s lamp handler calls this too, so a button press toggles the rail
 * without waiting for a state transition. */
void led_boost_follow_lamps(system_ctx_t *ctx)
{
    for (uint8_t i = 0; i < 4; i++) {
        if (ctx->lamp_level[i] != 0) {
            enable_led_boost();
            return;
        }
    }
    disable_led_boost();
}

static void enter_idle(system_ctx_t *ctx)
{
    disable_charge_switch();
    /* BATTERY_EN + OUTPUT_EN stay on so I_DISCHARGE (across R427, downstream
     * of Q46) can register a load on 3VOUT and wake us into DISCHARGE_ONLY.
     * USB boost (MIC2876) is held on so USB-connector loads can draw current
     * and trip has_load. The LED boost follows lamp state (dark rail when all
     * lamps are off) — LED outputs are passive (LED string + PNP CC), so the
     * current source must be running for any current to flow, see
     * set_led_current() in main(). */
    enable_battery_switch();
    enable_output_switch();
    enable_usb_boost();
    led_boost_follow_lamps(ctx);
    disable_input_buck();

    ctx->idle_start_ms = time_now();
    ctx->idle_sleep_pending = false;
}

static void enter_charge_only(system_ctx_t *ctx)
{
    enable_battery_switch();
    /* OUTPUT_EN + USB_EN on for the same reason as in enter_idle — otherwise a
     * load appearing mid-charge can never trip has_load and we'd never advance
     * to CHARGE_AND_LOAD. LED boost follows lamp state (see led_boost_follow_lamps). */
    enable_output_switch();
    enable_usb_boost();
    led_boost_follow_lamps(ctx);

    activate_charger_region(ctx);
}

static void enter_charge_and_load(system_ctx_t *ctx)
{
    enable_battery_switch();
    enable_output_switch();
    enable_usb_boost();
    led_boost_follow_lamps(ctx);

    activate_charger_region(ctx);
}

static void enter_discharge_only(system_ctx_t *ctx)
{
    disable_charge_switch();
    disable_input_buck();
    enable_battery_switch();
    enable_output_switch();
    enable_usb_boost();
    led_boost_follow_lamps(ctx);
}

static void enter_safe_mode(system_ctx_t *ctx)
{
    disable_charge_switch();
    disable_input_buck();
    enable_battery_switch();   /* battery stays connected (not draining into loads) */
    disable_output_switch();   /* shed loads */
    disable_usb_boost();       /* shed USB */
    disable_led_boost();       /* shed LED boost */

    /* The charge path is shed on every SAFE_MODE entry. If a supervised
     * undervolt rescue is warranted (see safe_mode_rescue_tick), the in-state
     * handler brings the buck + charge switch back up on the next tick — a
     * deliberate re-arm, never a stale enable carried across the transition. */

    /* SAFE_MODE also sleeps: with loads shed, the awake MCU and its support
     * rails are the dominant drain on an already-low cell, so after
     * IDLE_SLEEP_TIMEOUT_MS of unchanged SAFE_MODE the system
     * drops into STANDBY and wake-checks for V_bat recovery / buttons.
     * idle_start_ms doubles as the inactivity anchor for both IDLE and
     * SAFE_MODE — they are mutually exclusive states. */
    ctx->idle_start_ms = time_now();
    ctx->idle_sleep_pending = false;
}

/* =========================================================================
 * STATE TRANSITION LOGIC
 * =========================================================================
 *
 * Each function evaluates guards in strict priority order (lower number
 * = higher priority = checked first). Returns the new state.
 *
 * If no guard fires, returns the current state (no transition).
 *
 * The priority ordering is critical for safety:
 *   - SAFE_MODE transitions are always priority 1 where applicable
 *   - More specific guards come before less specific ones
 *   - <default> (stay) is always last
 */

static energy_mode_state_t eval_idle(const system_ctx_t *ctx)
{
    /* P1: sun + load + not full → charge and load */
    if (HAS_SUN && HAS_LOAD && !BAT_FULL)
        return EM_CHARGE_AND_LOAD;

    /* P2: sun + no load + not full → charge only */
    if (HAS_SUN && !HAS_LOAD && !BAT_FULL)
        return EM_CHARGE_ONLY;

    /* P3: no sun + load + battery ok → discharge */
    if (!HAS_SUN && HAS_LOAD && !BAT_LOW)
        return EM_DISCHARGE_ONLY;

    /* P4: no sun + battery low → safe mode.
     * No HAS_LOAD requirement: IDLE holds the USB + LED boosts on for load
     * detection, and those idle-draw from the battery. With no usable sun to
     * replenish it, a low battery must shed them or it bleeds to death (the
     * original guard required a load and so never fired in the no-load case,
     * letting the cell run down to the 2.0 V undervolt fault). */
    if (!HAS_SUN && BAT_LOW)
        return EM_SAFE_MODE;

    /* P5: default — stay idle */
    return EM_IDLE;
}

static energy_mode_state_t eval_charge_only(const system_ctx_t *ctx)
{
    /* P1: no usable sun + battery low → safe mode (safety first).
     * Higher priority than the load/idle guards: if the panel can't charge
     * (has_sun now reflects usable power, not just Voc) and the battery has
     * fallen to bat_low, shed the housekeeping boosts instead of dropping to
     * IDLE, which would leave them on and keep bleeding the cell. When the
     * sun IS usable we keep charging even at bat_low — that's how a depleted
     * battery recovers, so this guard is gated on !HAS_SUN. */
    if (!HAS_SUN && BAT_LOW)
        return EM_SAFE_MODE;

    /* P2: load appeared → charge and load */
    if (HAS_LOAD)
        return EM_CHARGE_AND_LOAD;

    /* P3: sun lost → idle */
    if (!HAS_SUN)
        return EM_IDLE;

    /* P4: battery full → idle */
    if (BAT_FULL)
        return EM_IDLE;

    /* P5: stay */
    return EM_CHARGE_ONLY;
}

static energy_mode_state_t eval_charge_and_load(const system_ctx_t *ctx)
{
    /* P1: sun lost + battery low → safe mode (safety first) */
    if (!HAS_SUN && BAT_LOW)
        return EM_SAFE_MODE;

    /* P2: sun lost + battery ok → discharge */
    if (!HAS_SUN && !BAT_LOW)
        return EM_DISCHARGE_ONLY;

    /* P3: full + no load → idle */
    if (BAT_FULL && !HAS_LOAD)
        return EM_IDLE;

    /* P4: full + load → discharge (stop charging, feed from battery) */
    if (BAT_FULL && HAS_LOAD)
        return EM_DISCHARGE_ONLY;

    /* P5: load removed + not full → charge only */
    if (!HAS_LOAD && !BAT_FULL)
        return EM_CHARGE_ONLY;

    /* P6: stay */
    return EM_CHARGE_AND_LOAD;
}

static energy_mode_state_t eval_discharge_only(const system_ctx_t *ctx)
{
    /* P1: battery low + load → safe mode */
    if (BAT_LOW && HAS_LOAD)
        return EM_SAFE_MODE;

    /* P2: battery low + no load → idle */
    if (BAT_LOW && !HAS_LOAD)
        return EM_IDLE;

    /* P3: sun appeared + load → charge and load */
    if (HAS_SUN && HAS_LOAD)
        return EM_CHARGE_AND_LOAD;

    /* P4: sun appeared + no load → charge only */
    if (HAS_SUN && !HAS_LOAD)
        return EM_CHARGE_ONLY;

    /* P5: load removed → idle */
    if (!HAS_LOAD)
        return EM_IDLE;

    /* P6: stay */
    return EM_DISCHARGE_ONLY;
}

static energy_mode_state_t eval_safe_mode(const system_ctx_t *ctx)
{
    /* While the protection wake probe is busy, V_bat is either being
     * DRIVEN by the buck (a 3.65 V reading with no battery attached) or
     * still flushing through the moving average — recovery must not be
     * judged on it. Hold SAFE_MODE; the probe is bounded to a few seconds. */
    if (bat_wake_probe_busy(ctx))
        return EM_SAFE_MODE;

    /*
     * SAFE_MODE recovery requires V_bat > BAT_SAFE_RECOVER_MV (3200 mV).
     * This is a raw voltage comparison, NOT the debounced bat_low flag.
     * The 400 mV gap (2800 set → 3200 recover) prevents oscillation.
     * See CHANGELOG v0.01 for why !has_load → IDLE was removed.
     */
    bool recovered = (V_BAT > BAT_SAFE_RECOVER_MV);

    /* P1: recovered + sun + load + not full → charge and load */
    if (recovered && HAS_SUN && HAS_LOAD && !BAT_FULL)
        return EM_CHARGE_AND_LOAD;

    /* P2: recovered + sun + no load → charge only */
    if (recovered && HAS_SUN && !HAS_LOAD)
        return EM_CHARGE_ONLY;

    /* P3: recovered + no sun + load → discharge
     * FIXED v0.02: was missing — battery recovered overnight */
    if (recovered && !HAS_SUN && HAS_LOAD)
        return EM_DISCHARGE_ONLY;

    /* P4: recovered + no sun + no load → idle */
    if (recovered && !HAS_SUN && !HAS_LOAD)
        return EM_IDLE;

    /* P5: stay — loads shed until battery recovers */
    return EM_SAFE_MODE;
}

/* =========================================================================
 * MAIN UPDATE FUNCTION
 * =========================================================================
 *
 * Called every TICK_MAIN_MS (50 ms) as pipeline step 5.
 *
 * Flow:
 *   1. Evaluate transition for current state
 *   2. If state changed:
 *      a. Run exit actions (deactivate charger if leaving a charging state)
 *      b. Update ctx->energy_mode
 *      c. Run entry actions (set hardware enables for new state)
 *   3. If state unchanged:
 *      - In IDLE: check sleep timeout
 *      - Otherwise: nothing (hardware stays as-is)
 */
/*
 * apply_entry_actions — dispatch the entry action for a given state.
 * Idempotent: GPIO writes are unconditional, so calling it on the
 * already-current state is harmless. Used both on real transitions and
 * to re-arm hardware after a fault clears.
 */
static void apply_entry_actions(system_ctx_t *ctx, energy_mode_state_t s)
{
    switch (s) {
        case EM_IDLE:            enter_idle(ctx);            break;
        case EM_CHARGE_ONLY:     enter_charge_only(ctx);     break;
        case EM_CHARGE_AND_LOAD: enter_charge_and_load(ctx); break;
        case EM_DISCHARGE_ONLY:  enter_discharge_only(ctx);  break;
        case EM_SAFE_MODE:       enter_safe_mode(ctx);       break;
        default:                 enter_idle(ctx);            break;
    }
}

/* Public wrapper: restore the current state's hardware enables without a
 * transition. Used by system_sleep() after wake (and mirrors the internal
 * fault-clear re-arm path). Idempotent — GPIO writes are unconditional. */
void energy_mode_reapply_entry(system_ctx_t *ctx)
{
    apply_entry_actions(ctx, ctx->energy_mode);
}

/*
 * maybe_arm_sleep — in-state check for IDLE and SAFE_MODE.
 *
 * Arms idle_sleep_pending (consumed by main(), which calls system_sleep())
 * once the state has been unchanged for IDLE_SLEEP_TIMEOUT_MS, gated on:
 *
 *   - No latched fault: fault recovery is throttled but still needs the
 *     pipeline to tick every FAULT_RECOVER_WAIT_MS; sleeping would freeze
 *     a recoverable fault in place indefinitely.
 *   - (IDLE only) all lamps off: in IDLE the LED boost is up and a
 *     non-zero lamp_level is a lamp the user deliberately lit — sleep
 *     kills the LED rail, so never enter it over a burning lamp. Not
 *     checked in SAFE_MODE, where the rail is already shed and the lamps
 *     are dark regardless of their stored levels.
 */
static void maybe_arm_sleep(system_ctx_t *ctx, bool check_lamps)
{
    if (ctx->idle_sleep_pending)
        return;

    if (ctx->fault.code != FAULT_NONE)
        return;

    if (check_lamps) {
        for (uint8_t i = 0; i < 4; i++) {
            if (ctx->lamp_level[i] != 0)
                return;
        }
    }

    if ((time_now() - ctx->idle_start_ms) >= IDLE_SLEEP_TIMEOUT_MS)
        ctx->idle_sleep_pending = true;
}

/*
 * safe_mode_rescue_active — is a supervised BAT_UNDERVOLT rescue permitted?
 *
 * All four must hold:
 *   - we are in SAFE_MODE (the state where the undervolt latch strands us);
 *   - BAT_UNDERVOLT is the SOLE latched fault. Requiring it alone means no
 *     charge-blocking fault (overtemp, temp-charge-block, overvolt,
 *     overcurrent-chg, precharge-timeout) can be co-latched, so we never
 *     trickle into a hot/cold/faulted cell — and the moment the rescue's own
 *     escalation raises FAULT_PRECHARGE_TIMEOUT this goes false and the rescue
 *     stops. (USB-overvolt / discharge-overcurrent don't persist here — their
 *     rails are shed in SAFE_MODE — so they only ever delay a rescue by one
 *     recovery window, never block it.)
 *   - there is usable sun (flag_has_sun already folds in the dusk power-gate);
 *   - raw V_bat is at/above the hard floor (below it, stay latched);
 *   - the protection wake probe is not busy: while it drives the battery
 *     node the V_bat reading is untrusted, and a driven ≥1500 mV reading
 *     would otherwise start the rescue mid-probe and fight for the buck.
 *
 * Consumed by safe_mode_rescue_tick() here and by the charger's run gate.
 */
bool safe_mode_rescue_active(const system_ctx_t *ctx)
{
    return (ctx->energy_mode == EM_SAFE_MODE) &&
           (ctx->fault.code == FAULT_BAT_UNDERVOLT) &&
           ctx->flag_has_sun.value &&
           (ctx->meas.bat_voltage >= BAT_RESCUE_MIN_MV) &&
           !bat_wake_probe_busy(ctx);
}

/*
 * safe_mode_rescue_tick — in-state manager for the supervised rescue.
 *
 * Runs each tick while SAFE_MODE is unchanged. When the rescue is warranted it
 * brings the charge path up (loads stay shed — enter_safe_mode already
 * disabled output/USB/LED and we never re-enable them here); when it is not, it
 * tears the charge path back down to the fully-shed SAFE_MODE configuration.
 *
 * activate_charger_region() pre-positions PWM and self-guards on CHG_INACTIVE,
 * so calling it every tick is a no-op once the charger self-activates into
 * PRECHARGE. The charger runs because charger_update()'s gate also honours
 * safe_mode_rescue_active(); power_budget clamps the intake to <=200 mA below
 * 3000 mV, so the trickle is inherently precharge-rate. If the cell never
 * climbs past 3000 mV, the charger's PRECHARGE timeout (extended to
 * BAT_RESCUE_TIMEOUT_MS while undervolt is latched) escalates to
 * FAULT_PRECHARGE_TIMEOUT — which then makes this predicate false and ends the
 * rescue at the correct terminal, user-assisted state.
 */
static void safe_mode_rescue_tick(system_ctx_t *ctx)
{
    if (safe_mode_rescue_active(ctx)) {
        activate_charger_region(ctx);          /* buck + charge switch up */
    } else if (ctx->charger.state != CHG_INACTIVE) {
        /* Window closed (sun lost, cell fell below the hard floor, or a
         * charge-blocking fault appeared): return to the shed config. */
        deactivate_charger_region(ctx);        /* buck off, charger INACTIVE */
        disable_charge_switch();               /* the one switch deactivate leaves */
    }
}

/* =========================================================================
 * BATTERY PROTECTION WAKE PROBE (SAFE_MODE in-state)
 * =========================================================================
 *
 * Recovers from the S-8240 protection-open lockout (see the BAT_WAKE_*
 * block in hw_config.h and docs/bug_battery_hotplug_800mv_lockout.md):
 * with Q416/Q417 open, V_BATM reads a ≈0.8 V bias node, FAULT_BAT_UNDERVOLT
 * latches on that invalid measurement, and neither the fault recovery
 * (needs 3200 mV) nor the supervised rescue (needs 1500 mV) can ever run —
 * while the one stimulus that would close the protection FETs again, a
 * charger connection, is exactly what SAFE_MODE keeps disabled.
 *
 * The probe applies that stimulus briefly and safely:
 *
 *   MONITOR    — candidate = the 0.8 V signature has been stable for
 *                BAT_WAKE_DETECT_TICKS while undervolt is the SOLE fault,
 *                usable sun is present, loads are shed and 3VOUT has
 *                collapsed. A missing battery produces the SAME signature,
 *                so candidacy alone never clears a fault or declares a cell.
 *   WAKE_PROBE — buck + charge switch up at the FIXED LUT target
 *                BAT_WAKE_PROBE_TARGET_MV (3.65 V, the CV limit). Never
 *                V_bat + headroom: the measured V_bat is the one value we
 *                know to be wrong, and it would command ≈2.79 V — below the
 *                cell, no release differential. Cut early once
 *                BAT_WAKE_PROBE_CUT_MA flows (stimulus delivered), on
 *                timeout, or on any abort (sun lost, panel collapse below
 *                PANEL_SAFETY_MV, any additional fault latching).
 *   SETTLE     — buck and charge switch OFF; wait out the 640 ms V_bat
 *                moving average (plus margin) so no driven samples remain.
 *   VALIDATE   — the arbiter. A driven reading proves nothing (the buck
 *                charges an EMPTY connector to the commanded voltage), so
 *                require the OFF-state V_bat to hold ≥ BAT_WAKE_VALID_MIN_MV
 *                for the whole window:
 *                  ≥ 1500 mV → VALIDATED. The measurement is real again;
 *                    the existing machinery takes over untouched (rescue
 *                    for 1500..3200, fault recovery + SAFE exit at ≥3200).
 *                  back in the signature → NO_BATTERY (or still open):
 *                    rate-limited retry, bounded attempts.
 *                  in between → WAKE_FAILED: a real but deeply damaged
 *                    cell. Terminal — keep the fault, no unattended charge.
 *   RETRY_WAIT — one attempt per BAT_WAKE_RETRY_MS, at most
 *                BAT_WAKE_MAX_ATTEMPTS per budget.
 *   TERMINAL   — attempts exhausted or WAKE_FAILED. Holds until the panel
 *                is removed or the fault set changes (both reset the
 *                budget), so an empty connector is never pulsed forever.
 *
 * While the probe is BUSY (PROBE/SETTLE/VALIDATE) the V_bat measurement is
 * untrusted-by-construction, so three consumers are gated on
 * bat_wake_probe_busy(): eval_safe_mode's recovery comparison (or a driven
 * 3.65 V reading would bounce SAFE → CHARGE_ONLY onto a phantom battery),
 * fault_mgr's BAT_UNDERVOLT recovery (same phantom would clear the latch),
 * and safe_mode_rescue_active() (a driven reading ≥1500 mV would start the
 * rescue mid-probe and fight for the buck).
 */

bool bat_wake_probe_busy(const system_ctx_t *ctx)
{
    return (ctx->bat_wake.phase == BAT_WAKE_PROBE)  ||
           (ctx->bat_wake.phase == BAT_WAKE_SETTLE) ||
           (ctx->bat_wake.phase == BAT_WAKE_VALIDATE);
}

/* Tear the probe stimulus down NOW (mirror of the SAFE_MODE shed config). */
static void bat_wake_cut_stimulus(system_ctx_t *ctx)
{
    ctx->pwm = PWM_MIN_DUTY;
    disable_input_buck();
    disable_charge_switch();
}

/* The full candidate condition — every clause must hold each tick. */
static bool bat_wake_candidate(const system_ctx_t *ctx)
{
    return (ctx->fault.code == FAULT_BAT_UNDERVOLT) &&      /* SOLE fault  */
           ctx->flag_has_sun.value &&                       /* usable panel */
           !ctx->has_load &&                                /* loads quiet  */
           (ctx->meas.out_voltage < BAT_WAKE_OUT_COLLAPSED_MV) &&
           (ctx->meas.bat_voltage >= BAT_PROT_SIG_MIN_MV) &&
           (ctx->meas.bat_voltage <= BAT_PROT_SIG_MAX_MV);
}

/* Attempt-budget / hold reset condition: the panel went away or the fault
 * set changed (undervolt cleared, or another fault joined). Either way the
 * situation the budget was counting against no longer exists. */
static bool bat_wake_reset_condition(const system_ctx_t *ctx)
{
    return !ctx->flag_has_sun.value ||
           (ctx->fault.code != FAULT_BAT_UNDERVOLT);
}

static void bat_wake_tick(system_ctx_t *ctx)
{
    bat_wake_ctx_t *bw = &ctx->bat_wake;
    uint32_t now = time_now();

    switch (bw->phase) {

    case BAT_WAKE_MONITOR:
        if (bat_wake_reset_condition(ctx)) {
            bw->attempts = 0;
            bw->detect_ticks = 0;
            break;
        }
        if (!bat_wake_candidate(ctx)) {
            bw->detect_ticks = 0;
            break;
        }
        if (++bw->detect_ticks < BAT_WAKE_DETECT_TICKS)
            break;
        bw->detect_ticks = 0;
        if (bw->attempts >= BAT_WAKE_MAX_ATTEMPTS) {
            bw->phase = BAT_WAKE_TERMINAL;      /* budget spent */
            break;
        }
        /* Start the probe. Fixed LUT target — see the header comment. */
        bw->attempts++;
        bw->last_result = BAT_WAKE_RES_NONE;
        bw->phase = BAT_WAKE_PROBE;
        bw->phase_start_ms = now;
        ctx->pwm = set_charging_voltage(BAT_WAKE_PROBE_TARGET_MV);
        enable_input_buck();
        enable_charge_switch();
        /* Loads stay shed: enter_safe_mode already disabled output/USB/LED
         * and nothing here re-enables them. Battery switch is already on. */
        break;

    case BAT_WAKE_PROBE: {
        /* Aborts first: any extra fault (its protective action may already
         * have cut the hardware — make the context state match), sun lost,
         * or the panel collapsing under the probe (weak panel). */
        if ((ctx->fault.code != FAULT_BAT_UNDERVOLT) ||
            !ctx->flag_has_sun.value ||
            (ctx->meas.panel_voltage < PANEL_SAFETY_MV)) {
            bat_wake_cut_stimulus(ctx);
            bw->last_result = BAT_WAKE_RES_ABORTED;
            bw->phase = BAT_WAKE_RETRY_WAIT;
            bw->phase_start_ms = now;
            break;
        }
        /* Stimulus delivered (bounded charge current is flowing) or the
         * probe window closed — either way stop driving and validate. */
        bool current_cut = (ctx->meas.chg_current >
                            (int16_t)BAT_WAKE_PROBE_CUT_MA);
        bool timed_out   = (now - bw->phase_start_ms) >= BAT_WAKE_PROBE_MS;
        if (current_cut || timed_out) {
            bat_wake_cut_stimulus(ctx);
            bw->phase = BAT_WAKE_SETTLE;
            bw->phase_start_ms = now;
        }
        break;
    }

    case BAT_WAKE_SETTLE:
        if ((now - bw->phase_start_ms) >= BAT_WAKE_SETTLE_MS) {
            bw->phase = BAT_WAKE_VALIDATE;
            bw->phase_start_ms = now;
            bw->validate_min_mv = 0xFFFF;
        }
        break;

    case BAT_WAKE_VALIDATE:
        if (ctx->meas.bat_voltage < bw->validate_min_mv)
            bw->validate_min_mv = ctx->meas.bat_voltage;
        if ((now - bw->phase_start_ms) < BAT_WAKE_VALIDATE_MS)
            break;
        if (bw->validate_min_mv >= BAT_WAKE_VALID_MIN_MV) {
            /* Off-state persistence: a real cell is connected and the
             * measurement path is valid again. Hand off — the supervised
             * rescue (1500..3200) or the normal fault recovery + SAFE exit
             * (≥3200) proceed from here with no special casing. */
            bw->last_result = BAT_WAKE_RES_VALIDATED;
            bw->attempts = 0;
            bw->phase = BAT_WAKE_MONITOR;
        } else if (bw->validate_min_mv <= BAT_PROT_SIG_MAX_MV) {
            /* Collapsed back into the signature: no battery, or the
             * protection circuit is still open. Retry, rate-limited. */
            bw->last_result = BAT_WAKE_RES_NO_BATTERY;
            bw->phase = BAT_WAKE_RETRY_WAIT;
            bw->phase_start_ms = now;
        } else {
            /* Persisted between the signature ceiling and the rescue
             * floor: a real cell, genuinely below the hard safety floor.
             * Terminal — retain the fault, no unattended charging. */
            bw->last_result = BAT_WAKE_RES_WAKE_FAILED;
            bw->phase = BAT_WAKE_TERMINAL;
        }
        break;

    case BAT_WAKE_RETRY_WAIT:
        if (bat_wake_reset_condition(ctx)) {
            bw->attempts = 0;
            bw->detect_ticks = 0;
            bw->phase = BAT_WAKE_MONITOR;
        } else if ((now - bw->phase_start_ms) >= BAT_WAKE_RETRY_MS) {
            bw->phase = BAT_WAKE_MONITOR;       /* re-detect; budget persists */
        }
        break;

    case BAT_WAKE_TERMINAL:
        if (bat_wake_reset_condition(ctx)) {
            bw->attempts = 0;
            bw->detect_ticks = 0;
            bw->phase = BAT_WAKE_MONITOR;
        }
        break;

    default:
        bw->phase = BAT_WAKE_MONITOR;
        break;
    }
}

void energy_mode_update(system_ctx_t *ctx)
{
    energy_mode_state_t old_state = ctx->energy_mode;
    energy_mode_state_t new_state;

    /* ── Fault-clear edge detection ──
     *
     * fault_take_action() in fault_mgr can disable hardware (input buck,
     * charge switch, output switch, USB boost, LED boost) when a fault
     * is raised.
     * When the fault later auto-recovers, fault_mgr clears the bit but
     * does NOT re-enable any GPIOs — that is energy_mode's job, and
     * energy_mode only writes GPIOs on a state transition. If no
     * transition fires (e.g. still in CHARGE_ONLY because conditions
     * never changed), the GPIOs stay disabled silently and the buck
     * appears "running" (PWM toggling) but actually delivers no current.
     *
     * Detect fault.code falling edge (non-zero → zero) and re-apply the
     * current state's entry actions to restore the correct enables. */
    bool fault_just_cleared = (ctx->fault.prev_code != FAULT_NONE) &&
                              (ctx->fault.code      == FAULT_NONE);
    ctx->fault.prev_code = ctx->fault.code;

    /* ── Evaluate transition ── */
    switch (old_state) {
        case EM_IDLE:            new_state = eval_idle(ctx);            break;
        case EM_CHARGE_ONLY:     new_state = eval_charge_only(ctx);     break;
        case EM_CHARGE_AND_LOAD: new_state = eval_charge_and_load(ctx); break;
        case EM_DISCHARGE_ONLY:  new_state = eval_discharge_only(ctx);  break;
        case EM_SAFE_MODE:       new_state = eval_safe_mode(ctx);       break;
        default:                 new_state = EM_IDLE;                   break;
    }

    /* ── FAULT_BAT_UNDERVOLT forces SAFE_MODE ──
     *
     * The eval_* guards deliberately pick a state from the flags alone, and
     * with sun present none of them selects SAFE_MODE — so an undervolt
     * latched while has_sun was true (battery hot-unplug mid-charge, or a
     * protection-open ≈0.8 V signature appearing with the panel already
     * connected) left EM parked in CHARGE_ONLY: the fault's protective
     * action had disabled all the hardware, the charger toggled PWM into a
     * dead buck until FAULT_PRECHARGE_TIMEOUT, and neither the supervised
     * rescue nor the wake probe could ever run because both live in
     * SAFE_MODE. Route every undervolt latch to SAFE_MODE instead: it is
     * the state that models the torn-down hardware, and its in-state
     * handlers (rescue, wake probe) are the only recovery paths.
     * docs/fault_recovery.md already documents this as the invariant.
     * eval_safe_mode keeps EM there until V_bat genuinely recovers. */
    if (ctx->fault.code & FAULT_BAT_UNDERVOLT)
        new_state = EM_SAFE_MODE;

    /* ── No transition → handle in-state behaviour ── */
    if (new_state == old_state) {
        if (fault_just_cleared) {
            /* Re-arm any GPIOs the fault path tore down. */
            apply_entry_actions(ctx, old_state);
        }
        if (old_state == EM_IDLE) {
            /* Idle sleep timeout: if nothing happens for IDLE_SLEEP_TIMEOUT_MS
             * (and no lamp is lit, no fault latched), signal the main loop to
             * enter STANDBY via system_sleep(). */
            maybe_arm_sleep(ctx, true);
        }
        if (old_state == EM_SAFE_MODE) {
            /* Protection wake probe first: it must see the tick's fault and
             * measurement state before the rescue evaluates (both consult
             * bat_wake_probe_busy(), so ordering keeps them coherent within
             * the tick). */
            bat_wake_tick(ctx);

            /* Supervised undervolt rescue: bring the charge path up (or take
             * it back down) per the rescue conditions, before the sleep check.
             * While the rescue runs, BAT_UNDERVOLT is latched so maybe_arm_sleep
             * keeps the pipeline awake — which is what a supervised trickle
             * wants. */
            safe_mode_rescue_tick(ctx);

            /* SAFE_MODE sleeps too — loads are shed, so the awake MCU is the
             * main drain on the depleted cell. Lamps are dark here (LED boost
             * shed), so only the fault gate applies. */
            maybe_arm_sleep(ctx, false);
        }
        return;
    }

    /* ── Exit actions ── */

    /*
     * Deactivate charger region when leaving a charging state.
     * This is a no-op if leaving a non-charging state.
     *
     * The charger deactivation resets: PWM → min duty, BUCK_DIS asserted,
     * charger → INACTIVE, bat_full → false. It deliberately does NOT reset
     * mppt_limit_ma; MPPT drops to DISABLED on its next mppt_update tick,
     * preserving the learned panel limit (see deactivate_charger_region).
     */
    bool was_charging = (old_state == EM_CHARGE_ONLY ||
                         old_state == EM_CHARGE_AND_LOAD);
    bool will_charge  = (new_state == EM_CHARGE_ONLY ||
                         new_state == EM_CHARGE_AND_LOAD);

    if (was_charging && !will_charge) {
        deactivate_charger_region(ctx);
    }

    /* ── Commit transition ── */
    ctx->energy_mode = new_state;

    /* ── Entry actions ── */
    apply_entry_actions(ctx, new_state);
}
