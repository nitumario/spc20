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
 *   MPPT is reset to DISABLED with mppt_limit = BUCK_MAX when charger
 *   is deactivated, so power_budget sees no panel constraint next tick.
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
    ctx->charger.bat_full_timing = false;
    ctx->charger.bat_full_signaled = false;

    /* MPPT → DISABLED, remove panel constraint */
    ctx->mppt.state = MPPT_DISABLED;
    ctx->mppt.mppt_limit_ma = BUCK_MAX_CURRENT_MA;

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

    enable_input_buck();
    enable_charge_switch();

    /* Charger will self-determine PRECHARGE vs CC on its next tick
     * based on ctx->meas.bat_voltage. We leave state as INACTIVE
     * and let charger_update() handle the activated transition. */
}

/* ── Per-state entry actions ── */

static void enter_idle(system_ctx_t *ctx)
{
    disable_charge_switch();
    disable_battery_switch();
    disable_output_switch();
    disable_usb_boost();
    disable_input_buck();

    ctx->idle_start_ms = time_now();
    ctx->idle_sleep_pending = false;
}

static void enter_charge_only(system_ctx_t *ctx)
{
    enable_battery_switch();
    disable_output_switch();
    disable_usb_boost();

    activate_charger_region(ctx);
}

static void enter_charge_and_load(system_ctx_t *ctx)
{
    enable_battery_switch();
    enable_output_switch();
    enable_usb_boost();

    activate_charger_region(ctx);
}

static void enter_discharge_only(system_ctx_t *ctx)
{
    disable_charge_switch();
    disable_input_buck();
    enable_battery_switch();
    enable_output_switch();
    enable_usb_boost();
}

static void enter_safe_mode(system_ctx_t *ctx)
{
    disable_charge_switch();
    disable_input_buck();
    enable_battery_switch();   /* battery stays connected (not draining into loads) */
    disable_output_switch();   /* shed loads */
    disable_usb_boost();       /* shed USB */
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

    /* P4: no sun + load + battery low → safe mode */
    if (!HAS_SUN && HAS_LOAD && BAT_LOW)
        return EM_SAFE_MODE;

    /* P5: default — stay idle */
    return EM_IDLE;
}

static energy_mode_state_t eval_charge_only(const system_ctx_t *ctx)
{
    /* P1: load appeared → charge and load */
    if (HAS_LOAD)
        return EM_CHARGE_AND_LOAD;

    /* P2: sun lost → idle */
    if (!HAS_SUN)
        return EM_IDLE;

    /* P3: battery full → idle */
    if (BAT_FULL)
        return EM_IDLE;

    /* P4: stay */
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
void energy_mode_update(system_ctx_t *ctx)
{
    energy_mode_state_t old_state = ctx->energy_mode;
    energy_mode_state_t new_state;

    /* ── Evaluate transition ── */
    switch (old_state) {
        case EM_IDLE:            new_state = eval_idle(ctx);            break;
        case EM_CHARGE_ONLY:     new_state = eval_charge_only(ctx);     break;
        case EM_CHARGE_AND_LOAD: new_state = eval_charge_and_load(ctx); break;
        case EM_DISCHARGE_ONLY:  new_state = eval_discharge_only(ctx);  break;
        case EM_SAFE_MODE:       new_state = eval_safe_mode(ctx);       break;
        default:                 new_state = EM_IDLE;                   break;
    }

    /* ── No transition → handle in-state behaviour ── */
    if (new_state == old_state) {
        if (old_state == EM_IDLE) {
            /* Idle sleep timeout: if nothing happens for IDLE_SLEEP_TIMEOUT_MS,
             * signal that main loop should enter low-power mode. */
            if (!ctx->idle_sleep_pending &&
                (time_now() - ctx->idle_start_ms) >= IDLE_SLEEP_TIMEOUT_MS) {
                ctx->idle_sleep_pending = true;
            }
        }
        return;
    }

    /* ── Exit actions ── */

    /*
     * Deactivate charger region when leaving a charging state.
     * This is a no-op if leaving a non-charging state.
     *
     * The charger deactivation resets: PWM → min duty, BUCK_DIS asserted,
     * charger → INACTIVE, MPPT → DISABLED, bat_full → false.
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
    switch (new_state) {
        case EM_IDLE:            enter_idle(ctx);            break;
        case EM_CHARGE_ONLY:     enter_charge_only(ctx);     break;
        case EM_CHARGE_AND_LOAD: enter_charge_and_load(ctx); break;
        case EM_DISCHARGE_ONLY:  enter_discharge_only(ctx);  break;
        case EM_SAFE_MODE:       enter_safe_mode(ctx);       break;
        default:                 enter_idle(ctx);            break;
    }
}
