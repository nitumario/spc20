/*
 * energy_mode.c — Energy Mode FSM (Pipeline Step 5)
 *
 * Implements docs/transition_table.csv. Evaluates priority-ordered guards,
 * applies exit/entry actions (hardware enables), and controls charger/MPPT.
 *
 * Guard inputs: flag_has_sun, flag_bat_low, has_load, bat_full, bat_voltage.
 * Hardware outputs: CHARGER_EN, BATTERY_EN, OUTPUT_EN, USB_EN, BUCK_DIS.
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
 * Entry actions set hardware enables for the new state (idempotent).
 * deactivate_charger_region is the exit action when leaving a charging state.
 */

/* Shut down charger + MPPT on exit from a charging state. */
static void deactivate_charger_region(system_ctx_t *ctx)
{
    /* Buck off */
    ctx->pwm = PWM_MIN_DUTY;
    disable_input_buck();

    /* Charger → INACTIVE */
    ctx->charger.state = CHG_INACTIVE;
    ctx->charger.precharge_start_ms = 0;
    ctx->charger.active_start_ms = 0;
    ctx->charger.cc_last_downstep_ms = 0;
    ctx->charger.mosfet_close_ms = 0;
    ctx->charger.bat_full_timing = false;
    ctx->charger.bat_full_signaled = false;

    /* MPPT → DISABLED */
    ctx->mppt.state = MPPT_DISABLED;
    ctx->mppt.mppt_limit_ma = BUCK_MAX_CURRENT_MA;

    /* bat_full cleared — must be re-detected on next charge cycle */
    ctx->bat_full = false;
}

/*
 * Enable buck on entry to a charging state; leave MOSFET open.
 * The charger FSM closes the MOSFET after the buck settle window.
 * No-op if charger is already active (e.g. CHARGE_ONLY → CHARGE_AND_LOAD).
 */
static void activate_charger_region(system_ctx_t *ctx)
{
    if (ctx->charger.state != CHG_INACTIVE)
        return;
    disable_charge_switch();
    enable_input_buck();
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
    enable_battery_switch();
    disable_output_switch();
    disable_usb_boost();
}

/* =========================================================================
 * STATE TRANSITION LOGIC
 * =========================================================================
 *
 * Each function evaluates guards in priority order and returns the new state.
 * SAFE_MODE guards are P1 where applicable. No-transition returns current state.
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
    /* Raw voltage (not debounced): 400 mV gap prevents oscillation. */
    bool recovered = (V_BAT > BAT_SAFE_RECOVER_MV);

    /* P1: recovered + sun + load + not full → charge and load */
    if (recovered && HAS_SUN && HAS_LOAD && !BAT_FULL)
        return EM_CHARGE_AND_LOAD;

    /* P2: recovered + sun + no load → charge only */
    if (recovered && HAS_SUN && !HAS_LOAD)
        return EM_CHARGE_ONLY;

    /* P3: recovered + no sun + load → discharge */
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
 * ========================================================================= */

/* Dispatch entry action for a state. Idempotent; also used to re-arm
 * hardware after a fault clears without a state transition. */
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

void energy_mode_update(system_ctx_t *ctx)
{
    energy_mode_state_t old_state = ctx->energy_mode;
    energy_mode_state_t new_state;

    /* ── Fault-clear edge detection ──
     * fault_mgr clears fault bits but does NOT re-enable GPIOs.
     * On falling edge (faults → none), re-apply entry actions to restore enables. */
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

    /* ── No transition → handle in-state behaviour ── */
    if (new_state == old_state) {
        if (fault_just_cleared) {
            /* Re-arm any GPIOs the fault path tore down. */
            apply_entry_actions(ctx, old_state);
        }
        if (old_state == EM_IDLE) {
            if (!ctx->idle_sleep_pending &&
                (time_now() - ctx->idle_start_ms) >= IDLE_SLEEP_TIMEOUT_MS) {
                ctx->idle_sleep_pending = true;
            }
        }
        return;
    }

    /* ── Exit actions ── */
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
