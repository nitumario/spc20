/*
 * fault_mgr.c — Fault Detection, Latching, Recovery (Pipeline Step 4)
 * =========================================================================
 *
 * Execution model
 * ---------------
 * Runs every TICK_MAIN_MS from main(), after flags_update() and before
 * energy_mode_update(). For each fault type:
 *
 *   1. Evaluate the detection condition against the latest measurements.
 *   2. If tripped and not already latched → set the bit + take immediate
 *      protective hardware action.
 *   3. Every FAULT_RECOVER_WAIT_MS, evaluate recovery for all latched
 *      faults and clear any whose recovery condition holds.
 *
 * Protective actions are the minimum necessary to contain the fault.
 * energy_mode_update() re-runs later in the same tick and sees the
 * fault bits; it is responsible for the overall mode selection.
 * Because energy_mode calls its own enable/disable functions based on
 * its target state, a latched fault should be interpreted by it as an
 * override — but today energy_mode does not consult ctx->fault.code
 * directly. We therefore also disable hardware here so a fault can't
 * be silently papered over.
 *
 * Hysteresis — why recovery thresholds differ from trip thresholds:
 *   Example: bat overvolt trips at 3700 mV, recovers at 3400 mV. Without
 *   the 300 mV gap, V_bat would oscillate around 3700 mV as the charger
 *   cycles on/off, latching and clearing the fault repeatedly.
 */

#include "fault_mgr.h"
#include "SPCBoardAPI.h"

/* =========================================================================
 * Internal helpers
 * ========================================================================= */

/* Update the derived ctx->fault.active shorthand. */
static inline void fault_refresh_active(fault_ctx_t *f)
{
    f->active = (f->code != FAULT_NONE);
}

/* Apply immediate protective hardware action for a newly raised fault. */
static void fault_take_action(uint16_t fault_bit)
{
    switch (fault_bit) {
        case FAULT_OVERTEMP:
            /* Thermal event — shut the whole power path down. */
            disable_input_buck();
            disable_charge_switch();
            disable_output_switch();
            disable_usb_boost();
            break;

        case FAULT_BAT_OVERVOLT:
        case FAULT_OVERCURRENT_CHG:
        case FAULT_PRECHARGE_TIMEOUT:
        case FAULT_TEMP_CHARGE_BLOCK:
            /* Charge-side faults — stop pushing current into the battery. */
            disable_input_buck();
            disable_charge_switch();
            break;

        case FAULT_OVERCURRENT_DSG:
            /* Discharge-side overcurrent — drop the loads. */
            disable_output_switch();
            disable_usb_boost();
            break;

        case FAULT_BAT_UNDERVOLT:
            /* Battery critically low — shed everything, keep battery
             * switch decision to energy_mode (it will exit to SAFE_MODE
             * on bat_low anyway). */
            disable_output_switch();
            disable_usb_boost();
            disable_input_buck();
            disable_charge_switch();
            break;

        case FAULT_USB_OVERVOLT:
            /* USB output over-voltage — kill the boost. */
            disable_usb_boost();
            break;

        default:
            break;
    }
}

/* Evaluate the recovery condition for a given latched fault.
 * Returns true if the condition is met and the bit can be cleared. */
static bool fault_recovery_met(const system_ctx_t *ctx, uint16_t fault_bit)
{
    const measurements_t *m = &ctx->meas;

    switch (fault_bit) {
        case FAULT_OVERTEMP:
            /* Require both sensors to drop BELOW their limits by the
             * hysteresis margin before allowing recovery. */
            return (m->bat_temp   < (BAT_TEMP_MAX_DISCHARGE_C - TEMP_HYSTERESIS_C)) &&
                   (m->board_temp < (BOARD_TEMP_MAX_C         - TEMP_HYSTERESIS_C));

        case FAULT_BAT_OVERVOLT:
            return (m->bat_voltage < BAT_OVERVOLT_RECOVER_MV);

        case FAULT_OVERCURRENT_CHG:
            /* Auto-retry after wait period — current is only meaningful
             * when charger is running. The wait itself is the debounce. */
            return (m->chg_current < BAT_CC_MAX_MA);

        case FAULT_OVERCURRENT_DSG:
            return (m->dsg_current < BAT_CC_MAX_MA);

        case FAULT_BAT_UNDERVOLT:
            return (m->bat_voltage > BAT_UNDERVOLT_RECOVER_MV);

        case FAULT_USB_OVERVOLT:
            return (m->usb1_voltage < FAULT_USB_OVERVOLT_MV) &&
                   (m->usb2_voltage < FAULT_USB_OVERVOLT_MV);

        case FAULT_PRECHARGE_TIMEOUT:
            /* Only recovers if someone (user) forces a retry by the
             * battery visibly climbing above the precharge threshold.
             * Likely means the cell was replaced. */
            return (m->bat_voltage > BAT_PRECHARGE_MV);

        case FAULT_TEMP_CHARGE_BLOCK:
            /* Mirror temp_charge_ok — hysteresis already applied there. */
            return ctx->temp_charge_ok;

        default:
            return false;
    }
}

/* =========================================================================
 * Public: raise a fault
 * =========================================================================
 * Idempotent — re-raising a latched fault does nothing. */
void fault_raise(system_ctx_t *ctx, uint16_t fault_bit)
{
    if (ctx->fault.code & fault_bit) {
        return;  /* already latched */
    }

    ctx->fault.code |= fault_bit;
    fault_refresh_active(&ctx->fault);
    fault_take_action(fault_bit);
}

/* =========================================================================
 * Detection pass — each check reads ctx->meas and raises if tripped.
 * ========================================================================= */
static void fault_detect(system_ctx_t *ctx)
{
    const measurements_t *m = &ctx->meas;

    /* ── Thermal ── */
    if ((m->bat_temp   > BAT_TEMP_MAX_DISCHARGE_C) ||
        (m->board_temp > BOARD_TEMP_MAX_C)) {
        fault_raise(ctx, FAULT_OVERTEMP);
    }

    /* ── Battery voltage envelope ── */
    if (m->bat_voltage > BAT_OVERVOLT_MV) {
        fault_raise(ctx, FAULT_BAT_OVERVOLT);
    }
    /* Undervolt is meaningful only once we've seen a plausible battery.
     * A disconnected battery reads near 0 mV and would trigger on boot;
     * requiring a small floor (500 mV) avoids that false positive while
     * still catching a real undervolt (cell reading ~2 V). */
    if (m->bat_voltage > 500 && m->bat_voltage < BAT_UNDERVOLT_MV) {
        fault_raise(ctx, FAULT_BAT_UNDERVOLT);
    }

    /* ── Current limits ── */
    if (m->chg_current > (int16_t)FAULT_OVERCURRENT_CHG_MA) {
        fault_raise(ctx, FAULT_OVERCURRENT_CHG);
    }
    if (m->dsg_current > FAULT_OVERCURRENT_DSG_MA) {
        fault_raise(ctx, FAULT_OVERCURRENT_DSG);
    }

    /* ── USB output ── */
    if ((m->usb1_voltage > FAULT_USB_OVERVOLT_MV) ||
        (m->usb2_voltage > FAULT_USB_OVERVOLT_MV)) {
        fault_raise(ctx, FAULT_USB_OVERVOLT);
    }

    /* ── Soft: temperature outside charge window ──
     * temp_charge_ok already applies 10°C hysteresis in flags_update(),
     * so we can latch directly on its negation. */
    if (!ctx->temp_charge_ok) {
        fault_raise(ctx, FAULT_TEMP_CHARGE_BLOCK);
    }

    /* FAULT_PRECHARGE_TIMEOUT is raised by charger_update() via
     * fault_raise() — not detected here. */
}

/* =========================================================================
 * Recovery pass — throttled to FAULT_RECOVER_WAIT_MS cadence.
 * ========================================================================= */
static void fault_recover(system_ctx_t *ctx)
{
    if (ctx->fault.code == FAULT_NONE) {
        return;  /* nothing to recover */
    }

    uint32_t now = time_now();
    if ((now - ctx->fault.last_recovery_ms) < FAULT_RECOVER_WAIT_MS) {
        return;  /* wait period not elapsed */
    }
    ctx->fault.last_recovery_ms = now;

    /* Walk each possible bit. Tight loop — eight bits, no branches
     * beyond the switch in fault_recovery_met(). */
    static const uint16_t all_bits[] = {
        FAULT_OVERTEMP,
        FAULT_BAT_OVERVOLT,
        FAULT_OVERCURRENT_CHG,
        FAULT_OVERCURRENT_DSG,
        FAULT_BAT_UNDERVOLT,
        FAULT_USB_OVERVOLT,
        FAULT_PRECHARGE_TIMEOUT,
        FAULT_TEMP_CHARGE_BLOCK,
    };

    for (unsigned i = 0; i < sizeof(all_bits)/sizeof(all_bits[0]); ++i) {
        uint16_t bit = all_bits[i];
        if ((ctx->fault.code & bit) && fault_recovery_met(ctx, bit)) {
            ctx->fault.code &= (uint16_t)~bit;
        }
    }

    fault_refresh_active(&ctx->fault);
}

/* =========================================================================
 * Public entry point
 * ========================================================================= */
void fault_mgr_update(system_ctx_t *ctx)
{
    fault_detect(ctx);
    fault_recover(ctx);
}
