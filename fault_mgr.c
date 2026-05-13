/*
 * fault_mgr.c — Fault Detection, Latching, Recovery (Pipeline Step 4)
 *
 * Per tick: detect conditions, latch bits, take immediate protective actions.
 * Every FAULT_RECOVER_WAIT_MS, evaluate recovery and clear resolved faults.
 * Does NOT re-enable switches on recovery — energy_mode does that.
 * Recovery thresholds differ from trip thresholds to prevent oscillation.
 */

#include "fault_mgr.h"
#include "SPCBoardAPI.h"

/* =========================================================================
 * Internal helpers
 * ========================================================================= */

/* Update fault.active shorthand. */
static inline void fault_refresh_active(fault_ctx_t *f)
{
    f->active = (f->code != FAULT_NONE);
}

/* Immediate protective action for a newly raised fault. */
static void fault_take_action(uint16_t fault_bit)
{
    switch (fault_bit) {
        case FAULT_OVERTEMP:
            disable_input_buck();
            disable_charge_switch();
            disable_output_switch();
            disable_usb_boost();
            break;

        case FAULT_BAT_OVERVOLT:
        case FAULT_PRECHARGE_TIMEOUT:
        case FAULT_TEMP_CHARGE_BLOCK:
            disable_input_buck();
            disable_charge_switch();
            break;

        case FAULT_OVERCURRENT_DSG:
            disable_output_switch();
            disable_usb_boost();
            break;

        case FAULT_BAT_UNDERVOLT:
            disable_output_switch();
            disable_usb_boost();
            disable_input_buck();
            disable_charge_switch();
            break;

        case FAULT_USB_OVERVOLT:
            disable_usb_boost();
            break;

        default:
            break;
    }
}

/* Returns true when a latched fault's recovery condition is met. */
static bool fault_recovery_met(const system_ctx_t *ctx, uint16_t fault_bit)
{
    const measurements_t *m = &ctx->meas;

    switch (fault_bit) {
        case FAULT_OVERTEMP:
            return (m->bat_temp   < (BAT_TEMP_MAX_DISCHARGE_C - TEMP_HYSTERESIS_C)) &&
                   (m->board_temp < (BOARD_TEMP_MAX_C         - TEMP_HYSTERESIS_C));

        case FAULT_BAT_OVERVOLT:
            return (m->bat_voltage < BAT_OVERVOLT_RECOVER_MV);

        case FAULT_OVERCURRENT_DSG:
            return (m->dsg_current < BAT_CC_MAX_MA);

        case FAULT_BAT_UNDERVOLT:
            return (m->bat_voltage > BAT_UNDERVOLT_RECOVER_MV);

        case FAULT_USB_OVERVOLT:
            return (m->usb1_voltage < FAULT_USB_OVERVOLT_MV) &&
                   (m->usb2_voltage < FAULT_USB_OVERVOLT_MV);

        case FAULT_PRECHARGE_TIMEOUT:
            return (m->bat_voltage > BAT_PRECHARGE_MV);

        case FAULT_TEMP_CHARGE_BLOCK:
            return ctx->temp_charge_ok;

        default:
            return false;
    }
}

/* =========================================================================
 * Public: raise a fault (idempotent)
 * ========================================================================= */
void fault_raise(system_ctx_t *ctx, uint16_t fault_bit)
{
    /* history records every fault ever raised this boot, even after recovery */
    ctx->fault.history |= fault_bit;

    if (ctx->fault.code & fault_bit)
        return;

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

    /* ── Charge current: non-latching soft chopper ──
     * Open/close charge MOSFET per-tick to cap average current without
     * latching a fault. cc_regulate sees chopper_active and freezes its step. */
    bool charger_active = (ctx->charger.state == CHG_PRECHARGE) ||
                          (ctx->charger.state == CHG_CC) ||
                          (ctx->charger.state == CHG_CV);

    if (charger_active) {
        if (m->chg_current > (int16_t)FAULT_OVERCURRENT_CHG_MA) {
            disable_charge_switch();
            ctx->charger.chopper_active = true;
            ctx->charger.chopper_last_active_ms = time_now();
        } else if (ctx->charger.chopper_active) {
            enable_charge_switch();
            ctx->charger.chopper_active = false;
        }
    } else {
        ctx->charger.chopper_active = false;
    }

    if (m->dsg_current > FAULT_OVERCURRENT_DSG_MA) {
        fault_raise(ctx, FAULT_OVERCURRENT_DSG);
    }

    /* ── USB output ── */
    if ((m->usb1_voltage > FAULT_USB_OVERVOLT_MV) ||
        (m->usb2_voltage > FAULT_USB_OVERVOLT_MV)) {
        fault_raise(ctx, FAULT_USB_OVERVOLT);
    }

    /* ── Temperature outside charge window (hysteresis applied in flags_update) ── */
    if (!ctx->temp_charge_ok) {
        fault_raise(ctx, FAULT_TEMP_CHARGE_BLOCK);
    }

    /* FAULT_PRECHARGE_TIMEOUT is raised by charger_update(), not here. */
}

/* =========================================================================
 * Recovery pass — throttled to FAULT_RECOVER_WAIT_MS cadence.
 * ========================================================================= */
static void fault_recover(system_ctx_t *ctx)
{
    if (ctx->fault.code == FAULT_NONE)
        return;

    uint32_t now = time_now();
    if ((now - ctx->fault.last_recovery_ms) < FAULT_RECOVER_WAIT_MS)
        return;
    ctx->fault.last_recovery_ms = now;

    static const uint16_t all_bits[] = {
        FAULT_OVERTEMP,
        FAULT_BAT_OVERVOLT,
        /* FAULT_OVERCURRENT_CHG: non-latching chopper in fault_detect */
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
