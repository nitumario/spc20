/*
 * measurements.c — ADC Reading + Flag Evaluation
 * ================================================
 */

#include "measurements.h"
#include "SPCBoardAPI.h"

/* =========================================================================
 * DEBOUNCE HELPER
 * =========================================================================
 *
 * Generic debounce update for any debounce_flag_t.
 *
 * How it works:
 *   Flag is NOT set:
 *     - If set_condition is true, increment count.
 *       When count reaches threshold, flag latches true.
 *     - If set_condition is false, reset count to 0.
 *       (Must see consecutive readings — one false resets the counter.)
 *
 *   Flag IS set:
 *     - Only clears when clear_condition is true.
 *       (Clear threshold differs from set threshold — that's the hysteresis.)
 *     - If clear_condition is false, flag stays set regardless.
 *
 * The caller computes the boolean conditions from measurements
 * and thresholds. This function just manages the counter logic.
 */
static void debounce_update(debounce_flag_t *f,
                            bool set_condition,
                            bool clear_condition)
{
    if (f->value) {
        /* Flag is currently set — can only clear */
        if (clear_condition) {
            f->value = false;
            f->count = 0;
        }
    } else {
        /* Flag is currently clear — count toward setting */
        if (set_condition) {
            f->count++;
            if (f->count >= f->count_threshold) {
                f->value = true;
            }
        } else {
            f->count = 0;  /* reset — need consecutive readings */
        }
    }
}

/* =========================================================================
 * STEP 1: MEASUREMENTS UPDATE
 * =========================================================================
 *
 * Reads every sensor through the HAL and fills ctx->meas.
 * No filtering, no decisions — just a snapshot of hardware state.
 *
 * The HAL's get_*() functions already apply:
 *   - Moving-average filtering (64-sample window in SPCBoardAPI.c)
 *   - Unit conversion (raw ADC counts → mV and mA)
 *
 * So the values we store here are already filtered.
 *
 * NOTE on sensor mapping:
 *   TEMP1 (NTCC_10K thermistor) → bat_temp
 *   TEMP3 (NCP18X thermistor)   → board_temp
 */
void measurements_update(system_ctx_t *ctx)
{
    measurements_t *m = &ctx->meas;

    /* ── Voltages (mV) ── */
    m->bat_voltage   = get_battery_voltage();
    m->chg_voltage   = get_charge_voltage();
    m->out_voltage   = get_output_voltage();
    m->panel_voltage = get_input_voltage();
    m->usb1_voltage  = get_usb_voltage(USB1);
    m->usb2_voltage  = get_usb_voltage(USB2);

    /* ── Currents (mA) ── */

    /*
     * Panel current: HAL returns int16_t but the panel can only source
     * current (positive). A negative reading means sensor noise or offset
     * error — clamp to zero rather than storing a huge uint16_t.
     */
    int16_t i_panel_raw = get_input_current();
    m->panel_current = (i_panel_raw > 0) ? (uint16_t)i_panel_raw : 0;

    m->chg_current = get_charge_current();   /* signed — can be negative */
    m->dsg_current = get_discharge_current();

    /* ── Derived values ── */

    /*
     * Panel power (mW):
     *   panel_voltage (mV) × panel_current (mA) = µW
     *   Divide by 1000 to get mW.
     *   Cast to int32_t before multiply to avoid uint16_t overflow:
     *   15000 mV × 2000 mA = 30,000,000 — fits int32_t (max 2.1 billion).
     */
    m->panel_power = (int32_t)m->panel_voltage * m->panel_current / 1000;

    /*
     * Net battery current (mA, signed):
     *   Positive = battery is gaining charge (charging)
     *   Negative = battery is losing charge (discharging)
     *
     *   chg_current is what flows through the charge MOSFET into the battery.
     *   dsg_current is what flows out through the discharge MOSFET to loads.
     *   The difference is the net current into the battery.
     */
    m->i_bat_net = (int32_t)m->chg_current - (int32_t)m->dsg_current;

    /* ── Temperature (°C) ── */
    m->bat_temp   = get_temperature(TEMP1);
    m->board_temp = get_temperature(TEMP3);
}

/* =========================================================================
 * STEP 2: FLAGS UPDATE
 * =========================================================================
 *
 * Translates raw measurements into the boolean flags that drive
 * all state machine transitions.   
 *
 * Three types of flag filtering:
 *
 *   1. Debounced + hysteresis (debounce_flag_t):
 *      bat_low, has_sun — need both noise rejection (debounce count)
 *      and different set/clear thresholds (hysteresis).
 *
 *   2. Hysteresis only (plain bool):
 *      has_load — load changes are deliberate (switch/plug events),
 *      not noisy like solar irradiance. The 50/30 mA gap is sufficient.
 *
 *   3. Direct comparison (plain bool):
 *      panel_limited, temp_charge_ok — derived from other values,
 *      updated fresh each tick.
 */
void flags_update(system_ctx_t *ctx)
{
    measurements_t *m = &ctx->meas;

    /* ── bat_low: debounced with hysteresis ──
     *
     * Set when V_bat drops below 2800 mV for 3 consecutive ticks (150 ms).
     * Clear when V_bat rises above 2900 mV (any single reading).
     *
     * Why debounce: a load step can momentarily dip V_bat below 2800 mV.
     * Without debounce, that transient triggers SAFE_MODE, shedding loads
     * unnecessarily. 150 ms filters brief dips.
     *
     * Why hysteresis: once bat_low is set, requiring 2900 mV to clear
     * prevents oscillation at the 2800 mV boundary.
     */
    debounce_update(&ctx->flag_bat_low,
                    m->bat_voltage < BAT_LOW_MV,         /* set condition   */
                    m->bat_voltage > BAT_LOW_CLEAR_MV);  /* clear condition */

    /* ── has_sun: debounced with hysteresis ──
     *
     * Set when V_panel exceeds 9000 mV for 3 consecutive ticks (150 ms).
     * Clear when V_panel drops below 8000 mV.
     *
     * Why debounce: clouds cause rapid V_panel swings. Without debounce,
     * partial shading toggles has_sun every few hundred ms, causing
     * ENERGY_MODE to bounce between CHARGE_* and DISCHARGE/IDLE.
     *
     * Note: set_condition checks ABOVE threshold (not below like bat_low).
     */
    debounce_update(&ctx->flag_has_sun,
                    m->panel_voltage > PANEL_MIN_MV,         /* set condition   */
                    m->panel_voltage < PANEL_MIN_CLEAR_MV);  /* clear condition */

    /* ── has_load: hysteresis only ──
     *
     * Set when I_discharge > 50 mA.
     * Clear when I_discharge < 30 mA.
     *
     * The 20 mA gap prevents toggling from sensor noise around the
     * threshold. No debounce count needed — load connect/disconnect
     * events are clean transitions, not gradual like solar.
     */
    if (ctx->has_load) {
        if (m->dsg_current < LOAD_DETECT_CLEAR_MA)
            ctx->has_load = false;
    } else {
        if (m->dsg_current > LOAD_DETECT_MA)
            ctx->has_load = true;
    }

    /* ── panel_limited: direct comparison ──
     *
     * True when the panel can't deliver what the power budget allows.
     * This is the trigger for MPPT to start tracking.
     *
     * Guard against unsigned underflow:
     *   If allowed_chg <= PANEL_LIMITED_MARGIN_MA (100), the subtraction
     *   would wrap to ~65435 on uint16_t, making the comparison meaningless.
     *   In that case, panel_limited = false (budget is so small that
     *   "panel can't deliver enough" doesn't apply).
     *
     * The chg_current (int16_t) vs uint16_t comparison works correctly
     * in C because both promote to int (32-bit on ARM) before comparison.
     */
    if (ctx->flag_has_sun.value && ctx->allowed_chg > PANEL_LIMITED_MARGIN_MA) {
        ctx->panel_limited =
            (m->chg_current < (int16_t)(ctx->allowed_chg - PANEL_LIMITED_MARGIN_MA));
    } else {
        ctx->panel_limited = false;
    }

    /* ── temp_charge_ok: hysteresis on both bounds ──
     *
     * LiFePO4 safe charge range: 0°C to 45°C.
     *
     * When currently OK (charging allowed):
     *   Block if temp leaves the 0–45°C window.
     *
     * When currently blocked:
     *   Resume only after temp recovers by TEMP_HYSTERESIS_C (10°C):
     *     - Cold: blocked at 0°C, resume at 10°C
     *     - Hot:  blocked at 45°C, resume at 35°C
     *
     * Why 10°C hysteresis: prevents rapid on/off cycling of the charger
     * when temperature hovers near the limit. The charger itself generates
     * heat, so without hysteresis: charge → heats up → blocks → cools →
     * charge → repeat.
     */
    if (ctx->temp_charge_ok) {
        ctx->temp_charge_ok =
            (m->bat_temp >= BAT_TEMP_MIN_CHARGE_C) &&
            (m->bat_temp <= BAT_TEMP_MAX_CHARGE_C);
    } else {
        ctx->temp_charge_ok =
            (m->bat_temp >= BAT_TEMP_MIN_CHARGE_C + TEMP_HYSTERESIS_C) &&
            (m->bat_temp <= BAT_TEMP_MAX_CHARGE_C - TEMP_HYSTERESIS_C);
    }

    /*
     * bat_full: NOT set here.
     * Written by charger_update() when CV taper completes.
     * Cleared by energy_mode when charger is deactivated.
     */
}
