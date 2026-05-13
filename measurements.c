/*
 * measurements.c — ADC Reading + Flag Evaluation
 * ================================================
 */

#include "measurements.h"
#include "SPCBoardAPI.h"

/* Generic debounce with hysteresis: count consecutive set_condition ticks to
 * latch; clear immediately on clear_condition. One false reading resets count. */
static void debounce_update(debounce_flag_t *f,
                            bool set_condition,
                            bool clear_condition)
{
    if (f->value) {
        if (clear_condition) {
            f->value = false;
            f->count = 0;
        }
    } else {
        if (set_condition) {
            f->count++;
            if (f->count >= f->count_threshold) {
                f->value = true;
            }
        } else {
            f->count = 0;
        }
    }
}

/* =========================================================================
 * STEP 1: MEASUREMENTS UPDATE
 * =========================================================================
 * Read all sensors into ctx->meas. HAL applies 64-sample moving average.
 * TEMP1 → bat_temp, TEMP3 → board_temp.
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

    /* Panel sources only; clamp noise-induced negatives to zero. */
    int16_t i_panel_raw = get_input_current();
    m->panel_current = (i_panel_raw > 0) ? (uint16_t)i_panel_raw : 0;

    m->chg_current = get_charge_current();   /* signed — can be negative */
    m->dsg_current = get_discharge_current();

    /* ── Derived values ── */
    /* mV × mA / 1000 = mW; cast before multiply to avoid uint16_t overflow */
    m->panel_power = (int32_t)m->panel_voltage * m->panel_current / 1000;
    /* positive = charging, negative = discharging */
    m->i_bat_net = (int32_t)m->chg_current - (int32_t)m->dsg_current;

    /* ── Temperature (°C) ── */
    m->bat_temp   = get_temperature(TEMP1);
    m->board_temp = get_temperature(TEMP3);
}

/* =========================================================================
 * STEP 2: FLAGS UPDATE
 * =========================================================================
 * bat_low, has_sun: debounced + hysteresis.
 * has_load: hysteresis only (load events are clean, not noisy like solar).
 * panel_limited, temp_charge_ok: direct comparison, updated each tick.
 */
void flags_update(system_ctx_t *ctx)
{
    measurements_t *m = &ctx->meas;

    /* ── bat_low: 3-tick debounce (filters load-step dips); 100 mV hysteresis ── */
    debounce_update(&ctx->flag_bat_low,
                    m->bat_voltage < BAT_LOW_MV,         /* set condition   */
                    m->bat_voltage > BAT_LOW_CLEAR_MV);  /* clear condition */

    /* ── has_sun: 3-tick debounce (filters cloud shadows); 1 V hysteresis ── */
    debounce_update(&ctx->flag_has_sun,
                    m->panel_voltage > PANEL_MIN_MV,         /* set condition   */
                    m->panel_voltage < PANEL_MIN_CLEAR_MV);  /* clear condition */

    /* ── has_load: 20 mA hysteresis ── */
    if (ctx->has_load) {
        if (m->dsg_current < LOAD_DETECT_CLEAR_MA)
            ctx->has_load = false;
    } else {
        if (m->dsg_current > LOAD_DETECT_MA)
            ctx->has_load = true;
    }

    /* ── panel_limited: chg_current below allowed_chg by margin.
     * Guard: if allowed_chg <= margin, unsigned subtraction wraps. ── */
    if (ctx->flag_has_sun.value && ctx->allowed_chg > PANEL_LIMITED_MARGIN_MA) {
        ctx->panel_limited =
            (m->chg_current < (int16_t)(ctx->allowed_chg - PANEL_LIMITED_MARGIN_MA));
    } else {
        ctx->panel_limited = false;
    }

    /* ── temp_charge_ok: 0–45°C window with 10°C hysteresis.
     * Hysteresis prevents cycling: charger heat raises temp → blocks → cools → repeat. ── */
    if (ctx->temp_charge_ok) {
        ctx->temp_charge_ok =
            (m->bat_temp >= BAT_TEMP_MIN_CHARGE_C) &&
            (m->bat_temp <= BAT_TEMP_MAX_CHARGE_C);
    } else {
        ctx->temp_charge_ok =
            (m->bat_temp >= BAT_TEMP_MIN_CHARGE_C + TEMP_HYSTERESIS_C) &&
            (m->bat_temp <= BAT_TEMP_MAX_CHARGE_C - TEMP_HYSTERESIS_C);
    }

    /* bat_full: set by charger (CV taper), cleared by energy_mode on deactivation */
}
