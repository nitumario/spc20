/*
 * charger.h — Charge Controller
 * =========================================================================
 *
 * Implements the CHARGER orthogonal region of ENERGY_MGMT.
 * See docs/charger_states.csv for the authoritative transition table.
 *
 *   Runs AFTER mppt so the charger can observe ctx->mppt.state and
 *   yield PWM control while MPPT is TRACKING. Runs BEFORE apply_pwm
 *   so whatever the charger writes to ctx->pwm ends up in the timer.
 *
 * States:
 *   CHG_INACTIVE   — region not running; energy_mode is not in a
 *                    charging state, or a charge-blocking fault is
 *                    latched. Does nothing.
 *   CHG_BUCK_SETTLE— buck enabled with the charge switch open until VCHG is
 *                    confirmed above Vbat, preventing FCCM reverse pumping.
 *   CHG_PRECHARGE  — V_bat was < 3000 mV on activation. Regulates at
 *                    allowed_chg (power_budget already clamps this to
 *                    ≤ 200 mA). 15-minute timeout raises
 *                    FAULT_PRECHARGE_TIMEOUT.
 *   CHG_CC         — bulk charging. Under CHARGER_INPUT_VREG=1 (live) it
 *                    regulates panel INPUT voltage to
 *                    ctx->mppt.vreg_setpoint_mv, clamped so I_charge never
 *                    exceeds ctx->allowed_chg; legacy builds target a fixed
 *                    current = ctx->allowed_chg. Transitions to CV at
 *                    V_bat ≥ 3650 mV.
 *   CHG_CV         — constant voltage at 3650 mV; sets ctx->bat_full
 *                    once I_charge stays below 200 mA for 30 s.
 *
 * Contracts with the rest of the system:
 *   - energy_mode handles deactivation directly (via
 *     deactivate_charger_region() in energy_mode.c), which sets
 *     ctx->charger.state = CHG_INACTIVE. The charger does not need
 *     a separate deactivate entry point.
 *   - energy_mode handles re-activation by opening the charge switch,
 *     pre-positioning and enabling the buck, and entering BUCK_SETTLE.
 *     The charger closes the switch only after VCHG is ready.
 *   - Legacy only (CHARGER_INPUT_VREG=0): while MPPT is TRACKING the
 *     charger skips PWM regulation (MPPT owns PWM). Under =1 the charger
 *     owns PWM at all times and realises each MPPT setpoint probe.
 *     Transition guards (V_bat thresholds, taper) and panel safety
 *     always run regardless.
 *   - FAULT_PRECHARGE_TIMEOUT is raised via fault_raise(), letting
 *     fault_mgr apply the hardware shutdown and recovery policy.
 *
 * Regulation rules (PWM sign convention):
 *   Lower pwm value → higher buck duty → more current.
 *   CC (CHARGER_INPUT_VREG=1): steer V_panel toward vreg_setpoint_mv,
 *       clamped so I_charge never exceeds allowed_chg (see cc_regulate).
 *   CC (legacy): if I < target − deadband, pwm -= 1  (push more current)
 *                if I > target + deadband, pwm += 1  (pull back)
 *   CV: if V_bat < 3650,              pwm -= 1
 *       if V_bat > 3655 (3650 + 5),   pwm += 1
 *   Panel safety: V_panel < PANEL_SAFETY_MV (4.8 V) → pwm +=
 *                 PANEL_BACKOFF_STEP, skip regulation this tick.
 */

#ifndef CHARGER_H
#define CHARGER_H

#include "system_types.h"

/* Step 7: run the charger state machine and regulate ctx->pwm. */
void charger_update(system_ctx_t *ctx);

/* Foreground input-loss guard. Runs BEFORE charger_fast_guard: it trips on the
 * voltage collapse that PRECEDES reverse current, so the cell is isolated
 * before it can back-feed. Losing the source is not a fault — this stands the
 * charger down without latching (ctx->charger.input_lost_pending). */
void charger_input_guard(system_ctx_t *ctx);

/* Foreground fast trip for reverse current. The normal regulator uses filtered
 * data; this guard cuts the power path from the latest 10 ms ADC conversion.
 * Backstop to charger_input_guard: reverse current on a dead input stands down
 * cleanly, on a LIVE input it latches FAULT_REVERSE_PUMP. */
void charger_fast_guard(system_ctx_t *ctx);

/* Half-width of the V_panel regulation band, in mV.
 *
 * NOT a constant: derived from the plant gain the voltage loop measures off
 * its own paced steps (charger.plant_mv_per_count), because the band's real
 * criterion is "wider than one PWM count of V_panel" and the mV that
 * corresponds to varies ~20x between panels. Falls back to the static
 * PANEL_VREG_DEADBAND_MV until a gain exists, and never exceeds it. See the
 * adaptive-deadband note in hw_config.h.
 *
 * mppt.c sizes the FOCV seed, both setpoint clamps, the cliff floor and the
 * dip classifier off this, so the outer loop's notion of the band is always
 * the one the inner loop is actually enforcing. */
uint16_t charger_vreg_deadband_mv(const system_ctx_t *ctx);

#endif /* CHARGER_H */
