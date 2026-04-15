/*
 * charger.h — Charge Controller (Pipeline Step 7)
 * =========================================================================
 *
 * Implements the CHARGER orthogonal region of ENERGY_MGMT.
 * See docs/charger_states.csv for the authoritative transition table.
 *
 * Pipeline position (step 7):
 *   ... → energy_mode (5) → mppt (6) → charger (7) → apply_pwm (8)
 *
 *   Runs AFTER mppt so the charger can observe ctx->mppt.state and
 *   yield PWM control while MPPT is TRACKING. Runs BEFORE apply_pwm
 *   so whatever the charger writes to ctx->pwm ends up in the timer.
 *
 * States:
 *   CHG_INACTIVE   — region not running; energy_mode is not in a
 *                    charging state, or a charge-blocking fault is
 *                    latched. Does nothing.
 *   CHG_PRECHARGE  — V_bat was < 3000 mV on activation. Regulates at
 *                    allowed_chg (power_budget already clamps this to
 *                    ≤ 200 mA). 15-minute timeout raises
 *                    FAULT_PRECHARGE_TIMEOUT.
 *   CHG_CC         — constant current; target = ctx->allowed_chg.
 *                    Transitions to CV when V_bat ≥ 3650 mV.
 *   CHG_CV         — constant voltage at 3650 mV; sets ctx->bat_full
 *                    once I_charge stays below 200 mA for 30 s.
 *
 * Contracts with the rest of the system:
 *   - energy_mode handles deactivation directly (via
 *     deactivate_charger_region() in energy_mode.c), which sets
 *     ctx->charger.state = CHG_INACTIVE. The charger does not need
 *     a separate deactivate entry point.
 *   - energy_mode handles re-activation by enabling the buck/charge
 *     hardware and leaving state = INACTIVE. The charger's first
 *     post-activation tick chooses PRECHARGE or CC based on V_bat.
 *   - While MPPT is TRACKING, the charger skips PWM regulation
 *     (MPPT owns PWM). Transition guards (V_bat thresholds, taper)
 *     and panel safety still run.
 *   - FAULT_PRECHARGE_TIMEOUT is raised via fault_raise(), letting
 *     fault_mgr apply the hardware shutdown and recovery policy.
 *
 * Regulation rules (PWM sign convention):
 *   Lower pwm value → higher buck duty → more current.
 *   CC: if I < target − deadband, pwm -= 1  (push more current)
 *       if I > target + deadband, pwm += 1  (pull back)
 *   CV: if V < 3650,              pwm -= 1
 *       if V > 3655 (3650 + 5),   pwm += 1
 *   Panel safety: V_panel < 10 V → pwm += PANEL_BACKOFF_STEP, skip
 *                 regulation this tick.
 */

#ifndef CHARGER_H
#define CHARGER_H

#include "system_types.h"

/* Step 7: run the charger state machine and regulate ctx->pwm. */
void charger_update(system_ctx_t *ctx);

#endif /* CHARGER_H */
