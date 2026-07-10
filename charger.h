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
 *   - energy_mode handles re-activation by enabling the buck/charge
 *     hardware and leaving state = INACTIVE. The charger's first
 *     post-activation tick chooses PRECHARGE or CC based on V_bat.
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

#endif /* CHARGER_H */
