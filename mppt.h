/*
 * mppt.h — Maximum Power Point Tracker (Pipeline Step 6)
 * =========================================================================
 *
 * Third orthogonal region inside ENERGY_MGMT. Two implementations,
 * selected by CHARGER_INPUT_VREG (see mppt.c header for the full story):
 *
 *   =1 (current): SETPOINT-P&O. Hill-climbs the input-vreg setpoint
 *      (ctx->mppt.vreg_setpoint_mv, consumed by charger.c cc_regulate)
 *      to maximise averaged delivered charge current. NEVER writes
 *      ctx->pwm and never updates mppt_limit_ma. Seeds the setpoint
 *      from the captured open-circuit voltage on each activation.
 *   =0 (legacy): PWM-perturbing incremental conductance. Owns ctx->pwm
 *      during TRACKING (charger skips regulation) and publishes
 *      mppt_limit_ma to the power budget from HOLD.
 *
 * Pipeline position:
 *   Runs after energy_mode (step 5) and BEFORE charger (step 7), so the
 *   charger sees this tick's setpoint (or, legacy, whether MPPT owns
 *   the PWM).
 *
 * States (same names in both modes, semantics differ):
 *   MPPT_DISABLED  — charge path disconnected (including BUCK_SETTLE).
 *                    Setpoint/limit preserved.
 *   MPPT_TRACKING  — probing. Setpoint mode: 3 s settle+measure dwells,
 *                    one MPPT_SP_STEP_MV per dwell. Legacy: paced PWM
 *                    perturbations.
 *   MPPT_HOLD      — converged (or capped). Setpoint mode: setpoint
 *                    frozen, re-probes after MPPT_SP_HOLD_TIME_MS while
 *                    panel_limited. Legacy: PWM parked at best point,
 *                    mppt_limit_ma published.
 *
 * Activation policy:
 *   MPPT only runs after the charger has closed Q49 and entered PRECHARGE,
 *   CC, or CV. BUCK_SETTLE owns unloaded rail acquisition and keeps MPPT
 *   disabled; disconnecting the charge path returns MPPT to DISABLED.
 */

#ifndef MPPT_H
#define MPPT_H

#include "system_types.h"

/* Step 6: run MPPT state machine, may write ctx->pwm and ctx->mppt. */
void mppt_update(system_ctx_t *ctx);

#endif /* MPPT_H */
