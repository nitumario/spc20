/*
 * mppt.h — Maximum Power Point Tracker (Pipeline Step 6)
 * =========================================================================
 *
 * Incremental-conductance MPPT with adaptive step size, running as the
 * third orthogonal region inside ENERGY_MGMT.
 *
 * Pipeline position:
 *   Runs after energy_mode (step 5) and BEFORE charger (step 7), so the
 *   charger can see whether MPPT currently owns the PWM.
 *
 * States (see docs/MPPT_transition_table.csv):
 *   MPPT_DISABLED  — panel is not the bottleneck. mppt_limit_ma =
 *                    BUCK_MAX_CURRENT_MA (no constraint on power budget).
 *                    CC/CV owns the PWM.
 *   MPPT_TRACKING  — actively perturbing PWM to find MPP. MPPT owns PWM;
 *                    the charger's CC loop must skip regulation.
 *   MPPT_HOLD      — converged (or timed out). Parks PWM at the best
 *                    point found. mppt_limit_ma reflects what the panel
 *                    can deliver through the buck at that point.
 *                    After MPPT_HOLD_TIME_MS, re-enters TRACKING.
 *
 * Activation policy:
 *   MPPT only runs while energy_mode has activated the charger region
 *   (EM_CHARGE_ONLY or EM_CHARGE_AND_LOAD). When energy_mode deactivates
 *   the charger region it resets MPPT to DISABLED directly.
 *
 * Interaction with charger:
 *   The charger reads ctx->mppt.state. While MPPT_TRACKING, the charger
 *   must skip its CC regulation step — MPPT is writing ctx->pwm.
 *   Otherwise the charger writes ctx->pwm from its CC/CV loop.
 *
 * Output to power_budget:
 *   ctx->mppt.mppt_limit_ma is consumed by power_budget_update() on the
 *   following tick to cap i_buck_max.
 */

#ifndef MPPT_H
#define MPPT_H

#include "system_types.h"

/* Step 6: run MPPT state machine, may write ctx->pwm and ctx->mppt. */
void mppt_update(system_ctx_t *ctx);

#endif /* MPPT_H */
