/*
 * charger.h — Charge Controller (Pipeline Step 7)
 *
 * CHARGER orthogonal region of ENERGY_MGMT. See docs/charger_states.csv.
 *
 * States:
 *   CHG_INACTIVE    — region off; non-charging mode or blocking fault.
 *   CHG_BUCK_SETTLE — MOSFET open, waiting BUCK_SETTLE_MS for buck soft-start.
 *   CHG_PRECHARGE   — V_bat < 3000 mV; ≤200 mA trickle, 15-min timeout.
 *   CHG_CC          — constant current at allowed_chg up to 3650 mV.
 *   CHG_CV          — constant voltage at 3650 mV; bat_full after ≤200 mA / 30 s.
 *
 * While MPPT_TRACKING, charger skips PWM regulation (MPPT owns PWM).
 * Sign convention: lower pwm = higher duty = more current.
 */

#ifndef CHARGER_H
#define CHARGER_H

#include "system_types.h"

/* Step 7: run the charger state machine and regulate ctx->pwm. */
void charger_update(system_ctx_t *ctx);

#endif /* CHARGER_H */
