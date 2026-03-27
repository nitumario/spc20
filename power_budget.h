/*
 * power_budget.h — Current Allocation (Pipeline Step 3)
 * ======================================================
 *
 * Answers one question: "How much current can go to the battery?"
 *
 * The buck converter pushes current into a shared bus.
 * Loads and battery are both connected to that bus.
 * Loads take what they need — they're not negotiable.
 * Whatever is left over can go to the battery.
 *
 *   allowed_chg = (buck capacity) - (load demand),  clamped to safe limits
 *
 * This module does NOT decide whether to charge. That's energy_mode's job.
 * This module does NOT regulate PWM. That's the charger's job.
 * This module just computes the number that the charger targets.
 */

#ifndef POWER_BUDGET_H
#define POWER_BUDGET_H

#include "system_types.h"

/* Step 3: compute i_buck_max and allowed_chg from current measurements */
void power_budget_update(system_ctx_t *ctx);

#endif /* POWER_BUDGET_H */
