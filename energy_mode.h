/*
 * energy_mode.h — Energy Mode FSM
 * ===================================================
 *
 * The traffic controller. Decides WHAT the system does based on
 * debounced flags (has_sun, bat_low, has_load, bat_full) and
 * controls which hardware power paths are enabled.
 *
 * States:
 *   EM_IDLE             — nothing connected or no actionable condition
 *   EM_CHARGE_ONLY      — sun, no load. Buck on, charge battery.
 *   EM_CHARGE_AND_LOAD  — sun + load. Buck on, feeds both.
 *   EM_DISCHARGE_ONLY   — no sun, load present. Battery feeds load.
 *   EM_SAFE_MODE        — battery low. Loads shed to protect cell.
 *
 * Hardware enables controlled per state:
 *
 *   State             CHARGER_EN  BATTERY_EN  OUTPUT_EN  USB_EN  BUCK_DIS
 *   ──────────────────────────────────────────────────────────────────────
 *   IDLE              off         off         off        off     asserted
 *   CHARGE_ONLY       on          on          off        off     released
 *   CHARGE_AND_LOAD   on          on          on         on      released
 *   DISCHARGE_ONLY    off         on          on         on      asserted
 *   SAFE_MODE         off         on          off        off     asserted
 *
 * Charger/MPPT region activation:
 *   Charger is activated in CHARGE_ONLY and CHARGE_AND_LOAD.
 *   MPPT follows charger — enabled when charger is active.
 *   Both are deactivated (→ INACTIVE/DISABLED) in all other states.
 *
 */

#ifndef ENERGY_MODE_H
#define ENERGY_MODE_H

#include "system_types.h"

/* evaluate flags → transition energy mode → set hardware enables */
void energy_mode_update(system_ctx_t *ctx);

#endif /* ENERGY_MODE_H */
