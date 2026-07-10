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
 *   IDLE              off         on          on         on      asserted
 *   CHARGE_ONLY       on          on          on         on      released
 *   CHARGE_AND_LOAD   on          on          on         on      released
 *   DISCHARGE_ONLY    off         on          on         on      asserted
 *   SAFE_MODE         off         on          off        off     asserted
 *
 *   BATTERY_EN stays on in every runtime state (the cell buffers the bus).
 *   OUTPUT_EN + USB_EN (and the LED boost) also stay on in IDLE and the
 *   charging states so a load appearing can be sensed (has_load) and wake
 *   the FSM — only SAFE_MODE sheds them. LED boost tracks OUTPUT_EN and is
 *   off in SAFE_MODE; it is not shown as a separate column.
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

/* Re-apply the CURRENT state's entry actions (hardware enables) without a
 * transition. Used by system_sleep() on full wake to restore the rails it
 * tore down for STANDBY (LED boost, sense front-end) exactly as the active
 * state prescribes — IDLE gets its detection rails back, SAFE_MODE stays
 * shed. Also re-anchors the state's inactivity timer (enter_* actions).
 * Same mechanism as the fault-clear re-arm inside energy_mode_update(). */
void energy_mode_reapply_entry(system_ctx_t *ctx);

#endif /* ENERGY_MODE_H */
