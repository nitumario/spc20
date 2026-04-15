/*
 * fault_mgr.h — Fault Detection, Latching, and Recovery
 * =========================================================================
 *
 * FAULT_MGR is a parallel child HSM that runs every TICK_MAIN_MS (50 ms)
 * inside SYS_RUN, alongside ENERGY_MGMT. It is pipeline step 4 — it runs
 * after measurements and flags are updated, and BEFORE energy_mode so the
 * latter sees fresh fault flags.
 *
 * Responsibilities:
 *   1. Detect fault conditions by comparing ctx->meas against thresholds
 *      from hw_config.h.
 *   2. Latch each fault as a bit in ctx->fault.code.
 *   3. Take IMMEDIATE protective action for hard faults by disabling the
 *      relevant hardware switches directly (don't wait for energy_mode).
 *   4. Periodically attempt recovery: when a fault's recovery condition
 *      is met and FAULT_RECOVER_WAIT_MS has elapsed, clear its bit.
 *
 * What it does NOT do:
 *   - It does not pull the system out of SYS_RUN (there is no SYS_FAULT).
 *   - It does not re-enable switches on recovery. energy_mode re-evaluates
 *     its transitions every tick and will re-enable the appropriate
 *     switches once ctx->fault.code is clear.
 *   - It does not decide the energy mode. It just latches flags; the
 *     energy mode FSM reads those flags and chooses a safe state.
 *
 * Fault bits (defined in system_types.h):
 *   FAULT_OVERTEMP          — bat_temp or board_temp over limit
 *   FAULT_BAT_OVERVOLT      — V_bat > BAT_OVERVOLT_MV
 *   FAULT_OVERCURRENT_CHG   — I_charge > FAULT_OVERCURRENT_CHG_MA
 *   FAULT_OVERCURRENT_DSG   — I_discharge > FAULT_OVERCURRENT_DSG_MA
 *   FAULT_BAT_UNDERVOLT     — V_bat < BAT_UNDERVOLT_MV
 *   FAULT_USB_OVERVOLT      — USB1 or USB2 voltage > FAULT_USB_OVERVOLT_MV
 *   FAULT_PRECHARGE_TIMEOUT — set externally by charger on precharge timeout
 *   FAULT_TEMP_CHARGE_BLOCK — bat temp outside charge window (soft fault)
 */

#ifndef FAULT_MGR_H
#define FAULT_MGR_H

#include "system_types.h"

/* Step 4: detect/latch faults, take protective actions, attempt recovery. */
void fault_mgr_update(system_ctx_t *ctx);

/*
 * Raise a specific fault bit and take its immediate protective action.
 * Exposed so other modules can signal faults (e.g., charger on precharge
 * timeout). Idempotent — calling with an already-set bit is a no-op.
 */
void fault_raise(system_ctx_t *ctx, uint16_t fault_bit);

#endif /* FAULT_MGR_H */
