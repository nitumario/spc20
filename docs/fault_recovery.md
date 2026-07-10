# Fault Recovery Matrix

`fault_mgr_update()` runs once per 50 ms main tick after measurements,
flags, and power budgeting. Faults latch in `ctx->fault.code`; recovery is
attempted no faster than `FAULT_RECOVER_WAIT_MS` (10 s). Clearing a fault bit
does not re-enable hardware directly. When all live fault bits clear,
`energy_mode_update()` detects the `fault.code != 0 -> 0` falling edge and
re-applies the current energy-mode entry action to re-arm the outputs that are
valid for that state.

`ctx->fault.history` is sticky for the full boot and records every fault that
was ever raised, even after live recovery.

| Fault | Immediate containment | Recovery condition | Recovery behavior |
| --- | --- | --- | --- |
| `FAULT_OVERTEMP` | Disable buck, charge switch, output switch, USB boost, and LED boost. | Battery temperature is below `BAT_TEMP_MAX_DISCHARGE_C - TEMP_HYSTERESIS_C` and board temperature is below `BOARD_TEMP_MAX_C - TEMP_HYSTERESIS_C`. | Auto-clear after the next recovery window once both sensors cool. The current energy state re-arms only the rails it normally owns. |
| `FAULT_BAT_OVERVOLT` | Disable buck and charge switch. | `V_bat < BAT_OVERVOLT_RECOVER_MV`. | Auto-clear after the cell relaxes below the recovery threshold. Charger may restart only if energy mode still calls for charging and no other charge-blocking fault remains. |
| `FAULT_OVERCURRENT_CHG` | Disable buck and charge switch. | `I_charge < BAT_CC_MAX_MA`. | Timed auto-retry. With the charger disabled, current should fall near zero; if the short or control fault remains, the next re-arm trips the fault again and history keeps the trace. |
| `FAULT_OVERCURRENT_DSG` | Disable output switch, USB boost, and LED boost. | `I_discharge < BAT_CC_MAX_MA`. | Timed auto-retry of the load rails through the energy-mode re-arm path. A persistent overload re-latches the fault on the next tick. |
| `FAULT_BAT_UNDERVOLT` | Disable loads, USB boost, LED boost, buck, and charge switch. | `V_bat > BAT_UNDERVOLT_RECOVER_MV`. | Auto-clear only after the battery is genuinely recovered. Energy mode will usually be in `EM_SAFE_MODE`, whose own exit also requires `V_bat > BAT_SAFE_RECOVER_MV`. |
| `FAULT_USB_OVERVOLT` | Disable USB boost. | Both USB output readings are below `FAULT_USB_OVERVOLT_MV`. | Timed auto-retry of USB boost. If the boost remains bad, USB overvoltage immediately re-latches. |
| `FAULT_PRECHARGE_TIMEOUT` | Disable buck and charge switch. | `V_bat > BAT_PRECHARGE_MV`. | External-intervention recovery. The firmware should not keep trickle-charging a cell that stayed below 3.0 V for 15 minutes; the bit clears only if the battery is replaced or externally brought above precharge voltage. |
| `FAULT_TEMP_CHARGE_BLOCK` | Disable buck and charge switch; loads stay available. | `ctx->temp_charge_ok == true` after the charge-temperature hysteresis in `flags_update()`. | Soft charge-only recovery. Charging resumes when the cell returns to the allowed charge-temperature window; discharge/load operation is not blocked by this fault. |

## Non-recoverable By Fault Manager

CPU HardFaults, default interrupt-vector traps, corrupt control flow, and
hardware failures that keep measurements outside their recovery thresholds are
not recoverable by `fault_mgr_update()`. The existing HardFault handler logs a
post-mortem over UART; returning to normal operation after that requires reset
or a firmware-level watchdog policy outside this fault bitmask.
