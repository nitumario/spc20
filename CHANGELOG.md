# [v0.01] - 18.03.26
## format change for tables

 - added priority column: integer per source state, evaluated as strict if/else chains (lower - first checked, higher - checked later)
 - added NOTES column for implementation instructions
 - added explicit '<default>' stay-in-state row per source state to make it clear that no unhandeled condition exists

 ---

 ### BUG FIX: DISCHARGE_ONLY → SAFE_MODE (critical)
- **Old:** `bat_low AND !has_load AND !has_sun → SAFE_MODE`
- **Problem:** The most dangerous case — `bat_low AND has_load AND !has_sun` — had NO transition to SAFE_MODE. Battery drains to damage under load.
- **New:** `bat_low AND has_load → SAFE_MODE` (priority 1). Added `bat_low AND !has_load → IDLE` (priority 2) since with no load there's nothing to shed.


### BUG FIX: SAFE_MODE oscillation (medium)
- **Old:** `!has_load → IDLE` was an exit from SAFE_MODE.
- **Problem:** SAFE_MODE sheds loads (disables OUTPUT_EN/USB_EN) → has_load drops to false → exits to IDLE → load device still physically connected → re-enables outputs → current flows → bat_low → SAFE_MODE → repeat. Oscillation.
- **New:** Removed `!has_load → IDLE`. ALL SAFE_MODE exits now require `V_bat > SAFE_RECOVER_MV (3200mV)`. System stays in SAFE_MODE with loads shed until battery genuinely recovers (e.g., sun returns and charges it above 3200mV).

### FIX: CHARGE_ONLY overlapping guards
- **Old:** `!has_sun OR bat_full → IDLE` was a single disjunctive guard.
- **Problem:** Evaluated as one transition, but masks the priority between "sun lost" (urgent, stop buck) vs "battery full" (graceful, just stop pushing current). Different exit actions may be needed.
- **New:** Split into two separate rows with explicit priority. `!has_sun → IDLE` (priority 2), `bat_full → IDLE` (priority 3). `has_load → CHARGE_AND_LOAD` remains priority 1.
 
### FIX: IDLE missing path for has_load AND bat_low AND !has_sun
- **Old:** `!has_sun AND has_load AND !bat_low → DISCHARGE_ONLY`. If bat_low was true, no transition matched.
- **New:** Added `!has_sun AND has_load AND bat_low → SAFE_MODE` (priority 4).
 
### FIX: CHARGE_AND_LOAD guard priority
- **Old:** Five exit guards with no defined order. `!has_sun AND bat_low` and `bat_full AND has_load` could overlap in edge cases.
- **New:** Explicit priority ordering. Safety-critical `!has_sun AND bat_low → SAFE_MODE` is priority 1.



---
## MPPT transition table
 
### BUG FIX: HOLD → DISABLED guard (critical typo)
- **Old:** `!panel_limited AND panel_limited → DISABLED` — logical contradiction, always false. This transition could never fire.
- **New:** `!panel_limited → DISABLED`.
 
### FIX: HOLD → TRACKING missing has_sun guard
- **Old:** `hold_time expired AND panel_limited → TRACKING`
- **Problem:** Could re-enter TRACKING after sun dropped if panel_limited flag was stale.
- **New:** `hold_time expired AND panel_limited AND has_sun → TRACKING`
 
### FIX: HOLD missing !has_sun → DISABLED
- **Old:** No explicit sun-loss transition from HOLD.
- **New:** `!has_sun → DISABLED` (priority 3). Consistent with TRACKING behavior.
 
---
 
## CHARGER transition table
 
### NEW: Explicit INACTIVE state and deactivation transitions
- **Old:** "Activated/deactivated by ENERGY MODE" with no defined behavior.
- **New:** Added INACTIVE state. Every active state (PRECHARGE, CC, CV) has a `deactivated → INACTIVE` transition with mandatory exit action: `pwm = PWM_MIN_DUTY (399), assert BUCK_DIS, reset charger state`.
- This prevents the buck from free-running after charger deactivation.
 
### NEW: PRECHARGE timeout → FAULT
- **Old:** "15min timeout failure" — destination unspecified.
- **New:** Explicit `timeout → FAULT` transition. A battery that can't exit precharge in 15 minutes is likely damaged or disconnected.

