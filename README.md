# SPC_20 — Architecture

## Overview

SPC_20 is a solar-powered battery charging and load management system built on the TI MSPM0 microcontroller. It charges a LiFePO4 battery from a solar panel through a buck converter, while simultaneously powering LED lamps and USB outputs.

The battery acts as a **buffer** on a shared bus and it absorbs excess power when solar exceeds load demand, and supplies the deficit when loads exceed solar. The firmware manages this power flow through a buck converter controlled by PWM.

The firmware is structured as a **Hierarchical State Machine (HSM)** with **orthogonal regions** that run in parallel inside the RUN state.

---


## Power Flow Model

The battery bus is a shared voltage rail. The battery acts as a **buffer** — it absorbs excess power (charging) or supplies deficit power (discharging) automatically based on the voltage balance at the bus node.


Solar panel -> buck converter ----(through the battery bus)---> battery AND loads


### Kirchhoff's Current Law at the Bus Node
```
I_buck = I_bat + I_load
```
Rearranged:
```
I_bat = I_buck − I_load
```
Where:
  I_buck ≥ 0     always (buck can only source current)
  I_load ≥ 0     always (loads can only sink current)
  I_bat  SIGNED  (positive = charging, negative = discharging)



what this means:

when: I_buck > I_load, I_bat is positive, battery is CHARGING
    I_buck = I_load, I_bat is zero, battery is FLOATING
    I_buck < I_load, I_bat is negative, battery is DISCHARGING


The battery charges or discharges **automatically by physics**. The firmware does not command the battery to discharge,it happens when loads exceed buck output. The firmware's only control input is the buck PWM.


### The Buck Runs Whenever There's Sun

The buck converter runs in any mode with sun, even if the charger target is zero.

```
buck_needed = has_sun    (always, regardless of charging)
can_charge  = has_sun AND allowed_chg > 0
```

## Power budgeting 

The power budget runs every 100ms? and computes how much charging current the CC loop should target

### equation
```
i_buck_max  = MIN(BUCK_MAX_CURRENT, mppt_limit)
allowed_chg = i_buck_max − I_load
clamp:  if allowed_chg > battery_limit → allowed_chg = battery_limit
        if allowed_chg < 0             → allowed_chg = 0
```

### Legentd

- **BUCK_MAX_CURRENT** = 2000mA This is the buck's hardware limit. Fixed constant from the component rating.
- **MPPT_LIMIT**  = maximum total current the panel can deliver through the buck at the current operating point 
- **I_buck_max** = MIN of the 2 above. This is the maximum total current the buck can push into the battery bus.
- **I_load** =  Measured total load current. Loads take what they need - the buck must cover this first before any current can go to the battery
- **battery limkit** = maximum safe current that the battery can take based on its voltage( it can be in 3 zones, precharge(V_bat < 3000mv -> 200ma), CC(v_bat < 3650mV -> 2000mA), CV(v_bat >= 3650mV -> 200mA))
- **allowed_chg** = the CC regulation target. How much current CC should push into the battery. This is the value that it's left after loads and clamped to battery safety. Can be zero if loads consume all available power.

### Why is allowed_chg >= 0 (never negative)

allowed_chg is a  CC regulation target, it has nothing to do with what the battery is doing. When allowed_chg is 0, the charger stops pushing current into the battery. the battery may still discharge to cover loads but that happens automatically. This code does not command negative charging.

s
### Net battery current (for monitoring)

The system tracks actual net battery current as a signed value.

```
i_bat_net = I_charge − I_load    (signed, from measurements)

Positive = battery gaining energy
Zero     = battery stable
Negative = battery losing energy
```


This is used by the ENERGY_MGMT state machine to detect that the battery is draining even during charging mode, and to trigger safe mode, and shed loads if needed.

### flags used

`has_sun` | V_panel > 9v
`has_load` | I_load > 50mA
`bat_low` | V_bat < 2800mV
`bat_full` | V_bat >= 3650mV AND I_charge < 200mA 
`can_charge` | has_sun AND allowed_chg > 0
`panel_limited` | I_charge < allowed_chg − 100mA AND has_sun -> used by MPPT


## State Machine Hierarchy
```
System(ROOT HSM):
INIT:
  entry /  init GPIO, ADC, PWM, timers, UART, disable all outputs

when hardware ready -> RUN:

    ENERGY_MGMT(Child HSM):

      [orthogonal region 1: ENERGY MODE]:
        - IDLE(no sun, no load, sleep, everything off)
        - CHARGE_ONLY(sun, no load, buck on, charge battery)
        - CHARGE_AND_LOAD(sun, load, buck on, charge battery and feed loads, includes allowed_chg == 0 where buck feeds loads, battery floats or discharges as buffer)
        - DISCHARGE_ONLY (no sun, load, battery feeds loads alone )
        - SAFE_MODE (bat_low + load, shed loads)

      [orthogonal region 2: CHARGER]:
        - PRECHARGE (V_bat < 3V, gentle current)
        - CC (constant current, target = allowed_chg(current))
        - CV (constant voltage, target = 3650mV)

      [orthogonal region 3: MPPT]:
        - DISABLED (panel not limited, mppt_limit=BUCK_MAX)
        - TRACKING (perturbing PWM, incremental conductance)
        - HOLD (converged, wait for the next MPPT call)

    FAULT_MGR(Child HSM):

    UI_MGR(Child HSM):
```

## Orthogonal regions:

The 3 states inside ENERGY_MGMT run **in paralel**

1. **ENERGY MODE** decides what the system does (charge, discharge, idle, safe mode). It controls hardware enable switches (`CHARGER_EN BATTERY_EN, OUTPUT_EN, USB_EN`). It activates/deactivates the CHARGER region. It decides if the buck runs(`has_sun`).

2. **CHARGER** decides how to charge (PRECHARGE, CC, CV). It controls the buck PWM. it recives `allowed_chg` from the power budget as its CC target. It does now know about loads, that information is taken from `allowed_chg` in the power budget.

3. **MPPT** optimizes the solar panel operating piont. It temporarily takes control of PWM during `TRACKING`. When it finishes, PWM carries over to CC and it writes `mppt_limit` back into the power budget context



## CHARGER state machine

Three states. Activated/deactivated by ENERGY MODE. Only active when ENERGY MODE is in CHARGE_ONLY, CHARGE_AND_LOAD, or SAFE_MODE (with sun).

### States

| state     | regulation | target                                     | entry_from                                       | exit_to                                         |
|-----------|------------|--------------------------------------------|--------------------------------------------------|-------------------------------------------------|
| PRECHARGE | current    | allowed_chg(budget limits to <=200mA)      | Activation with V_bat < 3000mV                   | CC when V_bat ≥ 3000mV or 15min timeout failure |
| CC        | current    | allowed_chg(takes load into consideration) | Activation with V_bat ≥ 3000mV or from PRECHARGE | CV when V_bat ≥ 3650mV                          |
| CV        | voltage    | 3650mV (3650-3660mV)                       | CC                                               | Signal bat_full when I_charge < 200mA for 30s   |


[charger_states](docs/charger_states.csv)



### CC internal logic(per cycle)

1. **Transition check:** if V_bat >= 3650mV -> transition to CV
2. **Panel safety:** if V_panel < 10V? -> increase PWM, return 
3. **MPPT check** if MPPT is in `TRACKING` -> return(MPPT has PWM control, CC skips regulation)
4. **Current regulation:** 
```
target = allowed_chg
   if I_charge < target − 25mA:  pwm -= 1  (increase duty, push more current)
   if I_charge > target + 25mA:  pwm += 1  (decrease duty, reduce current)
   else: do nothing (within +-25mA deadband, stable)
```  

### PWM behaviour

 - PWM is not reset on state entry, it is saved from previous state
 - PWM range : 1 - 399. Period = 400. Duty cycle = (400-PWM)/400
 - Lower PWM = higher duty cycle = more current from buck.

## MPPT algorithm

Incremental conductance with adaptive step size. Runs parallel to the charger state.

### States

| state    | functionality                                                 | PWM_control                                                          | mppt_limit_output              |
|----------|---------------------------------------------------------------|----------------------------------------------------------------------|--------------------------------|
| DISABLED | nothing                                                       | CC controls pwm                                                      | BUCK_MAX(no panel constraint)  |
| TRACKING | perturbs PWM measures V/I and applies incremental conductance | MPPT controls PWM. CC skips regulation but still runs safety checks. | previous value                 |
| HOLD     | Waits MPPT_HOLD_TIME before re-tracking                       | CC controls PWM (inherits MPPT's position)                           | Computed from best power found |






[MPPT_states](docs/MPPT_states.csv)


### TRACKING Entry Actions

- V_prev = V_panel (fresh)
- I_prev = I_panel (fresh)
- step_size = MPPT_MAX_STEP_SIZE
- max_reversals = 0
- max_panel_power = 0
- Force first perturbation: pwm -= step_size
- Record tracking_start_ms


### MPPT transition table

| from     | guard                               | to       |
|----------|-------------------------------------|----------|
| DISABLED | panel_limited AND has_sun           | TRACKING |
| TRACKING | converged (step==1 AND reversals>6) | HOLD     |
| TRACKING | timeout (MPPT_RUNTIME expired)      | HOLD     |
| TRACKING | !has_sun                            | DISABLED |
| HOLD     | hold_time expired AND panel_limited | TRACKING |
| HOLD     | !panel_limited AND panel_limited    | DISABLED |

[MPPT_transition_table](docs/MPPT_transition_table.csv)


## File Structure

```
├── hw_config.h         — all hardware constants and thresholds
├── system_types.h      — all enums, structs, typedefs (the architecture in types)
├── measurements.h/c    — ADC
├── power_budget.h/c    — the allowed_chg equation and flags
├── charger.h/c         — PRECHARGE  CC  CV state machine
├── mppt.h/c            — DISABLED  TRACKING  HOLD state machine
├── energy_mode.h/c     — IDLE  CHARGE  DISCHARGE  SAFE state machine
├── fault_mgr.h/c       — fault detection and management
├── hw_hal.h/c          — GPIO/PWM hardware abstraction layer
├── main.c              — system state machine, main loop, SysTick ISR
├── SPC_20.syscfg       — TI SysConfig peripheral configuration
└── ARCHITECTURE.md     — this file
```