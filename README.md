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

###

