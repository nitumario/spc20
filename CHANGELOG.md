# [v0.35] - 10.09.26
## Revert v0.34's backoff clamp: commanding below V_bat IS the escape

Changed files: `charger.c`, `system_types.h`, `main.c`, `CHANGELOG.md`

### What v0.34 got wrong
v0.34 clamped `charger_input_guard`'s backoff at the zero-draw point on the
reasoning that backing off further only makes the buck sink. The bench said
otherwise, immediately:

```
v0.33:  INTRACE @ 46007 RECOVERED  pwm:95->115     (20 counts, panel came back)
v0.34:  INTRACE @ 18699 PENDING    pwm:97->106     (9 counts, clamped)
        INTRACE @ 18811 STANDDOWN  pwm:106->399
```

Every collapse v0.33 rescued became a stand-down. The clamp removed the only
escape that has ever worked, because of a physical fact the v0.32 notes stated
and v0.34 failed to apply: **once the input has collapsed the buck is in
dropout, and in dropout the FB target has no authority over duty.** The high
side is on because the output cannot reach the target, and the only command
that changes that is one the output is already *above* — i.e. below V_bat.

So on this hardware "relieve the panel" and "briefly sink" are the same
command. That is a property of the FCCM part, not a tuning error.

### The fix
- The backoff is unclamped again (v0.32/v0.33 behaviour).
- `CHG_REVERSE_BLANK_MS` stays, but inside the window `charger_fast_guard`
  now simply **yields**: no latch (v0.32's bug) and no corrective lift
  (v0.34's bug — lifting the target back cancels the rescue). The input guard
  owns the event and resolves it within `CHG_INPUT_LOST_SAMPLES` conversions
  (~30 ms) either way.
- `charger.zero_draw_pwm` and `charger_refresh_zero_draw()` removed.
  `lookup_charging_pwm()` stays — it is a correct accessor regardless.

**Bounded exposure:** reverse current now persists for up to 3 conversions
(~30 ms) on a collapse instead of being pre-empted. The design already
tolerated one or two 10 ms samples on every unplug; this is the same order.
**Bench check:** scope V_panel during a collapse and confirm it does not climb
toward the TPS564247's ~17 V input ceiling during those 30 ms.

### Status
**Builds clean on both `CHARGER_INPUT_VREG` branches; awaiting bench.**
This restores v0.33's working rescue *without* v0.33's false `0x0100`. It is
a correction, not progress — see the analysis note below on why the inner
loop cannot hold an MPP at 400 ms pacing.

---

# [v0.34] - 10.09.26
## The guard was rescuing the input and faulting the charger for it

Changed files: `charger.c`, `SPCBoardAPI.c`, `SPCBoardAPI.h`, `main.c`,
`hw_config.h`, `system_types.h`, `CHANGELOG.md`

### The problem
First v0.33 bench run, `serial_20260910_174314.log` — and the teardowns are
still there. But the log finally says why, in one line:

```
CHG: CC -> OFF @ 46007 ms
INTRACE @ 46007 ms RECOVERED pwm:95->115 dips:1 Vpanel_raw: … 11912 11901 6483 3494
```

**`RECOVERED` and `CC -> OFF` at the same millisecond.** All four teardowns of
the session look like that: four collapses, four rescues, four teardowns. The
v0.32 guard did its job every time and the charger went down anyway — with
`fault:0100` (`FAULT_REVERSE_PUMP`) latched, which is why the restarts took
`FAULT_RECOVER_WAIT_MS` (10 s) instead of `CHG_INPUT_REARM_MS` (2 s). The
`flt_hist` is `0100` from 46 s onward. This is precisely the regression v0.32
listed as the one to report.

The two guards were fighting:

1. Panel goes over its knee, V_panel pinned at V_bat.
2. `charger_input_guard` backs off — pwm 95 → 115, **two full
   `CHG_INPUT_RECOVER_STEP`s = ~57 mV of commanded rail**, blind to where
   V_bat is. The pre-collapse rail was V_bat + ~20 mV, so the new target sits
   ~37 mV *under the cell*.
3. The panel recovers in milliseconds. Now the buck is a working converter
   regulating to a target below the battery, so the sync FETs pump the cell
   backwards — the exact physics `CHG_ACTIVATION_HEADROOM_MV` exists to
   prevent at activation.
4. `charger_fast_guard` samples reverse current with the input **live** —
   its definition of the pathology — and latches.

The backoff was manufacturing the fault the other guard latched on.

A second, quieter defect showed up in the same log. `pwmf` ratcheted 101 →
107 → 113 as designed, and the warm resume entered at 113 as designed — then
`CHG_BUCK_SETTLE`'s acquisition ramp walked straight through the fence to
107, and CC held 107 for 156 s until the panel fell over there again.
`pwm_draw_more` cannot pull it back: below the fence it is a no-op by design.
The ramp steps every 50 ms while the rail is merely still *slewing*, so it
overshoots by 6-16 counts on every activation — harmless from 399, fatal to a
warm resume.

### The fix
**Backoffs stop at the zero-draw point** (`charger.c`,
`charger_refresh_zero_draw`). `charger.zero_draw_pwm` is the LUT count for
V_bat + `CHG_ACTIVATION_HEADROOM_MV` — the same number activation
pre-positions to, the point where the buck delivers ~nothing and still cannot
sink. `charger_input_guard` clamps its backoff there. There was never any
relief past it, only reversal. Refreshed every charger tick because it tracks
V_bat: a CC session walking the cell 3.29 → 3.65 V moves it ~125 counts, so a
value latched at activation is stale within a minute. `lookup_charging_pwm()`
is a new non-writing LUT accessor (`SPCBoardAPI.c`) — the foreground guard
needs the number, not a compare-register write.

**The reverse-pump latch is blanked during a rescue** (`charger_fast_guard`).
For `CHG_REVERSE_BLANK_MS` (200 ms) after the guard's last backoff, reverse
current with a live input is the *rescue's own tail*, not the pathology — the
panel is back at full voltage while the buck still carries the backed-off
target. Inside the window: lift the target to the zero-draw point and give it
one conversion; still reversing on the next one, stand down cleanly, the same
call v0.26 makes for a dead input. Never latch. Outside the window nothing
changes — a sustained reverse-pump event is still a fault. Exposure stays
bounded at one 10 ms sample, as before.

**The learned ceiling is re-asserted at the SETL → CC/PRECHARGE handoff.**
Not inside the ramp — fencing the ramp is how a current limit becomes the
"stranded in SETL forever" lockup the ramp was added to fix. At the handoff
Q49 has just closed and the correction only ever reduces current.

### Status
**Builds clean on both `CHARGER_INPUT_VREG` branches; awaiting bench.**
From a fresh `CHG_ONLY` capture:
1. **No `0x0100`, and `flt_hist:0000`.** This is the headline check.
2. `INTRACE … RECOVERED` with `CHG` staying `CC` across it — the region no
   longer goes down on a rescued collapse, so no 10 s hole.
3. `pwm` after `SETL -> CC` equals `pwmf` when a fence is live, instead of
   sitting several counts under it.
4. Unplug regression unchanged: pull the panel under a ~1 A charge — expect
   `CHG: CC -> OFF` within ~50 ms, an `INTRACE … STANDDOWN`, and no `0x0100`.
   A *sustained* reverse-pump condition must still latch; the blank window is
   200 ms, it is not a disable.

Watch the harvest trade-off: the fence gave up ~540 mA (600 → 250 mA) in this
session, and once rescues stop tearing the region down the fence becomes the
only thing setting the operating point. If `pwmf` proves over-conservative,
the relax rate (`CHG_CLIFF_PWM_RELAX_MS`) is the knob.

---

# [v0.33] - 10.09.26
## The setpoint ran out of authority: learn the cliff in the PWM domain, and resume there

Changed files: `charger.c`, `mppt.c`, `energy_mode.c`, `main.c`, `hw_config.h`,
`system_types.h`, `docs/constants_and_faults.md`, `CHANGELOG.md`

### The problem
Bench capture `serial_20260910_165847.log` — v0.30, 36 min of late-afternoon
sun, `EM` never left `CHG_ONLY`, no fault ever latched, and the charger tore
down **13 times**. Every one is the same `INTRACE`: flat V_panel, one sample
of descent, pinned at V_bat, Voc again 400 ms later. Eleven of the thirteen fired
with MPPT in `HLD`, i.e. with nothing perturbing anything — **the tracker was
not the trigger.** v0.32 turns most of that class into a rescue, but it does
not explain why the same operating point keeps falling over.

The `pwm` at collapse tells the story: 93, 91, 94, 95, 97, 101, then **106
seven times running**. Isc is falling with the sun under a draw that never
moves, because nothing in the loop moves it:

- `V_panel` 12736, setpoint 11576 → inside the ±`PANEL_VREG_DEADBAND_MV`
  band, so `cc_regulate` holds PWM. The plant sits wherever the frozen count
  put it.
- The setpoint cannot rise to where it would command a backoff. That needs
  `sp > V_panel + 1200` = 13936 mV, above Voc (13109), and
  `MPPT_SP_VOC_GUARD_MV` correctly fences it 1500 mV *below* Voc. `spf` had
  been pinned at that ceiling since 761 s — `sp_learn_cliff` had nothing left
  to give.

That is the structural hole: at a **near-Voc operating point** — every panel
at low irradiance, so every late afternoon — a collapse is a *current* event
with no precursor in V_panel, and the setpoint domain has no reachable value
that responds to it. The cliff memory was in the wrong units.

### The fix
**Learn the cliff in PWM** (`mppt.c`, `learn_cliff_pwm`). Every collapse the
guard sees — rescued (`input_dip_events`) or torn down — raises
`mppt.cliff_pwm_min` to `CHG_CLIFF_PWM_MARGIN` (6 counts, ~270 mA) above
`input_trace_pwm_from`, the count that actually fell over. It runs *before*
`sp_learn_cliff`'s early returns: those guard the setpoint estimate, and
neither says anything about the PWM. Same lifetime as `sp_session_floor_mv`
— survives bounces and HOLD, released only by `enter_tracking_fresh`, so a
new day (or any gap that drops `has_sun`) starts unfenced.

**The regulator is fenced by it** (`charger.c`, `pwm_draw_more`). Only the
branches that draw *more* go through it: the panel-voltage loop, the in-band
re-acquire, and CV's pull-up. Backoffs, the reverse-current escape and the
`CHG_BUCK_SETTLE` ramp keep using `pwm_step` — the ceiling bounds current,
and blocking an escape or an interlock ramp with it would turn a current
limit into a lockup.

**One count back per `CHG_CLIFF_PWM_RELAX_MS`** (180 s) of collapse-free
charging, so a haze that lifts without ever dropping `has_sun` is not capped
at the level it imposed. The relax has to be slower than the drift it fences
or it simply cancels the ratchet: replaying the session's 13 collapse points
through the ratchet, one count per 60 s fences off 9 of them, one per 180 s
fences 12 (the first is unlearnable by construction). (A replay of fixed fall points is arithmetic, not a prediction —
a fenced loop parks somewhere different and changes the sequence.)

**Warm resume** (`energy_mode.c`) — the reported symptom. Re-arming after a
stand-down re-derived the entry point from the V_bat LUT every time: `pwm`
399 → 121 → walk down to 106, ~2.5 s of the ~4.5 s outage spent re-finding
the number it held two seconds earlier, on every restart. The learned
ceiling *is* that number, already backed off, so activation now enters there
instead. Gated on `!seed_invalidated` (the same "same panel, same day" test
the setpoint resume uses) and bounded by `CHG_RESUME_MAX_ADVANCE` (10 counts)
so the pre-position still bounds the inrush when `CHG_BUCK_SETTLE` closes
Q49. A cold boot, a lost `has_sun` or a gap past `MPPT_RESEED_GAP_MS` take
the LUT path exactly as before.

**Telemetry**: the log line carries `pwmf:` (the live ceiling, 0 = none)
right after `pwm:`.

### What this does not change
The ±1200 mV deadband stays. It is why the plant is free to sit 1.16 V above
the setpoint in the first place, and narrowing it is a separate change with
its own whipsaw history (v0.21) — the ceiling bounds the damage of the wide
band without re-opening that question.

### Status
**Builds clean on both `CHARGER_INPUT_VREG` branches; awaiting bench.**
Note the capture that motivated this is **v0.30** — v0.31/v0.32 have not been
on the hardware yet, so the next flash carries three versions of change.
From a fresh `CHG_ONLY` capture in fading sun:
1. `pwmf` rises a few counts on each collapse and holds; `pwm` never goes
   below it.
2. Repeat teardowns at an identical `pwm` (the 106 × 7 signature) are gone.
   With v0.32 most collapses should be `INTRACE … RECOVERED` with `CHG`
   staying `CC`.
3. After any stand-down, `CHG: OFF -> SETL` enters at ≈`pwmf` rather than at
   the LUT count, and `Ichg` is back within ~2 s instead of ~4.5 s.
4. `pwmf` back to 0 after an overnight idle or any `has_sun` drop.

---

# [v0.32] - 10.09.26
## An input collapse is not input loss: back the draw off first, stand down only if the source doesn't come back

Changed files: `charger.c`, `mppt.c`, `main.c`, `hw_config.h`,
`system_types.h`, `docs/constants_and_faults.md`, `CHANGELOG.md`

### The problem
The v0.30 bench result (above) closes the loop on what `charger_input_guard`
has actually been reacting to. Eighteen `INTRACE` dumps across two sessions,
all the same shape: steady 11–12 V, one sample of descent, then **pinned at
V_bat + ~190 mV** — and Voc again 400 ms after the stand-down. That is a live
panel pushed over its knee, sitting on its current-source side pushing Isc
through the buck (in dropout) into the cell. It is *forward* current, not the
reverse-pump condition the guard exists to pre-empt, and it recovers the
instant the draw drops below Isc — the input cap recharges in milliseconds.

The guard treated every one of them as a removed source: Q49 open, buck off,
`CHG_INPUT_REARM_MS` (2 s), `CHG_BUCK_SETTLE`, then a 25-count inner-loop
walk back from the activation pre-position at one count per 400 ms. About
**15 s of full sun per event**, for a fault that a 10-count backoff would
have cleared in one 10 ms conversion. v0.28–v0.30 reduced how often the
tracker *causes* the collapse (probe class: gone) but could do nothing about
the parked class, because nothing in the pipeline can see a 10 ms event
through a 640 ms average.

### The fix
**`charger_input_guard` backs off first** (`charger.c`). On each counted
sub-margin conversion it raises `pwm` by `CHG_INPUT_RECOVER_STEP` (10 counts
≈ 450 mA less) and writes the timer directly — the one foreground writer
besides `apply_pwm`, same clamp, reduction only. If the input is back over
the margin on the next conversion: **RECOVERED** — charging never stopped,
`input_dip_events` ticks, `cc_regulate` walks the 10–20 counts back at its
normal pace (4–8 s at reduced current, not 15 s at zero). If it is still
under after `CHG_INPUT_LOST_SAMPLES` (3) conversions, that is a removed
source and `charger_input_stand_down` runs exactly as before.

**MPPT learns from a rescue** (`mppt.c`). A recovered collapse is a cliff
sighting — the same information a teardown carried — so `sp_learn_cliff`
now also fires when `input_dip_events` moves (`seen_dip_events`), before the
tick's `V_panel` is recorded so it uses the pre-dip reading. In HOLD the
lifted setpoint takes effect the same tick; in TRACKING the running dwell is
restarted with a fresh baseline. The parked point ratchets up a few hundred
mV per rescue until it stops falling over — the margin the knee needs,
learned instead of guessed.

**`INTRACE` says what happened**: `INTRACE @ ms RECOVERED|STANDDOWN|PENDING
pwm:95->105 dips:N Vpanel_raw: …`. `charger_fast_guard` stand-downs now get
a trace too.

### What changed about safety
The guard no longer pre-empts a genuine removal at the voltage crossover.
The source falls through V_bat during the window and the reverse current
that follows is caught by `charger_fast_guard` on its first ≥100 mA sample —
which since v0.26 classifies a dead input as the same clean stand-down, no
fault. That is the "narrowly lost the race" path the design already
tolerated on fast unplugs; it is now the only path, so the FETs see one or
two 10 ms samples of reverse current on every unplug instead of some. The
backoff does not change that magnitude (in dropout the high side is on
regardless of the FB target). **Regression check:** unplug the panel under
a 1 A charge several times — expect `CHG: CC -> OFF` within ~50 ms, an
`INTRACE … STANDDOWN` line, and **no `0x0100`**. If `FAULT_REVERSE_PUMP`
appears on unplugs, that is the regression to report.

### Status
**Builds clean on both `CHARGER_INPUT_VREG` branches; awaiting bench.**
From a fresh `CHG_ONLY` capture:
1. `INTRACE … RECOVERED` lines with `CHG` staying `CC` across them and
   `dips:` counting up; `CC -> OFF` only for the temperature fault or a
   real unplug.
2. `spf` steps up on each rescue; after a few, rescues in HOLD stop (the
   parked point has margin).
3. `Ichg` dips to roughly half for ~4–8 s per rescue, never to zero.
4. No `0x0100`. Unplug regression check above.

---

# [v0.31] - 10.09.26
## Thermistors were biased off VREF, not VDD: every temperature read 11-16 degC hot

Changed files: `SPCBoardAPI.c`, `hw_config.h`, `measurements.c`,
`fault_mgr.c`, `system_types.h`, `main.c`, `thermal.c` (new), `thermal.h`
(new), `Debug/subdir_vars.mk`, `Debug/makefile`, `CHANGELOG.md`

### The problem
Both NTC dividers are biased from the **2.5V_VREF** rail (schematic sheet
`OUTPUTS_CH`: R51/R52 10K 0603 pull-ups to `2.5V_VREF`, R55 NTC 10K to GND,
R54/R53 200R to the test points). `get_temperature()` instead passed the
*measured VDD* — `get_vdd()`, ~3300 mV — as the divider supply:

```c
float gAdcResultVolts = (adcResultVDD * ADC.VREF*3) / (ADC.max_adc0_value);
... _convertToTemp(node_mv, gAdcResultVolts, NCP18X);
```

`Rt = 10k * V / (Vsupply - V)`, so an over-stated supply under-states Rt, and
an under-stated resistance on an NTC reads hot. The error is +11 to +16 degC
across the working range:

| ADC count | reported (VDD 3300) | actual (VREF 2500) | error |
|---|---|---|---|
| 1000 | 69 degC | 57 degC | +12 |
| 1400 | 55 degC | 43 degC | +12 |
| 1800 | 44 degC | 31 degC | +13 |
| 2200 | 35 degC | 21 degC | +14 |

Re-reading `serial_20260910_155320.log` through the corrected math:

| logged | actual | |
|---|---|---|
| board 47 | 34 degC | session start |
| board 61 | 49 degC | where OVERTEMP latched |
| board 70 | 58 degC | session peak |
| pack 46 | 36 degC | where TEMP_CHARGE_BLOCK latched |

So **both thermal faults in that session were false**. The board peaked at
~58 degC against a 60 degC limit and the pack at ~36 degC against 45 degC.
Neither should have tripped.

Because the bias rail *is* the ADC reference rail, the corrected conversion
is ratiometric: a real VREF error now cancels instead of being injected as a
temperature offset. That cancellation is why the divider was designed off
VREF in the first place; feeding it VDD threw it away.

### Also fixed in the same path

**Unfiltered samples.** `get_temperature()` read `ADC.AdcNResult[]` — a
single raw conversion — while every other channel on the board reads
`avg_readings[]` (64-sample moving average). One noisy sample was enough to
latch OVERTEMP, which sheds lamps, USB, output and charging together. Now
reads `avg_readings[3]` (TEMP3) and `avg_readings[14]` (TEMP1); those entries
were already being maintained and simply were not used.

**No plausibility check.** `_convertToTemp` clamps to the ends of the lookup
table, so a *shorted* NTC reported the hot end of the table (instant,
unrecoverable OVERTEMP) and an *open* one the cold end (permanent
TEMP_CHARGE_BLOCK) — neither distinguishable from a real reading. Readings
outside `[THERMISTOR_ADC_MIN, THERMISTOR_ADC_MAX]` now return
`TEMP_INVALID_C`; `measurements_update` holds the last good value and, after
`TEMP_SENSOR_FAIL_TICKS` (3 s), clears `ctx->temp_sensor_ok`.

The response to a dead sensor is deliberately asymmetric: it blocks
**charging** (a cell of unknown temperature must not be charged) but does not
raise OVERTEMP, so a failed 10K resistor cannot black out the user's lamp.
`fault_detect`'s thermal check is now gated on `temp_sensor_ok`.

**Charge-block hysteresis was symmetric.** `TEMP_HYSTERESIS_C` 10 degC on
both bounds meant one 46 degC sample locked charging out until the pack fell
to 35 degC — in an enclosure on a sunny day, the rest of the afternoon. The
log shows exactly that: TEMP_CHARGE_BLOCK latched at t=2658 s and never
cleared. Hot side is now `TEMP_HYSTERESIS_HOT_C` = 5 degC (45 -> resume 40);
the cold side keeps 10 degC.

### New: thermal foldback (`thermal.c`, pipeline step 4b)
FAULT_OVERTEMP is a cliff — it sheds every rail at once and needs a 10 degC
recovery. With the calibration fixed the board reaches ~58 degC under a
sustained 2.7 A lamp load and *stays* there, which is 2 degC of permanent
margin. Foldback is the graceful stage underneath: above
`THERMAL_FOLDBACK_START_C` (55) it walks `ctx->thermal.derate_pct` down 5 %
per 4 s to a floor of 25 %, and recovers below `THERMAL_FOLDBACK_RESUME_C`
(50). The 5 degC deadband stops it hunting against a plant whose time
constant is minutes.

`derate_pct` multiplies the *requested LED current* in `lamp_drive_ma()`; it
never touches `ctx->lamp_level[]`. So the user's brightness choice survives
the event, the buttons keep working while derated, and
`led_boost_follow_lamps()` still sees a lit lamp as lit. The result is
clamped to the lowest non-zero rung — foldback dims, it never switches a lamp
off behind the user.

Lamps are the only load the firmware can modulate: USB cannot be derated (the
AP2151 switches cannot be pulse-gated) and the charger has its own limits. If
the heat is coming from USB, foldback runs to its floor with no effect and
the hard fault remains the backstop.

Two stand-downs: a live OVERTEMP resets the derate to 100 % (the rails are
already shed, nothing left to trade), and so does `!temp_sensor_ok` (never
dim on a reading the measurement layer has disowned).

### Telemetry
Two new fields on the log line: `tsens:` (both NTCs plausible) and `derate:`
(foldback multiplier). Foldback steps also print
`THERM: derate 85% @ 58C @ <ms>`, printed only on a change.

### NOT changed, and why
`BAT_TEMP_MAX_CHARGE_C` (45), `BAT_TEMP_MIN_CHARGE_C` (0),
`BAT_TEMP_MAX_DISCHARGE_C` (60) and `BOARD_TEMP_MAX_C` (60) are untouched.
The first three are LiFePO4 datasheet ratings, not tuning knobs, and with the
sensor corrected the board limit means what it says. The readings were wrong,
not the limits.

### Open, needs bench
`serial_20260910_155320.log` shows OVERTEMP latching at t=1680 s and the load
**continuing to flow** for the next 27 minutes: `Idsg` held 2708-2766 mA,
`Vout` stayed at Vbat-184 mV (a conducting FET at 2.7 A) and `Vusb1` stayed
pinned at 5208 mV. `fault_take_action(FAULT_OVERTEMP)` calls
`disable_output_switch()` / `disable_usb_boost()` / `disable_led_boost()`, EM
stayed CHG+LOAD the whole time (so no entry action re-armed anything),
`fault_just_cleared` requires a fully clean `fault.code`, and the only other
`enable_output_switch()` call sites are the energy-mode entry actions and
boot. The firmware path holds up under reading; the rails did not come down.
**Scope EN_OUTPUT / EN_USB / EN_LED at the trip.** Until that is understood,
the hard thermal cutoff cannot be relied on and foldback is the only
functioning thermal response.

# [v0.30] - 10.09.26
## MPPT learns the cliff from a teardown and keeps it; Voc-relative setpoint floor

Changed files: `mppt.c`, `main.c`, `energy_mode.c`, `hw_config.h`,
`system_types.h`, `CHANGELOG.md`

### The problem
The v0.29 bench result (above) shows the remaining teardowns are MPPT's own
doing: it probes the setpoint downward, the inner loop obediently walks
`pwm` down one count per 400 ms toward a band top that is below the knee,
and the panel falls over mid-dwell. At 5 Hz:

```
355.1  Vpanel 11791  Ppanel 3537  Ichg 1023  pwm 74  sp 9054  TRK CC
356.5  Vpanel 11472  Ppanel 3854  Ichg 1082  pwm 70  sp 9054  TRK CC
356.9  Vpanel 11263  Ppanel 4009  Ichg 1079  pwm 69  sp 8054  TRK CC
357.1  Vpanel 10373  Ppanel 3340  Ichg    0  pwm 399 OFF OFF     ← cliff
```

Power was still *rising* toward the 4136 mW peak when it went over, so
P&O had every reason to keep going. Three things let this repeat forever:

1. **The tracker never hears about the collapse.** Both cliff detectors —
   the collapse branch (`V_panel < PANEL_SAFETY_MV`) and the dip classifier
   (`V_panel < sp − deadband − 300`) — read the 640 ms average. The
   collapse reaches V_bat in 10–20 ms and `charger_input_guard` tears the
   region down ~40 ms in, so the average never gets close: minimum averaged
   `Vpanel` in the session was 10 373, and the collapse branch fired **zero
   times** in 14 minutes. The session floor that exists precisely to stop
   re-probing a failed level was never raised.
2. **What little it learned was erased on every entry.** `tracking_common()`
   reset `sp_session_floor_mv` to `MPPT_SP_MIN_MV` on every TRACKING entry —
   and a teardown ends the session, so the bounce re-entry (v0.28 Fix 1's
   whole point was to make that a resume) came back with a clean slate and
   probed straight back down. Bursts of 3–5 teardowns 20 s apart.
3. **The absolute floor is a safety-ordering bound, not a plausibility one.**
   `MPPT_SP_MIN_MV` = 6500 mV keeps the band's low edge above
   `PANEL_SAFETY_MV`; it says nothing about where an MPP could be. The
   tracker reached it on a panel whose MPP is 11 850 mV.

### The fix
**`sp_learn_cliff()` (`mppt.c`).** When the region goes down and
`rearm_block_ms` is in the future — energy_mode stamps that at step 5 of the
same tick, and only for an input-loss stand-down — the panel just collapsed
under the tracker. Learn the level from the *plant*, not the setpoint: with a
±1200 mV band the setpoint says little about where the panel actually sat,
but the last clean averaged `V_panel` does. Set the floor so the band top
lands `MPPT_SP_CLIFF_MARGIN_MV` (250) above that voltage, lift `sp` to the
floor, point the search up. "Clean" = the tick-to-tick fall was under
`MPPT_SP_VPANEL_DROP_MV` (300); a collapsed 10 ms sample drags the 64-deep
average down 125–190 mV, honest regulation moves it ~50 mV per tick, so
this rejects readings that already contain the fall. The floor is capped at
the Voc ceiling — a collapse from up there (sun dimmed under a parked point)
is not a setpoint problem, and a floor above the ceiling would park the loop
at open circuit drawing nothing. If the first estimate is still under the
knee (the average lags the real voltage by ~one PWM count), the next
teardown ratchets it once more; that converges in one or two instead of
never.

**The floor belongs to the learned panel, not the session.** The reset moved
out of `tracking_common()` into `enter_tracking_fresh()`, next to the Voc
reset it already does. It now survives input-loss bounces *and* HOLD
re-probes; only a fresh seed (sun lost, or `MPPT_RESEED_GAP_MS`) releases
it. `ctx_init()` seeds it to `MPPT_SP_MIN_MV` since nothing else does any
more. Known trade-off: after a light cloud lowers the knee, the tracker
stays fenced a few hundred mV above the new MPP until the next fresh seed —
a few percent under a cloud, against 12 s of full sun per teardown.

**`MPPT_SP_FLOOR_PCT` (80).** `sp_clamp()` floors the setpoint at
`0.80·Voc − deadband` once a credible Voc exists, i.e. the realised
operating point cannot be commanded below 0.80·Voc. No crystalline-silicon
MPP sits below ~0.7·Voc; 80 fences a textbook 0.76 panel ~4 % above its MPP
(stable, a few % of power) and lifts this panel's floor from 6500 to
~9260 mV. `MPPT_SP_MIN_MV` still applies underneath.

**Telemetry: `spf:` column** — the learned floor, between `sp:` and `dips:`.
The acceptance test for this change is that `spf` steps up on a teardown
and `sp` never goes below it afterwards.

### What this does not fix
The parked-and-dimming class (v0.29 bench result, last bullet). A floor
learned from such a teardown lands above where the panel sat, so the resume
draws less — a reasonable response to "the sun dimmed" — but nothing here
prevents the collapse itself. If the teardown rate stays well above single
digits with `spf` visibly holding, that class is what is left, and the
answer is a raw-sample backoff in the inner loop, not more setpoint logic.

### Status
**Builds clean on both `CHARGER_INPUT_VREG` branches; awaiting bench.**
Verify from a fresh `CHG_ONLY` capture:
1. `spf` rises on (most) `CHG: CC -> OFF` events and `sp ≥ spf` at all times
   until the next `MPPT: OFF -> TRK` with a fresh seed.
2. Teardowns in tight bursts (< 60 s apart) disappear; the residual rate is
   the dimming class only.
3. `sp` never goes below ~0.80·Voc − 1200 (≈ 9260 on the 13.3 V panel).
4. No fault codes; `dips:` stays 0 (a non-zero value would be the first
   observed near-miss and worth an INTRACE look).
5. Mean `Ppanel` continues toward ~3500 mW; MPPT `TRK` power approaches
   `HLD` power.

### Bench result (`serial_20260910_165000.log`, panel connected 162 s → TEMP_CHARGE_BLOCK at 307 s)
145 s of charging, 4 `CC -> OFF`: one is the temperature fault (same tick,
no `INTRACE`, ignore), leaving **3 real collapses**.

- **The floor works mechanically.** `spf` 6500 → 10533 → 10687 → 10929,
  `sp ≥ spf` at every sample. Only the *first* collapse was a downward
  probe (`sp` 9562 — nothing learned yet, the unavoidable learning event);
  no session after it probed below the floor. **The probe class is gone.**
- **The other two are parked in HOLD** — `pwm 95/96`, `Vpanel` 11 637 /
  11 879 steady or slightly *rising*, `Ipanel` flat at ~285 / ~255 mA, no
  dimming, no precursor in any averaged channel, then pinned at V_bat + 190
  mV within one 10 ms sample. `INTRACE` for both: steady 11.6–11.9 V, one
  sample of descent, V_bat. Not a shadow slow enough to see; either a
  transient one or the input going unstable where the panel's impedance
  peaks (the knee). No setpoint logic can pre-empt a 10 ms event.
- Every one of the 18 traces captured across v0.29/v0.30 recovers to Voc
  within 400 ms of the stand-down. **These are live panels over their knee,
  not removed sources** — and the guard cannot tell the difference. That is
  what v0.32 fixes.

---

# [v0.29] - 10.09.26
## Stop the charger teardown limit cycle: debounce the input guard, stop re-seeding MPPT after a bounce, fix the FOCV fraction

Changed files: `charger.c`, `mppt.c`, `main.c`, `hw_config.h`,
`system_types.h`, `SPCBoardAPI.c`, `SPCBoardAPI.h`, `CHANGELOG.md`

Ships in the same binary as the still-unverified v0.28 log-rate work; the
banner is v0.29 so a capture identifies which of the two it came from.

### The problem
Two bench captures in `EM:CHG_ONLY` sun, both with the charger tearing itself
down over and over while nothing was actually wrong:

| | `spc20_f027_test3_solarissue.txt` (09.09) | `serial_20260910_144840.log` (10.09) |
|---|---|---|
| span | 100.9 min | 47.7 min |
| `CHG: CC -> OFF` | 2030 (1207/hr) | 57 (72/hr) |
| OFF dwell, median | 2049 ms | 2044 ms |
| min **averaged** `Vpanel` | 11021 mV | 10989 mV |
| samples under `PANEL_SAFETY_MV` (4800) | 0 / 6057 | 0 / 2533 |
| `fault:` during the events | 0000 | 0000 |

The 2 s dwell is exactly `CHG_INPUT_REARM_MS` and no fault ever latched, so
every one of these is `charger_input_stand_down()` reached from
`charger_input_guard()` — the *instantaneous* `V_panel < V_bat +
CHG_INPUT_LOST_MARGIN_MV` test. The averaged panel voltage never went below
11.0 V in either capture, so `panel_safety_backoff` (the intended graceful
response at 4800 mV) is structurally blind to the event. Moving
`PANEL_SAFETY_MV` cannot help; it cannot see it.

Three separate defects stack up here.

**1. Every resumption threw away the learned MPP.** `enter_disabled()`
deliberately preserves `vreg_setpoint_mv` and `panel_voc_mv` across a
teardown — and then the `MPPT_DISABLED` entry called
`enter_tracking_fresh()` unconditionally, which sets `voc_pending = true`
and `panel_voc_mv = 0`. The preservation was undone one tick later, every
time. In the 10.09 capture 30 of 51 teardowns dropped `sp` from ~11.5 V to
~8.65 V (= 0.76·Voc − deadband) within 10 s, and the median time back to
≥850 mA was ~12.4 s — six times what the 2 s re-arm block itself costs.
Self-perpetuating, too: while `sp` sits ~3 V below the knee the inner loop is
chasing a target in the constant-current region and walks the panel off the
cliff again, which is why the teardowns arrive in tight bursts rather than
singly.

**2. The input guard acted on one unconfirmed conversion.**
`charger_input_present()` reads `get_input_voltage_now()`, i.e. raw
`ADC.Adc1Result[0]`, not the 64-sample average. Nothing corroborated it, and a
false positive costs a full teardown plus the re-arm block plus the MPPT
re-climb. On the steady-state teardowns (`sp` and `pwm` frozen, operating
point dead still for 25+ s at Vpanel 11917 / Ppanel 3492 / Ichg 940 / pwm 74)
there is no precursor in *any* averaged channel, and the average moves ~154 mV
across the teardown second — almost exactly the 11780/64 = 184 mV that one
zeroed sample in a 64-deep window would explain. Everything else in this
firmware debounces a decision this expensive (`has_sun` 3x/30x, the dusk path
80x, every fault with hysteresis); this did not.

**3. `MPPT_SP_FRACTION_PCT` was 76 on a panel that measures 0.87-0.89.** The
seed is `k·Voc − PANEL_VREG_DEADBAND_MV` and the inner loop parks at the band
TOP = `k·Voc`, so k is the voltage the plant is actually driven to. Measured
MPP/Voc was 0.87 (09.09) and 0.89 (10.09, Voc 13307 mV, best point 11846 mV at
4134 mW) under two different irradiance conditions. 0.76·Voc lands ~1.5 V past
the knee where the buck behaves as a constant-power load and the operating
point cannot be held. On 09.09 that alone was the whole failure: `pwm` stepped
down one count per `PANEL_VREG_INTERVAL_MS` until the panel fell over, 2030
times.

### The fix

**MPPT re-seeds only after a real gap** (`mppt.c`, `system_types.h`,
`hw_config.h`). `enter_disabled()` now stamps `disabled_since_ms` and clears
`seed_invalidated`; while the region is down, `mppt_update()` sets
`seed_invalidated` if `has_sun` clears or the gap passes the new
`MPPT_RESEED_GAP_MS` (60 s). The `MPPT_DISABLED` entry then picks
`enter_tracking_fresh()` (cold boot, new panel, long gap) or
`enter_tracking_reprobe()` (a bounce — keeps the setpoint and Voc, restarts at
`MPPT_SP_STEP_MIN_MV`), which is exactly what that function already existed to
do for HOLD re-probes. The discriminator is the one the design already had:
`CHG_INPUT_REARM_MS`'s own sizing note says a genuine disconnect is resolved by
`has_sun` clearing in 1.5 s, and a bounce never holds `V_panel` down that long.
`panel_voc_mv < PANEL_MIN_MV` still forces the fresh path, so a stale flag can
never route a boot with no credible Voc into a path that has neither a real
setpoint nor a setpoint ceiling. `ctx_init()` seeds `seed_invalidated = true`.

**The input guard is debounced over `CHG_INPUT_LOST_SAMPLES` (3) distinct
conversions** (`charger.c`, `SPCBoardAPI.c/.h`, `system_types.h`,
`hw_config.h`). The subtlety that makes this non-trivial:
`charger_input_guard()` runs from the free-running super loop, thousands of
times per second, against a value refreshed every `TICK_ADC_MS` (10 ms) by
`SysTick_Handler`. A plain `if (++count >= N)` reaches N within microseconds
off one stale sample and debounces nothing. So the counter is gated on a new
`adc_sample_seq()` — the existing `num_reads` harvest counter, now `volatile`
and exported — and advances at most once per conversion. One good sample
re-arms. `num_reads++` also moved to the *end* of `read_adc_values()` so the
sequence number is only published once the whole conversion set is coherent.

3 samples = 30 ms of added exposure on a genuine collapse, which is acceptable
because this guard was never the guaranteed protection: `charger_fast_guard`
catches the reverse current that follows and its DEAD-input branch reaches the
same stand-down. `charger_input_stand_down()` is deliberately **not**
debounced — `charger_fast_guard` also calls it, and that call is already
corroborated by measured reverse current.

**`MPPT_SP_FRACTION_PCT` 76 -> 87.** Just under both measurements. It is only
the starting point — P&O hill-climbs from there — and with the re-seed fix it
now runs at cold boot rather than once per teardown.

**Instrumentation for the remaining question.** The steady-state teardowns are
still unexplained: either a real 10-20 ms dip on the panel input (connector,
input cap, ringing) or a single bad conversion. `read_adc_values()` now keeps a
16-deep ring of raw `V_PANEL` (`ADC_PANEL_TRACE_DEPTH`, 160 ms of history);
`charger_input_guard()` snapshots it into the context when it trips and
`main.c` prints it on the next tick as an `INTRACE` line next to the
`CHG: CC -> OFF` it explains. An isolated outlier between healthy ~12 V
neighbours means a bad conversion and the debounce is the complete answer; a
smooth descent over 3-4 samples means a real electrical event and the hardware
needs a look. The telemetry line gains a `dips:` column counting sub-margin
dips that did *not* reach the threshold — i.e. the teardowns this change
prevented.

### Deliberately not in scope
`PANEL_VREG_DEADBAND_MV` is +/-1200 mV, but on the 10.09 panel the entire
usable region between the knee (11.9 V) and open circuit (13.3 V) is 1.4 V —
the band is wider than the region, so "in band" means no restoring force at
all. That is arguably a bigger design issue than the fraction, but narrowing it
risks the whipsaw the wide band was chosen to prevent (see the charge-on/off
limit cycle entry under v0.21). Raise it separately once these are bench-
verified.

### Status
**Builds clean on both `CHARGER_INPUT_VREG` branches; awaiting bench.**
Acceptance criteria from a fresh `CHG_ONLY` capture:
1. `CHG: CC -> OFF` drops from ~72/hr to single digits per hour.
2. After any teardown that still happens, `sp` stays within ~500 mV of its
   pre-teardown value for the next 10 s (was 30/51 failing).
3. Median time back to >=850 mA falls from ~12.4 s to ~2-3 s.
4. No new `fault:` codes — in particular `0x0100` (`FAULT_REVERSE_PUMP`) must
   not become more frequent. If it does, the debounce window is letting real
   collapses through; report it rather than reverting the debounce.
5. Mean `Ppanel` rises toward the ~3500 mW steady-state value.
6. `INTRACE` lines answer the bad-conversion vs. real-dip question, and `dips:`
   shows how many teardowns the debounce absorbed.

Regression checks: a genuine panel unplug must still isolate within ~30 ms and
leave `CHARGE_ONLY` via `has_sun` clearing without latching a fault; a cold
boot with no learned Voc must still seed from FOCV.

### Bench result (`serial_20260910_155320.log`, 14.1 min, cold boot, v0.29)
| # | criterion | result |
|---|---|---|
| 1 | teardowns to single digits/hr | **63.8/hr** (15 in 14.1 min; was 72) — FAIL |
| 2 | `sp` kept across a teardown | **14/15** (was 21/51) — PASS |
| 3 | recovery to ≥850 mA in 2–3 s | median **11.6 s** (was 12.4) — FAIL |
| 4 | no new faults | `fault:0000` all session, history 0000 — PASS |
| 5 | mean `Ppanel` toward 3500 | **2932 mW** (was 2073); CC duty 63 → **95.5 %**, mean `Ichg` 512 → **816 mA** |

Fix 1 works exactly as designed and Fix 3 moved real energy. The teardown
rate did not move because the diagnosis of *what trips the guard* was wrong:

- **`dips:0` for the whole session** — not one sub-margin dip recovered before
  the threshold. All 15 `INTRACE` dumps look alike: steady ~11–12 V, one
  intermediate sample, then **pinned at ~3600 mV = V_bat** for the rest of
  the window. That is a real, sustained panel collapse with a 10–20 ms fall,
  not a bad conversion. The "184 mV ≈ one zeroed sample" arithmetic was a
  coincidence: the average barely moves because the guard stands the region
  down ~40 ms after the fall, so only a few collapsed samples ever enter the
  640 ms window. **Fix 2 guards a failure mode that has never been observed
  on this hardware.** It is harmless (criterion 4) and stays as insurance.
- The debounce also does not trip on the 3rd conversion as its comment
  claimed: trips took **3–7 conversions, median 4**, because the counter
  advances per *observed* sample and the blocking 5 Hz telemetry TX hides
  whole conversions from the foreground. Counting elapsed conversions instead
  would be worse (one reading after a TX block would carry several samples'
  credit and trip alone). Comment corrected; logic unchanged.
- **The real trigger is MPPT probing the setpoint down past the knee** —
  13 of 15 teardowns followed a downward `sp` step within 6 s (median
  1.6 s), `sp` ranged **6500 … 11554 mV** against a measured MPP of 11 850,
  and the tracker sat at the absolute floor `MPPT_SP_MIN_MV` at one point.
  MPPT `HLD` averaged 3342 mW / 938 mA; `TRK` 2701 mW / 757 mA; 12 of 15
  teardowns fired in `TRK`. Tracking is the destabiliser. See v0.30.
- A third class exists that no setpoint logic can touch: parked in `HLD`
  (`pwm 75`, `sp 11304`, `Vpanel 11934` steady for 25 s), `Ichg` drifting
  960 → 872 mA as the sun dims, then the panel falls over with zero
  firmware action. Only a fast inner-loop backoff on the raw sample could
  pre-empt that; out of scope.

---

# [v0.28] - 10.09.26
## Selectable UART telemetry rate: ctx.log_mode (off / 1 Hz / 5 Hz), default 5 Hz

Changed files: `hw_config.h`, `system_types.h`, `main.c`,
`docs/flashing_and_uart_debug.md`, `docs/constants_and_faults.md`, `CHANGELOG.md`

### The problem
The fixed 1 s log tick undersamples the events it is being used to chase. The
teardown limit cycle in `spc20_f027_test3_solarissue.txt` has a median
`CHG: CC -> OFF` dwell of 2049 ms — two samples per cycle — and
`charger_input_guard()` trips on an *instantaneous* `V_panel` reading the log
can miss entirely between lines. Anything shorter than a second (the guard trip,
the dip that caused it, one `PANEL_VREG_INTERVAL_MS` step) is aliased away.

A fixed *fast* tick is not the answer either: a blocking ~27 ms TX five times a
second is not what you want during an overnight capture, and there is no reason
to pay it at all when nobody is reading the port.

### The fix
`ctx->log_mode` selects the rate — `LOG_MODE_OFF` (0) / `LOG_MODE_1HZ` (1) /
`LOG_MODE_FAST` (2, 200 ms), seeded from `LOG_MODE_DEFAULT` (currently FAST) by
`ctx_init()`. `log_interval_ms()` in `main.c` maps mode to interval and the
super-loop re-reads it every pass, so the field is live-writable from a CCS
breakpoint (Expressions -> `ctx.log_mode`) without a reflash. It is `volatile`
for exactly that reason: a debugger write is outside what the compiler can see.

Two deliberate details:

- **OFF silences state-transition lines too**, not just telemetry. Those are
  also blocking TX, so leaving them in would mean "off" still parks the loop
  ~27 ms at every FSM edge — the one thing OFF exists to prevent. The boot
  banner and the HardFault dump stay unconditional.
- **An unrecognised mode logs nothing** (`log_interval_ms()` defaults to 0), so
  a fat-fingered debugger write goes quiet rather than picking some rate.

Switching OFF -> on leaves `last_log` stale, so the first line lands on the next
pass — which is the wanted behaviour when you flip it on to watch something.

`TICK_LOG_MS` is gone, replaced by `TICK_LOG_1HZ_MS` / `TICK_LOG_FAST_MS`.
Boot banner bumped to v0.28 so a capture identifies the build it came from.

### Cost, and why 200 ms is the floor
A line is ~300 chars (315 max across the captured logs) and `printToUART()`
blocks, so at 115200 8N1 each line parks the super-loop ~27 ms: ~14 % of wall
time at FAST, ~3 % at 1HZ, zero at OFF. What that touches:

- The 50 ms pipeline tick is *delayed*, never skipped or bunched — `last_main =
  now` reloads from the actual service time, so there is no catch-up burst.
- The filter-pacing constants (`PANEL_VREG_INTERVAL_MS` 400,
  `CC_DOWNSTEP_INTERVAL_MS` 300) are `>=` tests, so jitter can only stretch
  them, never shorten them. They exist to not outrun the 64-sample ADC average;
  this cannot make them outrun it.
- ADC harvest runs in the SysTick ISR, which preempts the blocking TX.
- `charger_input_guard()` / `charger_fast_guard()` / `bat_wake_fast_guard()` at
  the top of the loop can be delayed by one line's TX. Worst-case guard latency
  is the same ~27 ms in every non-OFF mode — FAST just reaches it more often.

Do not add a faster mode at this baud: below ~150 ms the TX stops fitting
between lines and the loop lives inside `printToUART()`. That needs
`UART1.targetBaudRate` raised in `SPC_20.syscfg` (460800 -> ~7 ms/line, room for
20 Hz) and the terminal changed to match.

**Status: builds clean, awaiting bench.** Verify: (1) default boot gives 5
lines/s by the `ms:` column and the banner reads v0.28; (2) writing
`ctx.log_mode = 1` then `0` at a breakpoint changes the rate and then silences
the port, transitions included, with the unit still running; (3) a CC session
still regulates normally at FAST — the added loop jitter is the only regression
risk here.

---

# [v0.27] - 31.07.26
## Sleep current: shed OUTPUT_EN + USB boost for the full STANDBY0 window

Changed files: `main.c`, `CHANGELOG.md`

### The problem
Bench measured ~4 mA during STANDBY0 sleep — nowhere near the uA class
STANDBY0 should cost. `system_sleep()` only ever powered down the LED boost,
LED bar, and the measurement front-end; OUTPUT_EN and the USB boost (MIC2876 +
2x AP2151 load switches) were left exactly as the entered state set them.
`enter_idle()`, `enter_charge_only()`, and `enter_charge_and_load()` all turn
both on deliberately, so a plugged-in load can be sensed — but a boost
converter's regulation overhead is easily mA-class next to STANDBY0's own
draw, and that's what was actually being measured.

`enter_safe_mode()` already sheds both rails on entry, so this only ever
applied to sleep entered from IDLE.

### The fix
`system_sleep()` now calls `disable_output_switch()` + `disable_usb_boost()`
in its power-down section, next to the existing LED/measurement shutdown.
Restoring them needed no new code: `energy_mode_reapply_entry()` on full wake
already re-runs the current state's entry actions, which already set
OUTPUT_EN and USB_EN correctly per state (on for IDLE, off for SAFE_MODE).

The now-dead `SLEEP_WAKE_LOAD` periodic wake-check is removed: with the rail
off, I_dsg reads dead regardless of what's plugged in, so it could never fire.

### Traded away
A USB device plugged in while the unit is already asleep now gets no power
and isn't sensed until something else triggers a full wake (button, sun,
battery threshold) — previously it charged immediately and the next 10 s
wake-check noticed the draw. Once something does wake the unit,
`enter_idle()`'s `enable_usb_boost()` is a cold restart of both AP2151s: the
2026-07-08 revert established that they cannot cleanly restart into a load
that's already drawing (stalls ~0.9 V; preloading the boost and only pulsing
the switches didn't help either). The specific scenario this change newly
exposes — something plugged in *during* sleep, then a later unrelated wake —
has not been bench-tested.

**Status: builds clean, awaiting bench.** Verify in order: (1) STANDBY0 draw
actually drops to uA-class on a meter with nothing plugged in; (2) a normal
wake (button/sun/vbat) with nothing plugged in restores USB/output cleanly;
(3) the open risk — plug a USB load in while asleep, then trigger an
unrelated wake, and confirm the AP2151s come back instead of stalling at
~0.9 V. If (3) fails, this needs a hardware fix, not another firmware retry —
the same failure already survived one firmware-only attempt.

---

# [v0.26] - 30.07.26
## Input loss is no longer a fault: preventive stand-down, dedicated reverse-pump bit, charger re-arm path

Changed files: `charger.c`, `charger.h`, `energy_mode.c`, `fault_mgr.c`, `fault_mgr.h`, `hw_config.h`, `main.c`, `system_types.h`, `docs/fault_recovery.md`

### The bug
Switching the bench PSU off mid-charge latched `fault:0004`
(`FAULT_OVERCURRENT_CHG`) and tore the session down. The code was misleading:
the third teardown in `serial_20260730_193108.log` latched it at `Ichg:664`,
nowhere near `FAULT_OVERCURRENT_CHG_MA` (2200). `charger_fast_guard` was
reusing that bit for **reverse current**, and killing the input is exactly what
produces reverse current — with the buck at deep duty (pwm 31 ≈ 92 %) and Q49
closed, the cell back-feeds into the collapsing input node.

The guard was also the only thing fast enough to notice. It runs in the
foreground on instantaneous conversions; everything else that could have shut
down gracefully reads the 64-sample average, which still showed `Vpanel:4000` a
full second after the rail was gone, and `has_sun` needs 30 ticks (1.5 s) below
`PANEL_MIN_CLEAR_MV`. So the guard won the race on every disconnect.

### `charger_input_guard` — trip on the voltage, not the current
New foreground guard, called before `charger_fast_guard`, active only while the
charge path is *connected* (PRECHARGE/CC/CV — `CHG_BUCK_SETTLE` holds VCHG below
V_bat by design). It stands the charger down when instantaneous `V_panel` falls
below `V_bat + CHG_INPUT_LOST_MARGIN_MV` (400 mV): Q49 opens **first**, so the
cell's path to the dying rail is cut rather than the reverse current being
caught after the fact.

Judged on panel voltage, not VCHG: at 2 A the whole VCHG-to-V_bat separation is
~44 mV (bench: `Vbat:3570 Vchg:3614`), unusable from a single conversion, while
`V_panel` sits ~9 V clear.

The margin is **sized to stay below `PANEL_SAFETY_MV`** (4800 mV). A
sagging-but-present panel belongs to `panel_safety_backoff`, which steps demand
down and recovers; a guard that could fire in that band would answer an ordinary
over-draw with a full teardown and re-arm, reintroducing the charge-on/off limit
cycle v0.21 fixed. 400 mV puts the trip at 1.9–4.05 V across the cell range,
always clear of that floor, and correct at every cell voltage because the
back-feed crossover tracks V_bat (`V_panel ≈ V_bat`) — the FB `pwm` value is the
buck's *target voltage*, not its switching duty, which is internally
`VCHG/V_panel ≈ 0.28` while charging. Staying under the backoff floor means the
trip lands near the crossover rather than well above it, so it pre-empts the
reversal only sometimes; it always bounds exposure to one 10 ms sample, with
`charger_fast_guard` as the guaranteed backstop.

The guard sets `charger.input_lost_pending` rather than touching the FSM;
`energy_mode_update` calls the existing `deactivate_charger_region()` on the next
tick. That preserves `mppt_limit_ma` across the bounce (as every normal mode exit
does) and keeps the `CHG: … -> OFF` line in the log — `main.c` snapshots the old
charger state at the *start* of the tick, so a state change made between ticks
would compare equal to itself and never be reported.

### `FAULT_REVERSE_PUMP` (bit 8) — the case that still deserves a latch
Reverse current with the input **live** is a real control failure: the cell is
pushed back into a working panel, which can drive `V_panel` above open circuit
toward the TPS564247's ~17 V ceiling (May trace: −1010 mA at 14.1 V on a 13 V
panel). That now latches its own bit. `charger_fast_guard` re-tests input
presence and routes a dead-input trip to the clean stand-down instead, covering
the case where the input dies inside one 10 ms conversion interval.

Registered at all five sites the table-driven fault subsystem requires —
`fault_take_action`, `fault_recovery_met`, `all_bits[]`, `CHG_FAULT_BLOCK_MASK`,
and the header docs. Missing any one fails silently: the recovery default is
`return false` (latched until reboot) and the action default is `break` (no
containment at all).

### `charger_rearm_due` — replacing the restart timer the fault was providing
`charger_update()` hard-returns while `CHG_INACTIVE` ("energy_mode owns
activation") and energy_mode only writes GPIOs on a transition, so the **only**
re-arm path was the `fault.code` falling edge. The latched fault was doing
double duty as a 10 s restart timer; removing it from ordinary input loss would
have stranded the charger inactive inside a `CHARGE_ONLY` that never
transitions. A full disconnect self-heals via `has_sun`, but a dropout shorter
than 1.5 s does not — charging would have stopped silently and permanently.

The no-transition branch now also re-applies entry actions when the mode still
calls for charging, the charger is `CHG_INACTIVE`, `fault.code` is clean, sun is
present, and `CHG_INPUT_REARM_MS` (2000 ms) has elapsed since the stand-down.
Conditions mirror the fault-clear path deliberately, and
`activate_charger_region()` self-guards on `CHG_INACTIVE`, so the call is a
no-op once the charger is running again.

### Net effect
Unplugging the panel, flipping the PSU, a blown input fuse: clean
`CHG: … -> OFF`, no latch, no 10 s lockout, no `flt_hist` noise, ~2 s to restart
instead of 10. The dusk path (`has_sun` power gate) and `PANEL_SAFETY_MV` soft-
collapse backoff are untouched.

**Status: builds clean, awaiting bench.** Test order matches the implementation
order — (1) unplug mid-charge, expect the new code not `0004`; (2) sub-second
dropout, expect charging to resume ~2 s later; (3) confirm the stand-down
pre-empts the guard so neither code appears on a normal disconnect.

---

# [v0.25] - 30.07.26
## Staged charger activation (CHG_BUCK_SETTLE), wake-probe hardening, high-resolution lamp dimming

Changed files: `charger.c`, `charger.h`, `energy_mode.c`, `energy_mode.h`, `fault_mgr.c`, `fault_mgr.h`, `hw_config.h`, `main.c`, `measurements.c`, `mppt.c`, `mppt.h`, `system_types.h`, `SPCBoardAPI.c`, `SPCBoardAPI.h`, `SPC_20.syscfg`, `docs/charger_states.csv`, `docs/transition_table.csv`, `docs/MPPT_states.csv`, `docs/MPPT_transition_table.csv`, `docs/fault_recovery.md`

### Charger: activation staged through a new CHG_BUCK_SETTLE state
`energy_mode` now owns the whole activation: it opens Q49, pre-positions the
buck from the voltage LUT, and enters `CHG_BUCK_SETTLE` (`SETL` in the logs);
the charger no longer self-activates from INACTIVE. `SETL` holds Q49 open until
the **instantaneous** VCHG reading is at least `CHG_BUCK_READY_MARGIN_MV`
(20 mV) above both the activation-time and live V_bat — so a failed or weak
buck start can never connect a below-Vbat rail and reverse-pump the input
(TPS564247 FCCM). Because the LUT's unloaded result can land ~300 mV low
(which used to strand `SETL` forever), the state walks PWM down 2 counts/tick
under raw VCHG feedback until ready, then closes Q49 and picks
PRECHARGE/CC/CV from V_bat. Deactivation now opens Q49 *before* stopping the
buck, for the same reverse-pump reason. MPPT and the `has_sun` dusk power-clear
are keyed on the *connected* charge path (PRECHARGE/CC/CV) so neither runs
against the intentionally unloaded `SETL` rail.

### Charger: foreground reverse-current guard — and its false trip on activation under load
New `charger_fast_guard()` runs every super-loop pass on the latest 10 ms ADC
conversion and raises `FAULT_OVERCURRENT_CHG` (reusing its containment/retry
path) on reverse current while connected. As first written it judged
`chg_current` alone — but chg (R440) is **net cell current**
(`I_cell = I_buck − I_load`), and an activation under load closes Q49 with the
buck at ~zero delivery, so the net legitimately reads −I_load until CC walks
the delivery up. Bench 2026-07-29 (panel replug with lamps on, ~600 mA): every
activation latched `fault:0004` within one tick of `SETL → CC`, then looped
through the 10 s fault retry. Fixed by judging the buck branch,
`I_buck = chg_now + dsg_now`, against `−CHG_REVERSE_CURRENT_MA` — genuine
reverse pumping still trips at the same sensitivity, load current cancels out.

### Wake probe v2 (bat_wake, energy_mode.c)
- **Collapsed-signature lockout detected**: bench 2026-07-29 — with the panel
  absent and the bus fully discharged, the protection-open node reads a hard
  0 mV (below the 500 mV disconnected-cell guard), so no fault ever latched and
  no recovery path armed. `V_bat ≤ BAT_PROT_SIG_COLLAPSED_MAX_MV` (300 mV) is
  now treated as the same lockout; `energy_mode_update()` forces SAFE_MODE on a
  suspected lockout so the probe (SAFE_MODE in-state) can actually run.
- **Staged probe targets**: first attempt at 3.0 V (gentler into a genuinely
  depleted pack), later attempts at the full 3.65 V CV target needed to release
  a healthy protection-open pack. Both fixed, never derived from the invalid
  reading.
- **Probe staged through its own BUCK_SETTLE** with the same Q49/VCHG-ready
  interlock, plus `bat_wake_fast_guard()`: foreground cut at
  `BAT_WAKE_PROBE_CUT_MA` delivered or on reverse current.
- **Validation split**: off-state minimum ≥ 2000 mV → VALIDATED with direct
  handoff to normal charging (`fault_clear()` + `EM_CHARGE_ONLY`, PRECHARGE
  below 3 V); 1500–1999 mV → VALIDATED but kept on the supervised rescue.
- **Sleep gate**: STANDBY is blocked while a wake sequence (probe, handoff, or
  retry wait) is in flight — the collapsed case latches no fault, so the
  existing fault gate didn't cover it, and the 60 s retry cadence outlasts the
  2 min idle-sleep timeout.

### Fault manager
- `fault_clear()` exported for the validated-probe handoff; it only clears the
  bit — hardware re-arm still rides energy_mode's falling-edge handshake.
- Raising a fault now re-anchors the recovery dwell (`last_recovery_ms`), so a
  fault raised after a long quiet interval can't satisfy the recovery cadence
  in the same tick and hide its falling edge from energy_mode.
- Deep-discharge precharge timeout (30 min `BAT_RESCUE_TIMEOUT_MS`) is now
  selected on `fault.history`, not the live code: the wake probe clears the
  undervolt latch before the rescued cell reaches PRECHARGE, and the live-code
  test handed it the short 15 min window — declaring a normally recovering
  cell dead (bench 2026-07-29: probe rescued to 2.65 V, then timed out).

### SAFE_MODE: fault-free low cell with sun now charges
A cell between `BAT_UNDERVOLT_MV` and the 3.2 V recovery threshold with no
latched fault and usable sun exits SAFE_MODE into CHARGE_ONLY (staged
activation, PRECHARGE below 3 V) instead of being stranded with the charger
off. The 3.2 V threshold still gates restoring loads without sun.

### Lamps: 8× LED-current PWM resolution, hold-to-dim floor
The four LED-current timers move from a 100- to an 800-count period
(`SPC_20.syscfg`), and `set_led_current()` linearly interpolates the measured
100-count calibration LUT onto the finer grid — every one of the 40 dim levels
now maps to a distinct current reference. Press-and-hold dimming stops at
`LAMP_DIM_MIN_LEVEL` (dimmest ON level) instead of walking through off: a hold
only changes brightness; off stays tap-only.

### Bench validation needed
(1) Lamps-on panel replug: `SETL → CC` holds, `Ichg` climbs from ≈ −600 mA to
positive over a few 400 ms steps, no `fault:0004`. (2) Staged activation on
both panels: no reverse pump at Q49 close, no `SETL` strand (LUT-low case).
(3) Collapsed-signature lockout (dead bus, no panel → plug panel): probe runs,
staged 3.0 V then 3.65 V targets, TERMINAL after 3 attempts with no battery.
(4) Rescued deep cell precharges on the 30 min window. (5) Hold-to-dim parks at
the floor; per-level brightness is visibly monotonic on the 800-count grid.

---

# [v0.24] - 13.07.26
## Keep absent-source LED bars dark

Changed files: `main.c`, `hw_config.h`, `energy_mode.c`, `CHANGELOG.md`

- The battery bar now stays fully off while `V_bat < UI_BAT_PRESENT_MV`
  instead of flashing all five segments.
- The panel bar now stays fully off while `flag_has_sun` is clear instead of
  flashing all five segments. Normal battery-level and panel-power gauges, and
  the one-time boot sweep, are unchanged.
- Removed the unused `UI_BLINK_PERIOD_MS` setting and updated the boot banner
  to `v0.24`.

---

# [v0.23] - 12.07.26
## Battery protection wake probe: recover the hot-plug ≈0.8 V lockout (S-8240 protection-open)

Changed files: `hw_config.h`, `system_types.h`, `energy_mode.c`, `energy_mode.h`, `fault_mgr.c`, `main.c`, `docs/fault_recovery.md`

### Why — the undervolt latch can fire on a measurement that isn't the cell
Bench 2026-07-12 (`serial_20260712_215354.log`, full analysis in
`docs/bug_battery_hotplug_800mv_lockout.md`): removing the battery while the MCU
stays powered over debug makes the S-8240 protection IC (U46) open Q416/Q417 in
the battery-NEGATIVE path. With cell negative disconnected from system ground,
the single-ended `V_BATM` channel reads a protection-biased node at a stable
744–896 mV — **not** the cell. Firmware classified that as genuine undervoltage
(the 500 mV disconnected-cell guard assumed "missing battery reads ~0", which
the bench disproved), latched `FAULT_BAT_UNDERVOLT`, entered SAFE_MODE, and shed
the charge path. But a charger connection is exactly the S-8240's release
condition — so the protective action removed the only stimulus that could ever
make the measurement valid again. Reconnecting a healthy 3.3 V battery changed
nothing (still ~0.85 V), a 13 V panel with `has_sun:1` changed nothing, and the
lockout even survived MCU reset (the FETs stay open). The v0.22 rescue can't
start (reading < `BAT_RESCUE_MIN_MV`), and lowering that floor wouldn't help:
`activate_charger_region()` would target the false `V_bat` + 50 mV ≈ the 2.79 V
LUT floor — *below* the real cell, no release differential, and it would also
gut the deep-cell safety floor.

### Fix — a bounded wake probe + buck-OFF persistence validation (`bat_wake_tick`, `energy_mode.c`)
A SAFE_MODE in-state FSM, separate from the charger FSM and from the deep-cell
rescue. Key principle: firmware **cannot** distinguish "battery missing",
"protection open", and "genuinely damaged deep cell" from the initial reading —
so the probe never clears a fault, never trusts an on-state voltage (the buck
drives an EMPTY connector to the commanded voltage), and decides everything
from what persists with the stimulus removed.
- **Candidate detection** (`BAT_WAKE_DETECT_TICKS` = 2 s, all clauses each tick):
  `V_bat` inside the signature window `BAT_PROT_SIG_MIN/MAX_MV` (600–1100 mV),
  `FAULT_BAT_UNDERVOLT` the **sole** latched fault, usable sun, no load, 3VOUT
  collapsed (< 1 V). Detection alone declares nothing.
- **Probe** (≤ `BAT_WAKE_PROBE_MS` = 3 s): buck + charge switch up at the
  **fixed** LUT target `BAT_WAKE_PROBE_TARGET_MV` (= 3650 mV CV limit — above
  any healthy cell's resting voltage, so the S-8240's VM pin sees the charger
  differential; never `V_bat + headroom`, the one number known to be wrong).
  Loads stay shed. Cut early once `BAT_WAKE_PROBE_CUT_MA` (200 mA, the
  precharge ceiling) flows — stimulus demonstrably delivered. Abort on sun
  loss, `V_panel < PANEL_SAFETY_MV` (weak panel), or any additional fault.
- **Validation**: buck OFF, `BAT_WAKE_SETTLE_MS` (1.5 s) to flush the 640 ms
  V_bat moving average, then the off-state minimum over `BAT_WAKE_VALIDATE_MS`
  (1 s) decides: ≥ 1500 mV → **VALIDATED** (measurement real again — existing
  machinery takes over: rescue for 1500..3200, fault recovery + SAFE exit at
  ≥ 3200, all unchanged); back in the signature → **NO_BATTERY** (or still
  open) → rate-limited retry; between → **WAKE_FAILED** (real cell below the
  hard floor) → terminal, fault retained, no unattended charging.
- **Retry policy**: one attempt per `BAT_WAKE_RETRY_MS` (60 s, the panel relock
  cadence), max `BAT_WAKE_MAX_ATTEMPTS` (3), then TERMINAL until the panel is
  removed or the fault set changes — an empty connector is never pulsed forever.
- **Untrusted-measurement gates** (`bat_wake_probe_busy()`, `energy_mode.h`):
  while the probe drives the node or the MA is flushing, three consumers hold:
  `fault_mgr`'s undervolt recovery (a driven 3.65 V phantom would clear the
  latch and re-arm loads onto an empty connector via the fault-clear re-arm
  edge), `eval_safe_mode`'s recovery comparison (same phantom would exit SAFE →
  CHARGE_ONLY), and `safe_mode_rescue_active()` (a driven ≥1500 mV reading
  would start the rescue mid-probe and fight for the buck).
- **UART**: `BATWAKE: <phase> -> <phase> res:<result> try:n/3 Vbat:<raw>` event
  lines from the existing transition logger; the signature value is only ever
  logged as the raw untrusted reading, never as a confirmed cell voltage.

### Also fixed — undervolt latched with sun present never reached SAFE_MODE
The `eval_*` guards pick a state from flags alone; with `has_sun` true none of
them selects SAFE_MODE. So an undervolt latched *while the panel was connected*
(hot-unplug mid-charge, or the 0.8 V signature appearing under sun) parked EM in
CHARGE_ONLY with all hardware disabled by the fault action — the charger toggled
PWM into a dead buck until `FAULT_PRECHARGE_TIMEOUT`, and neither the rescue nor
the wake probe (both SAFE_MODE in-state) could ever run. `energy_mode_update()`
now forces `EM_SAFE_MODE` while `FAULT_BAT_UNDERVOLT` is latched — the state
that actually models the torn-down hardware, and the one `docs/fault_recovery.md`
already documented as the invariant ("Energy mode will be in EM_SAFE_MODE").
This routes ALL undervolt handling — genuine and protection-open — through the
rescue/probe recovery paths regardless of sun at latch time.

### Bench validation needed (acceptance criteria in the bug doc)
(1) Hot-plug recovery: debug-power boot, no battery → connect healthy battery →
connect panel: one `WAKE_PROBE`, `VALIDATED`, real `Vbat` reported, undervolt
clears ≤ 10 s later, EM exits SAFE → CHARGE_ONLY. (2) No-battery probe: panel
present, no battery — connector energized ≤ 3 s per attempt, `NO_BATTERY`,
3 attempts 60 s apart, then TERMINAL. (3) Weak-panel probe → `ABORTED`, no rapid
cycling. (4) Deep cell (bench source 1.2–1.4 V behind the released FETs) →
`WAKE_FAILED`, terminal, no unattended charge. (5) Undervolt under sun now
lands in SAFE (not CHG_ONLY) and the rescue/probe engage. (6) Normal charging,
MPPT, CV taper regression unchanged after a validated connection.

---

# [v0.22] - 10.07.26
## Supervised undervolt rescue: make BAT_UNDERVOLT recoverable instead of self-blocking

Changed files: `hw_config.h`, `energy_mode.c`, `energy_mode.h`, `charger.c`, `fault_mgr.c`, `main.c`, `docs/fault_recovery.md`

> (v0.21 — the has_sun dusk-counter split + `LOAD_REACQUIRE_MA` in-band re-acquire
> that fixed the sharp-knee charge-on/off limit cycle — shipped in the source under
> the v0.20 boot banner and never got its own CHANGELOG entry. This entry is v0.22.)

### Why — a Class 3 defect: the fault could never clear itself
`FAULT_BAT_UNDERVOLT` latches at `BAT_UNDERVOLT_MV` (2000 mV) and its recovery
condition is `V_bat > BAT_UNDERVOLT_RECOVER_MV` (3200 mV). But the fault's
protective action disables the buck + charge switch, and by that voltage
`bat_low` has long since forced `EM_SAFE_MODE`, which *also* holds the buck off.
Nothing in the system could raise the cell from ~2.0 V to 3.2 V — relaxation
rebound doesn't cover 1.2 V — so the fault blocked its own cure. This is the
same "no unattended solar recovery once SAFE_MODE latches" caveat from the June
dusk-drain work, now expressed as a fault that can never clear.

### Fix — a supervised, precharge-rate rescue trickle inside SAFE_MODE
While `FAULT_BAT_UNDERVOLT` is the **sole** latched fault, there is usable sun,
and `V_bat >= BAT_RESCUE_MIN_MV` (1500 mV), SAFE_MODE re-permits the charge path
(loads stay shed) so the cell can climb back to the 3200 mV recovery threshold.
`power_budget` already SoC-gates `battery_limit` to 200 mA below 3000 mV, so no
new current limit is needed — the rescue is inherently a precharge trickle.
- **Sole-fault gate** (`safe_mode_rescue_active()`, `energy_mode.c/.h`): requiring
  `fault.code == FAULT_BAT_UNDERVOLT` means no charge-blocking fault (overtemp,
  temp-charge-block, overvolt, overcurrent-chg, precharge-timeout) can be
  co-latched, so we never trickle into a hot/cold/faulted cell. The moment the
  rescue's own escalation raises `FAULT_PRECHARGE_TIMEOUT`, the predicate flips
  false and the rescue stops.
- **Hard floor** `BAT_RESCUE_MIN_MV` (1500 mV): below it, stay latched — a
  LiFePO4 this deep is in copper-dissolution territory and must not be recharged
  unattended. 500..1500 mV is the hard-latched band (500 mV is the existing
  disconnected-cell sense guard).
- **Charge-path re-arm** (`safe_mode_rescue_tick()`, `energy_mode.c`): the SAFE_MODE
  in-state handler calls `activate_charger_region()` when the rescue holds (buck +
  charge switch up, PWM pre-positioned, self-guarded on `CHG_INACTIVE`) and
  `deactivate_charger_region()` + `disable_charge_switch()` when it stops. Every
  SAFE_MODE *entry* still sheds the charge path first — the rescue is a deliberate
  re-arm on the next tick, never a stale enable carried across the transition.
- **Charger run gate** (`charger.c`): `charger_update()` now also runs when
  `safe_mode_rescue_active()` is true (SAFE_MODE is otherwise not a charging mode).
  Because the rescue requires undervolt to be the sole fault, `CHG_FAULT_BLOCK_MASK`
  can never be set while it fires, so the two conditions never conflict.
- **Rescue timeout** (`charger.c`, `BAT_RESCUE_TIMEOUT_MS` = 30 min): the rescue
  enters `CHG_PRECHARGE` (V_bat < 3000) from a deeper start than a normal
  precharge, so while undervolt is latched `tick_precharge` uses the longer
  30-min window before escalating to `FAULT_PRECHARGE_TIMEOUT` — the correct
  "cell is damaged" terminal, user-assisted state. A normal precharge keeps its
  15-min `BAT_PRECHARGE_TIMEOUT_MS`.
- **Recovery alignment**: `FAULT_BAT_UNDERVOLT` recovery stays at 3200 mV, which
  now equals `BAT_SAFE_RECOVER_MV` — the fault clears and SAFE_MODE exits to
  CHARGE_ONLY on the same threshold, handing the (already-active) charger off
  without a reset (`activate_charger_region` no-ops on a running charger).

### Scope / known limitation
The rescue is **supervised**: it only runs while awake, and the MCU stays awake
whenever a fault is latched (`maybe_arm_sleep` gate, unchanged). Sun appearing
while awake starts the rescue within a tick. It does NOT yet recover fully
unattended overnight — with undervolt latched and no sun the MCU cannot sleep,
so it slowly bleeds the cell (pre-existing behaviour, strictly improved: the
cell was previously bricked forever). A fully unattended version would also need
SAFE_MODE to sleep on a lone-undervolt + no-sun condition and the SAFE_MODE
wake-check to wake on sun — deliberately left out of this change.

### Bench validation needed
Own bench session (changes battery-safety behaviour): (1) drain a cell into
SAFE_MODE + `FAULT_BAT_UNDERVOLT`, then illuminate the panel — UART should show
the charge path re-arm (`CHG: OFF -> PRE`) with EM still `SAFE`, `Ichg` at
precharge trickle, loads staying shed; (2) confirm `V_bat` climbs and at 3200 mV
the undervolt fault clears and EM exits SAFE → CHARGE_ONLY cleanly; (3) hold a
cell below 1500 mV under sun and confirm NO rescue (hard latch); (4) co-latch a
temp-charge-block (cold cell) and confirm the rescue does not fire; (5) a
non-climbing cell escalates to `FAULT_PRECHARGE_TIMEOUT` at ~30 min.

---

# [v0.20] - 08.07.26
## Real deep sleep: STANDBY0 with button wake + periodic wake-checks (IDLE and SAFE_MODE)

Changed files: `main.c`, `energy_mode.c`, `energy_mode.h`, `SPCBoardAPI.c`, `SPCBoardAPI.h`, `system_types.h`, `hw_config.h`

### Why
The old idle sleep was a bare `__WFI()` that could not work. The SysConfig power policy has always been STANDBY0, so the WFI did drop the core into deep sleep — but **no wake source was armed**: the buttons' GPIO interrupts were configured at pin level yet the GROUP1 NVIC line was never enabled (`usb_fault_init()` is commented out), and the LFCLK wake timer provisioned in the syscfg (`ADC_LOW_POWER`, TIMG8) was never started. The only enabled interrupts were UART RX (noise), RTC READY, and stale ADC completions — so the device either woke instantly on a leftover ADC/SysTick pend (and then ran at full power for another 2 minutes) or dozed with no legitimate way to ever notice sun, a load, or a button again. Worse, STANDBY freezes the PD1 PWM timers, so a wake would have resumed with all four PWM peripherals' registers wiped (see retention below) and — with the LED boost left enabled by `enter_idle()` — a LEDCTRL pin frozen low is the runaway max-LED-current state from the 2026-06-16 bench session.

### NEW: `system_sleep()` (`main.c`)
- Entered from the main loop on `idle_sleep_pending`. Holds the MCU in STANDBY0; only returns on a real wake condition, with hardware restored and the ms clock corrected.
- **Wake sources** (armed only inside sleep): BTN1/BTN2 edges via GROUP1 (immediate full wake — `gButtonWakeFlag` set by the ISR), and TIMG8 `ADC_LOW_POWER` (STANDBY-capable, LFCLK 16 Hz) firing every `SLEEP_WAKE_INTERVAL_MS` (10 s) for a wake-check.
- **Wake-check** (~`SLEEP_CHECK_SETTLE_MS` = 60 ms awake): sense rails re-asserted, SysTick resumes and refills the ADC, then raw single-sample thresholds decide full wake vs re-sleep. From IDLE: `V_panel > PANEL_MIN_MV` (respecting the dusk relock), `I_dsg > LOAD_DETECT_MA`, or `V_bat < BAT_LOW_MV` (cell present > 500 mV). From SAFE_MODE: only `V_bat > BAT_SAFE_RECOVER_MV` — waking for sun would spin awake all day while the recovery latch holds. False wakes are cheap: the debounced pipeline re-verifies and re-sleeps after one idle timeout.
- **Lost-wakeup race closed**: WFI runs under PRIMASK with SysTick stopped and its pended exception cleared (`PENDSTCLR`) — a button edge in the check window pends in the NVIC and makes WFI fall through instead of being consumed early.
- **Timekeeping**: slept duration is measured on the LFCLK timer (full interval when the ZERO event is NVIC-pending, else `LOAD − count`) and credited via new `timestamp_advance()` under PRIMASK — `time_now()` is continuous across sleep, so debounce windows, the has_sun relock, and tick baselines survive unaware.
- **PD1 retention handled**: TIMG6 (buck FB), TIMG7 (LED boost), TIMA0/TIMA1 (LEDCTRL) lose their registers in STANDBY (SysConfig warning; everything else used is PD0 or retentive). `SYSCFG_DL_saveConfiguration()` at entry, `SYSCFG_DL_restoreConfiguration()` at wake, then `set_buck_pwm(ctx->pwm)` / `set_led_voltage()` / `set_led_current()` re-commit application values before `energy_mode_reapply_entry()` re-enables any rail.
- **What sleep powers down** beyond the entered state: LED boost rail (mandatory — LEDCTRL freeze hazard; gated on all lamps off so nothing visible is lost), LED bar display (content blanked + anodes parked), measurement front-end (`VBATM_EN` + `CRT_SNS_EN`, re-asserted per check — the enable edge is the same re-lock `refresh_vbatm_sense()` uses), and the unused RTC READY interrupt. **USB boost + AP2151s stay ON in IDLE**: they cannot restart into a plugged load (2026-07-08 revert), so a USB device plugged mid-sleep is powered by hardware immediately and detected at the next check.
- UART: entry/exit lines (`SLEEP: enter (IDLE) …` / `SLEEP: wake=BTN|SUN|LOAD|VBAT slept=… checks=…`); TX shifter drained before clock-gating so the last byte isn't garbled.

### CHANGED: sleep entry policy (`energy_mode.c`)
- `maybe_arm_sleep()` replaces the inline IDLE timeout: arms after `IDLE_SLEEP_TIMEOUT_MS` gated on **no latched fault** (recovery needs pipeline ticks — sleeping would freeze a recoverable fault indefinitely) and, in IDLE, **all lamps off** (never kill a lamp the user lit to save power).
- **SAFE_MODE now sleeps too** (same timeout, no lamp gate — the rail is already shed): with loads shed, the awake MCU plus the blinking bar graphs were the dominant drain on an already-low cell. `enter_safe_mode()` anchors `idle_start_ms` (which is now the shared IDLE/SAFE inactivity anchor). Note the bars stop blinking while asleep; a button press wakes for one timeout window and shows the SAFE blink again.
- `energy_mode_reapply_entry()` exported: re-applies the current state's entry actions (same mechanism as the fault-clear re-arm) so sleep restores IDLE's detection rails but keeps SAFE_MODE shed.

### NEW: HAL support (`SPCBoardAPI.c/.h`)
- `gButtonWakeFlag` + GROUP1 ISR button-edge detection; `timestamp_advance()`; `enable/disable_measure_sense()` (VBATM_EN + CRT_SNS_EN pair); `get_input_voltage_now()` / `get_battery_voltage_now()` / `get_discharge_current_now()` — instantaneous (latest-conversion) variants for the wake-check, since the 64-sample averages are stale after a STANDBY window.

### Bench validation needed
Timer-only change validated by compile (tiarmclang 4.0.4, zero warnings) — needs hardware: (1) UART shows `SLEEP: enter` ~2 min after dark/no-load, then silence with wake-check gaps; (2) supply current drops between checks (MCU STANDBY ~µA vs mA); (3) button press wakes instantly and the tap still toggles the lamp; (4) lamp/USB PWM behavior after a sleep→wake cycle (retention restore!); (5) `wake=SUN` on panel illumination and `wake=LOAD` ≤10 s after plugging a USB load; (6) SAFE_MODE sleep + `wake=VBAT` on recovery; (7) JTAG note: STANDBY kills the debug session — bench with UART, not the debugger.

---

# [v0.19] - 17.06.26
## Front-panel LED bar graphs: battery + panel-power gauges with boot sweep

Changed files: `SPCBoardAPI.c`, `main.c`, `hw_config.h`

### Why
The two 5-segment LED bar graphs were dead: `update_led_display()` was a skeleton that lit every segment on both bars and never read any content, the segment/anode primitives (`update_led_bar`, `led_display_init`) were unimplemented, and nothing in the main loop pumped the multiplexer. The bars are now live gauges.

### NEW: content-driven bar-graph multiplexer (`SPCBoardAPI.c`)
- `update_led_bar(data, id)` stores a raw 5-bit segment mask into the shared `leds[]` buffer (bit i ↔ DISP_LED(i+1)); `update_led_display()` renders it. The mux blanks the off-bar's common anode, drives the active bar's segments from `leds[]`, then raises its anode — alternating per 4 ms call (~125 Hz/bar, no ghosting). Polarity recovered from the pre-refactor HAL (commit `6d8fe0c`): segment cathodes (GPIOA DISP_LED1..5) are **active-LOW** (clear = lit), per-bar common anodes (DIG1/DIG2) are **active-HIGH** (set = bar selected).
- `led_display_init()` now blanks the bars; `disable_led_bar()` fixed to actually go dark (both anodes LOW + all segments HIGH) — the old version drove both anodes HIGH (lit) right before `__WFI()`, leaving the display drawing current in sleep.

### NEW: UI policy layer (`main.c`)
- `ui_display_update()` (run on the 50 ms pipeline tick) maps `meas.bat_voltage` → LED_BAR_1 (battery SoC, fills DISP_LED1→5) and `meas.panel_power` → LED_BAR_2 (panel power, fills DISP_LED5→1; the two bars are mirror-imaged on the PCB). A bar flashes all 5 segments when its source is absent: battery when `V_bat < UI_BAT_PRESENT_MV` (no cell), panel when `flag_has_sun` is clear (no usable sun — reuses the debounced dusk power-gate). Present-but-empty / sunny-but-idle shows 0 solid segments, not a blink.
- `led_boot_animation()` — blocking power-on sweep run during bring-up: each bar lights one more segment every `UI_BOOT_ANIM_STEP_MS` (the SysTick-driven mux refreshes each frame).
- `update_led_display()` is serviced from the **1 ms SysTick ISR** (self-rate-limited to 4 ms/bar, ~125 Hz). It was first wired into the foreground loop, which flickered badly: the ~1 s blocking UART telemetry write (and other blocking work) starved the mux for tens of ms, freezing one bar lit and the other dark. Driving it from the ISR makes refresh immune to foreground stalls. `leds[]` is still produced in the foreground; single-byte reads in the ISR are atomic on Cortex-M0+.

### CHANGED
- `hw_config.h` §11: new UI constants — `UI_BAT_PRESENT_MV`, the `UI_BAT_SEG1..5_MV` / `UI_PANEL_SEG1..5_MW` thresholds, `UI_BLINK_PERIOD_MS`, `UI_BOOT_ANIM_STEP_MS`. Panel-power thresholds are display-only and tunable to the deployed panel.

---

# [v0.18] - 12.06.26
## Setpoint-P&O MPPT: per-panel MPP tracking on top of the input-vreg loop

Changed files: `mppt.c`, `mppt.h`, `charger.c`, `energy_mode.c`, `measurements.c`, `system_types.h`, `hw_config.h`, `main.c`, `README.md`, `docs/MPPT_states.csv`, `docs/MPPT_transition_table.csv`

### Why
Bench log `1206_sp.log` (12.06.26, 13 V OC panel): the fixed `PANEL_VREG_SETPOINT_MV` (6500, measured MPP of the 4x array) put the regulation band top (7.7 V) below that panel's I-V knee (~10.9 V). The inner loop had no reachable operating point, walked the panel over the knee every ~8 s, collapsed it to 3.4 V, backed off, and repeated — a permanent collapse/recover sawtooth charging in bursts. The setpoint is a per-panel quantity; it now follows the panel.

### NEW: outer P&O loop on the vreg setpoint (`mppt.c`, full rewrite for `CHARGER_INPUT_VREG=1`)
- MPPT region repurposed: perturbs `ctx->mppt.vreg_setpoint_mv` (consumed by `cc_regulate` instead of the constant) and observes charge current averaged over paced dwells (2 s settle ≥ inner-loop walk + 640 ms ADC window, then 1 s average). It NEVER writes `ctx->pwm` — the inner voltage loop keeps exclusive PWM ownership, so pacing/backoff/reverse-escape/budget clamp stay active during probing. `mppt_owns_pwm()` is hard false in this mode.
- FOCV seeding: Voc learned as a running max of the filtered panel voltage (the activation-tick reading races the 640 ms MA — capturing it raw seeded 8.5 V on a 13.3 V panel in simulation); seed `= 76 %·Voc − deadband`, deferred to the first settled dwell.
- Adaptive probe step 1000→250 mV (halve per reversal, re-double per accept, re-probes start fine): a fixed 1 V step over-jumps the single best parkable PWM count near a steep knee (~10 % of MPP in simulation).
- Collapse branch + dip classifier: V_panel below `PANEL_SAFETY_MV`, or below the band low edge during a measure window (band-hop whipsaw — dwell averages there are phase noise that random-walks P&O), pushes the setpoint up one step and raises a per-session floor so the cliff is tested at most once per session. Pushes don't count toward convergence (counting them could HOLD inside a whipsaw).
- Convergence: ±15 mA noise gate turns flat dwells into reversals (no random walk on flat tops; freezes when `allowed_chg`, not the panel, binds); 3 reversals or the 45 s session cap → HOLD (60 s). Re-probe only while `panel_limited` and charger in PRECHARGE/CC — this is the upward re-probe path that was missing since the stage-2 mppt_limit fix. Collapse while parked (knee rose above the held band) escapes HOLD immediately, paced and only if the step can move.
- Verified in a host-side closed-loop simulation (real `mppt.c` + `charger.c` against a modeled panel/buck/ADC plant): 13 V panel parks at the quantization-limited optimum (89 % of ideal incl. startup, collapse events 127→10 per 6 min vs the naive tracker), stiff source regulates at the budget clamp with no fault, cloud step re-converges.

### CHANGED: preserve learned panel capability + debounced `has_sun` clear (`energy_mode.c`, `measurements.c`, `system_types.h`, `hw_config.h`)
- `deactivate_charger_region()` no longer resets `mppt.state` or `mppt_limit_ma` — the learned panel limit (and now `vreg_setpoint_mv`) survives a brief EM teardown. Previously a momentary bounce released `allowed_chg` to `BUCK_MAX`, the buck overloaded the panel, `V_panel` collapsed below `PANEL_MIN_CLEAR_MV`, `has_sun` cleared, EM dropped to IDLE, and the system oscillated. State transitions are now left to `mppt_update()` on its next tick; `ctx_init()` seeds `mppt_limit_ma = MPPT_LIMIT_DEFAULT_MA`.
- `activate_charger_region()` pre-positions `ctx->pwm` to `set_charging_voltage(V_bat + CHG_ACTIVATION_HEADROOM_MV)` (50 mV) so the buck enters conduction with positive forward bias on tick 1 — avoids the TPS564247 sync-FET reverse-pump and the CC slow-walk that stranded MPPT on a non-conducting buck.
- `debounce_update()` gains a `clear_threshold`: a set flag now needs `clear_threshold` consecutive clear readings to un-latch (and the set path resets `count` on latch). `has_sun` uses `>1` so a transient `V_panel` sag (MPPT perturbation, activation inrush, CC briefly over-driving the panel) no longer tears the charger down; `bat_low` keeps `1` for immediate recovery un-latch.

### CHANGED
- `cc_regulate` targets `ctx->mppt.vreg_setpoint_mv`; `PANEL_VREG_SETPOINT_MV` is now only the cold-boot fallback.
- Telemetry: new `sp` column (live vreg setpoint, mV) between `pwm` and `fault` — log lines are now 29 columns.
- Legacy PWM-perturbing inc-conductance retained verbatim under `CHARGER_INPUT_VREG=0` (both configurations compile).
- NOTE: with this build, `MPPT: OFF -> TRK` lines under `CHARGER_INPUT_VREG=1` are EXPECTED — the previous "stale-build tell" is inverted.

---

# [v0.17] - 29.04.26
## Bring-up diagnostics: boot banner, transition log, sticky fault history, 1 KB stack

Changed files: `main.c`, `system_types.h`, `fault_mgr.c`, `spc20_linker_overrides.cmd` (new), `.cproject`

### NEW: Boot banner over UART before main loop (`main.c`)
- `log_boot_banner()` emits a fixed three-line banner over UART (via `printToUART()`) immediately after the `SYS_INIT → SYS_RUN` transition, before `log_header()`. A terminal attached partway through bring-up now sees a clear "the firmware just started" marker without waiting for the next 1 s log line, and the banner is visually distinct from the `!! HARDFAULT !!` post-mortem so the two can't be confused on a noisy serial trace.
- Banner string lives in flash (`const`) — no RAM cost. Sent before any periodic logging, so it cannot interleave with a tab-separated data line.

### NEW: State-transition logger — 1 line per EM/CHG/MPPT change (`main.c`)
- `log_state_transitions()` snapshots `ctx.energy_mode`, `ctx.charger.state`, and `ctx.mppt.state` at the top of each 50 ms pipeline tick, then compares against the post-tick values and prints one line per region that moved (e.g., `EM: IDLE -> CHARGE_ONLY @ 12345 ms`).
- Snapshot/compare lives in `main.c` rather than inside each FSM module — the FSM modules stay free of UART awareness, and adding/removing the logger is a single-file change. The 1 s periodic log already includes the current state names; transition lines fire only on the edge so quiet operation produces no extra UART traffic.
- The same snprintf buffer (`uart_buf`) used by `log_measurements()` is reused — the two callers run sequentially in `main()`, so there is no overlap.

### NEW: Sticky fault-history bitmask alongside live `fault.code` (`system_types.h`, `fault_mgr.c`, `main.c`)
- Added `uint16_t history` to `fault_ctx_t`. `fault_raise()` ORs the bit into `ctx->fault.history` on every call (including re-raises of an already-latched fault), and the recovery pass in `fault_recover()` deliberately leaves `history` alone — only `ctx_init()` (i.e., reset) clears it.
- Without this, `fault.code` shows only what is *currently* active. A fault that triggers, takes its protective action, then clears via the 10 s recovery cadence becomes invisible by the next log tick — exactly the case during bring-up where transient overcurrent or overvolt events disappear before the operator can read them. `flt_hist` now appears as the last column in the periodic log (`%04X`) and the bit-OR survives until reset.
- Re-raising a latched fault still short-circuits the protective action (no double-disable), but the `history |=` happens above the early return so the trace is faithful even for repeated trips.

### FIX: Linker stack 512 → 1024 B via project-local override fragment (`spc20_linker_overrides.cmd`, `.cproject`)
- `--stack_size=512` is the SDK default for MSPM0G3507 — see `LINKERMSPM0options.js:16` in the MSPM0 SDK (`StackSizeOptions["MSPM0G3507"] = 512`). The `HardFault_Handler` post-mortem path added in v0.14 prints nine `uint32_t` fields with eight blocking UART writes per field (`hf_puthex32`); a deeply-nested call (e.g., HardFault during a printf inside a logger) approached the budget, and the pipeline's snprintf paths in `log_measurements()` already burn ~200 B of stack per invocation.
- Editing `Debug/device_linker.cmd` directly is non-durable: SysConfig regenerates that file from the SDK template (`source/ti/project_config/.meta/linker/device_linker.cmd.xdt` → `LINKERMSPM0options.js`) on every build, reverting the value. The whole `Debug/` tree is gitignored, so a direct edit also can't be committed.
- Solution: a tracked, project-local linker fragment `spc20_linker_overrides.cmd` containing `--stack_size=1024`, added to the linker LIBRARY list in `.cproject`. The TI Arm linker resolves `--stack_size=` last-wins, and the override is sourced AFTER `device_linker.cmd`, so the SysConfig-generated 512 is overridden by 1024 regardless of how often the SDK template regenerates. CCS regenerates `Debug/makefile` from `.cproject` on next build, picking up the new `-Wl,-lspc20_linker_overrides.cmd` automatically.
- SRAM has 32 KB total, so the extra 512 B is negligible against `.data + .bss + .stack` headroom.

---

# [v0.16] - 29.04.26
## Anchor idle-sleep window at SYS_RUN entry

Changed files: `main.c`

### BUG FIX: board enters __WFI() exactly 2 minutes after boot (critical, bring-up blocker)
- `ctx_init()` zeroes the entire context struct, so `ctx.idle_start_ms = 0`. The initial energy mode is `EM_IDLE` (the zero enum value).
- `enter_idle()` (`energy_mode.c:122-132`) is the only place that writes `idle_start_ms = time_now()`, but entry actions only fire on a *state transition* — not when the FSM initialises in `EM_IDLE` already.
- If the board boots dark (no sun, no load — common indoors during bring-up), `eval_idle()` returns `EM_IDLE` every tick, no transition fires, and the no-transition branch in `energy_mode_update()` (`energy_mode.c:340-348`) compares `(time_now() - 0) >= IDLE_SLEEP_TIMEOUT_MS`. With `time_now()` counting from 0, this trips at exactly t = 120,000 ms = 2 minutes.
- Main loop sees `ctx.idle_sleep_pending = true` and calls `__WFI()`. With a JTAG debugger attached this drops the XDS110 debug session.

### FIX: `ctx.idle_start_ms = time_now()` after SYS_RUN entry in `main()`
- One line added immediately after `ctx.system_state = SYS_RUN`. Anchors the 2-minute idle window to the SYS_RUN epoch instead of the zero epoch.
- No change to `enter_idle()` — its existing assignment still owns the timer for every subsequent re-entry into IDLE.

---

# [v0.15] - 29.04.26
## Force buck PWM to safe duty before any switch enable

Changed files: `main.c`

### BUG FIX: buck timer CC starts at SysConfig default, not minimum duty (critical)
- `SYSCFG_DL_init()` programs the buck PWM timer via SysConfig-generated code. The capture-compare register's reset value is whatever was specified in `SPC_20.syscfg`. If that default isn't exactly 1 (which encodes `PWM_MIN_DUTY = 399` after `set_pwm_duty_cycle()`'s `400 - duty` inversion in `SPCBoardAPI.c:959`), the timer holds a non-safe duty from the moment the counter starts.
- `ctx_init()` sets `ctx.pwm = PWM_MIN_DUTY` (`system_types.h:469`), but that only initialises the C struct — the hardware register is untouched until the first `apply_pwm()` call at the end of the first 50 ms pipeline tick.
- Risky window: the first transition into `EM_CHARGE_ONLY` or `EM_CHARGE_AND_LOAD` runs `activate_charger_region()` (`energy_mode.c:107-118`) at pipeline step 5, which calls `enable_input_buck()` — releasing BUCK_DIS. The gate driver immediately starts switching with whatever CC the SysConfig default left in the register. Steps 6 (`mppt_update`) and 7 (`charger_update`) write `ctx->pwm`, and step 8 (`apply_pwm`) finally commits it to hardware — but until that commit, the inductor sees the SysConfig default. At a high-duty default (CC near full scale) the inductor can saturate in microseconds.

### FIX: `set_buck_pwm(PWM_MIN_DUTY)` immediately after `system_init()` in `main()`
- One call inserted between `system_init()` and `timer_init()`. By the time `system_init()` returns, the buck timer has been initialised and started by SysConfig; `set_buck_pwm()` stops the counter, writes the inverted CC for `PWM_MIN_DUTY`, and restarts. From that point onward, every code path that asserts `enable_input_buck()` sees a guaranteed-safe duty in the register, regardless of when `apply_pwm()` next runs.
- Defense-in-depth: `apply_pwm()`'s clamp and `ctx_init()`'s `ctx.pwm = PWM_MIN_DUTY` both still apply — this fix closes the hardware-register gap that neither addressed.

---

# [v0.14] - 28.04.26
## Add HardFault_Handler with UART post-mortem

Changed files: `main.c`

### BUG FIX: hard faults silently freeze the MCU (critical, bring-up blocker)
- The SDK startup file (`startup_mspm0g350x_ticlang.c`) declares `HardFault_Handler` as a weak alias to `Default_Handler`, whose body is `while (1) {}`. The project did not override it.
- On Cortex-M0+ the HardFault vector catches every CPU exception: invalid memory access, executing from unmapped flash, unaligned word access, stack overflow into an invalid region, bad PC after a corrupted return, etc. Without an override, the first fault traps the CPU in `Default_Handler`'s infinite loop — externally indistinguishable from a hung `while(1)` in the main loop. SysTick keeps firing but the pipeline never runs again, and there is no clue on the wire as to what went wrong.
- Especially load-bearing during bring-up: any of the IRQ-handler gaps fixed in v0.12 / v0.13, or any uninitialized-pointer call in module stubs being filled in, would have surfaced as a frozen board with no diagnostic output.

### NEW: HardFault_Handler — naked entry + C body in `main.c`
- Naked stub inspects bit 2 of `EXC_RETURN` (= the value of LR on exception entry) to pick MSP vs PSP, then tail-calls `HardFault_HandlerC(stack, exc_return)` with the active stack pointer in r0 and EXC_RETURN in r1. Cortex-M0+ has no CFSR / HFSR / MMFAR / BFAR, so the hardware-pushed 8-word frame (R0–R3, R12, LR, PC, xPSR) is the only forensics available.
- C handler prints all eight stacked registers, the EXC_RETURN, and the active SP via blocking UART writes (`DL_UART_Main_transmitDataBlocking` on `UART_0_INST`). Output is plain hex with a fixed-format printer (no `snprintf`, no varargs, no heap) so it survives a corrupt BSS or stack. The PC value points directly at the faulting instruction — addr2line / disasm of the `.elf` resolves it to a source line.
- Final state is `while (1) { __asm("wfi"); }` to keep the CPU pinned for JTAG attach without burning power.

---

# [v0.13] - 28.04.26
## Add missing UART / RTC / GPIO-group IRQ handlers

Changed files: `SPCBoardAPI.c`

### BUG FIX: three armed IRQs trap the CPU in Default_Handler (critical)
- `uart_init()` calls `NVIC_EnableIRQ(UART_0_INST_INT_IRQN)` (UART0_INT_IRQn = 15) and SysConfig enables RX/TX interrupts on UART0 (`UART1.enabledInterrupts = ["RX","TX"]`). A single received byte — including a noise spike on PA11 — would fire the IRQ.
- `start_rtc()` calls `NVIC_EnableIRQ(RTC_INT_IRQn)` (= 30) and SysConfig enables the RTC `READY` interrupt. The READY edge fires shortly after `DL_RTC_Common_initCalendar()`.
- SysConfig enables GPIO edge interrupts on PA0 (USB_FLT, FALL) and PB6/PB7 (BTN1/BTN2, RISE_FALL). On MSPM0G3507 both `GPIOA_INT_IRQn` and `GPIOB_INT_IRQn` resolve to NVIC slot 1 (the GROUP1 line). `usb_fault_init()` calls `NVIC_EnableIRQ(FAULT_USB_FLT_PIN)`, and `FAULT_USB_FLT_PIN == DL_GPIO_PIN_0 == 1U`, so it coincidentally arms the GROUP1 slot. Any button press or USB fault edge would vector through it.
- None of `UART0_IRQHandler`, `RTC_IRQHandler`, or `GROUP1_IRQHandler` were defined in the project. The SDK startup file (`startup_mspm0g350x_ticlang.c`) provides each as a weak alias to `Default_Handler` (body: `while (1) {}`). The first edge on any of these sources would freeze the MCU permanently — SysTick stops, the main pipeline never runs again.

### NEW: UART_0_INST_IRQHandler / RTC_IRQHandler / GROUP1_IRQHandler
- Each handler reads-and-clears its pending interrupt status, which is sufficient to release the IRQ line and prevent immediate re-entry. The application does not consume any of these events today — UART is TX-only, the RTC is polled via `gRTCReadReady`, and buttons are polled in `update_buttons()`.
- `UART_0_INST_IRQHandler` reads the UART IIDX (auto-clears it); on `DL_UART_IIDX_RX` it drains the receive register so a held byte does not keep the RX line asserted.
- `RTC_IRQHandler` reads `DL_RTC_getPendingInterrupt(RTC)` to clear the IIDX.
- `GROUP1_IRQHandler` reads enabled interrupt status on both `GPIOA` and `GPIOB` and clears whatever is set, covering the PA0 fault edge and the PB6/PB7 button edges through the same vector.

---

# [v0.12] - 28.04.26
## Add missing ADC IRQ handlers

Changed files: `SPCBoardAPI.c`

### BUG FIX: ADC interrupt traps the CPU in Default_Handler (critical)
- `adc_init()` calls `NVIC_EnableIRQ(ADC0_INST_INT_IRQN)` and `NVIC_EnableIRQ(ADC1_INST_INT_IRQN)`, and SysConfig enables MEM-result interrupts on each peripheral (MEM0/5/8 on ADC0, MEM1/2/4 on ADC1). However, `ADC0_IRQHandler` and `ADC1_IRQHandler` were never defined in the project.
- The MSPM0 SDK startup file (`startup_mspm0g350x_ticlang.c`) provides both as weak aliases to `Default_Handler`, whose body is `while (1) {}`. The first conversion-complete IRQ would land there and freeze the MCU permanently — no measurements, no pipeline, no UART.
- Even without the lockup, `read_adc_values()` gates on `gCheckADC1 && gCheckADC2`, but nothing in the codebase ever sets those flags — so the moving average would have stayed at zero and every getter (`get_battery_voltage`, `get_charge_current`, `get_temperature`, …) would return 0.

### NEW: ADC0_INST_IRQHandler / ADC1_INST_IRQHandler
- Each handler calls `DL_ADC12_getPendingInterrupt()` (which reads + clears the highest-priority pending IIDX) and sets the corresponding `gCheckADCx` flag only on the **last** memory in the configured sequence — `MEM8` for ADC0 (V_USB_2 is the final channel) and `MEM4` for ADC1 (TEMP1 is the final channel). Earlier-mem interrupts in the sequence are auto-cleared by the read; the IRQ tail-chains to drain them before exiting.
- Using only the last index as the "data ready" signal preserves the existing contract in `read_adc_values()`: the function only copies results when **both** sequences are complete, then re-arms and restarts conversions.

---

# [v0.11] - 17.04.26
## Stub unimplemented HAL button/display functions

Changed files: `SPCBoardAPI.c`

### FIX: link errors on buttons_init / led_display_init / get_button_state
- `SPCBoardAPI.h` declared these plus `extern volatile bool check_buttons`, but none were defined in `SPCBoardAPI.c`. `main.c:171` calls `buttons_init()` → unresolved symbol at link time, no `.elf` produced.
- Added empty stubs for `buttons_init()` and `led_display_init()`, a `get_button_state()` that returns `false`, and a definition `volatile bool check_buttons = false`.
- Buttons/display are intentionally inert for now; `update_buttons()` and `update_led_display()` were already implemented and remain functional if called.

---

# [v0.10] - 16.04.26
## Implement full main loop with deterministic pipeline

Changed files: `main.c` (rewritten), `SPCBoardAPI.h`, `SPCBoardAPI.c`

### REWRITE: main.c — bringup stub → production pipeline
- Replaces the v0.05 bringup version (measurements + UART only) with the full 8-step deterministic pipeline
- All modules wired in strict order: `measurements → flags → power_budget → fault_mgr → energy_mode → mppt → charger → apply_pwm`
- Three independent tick rates in the super-loop:
  - 20 ms: button polling (`update_buttons()`)
  - 50 ms: deterministic pipeline (steps 1–8)
  - 1000 ms: UART diagnostic logging

### NEW: apply_pwm() — pipeline step 8
- The **only** function that touches the buck timer register
- Reads `ctx->pwm` (written by charger or MPPT in steps 6/7) and commits to hardware via `set_buck_pwm()`
- Defense-in-depth clamping to `[PWM_MAX_DUTY=1, PWM_MIN_DUTY=399]` — prevents inductor saturation (0) or counter overflow (400) even if upstream has a bug

### NEW: SysTick_Handler — 1 ms ISR
- Increments millisecond timestamp via `update_timestamp()`
- Kicks ADC conversions every `TICK_ADC_MS` (10 ms): `read_adc_values()` harvests completed conversions, `adc_read_step()` starts next pair

### NEW: SYS_INIT → SYS_RUN sequence
- `system_init()` → `timer_init()` → `buttons_init()` → `ctx_init()`
- Enables battery switch early so V_bat is available from the first ADC sample
- Transitions to `SYS_RUN`, sends UART header, enters super-loop

### NEW: idle sleep support
- When `energy_mode` signals `idle_sleep_pending`, main loop disables LED bar and enters `__WFI()` low-power stop mode
- On wake (GPIO or RTC interrupt), clears pending flag, resets idle timer, and resumes pipeline

### UPGRADED: UART logging
- Now logs full system state: all measurements, all flags, energy mode / charger / MPPT state names, power budget outputs, PWM, and fault code
- Tab-separated format preserved for spreadsheet compatibility

### NEW: set_buck_pwm() — HAL function (SPCBoardAPI)
- Thin wrapper: writes a raw PWM value `[1..399]` to the buck converter channel via `set_pwm_duty_cycle(&_pwm_outputs[0], ...)`
- Single point of contact between the pipeline and the buck timer — `apply_pwm()` calls only this function

---

# [v0.09] - 15.04.26
## Implement charger module (pipeline step 7)

New files: `charger.h`, `charger.c`

### NEW: charger_update() — pipeline step 7
- Runs after `mppt_update()` and before `apply_pwm()` so the charger can yield PWM control when MPPT is TRACKING, and its regulation output is what ends up in the timer
- Four-state machine per `docs/charger_states.csv`: `CHG_INACTIVE`, `CHG_PRECHARGE`, `CHG_CC`, `CHG_CV`
- Activation gated by `energy_mode ∈ {EM_CHARGE_ONLY, EM_CHARGE_AND_LOAD}` AND no charge-blocking fault latched; otherwise forced to `CHG_INACTIVE` with `pwm = PWM_MIN_DUTY`

### Self-activation from INACTIVE
- `energy_mode` enables buck + charge switch and leaves `state = CHG_INACTIVE`. The charger picks:
  - `V_bat < 3000 mV` → `CHG_PRECHARGE`
  - `V_bat ≥ 3000 mV` → `CHG_CC`
- Falls through to run a regulation tick in the new state — no wasted tick
- PWM is **not** reset on state entry (per README contract): PRECHARGE → CC → CV hand off the PWM value

### Transitions (per charger_states.csv)
- `PRECHARGE → CC` on `V_bat ≥ BAT_PRECHARGE_MV` (3000 mV)
- `PRECHARGE → FAULT` on 15-minute timeout (`BAT_PRECHARGE_TIMEOUT_MS`) — raises `FAULT_PRECHARGE_TIMEOUT` via `fault_raise()` and parks pwm at `PWM_MIN_DUTY`
- `CC → CV` on `V_bat ≥ BAT_CV_VOLTAGE_MV` (3650 mV) — falls through to CV regulation on the same tick
- `CV → bat_full` when `I_charge < BAT_CV_TAPER_MA` (200 mA) held **continuously** for `BAT_FULL_HOLD_MS` (30 s). Any sample above taper resets the window. Sets `ctx->bat_full = true` which `energy_mode` consumes to exit charging

### Regulation (bang-bang with deadband, identical order across all active states)
1. **Panel safety** (runs first): `V_panel < PANEL_SAFETY_MV` (10 V) → `pwm += PANEL_BACKOFF_STEP` (5), skip remaining regulation. Runs **before** the MPPT gate so the panel is protected even during MPPT perturbations that caused the collapse
2. **MPPT gate**: if `ctx->mppt.state == MPPT_TRACKING`, skip regulation. Transition guards and CV taper tracking still run — they observe measurements regardless of who owns PWM
3. **Bang-bang regulator**:
   - CC/PRECHARGE: target = `ctx->allowed_chg`, deadband `±CC_DEADBAND_MA` (25 mA). Outside deadband, step pwm by `CC_PWM_STEP` (1)
   - CV: target window `[BAT_CV_VOLTAGE_MV, BAT_CV_VOLTAGE_MV + CV_DEADBAND_MV]` (3650..3655 mV), step pwm by `CV_PWM_STEP` (1)

### PRECHARGE target derivation
- No separate 200 mA clamp in the charger — `power_budget_update()` already clamps `allowed_chg` to `BAT_PRECHARGE_MAX_MA` (200 mA) when `V_bat < 3000 mV`. CC regulation is therefore reused directly; "precharge-ness" is enforced upstream

### Fault interaction
- Charge-blocking fault mask: `FAULT_OVERTEMP | FAULT_BAT_OVERVOLT | FAULT_OVERCURRENT_CHG | FAULT_PRECHARGE_TIMEOUT | FAULT_TEMP_CHARGE_BLOCK`
- When any is latched, the charger deactivates but does **not** touch GPIO switches — `fault_mgr` already took the hardware action, `energy_mode` owns the enables. Avoids double-ownership of hardware state
- `FAULT_BAT_UNDERVOLT` deliberately excluded: by the time it trips, `energy_mode` has already moved to `EM_SAFE_MODE` which deactivates the charger path

### PWM sign convention (codified in helpers)
- `pwm_clamp()` enforces range `[PWM_MAX_DUTY=1, PWM_MIN_DUTY=399]`
- `pwm_step(delta)` is the only mutation path: `delta < 0` → higher duty → more current; `delta > 0` → lower duty → less current
- Comments repeat the convention on every regulation step so a future edit can't silently flip a sign

---

# [v0.08] - 15.04.26
## Implement MPPT module (pipeline step 6)

New files: `mppt.h`, `mppt.c`

### NEW: mppt_update() — pipeline step 6
- Runs after `energy_mode_update()` and before `charger_update()` so the charger can see whether MPPT currently owns PWM
- Three-state machine per `docs/MPPT_transition_table.csv`: `DISABLED`, `TRACKING`, `HOLD`
- Activation gated by the charger region being active (`EM_CHARGE_ONLY` or `EM_CHARGE_AND_LOAD`); otherwise forced to `DISABLED` with `mppt_limit_ma = BUCK_MAX_CURRENT_MA` (no panel constraint)

### Algorithm: incremental conductance, integer math only
- Decision quantity: `X = dI·V + I·dV`, signed by `dV` to recover `sign(dP/dV)`:
  - `X > 0` → left of MPP → raise V → `pwm += step` (lower duty)
  - `X < 0` → right of MPP → lower V → `pwm -= step` (higher duty)
  - `X == 0` or `dV == 0 && dI == 0` → hold last direction
- Edge case `dV == 0, dI != 0`: direction chosen from sign of `dI` (V stuck, I moving means we're on a flat stretch)
- Overflow safe on int32: max term `15000 mV × 2000 mA = 3e7` ≪ 2.1e9
- PWM convention explicit in code comments: lower pwm value = higher duty = more current drawn = V_panel falls

### Adaptive step size
- Starts at `MPPT_MAX_STEP_SIZE` (8), halved on every direction reversal down to `MPPT_MIN_STEP_SIZE` (1)
- Convergence: `step_size == 1 AND reversals >= MPPT_CONVERGE_REVERSALS` (6) → transition to HOLD
- Runtime safety: `tracking_start_ms` timer forces HOLD after `MPPT_RUNTIME_MS` (300 ms) even without convergence — handles fast-changing irradiance

### TRACKING entry actions (matches README spec)
- `V_prev = V_panel`, `I_prev = I_panel`, `step_size = MAX`, `reversals = 0`, `max_power = 0`, `last_direction = -1`
- Forces first perturbation `pwm -= step_size` so next tick has a valid `dV`/`dI`
- Records `tracking_start_ms`

### HOLD entry actions
- Parks `ctx->pwm = max_power_pwm` (best operating point seen during the session)
- Publishes `mppt_limit_ma = (max_power_mW × 1000) / V_bat_mV`, clamped to `[0, BUCK_MAX_CURRENT_MA]`. This is what `power_budget_update()` consumes next tick to cap `i_buck_max`
- `V_bat` floored at 1000 mV to avoid a divide-by-near-zero on a missing/disconnected battery

### Transitions implemented (per MPPT_transition_table.csv)
- `DISABLED → TRACKING` on `panel_limited AND has_sun`
- `TRACKING → DISABLED` on `!has_sun` (safety-first, priority 1)
- `TRACKING → HOLD` on convergence or runtime timeout
- `HOLD → DISABLED` on `!panel_limited` or `!has_sun`
- `HOLD → TRACKING` on `hold_time expired AND panel_limited AND has_sun` (`MPPT_HOLD_TIME_MS` = 30 s)

### Contract with charger module
- While `ctx->mppt.state == MPPT_TRACKING`, the charger must skip its CC regulation step — MPPT writes `ctx->pwm`. Outside of TRACKING, CC/CV owns the PWM
- `enter_disabled()` resets `step_size`, `reversals`, `last_direction` so a subsequent reactivation starts from a clean state

---

# [v0.07] - 15.04.26
## Implement fault manager module (pipeline step 4)

New files: `fault_mgr.h`, `fault_mgr.c`

### NEW: fault_mgr_update() — pipeline step 4
- Runs every TICK_MAIN_MS between `power_budget_update()` and `energy_mode_update()`
- Two-phase: detection pass raises/latches faults; recovery pass throttled to `FAULT_RECOVER_WAIT_MS` (10 s) clears latched bits whose recovery condition holds
- Detection covers all 8 fault bits already defined in `system_types.h`:
  - `FAULT_OVERTEMP`: `bat_temp > 60°C` or `board_temp > 60°C`
  - `FAULT_BAT_OVERVOLT`: `V_bat > 3700 mV`
  - `FAULT_BAT_UNDERVOLT`: `V_bat < 2000 mV` — gated by a 500 mV floor so a disconnected battery on boot doesn't false-trip
  - `FAULT_OVERCURRENT_CHG`: `I_chg > 2200 mA`
  - `FAULT_OVERCURRENT_DSG`: `I_dsg > 5000 mA`
  - `FAULT_USB_OVERVOLT`: either USB output > 6000 mV
  - `FAULT_TEMP_CHARGE_BLOCK`: mirror of `!temp_charge_ok` (hysteresis already applied in `flags_update()`)
- `FAULT_PRECHARGE_TIMEOUT` is **not** detected here — it is raised externally by `charger_update()` via `fault_raise()`
- Recovery thresholds use the `*_RECOVER_MV` constants from `hw_config.h` (e.g., overvolt trips at 3700, clears at 3400 — 300 mV hysteresis)

### NEW: fault_raise() helper — public
- Idempotent: re-raising a latched fault is a no-op
- Applies immediate protective hardware action on first raise, scoped per fault class:
  - Charge-side faults (overvolt, chg overcurrent, precharge timeout, temp-charge-block) → disable buck + charge switch
  - Discharge-side (dsg overcurrent) → disable output switch + USB boost
  - Overtemp → disable buck + charger + output + USB (full shutdown)
  - USB overvolt → disable USB boost only
  - Undervolt → disable loads AND charger (prevents further drain + prevents re-triggering overvolt during recovery ramp)
- Exposed in the header so `charger_update()` can signal `FAULT_PRECHARGE_TIMEOUT` when the 15 min precharge timer expires

### Design note
- `energy_mode_update()` does not yet consult `ctx->fault.code`. The immediate hardware shutdown in `fault_take_action()` contains the fault for the current tick, but energy_mode's next-tick entry actions may re-enable switches. A follow-up change should gate energy_mode transitions on `ctx->fault.active` (likely forcing `EM_IDLE` or `EM_SAFE_MODE` when a hard fault is latched).

---

# [v0.06] - 03.04.26
## Implement energy mode finite state machine (pipeline step 5)

New files: `energy_mode.h`, `energy_mode.c`

### NEW: energy_mode_update() — pipeline step 5
- Runs after `fault_mgr_update()` and before `mppt_update()` — the traffic controller that decides which hardware power paths are active
- Five-state machine per `docs/transition_table.csv`: `EM_IDLE`, `EM_CHARGE_ONLY`, `EM_CHARGE_AND_LOAD`, `EM_DISCHARGE_ONLY`, `EM_SAFE_MODE`
- Guard inputs are all read-only fields set by earlier pipeline steps: `flag_has_sun.value`, `flag_bat_low.value`, `has_load`, `bat_full`, `meas.bat_voltage`

### Hardware enable matrix (set on entry)
| State            | CHARGER_EN | BATTERY_EN | OUTPUT_EN | USB_EN | BUCK_DIS |
|------------------|------------|------------|-----------|--------|----------|
| IDLE             | off        | off        | off       | off    | asserted |
| CHARGE_ONLY      | on         | on         | off       | off    | released |
| CHARGE_AND_LOAD  | on         | on         | on        | on     | released |
| DISCHARGE_ONLY   | off        | on         | on        | on     | asserted |
| SAFE_MODE        | off        | on         | off       | off    | asserted |

### Charger/MPPT region control
- `activate_charger_region()`: on entry to CHARGE_ONLY or CHARGE_AND_LOAD — enables buck + charge switch; charger self-determines PRECHARGE vs CC on its next tick. Guarded by `charger.state == CHG_INACTIVE` to avoid resetting a running charge cycle (e.g., CHARGE_ONLY → CHARGE_AND_LOAD keeps charger state intact)
- `deactivate_charger_region()`: on exit from a charging state to a non-charging state — parks PWM at `PWM_MIN_DUTY`, asserts `BUCK_DIS`, resets charger → `CHG_INACTIVE`, MPPT → `MPPT_DISABLED`, clears `bat_full`

### Transition guards — priority-ordered per state
- Each state has a dedicated `eval_*()` function with guards checked in strict if/else priority order (lower number = higher priority = checked first)
- Safety-critical transitions (→ SAFE_MODE) are always highest priority where applicable
- SAFE_MODE recovery requires `V_bat > BAT_SAFE_RECOVER_MV` (3200 mV) — raw voltage, intentionally not debounced. All exits require genuine battery recovery; `!has_load` alone is not an exit (prevents oscillation — see CHANGELOG v0.01)

### Idle sleep timeout
- In IDLE with no transition, tracks time since `idle_start_ms`. After `IDLE_SLEEP_TIMEOUT_MS` (2 min), sets `ctx->idle_sleep_pending = true` for main loop to enter low-power mode

---

# [v0.05] - 01.04.26
## Implement main system entry point with measurements and UART logging

New files: `main.c` (bringup stub)

### NEW: main.c — bringup version
- Minimal entry point for hardware validation: initialises system, runs measurements + flags on a 50 ms tick, logs all ADC values over UART on a 1 s tick
- No state machines, no charger, no MPPT, no fault management — purely for verifying ADC readings and flag behaviour on physical hardware
- SysTick ISR at 1 ms: increments timestamp, kicks ADC conversions every 10 ms
- Tab-separated UART output for spreadsheet import

---

# [v0.04] - 27.03.26
## Implement power budget module (pipeline step 3)

New files: `power_budget.h`, `power_budget.c`

### NEW: power_budget_update() — pipeline step 3
- Computes `i_buck_max = MIN(BUCK_MAX_CURRENT_MA, mppt_limit_ma)`
- Computes `allowed_chg = i_buck_max - I_load`, clamped to `[0, bat_limit]`
- Three battery voltage zones for `bat_limit`:
  - V_bat < 3000 mV → 200 mA (precharge, gentle trickle)
  - V_bat < 3650 mV → 2000 mA (CC, normal bulk charge)
  - V_bat >= 3650 mV → 200 mA (near-full safety cap, defense-in-depth against one-tick CC overshoot before charger transitions to CV)
- Signed arithmetic prevents unsigned underflow when loads exceed buck capacity

---

# [v0.03] - 20.03.26
## Implement measurements module (pipeline steps 1 & 2)

New files: `measurements.h`, `measurements.c`

### NEW: measurements_update() — pipeline step 1
- Reads all ADC channels via HAL `get_*()` functions into `ctx->meas`
- Clamps `panel_current` to 0 if HAL returns negative (sensor noise protection)
- Computes derived values: `panel_power` (mW), `i_bat_net` (signed mA)

### NEW: flags_update() — pipeline step 2
- `bat_low`: debounced (3-count) + hysteresis (2800/2900 mV) — unchanged from prior design
- `has_sun`: debounced (3-count) + hysteresis (9000/8000 mV) — **upgraded from plain bool** to `debounce_flag_t` to filter cloud transients
- `has_load`: hysteresis only (50/30 mA) — load events are clean, no debounce needed
- `panel_limited`: guards against unsigned underflow when `allowed_chg < PANEL_LIMITED_MARGIN_MA`
- `temp_charge_ok`: 10°C hysteresis on recovery (blocked at 0/45°C, resumes at 10/35°C)
- `bat_full`: not touched — owned by charger module

### NEW: debounce_update() helper — static in measurements.c
- Generic debounce logic for any `debounce_flag_t`: counts consecutive set-condition readings, clears on clear-condition

### Changed: system_types.h
- `has_sun` (plain `bool`) → `flag_has_sun` (`debounce_flag_t`) — state machines will read `ctx->flag_has_sun.value`
- `ctx_init()`: added `flag_has_sun` debounce configuration (count_threshold = 3)

### Changed: hw_config.h
- Added `HAS_SUN_DEBOUNCE_COUNT` (3) — consecutive readings before `has_sun` sets (×50 ms = 150 ms)

---

# [v0.02] - 20.03.26
## SPCBoardAPI refactor — HAL/application split

Stripped application logic from SPCBoardAPI so it is a pure HAL.
All removed items are either replaced by the new module architecture
(energy_mode, charger, mppt, UI_MGR) or are pending move there.

### Removed — application logic that doesn't belong in HAL
- `power_manager()` — replaced by energy_mode + charger + power_budget
- `apply_mppt_perturb_observe_step()` and `MpptStep` enum — replaced by mppt module
- `handle_button_input()` — button reading stays in HAL; action policy moves to UI_MGR
- `handle_ir()`, `receive_command()`, `get_command()` — IR command interpretation is application logic
- `receive_command_sw()`, `on_received_edge()`, `get_decoded_value()`, `get_msg_led()`, `l_sw_init()`, `reset_sw_receive()` — entire light switch decoder removed
- `handle_uart()`, `UARTReceive()`, `get_UART_buffer()` — UART receive/parse is application logic; `printToUART()` kept
- `turn_on_outputs()`, `turn_off_outputs()` — energy_mode calls individual enable/disable functions directly
- `startup_safe_connect()` — replaced by new init sequence in main
- `get_system_state()` — replaced by ctx_print() style logging from system_ctx_t

### Removed — toggle functions
- `toggle_charger_switch()`, `toggle_battery_switch()`, `toggle_output_switch()`, `toggle_usb()`, `toggle_buck()`, `toggle_led()` — dangerous in a state machine; state machine uses explicit enable/disable so switch state is always known

### Removed — _log measurement variants
- `get_charge_current_log()`, `get_battery_voltage_log()`, and all other `_log` getters — logging reads `ctx->meas` directly; no need for a parallel getter set
- Internal `buffer_log`, `sum_log`, `avg_readings_log`, `add_sample_log()`, `get_average_log()` also removed

### Removed — display policy functions
- `display_time()`, `display_error_fault()`, `display_ovp_fault()`, `display_ocp_fault()`, `displayCurrentPower()`, `displayChargeStorage()` — what to show is UI_MGR policy; HAL keeps `update_led_bar()` and `update_seven_segment_display()` as raw primitives
- `update_led_display()` mux timing kept; policy calls replaced with `/* UI_MGR sets content here */` comment

### Changed — return types
- `get_temperature()`: `float` → `int16_t` (°C truncated). Thermistor math unchanged internally.
- `get_charge_current()`: `int32_t` → `int16_t`. Max range is ~±2500 mA; int16_t (±32767) is sufficient and avoids wasting a register pair on M0+.

---

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


### BUG FIX: SAFE_MODE missing exit for !has_sun AND has_load AND V_bat recovered (critical)
- **Old:** No transition existed for `V_bat > SAFE_RECOVER_MV AND !has_sun AND has_load`.
- **Problem:** If battery recovers above 3200mV overnight (no sun, load still physically connected), there was no exit from SAFE_MODE. System would stay locked in SAFE_MODE with loads shed indefinitely even though the battery is healthy and able to supply the load.
- **New:** Added `V_bat > SAFE_RECOVER_MV AND !has_sun AND has_load → DISCHARGE_ONLY` (priority 3). Existing `!has_sun AND !has_load → IDLE` shifted to priority 4; `<default>` shifted to priority 5.

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
