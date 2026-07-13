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

