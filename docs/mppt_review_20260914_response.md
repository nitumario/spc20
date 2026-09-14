# Response to `mppt_review_20260914.md` — verification and fix (v0.41)

Every claim in the review was checked against the working tree (v0.40) and
the frozen logs in `/tmp/spc20-mppt-review` (SHA-256 of the v0.40 capture
matches: `43e1811b…60e49d`, 10310 lines, 2051.641 s). Line numbers below refer
to the v0.40 tree as the review cited them; the corresponding v0.41 code has
moved.

## Versions and evidence table

| Review says | Verified |
|---|---|
| `174314` v0.33 banner line 3; 8 reverse episodes, ~80 s disabled; 6 collapse traces | ✅ 8 `CC -> OFF`, all `fault:0100`, 80.0 s off; 6 INTRACE (5 RECOVERED, 1 PENDING) |
| `180041` v0.34 banner; 468 reverse edges | ✅ 468 (of 615 CHG->OFF; 30 stand-downs) |
| `085920` headerless continuation of v0.36 (`085446`); 27 episodes, ~270 s | ✅ 27, 269.9 s |
| `092508` v0.37-FASTVLOOP; 11 episodes | ✅ 11 |
| `093947` v0.39 banners at lines 3 and 2372; stand-downs with `dips:0` | ✅ both banners; 7 STANDDOWN, 1 reverse |
| `095416` continuation of v0.39; 5 hard stand-downs, 2 reverse | ✅ uptime continuous (398.7 → 402.9 s); 5 STANDDOWN, 2 reverse |
| `110639` v0.40; 6 episodes, ~60 s, 25 droops, no INTRACE, `dips:0` | ✅ 6, 60.0 s, drp 0→25, 0 INTRACE, dips 0 |

## §1 — v0.40 does not physically undo rejected probes

**Confirmed.** `mppt.c:552` reverts the fence variable only; `knee_learn` raises
it only; `pwm_draw_more` (`charger.c:113-132`) returns unchanged whenever
`pwm <= floor`; `mppt.c:459` counts `pwm <= cliff_pwm_min` as arrival.
The log table (lines 880-900) reads exactly as stated: pwm stays 103 while
the fence goes 103 → 106 → 105 → HOLD with `knee:105`. Recount: **3318 of
9824** fenced CC samples have `pwm < pwmf` (33.8 %; the review's 3319/9825
differs by one boundary sample). Longest run 121.099 s, lines 4614-5219 ✅.

**Fix:** clamp 1b in `cc_regulate` (and `cv_regulate`) walks pwm UP to the
fence; the dwell's settle clock starts when `pwm == cliff_pwm_min`; the
search cap parks at the best measured fence. Harness T1/T2.

## §2 — Reverse-current recovery is incomplete and inconsistent

**Confirmed, and quantified.** The code references (`charger.c:259, 290, 572,
912, 933, 493, 499`) all check out; the droop guard never set
`input_rescue_ms`. v0.33 lines 234-241 and the ~56 s restart (`OFF -> SETL @
56003`) ✅; six of its eight faults are adjacent to its six INTRACE lines ✅.
The four v0.40 droop→fault pairs ✅ (51.369/51.482, 214.279-479/214.642,
330.669/331.635, 530.079/530.794).

What the review could not settle from the log — real reverse vs noise — is
settled by the noise statistics: the averaged `Ichg` has sd ≈ 30 mA at every
settled operating point (37 windows), i.e. ~240 mA RMS per raw conversion.
The -100 mA × 3-sample raw test therefore latches within a second at any
delivery under ~100 mA. Every one of the six latches happened at pwm 110-112
(settled delivery 26-50 mA at 109-110) — after a droop backoff (four) or the
SETL snap (two). Both real reverse (the backoff went 1-2 counts past zero =
-45..-90 mA nominal) and noise contributed; neither was the pathology the
fault exists for.

**Fix:** fast guard rewritten (FAST −500 mA raw × 3, AVG −100 mA averaged for
1 s, both blanked after any rescue); droop backoff bounded at zero-draw + 1
and stamps the shared rescue clock; input-guard RECOVERED restores the rail
to zero-draw + 1 immediately. Harness T3, T6, T8.

## §3 — Reconnection invalidates its own interlock

**Confirmed.** `tick_buck_settle` validated VCHG at 108 and then commanded 110
(`charger.c:1173-1177`); log 540.879 → 541.596 s ✅; the 150 ms connect blank
expires before the first 400 ms regulator step ✅.

**Fix:** the SETL ramp lands on the fence and stops; if the rail cannot clear
the cell there within ~300 ms the fence is discarded (`fdrop`); no snap at
close; a close under the fence is corrected by clamp 1b at the paced rate.
Harness T7.

## §4 — Blocking UART in the protection latency path

**Confirmed.** `send_string → printToUART → DL_UART_Main_transmitDataBlocking`
per byte; ~380-character lines at 115200 baud ≈ 33 ms; guards read only the
latest conversion (`adc_sample_seq` gating, no replay). Traces with one or two
precursor conversions exist (v0.40 changelog, the raw traces).

**Fix:** software TX ring + non-blocking pump in the super loop; blocking
flush only before STANDBY; HardFault keeps the blocking writer; dropped lines
counted (`txd`). Not measurable on the host; watch `txd` on the bench.

## §5 — Dwell, convergence and memory

All five bullets **confirmed** in code (`mppt.c:462`, `527-572`, `701-704`,
`724-728`; `charger.c:411-414`; committed v0.38 `598-605`, `751-754`). The
older-log figures ✅: v0.33 2.026 W at pwm 95 (30-45 s), pwm 107 held 156 s,
pwm 109 held 349 s; v0.39 continuation 4.938 W at 463.310 s then a raw 3571 mV
stand-down 115 ms later; 1000-1280 s CC mean 1.723 W, floor 94-95, pwm 99,
sp 12660. The warm-resume description (`energy_mode.c:164-207`, 10-count
cap) ✅.

Additional finding not in the review: 8 of the 25 droops fired at 98-99.6 %
of the session's Voc while delivering 54-95 mA and each ratcheted the fence to
the zero-draw count. The settled I(pwm) curve for the session runs from 563 mA
at pwm 89 to 26 mA at 110 — the panel was never loaded past 96 % of Voc, so
those events were not knee sightings, whatever they were.

**Fix:** arrival-timed settle, contaminated-measure restart, battery-limited
→ HOLD, best-point parking on the cap (all §1/§5); event-sourced knee claims
gated on delivery ≥ 60 mA and V_panel ≤ 95 % Voc; a knee-backed fence pinned
at zero delivery is discarded. **Not changed:** the HOLD setpoint's free
in-band follow (redundant with the droop reference at the same rate; left
until the rest is bench-confirmed). Harness T3, T4, T5.

## Reproduction

The review's `/tmp/spc20_mppt_review/reproduce.c` compiled and reproduced all
five current-behaviour results before the change. `tools/host_harness/` is
the v0.41 counterpart: the real `charger.c` + `mppt.c` against stubbed
hardware and a plant model; 29 checks pass. Both are control-logic checks,
not electrical ones.

## Telemetry note (review's closing paragraph)

Agreed and adopted: `drp` counts guard actions, `dips` counts rescued input
collapses, neither counts 0x0100 teardowns. The v0.40 telemetry comment is
replaced; `DROOP`/`REVTRACE` event lines carry the evidence from now on.

## Addendum after the first v0.41 bench run (v0.42)

`serial_20260914_122229.log` (20 min): no `FAULT_REVERSE_PUMP` at all, so
§2's fix holds on hardware. Three things the review could not have seen from
the v0.40 log came out of the new `DROOP` traces and are fixed in v0.42
(see its CHANGELOG entry): 40 of 48 droop sightings were the buck's own
PWM-write transient (`set_pwm_duty_cycle` restarted the timer on every
50 ms write); the v0.41 Voc gate rejected a real collapse at 96.6 % of Voc
and is removed; and HOLD had no way back to the fence after a kick. The
v0.41 loop-paced UART drain was also far too slow on this 4 MHz part and is
now interrupt-driven — §4's fix as originally shipped made the log worse,
not the guards.
