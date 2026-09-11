# Implementation brief — v0.28: stop the charger teardown limit cycle

**Audience:** implementing agent. Read `CODEBASE_EXPLAINED.md` (charger + MPPT
sections) and `CHANGELOG.md` before touching anything. All three fixes are
firmware-only; no hardware change is required or implied.

**Baseline:** working tree at v0.27 (`main.c:85` banner). `CHARGER_INPUT_VREG`
is `1` — only that branch matters here.

---

## 0. The evidence these fixes are answering

Two bench captures, both `EM:CHG_ONLY` throughout, both with `fault:0000`
during the events:

| | `spc20_f027_test3_solarissue.txt` (09.09) | `serial_20260910_144840.log` (10.09, first 1463 s) |
|---|---|---|
| span | 100.9 min | 24.4 min |
| `CHG: CC -> OFF` events | 2030 (1207/hr) | 28 (69/hr) |
| OFF dwell, median | 2049 ms | 2044 ms |
| CC duty | 30% → 0% | 95.3% |
| mean `Ichg` | 162 mA → 0 | 820 mA |
| min **averaged** `Vpanel` | 11021 mV | 11175 mV |
| samples below `PANEL_SAFETY_MV` (4800) | **0 / 6057** | **0 / 1463** |

The 2 s dwell is exactly `CHG_INPUT_REARM_MS`, and no fault is latched, so
every one of these teardowns is `charger_input_stand_down()` reached from
`charger_input_guard()` (`charger.c:136`) — the *instantaneous*
`V_panel < V_bat + CHG_INPUT_LOST_MARGIN_MV` test.

The averaged panel voltage never drops below 11.0 V in either capture, so
`panel_safety_backoff` (the intended graceful response, 4800 mV) is
structurally blind to whatever is happening. Do not try to fix this by moving
`PANEL_SAFETY_MV`; it cannot see the event.

In the 10.09 capture the 28 teardowns split cleanly:

* **16** had `sp` or `pwm` moving in the preceding 6 s — MPPT probed the
  setpoint down and the inner loop chased the panel over its I-V knee.
* **12** had `sp` *and* `pwm` frozen, the operating point dead steady for
  25+ s (Vpanel 11917 mV, Ppanel 3492 mW, Ichg 940 mA, pwm 74), and then
  stood down with no precursor in any averaged channel. The 64-sample average
  moves ~154 mV across the teardown second; one zeroed sample in a 64-deep
  window would move it 11780/64 = 184 mV. That is a one-to-two-sample event.

Measured panel MPP/Voc was **0.89 on both days** (0.87 on 09.09, 0.89 on
10.09) against the firmware's `MPPT_SP_FRACTION_PCT` of 76.

---

## Fix 1 — do not re-seed FOCV after a bounce  *(highest payoff)*

### Problem

`enter_disabled()` (`mppt.c:282`) deliberately preserves `vreg_setpoint_mv`
and `panel_voc_mv` across a teardown. But when charging resumes, the
`MPPT_DISABLED` case (`mppt.c:467`) calls `enter_tracking_fresh()`
(`mppt.c:236`), which unconditionally does:

```c
ctx->mppt.voc_pending  = true;
ctx->mppt.panel_voc_mv = 0;   /* relearn Voc from scratch */
```

so the preservation is immediately undone. Measured in the 10.09 capture:
**26 of 28 teardowns dropped `sp` from ~11.2 V back to ~8.65 V**
within 10 s — the FOCV seed firing again. Median time to climb back to
≥850 mA was **12.4 s**, not the 2 s the re-arm block costs.

Worse, it is self-perpetuating: while `sp` sits at 8.65 V the inner loop is
chasing a target ~3 V below the knee, so it walks the panel off the cliff
again. That is why the class-A teardowns arrive in tight bursts
(201/211/234 s, 900/921/944/964 s, 1136/1143/1172/1179/1203 s) rather than
singly.

### Change

Re-seed only on a genuine re-activation, not on an input-loss bounce. The
discriminator already exists in the design: `CHG_INPUT_REARM_MS`'s own comment
in `hw_config.h` says a *genuine* disconnect is resolved by `has_sun` clearing
(`HAS_SUN_CLEAR_COUNT` = 30 ticks = 1.5 s) and the mode leaving `CHARGE_ONLY`.
A bounce never holds `V_panel` down long enough to do that.

1. `system_types.h`, `mppt_ctx_t` — add two fields next to `voc_pending`:

   ```c
   uint32_t disabled_since_ms;  /* time_now() when the region went DISABLED;
                                 * 0 = never (cold boot)                    */
   bool     seed_invalidated;   /* set while DISABLED if has_sun dropped or
                                 * the gap ran long — the learned Voc and
                                 * setpoint can no longer be trusted        */
   ```

   Initialise both in `ctx_init()` alongside the existing `ctx->mppt.*`
   defaults near `system_types.h:675` (`disabled_since_ms = 0`,
   `seed_invalidated = true`, so a cold boot still seeds).

2. `mppt.c`, `enter_disabled()` — record the timestamp and clear the flag:

   ```c
   ctx->mppt.disabled_since_ms = time_now();
   ctx->mppt.seed_invalidated  = false;
   ```

3. `mppt.c`, in `mppt_update()` where the `!charging` early-return lives
   (`mppt.c:449`) — while DISABLED, invalidate the seed if either signal
   fires. This must run *before* the early return, i.e. on ticks where the
   charger is not connected:

   ```c
   if (!ctx->flag_has_sun.value ||
       (time_now() - ctx->mppt.disabled_since_ms) >= MPPT_RESEED_GAP_MS)
       ctx->mppt.seed_invalidated = true;
   ```

4. `mppt.c`, the `MPPT_DISABLED` case (`mppt.c:467`) — pick the entry path:

   ```c
   if (has_sun) {
       if (m->seed_invalidated || m->panel_voc_mv < PANEL_MIN_MV)
           enter_tracking_fresh(ctx);     /* cold / new panel: full FOCV seed */
       else
           enter_tracking_reprobe(ctx);   /* bounce: keep the learned point   */
   }
   ```

   `enter_tracking_reprobe()` (`mppt.c:250`) already does exactly the right
   thing for a resume: `voc_pending = false`, keeps the setpoint and Voc, and
   starts at `MPPT_SP_STEP_MIN_MV`.

5. `hw_config.h` — new constant next to `CHG_INPUT_REARM_MS`:

   ```c
   /* How long the charger region may stay INACTIVE before the learned Voc and
    * MPP setpoint are considered stale and MPPT re-seeds from FOCV. Sized well
    * above CHG_INPUT_REARM_MS (2 s) so an input-loss bounce resumes from the
    * converged point, and above the fault recovery wait (10 s), but short
    * enough that a real panel swap or a sunrise relearns. */
   #define MPPT_RESEED_GAP_MS        60000UL
   ```

### Do not

* Do not delete the FOCV seed. It is still the correct cold-boot behaviour and
  `PANEL_VREG_SETPOINT_MV` alone is a poor fallback.
* Do not change `enter_disabled()`'s preservation of `vreg_setpoint_mv` /
  `panel_voc_mv` — that part is already right; it was just being overwritten.
* Do not skip the `panel_voc_mv < PANEL_MIN_MV` guard in step 4, or a cold
  boot with a stale `seed_invalidated` would take the re-probe path with no
  credible Voc and no setpoint ceiling.

---

## Fix 2 — debounce `charger_input_guard`

### Problem

`charger_input_guard()` (`charger.c:136`) acts on **one unconfirmed ADC
conversion**:

```c
static inline bool charger_input_present(void)
{
    return (uint32_t)get_input_voltage_now() >
           ((uint32_t)get_battery_voltage_now() + CHG_INPUT_LOST_MARGIN_MV);
}
```

`get_input_voltage_now()` (`SPCBoardAPI.c:799`) returns the raw
`ADC.Adc1Result[0]`, not the average. Nothing corroborates it, and the penalty
for a false positive is a full charger teardown plus a 2 s re-arm block — at
~940 mA, per the 10.09 data, ~12 s of lost harvest each.

Everything else in this firmware debounces a decision this expensive
(`has_sun` 3×/30×, the dusk path 80×, faults with hysteresis). This one does
not.

`hw_config.h:534` already concedes that `charger_fast_guard` is the guaranteed
backstop and that this guard "always bounds the exposure to one 10 ms sample" —
so requiring 2–3 consecutive samples adds 20–30 ms of exposure to a path that
was never the primary protection anyway.

### ⚠️ The one thing that will go wrong if you are not careful

`charger_input_guard()` is called from the **free-running super loop**
(`main.c:974`), i.e. thousands of times per second. `ADC.Adc1Result[0]` is
refreshed by `read_adc_values()` from `SysTick_Handler` only every
`TICK_ADC_MS` = **10 ms** (`main.c:831`).

A naive `if (++count >= N) stand_down();` therefore reaches N within
microseconds, re-testing **the same stale sample** N times, and changes
nothing. The counter must advance **once per distinct ADC harvest**.

### Change

1. `SPCBoardAPI.c:552` — make the existing harvest counter visible and safe to
   read from the foreground. It is written in an ISR, so it needs `volatile`
   (a 32-bit aligned access is already atomic on Cortex-M0+, so no critical
   section is required):

   ```c
   volatile uint32_t num_reads = 0;
   ```

2. `SPCBoardAPI.h` — declare an accessor near `get_input_voltage_now()`
   (`SPCBoardAPI.h:159`):

   ```c
   /* Monotonic count of completed ADC harvests (one per TICK_ADC_MS). Lets a
    * foreground guard tell a NEW conversion from a re-read of the same one. */
   uint32_t adc_sample_seq(void);
   ```

   Implement it in `SPCBoardAPI.c` as `return num_reads;`.

3. `system_types.h`, `charger_ctx_t` — add next to `input_lost_pending`:

   ```c
   uint8_t  input_lost_count;    /* consecutive DISTINCT ADC samples with the
                                  * input below the crossover margin         */
   uint32_t input_lost_seq;      /* adc_sample_seq() of the last sample that
                                  * advanced input_lost_count                */
   ```

   Zero both in `ctx_init()`.

4. `charger.c`, `charger_input_guard()` — replace the body with a
   sample-gated debounce. Keep the existing early-out exactly as it is:

   ```c
   void charger_input_guard(system_ctx_t *ctx)
   {
       if (!charger_connected(ctx) || ctx->charger.input_lost_pending) {
           ctx->charger.input_lost_count = 0;
           return;
       }

       if (charger_input_present()) {
           ctx->charger.input_lost_count = 0;   /* one good sample re-arms */
           return;
       }

       /* Below the margin. Only count it once per conversion — this function
        * runs at loop rate (kHz) against a value that updates at 100 Hz, so
        * counting calls would trip on a single sample re-read N times. */
       uint32_t seq = adc_sample_seq();
       if (seq == ctx->charger.input_lost_seq)
           return;
       ctx->charger.input_lost_seq = seq;

       if (++ctx->charger.input_lost_count >= CHG_INPUT_LOST_SAMPLES)
           charger_input_stand_down(ctx);
   }
   ```

5. `charger.c`, `charger_input_stand_down()` (`charger.c:128`) — reset
   `ctx->charger.input_lost_count = 0;` there too, so the next connection
   starts clean.

6. `hw_config.h` — new constant next to `CHG_INPUT_LOST_MARGIN_MV`
   (`hw_config.h:541`):

   ```c
   /* Consecutive DISTINCT 10 ms conversions below the crossover margin before
    * charger_input_guard stands the charger down.
    *
    * The guard reads an UNAVERAGED conversion and its false-positive costs a
    * full teardown plus CHG_INPUT_REARM_MS (~12 s of lost harvest at 1 A,
    * bench 10.09.26: 12 of 28 teardowns had no precursor in ANY averaged
    * channel, and the 64-sample average moved by exactly the ~184 mV that one
    * zeroed sample would explain).
    *
    * 3 samples = 30 ms of added exposure on a genuine collapse. That is
    * acceptable because this guard was never the guaranteed protection:
    * charger_fast_guard catches the reverse current that follows, and its
    * DEAD-input branch reaches the same stand-down. See the sizing note on
    * CHG_INPUT_LOST_MARGIN_MV above. */
   #define CHG_INPUT_LOST_SAMPLES    3U
   ```

### Do not

* **Do not** debounce `charger_input_stand_down()` itself. `charger_fast_guard`
  (`charger.c:145`) also calls it, from its `!charger_input_present()` branch —
  and *that* call is already corroborated by measured reverse current, so it
  must still act immediately. The debounce belongs only in
  `charger_input_guard()`.
* Do not raise `CHG_INPUT_LOST_MARGIN_MV`. Its sizing note is correct: it must
  stay below `PANEL_SAFETY_MV` (4800) or an ordinary over-draw gets answered
  with a teardown, which is the v0.21 limit cycle all over again.
* Do not make the debounce time-based (`time_now()` deltas). The ADC sequence
  counter is the correct clock here; a time gate would still let a single bad
  sample trip it after a stall.

---

## Fix 3 — `MPPT_SP_FRACTION_PCT` 76 → 87

### Problem

`hw_config.h:578` sets the FOCV fraction to 76%. The seed
(`mppt.c:338`) is `k·Voc − PANEL_VREG_DEADBAND_MV`, and the inner loop parks
at the band *top* = `sp + deadband` = `k·Voc`. So the loop is driven toward
0.76·Voc.

Measured MPP/Voc was **0.87 (09.09)** and **0.89 (10.09)** — two different
irradiance conditions, same ratio. 0.76·Voc lands ~1.5 V past the knee, in the
constant-current region where the buck behaves as a constant-power load and the
operating point cannot be held. On 09.09 that alone was the entire failure: the
loop stepped `pwm` down 1 count / 400 ms until the panel fell off, 2030 times.

### Change

```c
#define MPPT_SP_FRACTION_PCT      87
```

Update the surrounding comment to record that 76 was measured wrong on this
panel class (bench 09.09.26 and 10.09.26, MPP/Voc 0.87 and 0.89), and that the
value only sets the *starting* point — P&O still hill-climbs from there.

### Note for whoever reviews this

This is the least important of the three. With Fix 1 in place the seed only
runs at cold boot and after a ≥60 s gap, so a wrong fraction costs one climb
rather than one per teardown. Do it anyway — it makes the cold-boot behaviour
correct — but do not let it substitute for Fix 1.

### Related, deliberately NOT in scope

`PANEL_VREG_DEADBAND_MV` is ±1200 mV. On the 10.09 panel the entire usable
region between the knee (11.9 V) and open circuit (13.3 V) is 1.4 V, so the
band is wider than the region and "in band" means no restoring force at all.
That is arguably a bigger design issue than the fraction, but narrowing it
risks the whipsaw the wide band was chosen to prevent (see the
`charge-on/off limit cycle` entry in `CHANGELOG.md`). **Leave it alone in this
change set** and raise it separately once Fixes 1 and 2 are bench-verified.

---

## Optional but recommended — instrumentation for the class-B question

12 of 28 teardowns on 10.09 have no explanation in any averaged channel. Two
hypotheses remain and the firmware can distinguish them for free:

* a real 10–20 ms dip on the panel input (connector / input cap / ringing), or
* a single bad ADC conversion.

Add a small ring buffer of the last 16 raw `ADC.Adc1Result[0]` values (written
in `read_adc_values()`, `SPCBoardAPI.c:689`) and dump it over UART when
`charger_input_guard()` stands down. Interpretation:

* trip sample 0 or wildly out of range with 12 V neighbours → bad conversion,
  Fix 2 is the complete answer;
* a smooth descent across 3–4 samples → real electrical event, and the
  hardware needs a look.

Note that with `CHG_INPUT_LOST_SAMPLES` = 3, a single bad conversion no longer
trips the guard at all — so if you want this answer, land the instrumentation
and Fix 2 in the same build and log the near-misses (count them even when they
do not reach the threshold).

---

## Build and verify

```
cd Debug && make "SPC20-Nitu Mario.out"
```

Plain `make` compiles but does not link — use the quoted `.out` target.

Bump the banner at `main.c:85` to `v0.28` and add a `CHANGELOG.md` entry in the
existing format (`# [v0.28] - DD.MM.YY`, `## <title>`, `Changed files:`,
`### The problem`, `### The fix`).

### Acceptance criteria, from a fresh bench capture in `CHG_ONLY` sun

All the numbers quoted in this brief come from **1 Hz** telemetry captures. The
log cadence is now **200 ms**, so a fresh capture resolves each teardown across
~5 samples instead of 1 — the criteria below get easier to judge, and the
class-B "no precursor" claim should be re-tested at the higher rate before it is
treated as settled.

1. `CHG: CC -> OFF` rate drops from ~69/hr to single digits per hour.
2. After any teardown that does still occur, `sp` in the following 10 s stays
   within ~500 mV of its pre-teardown value (no drop to ~0.76·Voc). This is the
   direct test of Fix 1 — it was 26/28 failing before.
3. Median time to regain ≥850 mA after a teardown falls from ~12.4 s to ~2–3 s.
4. No new `fault:` codes appear — in particular `0x0100`
   (`FAULT_REVERSE_PUMP`) must not become more frequent. If it does, the
   debounce window is letting real collapses through to `charger_fast_guard`;
   report it rather than papering over it by reverting Fix 2.
5. Mean `Ppanel` over the session rises toward the ~3500 mW steady-state value
   (it was 3080 mW on 10.09).

### Regression checks these fixes must not break

* A genuine source removal (unplug the panel) must still isolate within
  ~30 ms and leave `CHARGE_ONLY` normally via `has_sun` clearing — not latch a
  fault.
* Cold boot with no learned Voc must still seed from FOCV.
* `CHARGER_INPUT_VREG=0` (legacy) build must still compile. Fix 1 touches code
  inside `mppt.c`; check which branch it is in before assuming.
