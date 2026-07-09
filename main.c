/*
 * main.c — System Entry Point, SysTick ISR, Main Loop
 * =====================================================
 *
 * This is the top-level orchestrator. It owns:
 *   - Hardware initialisation sequence
 *   - The SYS_INIT → SYS_RUN transition
 *   - The main super-loop with tick-based scheduling
 *   - The deterministic pipeline (50 ms tick)
 *   - UART diagnostic logging (1 s tick)
 *   - SysTick ISR (1 ms): timestamp + ADC kick
 *
 * Pipeline (every TICK_MAIN_MS = 50 ms, sequential, no races):
 *
 *   Step 1:  measurements_update(ctx)  — ADC → ctx->meas
 *   Step 2:  flags_update(ctx)         — debounce / hysteresis → flags
 *   Step 3:  power_budget_update(ctx)  — i_buck_max, allowed_chg
 *   Step 4:  fault_mgr_update(ctx)     — detect/latch faults, protect HW
 *   Step 5:  energy_mode_update(ctx)   — mode FSM, hardware enables
 *   Step 6:  mppt_update(ctx)          — MPPT FSM, may write ctx->pwm
 *   Step 7:  charger_update(ctx)       — charger FSM, may write ctx->pwm
 *   Step 8:  apply_pwm(ctx)            — write ctx->pwm → timer register
 *
 * Other periodic tasks (independent of the pipeline):
 *   - Buttons polled every TICK_BUTTON_MS (20 ms)
 *   - UART logging every TICK_LOG_MS (1000 ms)
 *
 * Deep sleep: after IDLE_SLEEP_TIMEOUT_MS of unchanged IDLE/SAFE_MODE,
 * system_sleep() parks the MCU in STANDBY0 with button-edge wake and a
 * periodic LFCLK wake-check (see the DEEP SLEEP section below).
 */

#include "ti_msp_dl_config.h"
#include "SPCBoardAPI.h"
#include "system_types.h"
#include "measurements.h"
#include "power_budget.h"
#include "fault_mgr.h"
#include "energy_mode.h"
#include "mppt.h"
#include "charger.h"
#include <stdio.h>
#include <string.h>

/* ── The one context struct ── */
static system_ctx_t ctx;

/* ── UART transmit buffers ──
 *
 * Two separate buffers so a state-transition log line emitted between
 * ticks cannot clobber an in-flight per-tick telemetry line (and vice
 * versa). printToUART is blocking, but snprintf into a shared buffer
 * still corrupts a partially-formatted line if a second formatter runs
 * after send_string() returns and before the caller is done with it. */
#define UART_BUF_SIZE 512
static char uart_buf[UART_BUF_SIZE];      /* per-tick telemetry */
static char uart_evt_buf[128];            /* state-transition events */

/* =========================================================================
 * UART LOGGING
 * =========================================================================
 *
 * Tab-separated output for direct paste into a spreadsheet.
 * Header sent once on startup, data lines every TICK_LOG_MS.
 *
 * Columns cover: uptime, all measurements, all flags, state machines,
 * power budget outputs, PWM, and fault code.
 */

static void send_string(const char *str)
{
    printToUART((char *)str, '\0');
}

/*
 * Boot banner — emitted once before the column header so a freshly
 * attached terminal can see "the firmware just (re)started" without
 * waiting for the next 1 s log tick. Distinguishes a clean reset from
 * a HardFault post-mortem (which prints "!! HARDFAULT !!").
 */
static void log_boot_banner(void)
{
    send_string("\r\n"
                "==============================================\r\n"
                " SPC_20 Solar Charge Controller - boot v0.20\r\n"
                "==============================================\r\n");
}

/*
 * State-transition logger — emits one line per FSM region (EM / CHG /
 * MPPT) when its state differs from the snapshot taken before the
 * pipeline ran. Snapshot/compare lives here in main.c so the FSM
 * modules don't need to know about UART or each other's state.
 *
 * Output is plain text, distinct from the tab-separated log line, so
 * it's grep-able after the fact:
 *   "EM: IDLE -> CHARGE_ONLY @ 12345 ms"
 */
static void log_state_transitions(uint32_t now,
                                  energy_mode_state_t em_old,
                                  charger_state_t chg_old,
                                  mppt_state_t mppt_old)
{
    if (ctx.energy_mode != em_old) {
        int len = snprintf(uart_evt_buf, sizeof uart_evt_buf,
            "EM: %s -> %s @ %lu ms\r\n",
            em_state_name(em_old), em_state_name(ctx.energy_mode),
            (unsigned long)now);
        if (len > 0 && len < (int)sizeof uart_evt_buf) send_string(uart_evt_buf);
    }
    if (ctx.charger.state != chg_old) {
        int len = snprintf(uart_evt_buf, sizeof uart_evt_buf,
            "CHG: %s -> %s @ %lu ms\r\n",
            chg_state_name(chg_old), chg_state_name(ctx.charger.state),
            (unsigned long)now);
        if (len > 0 && len < (int)sizeof uart_evt_buf) send_string(uart_evt_buf);
    }
    if (ctx.mppt.state != mppt_old) {
        int len = snprintf(uart_evt_buf, sizeof uart_evt_buf,
            "MPPT: %s -> %s @ %lu ms\r\n",
            mppt_state_name(mppt_old), mppt_state_name(ctx.mppt.state),
            (unsigned long)now);
        if (len > 0 && len < (int)sizeof uart_evt_buf) send_string(uart_evt_buf);
    }
}

/*
 * Per-tick telemetry. One line per tick, space-separated, with each
 * value prefixed by a "label:" tag so the line is self-describing
 * (e.g. "ms:34270 Vbat:3200 ..."). Fields, in order:
 *
 *   1  time_ms
 *   2  bat_voltage         (mV)
 *   3  chg_voltage         (mV)
 *   4  out_voltage         (mV)
 *   5  panel_voltage       (mV)
 *   6  usb1_voltage        (mV)
 *   7  usb2_voltage        (mV)
 *   8  panel_current       (mA)
 *   9  chg_current         (mA, signed)
 *  10  dsg_current         (mA)
 *  11  panel_power         (mW)
 *  12  i_bat_net           (mA, signed)
 *  13  bat_temp            (C)
 *  14  board_temp          (C)
 *  15  flag_bat_low
 *  16  flag_has_sun
 *  17  has_load
 *  18  temp_charge_ok
 *  19  panel_limited
 *  20  bat_full
 *  21  i_buck_max          (mA)
 *  22  allowed_chg         (mA)
 *  23  energy_mode         (name: IDLE/CHG_ONLY/CHG+LOAD/DSG_ONLY/SAFE)
 *  24  charger.state       (name: OFF/PRE/CC/CV)
 *  25  mppt.state          (name: OFF/TRK/HLD)
 *  26  pwm
 *  27  mppt.vreg_setpoint_mv (mV — live input-vreg target, owned by MPPT)
 *  28  fault.code          (hex)
 *  29  fault.history       (hex)
 */
static void log_measurements(void)
{
    measurements_t *m = &ctx.meas;
    int len = snprintf(uart_buf, UART_BUF_SIZE,
        "ms:%lu Vbat:%u Vchg:%u Vout:%u Vpanel:%u Vusb1:%u Vusb2:%u "
        "Ipanel:%u Ichg:%d Idsg:%u Ppanel:%ld Ibat_net:%ld "
        "Tbat:%d Tboard:%d "
        "bat_low:%u has_sun:%u has_load:%u temp_ok:%u p_limited:%u bat_full:%u "
        "i_buck_max:%u allowed_chg:%u "
        "EM:%s CHG:%s MPPT:%s pwm:%u sp:%u fault:%04X flt_hist:%04X\r\n",
        (unsigned long)time_now(),
        m->bat_voltage, m->chg_voltage, m->out_voltage,
        m->panel_voltage, m->usb1_voltage, m->usb2_voltage,
        m->panel_current, (int)m->chg_current, m->dsg_current,
        (long)m->panel_power, (long)m->i_bat_net,
        (int)m->bat_temp, (int)m->board_temp,
        (unsigned)ctx.flag_bat_low.value, (unsigned)ctx.flag_has_sun.value,
        (unsigned)ctx.has_load, (unsigned)ctx.temp_charge_ok,
        (unsigned)ctx.panel_limited, (unsigned)ctx.bat_full,
        ctx.i_buck_max, ctx.allowed_chg,
        em_state_name(ctx.energy_mode),
        chg_state_name(ctx.charger.state),
        mppt_state_name(ctx.mppt.state),
        ctx.pwm, ctx.mppt.vreg_setpoint_mv, ctx.fault.code, ctx.fault.history);
    if (len > 0 && len < UART_BUF_SIZE) send_string(uart_buf);
}
/* =========================================================================
 * PIPELINE STEP 8: apply_pwm
 * =========================================================================
 *
 * The ONLY function that touches the buck timer register.
 * Reads ctx->pwm (written by charger or MPPT in steps 6/7) and commits
 * it to hardware via the HAL.
 *
 * Range enforcement: ctx->pwm should already be in [1, 399] — clamped
 * by mppt and charger. We clamp here as defense-in-depth so a bug
 * upstream can never write 0 (100% duty, inductor saturation) or 400
 * (counter overflow / glitch).
 */
static void apply_pwm(system_ctx_t *c)
{
    uint16_t pwm = c->pwm;

    if (pwm < PWM_MAX_DUTY) pwm = PWM_MAX_DUTY;   /* floor at 1   */
    if (pwm > PWM_MIN_DUTY) pwm = PWM_MIN_DUTY;   /* ceil  at 399 */

    set_buck_pwm(pwm);
}

/* =========================================================================
 * LAMP BUTTON HANDLER
 * =========================================================================
 *
 * The two front-panel buttons each drive a synced PAIR of LED-output lamps:
 *
 *   BTN1 (BUTTONS[0]) -> LEDCTRL1+LEDCTRL2 (LED1, LED2) -> lamps 1 & 2
 *   BTN2 (BUTTONS[1]) -> LEDCTRL3+LEDCTRL4 (LED3, LED4) -> lamps 3 & 4
 *
 * All four LED current-source channels are wired to the MCU (schematic R2:
 * LEDCTRL1=PA9, LEDCTRL2=PB4, LEDCTRL3=PB8, LEDCTRL4=PA12). LEDCTRL1/3/4
 * share TIMA0 (CCP1/CCP0/CCP3); LEDCTRL2 is TIMA1 CCP0. The button->lamp
 * grouping lives in btn_lamp_group[]; the gesture below is applied to both
 * lamps in a group so they track as one.
 *
 * Two gestures per button (the HAL gives us press/release edges, a 500 ms
 * hold threshold in BUTTONS[].holdThreshold, and pressStartTime):
 *
 *   - SHORT TAP (released before the 500 ms hold threshold): toggle. Off ->
 *     full brightness; any on-level -> off.
 *   - PRESS & HOLD (held past 500 ms): dim. While the button stays down, drop
 *     one brightness level every LAMP_DIM_STEP_MS, walking full -> ... -> off
 *     (LAMP_DIM_LEVELS steps, ~5 s end to end). Release at any point keeps the
 *     level reached. Holding from off does nothing — tap to turn on first.
 *
 * Brightness lives in c->lamp_level[] (0 = off, LAMP_DIM_LEVELS = full),
 * mapped to an LED current by lamp_level_ma[]. c->lamp_hold_steps[] counts the
 * dim steps already applied in the current hold, so the cadence is paced off
 * pressStartTime and is independent of how often this runs.
 *
 * The shared LED boost / output switch are owned by energy_mode(), so a lamp
 * only physically lights when its level is non-zero AND the rail is up.
 *
 * Must run right after update_buttons() so the one-shot release edge (cleared
 * on the next poll) is still set.
 */

/* Brightness ladder, indexed by lamp_level: 0 = off (LED current 0 -> the
 * ~15 mA compare-99 floor), LAMP_DIM_LEVELS = full. Intermediate currents are
 * chosen to land on output_currents_led_mA LUT bins and rise monotonically to
 * LAMP_ON_CURRENT_MA. The LUT has only ~15 distinct bins between the floor and
 * full, so with LAMP_DIM_LEVELS (20) steps a handful of adjacent levels repeat
 * a current (set_led_current snaps to the nearest bin anyway); the dups are
 * spread out so a held dim still walks down smoothly. Must have
 * LAMP_DIM_LEVELS + 1 entries with index LAMP_DIM_LEVELS == LAMP_ON_CURRENT_MA. */
static const uint16_t lamp_level_ma[LAMP_DIM_LEVELS + 1] = {
    0,
     25,  35,  40,  50,  50,
     60,  70,  80,  80,  90,
    100, 105, 105, 115, 125,
    135, 135, 145, 145, LAMP_ON_CURRENT_MA,
};

static const LED_OUTPUT lamp_led[4] = { LED1, LED2, LED3, LED4 };

/* Button -> lamp-group map. Each front-panel button drives a synced pair of
 * lamps; the gesture is applied identically to both members of the group. */
static const uint8_t btn_lamp_group[2][2] = {
    { 0, 1 },   /* BTN1 (BUTTONS[0]) -> lamp 1 + lamp 2 */
    { 2, 3 },   /* BTN2 (BUTTONS[1]) -> lamp 3 + lamp 4 */
};

/* Drive a lamp's current level to its LED channel and log the change. */
static void lamp_apply(system_ctx_t *c, uint8_t i)
{
    uint16_t ma = lamp_level_ma[c->lamp_level[i]];
    set_led_current(ma, lamp_led[i]);

    char buf[40];
    int n;
    if (c->lamp_level[i] == 0)
        n = snprintf(buf, sizeof buf, "LAMP%u: OFF\r\n", (unsigned)(i + 1));
    else
        n = snprintf(buf, sizeof buf, "LAMP%u: L%u %umA\r\n",
                     (unsigned)(i + 1), (unsigned)c->lamp_level[i], (unsigned)ma);
    if (n > 0 && n < (int)sizeof buf) send_string(buf);
}

static void lamp_buttons_update(system_ctx_t *c)
{
    for (uint8_t b = 0; b < 2; b++) {
        Button *btn = &BUTTONS[b];
        const uint8_t *grp = btn_lamp_group[b];   /* the two lamps this button drives */

        /* New press: start a fresh hold-step count for this button. */
        if (is_button_pressed(btn))
            c->lamp_hold_steps[b] = 0;

        /* Hold = dim. While the button is physically down (state 0 = pressed),
         * apply one level drop per LAMP_DIM_STEP_MS elapsed since press to BOTH
         * lamps in the group. The while-loop catches up if this runs late,
         * keeping a steady cadence. */
        if (btn->state == 0) {
            uint32_t held = time_now() - btn->pressStartTime;
            uint8_t due = (uint8_t)(held / LAMP_DIM_STEP_MS);
            if (due > LAMP_DIM_LEVELS) due = LAMP_DIM_LEVELS;
            while (c->lamp_hold_steps[b] < due) {
                c->lamp_hold_steps[b]++;
                for (uint8_t k = 0; k < 2; k++) {
                    uint8_t i = grp[k];
                    if (c->lamp_level[i] > 0) {
                        c->lamp_level[i]--;
                        lamp_apply(c, i);
                    }
                }
            }
        }

        /* Tap = toggle. A release that never crossed the hold threshold
         * (isHeld still false) is a short tap; a hold-release is ignored here
         * because the dimming above already handled it. Toggle the whole group
         * together, following the first lamp so the pair tracks as one. */
        if (is_button_released(btn) && !is_button_held(btn)) {
            uint8_t level = (c->lamp_level[grp[0]] > 0) ? 0 : LAMP_DIM_LEVELS;
            for (uint8_t k = 0; k < 2; k++) {
                uint8_t i = grp[k];
                c->lamp_level[i] = level;
                lamp_apply(c, i);
            }
        }
    }
}

#if LOAD_DUMP_TEST
/* Bench-only synchronized load dump (hw_config.h LOAD_DUMP_TEST). In this build
 * the normal lamp buttons are disabled (see the call site) so a stray tap can't
 * change the load mid-test — this handler owns the lamps entirely:
 *
 *   - LOAD: with both buttons up and no load applied (at boot, and after each
 *     dump), it drives all four lamps to full and re-enables the USB rail (clearing
 *     usb_force_off) so there's a full load to dump, and emits "LOAD_ARMED".
 *   - DUMP: holding BOTH buttons past their 500 ms hold threshold drops all four
 *     lamps to the off-floor AND cuts the USB rail in ONE tick, redirecting the
 *     buck's surplus into the battery so CH1 (I_CHARGE) captures the transient, and
 *     emits "LOAD_DUMP". The four set_led_current() calls and disable_usb_boost()
 *     run back-to-back with NO UART between them, so the whole load drops within
 *     microseconds (lamp_apply() is avoided here precisely because its per-lamp
 *     logging would stagger the drops ~1 ms).
 *
 * Cutting USB needs the usb_force_off override: energy_mode OWNS USB_EN and
 * re-enables it on the very next mode transition — and dropping the USB load itself
 * causes that transition (DISCHARGE_ONLY → IDLE). Without the latch the rail comes
 * straight back on and, because the AP2151 load switches can't restart into a
 * still-plugged device (they stall ~0.9-1.6 V), oscillates 5V → ~1.6V → 0. The
 * DUMP branch sets usb_force_off so energy_mode's entry actions keep USB off for
 * the whole hold; the LOAD branch clears it on release.
 *
 * The load auto-reloads on the next both-up, so the test is repeatable: dump →
 * hold (lamps + USB stay off) → release both → lamps + USB come back → dump again.
 * NOTE: on release, re-enabling USB into a still-plugged CC bench load can itself
 * stall the AP2151 at ~1.6 V (hardware limit) — fine for a self-limiting/resistive
 * load; with a stubborn CC load, drop the load before releasing. */
static void load_dump_test(system_ctx_t *c)
{
    static bool loaded = false;   /* is the lamp load currently applied? */

    bool both_up   = (BUTTONS[0].state == 1) && (BUTTONS[1].state == 1);
    /* isHeld latches — update_buttons() only clears it on the next press-down
     * edge, not on release — so it stays set after the buttons come back up.
     * Gate on state==0 (currently pressed) too, or the dump re-fires every tick
     * after release and races the re-arm (endless LOAD_DUMP/LOAD_ARMED). */
    bool both_held = (BUTTONS[0].state == 0) && is_button_held(&BUTTONS[0]) &&
                     (BUTTONS[1].state == 0) && is_button_held(&BUTTONS[1]);

    /* (Re)load the lamps to full once both buttons are up and nothing is applied,
     * and release the USB override so energy_mode brings the rail back. */
    if (both_up && !loaded) {
        for (uint8_t i = 0; i < 4; i++) {
            c->lamp_level[i] = LAMP_DIM_LEVELS;
            set_led_current(lamp_level_ma[LAMP_DIM_LEVELS], lamp_led[i]);
        }
        c->usb_force_off = false;            /* let energy_mode re-enable USB */
        enable_usb_boost();
        loaded = true;
        send_string("LOAD_ARMED\r\n");
    }

    /* Dump: both held → cut all four lamps AND the USB rail in one tick, and latch
     * usb_force_off so energy_mode keeps USB off (it re-enables on the DISCHARGE_ONLY
     * → IDLE transition the load-drop causes) until the buttons are released. */
    if (loaded && both_held) {
        loaded = false;                      /* re-load on next both-up */
        for (uint8_t i = 0; i < 4; i++) {
            c->lamp_level[i] = 0;
            set_led_current(0, lamp_led[i]); /* all four back-to-back, no UART between */
        }
        c->usb_force_off = true;
        disable_usb_boost();                 /* boost EN + both AP2151 load switches, still no UART */
        send_string("LOAD_DUMP\r\n");        /* scope/telemetry marker */
    }
}
#endif

/* =========================================================================
 * LED BAR DISPLAY — UI policy layer
 * =========================================================================
 *
 * Two front-panel 5-segment bar graphs (driven by the multiplexer in
 * SPCBoardAPI.c — update_led_display()):
 *
 *   LED_BAR_1 → battery state-of-charge fuel gauge (fills DISP_LED1→5)
 *   LED_BAR_2 → solar panel output power           (fills DISP_LED5→1)
 *
 * The two bars are wired mirror-imaged on the PCB, so bar 2 fills from the
 * opposite segment to keep both gauges reading "up" in the same direction.
 *
 * Each bar shows a 0..5 level from the thresholds in hw_config.h. A bar instead
 * flashes ALL five segments when its source is missing:
 *   - battery bar blinks when no cell is present (V_bat < UI_BAT_PRESENT_MV),
 *   - panel bar blinks when there's no usable sun (flag_has_sun clear — this
 *     already folds in the dusk "panel floating but delivering <200 mW" case).
 * A present-but-empty battery or a sunny-but-idle panel shows 0 solid segments,
 * not a blink — blink means "source absent", not "source low".
 *
 * Policy only: this computes the two 5-bit segment masks each UI tick and hands
 * them to the HAL via update_led_bar(). The mux (update_led_display()) is what
 * actually lights them and is pumped from the 1 ms SysTick ISR.
 */

/* Level (0..5) → segment mask filling up from DISP_LED1 (bit 0). */
static uint8_t bar_mask_from_low(uint8_t level)
{
    if (level > 5) level = 5;
    return (uint8_t)((1u << level) - 1u);
}

/* Level (0..5) → segment mask filling down from DISP_LED5 (bit 4). */
static uint8_t bar_mask_from_high(uint8_t level)
{
    if (level > 5) level = 5;
    return (uint8_t)(((1u << level) - 1u) << (5 - level));
}

/* Battery voltage → fuel-gauge level 0..5. */
static uint8_t battery_level(const system_ctx_t *c)
{
    uint16_t mv = c->meas.bat_voltage;
    if (mv >= UI_BAT_SEG5_MV) return 5;
    if (mv >= UI_BAT_SEG4_MV) return 4;
    if (mv >= UI_BAT_SEG3_MV) return 3;
    if (mv >= UI_BAT_SEG2_MV) return 2;
    if (mv >= UI_BAT_SEG1_MV) return 1;
    return 0;
}

/* Panel power → bar level 0..5. */
static uint8_t panel_level(const system_ctx_t *c)
{
    int32_t mw = c->meas.panel_power;
    if (mw >= UI_PANEL_SEG5_MW) return 5;
    if (mw >= UI_PANEL_SEG4_MW) return 4;
    if (mw >= UI_PANEL_SEG3_MW) return 3;
    if (mw >= UI_PANEL_SEG2_MW) return 2;
    if (mw >= UI_PANEL_SEG1_MW) return 1;
    return 0;
}

/* Recompute both bars' segment content from the latest measurements. Cheap;
 * called on the UI tick. The mux renders whatever this last stored. */
static void ui_display_update(const system_ctx_t *c)
{
    /* All-on for the first half of each period, dark for the second. */
    bool blink_on = ((time_now() / UI_BLINK_PERIOD_MS) & 1u) == 0u;

    uint8_t bar1 = (c->meas.bat_voltage < UI_BAT_PRESENT_MV)
                   ? (blink_on ? 0x1Fu : 0x00u)
                   : bar_mask_from_low(battery_level(c));

    uint8_t bar2 = (!c->flag_has_sun.value)
                   ? (blink_on ? 0x1Fu : 0x00u)
                   : bar_mask_from_high(panel_level(c));

    update_led_bar(bar1, LED_BAR_1);
    update_led_bar(bar2, LED_BAR_2);
}

/*
 * Power-on sweep: light one more segment on each bar every UI_BOOT_ANIM_STEP_MS
 * until all five are on. Both bars fill from their "empty" end so the two
 * mirror-imaged gauges sweep toward each other. Blocking — runs once during
 * bring-up. SysTick is already live here (configured in system_init), so it
 * refreshes the mux while we just hold each frame for one step.
 */
static void led_boot_animation(void)
{
    for (uint8_t n = 1; n <= 5; n++) {
        update_led_bar(bar_mask_from_low(n),  LED_BAR_1);
        update_led_bar(bar_mask_from_high(n), LED_BAR_2);

        uint32_t t0 = time_now();
        while ((time_now() - t0) < UI_BOOT_ANIM_STEP_MS) {
            /* hold this frame; the SysTick ISR drives the multiplexer */
        }
    }
}

/* =========================================================================
 * HardFault_Handler — Cortex-M0+ fault trap with UART post-mortem
 * =========================================================================
 *
 * The SDK startup file declares HardFault_Handler as a weak alias to
 * Default_Handler (an unconditional while(1)). Without this override, any
 * fault — bad PC, unaligned access, executing from unmapped memory, stack
 * overflow into invalid regions — would silently freeze the MCU and look
 * identical to a hung main loop.
 *
 * On entry to the exception, the CPU has already pushed an 8-word frame
 * (R0, R1, R2, R3, R12, LR, PC, xPSR) onto whichever stack was active.
 * Bit 2 of EXC_RETURN (= the LR value at exception entry) selects MSP/PSP.
 * Cortex-M0+ has no CFSR/HFSR/MMFAR/BFAR — the stacked PC and LR are the
 * primary forensics.
 *
 * The naked entry stub picks the right stack pointer and tail-calls the C
 * handler, which prints registers via blocking UART writes (no interrupts,
 * no snprintf — minimal dependencies in case the heap or BSS is corrupt).
 */
static void hf_putc(char c)
{
    DL_UART_Main_transmitDataBlocking(UART_0_INST, (uint8_t)c);
}

static void hf_puts(const char *s)
{
    while (*s) hf_putc(*s++);
}

static void hf_puthex32(uint32_t v)
{
    static const char hex[] = "0123456789ABCDEF";
    for (int i = 7; i >= 0; --i) {
        hf_putc(hex[(v >> (i * 4)) & 0xF]);
    }
}

void HardFault_HandlerC(uint32_t *stack, uint32_t exc_return)
{
    hf_puts("\r\n!! HARDFAULT !!\r\n");
    hf_puts("PC  = 0x"); hf_puthex32(stack[6]); hf_puts("\r\n");
    hf_puts("LR  = 0x"); hf_puthex32(stack[5]); hf_puts("\r\n");
    hf_puts("PSR = 0x"); hf_puthex32(stack[7]); hf_puts("\r\n");
    hf_puts("R0  = 0x"); hf_puthex32(stack[0]); hf_puts("\r\n");
    hf_puts("R1  = 0x"); hf_puthex32(stack[1]); hf_puts("\r\n");
    hf_puts("R2  = 0x"); hf_puthex32(stack[2]); hf_puts("\r\n");
    hf_puts("R3  = 0x"); hf_puthex32(stack[3]); hf_puts("\r\n");
    hf_puts("R12 = 0x"); hf_puthex32(stack[4]); hf_puts("\r\n");
    hf_puts("EXC = 0x"); hf_puthex32(exc_return); hf_puts("\r\n");
    hf_puts("SP  = 0x"); hf_puthex32((uint32_t)stack); hf_puts("\r\n");
    hf_puts("frozen.\r\n");

    while (1) {
        __asm volatile ("wfi");
    }
}

__attribute__((naked))
void HardFault_Handler(void)
{
    __asm volatile (
        "movs r0, #4         \n"
        "mov  r1, lr         \n"
        "tst  r0, r1         \n"   /* EXC_RETURN bit 2: 0=MSP, 1=PSP */
        "bne  1f             \n"
        "mrs  r0, msp        \n"
        "b    2f             \n"
        "1:                  \n"
        "mrs  r0, psp        \n"
        "2:                  \n"
        "mov  r1, lr         \n"   /* pass EXC_RETURN as second arg */
        "ldr  r2, =HardFault_HandlerC\n"
        "bx   r2             \n"
    );
}

/* =========================================================================
 * DEEP SLEEP — STANDBY0 with button + periodic-check wake
 * =========================================================================
 *
 * Entered from the main loop when energy_mode arms idle_sleep_pending
 * (IDLE or SAFE_MODE unchanged for IDLE_SLEEP_TIMEOUT_MS, no lamp lit, no
 * fault latched). The SysConfig power policy is STANDBY0, so __WFI() here
 * gates everything but the LFCLK domain: CPU, SysTick, ADC, UART and all
 * PWM timers stop; GPIO levels and peripheral registers are retained; the
 * RTC and TIMG8 (both PD0) keep running off LFCLK.
 *
 * What sleep changes relative to the state it was entered from:
 *   - LED boost rail OFF. Mandatory, not just savings: TIMA0/TIMA1 freeze
 *     in STANDBY, and a LEDCTRL pin frozen low is the runaway max-current
 *     state (bench 2026-06-16). With EN_LED low the pin state is inert.
 *     Only sleeps with all lamps off (gated in energy_mode), so nothing
 *     visible is lost.
 *   - LED bar display blanked (content zeroed + anodes parked).
 *   - Measurement front-end (VBATM_EN + CRT_SNS_EN) gated off between
 *     wake-checks.
 *   - RTC READY interrupt masked: it is not used by any logic and would
 *     wake the core needlessly.
 *   Everything else keeps the entered state's configuration — notably the
 *   USB boost + load switches stay ON in IDLE (the AP2151s cannot restart
 *   into a plugged load — 2026-07 revert — so a USB device plugged during
 *   sleep is powered by hardware immediately and merely *detected* at the
 *   next wake-check).
 *
 * Wake sources (armed only inside this function):
 *   - BTN1/BTN2 edges via GROUP1 → immediate full wake (buttons stay
 *     polled in normal RUN; the NVIC line is enabled only while asleep).
 *   - ADC_LOW_POWER (TIMG8, LFCLK, STANDBY-capable — provisioned in the
 *     syscfg for exactly this) every SLEEP_WAKE_INTERVAL_MS → wake-check:
 *     sense rails up, SLEEP_CHECK_SETTLE_MS of normal SysTick-driven ADC
 *     harvesting, then raw thresholds decide full wake vs re-sleep.
 *
 * Wake-check conditions (raw, single-sample — the pipeline's debounced
 * flags re-verify everything after wake, so a false wake just costs one
 * idle timeout before re-sleeping):
 *   from IDLE:      V_panel > PANEL_MIN_MV (respecting the dusk relock)
 *                   I_dsg   > LOAD_DETECT_MA
 *                   V_bat   < BAT_LOW_MV (cell present: > 500 mV, the
 *                             same disconnected-sense guard fault_mgr uses)
 *   from SAFE_MODE: V_bat   > BAT_SAFE_RECOVER_MV
 *                   (no sun/load wake: SAFE_MODE only exits on recovery,
 *                    so waking for sun would burn the cell all day)
 *
 * Timekeeping: SysTick is stopped across each WFI, so the slept duration
 * is measured on the wake timer and credited to the ms timestamp
 * (timestamp_advance) before SysTick resumes — time_now() stays
 * continuous and every downstream schedule (debounce, relock, tick
 * baselines) survives sleep unaware.
 */

typedef enum {
    SLEEP_WAKE_NONE = 0,
    SLEEP_WAKE_BUTTON,      /* front-panel button edge                     */
    SLEEP_WAKE_SUN,         /* V_panel above has_sun set threshold         */
    SLEEP_WAKE_LOAD,        /* discharge current above load-detect         */
    SLEEP_WAKE_VBAT,        /* battery crossed a threshold the current
                             * state must react to (low in IDLE,
                             * recovered in SAFE_MODE)                     */
} sleep_wake_t;

static const char *sleep_wake_name(sleep_wake_t w)
{
    switch (w) {
        case SLEEP_WAKE_BUTTON: return "BTN";
        case SLEEP_WAKE_SUN:    return "SUN";
        case SLEEP_WAKE_LOAD:   return "LOAD";
        case SLEEP_WAKE_VBAT:   return "VBAT";
        default:                return "???";
    }
}

/* TIMG8 (ADC_LOW_POWER) ISR — the periodic wake source. Reaching the
 * handler at all IS the wake-up; it only has to read-clear the pending
 * ZERO event so the IRQ line releases. The sleep loop learns "the timer
 * fired" from NVIC pending state (read under PRIMASK, before this runs). */
void ADC_LOW_POWER_INST_IRQHandler(void)
{
    (void)DL_TimerG_getPendingInterrupt(ADC_LOW_POWER_INST);
}

static void system_sleep(system_ctx_t *c)
{
    int len = snprintf(uart_evt_buf, sizeof uart_evt_buf,
        "SLEEP: enter (%s) interval=%lu ms @ %lu ms\r\n",
        em_state_name(c->energy_mode),
        (unsigned long)SLEEP_WAKE_INTERVAL_MS,
        (unsigned long)time_now());
    if (len > 0 && len < (int)sizeof uart_evt_buf) send_string(uart_evt_buf);

    /* Drain the TX shifter: printToUART blocks per byte on FIFO space, not
     * on completion — clock-gating the UART mid-byte garbles the tail. */
    while (DL_UART_Main_isBusy(UART_0_INST)) { }

    /* ── Power down what sleep owns ── */
    update_led_bar(0x00, LED_BAR_1);     /* blank content so the SysTick mux */
    update_led_bar(0x00, LED_BAR_2);     /* renders dark during wake-checks  */
    disable_led_bar();
    disable_led_boost();                 /* LEDCTRL freeze hazard — see above */
    disable_measure_sense();
    DL_RTC_disableInterrupt(RTC, DL_RTC_INTERRUPT_READY);

    /* The four PWM timers (TIMG6 buck FB, TIMG7 LED boost, TIMA0/TIMA1
     * LEDCTRL) sit in PD1 and do NOT retain register contents through
     * STANDBY (SysConfig retention warning — everything else we use is
     * either PD0 or has retention). Snapshot them once here; restored on
     * full wake. Harmless while asleep: the buck is disabled (BUCK_DIS,
     * GPIO — retained) and the LED rail is off, so blank timers drive
     * nothing. */
    SYSCFG_DL_saveConfiguration();

    /* ── Arm wake sources ── */
    gButtonWakeFlag = false;
    DL_TimerG_stopCounter(ADC_LOW_POWER_INST);
    DL_TimerG_setLoadValue(ADC_LOW_POWER_INST, (uint32_t)SLEEP_TIMER_LOAD_COUNTS);
    NVIC_ClearPendingIRQ(ADC_LOW_POWER_INST_INT_IRQN);
    NVIC_EnableIRQ(ADC_LOW_POWER_INST_INT_IRQN);
    NVIC_ClearPendingIRQ(BTNS_INT_IRQN);
    NVIC_EnableIRQ(BTNS_INT_IRQN);

    sleep_wake_t wake = SLEEP_WAKE_NONE;
    uint32_t slept_ms = 0;
    uint32_t checks   = 0;

    while (wake == SLEEP_WAKE_NONE) {

        /* ── STANDBY window ──
         * PRIMASK closes the classic lost-wakeup race: a button edge
         * between the flag check and WFI pends in the NVIC (ISRs can't
         * run) and makes WFI fall straight through. SysTick must be
         * stopped AND its pended exception cleared first, or a tick that
         * fired in the last millisecond aborts every entry attempt. */
        __disable_irq();
        if (!gButtonWakeFlag) {
            DL_SYSTICK_disable();
            SCB->ICSR = SCB_ICSR_PENDSTCLR_Msk;

            /* Fresh full interval each pass (down-counter: LOAD → 0). */
            DL_TimerG_setTimerCount(ADC_LOW_POWER_INST,
                                    (uint32_t)SLEEP_TIMER_LOAD_COUNTS);
            DL_TimerG_startCounter(ADC_LOW_POWER_INST);

            __WFI();   /* STANDBY0 until an NVIC-enabled interrupt pends */

            DL_TimerG_stopCounter(ADC_LOW_POWER_INST);

            /* Credit the slept wall-clock time from the LFCLK timer while
             * PRIMASK still blocks the SysTick increment. If the ZERO
             * event is what woke us the counter has already reloaded, so
             * the pending IRQ means "one full interval plus whatever ran
             * off the fresh reload"; otherwise (button/spurious) the
             * elapsed counts are LOAD − current. */
            uint32_t cnt = DL_TimerG_getTimerCount(ADC_LOW_POWER_INST);
            if (cnt > SLEEP_TIMER_LOAD_COUNTS)      /* defensive: down-counter */
                cnt = SLEEP_TIMER_LOAD_COUNTS;      /* can't exceed LOAD       */
            uint32_t credit = ((SLEEP_TIMER_LOAD_COUNTS - cnt) * 1000UL)
                              / SLEEP_TIMER_TICK_HZ;
            if (NVIC_GetPendingIRQ(ADC_LOW_POWER_INST_INT_IRQN))
                credit += SLEEP_WAKE_INTERVAL_MS;
            timestamp_advance(credit);
            slept_ms += credit;

            DL_SYSTICK_enable();
        }
        __enable_irq();   /* pended ISRs (button edge, wake timer) run now */

        if (gButtonWakeFlag) {
            wake = SLEEP_WAKE_BUTTON;
            break;
        }

        /* ── Wake-check ──
         * Runs on the periodic tick and on any spurious wake (a stale ADC
         * completion at entry, UART RX noise, USB_FLT edge) — checking
         * costs ~60 ms and re-sleeping is always safe. SysTick is live
         * again: its 10 ms ADC tick restarts conversions and harvests
         * fresh Adc*Result sets into the (still valid — samples only ever
         * land while awake with sense on) moving-average window. */
        checks++;
        enable_measure_sense();

        uint32_t t0 = time_now();
        while ((time_now() - t0) < SLEEP_CHECK_SETTLE_MS) {
            if (gButtonWakeFlag) break;      /* press mid-check */
        }
        if (gButtonWakeFlag) {
            wake = SLEEP_WAKE_BUTTON;
            break;
        }

        uint16_t v_bat = get_battery_voltage_now();

        if (c->energy_mode == EM_SAFE_MODE) {
            /* Only battery recovery matters: eval_safe_mode() exits on
             * nothing else, so waking for sun/load would spin awake all
             * day while the latch holds (see SAFE_MODE recovery note). */
            if (v_bat > BAT_SAFE_RECOVER_MV)
                wake = SLEEP_WAKE_VBAT;
        } else {
            uint16_t v_panel = get_input_voltage_now();
            uint16_t i_dsg   = get_discharge_current_now();

            if (v_panel > PANEL_MIN_MV && time_now() >= c->has_sun_relock_ms)
                wake = SLEEP_WAKE_SUN;
            else if (i_dsg > LOAD_DETECT_MA)
                wake = SLEEP_WAKE_LOAD;
            else if (v_bat > 500 && v_bat < BAT_LOW_MV)
                wake = SLEEP_WAKE_VBAT;   /* wake to shed via SAFE_MODE —
                                           * the USB boost is still up and
                                           * bleeding the cell in IDLE   */
        }

        if (wake == SLEEP_WAKE_NONE)
            disable_measure_sense();
    }

    /* ── Full wake: disarm and restore ── */
    NVIC_DisableIRQ(ADC_LOW_POWER_INST_INT_IRQN);
    NVIC_ClearPendingIRQ(ADC_LOW_POWER_INST_INT_IRQN);
    NVIC_DisableIRQ(BTNS_INT_IRQN);
    NVIC_ClearPendingIRQ(BTNS_INT_IRQN);
    DL_TimerG_stopCounter(ADC_LOW_POWER_INST);
    DL_RTC_enableInterrupt(RTC, DL_RTC_INTERRUPT_READY);

    enable_measure_sense();
    refresh_vbatm_sense();   /* clean re-lock edge for a hot-plugged cell */

    /* Bring the PD1 PWM timers back from the dead (see save at entry),
     * then re-commit the application's output values through the normal
     * HAL paths — each one stop/sets/starts its timer, so all four come
     * back running regardless of restore's counter semantics. Order
     * matters: LEDCTRL references must be sane BEFORE the entry actions
     * below re-enable the LED boost rail, or the lamps would flash at an
     * undefined current for a tick. */
    SYSCFG_DL_restoreConfiguration();
    set_buck_pwm(c->pwm);                    /* parked at PWM_MIN_DUTY here */
    set_led_voltage(LED_BOOST_TARGET_MV);
    for (uint8_t i = 0; i < 4; i++)
        set_led_current(lamp_level_ma[c->lamp_level[i]], lamp_led[i]);

    /* Re-apply the current state's entry actions: IDLE gets its detection
     * rails (LED boost included) back, SAFE_MODE stays shed, and the
     * state's inactivity anchor restarts so an unfounded wake re-sleeps
     * after one timeout. The bar graphs repopulate on the next UI tick. */
    energy_mode_reapply_entry(c);

    len = snprintf(uart_evt_buf, sizeof uart_evt_buf,
        "SLEEP: wake=%s slept=%lu ms checks=%lu @ %lu ms\r\n",
        sleep_wake_name(wake),
        (unsigned long)slept_ms,
        (unsigned long)checks,
        (unsigned long)time_now());
    if (len > 0 && len < (int)sizeof uart_evt_buf) send_string(uart_evt_buf);
}

/* =========================================================================
 * SysTick ISR — fires every 1 ms
 * =========================================================================
 *
 * Responsibilities:
 *   1. Increment the millisecond timestamp (consumed by time_now())
 *   2. Kick ADC conversions every TICK_ADC_MS (10 ms) so the moving-
 *      average filter in SPCBoardAPI.c stays fed
 *
 * No heavy work here — the pipeline runs in the main loop.
 */
void SysTick_Handler(void)
{
    update_timestamp();

    static uint32_t adc_divider = 0;
    adc_divider++;
    if (adc_divider >= TICK_ADC_MS) {
        adc_divider = 0;
        read_adc_values();   /* harvest completed conversions into moving average */
        adc_read_step();     /* kick off next conversion pair */
    }

    /* Service the LED-bar multiplexer here, not in the main loop: the mux must
     * alternate the two bars on a steady ~4 ms cadence, but the foreground loop
     * stalls for tens of ms inside the blocking UART telemetry writes (and any
     * other blocking work), which froze one bar lit and the other dark long
     * enough to flicker visibly. Run from the 1 ms tick (self-rate-limited to
     * 4 ms inside) so refresh is immune to whatever the foreground is doing.
     * Content (leds[]) is still produced in the foreground via update_led_bar();
     * single-byte reads here are atomic on Cortex-M0+, so no lock is needed. */
    update_led_display();
}

/* =========================================================================
 * main
 * ========================================================================= */
int main(void)
{
    /* ────────────────────────────────────────────────────────────────────
     * SYS_INIT — hardware setup, all outputs disabled
     * ──────────────────────────────────────────────────────────────────── */

    system_init();     /* SYSCFG, ADC, PWM, UART, RTC, sensing rails */

    /*
     * SysConfig leaves the buck timer's capture-compare register at whatever
     * value the .syscfg file specified at reset. Force it to the known-safe
     * minimum-duty value before any code path can release BUCK_DIS, so the
     * first transition into a charging state cannot expose the inductor to
     * a random (potentially near-100%) duty between enable_input_buck() and
     * the first apply_pwm() at step 8.
     */
    set_buck_pwm(PWM_MIN_DUTY);

    /*
     * BUCK_DIS has no initialValue in SPC_20.syscfg, so SysConfig leaves the
     * pin CLEARED at reset — which is the *enabled* polarity for the buck
     * controller. The FSM boots in EM_IDLE without a transition, so
     * enter_idle() (which would call disable_input_buck()) never runs at
     * startup. Without this explicit assert, the buck IC is awake and
     * switching at PWM_MIN_DUTY from the moment system_init() returns; even
     * at ~0.25% on-time the minimum-on-time pulses pull enough average
     * current through the inductor to collapse a connected solar panel
     * (observed: 10 V OC → 4.2 V under buck loading, MCU stuck in EM_IDLE).
     */
    disable_input_buck();

    timer_init();      /* SysTick @ 1 ms */
    buttons_init();

    ctx_init(&ctx);    /* zero context, safe defaults, SYS_INIT state */

    /*
     * Enable the battery switch so we can read V_bat from the very first
     * ADC sample. The battery is a passive element — connecting it doesn't
     * push current anywhere unless the buck or output switches are also on
     * (which they aren't at this point).
     */
    enable_battery_switch();

    /* OUTPUT_EN + USB boost + LED boost on at boot so I_DISCHARGE can sense
     * a load (USB-connector loads need the MIC2876 producing 5V; LED-output
     * lamps need the TPS61088 producing the ~11.3V rail AND a non-zero
     * LEDCTRL PWM reference so the per-channel current sources are armed)
     * and let eval_idle leave EM_IDLE. enter_idle() is never called at
     * startup (FSM boots in EM_IDLE with no transition), so its enables
     * must be mirrored here.
     *
     * Bench bringup: 150 mA per LED channel (lands on the 145 mA / 155 mA
     * LUT bins). Far below the 350 mA per-channel hardware max called out
     * in the schematic. */
    enable_output_switch();
    enable_usb_boost();
    enable_led_boost();

    /* ADC warm-up before any voltage-targeting PWM op. set_led_voltage() →
     * set_pwm_duty_cycle() → scale_duty_cycle() → get_vdd(), which reads
     * ADC.Adc0Result[6]; that field is 0 until the first conversion lands.
     * Without this loop scale_duty_cycle ends up with 3300/0 and clamps to
     * an out-of-period CC value — PB15 parks at its init-low state and the
     * TPS61088 only regulates off its passive feedback divider, well below
     * the LED-string dropout. Mirrors the 1000-iter warm-up in V2.5.5/main.c. */
    for (int i = 0; i < 1000; i++) {
        read_adc_values();
        delay_cycles(400);
    }

    set_led_voltage(LED_BOOST_TARGET_MV);
    /* Arm each controllable lamp to its boot brightness (all four full by
     * default — see ctx_init). The buttons tap-toggle / hold-dim these at
     * runtime via lamp_buttons_update(): BTN1 -> lamps 1&2, BTN2 -> lamps 3&4. */
    for (uint8_t i = 0; i < 4; i++)
        set_led_current(lamp_level_ma[ctx.lamp_level[i]], lamp_led[i]);

    /* Bring up the front-panel bar graphs: blank, then a power-on segment
     * sweep so the operator sees the firmware came alive before the gauges
     * start tracking battery / panel power in the main loop. */
    led_display_init();
    led_boot_animation();

    /* ────────────────────────────────────────────────────────────────────
     * SYS_INIT → SYS_RUN
     * ──────────────────────────────────────────────────────────────────── */
    ctx.system_state = SYS_RUN;

    /*
     * Anchor the idle-sleep window at SYS_RUN entry. ctx_init() zeroes the
     * struct so idle_start_ms = 0; enter_idle() would set it but only fires
     * on a transition, and the FSM starts in EM_IDLE — so booting dark with
     * no transition leaves the unsigned (time_now() - 0) comparison ticking
     * from the zero epoch and trips IDLE_SLEEP_TIMEOUT_MS at exactly 2 min,
     * dropping the MCU into __WFI() and killing any attached JTAG session.
     */
    ctx.idle_start_ms = time_now();

    /* Send boot banner before anything else so a fresh terminal
     * sees the reset. The periodic log is now self-describing
     * (each value is prefixed with its label), so no column header. */
    log_boot_banner();

    /* Timing baselines */
    uint32_t last_main   = time_now();
    uint32_t last_log    = time_now();
    uint32_t last_button = time_now();
    uint32_t last_vbatm  = time_now();

    /* ────────────────────────────────────────────────────────────────────
     * MAIN LOOP — runs forever inside SYS_RUN
     * ──────────────────────────────────────────────────────────────────── */
    while (1       ) {
        uint32_t now = time_now();

        /* ── 20 ms tick: button polling ──
         * Runs at TICK_BUTTON_MS. update_buttons() updates internal
         * debounce state. UI_MGR (not yet implemented) would read the
         * results and drive the LED display. */
        if ((now - last_button) >= TICK_BUTTON_MS) {
            last_button = now;
            update_buttons();
#if LOAD_DUMP_TEST
            load_dump_test(&ctx);        /* both-button hold = load dump; normal lamp buttons disabled */
#else
            lamp_buttons_update(&ctx);   /* BTN1 -> lamps 1&2, BTN2 -> lamps 3&4 */
#endif
        }

        /* ── 50 ms tick: the deterministic pipeline ──
         * This is the core of the firmware. Every module runs exactly
         * once per tick, in strict order, reading fields written by
         * earlier steps and writing fields read by later steps. */
        if ((now - last_main) >= TICK_MAIN_MS) {
            last_main = now;

            /* Snapshot FSM states so we can log any region that
             * transitioned during this tick. Cheap (3 enum copies). */
            energy_mode_state_t em_old   = ctx.energy_mode;
            charger_state_t     chg_old  = ctx.charger.state;
            mppt_state_t        mppt_old = ctx.mppt.state;

            /* Step 1: read all sensors → ctx->meas */
            measurements_update(&ctx);

            /* Step 2: debounce / hysteresis → flags */
            flags_update(&ctx);

            /* Step 3: power budget → i_buck_max, allowed_chg */
            power_budget_update(&ctx);

            /* Step 4: fault detection/recovery → ctx->fault */
            fault_mgr_update(&ctx);

            /* Step 5: energy mode FSM → hardware enables */
            energy_mode_update(&ctx);

            /* Step 6: MPPT FSM → may write ctx->pwm */
            mppt_update(&ctx);

            /* Step 7: charger FSM → may write ctx->pwm */
            charger_update(&ctx);

            /* Step 8: commit ctx->pwm to the buck timer */
            apply_pwm(&ctx);

            /* One-line UART log per region whose state changed. */
            log_state_transitions(now, em_old, chg_old, mppt_old);

            /* Refresh the bar-graph content from this tick's measurements
             * (battery SoC + panel power, or a blink if either source is
             * absent). The mux pumped at the top of the loop renders it. */
            ui_display_update(&ctx);
        }

        /* ── 1 s tick: UART diagnostic logging ── */
        if ((now - last_log) >= TICK_LOG_MS) {
            last_log = now;
            log_measurements();
        }

        /* ── Slow tick: re-assert the battery-sense enable ──
         * VBATM_EN gates the V_bat sense divider and is otherwise asserted
         * only once, at boot, before a cell may be present. A battery
         * hot-plugged (or re-plugged) afterwards never gets a fresh enable
         * edge, so the sense path stays latched in its boot-time
         * (no-battery) state: the cell never "shows up", and a removed cell
         * never clears. Re-pulsing the enable re-locks the divider to
         * whatever is present now. Only the sense divider is touched (not
         * the battery↔bus FET), so the power path is undisturbed. */
        if ((now - last_vbatm) >= VBATM_REFRESH_MS) {
            last_vbatm = now;
            refresh_vbatm_sense();
        }

        /* ── Deep sleep ──
         * energy_mode arms idle_sleep_pending after IDLE_SLEEP_TIMEOUT_MS
         * of unchanged IDLE or SAFE_MODE (lamps off, no fault latched).
         * system_sleep() holds the MCU in STANDBY0 — waking briefly every
         * SLEEP_WAKE_INTERVAL_MS to sniff for sun / load / battery
         * movement, instantly on a button edge — and only returns on a
         * real wake condition, with the current state's hardware enables
         * re-applied and time_now() credited for the slept duration. The
         * next pipeline tick (which fires immediately: the tick baselines
         * below are now far in the past) re-evaluates everything through
         * the normal debounced path. */
        if (ctx.idle_sleep_pending) {
            ctx.idle_sleep_pending = false;
            system_sleep(&ctx);
        }
    }
}
