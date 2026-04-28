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

/* ── UART transmit buffer ── */
#define UART_BUF_SIZE 512
static char uart_buf[UART_BUF_SIZE];

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
                " SPC_20 Solar Charge Controller - boot v0.17\r\n"
                "==============================================\r\n");
}

static void log_header(void)
{
    send_string(
        "ms\t"
        "Vbat\tVchg\tVout\tVpanel\tVusb1\tVusb2\t"
        "Ipanel\tIchg\tIdsg\tPpanel\tIbat_net\t"
        "Tbat\tTboard\t"
        "bat_low\thas_sun\thas_load\ttemp_ok\tp_limited\tbat_full\t"
        "i_buck_max\tallowed_chg\t"
        "EM\tCHG\tMPPT\tpwm\tfault\tflt_hist\r\n"
    );
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
        int len = snprintf(uart_buf, UART_BUF_SIZE,
            "EM: %s -> %s @ %lu ms\r\n",
            em_state_name(em_old), em_state_name(ctx.energy_mode),
            (unsigned long)now);
        if (len > 0 && len < UART_BUF_SIZE) send_string(uart_buf);
    }
    if (ctx.charger.state != chg_old) {
        int len = snprintf(uart_buf, UART_BUF_SIZE,
            "CHG: %s -> %s @ %lu ms\r\n",
            chg_state_name(chg_old), chg_state_name(ctx.charger.state),
            (unsigned long)now);
        if (len > 0 && len < UART_BUF_SIZE) send_string(uart_buf);
    }
    if (ctx.mppt.state != mppt_old) {
        int len = snprintf(uart_buf, UART_BUF_SIZE,
            "MPPT: %s -> %s @ %lu ms\r\n",
            mppt_state_name(mppt_old), mppt_state_name(ctx.mppt.state),
            (unsigned long)now);
        if (len > 0 && len < UART_BUF_SIZE) send_string(uart_buf);
    }
}

static void log_measurements(void)
{
    measurements_t *m = &ctx.meas;

    int len = snprintf(uart_buf, UART_BUF_SIZE,
        "%lu\t"
        "%u\t%u\t%u\t%u\t%u\t%u\t"
        "%u\t%d\t%u\t%ld\t%ld\t"
        "%d\t%d\t"
        "%u\t%u\t%u\t%u\t%u\t%u\t"
        "%u\t%u\t"
        "%s\t%s\t%s\t%u\t0x%04X\t0x%04X\r\n",
        (unsigned long)time_now(),
        m->bat_voltage, m->chg_voltage, m->out_voltage,
        m->panel_voltage, m->usb1_voltage, m->usb2_voltage,
        m->panel_current, (int)m->chg_current, m->dsg_current,
        (long)m->panel_power, (long)m->i_bat_net,
        (int)m->bat_temp, (int)m->board_temp,
        (unsigned)ctx.flag_bat_low.value,
        (unsigned)ctx.flag_has_sun.value,
        (unsigned)ctx.has_load,
        (unsigned)ctx.temp_charge_ok,
        (unsigned)ctx.panel_limited,
        (unsigned)ctx.bat_full,
        ctx.i_buck_max, ctx.allowed_chg,
        em_state_name(ctx.energy_mode),
        chg_state_name(ctx.charger.state),
        mppt_state_name(ctx.mppt.state),
        ctx.pwm,
        ctx.fault.code,
        ctx.fault.history);

    if (len > 0 && len < UART_BUF_SIZE) {
        send_string(uart_buf);
    }
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
     * sees the reset, then the column header for the periodic log. */
    log_boot_banner();
    log_header();

    /* Timing baselines */
    uint32_t last_main   = time_now();
    uint32_t last_log    = time_now();
    uint32_t last_button = time_now();

    /* ────────────────────────────────────────────────────────────────────
     * MAIN LOOP — runs forever inside SYS_RUN
     * ──────────────────────────────────────────────────────────────────── */
    while (1) {
        uint32_t now = time_now();

        /* ── 20 ms tick: button polling ──
         * Runs at TICK_BUTTON_MS. update_buttons() updates internal
         * debounce state. UI_MGR (not yet implemented) would read the
         * results and drive the LED display. */
        if ((now - last_button) >= TICK_BUTTON_MS) {
            last_button = now;
            update_buttons();
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
        }

        /* ── 1 s tick: UART diagnostic logging ── */
        if ((now - last_log) >= TICK_LOG_MS) {
            last_log = now;
            log_measurements();
        }

        /* ── Idle sleep ──
         * energy_mode sets idle_sleep_pending after IDLE_SLEEP_TIMEOUT_MS
         * (2 min) with no sun and no load. Enter low-power stop mode;
         * a GPIO interrupt (button press or load connect) or the RTC
         * will wake the MCU back into this loop.
         *
         * After wake-up, clear the pending flag and reset the idle
         * timer so we get a fresh 2-minute window. The next pipeline
         * tick will re-evaluate energy_mode which will either stay in
         * IDLE (starting a new timeout) or transition out if conditions
         * changed while we slept. */
        if (ctx.idle_sleep_pending) {
            ctx.idle_sleep_pending = false;
            ctx.idle_start_ms = time_now();

            /* Disable peripherals that draw current in sleep */
            disable_led_bar();

            __WFI();  /* Wait For Interrupt — enter low-power mode */

            /* Woken by interrupt — resume. SysTick_Handler resumes
             * timestamp counting automatically. */
        }
    }
}
