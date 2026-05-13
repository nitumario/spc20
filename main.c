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

/* Separate buffers: transition events must not clobber in-flight telemetry. */
#define UART_BUF_SIZE 512
static char uart_buf[UART_BUF_SIZE];      /* per-tick telemetry */
static char uart_evt_buf[128];            /* state-transition events */

/* =========================================================================
 * UART LOGGING — space-separated, header on boot, data every TICK_LOG_MS.
 * ========================================================================= */

static void send_string(const char *str)
{
    printToUART((char *)str, '\0');
}

/* Emitted once on boot so a freshly attached terminal can see the reset. */
static void log_boot_banner(void)
{
    send_string("\r\n"
                "==============================================\r\n"
                " SPC_20 Solar Charge Controller - boot v0.19\r\n"
                "==============================================\r\n");
}

static void log_header(void)
{
    send_string(
        "ms "
        "Vbat Vchg Vout Vpanel Vusb1 Vusb2 "
        "Ipanel Ichg Idsg Ppanel Ibat_net "
        "Tbat Tboard "
        "bat_low has_sun has_load temp_ok p_limited bat_full "
        "i_buck_max allowed_chg "
        "EM CHG MPPT pwm fault flt_hist\r\n"
    );
}

/* Emit one line per FSM region that changed this tick. grep-able plain text. */
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

/* Per-tick telemetry: uptime, voltages, currents, temps, flags, states, pwm, faults. */
static void log_measurements(void)
{
    measurements_t *m = &ctx.meas;
    int len = snprintf(uart_buf, UART_BUF_SIZE,
        "%lu %u %u %u %u %u %u %u %d %u %ld %ld %d %d %u %u %u %u %u %u %u %u %s %s %s %u %04X %04X\r\n",
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
        ctx.pwm, ctx.fault.code, ctx.fault.history);
    if (len > 0 && len < UART_BUF_SIZE) send_string(uart_buf);
}

/* TEMP: raw VCHG_M bring-up probe. Remove once VCHG sense is verified. */
static void log_vchg_debug(void)
{
    int len = snprintf(uart_evt_buf, sizeof uart_evt_buf,
        "VCHG_DBG raw_MEM9=%u raw_VPANEL=%u Vchg=%u mV em=%s chg=%s pwm=%u\r\n",
        (unsigned)DL_ADC12_getMemResult(ADC0_INST, 9),
        (unsigned)DL_ADC12_getMemResult(ADC1_INST, 0),
        (unsigned)get_charge_voltage(),
        em_state_name(ctx.energy_mode),
        chg_state_name(ctx.charger.state),
        (unsigned)ctx.pwm);
    if (len > 0 && len < (int)sizeof uart_evt_buf) send_string(uart_evt_buf);
}

/* =========================================================================
 * PIPELINE STEP 8: apply_pwm
 * =========================================================================
 * Sole writer of the buck timer register. Defense-in-depth clamp on ctx->pwm.
 */
static void apply_pwm(system_ctx_t *c)
{
    uint16_t pwm = c->pwm;

    if (pwm < PWM_MAX_DUTY) pwm = PWM_MAX_DUTY;   /* floor: highest V_buck */
    if (pwm > PWM_MIN_DUTY) pwm = PWM_MIN_DUTY;   /* ceil:  off (in-LUT)   */

    set_buck_pwm(pwm);
}

/* =========================================================================
 * HardFault_Handler — overrides the SDK weak Default_Handler (infinite loop).
 * CPU pushes 8-word frame (R0-R3, R12, LR, PC, xPSR) onto active stack;
 * naked stub selects MSP/PSP via EXC_RETURN bit 2 and tail-calls the C handler.
 * ========================================================================= */
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
        /* wfi (Wait For Interrupt): halts the CPU clock until any interrupt
         * fires. No interrupt will resume meaningful execution here, so this
         * keeps the core frozen while burning minimal power. */
        __asm volatile ("wfi");
    }
}

__attribute__((naked))
void HardFault_Handler(void)
{
    __asm volatile (
        /* Bit 2 of EXC_RETURN (the value LR holds on exception entry) tells
         * us which stack was active before the fault:
         *   bit2 == 0 → MSP (main stack, used in handler mode or bare-metal)
         *   bit2 == 1 → PSP (process stack, used by an RTOS thread)
         * We need that stack pointer so the C handler can read the 8-word
         * auto-saved frame (R0-R3, R12, LR, PC, xPSR). */
        "movs r0, #4         \n"   /* r0 = 0x4, mask to isolate EXC_RETURN bit 2 */
        "mov  r1, lr         \n"   /* r1 = LR = EXC_RETURN value (0xFFFFFFF9 / ...FD / ...ED) */
        "tst  r0, r1         \n"   /* TST: AND r0,r1 and set flags; Z=1 if bit2==0 (MSP), Z=0 if bit2==1 (PSP) */
        "bne  1f             \n"   /* branch if Z==0 (bit2 set) → fault was on PSP */
        "mrs  r0, msp        \n"   /* MSP path: r0 = Main Stack Pointer → points at the saved frame */
        "b    2f             \n"   /* skip PSP path */
        "1:                  \n"   /* PSP path */
        "mrs  r0, psp        \n"   /* r0 = Process Stack Pointer → points at the saved frame */
        "2:                  \n"   /* common tail: r0 = stack frame pointer (1st arg to C handler) */
        "mov  r1, lr         \n"   /* r1 = EXC_RETURN (2nd arg): lets C handler know which stack was used */
        "ldr  r2, =HardFault_HandlerC\n" /* r2 = address of the C handler (PC-relative literal pool load) */
        "bx   r2             \n"   /* tail-call C handler; naked means no prologue/epilogue to corrupt the frame */
    );
}

/* =========================================================================
 * SysTick ISR — 1 ms: increment timestamp, kick ADC every TICK_ADC_MS.
 * ========================================================================= */
void SysTick_Handler(void)
{
    update_timestamp();

    static uint32_t adc_divider = 0;
    adc_divider++;
    if (adc_divider >= TICK_ADC_MS) {
        adc_divider = 0;
        read_adc_values();
        adc_read_step();
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

    /* Force safe PWM before any BUCK_DIS release — SysConfig may leave a random value. */
    set_buck_pwm(PWM_MIN_DUTY);

    timer_init();      /* SysTick @ 1 ms */
    buttons_init();

    ctx_init(&ctx);    /* zero context, safe defaults, SYS_INIT state */

    /* Connect battery so V_bat is available from the first ADC sample.
     * Battery is passive; no current flows without buck or output switch. */
    enable_battery_switch();

    /* ────────────────────────────────────────────────────────────────────
     * SYS_INIT → SYS_RUN
     * ──────────────────────────────────────────────────────────────────── */
    ctx.system_state = SYS_RUN;

    /* Anchor idle timer here; ctx_init() zeros it, causing early WFI if not set
     * (FSM starts in EM_IDLE with no transition to trigger enter_idle). */
    ctx.idle_start_ms = time_now();

    /* Boot banner then column header. */
    log_boot_banner();
    log_header();

    /* Timing baselines */
    uint32_t last_main   = time_now();
    uint32_t last_log    = time_now();
    uint32_t last_button = time_now();

    /* ────────────────────────────────────────────────────────────────────
     * MAIN LOOP — runs forever inside SYS_RUN
     * ──────────────────────────────────────────────────────────────────── */
    while (1       ) {
        uint32_t now = time_now();

        /* ── 20 ms: button polling ── */
        if ((now - last_button) >= TICK_BUTTON_MS) {
            last_button = now;
            update_buttons();
        }

        /* ── 50 ms: deterministic pipeline ── */
        if ((now - last_main) >= TICK_MAIN_MS) {
            last_main = now;

            /* Snapshot for transition logging. */
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

            log_state_transitions(now, em_old, chg_old, mppt_old);
        }

        /* ── 1 s: UART diagnostic logging ── */
        if ((now - last_log) >= TICK_LOG_MS) {
            last_log = now;
            log_measurements();
            log_vchg_debug();   /* TEMP: raw VCHG_M bring-up probe */
        }

        /* ── Idle sleep ── */
        if (ctx.idle_sleep_pending) {
            ctx.idle_sleep_pending = false;
            ctx.idle_start_ms = time_now();
            disable_led_bar();
            /* __WFI() emits a single "wfi" (Wait For Interrupt) instruction.
             * The CPU clock gates until the next SysTick (≤1 ms), so the
             * super-loop resumes on the next tick with no delay and no missed
             * events — SysTick and all other ISRs remain active during WFI. */
            __WFI();
        }
    }
}
