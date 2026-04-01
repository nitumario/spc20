/*
 * main.c — System Entry Point, SysTick ISR, Main Loop
 * =====================================================
 *
 * Bringup version: measurements + UART logging only.
 * No state machines, no charger, no MPPT, no fault management.
 *
 * What it does:
 *   1. Initialises hardware (system_init, ctx_init)
 *   2. Starts ADC continuous conversion
 *   3. Every TICK_MAIN_MS (50 ms): reads ADC → populates ctx.meas → updates flags
 *   4. Every TICK_LOG_MS (1000 ms): prints all ctx.meas values over UART
 *
 * The UART output is tab-separated for easy copy-paste into a spreadsheet.
 */

#include "ti_msp_dl_config.h"
#include "SPCBoardAPI.h"
#include "system_types.h"
#include "measurements.h"
#include <stdio.h>
#include <string.h>

/* ── The one context struct ── */
static system_ctx_t ctx;

/* ── UART transmit buffer ── */
#define UART_BUF_SIZE 512
static char uart_buf[UART_BUF_SIZE];

/*
 * send_string — wrapper around printToUART for null-terminated strings.
 * printToUART sends characters until it finds end_char. We use '\0'.
 */
static void send_string(const char *str)
{
    printToUART((char *)str, '\0');
}

/*
 * log_measurements — prints everything in ctx.meas and ctx flags to UART.
 *
 * Format: one header line (sent once), then data lines every TICK_LOG_MS.
 * Tab-separated so it pastes directly into Excel / Google Sheets.
 *
 * Columns:
 *   ms         — system uptime in milliseconds
 *   Vbat       — battery voltage (mV)
 *   Vchg       — buck output voltage (mV)
 *   Vout       — output rail voltage (mV)
 *   Vpanel     — panel input voltage (mV)
 *   Vusb1      — USB port 1 voltage (mV)
 *   Vusb2      — USB port 2 voltage (mV)
 *   Ipanel     — panel current (mA)
 *   Ichg       — charge current (mA, signed)
 *   Idsg       — discharge current (mA)
 *   Ppanel     — panel power (mW)
 *   Ibat_net   — net battery current (mA, signed)
 *   Tbat       — battery temperature (°C)
 *   Tboard     — board temperature (°C)
 *   bat_low    — debounced flag (0/1)
 *   has_sun    — debounced flag (0/1)
 *   has_load   — flag (0/1)
 *   temp_ok    — temp_charge_ok flag (0/1)
 *   p_limited  — panel_limited flag (0/1)
 */
static void log_header(void)
{
    send_string("ms\tVbat\tVchg\tVout\tVpanel\tVusb1\tVusb2\t"
                "Ipanel\tIchg\tIdsg\tPpanel\tIbat_net\t"
                "Tbat\tTboard\t"
                "bat_low\thas_sun\thas_load\ttemp_ok\tp_limited\r\n");
}

static void log_measurements(void)
{
    measurements_t *m = &ctx.meas;

    int len = snprintf(uart_buf, UART_BUF_SIZE,
        "%lu\t%u\t%u\t%u\t%u\t%u\t%u\t"
        "%u\t%d\t%u\t%ld\t%ld\t"
        "%d\t%d\t"
        "%u\t%u\t%u\t%u\t%u\r\n",
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
        (unsigned)ctx.panel_limited);

    if (len > 0 && len < UART_BUF_SIZE) {
        send_string(uart_buf);
    }
}

/* ── Tick timing state ── */
static volatile uint32_t tick_adc    = 0;
static volatile uint32_t tick_main   = 0;
static volatile uint32_t tick_log    = 0;

/* ============================================================================
 * SysTick ISR — fires every 1 ms
 * ============================================================================
 *
 * Only does two things:
 *   1. Increments the millisecond timestamp (for time_now())
 *   2. Kicks off ADC conversions every TICK_ADC_MS
 *
 * No heavy work here — everything else runs in the main loop.
 */
void SysTick_Handler(void)
{
    update_timestamp();

    tick_adc++;
    if (tick_adc >= TICK_ADC_MS) {
        tick_adc = 0;
        adc_read_step();
    }
}

/* ============================================================================
 * main
 * ============================================================================ */
int main(void)
{
    /* ── Hardware init ── */
    system_init();
    timer_init();

    /* ── Context init (all safe defaults) ── */
    ctx_init(&ctx);

    /* ── Enable measurement sensing rails ──
     * Battery voltage sensor and current sense are enabled in system_init().
     * Enable battery switch so we can read V_bat even in bringup.
     */
    enable_battery_switch();

    /* ── Transition to RUN ── */
    ctx.system_state = SYS_RUN;

    /* ── Send UART header ── */
    log_header();

    /* ── Timing baselines ── */
    uint32_t last_main = time_now();
    uint32_t last_log  = time_now();

    /* ── Main loop ── */
    while (1) {
        uint32_t now = time_now();

        /* ── 50 ms tick: measurements + flags ── */
        if ((now - last_main) >= TICK_MAIN_MS) {
            last_main = now;

            /* Pipeline step 1: read all sensors */
            measurements_update(&ctx);

            /* Pipeline step 2: update debounced flags */
            flags_update(&ctx);

            /*
             * Future pipeline steps (not yet implemented):
             *   power_budget_update(&ctx);
             *   fault_mgr_update(&ctx);
             *   energy_mode_update(&ctx);
             *   mppt_update(&ctx);
             *   charger_update(&ctx);
             *   apply_pwm(&ctx);
             */
        }

        /* ── 1 s tick: UART logging ── */
        if ((now - last_log) >= TICK_LOG_MS) {
            last_log = now;
            log_measurements();
        }
    }
}
