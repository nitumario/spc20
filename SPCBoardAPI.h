#ifndef SPCBOARDAPI_H
#define SPCBOARDAPI_H

/*
 * ============================================================================
 * SPC Board API - Hardware Abstraction Layer
 * ----------------------------------------------------------------------------
 * Provides board-level hardware interfaces:
 *  - System initialization
 *  - GPIO power-path control (enable/disable — no toggle)
 *  - ADC measurement and unit conversion
 *  - PWM configuration
 *  - LED display (raw segment/bar primitives; content/policy driven by main.c)
 *  - Button interface (read-only events; policy driven by main.c)
 *  - RTC management
 *  - UART transmit
 *
 * Excluded from HAL (moved to application layer or dropped):
 *  - power_manager(), apply_mppt_perturb_observe_step() → energy_mode / mppt
 *  - handle_button_input() → main.c (lamp_buttons_update)
 *  - handle_ir(), receive_command() → dropped; no IR/command receiver
 *  - handle_uart() → dropped; UART is transmit-only now
 *  - turn_on_outputs(), turn_off_outputs() → energy_mode (calls individual enables)
 *  - display_time(), displayChargeStorage(), etc. → main.c (LED bar gauges)
 *  - toggle_*() → removed; state machine uses explicit enable/disable
 *  - get_*_log() → removed; logging reads ctx->meas directly
 * ============================================================================
 */

#include "ti_msp_dl_config.h"
#include "stdint.h"
#include "hw_config.h"   /* ADC_PANEL_TRACE_DEPTH (panel trace ring) */

/* ============================================================================
 * GLOBAL VARIABLES
 * ============================================================================ */

extern volatile uint8_t leds[2];

void system_init(void);

/* ============================================================================
 * ENUMERATIONS
 * ============================================================================ */

typedef enum {
    LED1,
    LED2,
    LED3,
    LED4,
    LED_NONE
} LED_OUTPUT;

struct LED_MEM {
    uint8_t LED_BAR_1;
    uint8_t LED_BAR_2;
};

extern struct LED_MEM LED_MEMORY;

typedef enum {
    TEMP1,
    TEMP3,
} TEMP_SENSOR;

typedef enum {
    USB1,
    USB2,
} USB_SENSOR;

/* ============================================================================
 * SYSTEM TIMER (SYSTICK)
 * ============================================================================ */

void timer_init(void);
void update_timestamp(void);
uint32_t time_now(void);

/* Credit wall-clock time that passed while SysTick was stopped (STANDBY).
 * Called by the sleep loop with the slept duration measured on the LFCLK
 * wake timer, so time_now()-based scheduling stays continuous across sleep.
 * Call only with the SysTick interrupt masked/stopped (read-modify-write). */
void timestamp_advance(uint32_t ms);

/* ============================================================================
 * GPIO CONTROL FUNCTIONS
 * ============================================================================ */

void enable_led_bar(void);
void disable_led_bar(void);

void enable_led_boost(void);
void disable_led_boost(void);
void enable_input_buck(void);
void disable_input_buck(void);
void enable_usb_boost(void);
void disable_usb_boost(void);

void enable_output_switch(void);
void disable_output_switch(void);
void enable_charge_switch(void);
void disable_charge_switch(void);
void enable_battery_switch(void);
void disable_battery_switch(void);

/* Re-pulse VBATM_EN so the battery-voltage sense divider re-locks to a
 * cell hot-plugged/removed after boot. Call periodically on a slow tick. */
void refresh_vbatm_sense(void);

/* Gate the measurement front-end (VBATM_EN battery-sense divider +
 * CRT_SNS_EN current-sense amps) as a pair. Disabled across STANDBY
 * windows to stop their quiescent draw; every enable provides the fresh
 * rising edge the VBATM sense latch needs (same mechanism as
 * refresh_vbatm_sense). Settle is µs-scale — well inside the sleep
 * check's SLEEP_CHECK_SETTLE_MS. */
void enable_measure_sense(void);
void disable_measure_sense(void);

/* ============================================================================
 * ADC MODULE
 * ============================================================================ */

extern volatile bool gCheckADC1;
extern volatile bool gCheckADC2;

void adc_init(void);
void adc_read_step(void);

extern volatile bool gUSBFaultDetected;

void usb_fault_init(void);
void usb_fault_read(void);

void read_adc_values(void);

/* Measurement conversion functions — all return engineering units (mV, mA, °C) */
int16_t  get_charge_current(void);         /* mA, signed */
uint16_t get_charge_voltage(void);         /* mV */
uint16_t get_input_voltage(void);          /* mV */
int16_t  get_input_current(void);          /* mA, signed */
uint16_t get_discharge_current(void);      /* mA */
uint16_t get_battery_voltage(void);        /* mV */
uint16_t get_output_voltage(void);         /* mV */
uint16_t get_usb_voltage(USB_SENSOR usb);  /* mV */

uint16_t get_led_transistor_voltage(LED_OUTPUT led);  /* mV */
int16_t  get_temperature(TEMP_SENSOR temp_sensor);    /* °C, truncated */

int32_t  get_input_power(void);            /* mW */
uint32_t get_output_power(void);           /* mW */
int32_t  get_power_into_battery(void);     /* mW */

/* Instantaneous (latest-conversion) readings for the sleep wake-check.
 * The get_*() functions above read the 64-sample moving average, which
 * after a STANDBY window still holds samples from previous wake-checks;
 * these read the most recent raw conversion result instead (per-channel
 * HW averaging still applies), so a single fresh harvest is enough to
 * evaluate the coarse wake thresholds. Not for regulation — single-sample
 * noise is fine against thresholds with tens-of-mV / tens-of-mA margins. */
uint16_t get_input_voltage_now(void);      /* V_panel, mV */
uint16_t get_battery_voltage_now(void);    /* V_bat,   mV */
uint16_t get_discharge_current_now(void);  /* I_dsg,   mA */
uint16_t get_charge_voltage_now(void);     /* V_chg,   mV */
int16_t  get_charge_current_now(void);     /* I_chg,   mA, signed */

/* Monotonic count of completed ADC harvests (one per TICK_ADC_MS). Lets a
 * foreground guard running at loop rate tell a NEW conversion from a re-read
 * of the same one — see charger_input_guard(). */
uint32_t adc_sample_seq(void);

/* Post-mortem: last ADC_PANEL_TRACE_DEPTH raw V_panel conversions in mV,
 * oldest first. out[] must hold ADC_PANEL_TRACE_DEPTH entries. */
void     adc_panel_trace_mv(uint16_t *out);

/* ============================================================================
 * PWM MODULE
 * ============================================================================ */

typedef struct {
    GPTIMER_Regs *TIMER;
    uint8_t CC_INDEX;
    uint8_t is_complementary_output;
} PWM_Config;

void     pwm_init(void);
void     set_pwm_duty_cycle(const PWM_Config* pwm_channel, uint16_t duty_cycle);
void     set_buck_pwm(uint16_t pwm_value);
uint16_t lookup_charging_pwm(uint16_t voltage);
uint16_t set_charging_voltage(uint16_t voltage);
/* v0.42: set_buck_pwm() writes the timer only when the value changes; call
 * this after a STANDBY restore so the next write is unconditional. */
void     buck_pwm_cache_invalidate(void);
void     set_led_voltage(uint16_t voltage);
void     set_led_current(uint16_t current, LED_OUTPUT led);

/* ============================================================================
 * LED DISPLAY MODULE
 * ============================================================================ */

typedef enum {
    LED_BAR_1,
    LED_BAR_2
} LED_BAR_ID;

typedef enum {
    LED_DIGIT_1,
    LED_DIGIT_2,
    LED_DIGIT_3,
    LED_DIGIT_4,
} LED_DIGIT_ID;

extern const uint8_t DIGITS[10];
extern const uint8_t CHARACTERS[7];

void led_display_init(void);
void update_led_bar(uint8_t data, LED_BAR_ID led_bar_id);
void update_seven_segment_display(uint8_t data, LED_DIGIT_ID led_digit_id);
void update_led_display(void);   /* drives mux timing; content set by main.c via update_led_bar() */

/* ============================================================================
 * BUTTON INTERFACE
 * ============================================================================ */

typedef struct {
    GPIO_Regs* port;
    uint32_t pin;

    uint8_t state;
    bool wasPressed;
    bool wasReleased;
    bool shortPress;
    bool longPress;

    bool isHeld;

    uint32_t pressStartTime;
    uint32_t holdThreshold;
} Button;

extern volatile bool check_buttons;
extern Button BUTTONS[2];

/* Set by GROUP1_IRQHandler on any BTN1/BTN2 edge. Only meaningful while
 * the sleep loop has the GROUP1 NVIC line enabled (it is disabled in
 * normal RUN, where buttons are polled); the sleep loop clears it at
 * entry and treats it as "user touched the panel → full wake". */
extern volatile bool gButtonWakeFlag;

void buttons_init(void);
bool get_button_state(const Button* btn);
bool is_button_pressed(const Button* btn);
bool is_button_released(const Button* btn);
bool is_button_held(const Button* btn);
void update_buttons(void);

/* ============================================================================
 * RTC MODULE
 * ============================================================================ */

extern volatile bool gRTCReadReady;
extern DL_RTC_Common_Calendar Time;

void start_rtc(void);
void set_time(DL_RTC_Common_Calendar time);
void get_time(DL_RTC_Common_Calendar* time_struct);

/* ============================================================================
 * UART MODULE
 * ============================================================================ */

void uart_init(void);
void printToUART(char* string, char end_char);

/* v0.41: non-blocking transmit path for the telemetry and event lines.
 * uart_write() copies a NUL-terminated string into a software ring (whole
 * string or nothing — a line that does not fit is dropped and counted);
 * uart_pump() moves bytes from the ring into the hardware TX FIFO without
 * ever waiting, and is called once per super-loop pass; uart_flush() drains
 * ring and shifter BLOCKING, for the one place that needs the wire idle
 * (STANDBY entry). printToUART above stays blocking for the HardFault
 * post-mortem, which cannot rely on the loop running. */
void     uart_write(const char *str);
void     uart_pump(void);
void     uart_flush(void);
uint16_t uart_tx_dropped(void);

#endif
