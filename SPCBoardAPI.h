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
 *  - LED display (raw segment/bar primitives; policy handled by UI_MGR)
 *  - Button interface (read-only events; policy handled by UI_MGR)
 *  - RTC management
 *  - UART transmit
 *
 * Excluded from HAL (moved or pending move to application layer):
 *  - power_manager(), apply_mppt_perturb_observe_step() → energy_mode / mppt
 *  - handle_button_input() → UI_MGR
 *  - handle_ir(), receive_command(), handle_uart() → UI_MGR or dropped
 *  - turn_on_outputs(), turn_off_outputs() → energy_mode (calls individual enables)
 *  - display_time(), displayChargeStorage(), etc. → UI_MGR
 *  - toggle_*() → removed; state machine uses explicit enable/disable
 *  - get_*_log() → removed; logging reads ctx->meas directly
 * ============================================================================
 */

#include "ti_msp_dl_config.h"
#include "stdint.h"

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
uint16_t set_charging_voltage(uint16_t voltage);
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
void update_led_display(void);   /* drives mux timing; content set by UI_MGR via update_led_bar() */

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

#endif
