#ifndef SPCBOARDAPI_H
#define SPCBOARDAPI_H

/*
 * ============================================================================
 * SPC Board API - Hardware Abstraction Layer
 * ----------------------------------------------------------------------------
 * This header file provides the interface definitions for all board-level
 * peripherals and system control modules including:
 *  - GPIO control
 *  - ADC measurement and conversion
 *  - PWM configuration
 *  - LED display handling
 *  - Button interface
 *  - RTC management
 *  - UART communication
 *  - Power management
 *  - MPPT control
 *
 * The purpose of this API is to abstract hardware-level operations and
 * provide clean, modular access for application-layer firmware.
 * ============================================================================
 */

// Include necessary configuration files
#include "ti_msp_dl_config.h"
#include "stdint.h"

/* ============================================================================
 * GLOBAL VARIABLES
 * ============================================================================ */

extern volatile uint8_t leds[2];                 // LED brightness/state storage
extern volatile bool receiving_msg_led;          // Indicates IR LED message reception
void system_init(void);                          // Performs full system initialization

/* ============================================================================
 * ENUMERATIONS
 * ============================================================================ */

// Identifies LED output channels
typedef enum {
    LED1,
    LED2,
    LED_NONE
} LED_OUTPUT;

// Stores LED bar memory values for display multiplexing
struct LED_MEM{
    uint8_t LED_BAR_1;
    uint8_t LED_BAR_2;
};

extern struct LED_MEM LED_MEMORY;

// Temperature sensor identifiers
typedef enum {
    TEMP1,
    TEMP3,
} TEMP_SENSOR;

// USB sensor identifiers
typedef enum{
    USB1,
    USB2,
}USB_SENSOR;

/* ============================================================================
 * SYSTEM TIMER (SYSTICK)
 * ============================================================================ */

// Initializes system tick timer
void timer_init(void);

// Updates internal millisecond timestamp
void update_timestamp(void);

// Returns system uptime in milliseconds
uint32_t time_now(void);

/* ============================================================================
 * GPIO CONTROL FUNCTIONS
 * ============================================================================ */

// LED BAR Control
void enable_led_bar(void);
void disable_led_bar(void);

// Boost and Buck Converters Control
void enable_led_boost(void);
void disable_led_boost(void);
void enable_input_buck(void);
void disable_input_buck(void);
void enable_usb_boost(void);
void disable_usb_boost(void);

// Power Path Switch Control
void enable_output_switch(void);
void disable_output_switch(void);
void enable_charge_switch(void);
void disable_charge_switch(void);
void enable_battery_switch(void);
void disable_battery_switch(void);

// Toggle Utilities
void toggle_charger_switch(void);
void toggle_battery_switch(void);
void toggle_output_switch(void);
void toggle_usb(void);
void toggle_buck(void);
void toggle_led(void);

// Retrieves current system state into provided array
int get_system_state(int *output_array, int output_array_size);

// Ensures safe startup power sequencing
bool startup_safe_connect(void);

bool static buttonPressed = false;

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

// Reads and updates all ADC measurement buffers
void read_adc_values(void);

/* Measurement Conversion Functions */
int32_t get_charge_current(void);
uint16_t get_charge_voltage(void);
uint16_t get_input_voltage(void);
int16_t get_input_current(void);
uint16_t get_discharge_current(void);
uint16_t get_battery_voltage(void);
uint16_t get_output_voltage(void);
uint16_t get_usb_voltage(USB_SENSOR usb);

/* Logged Measurement Versions */
int32_t get_charge_current_log(void);
uint16_t get_charge_voltage_log(void);
uint16_t get_input_voltage_log(void);
int16_t get_input_current_log(void);
uint16_t get_discharge_current_log(void);
uint16_t get_battery_voltage_log(void);
uint16_t get_output_voltage_log(void);
uint16_t get_usb_voltage_log(void);

int32_t get_input_power_log(void);
uint32_t get_output_power_log(void);
int32_t get_power_into_battery_log(void);

// LED and Temperature Measurements
uint16_t get_led_transistor_voltage(LED_OUTPUT led);
float get_temperature(TEMP_SENSOR temp_sensor);

// Real-time Power Calculations
int32_t get_input_power(void);
uint32_t get_output_power(void);
int32_t get_power_into_battery(void);

/* ============================================================================
 * PWM MODULE
 * ============================================================================ */

// PWM Channel Configuration Structure
typedef struct {
    GPTIMER_Regs *TIMER;              // Pointer to timer register base
    uint8_t CC_INDEX;                 // Capture/Compare channel index
    uint8_t is_complementary_output;  // Indicates complementary PWM output
} PWM_Config;

void pwm_init(void);

// Sets duty cycle for specified PWM channel
void set_pwm_duty_cycle(const PWM_Config* pwm_channel, uint16_t duty_cycle);

// Sets battery charging voltage reference
uint16_t set_charging_voltage(uint16_t voltage);

// Sets LED boost converter voltage
void set_led_voltage(uint16_t voltage);

// Sets LED current limit for specified LED
void set_led_current(uint16_t current, LED_OUTPUT led);

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

extern const uint8_t DIGITS[10];       // Numeric segment encoding
extern const uint8_t CHARACTERS[7];    // Character segment encoding

void led_display_init(void);
void update_led_bar(uint8_t data, LED_BAR_ID led_bar_id);
void update_seven_segment_display(uint8_t data, LED_DIGIT_ID led_digit_id);
void update_led_display(void);
void display_time(void);
void display_error_fault(void);
void display_ovp_fault(void);
void display_ocp_fault(void);
void displayCurrentPower(void);
void displayChargeStorage(void);

/* ============================================================================
 * IR REMOTE INTERFACE
 * ============================================================================ */

#define TIMER_CAPTURE_DURATION (CAPTURE_0_INST_LOAD_VALUE)

extern volatile bool command_processed;

void receive_command(void);
uint32_t get_command(void);
void handle_ir(void);

/* ============================================================================
 * LIGHT SWITCH INTERFACE
 * ============================================================================ */

void l_sw_init(void);
void receive_command_sw(void);
void on_received_edge(void);
void reset_sw_receive(void);
uint32_t get_decoded_value(void);
uint8_t get_msg_led();

/* ============================================================================
 * BUTTON INTERFACE
 * ============================================================================ */

// Button structure for event-driven handling
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

enum {
    BUTTON_NONE = 0,
    BUTTON_1 = 1,
    BUTTON_2 = 2
} BUTTON_PRESSED, BUTTON_HELD;

enum LED1_VALUES{
    LED1_VALUE_0,
    LED1_VALUE_1,
    LED1_VALUE_2,
    LED1_VALUE_3,
    LED1_VALUE_4,
} led1_value;

enum LED2_VALUES{
    LED2_VALUE_0,
    LED2_VALUE_1,
    LED2_VALUE_2,
    LED2_VALUE_3,
    LED2_VALUE_4,
} led2_value;

extern volatile bool check_buttons;
extern Button BUTTONS[2];

void buttons_init(void);
bool get_button_state(const Button* btn);
bool is_button_pressed(const Button* btn);
bool is_button_released(const Button* btn);
bool is_button_held(const Button* btn);
void update_buttons(void);
void handle_button_input(void);

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

extern volatile bool data_received;

void uart_init(void);
void printToUART(char* string, char end_char);
void UARTReceive(void);
void get_UART_buffer(char* output_buffer);
void handle_uart(void);

/* ============================================================================
 * POWER MANAGEMENT
 * ============================================================================ */

void power_manager(void);

/* ============================================================================
 * LED OUTPUT CONTROL
 * ============================================================================ */

void turn_on_led(LED_OUTPUT led);
void turn_on(LED_OUTPUT led);
void turn_on_outputs(void);
void turn_off_outputs(void);

/* ============================================================================
 * MPPT CONTROL
 * ============================================================================ */

extern volatile bool do_mppt;

// MPPT state machine steps (Perturb & Observe method)
typedef enum {
    MPPT_GET_POWER1,
    MPPT_ADJUST_AND_WAIT,
    MPPT_GET_POWER2,
    MPPT_EVALUATE
} MpptStep;

// Executes one MPPT perturb-and-observe step
void apply_mppt_perturb_observe_step(void);

#endif