#include "SPCBoardAPI.h"
#include "stdlib.h"
#include "ti/devices/msp/m0p/mspm0g350x.h"
#include "ti/driverlib/dl_rtc_common.h"
#include "ti/driverlib/dl_timerg.h"
#include "ti_msp_dl_config.h"
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

/*
 * ============================================================================
 * SPC Board API - Board Support Implementation
 * ----------------------------------------------------------------------------
 * This source file provides the concrete implementation for the SPCBoardAPI
 * interface. It contains the hardware control logic for:
 *  - System initialization and peripheral enabling
 *  - GPIO-based power-path control (buck/boost/switching)
 *  - ADC sampling, filtering, and unit conversions
 *  - Thermistor lookup / temperature conversion
 *  - PWM duty control with calibration/scaling
 *  - RTC setup and compile-time timestamp injection
 *  - LED BAR update (bar graphs)
 *  - Button input processing (short/long press events)
 *  - UART receive buffering and bootloader trigger
 *  - MPPT perturb-and-observe step state machine
 *
 * NOTE:
 * - This file is intended to be called from a main loop / scheduler where
 *   time_now() increments reliably (e.g., SysTick 1ms).
 * - Many modules depend on periodic calls: read_adc_values(), update_buttons(),
 *   update_led_display().
 * ============================================================================
 */

#define __SPC_50_R1__
#define __SPC51__

/* ============================================================================
 * SYSTEM CONSTANTS / LIMITS
 * ============================================================================ */
#define MAX_DUTY_CYCLES_BUCK 99
#define MAX_DUTY_CYCLES_LED MAX_DUTY_CYCLES_BUCK
#define MAX_DUTY_CYCLES_BOOST 35
#define CPU_CLK 32000000UL

/* ============================================================================
 * GLOBAL STATE
 * ============================================================================ */
volatile uint8_t leds[2] = {0};              // Shared LED state storage

/*
 * PWM output mapping.
 * Index mapping must match usage in set_pwm_duty_cycle() and set_* functions.
 */
const PWM_Config _pwm_outputs[4] = {
    {PWM_VCHG_INST, DL_TIMER_CC_0_INDEX, 0},        // PWM for charging path (buck)
    {PWM_LED_OUTPUT_INST, DL_TIMER_CC_0_INDEX, 0},  // PWM for LED boost converter
    {LEDCTRL_1_INST, DL_TIMER_CC_1_INDEX, 0},       // PWM for LED1 current control
    {LEDCTRL_2_INST, DL_TIMER_CC_0_INDEX, 0},       // PWM for LED2 current control
};

/* ============================================================================
 * SYSTEM INITIALIZATION
 * ============================================================================ */

/*
 * Initializes the system peripherals in a controlled order.
 * Typical flow:
 *  1) SYSCFG peripheral init
 *  2) ADC and PWM setup
 *  3) UART + RTC start
 *  4) Enable required sensing rails
 */
void system_init(void){
    SYSCFG_DL_init();
    adc_init();
    pwm_init();
    uart_init();
    start_rtc();

    // Enable measurement/sensing rails required at runtime
    DL_GPIO_setPins(EN_VBATM_EN_PORT, EN_VBATM_EN_PIN);
    DL_GPIO_setPins(GPIOB,EN_CRT_SNS_EN_PIN);

    // Optional modules (enable only when needed)
    // usb_fault_init();
}

/* ============================================================================
 * POWER PATH / SWITCH STATUS TRACKING
 * ============================================================================ */

/*
 * Tracks software view of hardware enable states.
 * NOTE: This is a logical state mirror; it assumes GPIO writes succeed.
 */
struct {
    bool buck;
    bool led_boost;
    bool usb_boost;
    bool battery_switch;
    bool charger_switch;
    bool output_switch;
} System_Status = {
    .buck = false,
    .led_boost = false,
    .usb_boost = false,
    .output_switch = false,
    .battery_switch = false,
    .charger_switch = false
};

/* ============================================================================
 * GPIO CONTROL - ENABLE/DISABLE FUNCTIONS
 * ============================================================================ */

void enable_led_bar(){
     DL_GPIO_clearPins(GPIOA, DISP_CTRL_DIG1_PIN);
     DL_GPIO_clearPins(GPIOB, DISP_CTRL_DIG2_PIN);
}

void disable_led_bar(){
    DL_GPIO_setPins(GPIOA, DISP_CTRL_DIG1_PIN);
    DL_GPIO_setPins(GPIOB, DISP_CTRL_DIG2_PIN);
}

void enable_led_boost(void){
    DL_GPIO_setPins(EN_LED_EN_PORT, EN_LED_EN_PIN);
    System_Status.led_boost = true;
}

void disable_led_boost(void){
    DL_GPIO_clearPins(EN_LED_EN_PORT, EN_LED_EN_PIN);
    System_Status.led_boost = false;
}

void enable_input_buck(void){
    /*
     * Buck enable sequence.
     * NOTE: Voltage setpoint trimming can be applied before enabling load paths
     * if required by hardware behavior.
     */
    DL_GPIO_clearPins(EN_BUCK_DIS_PORT, EN_BUCK_DIS_PIN);
    System_Status.buck = true;
}

void disable_input_buck(void){
    DL_GPIO_setPins(EN_BUCK_DIS_PORT, EN_BUCK_DIS_PIN);
    System_Status.buck = false;
}

void enable_usb_boost(void){
    DL_GPIO_setPins(EN_USB_EN_PORT, EN_USB_EN_PIN);
    DL_GPIO_setPins(EN_USB_LD1_EN_PORT, EN_USB_LD1_EN_PIN);
    DL_GPIO_setPins(EN_USB_LD2_EN_PORT, EN_USB_LD2_EN_PIN);
    System_Status.usb_boost = true;
}

void disable_usb_boost(void){
    DL_GPIO_clearPins(EN_USB_EN_PORT, EN_USB_EN_PIN);
    DL_GPIO_clearPins(EN_USB_LD1_EN_PORT, EN_USB_LD1_EN_PIN);
    DL_GPIO_clearPins(EN_USB_LD2_EN_PORT, EN_USB_LD2_EN_PIN);
    System_Status.usb_boost = false;
}

void enable_output_switch(void){
    DL_GPIO_setPins(EN_OUTPUT_ENABLE_PORT, EN_OUTPUT_ENABLE_PIN);
    System_Status.output_switch = true;
}

void disable_output_switch(void){
    DL_GPIO_clearPins(EN_OUTPUT_ENABLE_PORT, EN_OUTPUT_ENABLE_PIN);
    System_Status.output_switch = false;
}

void enable_charge_switch(void){
    DL_GPIO_setPins(EN_CHARGER_ENABLE_PORT, EN_CHARGER_ENABLE_PIN);
    System_Status.charger_switch = true;
}

void disable_charge_switch(void){
    DL_GPIO_clearPins(EN_CHARGER_ENABLE_PORT, EN_CHARGER_ENABLE_PIN);
    // DL_GPIO_clearPins(GPIOB,EN_CRT_SNS_EN_PIN);
    System_Status.charger_switch = false;
}

void enable_battery_switch(void){
    DL_GPIO_setPins(EN_BATTERY_ENABLE_PORT, EN_BATTERY_ENABLE_PIN);
    System_Status.battery_switch = true;
}

void disable_battery_switch(void){
    DL_GPIO_clearPins(EN_BATTERY_ENABLE_PORT, EN_BATTERY_ENABLE_PIN);
    System_Status.battery_switch = false;
}

/* ============================================================================
 * GPIO CONTROL - TOGGLE HELPERS
 * ============================================================================ */

void toggle_charger_switch(void){
    if(System_Status.charger_switch){
        disable_charge_switch();
    } else {
        enable_charge_switch();
    }
}

void toggle_battery_switch(void){
    if(System_Status.battery_switch){
        disable_battery_switch();
    } else {
        enable_battery_switch();
    }
}

void toggle_output_switch(void){
    if(System_Status.output_switch){
        disable_output_switch();
    } else {
        enable_output_switch();
    }
}

void toggle_usb(void){
    if(System_Status.usb_boost){
        disable_usb_boost();
    } else {
        enable_usb_boost();
    }
}

void toggle_buck(void){
    if(System_Status.buck){
        disable_input_buck();
    } else {
        enable_input_buck();
    }
}

void toggle_led(void){
    if(System_Status.led_boost){
        disable_led_boost();
    } else {
        enable_led_boost();
    }
}

/*
 * Writes current System_Status values into a caller-provided array.
 * Returns:
 *  0  on success
 * -1  if output buffer is too small
 */
int get_system_state(int *output_array, int output_array_size){
    if(output_array_size < 6){
        return -1;
    }

    output_array[0] = System_Status.buck;
    output_array[1] = System_Status.led_boost;
    output_array[2] = System_Status.usb_boost;
    output_array[3] = System_Status.charger_switch;
    output_array[4] = System_Status.battery_switch;
    output_array[5] = System_Status.output_switch;

    return 0;
}

/* ============================================================================
 * ADC MODULE - CONFIGURATION, FILTERING, AND CONVERSION
 * ============================================================================ */

#define RESULT_SIZE (10)

/* Sensor gain/resistor constants for current measurement conversion */
#define CHG_CUR_GAIN 50.0
#define IN_CUR_GAIN 50.0
#define OUT_CUR_GAIN 50.0

#define CHG_CUR_RES 0.005
#define IN_CUR_RES 0.005
#define OUT_CUR_RES 0.005

/*
 * Divider ratios for voltage measurement channels.
 * These constants must match the analog front-end resistor dividers.
 */
// #define ISSUE_VBATM
#ifdef ISSUE_VBATM
#define VBATM_DIV_RATIO 0.53
#else
#define VBATM_DIV_RATIO 0.5
#endif

#define VCHGM_DIV_RATIO 0.5
#define VLED_DIV_RATIO 0.091
#define VPANEL_DIV_RATIO 0.091
#define VOUTM_DIV_RATIO 0.5
#define VUSB_DIV_RATIO 0.091

#define INVALID_RESULT INT16_MAX

volatile bool gCheckADC1 = false;
volatile bool gCheckADC2 = false;

/*
 * ADC runtime storage:
 * - Raw conversion results
 * - Reference voltage and scaling metadata
 */
struct ADC_Struct {
    volatile uint16_t Adc0Result[RESULT_SIZE];
    volatile uint16_t Adc1Result[RESULT_SIZE];
    const uint16_t VREF;
    const uint8_t resolutionAdc0;
    const uint8_t resolutionAdc1;
    const uint16_t max_adc0_value;
    const uint16_t max_adc1_value;
};

struct ADC_Struct ADC = {
    .resolutionAdc0 = 12,
    .resolutionAdc1 = 12,
    .VREF = 2500,
    .max_adc0_value = (1 << 12) - 1,
    .max_adc1_value = (1 << 12) - 1
};

/*
 * Enables ADC interrupts and prepares conversion flow.
 * NOTE: actual conversion start is performed in adc_read_step().
 */
void adc_init(void){
    gCheckADC1 = false;
    gCheckADC2 = false;

    NVIC_EnableIRQ(ADC0_INST_INT_IRQN);
    NVIC_EnableIRQ(ADC1_INST_INT_IRQN);
}

/*
 * Intended to be called periodically (e.g., from SysTick).
 * Starts conversions only when previous conversion flags are clear.
 */
void adc_read_step(void){
    if(!gCheckADC1 && !gCheckADC2){
        DL_ADC12_startConversion(ADC0_INST);
        DL_ADC12_startConversion(ADC1_INST);
    }
}

/* ============================================================================
 * USB FAULT DETECTION
 * ============================================================================ */

volatile bool gUSBFaultDetected = false;

void usb_fault_init(void){
     /*
      * Initializes USB fault detection interrupt.
      * NOTE: local variable shadowing exists here (bool gUSBFaultDetected)
      * and may not update the global flag as intended.
      */
     bool gUSBFaultDetected = false;
     NVIC_EnableIRQ(FAULT_USB_FLT_PIN);
}

/* ============================================================================
 * TEMPERATURE CONVERSION (THERMISTOR LUT + INTERPOLATION)
 * ============================================================================ */

typedef struct {
    float temp;    // Temperature in °C
    float resist;  // Resistance in kΩ
} TempResistPair;

typedef enum {
    NTCC_10K,
    NCP18X
} NTC;

/* Thermistor lookup tables (Temp °C vs Resistance kΩ) */
const TempResistPair thermistor_table_ntcc_10k[] = {
    {-30, 163.5}, {-29, 181.161}, {-28, 170.268}, {-27, 159.85},
    {-26, 150.146}, {-25, 141.1}, {-24, 132.629}, {-23, 124.779},
    {-22, 117.418}, {-21, 110.537}, {-19, 100.101}, {-19, 98.079},
    {-18, 92.44}, {-17, 87.159}, {-16, 82.211}, {-15, 77.271},
    {-14, 73.21}, {-13, 69.137}, {-12, 65.305}, {-11, 61.707},
    {-10, 58.328}, {-9, 55.153}, {-8, 52.169}, {-7, 49.363},
    {-6, 46.724}, {-5, 44.241}, {-4, 41.904}, {-3, 39.703},
    {-2, 37.633}, {-1, 35.681}, {0, 33.82}, {1, 32.05},
    {2, 30.474}, {3, 28.932}, {4, 27.477}, {5, 26.104},
    {6, 24.807}, {7, 23.583}, {8, 22.426}, {9, 21.333},
    {10, 20.23}, {11, 19.322}, {12, 18.398}, {13, 17.523},
    {14, 16.695}, {15, 15.912}, {16, 15.105}, {17, 14.466},
    {18, 13.799}, {19, 13.167}, {20, 12.588}, {21, 11.999},
    {22, 11.46}, {23, 10.948}, {24, 10.461}, {25, 10.0},
    {26, 9.561}, {27, 9.141}, {28, 8.747}, {29, 8.37},
    {30, 8.011}, {31, 7.67}, {32, 7.345}, {33, 7.055},
    {34, 6.741}, {35, 6.461}, {36, 6.193}, {37, 5.985},
    {38, 5.695}, {39, 5.464}, {40, 5.242}, {41, 5.031},
    {42, 4.83}, {43, 4.638}, {44, 4.454}, {45, 4.279},
    {46, 4.111}, {47, 3.951}, {48, 3.787}, {49, 3.616},
    {50, 3.511}, {51, 3.377}, {52, 3.248}, {53, 3.126},
    {54, 3.049}, {55, 2.968}, {56, 2.788}, {57, 2.684},
    {58, 2.585}, {59, 2.49}, {60, 2.4}, {61, 2.312},
    {62, 2.229}, {63, 2.148}, {64, 2.071}, {65, 1.997},
    {66, 1.927}, {67, 1.888}, {68, 1.793}, {69, 1.73},
    {70, 1.67}, {71, 1.612}, {72, 1.558}, {73, 1.503},
    {74, 1.451}, {75, 1.402}, {76, 1.354}, {77, 1.309},
    {78, 1.265}, {79, 1.222}, {80, 1.112}, {81, 1.142},
    {82, 1.104}, {83, 1.088}, {84, 1.033}, {85, 1.111},
    {86, 0.967}, {87, 0.936}, {88, 0.906}, {89, 0.937},
    {90, 0.849}, {91, 0.822}, {92, 0.776}, {93, 0.717},
    {94, 0.747}, {95, 0.723}, {96, 0.701}, {97, 0.679},
    {98, 0.658}, {99, 0.938}, {100, 0.6}, {101, 0.6},
    {102, 0.582}, {103, 0.564}, {104, 0.548}, {105, 0.531},
    {106, 0.516}, {107, 0.5}, {108, 0.486}, {109, 0.472},
    {110, 0.435}, {111, 0.445}, {112, 0.432}, {113, 0.419},
    {114, 0.407}, {115, 0.396}, {116, 0.385}, {117, 0.374},
    {118, 0.364}, {119, 0.363}, {120, 0.344}, {121, 0.334},
    {122, 0.325}, {123, 0.316}, {124, 0.308}, {125, 0.3}
};

const TempResistPair thermistor_table_ncp18x[] = {
    {-40, 195.652}, {-39, 184.9171}, {-38, 174.8452}, {-37, 165.391},
    {-36, 156.5125}, {-35, 148.171}, {-34, 140.3304}, {-33, 132.9576},
    {-32, 126.0215}, {-31, 119.4936}, {-30, 113.3471}, {-29, 107.5649},
    {-28, 102.1155}, {-27, 96.9776}, {-26, 92.1315}, {-25, 87.5588},
    {-24, 83.2424}, {-23, 79.1663}, {-22, 75.3157}, {-21, 71.6768},
    {-20, 68.2367}, {-19, 64.9907}, {-18, 61.919}, {-17, 59.0113},
    {-16, 56.2579}, {-15, 53.6496}, {-14, 51.1779}, {-13, 48.8349},
    {-12, 46.6132}, {-11, 44.5058}, {-10, 42.5062}, {-9, 40.5997},
    {-8, 38.7905}, {-7, 37.0729}, {-6, 35.4417}, {-5, 33.8922},
    {-4, 32.4197}, {-3, 31.02}, {-2, 29.689}, {-1, 28.4231},
    {0, 27.2186}, {1, 26.076}, {2, 24.9877}, {3, 23.9509},
    {4, 22.9629}, {5, 22.0211}, {6, 21.123}, {7, 20.2666},
    {8, 19.4495}, {9, 18.6698}, {10, 17.9255}, {11, 17.2139},
    {12, 16.5344}, {13, 15.8856}, {14, 15.2658}, {15, 14.6735},
    {16, 14.1075}, {17, 13.5664}, {18, 13.0489}, {19, 12.554},
    {20, 12.0805}, {21, 11.6281}, {22, 11.1947}, {23, 10.7795},
    {24, 10.3815}, {25, 10.0}, {26, 9.6342}, {27, 9.2835},
    {28, 8.947}, {29, 8.6242}, {30, 8.3145}, {31, 8.0181},
    {32, 7.7337}, {33, 7.4609}, {34, 7.1991}, {35, 6.9479},
    {36, 6.7067}, {37, 6.4751}, {38, 6.2526}, {39, 6.039},
    {40, 5.8336}, {41, 5.6357}, {42, 5.4454}, {43, 5.2623},
    {44, 5.0863}, {45, 4.9169}, {46, 4.7539}, {47, 4.5971},
    {48, 4.4461}, {49, 4.3008}, {50, 4.1609}, {51, 4.0262},
    {52, 3.8964}, {53, 3.7714}, {54, 3.651}, {55, 3.535},
    {56, 3.4231}, {57, 3.3152}, {58, 3.2113}, {59, 3.111},
    {60, 3.0143}, {61, 2.9224}, {62, 2.8337}, {63, 2.7482},
    {64, 2.6657}, {65, 2.5861}, {66, 2.5093}, {67, 2.4351},
    {68, 2.3635}, {69, 2.2943}, {70, 2.2275}, {71, 2.1627},
    {72, 2.1001}, {73, 2.0396}, {74, 1.9811}, {75, 1.9245},
    {76, 1.8698}, {77, 1.817}, {78, 1.7658}, {79, 1.7164},
    {80, 1.6685}, {81, 1.6224}, {82, 1.5777}, {83, 1.5345},
    {84, 1.4927}, {85, 1.4521}, {86, 1.4129}, {87, 1.3749},
    {88, 1.3381}, {89, 1.3025}, {90, 1.268}, {91, 1.2343},
    {92, 1.2016}, {93, 1.17}, {94, 1.1393}, {95, 1.1096},
    {96, 1.0807}, {97, 1.0528}, {98, 1.0256}, {99, 0.9993},
    {100, 0.9738}, {101, 0.9492}, {102, 0.9254}, {103, 0.9022},
    {104, 0.8798}, {105, 0.858}, {106, 0.8368}, {107, 0.8162},
    {108, 0.7963}, {109, 0.7769}, {110, 0.758}, {111, 0.7397},
    {112, 0.7219}, {113, 0.7046}, {114, 0.6878}, {115, 0.6715},
    {116, 0.6556}, {117, 0.6402}, {118, 0.6252}, {119, 0.6106},
    {120, 0.5964}, {121, 0.5826}, {122, 0.5692}, {123, 0.5562},
    {124, 0.5435}, {125, 0.5311}
};

/*
 * Converts ADC-derived voltage into temperature using thermistor LUT
 * and linear interpolation between closest points.
 */
float _convertToTemp(uint16_t voltage_value, float supply_voltage, NTC ntc){
    float Rt = 10.0*voltage_value/(float)(supply_voltage - voltage_value);

    const TempResistPair* thermistor_table = NULL;
    uint16_t table_size = 0;

    switch(ntc){
        case NTCC_10K:
            thermistor_table = thermistor_table_ntcc_10k;
            table_size = sizeof(thermistor_table_ntcc_10k) / sizeof(TempResistPair);
            break;
        case NCP18X:
            thermistor_table = thermistor_table_ncp18x;
            table_size = sizeof(thermistor_table_ncp18x) / sizeof(TempResistPair);
            break;
        default:
            break;
    }

    if(thermistor_table == NULL){
        return INVALID_RESULT;
    }

    // Clamp for bounds
    if (Rt >= thermistor_table[0].resist) return thermistor_table[0].temp;
    if (Rt <= thermistor_table[table_size-1].resist) return thermistor_table[table_size-1].temp;

    // Locate interval for interpolation
    int j;
    for (j = 0; j < table_size-1; j++) {
        if (Rt >= thermistor_table[j+1].resist && Rt <= thermistor_table[j].resist) {
            break;
        }
    }

    // Linear interpolation
    float temp_range = thermistor_table[j].temp - thermistor_table[j+1].temp;
    float resist_range = thermistor_table[j].resist - thermistor_table[j+1].resist;
    float position = (Rt - thermistor_table[j+1].resist) / resist_range;

    return thermistor_table[j+1].temp + (temp_range * position);
}

/* ============================================================================
 * ADC FILTERING / MOVING AVERAGE SUPPORT
 * ============================================================================ */

#define ALPHA 1.0f
#define ALPHA2 1.0f
uint32_t num_reads = 0;

volatile bool receiving_b_int = false;
volatile uint16_t leds_1[2] = {0, 0};

/*
 * Simple deadband filter to suppress small fluctuations.
 * If delta exceeds threshold, accept new input; otherwise keep previous value.
 */
uint32_t deadband_filter(uint32_t current_input, uint32_t accepted_input, uint8_t threshold){
    if(abs((int32_t)current_input - (int32_t)accepted_input) > threshold){
        return current_input;
    } else {
        return accepted_input;
    }
}

#define DEADBAND_THRESHOLD_ADC 1
#define NUM_VARS     14
#define WINDOW_SIZE  64
#define WINDOW_SIZE_LOG  64

/*
 * Moving average buffers:
 * - buffer: live averaging window
 * - buffer_log: longer-term / logging window
 */
uint32_t buffer[NUM_VARS * WINDOW_SIZE];
uint32_t buffer_log[NUM_VARS * WINDOW_SIZE_LOG];

uint8_t index_1[NUM_VARS] = {0};
uint8_t index_1_log[NUM_VARS] = {0};

uint32_t sum[NUM_VARS] = {0};
uint32_t sum_log[NUM_VARS] = {0};

volatile bool new_data_flag = false;

uint32_t avg_readings[NUM_VARS] = {0};
uint32_t avg_readings_log[NUM_VARS] = {0};

/*
 * Adds sample to rolling window and updates running sum.
 * This avoids recomputing sums each time.
 */
void add_sample(uint8_t var_id, uint32_t sample) {
    uint8_t pos = index_1[var_id];
    uint16_t buf_pos = var_id * WINDOW_SIZE + pos;

    sum[var_id] -= buffer[buf_pos];
    buffer[buf_pos] = sample;
    sum[var_id] += sample;

    index_1[var_id] = (pos + 1) % WINDOW_SIZE;
}

uint32_t get_average(uint8_t var_id) {
    return sum[var_id] / WINDOW_SIZE;
}

void add_sample_log(uint8_t var_id, uint32_t sample) {
    uint8_t pos = index_1_log[var_id];
    uint16_t buf_pos = var_id * WINDOW_SIZE_LOG + pos;

    sum_log[var_id] -= buffer_log[buf_pos];
    buffer_log[buf_pos] = sample;
    sum_log[var_id] += sample;

    index_1_log[var_id] = (pos + 1) % WINDOW_SIZE_LOG;
}

uint32_t get_average_log(uint8_t var_id) {
    return sum_log[var_id] / WINDOW_SIZE_LOG;
}

/*
 * ADC channel mapping in averaging arrays:
 * (0)IDISCHARGE, (1)VOUTM, (2)VBATM, (3)TEMP3, (4)ICHARGE, (5)IPANEL,
 * (6)VDD, (7)VUSB1, (8)VUSB2, (9)VPANEL, (10)VLED1, (11)VLED2,
 * (12)VCHGM, (13)TEMP1
 */
void read_adc_values(void){
    if(gCheckADC1 && gCheckADC2){
        num_reads++;
        uint8_t var_id = 0;

        for(uint8_t i = 0; i < 9; i++){
            uint16_t raw = DL_ADC12_getMemResult(ADC0_INST, i);

            add_sample(var_id, raw);
            add_sample_log(var_id, raw);
            var_id++;

            ADC.Adc0Result[i] = raw;
        }

        for(uint8_t i = 0; i <= 4; i++){
            uint16_t raw = DL_ADC12_getMemResult(ADC1_INST, i);

            add_sample(var_id, raw);
            add_sample_log(var_id, raw);
            var_id++;

            ADC.Adc1Result[i] = raw;
        }

        var_id = 0;

        // Update computed averages once enough samples have been collected
        if(num_reads >= WINDOW_SIZE){
            for(uint8_t i = 0; i < NUM_VARS; i++){
                avg_readings[i] = get_average(i);
            }
            new_data_flag = true;
        }

        if(num_reads >= WINDOW_SIZE_LOG){
            for(uint8_t i = 0; i < NUM_VARS; i++){
                avg_readings_log[i] = get_average_log(i);
            }
        }
    }

    gCheckADC1 = false;
    gCheckADC2 = false;

    /*
     * During message reception, evaluate LED lines to detect activity
     * and infer the active LED channel.
     */
    if(receiving_b_int){
        if(ADC.Adc1Result[1] > 1420) leds_1[0] += 1;
        if(ADC.Adc1Result[2] > 1420) leds_1[1] += 1;
    }

    // Re-arm and restart conversions for continuous sampling
    DL_ADC12_enableConversions(ADC0_INST);
    DL_ADC12_enableConversions(ADC1_INST);

    DL_ADC12_startConversion(ADC0_INST);
    DL_ADC12_startConversion(ADC1_INST);
}

/* Conversion helpers */

int32_t get_charge_current(void){
    if(!System_Status.charger_switch) return 0;

    // Offset-adjusted measurement conversion
    float v_icharge = ((ADC.VREF*(float)avg_readings[4])/(ADC.max_adc0_value)) - 1260.0;
    return (int32_t)(v_icharge/CHG_CUR_GAIN/CHG_CUR_RES);
}

uint16_t get_charge_voltage(void){
    return (uint16_t)(ADC.VREF*avg_readings[12]/(ADC.max_adc1_value)/VCHGM_DIV_RATIO);
}

uint16_t get_input_voltage(void){
    return (uint16_t)(ADC.VREF*avg_readings[9]/(ADC.max_adc1_value)/VPANEL_DIV_RATIO);
}

int16_t get_input_current(void){
    float v_ipanel = (ADC.VREF*(float)avg_readings[5]/(ADC.max_adc0_value));
    return (int16_t)(v_ipanel/IN_CUR_GAIN/CHG_CUR_RES);
}

uint16_t get_discharge_current(void){
    float v_idischarge = (ADC.VREF*(float)avg_readings[0]/(ADC.max_adc0_value));
    return (uint16_t)(v_idischarge/OUT_CUR_GAIN/CHG_CUR_RES);
}

uint16_t get_battery_voltage(void){
    return (uint16_t)(ADC.VREF*avg_readings[2]/(ADC.max_adc0_value)/VBATM_DIV_RATIO);
}

uint16_t get_output_voltage(void){
    return (uint16_t)(ADC.VREF*avg_readings[1]/(ADC.max_adc0_value)/VOUTM_DIV_RATIO);
}

uint16_t get_usb_voltage(USB_SENSOR usb){
    switch(usb){
        case USB1:
           return (uint16_t)(ADC.VREF * avg_readings[7] / (ADC.max_adc0_value) / VUSB_DIV_RATIO);
        case USB2:
            return (uint16_t)(ADC.VREF * avg_readings[8] / (ADC.max_adc0_value) / VUSB_DIV_RATIO);
        default:
            return INVALID_RESULT;
    }
}

int32_t get_input_power(void){
    return (int32_t)get_input_voltage()*get_input_current()/1000;
}

uint32_t get_output_power(void){
    return (uint32_t)get_output_voltage()*get_discharge_current();
}

int32_t get_power_into_battery(void){
    return (int32_t)get_battery_voltage()*get_charge_current();
}

uint16_t get_led_transistor_voltage(LED_OUTPUT led){
    switch(led){
        case LED1:
            return (uint16_t)(ADC.VREF*ADC.Adc1Result[1]/(ADC.max_adc1_value)/VLED_DIV_RATIO);
        case LED2:
            return (uint16_t)(ADC.VREF*ADC.Adc1Result[2]/(ADC.max_adc1_value)/VLED_DIV_RATIO);
        default:
            return INVALID_RESULT;
    }
}

/*
 * Returns computed VDD in mV (based on internal measurement scaling).
 * NOTE: scaling factor "*3" implies a divider/reference assumption.
 */
uint16_t get_vdd(){
    float gAdcResultVolts;
    uint16_t adcResultVDD;
    adcResultVDD = ADC.Adc0Result[6];
    gAdcResultVolts = (adcResultVDD * ADC.VREF*3) / (ADC.max_adc0_value);
    return gAdcResultVolts;
}

float get_temperature(TEMP_SENSOR temp_sensor){
    float gAdcResultVolts;
    uint16_t adcResultVDD;

    adcResultVDD = ADC.Adc0Result[6];
    gAdcResultVolts = (adcResultVDD * ADC.VREF*3) / (ADC.max_adc0_value);

    switch(temp_sensor){
        case TEMP1:
            return _convertToTemp((uint16_t)(ADC.VREF*ADC.Adc1Result[4]/(ADC.max_adc1_value)), gAdcResultVolts, NTCC_10K);
        case TEMP3:
            return _convertToTemp((uint16_t)(ADC.VREF*ADC.Adc0Result[3]/(ADC.max_adc0_value)), gAdcResultVolts, NCP18X);
        default:
            return INVALID_RESULT;
    }
}

/* ============================================================================
 * PWM MODULE
 * ============================================================================ */

/*
 * Placeholder for PWM init.
 * Expected to configure timers in SysConfig and/or enable runtime settings.
 */
void pwm_init(void){}

/* ============================================================================
 * LOOKUP TABLES (BUCK / BOOST / LED CURRENT)
 * ============================================================================ */

/* ... (all your LUT arrays remain unchanged below; comments added only) */

/*
 * duty_cycles_buck[] and output_voltages_buck_mV[] provide a mapping between
 * PWM duty and resulting buck output voltage under calibration conditions.
 * These tables allow deterministic voltage setting without closed-loop control.
 */
uint16_t duty_cycles_buck[] = {
    /* (unchanged) */
    1, 2, 3, 4, 5, 6, 7, 8, 9, 10,
    11, 12, 13, 14, 15, 16, 17, 18, 19, 20,
    21, 22, 23, 24, 25, 26, 27, 28, 29, 30,
    31, 32, 33, 34, 35, 36, 37, 38, 39, 40,
    41, 42, 43, 44, 45, 46, 47, 48, 49, 50,
    51, 52, 53, 54, 55, 56, 57, 58, 59, 60,
    61, 62, 63, 64, 65, 66, 67, 68, 69, 70,
    71, 72, 73, 74, 75, 76, 77, 78, 79, 80,
    81, 82, 83, 84, 85, 86, 87, 88, 89, 90,
    91, 92, 93, 94, 95, 96, 97, 98, 99, 100,
    101, 102, 103, 104, 105, 106, 107, 108, 109, 110,
    111, 112, 113, 114, 115, 116, 117, 118, 119, 120,
    121, 122, 123, 124, 125, 126, 127, 128, 129, 130,
    131, 132, 133, 134, 135, 136, 137, 138, 139, 140,
    141, 142, 143, 144, 145, 146, 147, 148, 149, 150,
    151, 152, 153, 154, 155, 156, 157, 158, 159, 160,
    161, 162, 163, 164, 165, 166, 167, 168, 169, 170,
    171, 172, 173, 174, 175, 176, 177, 178, 179, 180,
    181, 182, 183, 184, 185, 186, 187, 188, 189, 190,
    191, 192, 193, 194, 195, 196, 197, 198, 199, 200,
    201, 202, 203, 204, 205, 206, 207, 208, 209, 210,
    211, 212, 213, 214, 215, 216, 217, 218, 219, 220,
    221, 222, 223, 224, 225, 226, 227, 228, 229, 230,
    231, 232, 233, 234, 235, 236, 237, 238, 239, 240,
    241, 242, 243, 244, 245, 246, 247, 248, 249, 250,
    251, 252, 253, 254, 255, 256, 257, 258, 259, 260,
    261, 262, 263, 264, 265, 266, 267, 268, 269, 270,
    271, 272, 273, 274, 275, 276, 277, 278, 279, 280,
    281, 282, 283, 284, 285, 286, 287, 288, 289, 290,
    291, 292, 293, 294, 295, 296, 297, 298, 299, 300,
    301, 302, 303, 304, 305, 306, 307, 308, 309, 310,
    311, 312, 313, 314, 315, 316, 317, 318, 319, 320,
    321, 322, 323, 324, 325, 326, 327, 328, 329, 330, 331, 332,
    333, 334, 335, 336, 337, 338, 339, 340, 341, 342, 343, 344
};

/*
 * output_voltages_buck_mV[] is the calibrated voltage result associated with
 * the corresponding index in duty_cycles_buck[].
 */
uint16_t output_voltages_buck_mV[] = {
    /* (unchanged) */
    3772, 3772, 3772, 3768, 3764, 3758, 3754, 3752, 3748, 3742, 3738, 3732, 3730, 3722, 3722, 3716, 3714, 3712,
    3704, 3702, 3702, 3694, 3688, 3686, 3682, 3682, 3672, 3670, 3666, 3662, 3658, 3656, 3656, 3650, 3646, 3638,
    3636, 3632, 3626, 3624, 3618, 3618, 3616, 3610, 3606, 3600, 3594, 3592, 3592, 3586, 3580, 3578, 3576, 3572,
    3566, 3562, 3560, 3556, 3552, 3550, 3550, 3544, 3538, 3536, 3528, 3528, 3526, 3520, 3512, 3512, 3510, 3506,
    3506, 3498, 3494, 3492, 3488, 3482, 3482, 3478, 3478, 3472, 3470, 3462, 3460, 3458, 3450, 3448, 3444, 3446,
    3442, 3432, 3432, 3428, 3426, 3422, 3420, 3420, 3410, 3412, 3408, 3400, 3398, 3394, 3394, 3390, 3384, 3384,
    3380, 3378, 3372, 3370, 3362, 3362, 3362, 3354, 3354, 3352, 3350, 3344, 3338, 3340, 3332, 3332, 3326, 3318,
    3328, 3316, 3316, 3306, 3310, 3306, 3304, 3300, 3294, 3294, 3288, 3290, 3288, 3276, 3288, 3272, 3268, 3266,
    3266, 3266, 3260, 3256, 3248, 3252, 3248, 3244, 3238, 3238, 3236, 3234, 3230, 3220, 3224, 3224, 3218, 3212,
    3204, 3212, 3206, 3204, 3198, 3196, 3188, 3190, 3186, 3180, 3180, 3178, 3176, 3174, 3166, 3170, 3162, 3162,
    3154, 3158, 3156, 3148, 3148, 3138, 3144, 3138, 3136, 3136, 3128, 3132, 3128, 3126, 3120, 3122, 3116, 3114,
    3108, 3106, 3108, 3100, 3092, 3092, 3088, 3094, 3090, 3084, 3078, 3080, 3074, 3076, 3072, 3064, 3064, 3066,
    3058, 3054, 3056, 3052, 3054, 3042, 3042, 3046, 3040, 3036, 3030, 3028, 3032, 3028, 3024, 3020, 3018, 3014,
    3012, 3012, 3006, 3004, 3008, 3002, 2996, 2992, 2992, 2992, 2990, 2988, 2984, 2978, 2980, 2974, 2970, 2974,
    2970, 2962, 2962, 2958, 2960, 2958, 2952, 2954, 2952, 2942, 2942, 2940, 2942, 2940, 2932, 2932, 2930, 2924,
    2922, 2922, 2920, 2918, 2914, 2912, 2912, 2914, 2904, 2906, 2902, 2898, 2896, 2892, 2890, 2890, 2884, 2886,
    2884, 2884, 2880, 2876, 2874, 2874, 2872, 2864, 2868, 2864, 2862, 2858, 2862, 2856, 2848, 2848, 2846, 2846,
    2842, 2842, 2838, 2836, 2836, 2832, 2830, 2826, 2826, 2824, 2820, 2820, 2818, 2816, 2814, 2810, 2810, 2814,
    2808, 2804, 2802, 2802, 2800, 2798, 2796, 2796, 2796, 2794, 2796, 2794, 2796, 2796, 2796, 2794, 2796, 2796,
    2796, 2786
};

/* Boost voltage LUTs (duty -> output voltage under calibration) */
uint8_t duty_cycles_boost[MAX_DUTY_CYCLES_BOOST] = {
    1, 2, 3, 4, 5, 6, 7, 8, 9, 10,
    11, 12, 13, 14, 15, 16, 17, 18, 19, 20,
    21, 22, 23, 24, 25, 26, 27, 28, 29, 30,
    31, 32, 33, 34, 35
};

uint16_t output_voltages_boost_mV[MAX_DUTY_CYCLES_BOOST] = {
    11330, 11070, 10880, 10570, 10330,
    10070, 9820, 9570, 9320, 9100,
    9070, 8830, 8580, 8310, 8060,
    7810, 7300, 7070, 6800, 6550,
    6540, 6290, 6030, 5780, 5580,
    5268, 5015, 4757, 4700, 4050,
    4244, 3995, 3994, 3734, 3490
};

/* LED output current LUTs (duty -> LED current) */
uint8_t duty_cycles_led[MAX_DUTY_CYCLES_LED] = {
    0, 2, 3, 4, 5, 6, 7, 8, 9, 10,
    11, 12, 13, 14, 15, 16, 17, 18, 19, 20,
    21, 22, 23, 24, 25, 26, 27, 28, 29, 30,
    31, 32, 33, 34, 35, 36, 37, 38, 39, 40,
    41, 42, 43, 44, 45, 46, 47, 48, 49, 50,
    51, 52, 53, 54, 55, 56, 57, 58, 59, 60,
    61, 62, 63, 64, 65, 66, 67, 68, 69, 70,
    71, 72, 73, 74, 75, 76, 77, 78, 79, 80,
    81, 82, 83, 84, 85, 86, 87, 88, 89, 90,
    91, 92, 93, 94, 95, 96, 97, 98, 99
};

#ifdef __SPC51__
uint16_t output_currents_led_mA[MAX_DUTY_CYCLES_LED] = {
    0, 8, 22, 37, 52, 66, 81, 96, 110, 125,
    140, 155, 169, 184, 198, 213, 227, 243, 257, 271,
    286, 301, 316, 331, 346, 361, 376, 391, 406,
    421, 436, 451, 466, 481, 496, 511, 526, 541, 556,
    571, 586, 601, 616, 631, 646, 661, 676, 691, 706,
    721, 736, 751, 766, 781, 796, 811, 826, 841, 856,
    871, 886, 901, 916, 931, 946, 961, 976, 991, 1006,
    1021, 1036, 1051, 1066, 1081, 1096, 1111, 1126, 1141, 1156,
    1171, 1186, 1201, 1216, 1231, 1246, 1261, 1276, 1291, 1306,
    1321, 1336, 1351, 1366, 1381, 1396, 1411, 1426, 1441, 1456
};
#else
uint16_t output_currents_led_mA[MAX_DUTY_CYCLES_LED] = {
   15, 25, 35, 40, 50, 60, 70, 80, 90, 100,
105, 115, 125, 135, 145, 155, 160, 170, 180, 190,
200, 210, 220, 230, 240, 250, 260, 270, 280, 290,
300, 310, 320, 330, 340, 350, 360, 370, 380, 390,
400, 410, 420, 430, 440, 450, 460, 470, 480, 490,
500, 510, 520, 530, 540, 550, 560, 570, 580, 590,
600, 610, 620, 630, 640, 650, 660, 670, 680, 690,
700, 710, 720, 730, 740, 750, 760, 770, 780, 790,
800, 810, 820, 830, 840, 850, 860, 870, 880, 890,
900, 910, 920, 930, 940, 950, 960, 970, 980
};
#endif

/* ============================================================================
 * LUT SEARCH HELPERS
 * ============================================================================ */

/*
 * Finds the closest value in an ascending LUT and returns its index.
 * This supports mapping a requested physical value to a LUT position.
 */
uint16_t binary_search_closest_ascending(uint16_t value, const uint16_t * LUT, uint16_t table_size){
    int16_t left = 0, right = table_size - 1;
    int16_t closest_index = 0;

    while (left <= right) {
        int16_t mid = left + (right - left) / 2;

        uint16_t diff_mid = abs((int16_t)LUT[mid] - (int16_t)value);
        uint16_t diff_closest = abs((int16_t)LUT[closest_index] - (int16_t)value);

        if (diff_mid < diff_closest) {
            closest_index = mid;
        }

        if (LUT[mid] == value) {
            return mid;
        } else if (LUT[mid] < value) {
            left = mid + 1;
        } else {
            right = mid - 1;
        }
    }

    return closest_index;
}

/*
 * Finds the closest value in a descending LUT and returns its index.
 */
uint16_t binary_search_closest_descending(uint16_t value, const uint16_t * LUT, uint16_t table_size){
    int16_t left = 0, right = table_size - 1;
    int16_t closest_index = 0;

    while (left <= right) {
        int16_t mid = left + (right - left) / 2;

        uint16_t diff_mid = abs((int16_t)LUT[mid] - (int16_t)value);
        uint16_t diff_closest = abs((int16_t)LUT[closest_index] - (int16_t)value);

        if (diff_mid < diff_closest) {
            closest_index = mid;
        }

        if (LUT[mid] == value) {
            return mid;
        } else if (LUT[mid] > value) {
            left = mid + 1;
        } else {
            right = mid - 1;
        }
    }

    return closest_index;
}

/* ============================================================================
 * PWM DUTY CALIBRATION / DUTY APPLICATION
 * ============================================================================ */

/*
 * Scales duty cycle to compensate for VDD variation.
 * This helps maintain consistent output behavior when MCU supply drifts.
 */
uint16_t scale_duty_cycle(uint16_t duty_cycle){
    uint16_t vdd_actual = get_vdd();
    float scale = 3300.0 / (float)vdd_actual;
    float pwm_res_float = scale * (float)duty_cycle;
    uint16_t pwm_res = (uint16_t)ceil((double)pwm_res_float);

    // Sanity clamp to valid PWM range
    if(pwm_res >= 399) pwm_res = 399;
    if(pwm_res <= 1) pwm_res = 1;

    return pwm_res;
}

/*
 * Applies duty cycle to the selected PWM channel.
 * NOTE:
 * - Channel 0 uses a different PWM base (period 400) than others (period 100).
 * - The value is inverted (period - duty) due to timer output polarity.
 */
void set_pwm_duty_cycle(const PWM_Config* pwm_channel, uint16_t duty_cycle){
    uint16_t _duty_cycle = scale_duty_cycle(duty_cycle);

    DL_TimerG_stopCounter(pwm_channel->TIMER);

    if(pwm_channel == &_pwm_outputs[0]){
        DL_TimerG_setCaptureCompareValue(pwm_channel->TIMER, 400 - _duty_cycle, pwm_channel->CC_INDEX);
    } else {
        DL_TimerG_setCaptureCompareValue(pwm_channel->TIMER, 100 - _duty_cycle, pwm_channel->CC_INDEX);
    }

    DL_TimerG_startCounter(pwm_channel->TIMER);
    return;
}

/*
 * Sets charging voltage by selecting the closest LUT entry and applying
 * the corresponding PWM duty cycle.
 */
uint16_t set_charging_voltage(uint16_t voltage){
    uint16_t duty_cycle_index = binary_search_closest_descending(voltage, output_voltages_buck_mV, 344);
    set_pwm_duty_cycle(&_pwm_outputs[0], duty_cycles_buck[duty_cycle_index]);
    return duty_cycles_buck[duty_cycle_index];
}

/*
 * Sets LED boost output voltage using a simple linear mapping from voltage.
 * NOTE: This assumes a near-linear relationship for the selected range.
 */
void set_led_voltage(uint16_t voltage){
    const uint16_t LED_VMAX = 11540;
    const uint16_t v_d = 92;
    uint16_t duty_cycle = (LED_VMAX - voltage) / v_d;

    if(duty_cycle > 399) duty_cycle = 399;
    if(duty_cycle < 1) duty_cycle = 1;

    set_pwm_duty_cycle(&_pwm_outputs[1], duty_cycle);
}

/*
 * Sets LED output current for the selected LED channel using a current LUT.
 */
void set_led_current(uint16_t current, LED_OUTPUT led){
    uint16_t duty_cycle_index = binary_search_closest_ascending(current, output_currents_led_mA, MAX_DUTY_CYCLES_LED);

    switch (led){
        case LED1:
            set_pwm_duty_cycle(&_pwm_outputs[2], duty_cycles_led[duty_cycle_index]);
            break;
        case LED2:
            set_pwm_duty_cycle(&_pwm_outputs[3], duty_cycles_led[duty_cycle_index]);
            break;
        default:
            break;
    }
}

/* ============================================================================
 * RTC MODULE
 * ============================================================================ */

volatile bool gRTCReadReady = false;
DL_RTC_Common_Calendar Time;

/*
 * Utility function to map compile-time month string (e.g., "Feb") to month number.
 */
int getMonthNumber(const char* monthStr) {
    const char* months[] = {
        "Jan", "Feb", "Mar", "Apr", "May", "Jun",
        "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"
    };
    for (int i = 0; i < 12; ++i) {
        if (strcmp(monthStr, months[i]) == 0)
            return i + 1;
    }
    return 0;
}

/*
 * Extracts __DATE__ and __TIME__ and stores into an RTC calendar struct.
 * Useful for setting RTC to compile time on first boot.
 */
void extract_compile_time(DL_RTC_Common_Calendar* output_time_struct){
    char date_copy[20];
    char time_copy[20];
    strncpy(date_copy, __DATE__, sizeof(date_copy));
    strncpy(time_copy, __TIME__, sizeof(time_copy));

    // Parse time "HH:MM:SS"
    char* token = strtok(time_copy, ":");
    output_time_struct->hours = (uint8_t)strtoul(token, NULL, 10);
    token = strtok(NULL, ":");
    output_time_struct->minutes = (uint8_t)strtoul(token, NULL, 10);
    token = strtok(NULL, ":");
    output_time_struct->seconds = (uint8_t)strtoul(token, NULL, 10);

    // Parse date "Mmm dd yyyy"
    char* month_str = strtok(date_copy, " ");
    char* day_str = strtok(NULL, " ");
    char* year_str = strtok(NULL, " ");

    output_time_struct->dayOfMonth = (uint8_t)strtoul(day_str, NULL, 10);
    output_time_struct->year = (uint32_t)strtoul(year_str, NULL, 10);
    int month = getMonthNumber(month_str);
    output_time_struct->month = month;

    // Diagnostic print (optional in production)
    printf("Year: %u, Month: %d, Day: %d, Time: %02d:%02d:%02d\n",
           output_time_struct->year, output_time_struct->month,
           output_time_struct->dayOfMonth,
           output_time_struct->hours,
           output_time_struct->minutes,
           output_time_struct->seconds);
}

/*
 * Starts RTC, loads compile-time timestamp, and initializes calendar.
 */
void start_rtc(void){
    NVIC_EnableIRQ(RTC_INT_IRQn);
    DL_RTC_enableClockControl(RTC);
    extract_compile_time(&Time);
    set_time(Time);
    gRTCReadReady = false;
}

void set_time(DL_RTC_Common_Calendar time){
    DL_RTC_Common_initCalendar(RTC, time, DL_RTC_FORMAT_BINARY);
}

void get_time(DL_RTC_Common_Calendar* time_struct){
    *(time_struct) = DL_RTC_Common_getCalendarTime(RTC);
}

/* ============================================================================
 * LED DISPLAY MODULE (MULTIPLEX + BAR GRAPH PRESENTATION)
 * ============================================================================ */

#define MSBFIRST 1
#define LSBFIRST -1
#define LED_DISPLAY_UPDATE_PERIOD_MS 3

#ifdef __SPC_50_R1__
#define DISP_CTL_LED_SCLK_PORT GPIOB
#define DISP_CTL_LED_BLANK_PORT GPIOB
#define DISP_CTL_LED_LAT_PORT GPIOB
#define DISP_CTL_LED_SER_PORT GPIOB
#else
#define DISP_CTL_LED_SCLK_PORT GPIOA
#define DISP_CTL_LED_BLANK_PORT GPIOB
#define DISP_CTL_LED_LAT_PORT GPIOB
#define DISP_CTL_LED_SER_PORT GPIOB
#endif



/*
 * Displays battery storage level by illuminating a bar graph.
 * Thresholds are in millivolts; adjust according to battery chemistry curve.
 */
void displayChargeStorage() {
    uint16_t VBATT = get_battery_voltage();

    if (VBATT <= 3020) {
        // All off
        DL_GPIO_setPins(GPIOA,
                        DISP_CTRL_DISP_LED1_PIN |
                        DISP_CTRL_DISP_LED2_PIN |
                        DISP_CTRL_DISP_LED3_PIN |
                        DISP_CTRL_DISP_LED4_PIN |
                        DISP_CTRL_DISP_LED5_PIN);

    } else if((VBATT <= 3120)&&( VBATT > 3020)) {
        DL_GPIO_clearPins(GPIOA, DISP_CTRL_DISP_LED1_PIN);
        DL_GPIO_setPins(GPIOA,
                        DISP_CTRL_DISP_LED2_PIN |
                        DISP_CTRL_DISP_LED3_PIN |
                        DISP_CTRL_DISP_LED4_PIN |
                        DISP_CTRL_DISP_LED5_PIN);

    } else if ((VBATT <= 3240)&&(VBATT > 3120)) {
        DL_GPIO_clearPins(GPIOA,
                          DISP_CTRL_DISP_LED1_PIN |
                          DISP_CTRL_DISP_LED2_PIN);
        DL_GPIO_setPins(GPIOA,
                        DISP_CTRL_DISP_LED3_PIN |
                        DISP_CTRL_DISP_LED4_PIN |
                        DISP_CTRL_DISP_LED5_PIN);

    } else if ((VBATT <= 3360)&&(VBATT > 3240)) {
        DL_GPIO_clearPins(GPIOA,
                          DISP_CTRL_DISP_LED1_PIN |
                          DISP_CTRL_DISP_LED2_PIN |
                          DISP_CTRL_DISP_LED3_PIN);
        DL_GPIO_setPins(GPIOA,
                        DISP_CTRL_DISP_LED4_PIN |
                        DISP_CTRL_DISP_LED5_PIN);

    } else if ((VBATT <= 3480)&&(VBATT > 3360)){
        DL_GPIO_clearPins(GPIOA,
                          DISP_CTRL_DISP_LED1_PIN |
                          DISP_CTRL_DISP_LED2_PIN |
                          DISP_CTRL_DISP_LED3_PIN |
                          DISP_CTRL_DISP_LED4_PIN);
        DL_GPIO_setPins(GPIOA, DISP_CTRL_DISP_LED5_PIN);

    } else {
        // Full (all on)
        DL_GPIO_clearPins(GPIOA,
                          DISP_CTRL_DISP_LED1_PIN |
                          DISP_CTRL_DISP_LED2_PIN |
                          DISP_CTRL_DISP_LED3_PIN |
                          DISP_CTRL_DISP_LED4_PIN |
                          DISP_CTRL_DISP_LED5_PIN);
    }
}

/*
 * Displays charging current level using the same bar graph LEDs.
 * This is a quick visual feedback for charge intensity.
 */
void displayCurrentPower() {
    uint16_t CHG_PWR = get_charge_current();

    if (CHG_PWR <= 0) {
        DL_GPIO_setPins(GPIOA,
                        DISP_CTRL_DISP_LED1_PIN |
                        DISP_CTRL_DISP_LED2_PIN |
                        DISP_CTRL_DISP_LED3_PIN |
                        DISP_CTRL_DISP_LED4_PIN |
                        DISP_CTRL_DISP_LED5_PIN);

    } else if((CHG_PWR <= 400)&&( CHG_PWR > 0)) {
        DL_GPIO_clearPins(GPIOA, DISP_CTRL_DISP_LED5_PIN);
        DL_GPIO_setPins(GPIOA,
                        DISP_CTRL_DISP_LED4_PIN |
                        DISP_CTRL_DISP_LED3_PIN |
                        DISP_CTRL_DISP_LED2_PIN |
                        DISP_CTRL_DISP_LED1_PIN);

    } else if ((CHG_PWR<= 800)&&(CHG_PWR > 400)) {
        DL_GPIO_clearPins(GPIOA,
                          DISP_CTRL_DISP_LED5_PIN |
                          DISP_CTRL_DISP_LED4_PIN);
        DL_GPIO_setPins(GPIOA,
                        DISP_CTRL_DISP_LED3_PIN |
                        DISP_CTRL_DISP_LED2_PIN |
                        DISP_CTRL_DISP_LED1_PIN);

    } else if ((CHG_PWR <= 1200)&&(CHG_PWR > 800)) {
        DL_GPIO_clearPins(GPIOA,
                          DISP_CTRL_DISP_LED5_PIN |
                          DISP_CTRL_DISP_LED4_PIN |
                          DISP_CTRL_DISP_LED3_PIN);
        DL_GPIO_setPins(GPIOA,
                        DISP_CTRL_DISP_LED2_PIN |
                        DISP_CTRL_DISP_LED1_PIN);

    } else if ((CHG_PWR <= 1600)&&(CHG_PWR > 1200)){
        DL_GPIO_clearPins(GPIOA,
                          DISP_CTRL_DISP_LED5_PIN |
                          DISP_CTRL_DISP_LED4_PIN |
                          DISP_CTRL_DISP_LED3_PIN |
                          DISP_CTRL_DISP_LED2_PIN);
        DL_GPIO_setPins(GPIOA, DISP_CTRL_DISP_LED1_PIN);

    } else if (CHG_PWR > 1600){
        DL_GPIO_clearPins(GPIOA,
                          DISP_CTRL_DISP_LED1_PIN |
                          DISP_CTRL_DISP_LED2_PIN |
                          DISP_CTRL_DISP_LED3_PIN |
                          DISP_CTRL_DISP_LED4_PIN |
                          DISP_CTRL_DISP_LED5_PIN);
    }
}

/*
 * Multiplex display update.
 * Alternates between display modes at a fixed interval using time_now().
 */
void update_led_display() {
    static uint8_t com_on = 0;
    static uint32_t last_change_time = 0;

    if (time_now() - last_change_time < 4) {
        return;
    }
    last_change_time = time_now();

    switch (com_on) {
        case 0:
            DL_GPIO_setPins(GPIOA, DISP_CTRL_DIG1_PIN);
            DL_GPIO_setPins(GPIOA,
                            DISP_CTRL_DISP_LED1_PIN |
                            DISP_CTRL_DISP_LED2_PIN |
                            DISP_CTRL_DISP_LED3_PIN |
                            DISP_CTRL_DISP_LED4_PIN |
                            DISP_CTRL_DISP_LED5_PIN);

            DL_GPIO_clearPins(GPIOB, DISP_CTRL_DIG2_PIN);
            displayChargeStorage();
            break;

        case 1:
            DL_GPIO_setPins(GPIOB, DISP_CTRL_DIG2_PIN);
            DL_GPIO_setPins(GPIOA,
                            DISP_CTRL_DISP_LED1_PIN |
                            DISP_CTRL_DISP_LED2_PIN |
                            DISP_CTRL_DISP_LED3_PIN |
                            DISP_CTRL_DISP_LED4_PIN |
                            DISP_CTRL_DISP_LED5_PIN);

            DL_GPIO_clearPins(GPIOA, DISP_CTRL_DIG1_PIN);
            displayCurrentPower();
            break;

        default:
            break;
    }

    com_on = (com_on + 1) % 2;
}

/* ============================================================================
 * BUTTON INTERFACE (EVENT DETECTION)
 * ============================================================================ */

Button BUTTONS[2] = {
    {
        .port = GPIOB,
        .pin = BTNS_BTN1_PIN,
        .state = 1,
        .wasPressed = false,
        .wasReleased = false,
        .shortPress = false,
        .longPress = false,
        .isHeld = false,
        .holdThreshold = 500
    },
    {
        .port = GPIOB,
        .pin = BTNS_BTN2_PIN,
        .state = 1,
        .wasPressed = false,
        .wasReleased = false,
        .shortPress = false,
        .longPress = false,
        .isHeld = false,
        .holdThreshold = 500
    },
};

/* Thin helpers for readability */
bool is_button_pressed(const Button *btn)  { return btn->wasPressed; }
bool is_button_released(const Button *btn) { return btn->wasReleased; }
bool is_button_held(const Button *btn)     { return btn->isHeld; }
bool is_short_press(const Button *btn)     { return btn->shortPress; }
bool is_long_press(const Button *btn)      { return btn->longPress; }

/*
 * Button state machine update:
 * Call periodically (~5–20ms) to detect press/release and short/long presses.
 */
void update_buttons(void)
{
    uint32_t now = time_now();

    for (uint8_t i = 0; i < 2; i++)
    {
        Button *btn = &BUTTONS[i];

        // Read pin: 1 = released, 0 = pressed (active-low input)
        uint8_t raw = (DL_GPIO_readPins(btn->port, btn->pin) == btn->pin);

        // Reset per-cycle events
        btn->wasPressed  = false;
        btn->wasReleased = false;
        btn->shortPress  = false;
        btn->longPress   = false;

        // PRESS EVENT (HIGH -> LOW)
        if (raw == 0 && btn->state == 1)
        {
            btn->wasPressed = true;
            btn->pressStartTime = now;
            btn->isHeld = false;
        }

        // LONG PRESS / HELD DETECTION
        if (raw == 0)
        {
            if (!btn->isHeld &&
                (now - btn->pressStartTime >= btn->holdThreshold))
            {
                btn->isHeld = true;
                btn->longPress = true;
            }
        }

        // RELEASE EVENT (LOW -> HIGH)
        if (raw == 1 && btn->state == 0)
        {
            btn->wasReleased = true;
            uint32_t duration = now - btn->pressStartTime;

            if (duration < btn->holdThreshold)
                btn->shortPress = true;
        }

        // Store new state
        btn->state = raw;
    }
}

/*
 * Handles button actions for LED brightness control:
 * - Long press: turn off ALL LED output
 * - Short press: cycle through predefined brightness steps
 */
void handle_button_input(void){
    update_buttons();

    if (BUTTONS[0].longPress) {
        set_led_voltage(0);

    }

    if (BUTTONS[1].longPress) {
        set_led_voltage(0);

    }

    if(BUTTONS[0].wasPressed){
        buttonPressed = true;
        set_led_voltage(9500);
        switch(led1_value){
            case LED1_VALUE_0:
                set_led_current(0, LED1);
                led1_value = LED1_VALUE_1;
                break;
            case LED1_VALUE_1:
                set_led_current(10,LED1);
                led1_value=LED1_VALUE_2;
                break;
            case LED1_VALUE_2:
                set_led_current(20, LED1);
                led1_value=LED1_VALUE_3;
                break;
            case LED1_VALUE_3:
                set_led_current(30, LED1);
                led1_value=LED1_VALUE_4;
                break;
            case LED1_VALUE_4:
                set_led_current(50, LED1);
                led1_value=LED1_VALUE_0;
                break;
            default:
                break;
        }
    }

    if(BUTTONS[1].wasPressed){
        buttonPressed = true;
        set_led_voltage(9500);
        switch(led2_value){
            case LED2_VALUE_0:
                set_led_current(0, LED2);
                led2_value = LED2_VALUE_1;
                break;
            case LED2_VALUE_1:
                set_led_current(10, LED2);
                led2_value=LED2_VALUE_2;
                break;
            case LED2_VALUE_2:
                set_led_current(20, LED2);
                led2_value=LED2_VALUE_3;
                break;
            case LED2_VALUE_3:
                set_led_current(30, LED2);
                led2_value=LED2_VALUE_4;
                break;
            case LED2_VALUE_4:
                set_led_current(50, LED2);
                led2_value=LED2_VALUE_0;
                break;
            default:
                break;
        }
    }
}

/* ============================================================================
 * LIGHT SWITCH / EDGE CAPTURE DECODER
 * ============================================================================ */

/*
 * This module decodes edge timing intervals into a 2-nibble payload.
 * It uses a timer capture input (B_INT_INST) and a watchdog timer
 * (B_INT_TIMER_INST) to enforce expected timing windows.
 */

volatile uint32_t decoded_value;

#define NIB_LOW_BOUND 7599
#define NIB_HIGH_BOUND 14399
#define INIT_LOW_BOUND 19599
#define INIT_HIGH_BOUND 20399
#define NIB_ZERO 20

typedef enum {
    INIT1,INIT2,INIT3, NIB1,NIB2
} DATA_PARTS;

DATA_PARTS flag = INIT1;

volatile int8_t led = -1;

volatile uint32_t t1, t2, dt = 0;
volatile uint8_t received_edges = 0;
volatile uint32_t value = 0;
volatile uint32_t decoded_value = 0;
volatile uint32_t wait_durations[5] = { INIT_HIGH_BOUND, INIT_HIGH_BOUND, INIT_HIGH_BOUND, NIB_HIGH_BOUND, NIB_HIGH_BOUND};

void reset_sw_receive(){
    t1 = 0;
    t2 = 0;
    dt = 0;
    flag = INIT1;
    received_edges = 0;
    value = 0;
}

/*
 * Arms the decode watchdog timer with the expected timeout for the current state.
 */
void run_sw_timer(){
    DL_TimerG_stopCounter(B_INT_TIMER_INST);
    DL_TimerG_setTimerCount(B_INT_TIMER_INST, (wait_durations[flag] * 10) + 9);
    DL_TimerG_startCounter(B_INT_TIMER_INST);
}

volatile int8_t _LED = -1;

uint8_t get_msg_led(){
    return _LED;
}

/*
 * Maps message "brightness index" to LED current in mA.
 * This provides fine control via external message encoding.
 */
static uint16_t brightness_mapping_fine_mA[11] = {
    0, 50, 100, 150, 200, 250, 300, 350, 400, 450, 500
};

/*
 * Edge ISR handler (expected to be called from capture interrupt).
 * Computes delta time between edges and advances decode state machine.
 */
void on_received_edge(){
    t2 = B_INT_INST_LOAD_VALUE - (DL_Timer_getCaptureCompareValue(B_INT_INST, DL_TIMER_CC_1_INDEX));

    if(received_edges > 0){
        if(t2 < t1){
            dt = ((t2 - t1 + B_INT_INST_LOAD_VALUE) * 150) / B_INT_INST_LOAD_VALUE;
        } else {
            dt = ((t2 - t1) * 150) / B_INT_INST_LOAD_VALUE;
        }

        switch(flag){
            case INIT1:
                receiving_msg_led = true;
                leds_1[0] = 0;
                leds_1[1] = 0;
                /* fallthrough intentional */

            case INIT2:
            case INIT3: {
                    if(dt >= 49 && dt <= 51){
                        receiving_b_int = false;
                        flag += 1;
                        t1 = t2;
                        received_edges++;
                        run_sw_timer();
                        receiving_b_int = true;
                        return;
                    }
                    return;
                }

            case NIB1:
                if(dt >= 19 && dt <= 36){
                    value = (dt - NIB_ZERO) << 4;
                    flag += 1;
                    t1 = t2;
                    received_edges++;
                    run_sw_timer();
                    receiving_msg_led = true;
                    return;
                }
                return;

            case NIB2:
                if(dt >= 19 && dt <= 36){
                    receiving_msg_led = false;
                    receiving_b_int = false;
                    value = value | (dt - NIB_ZERO);

                    // Message validity check: expect multiples of 25
                    if(value % 25 != 0) return;

                    decoded_value = value;
                    flag = INIT1;
                    t1 = t2 = 0;
                    received_edges = 0;

                    // Determine which LED line had the most activity during reception
                    uint16_t max = 0;
                    for(int i = 0; i < 4; i++){
                        if(leds_1[i] > max){
                            max = leds_1[i];
                            _LED = i;
                        }
                    }

                    // Apply brightness command
                    uint8_t index = (decoded_value) / 25;
                    set_led_current(brightness_mapping_fine_mA[index], _LED);

                    DL_TimerG_stopCounter(B_INT_TIMER_INST);
                    return;
                }
                return;

            default:
                break;
        }
    } else {
        t1 = t2;
        received_edges++;
        run_sw_timer();
        return;
    }
}

/*
 * Initializes light-switch capture interface and enables related interrupts.
 */
void l_sw_init(void){
    received_edges = 0;

    NVIC_ClearPendingIRQ(B_INT_INST_INT_IRQN);
    NVIC_EnableIRQ(B_INT_INST_INT_IRQN);
    NVIC_EnableIRQ(B_INT_TIMER_INST_INT_IRQN);
    DL_TimerG_startCounter(B_INT_INST);
}

uint32_t get_decoded_value(){
    return decoded_value;
}

/* ============================================================================
 * UART MODULE
 * ============================================================================ */

#define MAX_BUFFER_SIZE 64

char UART_Buffer[MAX_BUFFER_SIZE];
volatile bool data_received = false;
volatile uint8_t char_index = 0;

/*
 * invokeBSLAsm():
 * Jumps to the Bootloader (BSL) after clearing SRAM areas.
 * This is a safety measure to ensure clean handoff.
 */
__STATIC_INLINE void invokeBSLAsm(void)
{
    __asm(
#if defined(__GNUC__)
        ".syntax unified\n"
#endif
        "ldr     r4, = 0x41C40018\n"
        "ldr     r4, [r4]\n"
        "ldr     r1, = 0x03FF0000\n"
        "ands    r4, r1\n"
        "lsrs    r4, r4, #6\n"
        "ldr     r1, = 0x20300000\n"
        "adds    r2, r4, r1\n"
        "movs    r3, #0\n"
        "init_ecc_loop: \n"
        "str     r3, [r1]\n"
        "adds    r1, r1, #4\n"
        "cmp     r1, r2\n"
        "blo     init_ecc_loop\n"
        "ldr     r1, = 0x20200000\n"
        "adds    r2, r4, r1\n"
        "movs    r3, #0\n"
        "init_data_loop:\n"
        "str     r3, [r1]\n"
        "adds    r1, r1, #4\n"
        "cmp     r1, r2\n"
        "blo     init_data_loop\n"
        "str     %[resetLvlVal], [%[resetLvlAddr], #0x00]\n"
        "str     %[resetCmdVal], [%[resetCmdAddr], #0x00]"
        : 
        : [ resetLvlAddr ] "r"(&SYSCTL->SOCLOCK.RESETLEVEL),
        [ resetLvlVal ] "r"(DL_SYSCTL_RESET_BOOTLOADER_ENTRY),
        [ resetCmdAddr ] "r"(&SYSCTL->SOCLOCK.RESETCMD),
        [ resetCmdVal ] "r"(
            SYSCTL_RESETCMD_KEY_VALUE | SYSCTL_RESETCMD_GO_TRUE)
        : "r1", "r2", "r3", "r4");
}

void uart_init(void){
    data_received = false;
    NVIC_ClearPendingIRQ(UART_0_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_0_INST_INT_IRQN);
}

void printf_user(char* string, uint8_t string_length, char end_char){
    for(uint8_t i = 0; i<string_length; i++){
        if(string[i] == end_char){
            putchar(string[i]);
            break;
        }
        putchar(string[i]);
    }
}

/*
 * Blocking UART transmit helper.
 * Sends until end_char is encountered.
 */
void printToUART(char* string, char end_char){
    uint32_t i = 0;
    while(1){
        DL_UART_Main_transmitDataBlocking(UART_0_INST, string[i]);
        i++;
        if(string[i] == end_char){
            break;
        }
    }
}

/*
 * UART receive handler:
 * Buffers input until newline / terminator or buffer limit.
 * Special case: receiving 0x22 triggers BSL entry if battery is low enough.
 */
void UARTReceive(){
    if(data_received == true){
        DL_UART_receiveData(UART_0_INST);
        return;
    }

    char c = DL_UART_receiveData(UART_0_INST);

    if (c == 0x22) {
        if(get_battery_voltage() > 3000){
            return;
        }
        invokeBSLAsm();
        return;
    }

    if(c == '\r' && char_index == 0) return;

    UART_Buffer[char_index] = c;
    char_index = char_index + 1;

    if(char_index > MAX_BUFFER_SIZE - 2 || c == '\n' || c == '~'){
        UART_Buffer[char_index] = '\0';
        data_received = true;
        char_index = 0;
    }
}

void get_UART_buffer(char* output_buffer){
    strcpy(output_buffer, UART_Buffer);
    memset(UART_Buffer, 0, MAX_BUFFER_SIZE);
    data_received = false;
}

/* ============================================================================
 * SYSTEM TIMEBASE (SYSTICK COUNTER)
 * ============================================================================ */

volatile uint32_t timestamp = 0;

void timer_init(void){
    timestamp = 0;
}

void update_timestamp(void){
    timestamp++;
}

uint32_t time_now(void){
    return timestamp;
}

/* ============================================================================
 * OUTPUT CONTROL
 * ============================================================================ */

void turn_on_leds(){
    set_led_voltage(9500);
}

void turn_on_outputs(void){
    enable_led_boost();
    enable_usb_boost();
}

void turn_off_outputs(void){
    disable_led_boost();
    disable_usb_boost();
}



