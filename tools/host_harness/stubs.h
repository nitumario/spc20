/* Host stubs standing in for SPCBoardAPI.h (the real header pulls in the TI
 * driverlib). Compiled with -DSPCBOARDAPI_H so the real header is skipped. */
#ifndef STUBS_H
#define STUBS_H
#include <stdint.h>
#include <stdbool.h>
#include "hw_config.h"

uint32_t time_now(void);
uint16_t get_input_voltage_now(void);
uint16_t get_battery_voltage_now(void);
uint16_t get_charge_voltage_now(void);
int16_t  get_charge_current_now(void);
uint16_t get_discharge_current_now(void);
uint32_t adc_sample_seq(void);
void     adc_panel_trace_mv(uint16_t *out);
void     set_buck_pwm(uint16_t pwm_value);
void     enable_charge_switch(void);
void     disable_charge_switch(void);
void     enable_input_buck(void);
void     disable_input_buck(void);
uint16_t set_charging_voltage(uint16_t voltage);
uint16_t lookup_charging_pwm(uint16_t voltage);

/* harness-visible model state */
extern uint32_t h_now;
extern uint16_t h_hw_pwm;
extern bool     h_q49;
extern bool     h_buck_en;
extern uint32_t h_seq;
extern uint16_t h_raw_vpanel;
extern int16_t  h_raw_ibuck;
extern uint16_t h_raw_vbat;
extern uint16_t h_raw_vchg;
extern uint16_t h_trace[ADC_PANEL_TRACE_DEPTH];
extern uint16_t h_faults_raised;
#endif
