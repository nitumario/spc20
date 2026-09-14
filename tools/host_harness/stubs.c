#include "stubs.h"
#include "system_types.h"
#include "fault_mgr.h"
#include "energy_mode.h"

uint32_t h_now = 1000;
uint16_t h_hw_pwm = 399;
bool     h_q49 = false;
bool     h_buck_en = false;
uint32_t h_seq = 0;
uint16_t h_raw_vpanel = 12900;
int16_t  h_raw_ibuck = 0;
uint16_t h_raw_vbat = 3320;
uint16_t h_raw_vchg = 3320;
uint16_t h_trace[ADC_PANEL_TRACE_DEPTH];
uint16_t h_faults_raised = 0;

uint32_t time_now(void) { return h_now; }
uint16_t get_input_voltage_now(void) { return h_raw_vpanel; }
uint16_t get_battery_voltage_now(void) { return h_raw_vbat; }
uint16_t get_charge_voltage_now(void) { return h_raw_vchg; }
int16_t  get_charge_current_now(void) { return h_q49 ? h_raw_ibuck : 0; }
uint16_t get_discharge_current_now(void) { return 0; }
uint32_t adc_sample_seq(void) { return h_seq; }
void     adc_panel_trace_mv(uint16_t *out) { for (unsigned i = 0; i < ADC_PANEL_TRACE_DEPTH; i++) out[i] = h_trace[i]; }
void     set_buck_pwm(uint16_t pwm_value) { h_hw_pwm = pwm_value; }
void     enable_charge_switch(void) { h_q49 = true; }
void     disable_charge_switch(void) { h_q49 = false; }
void     enable_input_buck(void) { h_buck_en = true; }
void     disable_input_buck(void) { h_buck_en = false; }
uint16_t lookup_charging_pwm(uint16_t voltage) { (void)voltage; return 121; }
uint16_t set_charging_voltage(uint16_t voltage) { h_hw_pwm = lookup_charging_pwm(voltage); return h_hw_pwm; }

void fault_raise(system_ctx_t *ctx, uint16_t fault_bit) { ctx->fault.code |= fault_bit; ctx->fault.history |= fault_bit; h_faults_raised++; }
bool safe_mode_rescue_active(const system_ctx_t *ctx) { (void)ctx; return false; }
