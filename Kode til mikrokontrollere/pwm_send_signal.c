#include <stdbool.h>
#include <stdint.h>
#include "nrf.h"
#include "app_error.h"
#include "bsp.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "nrf_gpiote.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrfx_pwm.h"
#include "nrf_pwm.h"
#include "pwm_send_signal.h"

#define D2    NRF_GPIO_PIN_MAP(0,5)
//#include "app.h"

static uint16_t number_of_pulses_sendth = 10; // Antall pulser som skal sendes ut hver gang send_pwm_signal() blir kalt

// Verdiene som skal sendes ut: 100% og 0% på. 
static nrf_pwm_values_common_t seq_values[] =
{
    0, 100
}; // Pay attention that it is INVERSE LOGICAL in the sequence (ex: 1= 99%, 90 = 10%)
                                      
// Her blir pulssekvensen definert. 
nrf_pwm_sequence_t const seq =
{
    .values.p_common = seq_values,
    .length          = NRF_PWM_VALUES_LENGTH(seq_values),
    .repeats         = 0,
    .end_delay       = 0
};
/** Is PWM Initialized */
static bool m_initialized = false;
static volatile bool ready_flag_pwm;            // A flag indicating PWM status.

nrfx_pwm_config_t pwm1_cfg = {
        .output_pins = {D2, NRFX_PWM_PIN_NOT_USED, NRFX_PWM_PIN_NOT_USED}, 
        .irq_priority = APP_IRQ_PRIORITY_MID, 
        .base_clock   = NRF_PWM_CLK_2MHz, 
        .load_mode    = NRF_PWM_LOAD_COMMON, 
        .top_value    = 23,                 // Her velges antall klokke-ticks for hver periode. 23 ticks på 2 MHz klokke approx 44 kHz
        .step_mode    = NRF_PWM_STEP_AUTO,
        .count_mode   = NRF_PWM_MODE_UP
    }; 
static nrfx_pwm_t PWM1 = NRFX_PWM_INSTANCE(0);                   // Create the instance "PWM1" using TIMER1.



void init_pwm(void){
    ret_code_t err_code;
    err_code = nrfx_pwm_init(&PWM1, &pwm1_cfg, NULL);
    APP_ERROR_CHECK(err_code);
}

// Funksjon for sending av pulssignal
void send_pwm_signal(void){
    nrfx_pwm_simple_playback(&PWM1, &seq, number_of_pulses_sendth, 0);
   // nrfx_pwm_stop(&PWM1, 0);
}
