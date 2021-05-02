
#include <stdbool.h>
#include <stdint.h>
#include "nrf.h"
#include "app_error.h"
#include "bsp.h"
#include "nrf_delay.h"
#include "app_pwm.h"
#include "nrf_gpio.h"
#include "nrf_gpiote.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

//long periodUs = 23; 
static volatile bool ready_flag_pwm;            // A flag indicating PWM status.


#define D2    NRF_GPIO_PIN_MAP(0,5)

void init_pwm(void);
void send_pwm_signal(void);