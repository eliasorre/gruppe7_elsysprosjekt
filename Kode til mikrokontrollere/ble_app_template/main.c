/**
 * Copyright (c) 2014 - 2020, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/** @file
 *
 * @defgroup ble_sdk_app_template_main main.c
 * @{
 * @ingroup ble_sdk_app_template
 * @brief Template project main file.
 *
 * This file contains a template for creating a new application. It has the code necessary to wakeup
 * from button, advertise, get a connection restart advertising on disconnect and if no new
 * connection created go back to system-off mode.
 * It can easily be used as a starting point for creating a new application, the comments identified
 * with 'YOUR_JOB' indicates where and how you can customize.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "app_timer.h"
#include "fds.h"
#include "peer_manager.h"
#include "peer_manager_handler.h"
#include "bsp_btn_ble.h"
#include "sensorsim.h"
#include "ble_conn_state.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "nrf_pwr_mgmt.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "time_sync.h"
#include "nrf_ppi.h"
#include "nrf_timer.h"
#include "nrf_gpiote.h"
#include "saadc.h"
#include "app_uart.h"
#include "ble_db_discovery.h"
#include "app_timer.h"
#include "app_util.h"
#include "bsp_btn_ble.h"
#include "ble_nus_c.h"
#include "ble_gap.h"
#include "nrf_ble_scan.h"
#include "nrf_gpiote.h"
#include "nrf_delay.h"
#include "pwm_send_signal.h"

#include "nrf_drv_comp.h"

#include "advertising.h"


#define DEAD_BEEF                       0xDEADBEEF                              /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */
#define DEVICE_NAME "asdf"

uint32_t t_0 = 0; 
uint32_t t_1 = 0;
uint32_t count_1 = 0;
static bool m_gpio_trigger_enabled;
static bool signal_on = 0; 

static void ts_evt_callback(const ts_evt_t* evt);
static void ts_gpio_trigger_enable(void);
static void ts_gpio_trigger_disable(void);

void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(0xDEADBEEF, line_num, p_file_name);
}

static bool shutdown_handler(nrf_pwr_mgmt_evt_t event)
{
    ret_code_t err_code;

    err_code = bsp_indication_set(BSP_INDICATE_IDLE);
    APP_ERROR_CHECK(err_code);

    switch (event)
    {
        case NRF_PWR_MGMT_EVT_PREPARE_WAKEUP:
            // Prepare wakeup buttons.
            err_code = bsp_btn_ble_sleep_mode_prepare();
            APP_ERROR_CHECK(err_code);
            break;

        default:
            break;
    }

    return true;
}

NRF_PWR_MGMT_HANDLER_REGISTER(shutdown_handler, APP_SHUTDOWN_HANDLER_PRIORITY);

void bsp_event_handler(bsp_event_t event)
{
    uint32_t err_code;
    switch (event)
    {
        case BSP_EVENT_KEY_0:
            {
                static bool m_send_sync_pkt = false;

                if (m_send_sync_pkt)
                {
                    m_send_sync_pkt = false;
                    m_gpio_trigger_enabled = false;

                    bsp_board_leds_off();

                    err_code = ts_tx_stop();
                    APP_ERROR_CHECK(err_code);

                    NRF_LOG_INFO("Stopping sync beacon transmission!\r\n");
                }
                else
                {
                    m_send_sync_pkt = true;

                    bsp_board_leds_on();

                    err_code = ts_tx_start(23);
                    APP_ERROR_CHECK(err_code);

                    ts_gpio_trigger_enable();

                    NRF_LOG_INFO("Starting sync beacon transmission!\r\n");
                }
            }
            break;

        case BSP_EVENT_KEY_1:
          {
            //print_timestamp();
            send_pwm_signal();
          }
          break;
        case BSP_EVENT_KEY_2:
        {
            uint64_t time_ticks;
            uint32_t time_usec;

            uint32_t timer_val;
            uint32_t counter_val;
            uint32_t peer_counter_val;

            get_timestamp(&timer_val, &counter_val, &peer_counter_val);
            advertising_update(counter_val + peer_counter_val, timer_val);

            time_ticks = ts_timestamp_get_ticks_u64();
            time_usec = TIME_SYNC_TIMESTAMP_TO_USEC(time_ticks);

            NRF_LOG_INFO("Timestamp: %d us (%d, %d)", time_usec, time_usec / 1000000, time_usec / 1000);
            break;
        }
        case BSP_EVENT_KEY_3:
        {
            if (!signal_on){
              NVIC_EnableIRQ(TIMER3_IRQn);
              signal_on = 1;
            }
            else {
              NVIC_DisableIRQ(TIMER3_IRQn);
              signal_on = 0;
            }      
        }

        default:
            break;
    }
}


/**@brief Function for initializing buttons and leds.
 *
 * @param[out] p_erase_bonds  Will be true if the clear bonding button was pressed to wake the application up.
 */
/**@brief Function for initializing buttons and leds. */
static void buttons_leds_init(void)
{
    ret_code_t err_code;
    bsp_event_t startup_event;

    err_code = bsp_init(BSP_INIT_LEDS | BSP_INIT_BUTTONS, bsp_event_handler);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_btn_ble_init(NULL, &startup_event);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing the nrf log module.
 */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}


/**@brief Function for initializing power management.
 */
static void power_management_init(void)
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the idle state (main loop).
 *
 * @details Handles any pending log operations, then sleeps until the next event occurs.
 */
static void idle_state_handle(void)
{
    if (NRF_LOG_PROCESS() == false)
    {
        nrf_pwr_mgmt_run();
    }
}


static void ts_gpio_trigger_enable(void)
{
    uint64_t time_now_ticks;
    uint32_t time_now_msec;
    uint32_t time_target;
    uint32_t err_code;

    if (m_gpio_trigger_enabled)
    {
        return;
    }

    // Round up to nearest second to next 250 ms to start toggling.
    // If the receiver has received a valid sync packet within this time, the GPIO toggling polarity will be the same.

    time_now_ticks = ts_timestamp_get_ticks_u64();
    time_now_msec = TIME_SYNC_TIMESTAMP_TO_USEC(time_now_ticks) / 1000;

    time_target = TIME_SYNC_MSEC_TO_TICK(time_now_msec) + (250 * 2);
    time_target = (time_target / 250) * 250;

    err_code = ts_set_trigger(time_target, nrf_gpiote_task_addr_get(NRF_GPIOTE_TASKS_OUT_3));
    APP_ERROR_CHECK(err_code);

    nrf_gpiote_task_set(NRF_GPIOTE_TASKS_CLR_3);

    m_gpio_trigger_enabled = true;
}

static void ts_gpio_trigger_disable(void)
{
    m_gpio_trigger_enabled = false;
}

static void ts_evt_callback(const ts_evt_t* evt)
{
    APP_ERROR_CHECK_BOOL(evt != NULL);
    switch (evt->type)
    {
        case TS_EVT_SYNCHRONIZED:
            ts_gpio_trigger_enable();
            break;
        case TS_EVT_DESYNCHRONIZED:
            ts_gpio_trigger_disable();
            break;
        case TS_EVT_TRIGGERED:
            if (m_gpio_trigger_enabled)
            {
                uint32_t tick_target;

                tick_target = evt->params.triggered.tick_target + 2;

                uint32_t err_code = ts_set_trigger(tick_target, nrf_gpiote_task_addr_get(NRF_GPIOTE_TASKS_OUT_3));
                APP_ERROR_CHECK(err_code);
            }
            else
            {
                // Ensure pin is low when triggering is stopped
                nrf_gpiote_task_set(NRF_GPIOTE_TASKS_CLR_3);
            }
            break;
        default:
            APP_ERROR_CHECK_BOOL(false);
            break;
    }
}

/**@brief Function for initializing the timer. */
static void timer_init(void)
{
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
}

static void sync_timer_init(void)
{
    uint32_t err_code;

    // Debug pin:
    // nRF52-DK (PCA10040) Toggle P0.24 from sync timer to allow pin measurement
    // nRF52840-DK (PCA10056) Toggle P1.14 from sync timer to allow pin measurement
#if defined(BOARD_PCA10040)
    nrf_gpiote_task_configure(3, NRF_GPIO_PIN_MAP(0, 24), NRF_GPIOTE_POLARITY_TOGGLE, NRF_GPIOTE_INITIAL_VALUE_LOW);
    nrf_gpiote_task_enable(3);
#elif defined(BOARD_PCA10056)
    nrf_gpiote_task_configure(3, NRF_GPIO_PIN_MAP(1, 14), NRF_GPIOTE_POLARITY_TOGGLE, NRF_GPIOTE_INITIAL_VALUE_LOW);
    nrf_gpiote_task_enable(3);
#else
#warning Debug pin not set
#endif

    ts_init_t init_ts =
    {
        .high_freq_timer[0] = NRF_TIMER3,
        .high_freq_timer[1] = NRF_TIMER4,
        .egu                = NRF_EGU3,
        .egu_irq_type       = SWI3_EGU3_IRQn,
        .evt_handler        = ts_evt_callback,
    };

    err_code = ts_init(&init_ts);
    APP_ERROR_CHECK(err_code);

	ts_rf_config_t rf_config =
	{
		.rf_chn = 80,
		.rf_addr = { 0xDE, 0xAD, 0xBE, 0xEF, 0x19 }
	};

    err_code = ts_enable(&rf_config);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_INFO("Started listening for beacons.\r\n");
    NRF_LOG_INFO("Press Button 1 to start transmitting sync beacons\r\n");
    NRF_LOG_INFO("GPIO toggling will begin when transmission has started.\r\n");
}

static volatile uint32_t voltage_falls_detected = 0;
static volatile uint32_t voltage_falls_total    = 0;
/**
 * @brief COMP event handler is called when COMP detects voltage drop.
 *
 * This function is called from interrupt context so it is very important
 * to return quickly. Don't put busy loops or any other CPU intensive actions here.
 * It is also not allowed to call soft device functions from it (if LPCOMP IRQ
 * priority is set to APP_IRQ_PRIORITY_HIGH).
 */
static void comp_event_handler(nrf_comp_event_t event)
{
    if (event == NRF_COMP_EVENT_DOWN)  // Trigger når AI2 (P0.05) går fra høy til lav 
    {
        uint64_t time_ticks;
        uint32_t time_usec;

        uint32_t timer_val;
        uint32_t counter_val;
        uint32_t peer_counter_val;
        
        //NRF_LOG_INFO("TRIG");
        nrf_drv_comp_stop();   
        get_timestamp(&timer_val, &counter_val, &peer_counter_val);    // Henter de interne klokkeverdiene
        advertising_update(counter_val + peer_counter_val, timer_val);  // Her vil det sendes ut advertising_pakke med de hentede verdiene fra den interne klokke
        /*NRF_LOG_INFO("Counter val: %d", counter_val + peer_counter_val); 
        NRF_LOG_INFO("Timer val:  %d", timer_val);*/
     }
}
/**
/**
 * @brief Initialize COMP driver.
 */
static void comp_init(void)
{
    uint32_t                err_code;
    nrf_drv_comp_config_t comp_config;
    memset(&comp_config, 0, sizeof(comp_config));
    comp_config.hyst = NRF_COMP_HYST_50mV;
    //comp_config.ext_ref = NRF_COMP_EXT_REF_1;
    comp_config.input    = NRF_COMP_INPUT_2;
    comp_config.interrupt_priority   =   NRFX_COMP_CONFIG_IRQ_PRIORITY;
    comp_config.main_mode            =   NRF_COMP_MAIN_MODE_SE;
    comp_config.speed_mode           =   NRF_COMP_SP_MODE_High;
    comp_config.reference            =   NRF_COMP_REF_Int2V4;
    comp_config.threshold.th_down    =   VOLTAGE_THRESHOLD_TO_INT(0.5, 1.8);  // Terskelverdi for at comp_event down 
    comp_config.threshold.th_up      =   VOLTAGE_THRESHOLD_TO_INT(1.5, 1.8);  // - || - comp_event_up

    err_code = nrf_drv_comp_init(&comp_config, comp_event_handler);
    APP_ERROR_CHECK(err_code);
    nrf_drv_comp_start(NRF_DRV_COMP_EVT_EN_DOWN_MASK,0); // Starter comp-modulen, og vil intterrupt skal trigge på Høy-til-lav
}

void device_name_init(void)
{
  ret_code_t err_code;
  ble_gap_conn_sec_mode_t sec_mode;

  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

  err_code = sd_ble_gap_device_name_set(&sec_mode,
                                        (const uint8_t *)DEVICE_NAME,
                                        strlen(DEVICE_NAME));
  APP_ERROR_CHECK(err_code);
}



/**@brief Function for application main entry.
 */
int main(void)
{
   // Initialize.
    //log_init();
    
    timer_init();
    log_init();
    buttons_leds_init();
    power_management_init();
    ble_stack_init();
    device_name_init();
    sync_timer_init();


     
    // Klokkesynkroniseringsmodulen
        uint32_t err_code;
        static bool m_send_sync_pkt;
        m_send_sync_pkt = true;

        bsp_board_leds_on();          
        err_code = ts_tx_start(23);   // Starter klokkesynkroniseringen med en oppdateringsfrekvens på 23 Hz
        APP_ERROR_CHECK(err_code);

        ts_gpio_trigger_enable();
        nrf_delay_ms(50);
        init_pwm();
        NRF_LOG_INFO("Curling tracking activatet. This is master");


    // Mottakermodul
        //comp_init();
        //advertising_init();
        //advertising_start();
            
    // Start execution.
    NVIC_EnableIRQ(TIMER3_IRQn);
    // Enter main loop.
    for (;;)
    {
        idle_state_handle();
    }
}


/**
 * @}
 */