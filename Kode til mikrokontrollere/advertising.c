#include <stdbool.h>
#include <stdint.h>
#include "nordic_common.h"
#include "bsp.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "ble_advdata.h"
#include "ble_advertising.h"

#include "advertising.h"


#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"


#define MS_DATA_LENGTH 0x08

#define APP_BLE_CONN_CFG_TAG            1                                  /**< A tag identifying the SoftDevice BLE configuration. */

#define NON_CONNECTABLE_ADV_INTERVAL    MSEC_TO_UNITS(100, UNIT_0_625_MS)  /**< The advertising interval for non-connectable advertisement (100 ms). This value can vary between 100ms to 10.24s). */
#define APP_COMPANY_IDENTIFIER 0x0059


//static ble_advertising_t m_advertising;

static ble_gap_adv_params_t m_adv_params;                                  /**< Parameters to be passed to the stack when starting advertising. */
static uint8_t              m_adv_handle = BLE_GAP_ADV_SET_HANDLE_NOT_SET; /**< Advertising handle used to identify an advertising set. */
static uint8_t              m_enc_advdata[BLE_GAP_ADV_SET_DATA_SIZE_MAX];  /**< Buffer for storing an encoded advertising set. */


/**@brief Struct that contains pointers to the encoded advertising data. */
/*
static ble_gap_adv_data_t m_adv_data =
{
    .adv_data =
    {
        .p_data = m_enc_advdata,
        .len    = BLE_GAP_ADV_SET_DATA_SIZE_MAX // Burde oppdateres til faktisk lengde, og ikke max
    },
    .scan_rsp_data =
    {
        .p_data = NULL,
        .len    = 0

    }
};
*/

static uint8_t m_enc_advdata_pingpong[2][BLE_GAP_ADV_SET_DATA_SIZE_MAX];

static ble_gap_adv_data_t m_adv_data_pingpong[2] =
{
  {
      .adv_data =
      {
          .p_data = m_enc_advdata_pingpong[0],
          .len    = BLE_GAP_ADV_SET_DATA_SIZE_MAX // Burde oppdateres til faktisk lengde, og ikke max
      },
      .scan_rsp_data =
      {
          .p_data = NULL,
          .len    = 0

      }
  },
  {
    .adv_data =
    {
        .p_data = m_enc_advdata_pingpong[1],
        .len    = BLE_GAP_ADV_SET_DATA_SIZE_MAX // Burde oppdateres til faktisk lengde, og ikke max
    },
    .scan_rsp_data =
    {
        .p_data = NULL,
        .len    = 0

    }
  }
};

static ble_advdata_t advdata;
static ble_advdata_manuf_data_t manuf_specific_data;

void advertising_update(uint32_t major_val, uint32_t minor_val)
{
  static uint32_t i = 0;
  i = (++i % 2);

  ret_code_t err_code;
  uint8_t ms_data[MS_DATA_LENGTH];

  uint32_big_encode(
    major_val, 
    &ms_data[0]
  );

  uint32_big_encode(
    minor_val, 
    &ms_data[4]
  );

  manuf_specific_data.data.p_data = (uint8_t *) ms_data;

  ble_advdata_encode(&advdata, m_adv_data_pingpong[i].adv_data.p_data, &m_adv_data_pingpong[i].adv_data.len);
  err_code = sd_ble_gap_adv_set_configure(&m_adv_handle, &m_adv_data_pingpong[i], NULL);
  APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing the Advertising functionality.
 *
 * @details Encodes the required advertising data and passes it to the stack.
 *          Also builds a structure to be passed to the stack when starting advertising.
 */
void advertising_init(void)
{
    uint32_t      err_code;
    uint8_t       flags = BLE_GAP_ADV_FLAG_BR_EDR_NOT_SUPPORTED;

    uint8_t ms_data[MS_DATA_LENGTH] = 
    {
      0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00,
    };

    manuf_specific_data.company_identifier = APP_COMPANY_IDENTIFIER;
    manuf_specific_data.data.p_data = (uint8_t *) ms_data;  // Denne variabelen bør være den eneste en trenger å endre
    manuf_specific_data.data.size = MS_DATA_LENGTH;

    // Build and set advertising data.
    memset(&advdata, 0, sizeof(advdata));

    advdata.name_type             = BLE_ADVDATA_FULL_NAME;
    advdata.flags                 = flags;
    advdata.p_manuf_specific_data = &manuf_specific_data;

    // Initialize advertising parameters (used when starting advertising).
    memset(&m_adv_params, 0, sizeof(m_adv_params));

    m_adv_params.properties.type = BLE_GAP_ADV_TYPE_NONCONNECTABLE_NONSCANNABLE_UNDIRECTED;
    m_adv_params.p_peer_addr     = NULL;    // Undirected advertisement.
    m_adv_params.filter_policy   = BLE_GAP_ADV_FP_ANY;
    m_adv_params.interval        = NON_CONNECTABLE_ADV_INTERVAL;
    m_adv_params.duration        = 0;       // Never time out.

    for (int i = 0; i < 2; ++i)
    {
      err_code = ble_advdata_encode(&advdata, m_adv_data_pingpong[i].adv_data.p_data, &m_adv_data_pingpong[i].adv_data.len);
      APP_ERROR_CHECK(err_code);
    }

    err_code = sd_ble_gap_adv_set_configure(&m_adv_handle, &m_adv_data_pingpong[0], &m_adv_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for starting advertising.
 */
void advertising_start(void)
{
    ret_code_t err_code;

    err_code = sd_ble_gap_adv_start(m_adv_handle, APP_BLE_CONN_CFG_TAG);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
void ble_stack_init(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);
}