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
// Board/nrf6310/ble/ble_app_hrs_rtx/main.c
/**
 *
 * @brief Heart Rate Service Sample Application with RTX main file.
 *
 * This file contains the source code for a sample application using RTX and the
 * Heart Rate service (and also Battery and Device Information services).
 * This application uses the @ref srvlib_conn_params module.
 */

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
#include "ble_bas.h"

#include "ble_dis.h"
#include "ble_conn_params.h"

#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "nrf_sdh_freertos.h"
#include "app_timer.h"
#include "peer_manager.h"
#include "peer_manager_handler.h"
#include "bsp_btn_ble.h"
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"
#include "fds.h"
#include "ble_conn_state.h"
#include "nrf_drv_clock.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "nrf_power.h"


#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"


#include "app_times.h"
#include "app_config.h"

#include "ble_nus_c.h"
#include "nrf_ble_scan.h"
#include "app_system.h"
#include "app_dis_scan.h"
#include "app_display.h"
#include "app_protocol.h"

#include "app_util.h"
#include "app_usbd_core.h"
#include "app_usbd.h"
#include "app_usbd_string_desc.h"
#include "app_usbd_cdc_acm.h"
#include "app_usbd_serial_num.h"
#include "misc.h"
#include "app_ui.h"
#include "bsp_adc.h"
#include "nrf.h"

#define DEVICE_NAME                         "Nordic_HRo"                            /**< Name of device. Will be included in the advertising data. */
#define MANUFACTURER_NAME                   "NordicSemiconductor"                   /**< Manufacturer. Will be passed to Device Information Service. */
#define HW_VERSION                          "V0.1.0"
#define FW_VERSION                          "V17.0.2"
#define SW_VERSION                          "V0.0.1"
#define APP_BLE_OBSERVER_PRIO               3                                       /**< Application's BLE observer priority. You shouldn't need to modify this value. */
#define APP_BLE_CONN_CFG_TAG                1                                       /**< A tag identifying the SoftDevice BLE configuration. */

#define APP_ADV_INTERVAL                    160                                     /**< The advertising interval (in units of 0.625 ms. This value corresponds to 187.5 ms). */
#define APP_ADV_DURATION                    18000                                       /**< The advertising duration (180 seconds) in units of 10 milliseconds. */

#define APP_ADV_SLOW_INTERVAL               1600


#define MIN_CONN_INTERVAL                   MSEC_TO_UNITS(40, UNIT_1_25_MS)        /**< Minimum acceptable connection interval (0.4 seconds). */
#define MAX_CONN_INTERVAL                   MSEC_TO_UNITS(80, UNIT_1_25_MS)        /**< Maximum acceptable connection interval (0.65 second). */
#define SLAVE_LATENCY                       0                                       /**< Slave latency. */
#define CONN_SUP_TIMEOUT                    MSEC_TO_UNITS(4000, UNIT_10_MS)         /**< Connection supervisory time-out (4 seconds). */

#define FIRST_CONN_PARAMS_UPDATE_DELAY      5000                                    /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY       30000                                   /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT        3                                       /**< Number of attempts before giving up the connection parameter negotiation. */

#define SEC_PARAM_BOND                      1                                       /**< Perform bonding. */
#define SEC_PARAM_MITM                      0                                       /**< Man In The Middle protection not required. */
#define SEC_PARAM_LESC                      0                                       /**< LE Secure Connections not enabled. */
#define SEC_PARAM_KEYPRESS                  0                                       /**< Keypress notifications not enabled. */
#define SEC_PARAM_IO_CAPABILITIES           BLE_GAP_IO_CAPS_NONE                    /**< No I/O capabilities. */
#define SEC_PARAM_OOB                       0                                       /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE              7                                       /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE              16                                      /**< Maximum encryption key size. */

#define DEAD_BEEF                           0xDEADBEEF                              /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define OSTIMER_WAIT_FOR_QUEUE              2                                       /**< Number of ticks to wait for the timer queue to be ready */

/**
 * The size of the stack for the Logger task (in 32-bit words).
 * Logger uses sprintf internally so it is a rather stack hungry process.
 */
#define LOGGER_STACK_SIZE 512
/**
 * The size of the stack for the USB task (in 32-bit words).
 */
#define USBD_STACK_SIZE   256
/**
 * The priority of the Logger task.
 */
#define LOGGER_PRIORITY 1
/**
 * The priority of the USBD task.
 */
#define USBD_PRIORITY   2
/**
 * The maximum delay inside the USB task to wait for an event.
 */
#define USB_THREAD_MAX_BLOCK_TIME portMAX_DELAY



BLE_BAS_DEF(m_bas);                                                 /**< Battery service instance. */                                                /**< Heart rate service instance. */
NRF_BLE_GATT_DEF(m_gatt);                                           /**< GATT module instance. */
NRF_BLE_QWR_DEF(m_qwr);                                             /**< Context for the Queued Write module.*/
BLE_ADVERTISING_DEF(m_advertising);                                 /**< Advertising module instance. */
BLE_DB_DISCOVERY_DEF(m_db_disc);
NRF_BLE_SCAN_DEF(m_scan);
BLE_NUS_C_DEF(m_ble_nus_c);
NRF_BLE_GQ_DEF(m_ble_gatt_queue,                                        /**< BLE GATT Queue instance. */
               NRF_SDH_BLE_CENTRAL_LINK_COUNT,
               NRF_BLE_GQ_QUEUE_SIZE);
static uint16_t m_conn_handle         = BLE_CONN_HANDLE_INVALID;    /**< Handle of the current connection. */
static unsigned char m_target_periph_name[11]="AMSU_E7CA7";
static ble_uuid_t m_adv_uuids[] =                                   /**< Universally unique service identifiers. */
{

    {BLE_UUID_DEVICE_INFORMATION_SERVICE, BLE_UUID_TYPE_BLE}
};
static ble_gap_scan_params_t m_scan_param =                 /**< Scan parameters requested for scanning and connection. */
{
    .active        = 0x01,
    .interval      = NRF_BLE_SCAN_SCAN_INTERVAL,
    .window        = NRF_BLE_SCAN_SCAN_WINDOW,
    .filter_policy = BLE_GAP_SCAN_FP_ACCEPT_ALL,
    .timeout       = 0,//1000,   //NRF_BLE_SCAN_SCAN_DURATION  10ms  1000
    .scan_phys     = BLE_GAP_PHY_1MBPS,
    .extended      = true,
};
ble_gap_addr_t p_addr[5];
ble_gap_scan_params_t  p_s_param[5];
int update_periph_db=0;


static TaskHandle_t m_usbd_thread;                                  /**< USB stack thread. */
#if NRF_LOG_ENABLED
static TaskHandle_t m_logger_thread;                                /**< Definition of Logger thread. */
#endif

// USB DEFINES START
static void cdc_acm_user_ev_handler(app_usbd_class_inst_t const * p_inst,
                                    app_usbd_cdc_acm_user_event_t event);

#define CDC_ACM_COMM_INTERFACE  0
#define CDC_ACM_COMM_EPIN       NRF_DRV_USBD_EPIN2

#define CDC_ACM_DATA_INTERFACE  1
#define CDC_ACM_DATA_EPIN       NRF_DRV_USBD_EPIN1
#define CDC_ACM_DATA_EPOUT      NRF_DRV_USBD_EPOUT1

static char m_cdc_data_array[BLE_NUS_MAX_DATA_LEN];
 
/** @brief CDC_ACM class instance. */
APP_USBD_CDC_ACM_GLOBAL_DEF(m_app_cdc_acm,
                            cdc_acm_user_ev_handler,
                            CDC_ACM_COMM_INTERFACE,
                            CDC_ACM_DATA_INTERFACE,
                            CDC_ACM_COMM_EPIN,
                            CDC_ACM_DATA_EPIN,
                            CDC_ACM_DATA_EPOUT,
                            APP_USBD_CDC_COMM_PROTOCOL_AT_V250);

// USB DEFINES END

void advertising_start(void);
/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in]   line_num   Line number of the failing ASSERT call.
 * @param[in]   file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}


/**@brief Function for handling Peer Manager events.
 *
 * @param[in] p_evt  Peer Manager event.
 */
static void pm_evt_handler(pm_evt_t const * p_evt)
{
    bool delete_bonds = false;
    pm_handler_on_pm_evt(p_evt);
    pm_handler_flash_clean(p_evt);
    switch (p_evt->evt_id)
    {
    case PM_EVT_PEERS_DELETE_SUCCEEDED:
        advertising_start();
        break;
    default:
        break;
    }
}





/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
static void gap_params_init(void)
{
    ret_code_t              err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);
    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);
    err_code = sd_ble_gap_appearance_set(BLE_APPEARANCE_HEART_RATE_SENSOR_HEART_RATE_BELT);
    APP_ERROR_CHECK(err_code);
    memset(&gap_conn_params, 0, sizeof(gap_conn_params));
    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;
    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the GATT module. */
static void gatt_init(void)
{
    ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, NULL);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling Queued Write Module errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void nrf_qwr_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing services that will be used by the application.
 *
 * @details Initialize the Heart Rate, Battery and Device Information services.
 */
static void services_init(void)
{
    ret_code_t         err_code;
    ble_bas_init_t     bas_init;
    ble_dis_init_t     dis_init;
    nrf_ble_qwr_init_t qwr_init = {0};
    // Initialize Queued Write Module.
    qwr_init.error_handler = nrf_qwr_error_handler;
    err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
    APP_ERROR_CHECK(err_code);
    // Initialize Battery Service.
    memset(&bas_init, 0, sizeof(bas_init));
    // Here the sec level for the Battery Service can be changed/increased.
    bas_init.bl_rd_sec        = SEC_OPEN;
    bas_init.bl_cccd_wr_sec   = SEC_OPEN;
    bas_init.bl_report_rd_sec = SEC_OPEN;
    bas_init.evt_handler          = NULL;
    bas_init.support_notification = true;
    bas_init.p_report_ref         = NULL;
    bas_init.initial_batt_level   = 100;
    err_code = ble_bas_init(&m_bas, &bas_init);
    APP_ERROR_CHECK(err_code);
    // Initialize Device Information Service.
    memset(&dis_init, 0, sizeof(dis_init));
    ble_srv_ascii_to_utf8(&dis_init.manufact_name_str, (char *)MANUFACTURER_NAME);
    dis_init.dis_char_rd_sec = SEC_OPEN;
    ble_srv_ascii_to_utf8(&dis_init.fw_rev_str, FW_VERSION);
    ble_srv_ascii_to_utf8(&dis_init.hw_rev_str, HW_VERSION);
    ble_srv_ascii_to_utf8(&dis_init.sw_rev_str, SW_VERSION);
    err_code = ble_dis_init(&dis_init);
    APP_ERROR_CHECK(err_code);
    ble_nus_sevices_init();
}




/**@brief Function for handling the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module which
 *          are passed to the application.
 *          @note All this function does is to disconnect. This could have been done by simply
 *                setting the disconnect_on_fail config parameter, but instead we use the event
 *                handler mechanism to demonstrate its use.
 *
 * @param[in]   p_evt   Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    ret_code_t err_code;
    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling a Connection Parameters error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module. */
static void conn_params_init(void)
{
    ret_code_t             err_code;
    ble_conn_params_init_t cp_init;
    memset(&cp_init, 0, sizeof(cp_init));
    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;
    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void sleep_mode_enter(void)
{
    ret_code_t err_code;
    err_code = bsp_indication_set(BSP_INDICATE_IDLE);
    APP_ERROR_CHECK(err_code);
    // Prepare wakeup buttons.
    err_code = bsp_btn_ble_sleep_mode_prepare();
    APP_ERROR_CHECK(err_code);
    // Go to system-off mode (this function will not return; wakeup will cause a reset).
    err_code = sd_power_system_off();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    uint32_t err_code;
    switch (ble_adv_evt)
    {
    case BLE_ADV_EVT_FAST:
        NRF_LOG_INFO("Fast advertising.");
//        err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
//        APP_ERROR_CHECK(err_code);
        sys_status.bluetooth_status=1;
        break;
    case BLE_ADV_EVT_IDLE:
        sys_status.bluetooth_status=0;
        //sleep_mode_enter();
        break;
    default:
        break;
    }
}


/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    uint32_t err_code;
    switch (p_ble_evt->header.evt_id)
    {
    case BLE_GAP_EVT_CONNECTED:
        NRF_LOG_INFO("Connected");
        APP_ERROR_CHECK(err_code);
        m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
        m_nus_conn_handle=m_conn_handle;
        err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
        APP_ERROR_CHECK(err_code);
        if(update_periph_db==1)
        {
            err_code = ble_db_discovery_start(&m_db_disc, p_ble_evt->evt.gap_evt.conn_handle);
            APP_ERROR_CHECK(err_code);
        }
        sys_status.bluetooth_status=2;
        break;
    case BLE_GAP_EVT_DISCONNECTED:
        NRF_LOG_INFO("Disconnected");
        m_conn_handle = BLE_CONN_HANDLE_INVALID;
        m_nus_conn_handle=m_conn_handle;
        sys_status.bluetooth_status=1;
        break;
    case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
    {
        NRF_LOG_DEBUG("PHY update request.");
        ble_gap_phys_t const phys =
        {
            .rx_phys = BLE_GAP_PHY_AUTO,
            .tx_phys = BLE_GAP_PHY_AUTO,
        };
        err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
        APP_ERROR_CHECK(err_code);
    }
    break;
    case BLE_GATTC_EVT_TIMEOUT:
        // Disconnect on GATT Client timeout event.
        NRF_LOG_DEBUG("GATT Client Timeout.");
        err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                         BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
        APP_ERROR_CHECK(err_code);
        break;
    case BLE_GATTS_EVT_TIMEOUT:
        // Disconnect on GATT Server timeout event.
        NRF_LOG_DEBUG("GATT Server Timeout.");
        err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                         BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
        APP_ERROR_CHECK(err_code);
        break;
    default:
        // No implementation needed.
        break;
    }
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
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
    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}


/**@brief Function for handling events from the BSP module.
 *
 * @param[in]   event   Event generated by button press.
 */
static void bsp_event_handler(bsp_event_t event)
{
    ret_code_t err_code;
    switch (event)
    {
    case BSP_EVENT_KEY_0:
        //GUI_StoreKeyMsg(WM_USER_KEY_UP, 1);   //�Ϸ���
        break;
    case BSP_EVENT_KEY_1:
        //GUI_StoreKeyMsg(WM_USER_KEY_ENTER, 1);    //ȷ�ϼ�
        break;
    case BSP_EVENT_KEY_2:
        //GUI_StoreKeyMsg(WM_USER_KEY_DOWN, 1);    //�·���
        break;
    case BSP_EVENT_KEY_0_LONG_PUSH:  //������
        //GUI_StoreKeyMsg(WM_USER_KEY_UP_LONG, 1);
        break;
    case BSP_EVENT_KEY_1_LONG_PUSH:  //���ؼ�����
        //GUI_StoreKeyMsg(WM_USER_KEY_BACK, 1);   //���ؼ�
        break;
    case BSP_EVENT_KEY_2_LONG_PUSH:  //ȷ�ϼ�����
        //GUI_StoreKeyMsg(WM_USER_KEY_DOWN_LONG, 1);
        break;
    default:
        break;
    }
}


/**@brief Function for the Peer Manager initialization. */
static void peer_manager_init(void)
{
    ble_gap_sec_params_t sec_param;
    ret_code_t           err_code;
    err_code = pm_init();
    APP_ERROR_CHECK(err_code);
    memset(&sec_param, 0, sizeof(ble_gap_sec_params_t));
    // Security parameters to be used for all security procedures.
    sec_param.bond           = SEC_PARAM_BOND;
    sec_param.mitm           = SEC_PARAM_MITM;
    sec_param.lesc           = SEC_PARAM_LESC;
    sec_param.keypress       = SEC_PARAM_KEYPRESS;
    sec_param.io_caps        = SEC_PARAM_IO_CAPABILITIES;
    sec_param.oob            = SEC_PARAM_OOB;
    sec_param.min_key_size   = SEC_PARAM_MIN_KEY_SIZE;
    sec_param.max_key_size   = SEC_PARAM_MAX_KEY_SIZE;
    sec_param.kdist_own.enc  = 1;
    sec_param.kdist_own.id   = 1;
    sec_param.kdist_peer.enc = 1;
    sec_param.kdist_peer.id  = 1;
    err_code = pm_sec_params_set(&sec_param);
    APP_ERROR_CHECK(err_code);
    err_code = pm_register(pm_evt_handler);
    APP_ERROR_CHECK(err_code);
}



/**@brief Function for initializing the Advertising functionality. */
static void advertising_init(void)
{
    ret_code_t             err_code;
    ble_advertising_init_t init;
    memset(&init, 0, sizeof(init));
    init.advdata.name_type               = BLE_ADVDATA_FULL_NAME;
    init.advdata.include_appearance      = true;
    init.advdata.flags                   = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
    init.advdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    init.advdata.uuids_complete.p_uuids  = m_adv_uuids;
    init.config.ble_adv_fast_enabled  = true;
    init.config.ble_adv_fast_interval = APP_ADV_INTERVAL;
    init.config.ble_adv_fast_timeout  = APP_ADV_DURATION;
    init.config.ble_adv_slow_enabled  = true;
    init.config.ble_adv_slow_interval = APP_ADV_INTERVAL;
    init.config.ble_adv_slow_timeout  = 0;
//    init.config.ble_adv_primary_phy  = BLE_GAP_PHY_CODED;
//    init.config.ble_adv_secondary_phy  = BLE_GAP_PHY_CODED;
//   init.config.ble_adv_extended_enabled = true;
    init.evt_handler = on_adv_evt;
    err_code = ble_advertising_init(&m_advertising, &init);
    APP_ERROR_CHECK(err_code);
    ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
}

/**@brief Function for starting advertising. */
void advertising_start(void)
{
//    bool erase_bonds = *(bool*)p_erase_bonds;
//    if (erase_bonds)
//    {
//        delete_bonds();
//        // Advertising is started by PM_EVT_PEERS_DELETE_SUCCEEDED event.
//    }
//    else
//    {
    ret_code_t err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
    APP_ERROR_CHECK(err_code);
//    }
}

/**@brief Function for initializing the nrf log module.
 */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);
    NRF_LOG_DEFAULT_BACKENDS_INIT();
}


/**@brief Function for initializing buttons and leds.
 *
 * @param[out] p_erase_bonds  Will be true if the clear bonding button was pressed to wake the application up.
 */
static void buttons_init(void)
{
    ret_code_t err_code;
    //  bsp_event_t startup_event;
    err_code = bsp_init( BSP_INIT_BUTTONS, bsp_event_handler);
    APP_ERROR_CHECK(err_code);
//    err_code = bsp_btn_ble_init(NULL, &startup_event);
//    APP_ERROR_CHECK(err_code);
    //  *p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);
}

/**@brief Clear bond information from persistent storage. */
static void delete_bonds(void)
{
    ret_code_t err_code;
    NRF_LOG_INFO("Erase bonds!");
    err_code = pm_peers_delete();
    APP_ERROR_CHECK(err_code);
}




#if NRF_LOG_ENABLED
/**@brief Thread for handling the logger.
 *
 * @details This thread is responsible for processing log entries if logs are deferred.
 *          Thread flushes all log entries and suspends. It is resumed by idle task hook.
 *
 * @param[in]   arg   Pointer used for passing some arbitrary information (context) from the
 *                    osThreadCreate() call to the thread.
 */
static void logger_thread(void * arg)
{
    UNUSED_PARAMETER(arg);
    while (1)
    {
        NRF_LOG_FLUSH();
        vTaskSuspend(NULL); // Suspend myself
    }
}
#endif //NRF_LOG_ENABLED

/**@brief A function which is hooked to idle task.
 * @note Idle hook must be enabled in FreeRTOS configuration (configUSE_IDLE_HOOK).
 */
void vApplicationIdleHook( void )
{
#if NRF_LOG_ENABLED
    vTaskResume(m_logger_thread);
#endif
}


//scan periph name  list
//scan_devices_t d_list[20];
//char devices_numbers=0;
static int device_num_ble =0;
static void scan_evt_handler(scan_evt_t const * p_scan_evt)
{

		int i =1;
		
#if 1
    ret_code_t err_code;
    ble_gap_evt_adv_report_t const * p_adv = p_scan_evt->params.filter_match.p_adv_report;
    ble_gap_scan_params_t    const * p_scan_param =  p_scan_evt->p_scan_params;
    switch(p_scan_evt->scan_evt_id)
    {
    case NRF_BLE_SCAN_EVT_FILTER_MATCH:
    {
        // Initiate connection.
        err_code = sd_ble_gap_connect(&p_adv->peer_addr,
                                      p_scan_param,
                                      &m_scan.conn_params,
                                      APP_BLE_CONN_CFG_TAG);
        APP_ERROR_CHECK(err_code);
    }
    break;
    case NRF_BLE_SCAN_EVT_NOT_FOUND:
    {
        //scan periph name  list
        uint16_t        parsed_name_len=0;
        uint8_t const * p_parsed_name;
        uint16_t data_offset=0;
        parsed_name_len = ble_advdata_search(p_adv->data.p_data, p_adv->data.len, &data_offset,BLE_GAP_AD_TYPE_COMPLETE_LOCAL_NAME);
        if( devices_numbers<20)
        {
            if(parsed_name_len==0)
            {
                int is_new=1;
                if(devices_numbers>0)
                {
                    for(int i=0; i<devices_numbers; i++)
                    {
                        if( memcmp(&d_list[i].mac_addr[0],&p_adv->peer_addr.addr,6)==0)
                        {
                            d_list[i].rssi=p_adv->rssi;
                            d_list[i].ch_index=p_adv->ch_index;
                            is_new=0;
                            break;
                        }
                    }
                }
                if(is_new==1)
                {
                    //memcpy(	&p_s_param[periph_numbers],p_scan_param,sizeof(ble_gap_scan_params_t))
                    memcpy(	&d_list[devices_numbers].mac_addr[0],&p_adv->peer_addr.addr,6);
                    d_list[devices_numbers].primary_phy=p_adv->primary_phy;
                    d_list[devices_numbers].tx_power=p_adv->tx_power;
                    d_list[devices_numbers].rssi=p_adv->rssi;
                    d_list[devices_numbers].ch_index=p_adv->ch_index;
                    memcpy(&d_list[devices_numbers].adv_data, &p_adv->data.p_data[0],p_adv->data.len);
                    d_list[devices_numbers].adv_len=p_adv->data.len;
                    devices_numbers++;
                }
            }
        }
        else
        {
            if(parsed_name_len==0)
            {
                if(devices_numbers>0)
                {
                    for(int i=0; i<devices_numbers; i++)
                    {
                        if(memcmp(&d_list[i].mac_addr[0],&p_adv->peer_addr.addr,6)==0)
                        {
                            d_list[i].rssi=p_adv->rssi;
                            d_list[i].ch_index=p_adv->ch_index;
                            break;
                        }
                    }
                }
            }
        }
    }
    break;
    case	NRF_BLE_SCAN_EVT_SCAN_TIMEOUT:
    {
        scan_complete=1;
    }
    break;
    default:
        break;
    }
#endif
}


/**@brief Function for initialization the scanning and setting the filters.
 */
static void scan_init(void)
{
    ret_code_t          err_code;
    nrf_ble_scan_init_t init_scan;
    memset(&init_scan, 0, sizeof(init_scan));
    init_scan.p_scan_param = &m_scan_param;
    err_code = nrf_ble_scan_init(&m_scan, &init_scan, scan_evt_handler);
    APP_ERROR_CHECK(err_code);
    APP_ERROR_CHECK(err_code);
}
/**@brief Function for initializing the scanning.
 */
void scan_start(void)
{
		scan_complete=0;
		devices_numbers=0;
    ret_code_t err_code;
		m_scan_param.timeout=200;
		nrf_ble_scan_params_set(&m_scan,&m_scan_param);
    err_code = nrf_ble_scan_start(&m_scan);
    APP_ERROR_CHECK(err_code);
}

void fast_scan_start(void)
{
		scan_complete=0;
    ret_code_t err_code;
		m_scan_param.timeout=100;
		nrf_ble_scan_params_set(&m_scan,&m_scan_param);
    err_code = nrf_ble_scan_start(&m_scan);
    APP_ERROR_CHECK(err_code);
}

void scan_stop(void)
{
    nrf_ble_scan_stop();
}

/**< BLE NUS service client instance. */
static void ble_nus_c_evt_handler(ble_nus_c_t * p_ble_nus_c, ble_nus_c_evt_t const * p_ble_nus_evt)
{
    ret_code_t err_code;
    switch (p_ble_nus_evt->evt_type)
    {
    case BLE_NUS_C_EVT_DISCOVERY_COMPLETE:
        NRF_LOG_INFO("Discovery complete.");
        err_code = ble_nus_c_handles_assign(p_ble_nus_c, p_ble_nus_evt->conn_handle, &p_ble_nus_evt->handles);
        APP_ERROR_CHECK(err_code);
        err_code = ble_nus_c_tx_notif_enable(p_ble_nus_c);
        APP_ERROR_CHECK(err_code);
        err_code = ble_nus_c_ecg_tx_notif_enable(p_ble_nus_c);
        APP_ERROR_CHECK(err_code);
        err_code = ble_nus_c_mix_tx_notif_enable(p_ble_nus_c);
        APP_ERROR_CHECK(err_code);
        NRF_LOG_INFO("Connected to device with Nordic UART Service.");
        //   sys_ble_status=2;
        break;
    case BLE_NUS_C_EVT_NUS_TX_EVT:
//            ble_nus_chars_received_uart_print(p_ble_nus_evt->p_data, p_ble_nus_evt->data_len);
//				if(mix_chanl_ontopen)
//				{
//				 err_code = ble_nus_c_mix_notif_enable(p_ble_nus_c);
//            APP_ERROR_CHECK(err_code);
//					mix_chanl_ontopen=0;
//				}
        break;
    case BLE_NUS_C_EVT_NUS_ECG_TX_EVT:
//        for(int i=0; i<10; i++)
//        {
//            ecg_buff[ecg_buff_index+i]=p_ble_nus_evt->p_data[i*2]<<8|p_ble_nus_euvt->p_data[i*2+1];;
//        }
//        ecg_buff_index+=10;
//        if(ecg_buff_index>=154)ecg_buff_index=0;
        // amsu_windosw_update();
        break;
    case BLE_NUS_C_EVT_NUS_MIX_TX_EVT:  //drones statue report
//        heart_rate=p_ble_nus_evt->p_data[0];
//        body_steps=p_ble_nus_evt->p_data[1]+p_ble_nus_evt->p_data[2]*256+p_ble_nus_evt->p_data[3]*65535;
        // memcpy(drones_status,p_ble_nus_evt->p_data,8);
        break;
    case BLE_NUS_C_EVT_DISCONNECTED:
        NRF_LOG_INFO("Disconnected.");
        //     scan_start();
        break;
    }
}
/**@brief Function for initializing the NUS Client. */
static void nus_c_init(void)
{
    ret_code_t       err_code;
    ble_nus_c_init_t   init;
    init.evt_handler = ble_nus_c_evt_handler;
    err_code = ble_nus_c_init(&m_ble_nus_c, &init);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling Database Discovery events.
 *
 * @details This function is a callback function to handle events from the database discovery module.
 *          Depending on the UUIDs that are discovered, this function should forward the events
 *          to their respective service instances.
 *
 * @param[in] p_event  Pointer to the database discovery event.
 */
static void db_disc_handler(ble_db_discovery_evt_t * p_evt)
{
    // ble_ancs_c_on_db_disc_evt(&m_ancs_c, p_evt);
    ble_nus_c_on_db_disc_evt(&m_ble_nus_c, p_evt);
}
/**@brief Function for initializing the database discovery module.
 */
static void db_discovery_init(void)
{
    ble_db_discovery_init_t db_disc_init;
    db_disc_init.evt_handler=db_disc_handler;
    db_disc_init.p_gatt_queue = &m_ble_gatt_queue;
    ret_code_t ret = ble_db_discovery_init(&db_disc_init);
    APP_ERROR_CHECK(ret);
}

void connect_scan_Peripheral(unsigned char Peripheral_index)
{
    ret_code_t          err_code;
    update_periph_db=1;
//    err_code = sd_ble_gap_connect(&p_addr[Peripheral_index],
//                                  &p_s_param[Peripheral_index],
//                                  &m_scan.conn_params,
//                                  APP_BLE_CONN_CFG_TAG);
    APP_ERROR_CHECK(err_code);
}
void send_cmd_to_Peripheral(unsigned char *cmd_string,unsigned short string_len)
{
    ret_code_t          err_code;
    err_code = ble_nus_c_string_send(&m_ble_nus_c,cmd_string,string_len);
    APP_ERROR_CHECK(err_code);
}

// USB CODE START
 bool m_usb_connected = false;
void cdc_acm_user_data_send(unsigned char * data, int len)
{
    app_usbd_cdc_acm_write(&m_app_cdc_acm,data,len);

}

/** @brief User event handler @ref app_usbd_cdc_acm_user_ev_handler_t */
static void cdc_acm_user_ev_handler(app_usbd_class_inst_t const * p_inst,
                                    app_usbd_cdc_acm_user_event_t event)
{
    app_usbd_cdc_acm_t const * p_cdc_acm = app_usbd_cdc_acm_class_get(p_inst);

    switch (event)
    {
        case APP_USBD_CDC_ACM_USER_EVT_PORT_OPEN:
        {
            /*Set up the first transfer*/
            ret_code_t ret = app_usbd_cdc_acm_read(&m_app_cdc_acm,
                                                   m_cdc_data_array,
                                                   1);
            UNUSED_VARIABLE(ret);
          //  ret = app_timer_stop(m_blink_cdc);
          //  APP_ERROR_CHECK(ret);
         //   bsp_board_led_on(LED_CDC_ACM_CONN);
            NRF_LOG_INFO("CDC ACM port opened");
            break;
        }

        case APP_USBD_CDC_ACM_USER_EVT_PORT_CLOSE:
            NRF_LOG_INFO("CDC ACM port closed");
            if (m_usb_connected)
            {
//                ret_code_t ret = app_timer_start(m_blink_cdc,
//                                                 APP_TIMER_TICKS(LED_BLINK_INTERVAL),
//                                                 (void *) LED_CDC_ACM_CONN);
//                APP_ERROR_CHECK(ret);
            }
            break;

        case APP_USBD_CDC_ACM_USER_EVT_TX_DONE:
            break;

        case APP_USBD_CDC_ACM_USER_EVT_RX_DONE:
        {
            ret_code_t ret;
            static uint8_t index = 0;
            index++;

            do
            {
                if ((m_cdc_data_array[index - 1] == '\n') ||
                    (m_cdc_data_array[index - 1] == '\r') ||
                    (index >= (80)))
                {
                    if (index > 1)
                    {
//                        bsp_board_led_invert(LED_CDC_ACM_RX);
//                        NRF_LOG_DEBUG("Ready to send data over BLE NUS");
//                        NRF_LOG_HEXDUMP_DEBUG(m_cdc_data_array, index);

//                        do
//                        {
//                            uint16_t length = (uint16_t)index;
//                            if (length + sizeof(ENDLINE_STRING) < BLE_NUS_MAX_DATA_LEN)
//                            {
//                                memcpy(m_cdc_data_array + length, ENDLINE_STRING, sizeof(ENDLINE_STRING));
//                                length += sizeof(ENDLINE_STRING);
//                            }

//                            ret = ble_nus_data_send(&m_nus,
//                                                    (uint8_t *) m_cdc_data_array,
//                                                    &length,
//                                                    m_conn_handle);

//                            if (ret == NRF_ERROR_NOT_FOUND)
//                            {
//                                NRF_LOG_INFO("BLE NUS unavailable, data received: %s", m_cdc_data_array);
//                                break;
//                            }

//                            if (ret == NRF_ERROR_RESOURCES)
//                            {
//                                NRF_LOG_ERROR("BLE NUS Too many notifications queued.");
//                                break;
//                            }

//                            if ((ret != NRF_ERROR_INVALID_STATE) && (ret != NRF_ERROR_BUSY))
//                            {
//                                APP_ERROR_CHECK(ret);
//                            }
//                        }
//                        while (ret == NRF_ERROR_BUSY);
                    }

                    index = 0;
                }

                /*Get amount of data transferred*/
                size_t size = app_usbd_cdc_acm_rx_size(p_cdc_acm);
                NRF_LOG_DEBUG("RX: size: %lu char: %c", size, m_cdc_data_array[index - 1]);

                /* Fetch data until internal buffer is empty */
                ret = app_usbd_cdc_acm_read(&m_app_cdc_acm,
                                            &m_cdc_data_array[index],
                                            1);
                if (ret == NRF_SUCCESS)
                {
                    index++;
                }
            }
            while (ret == NRF_SUCCESS);

            break;
        }
        default:
            break;
    }
}

static void usbd_user_ev_handler(app_usbd_event_type_t event)
{

}

void usb_new_event_isr_handler(app_usbd_internal_evt_t const * const p_event, bool queued)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    UNUSED_PARAMETER(p_event);
    UNUSED_PARAMETER(queued);
    ASSERT(m_usbd_thread != NULL);
    /* Release the semaphore */
    vTaskNotifyGiveFromISR(m_usbd_thread, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

static void usbd_thread(void * arg)
{
    ret_code_t ret;
    static const app_usbd_config_t usbd_config = {
        .ev_isr_handler = usb_new_event_isr_handler,
        .ev_state_proc  = usbd_user_ev_handler
    };
    UNUSED_PARAMETER(arg);

    ret = app_usbd_init(&usbd_config);
    APP_ERROR_CHECK(ret);
    app_usbd_class_inst_t const * class_cdc_acm = app_usbd_cdc_acm_class_inst_get(&m_app_cdc_acm);
    ret = app_usbd_class_append(class_cdc_acm);
    APP_ERROR_CHECK(ret);
    ret = app_usbd_power_events_enable();
    APP_ERROR_CHECK(ret);

    // Set the first event to make sure that USB queue is processed after it is started
    UNUSED_RETURN_VALUE(xTaskNotifyGive(xTaskGetCurrentTaskHandle()));
    // Enter main loop.
    for (;;)
    {
        /* Waiting for event */
        UNUSED_RETURN_VALUE(ulTaskNotifyTake(pdTRUE, USB_THREAD_MAX_BLOCK_TIME));
        while (app_usbd_event_queue_process())
        {
            /* Nothing to do */
        }
    }
}

// USB CODE END


/**@brief Function for initializing the clock.
 */
static void clock_init(void)
{
    ret_code_t err_code = nrf_drv_clock_init();
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
static void timers_init(void)
{
    // Initialize timer module.
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
}
/**@brief Function for application main entry.
 */
 
#include "nrf_saadc.h"
#include "nrfx_saadc.h"
extern TaskHandle_t GUI_thread;
uint64_t device_id_only;  
int main(void)
{
#if 1
		BAT_POW_Set(1);
		log_init();
		clock_init();
		// Activate deep sleep mode.
    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
		Misc_Init();
		
		//开机
		nrf_delay_ms(1000);
		
		//如果正在充电-或者充满状态，直接开机
		int MCU_CHRG_rel =nrf_gpio_pin_read(MCU_CHRG_);
		int MCU_STDBY_rel =nrf_gpio_pin_read(MCU_STDBY_);
		if( (MCU_CHRG_rel == 0  &&  MCU_STDBY_rel == 1) || (MCU_CHRG_rel == 1  &&  MCU_STDBY_rel == 0) ){  //正在充电 充电完成  直接开机
			//打开电源
			BAT_POW_Set(1);
			goto Run_goon;
		}
		
		while(1){
			if(nrf_gpio_pin_read(BUTTON_POW) ==0)
			{
				BAT_POW_Set(0);
			}else{
				BAT_POW_Set(1);
				break;
			}
			nrf_delay_ms(1000);
		}
	Run_goon:


		
    // Configure and initialize the BLE stack.
    ble_stack_init();
//		Misc_Init();
		nrf_gpio_pin_write(LCD_PWR_EN, 0);
		//////
    gap_params_init();
    gatt_init();
    advertising_init();
    services_init();
    conn_params_init();
    peer_manager_init();
    db_discovery_init();
    nus_c_init();
    scan_init();
		//获取设备唯一ID
		device_id_only = ((uint64_t)NRF_FICR->DEVICEID[1] << 32) | NRF_FICR->DEVICEID[0];
		bool erase_bonds=0;
		
//		//开机
//		nrf_delay_ms(1000);
//		while(1){
//			if(nrf_gpio_pin_read(BUTTON_POW) ==0)
//			{
//				BAT_POW_Set(0);
//			}else{
//				BAT_POW_Set(1);
//				break;
//			}
//			nrf_delay_ms(500);
//		}



//		if(nrf_gpio_pin_read(BUTTON_POW) ==0)
//		{
//			BAT_POW_Set(0);
//			while(1);
//		}
		MOTOR_start_tim(300);
		
		
		void qmc5883p_fun(void);
		qmc5883p_fun();
		
		void gui_fun(void * ass);
		if(xTaskCreate(gui_fun, "gui", 1024*20, NULL, 5, &GUI_thread) == 1)
    {
        ;//sys_status.app_task_bluetooth = 1;
    }
    // The task will run advertising_start() before entering its loop.
    nrf_sdh_freertos_init(NULL, &erase_bonds);//advertising_start
    NRF_LOG_INFO("HRS FreeRTOS example started.");
    // Start FreeRTOS scheduler.
    vTaskStartScheduler();
    for (;;)
    {
        APP_ERROR_HANDLER(NRF_ERROR_FORBIDDEN);
    }
#endif
}

extern lv_obj_t * img_main;
static lv_obj_t * img_logo =NULL;
unsigned int auto_pow_off_num =0;
extern TaskHandle_t Tim_thread;
uint8_t bettry_flag =0;

int16_t qmc_mag_x_y;
int16_t qmc_mag_y_y;
int16_t qmc_mag_z_y;

int16_t qmc_mag_x;
int16_t qmc_mag_y;
int16_t qmc_mag_z;
void Tim_fun(void * ass);
void gui_fun(void * ass)
{
	UNUSED_PARAMETER(ass);
	TickType_t xLastWakeTime;
	const TickType_t xFrequency = pdMS_TO_TICKS(50); // 50ms 周期
	
	lv_init(); 
	lv_port_disp_init(); 
	
    lv_obj_set_style_bg_color(lv_scr_act(), lv_color_white(), 0);
    lv_obj_set_style_bg_opa(lv_scr_act(), LV_OPA_COVER, 0);
		lv_timer_handler();
		lv_refr_now(NULL);
 
	
		LCD_PWM_Set(1);
		img_logo = lv_img_create(lv_scr_act());
		//lv_obj_set_size(img_logo, lv_disp_get_hor_res(NULL), lv_disp_get_ver_res(NULL));
		lv_img_set_src(img_logo, &logo);
		lv_obj_align(img_logo, LV_ALIGN_TOP_MID, 0, 150);
		lv_timer_handler();
		lv_refr_now(NULL);
		nrf_delay_ms(800);
		if(img_logo && lv_obj_is_valid(img_logo)){
			lv_obj_del(img_logo); 
			img_logo = NULL;
		}
	
		Lvgl_Init();
    lv_obj_set_style_bg_color(lv_scr_act(), lv_color_black(), 0);
    lv_obj_set_style_bg_opa(lv_scr_act(), LV_OPA_COVER, 0);
		lv_timer_handler();
		lv_refr_now(NULL);
		
		LCD_PWM_Set(1);
		//添加logo
//		img_logo = lv_img_create(img_main);
//		lv_img_set_src(img_logo, &logo);
//		lv_obj_align(img_logo, LV_ALIGN_TOP_MID, 0, 100);
//		lv_timer_handler();
//		lv_refr_now(NULL);
		
		//延时显示
//		nrf_delay_ms(5000);
//		if(img_logo && lv_obj_is_valid(img_logo)){
//			lv_obj_del(img_logo); 
//			img_logo = NULL;
//		}
		
		
		
//		lv_timer_handler();
//		lv_refr_now(NULL);
//		LCD_PWM_Set(1);
		//创建检测线程
		if(Tim_thread ==NULL){
			if(xTaskCreate(Tim_fun, "gui", 1024*2, NULL, 3, &Tim_thread) == 1)
			{
					;
			}
		}
		while(1){
				//sd_app_evt_wait();
				if(Police_Flag_Again==1){
					Police_Flag_Again =0;
				}
				//UI更新
				if(layer_ui_deal(key_update())==0){
					Gui_Pro();
				}
				//电池状态检测
				static int Batt_stat_time =0;
				if(Batt_stat_time++>20){
					Batt_stat_time =0;
					int rel =batt_sta_update();
					if(rel ==1 && bettry_flag ==0){
						Batt_Set_UI_2(1);	//正在充电
						auto_pow_off_num =0; //在插入电源不进入自动关机
					}else if((rel ==2 && bettry_flag ==1) || (rel ==1 && bettry_flag ==1)){
						Batt_Set_UI_2(2);	//充电完成
						auto_pow_off_num =0; //在插入电源不进入自动关机
					}else if(rel ==3){
						Batt_Set_UI_2(0);	//未接输入电源
					}
				}
				//电池电量检测
				static int Batt_check_time =0;
				if(Batt_check_time++>10){//if(Batt_check_time++>20){
					Batt_check_time =0;
					Batt_Set_UI_1();
				}
				//图片刷新
				static int lv_refr_now_num =0;
				lv_timer_handler();
				if(lv_refr_now_num++>1){
					lv_refr_now_num =0;
					lv_refr_now(NULL);
				}
				//自动关机
				if(AD8314_sta ==0){
					if(auto_pow_off_num++ > 180000){//if(auto_pow_off_num++ > 120000){  //5ms *  =
					  auto_pow_off_num =0;
						#if 1
						Device_POW_flag =0;
						MOTOR_start_tim(300);
						sound_flag =0;
						move_flag =0;
						LCD_PWM_Set(0);
						BAT_POW_Set(0);
						#endif
					}
				}else{
					auto_pow_off_num =0;
				}
				//报警设置
				static unsigned long long polic_num =0;
				static bool beep_sta =0;
				static int beep_freq =0;
				if(polic_ing){
					//此时正在报警-开始计数
					if(polic_num++ > (1000*2) * 3 ){   //10s * 3
						polic_num =0;
						//关闭报警-同事继续打开触发
						AD8314_deep_disable();
						AD8314_Int_enable();
						if(beep_sta){
							beep_sta =0;
							beeper_stop();
						}
					}else{
						if(beep_freq++<30){
							if(beep_sta==0){
								beep_sta =1;
								beeper_start();
							}
						}else if(beep_freq>30 && beep_freq<60){
							beep_freq =0;
							if(beep_sta){
								beep_sta =0;
								beeper_stop();
							}
						}
					}
				}else{
					polic_num =0;
					if(beep_sta){
						beep_sta =0;
						beeper_stop();
					}
				}
        //精确延迟到下一个周期
        vTaskDelay(2);//vTaskDelayUntil(&xLastWakeTime, xFrequency);
		}
}


extern short frfull_data_buffer[100];
extern short fr24g_data_buffer[100];
extern short hall_data_buffer[100];
extern short bat_data_buffer[100];
extern void device_adc_deal(void);
extern void ADC_Once_IR(void);
extern void ADC_Once_Hall(void);
extern void ADC_Once_Batty(void);
extern void ADC_Once_Wless(void);
extern int tag_data;
int tag_numbers =0;
int tag_rssi;
int scan_mode =1;
void Tim_fun(void * ass)
{
		UNUSED_PARAMETER(ass);
		Net_Init();
		Net_Init_Pow_Set(0);
		bsp_saadc_init();
		scan_start();
		while(1){
		
			//强磁模式
			if(curr_pos_set ==6){
					uint8_t data =0;
					ret_code_t qmc5883p_read_byte(uint8_t reg, uint8_t *data);
					qmc5883p_read_byte(9, &data);
					if(data & 0x01){
						//读取磁力数据
						ret_code_t qmc5883p_read_mag_data(int16_t *mag_x, int16_t *mag_y, int16_t *mag_z);
						qmc_mag_x_y =0;
						qmc_mag_y_y =0;
						qmc_mag_z_y =0;
						qmc5883p_read_mag_data(&qmc_mag_x_y, &qmc_mag_y_y, &qmc_mag_z_y);
					}
					
					if(qmc_mag_x_y < 0){
						qmc_mag_x = -qmc_mag_x_y;
					}else{
						qmc_mag_x = qmc_mag_x_y;
					}
					
					if(qmc_mag_y_y < 0){
						qmc_mag_y = -qmc_mag_y_y;
					}else{
						qmc_mag_y = qmc_mag_y_y;
					}
					
					if(qmc_mag_z_y < 0){
						qmc_mag_z = -qmc_mag_z_y;
					}else{
						qmc_mag_z = qmc_mag_z_y;
					}
					
					
					if(qmc_mag_x > qmc_mag_y){
						ADC_Hall = qmc_mag_x;
					}else{
						ADC_Hall = qmc_mag_y;
					}
					if(ADC_Hall<qmc_mag_z){
						ADC_Hall = qmc_mag_z;
					}
			}
		
			if(curr_pos_set ==2 || curr_pos_set ==3 || curr_pos_set ==4 ||curr_pos_set ==5 || curr_pos_set ==0){
				if(saadc_event_handler_flag){
						saadc_event_handler_flag =0;
						//处理数据
						ADC_IR =frfull_data_buffer[50];
						//ADC_Hall =hall_data_buffer[50];
						ADC_Batty =bat_data_buffer[50];
						//ADC_Wless =fr24g_data_buffer[50];
						short max_in100=0;
						for(int i=0;i<100;i++){
							 if(max_in100<fr24g_data_buffer[i]) //radio
								 max_in100=fr24g_data_buffer[i];
						}
						ADC_Wless =max_in100;
				}
			}
			
	#if 1
			static bool tag_flag =0;
			if(curr_pos_set == 5){
					tag_flag =1;
					if(scan_mode ==1){
						if(scan_complete==1)
						{
								tag_numbers=ble_tag_devices_detection();
								if(tag_numbers>0){
										scan_mode=2;
										fast_scan_start();//scann rssi
								}else{
										scan_start(); //scan devices go on
								}
						}
					}else if(scan_mode ==2){
						static int scan_complete_flag_num =0;
						if(scan_complete==1)
						{
							scan_complete_flag_num =0;
							tag_rssi=ble_tag_rssi_detection();
							static int tag_not_scan_times=0;
							if(tag_rssi!=0){
								tag_not_scan_times=0;
								int tag_data_t =0;
//									if(tag_rssi > -67){
//											tag_data_t=(tag_rssi+85)/4;//-40db -85db
//									}else{
//											tag_data_t=(tag_rssi+85)/6;//-40db -85db
//									}
									
									tag_data_t=(tag_rssi+85)/3;//-40db -85db
									
									if(tag_data_t>16)tag_data_t=16;
									if(tag_data_t<0)tag_data_t=0;//if(tag_data_t<0)tag_data_t=1;//tag_data_t=0;
									tag_data =tag_data_t;
							}else{
									if(tag_not_scan_times++>100)
									{
											tag_not_scan_times =0;
											tag_data=0;
									}
							}
							//继续触发快速扫描
							fast_scan_start();
						}else{
							if(scan_complete_flag_num++>500){
									scan_complete_flag_num =0;
									tag_data=0;
							}
						}
					}
				}else{
					if(tag_flag){
						tag_flag =0;
						tag_numbers =0;
						tag_data=0;
						memset(d_list, 0, sizeof(d_list));
						memset(tag_list, 0, sizeof(tag_list));
					}
					tag_flag =0;
					tag_data=0;
				}
				#endif
			vTaskDelay(5);
		}
}







