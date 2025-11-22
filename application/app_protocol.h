#ifndef __APP_PROTOCOL_H
#define __APP_PROTOCOL_H
#include "ble_nus.h"
#include "ble_bas.h"
#include "FreeRTOS.h"
#include "task.h"
extern TaskHandle_t m_bluetooth_thread;

typedef enum
{

    CMD_SYSTEM_NULL=0,
    CMD_SYSTEM_CHECK_STATUS,        //41 31 /A1
    CMD_SYSTEM_ECG_SWHICHE,           //41 32 /A2
    CMD_SYSTEM_ACC_SWHICHE,           //41 33 /A3
    CMD_SYSTEM_OPEN_DELAY,            //41 34 /A4
    CMD_SYSTEM_CLOSE_DELAY,           //41 35 /A5
    CMD_SYSTEM_ADVOUT_DELAY,          //41 36 /A6
    CMD_USER_ADV_BOOD_DATA,           //41 37 /A7

    CMD_SYSTEM_OTA_MODE,              //42 31 /B1
    CMD_SYSTEM_AIR_RESET,             //42 31 /B2
    CMD_SYSTEM_SLEEP_MODE,            //42 33 /B3
    CMD_SYSTEM_RESET_STEP,            //42 36 /B6
    CMD_SYSTEM_GET_STEPS,             //42 37 /B7
    CMD_SYSTEM_LED_CTR,               //42 38 /B8

    CMD_USER_SET_ID,                  //43 32 /C2
    CMD_USER_GET_ID,                  //43 33 /C3
    CMD_USER_SET_NAME,                //43 34 /C4
    CMD_USER_GET_NAME,                //43 35 /C5
    CMD_USER_SET_INF,                 //43 36 /C6
    CMD_USER_GET_INF,                 //43 37 /C7


    CMD_USER_SET_WIFI_NAME,           //44 32 /D2
    CMD_USER_GET_WIFI_NAME,           //44 33 /D3
    CMD_USER_SET_WIFI_CTRECT,         //44 34 /D4
    CMD_USER_GET_WIFI_CTRECT,         //44 35 /D5
    CMD_USER_SET_SYS_TIME,            //44 36 /D6
    CMD_USER_GET_SYS_TIME,            //44 37 /D7

    CMD_SYSTEM_AUTO_OFL_SWHICHE,      //44 38 /D8
    CMD_USER_SET_AFL_TIME,            //44 39 /D9

    CMD_USER_GET_UPLAOD_INF,          //44 3A /D7
    CMD_USER_GET_SPEACE_INF,          //44 3B /D8
    CMD_USER_DELEAT_USER_DATA,        //44 3C /D9
    CMD_USER_DELEAT_ALL_DATA,         //44 3D /DA

    CMD_FILE_CREATE,          //44 3A /D7
    CMD_FILE_DATA,          //44 3B /D8
    CMD_FILE_ERASE,        //44 3C /D9



} amsu_cmd_list;







extern unsigned char sys_readv_flag;
extern ble_nus_t m_nus;
extern unsigned short m_nus_conn_handle;
extern  ble_bas_t   m_bas;
void app_protocol_init(void);
int ble_nus_send_ecg_data(uint8_t* p_data,  uint16_t  * p_length );
int ble_nus_send_mix_data(uint8_t* p_data,  uint16_t  * p_length );
void ble_nus_sevices_init(void);
void app_ble_cmd_process(void);
void app_ble_send_process(void);
void app_system_cmd_thread(void * ass);
void app_system_send_thread(void * ass);
#endif



