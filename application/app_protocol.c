#include "bsp.h"
#include "app_protocol.h"
#include "ble_nus.h"
#include "nrf_sdh_soc.h"
#include "app_times.h"
#include "app_config.h"
#include "app_display.h"
#include <string.h>
#include "app_system.h"

#define BOOTLOADER_OTA_START                0xB1

BLE_NUS_DEF(m_nus,2);
ble_bas_t   m_bas;

unsigned short m_nus_conn_handle=BLE_CONN_HANDLE_INVALID;

static volatile unsigned char  ble_rx_cmd=0;
static unsigned char ble_rx_data[40];
static unsigned char ble_tx_data[20];
static unsigned short  send_len[1];
const unsigned char  info_OK[3]= {0x2B,0x4F,0x4B};
const unsigned char  info_WF[3]= {0x2B,0x57,0x46};
const unsigned char  info_ERR[4]= {0x2B,0x45,0x52,0x52};


static void nus_data_handler(ble_nus_evt_t * p_evt)
{
    if (p_evt->type == BLE_NUS_EVT_RX_DATA)
    {
        switch (p_evt->params.rx_data.p_data[0])
        {
        case 0x46:                         //文件相关指令
        {
            switch (p_evt->params.rx_data.p_data[1])
            {
            case 0x43:                  //FC 文件创建
            {
                ble_rx_cmd=CMD_FILE_CREATE;
                memcpy(&ble_rx_data[0],&p_evt->params.rx_data.p_data[3],p_evt->params.rx_data.length-3);
                ble_rx_data[p_evt->params.rx_data.length-3]=0;
            }
            break;
            case 0x44:                  //D12 删除本用户数据
            {
                ble_rx_cmd=CMD_FILE_DATA;
                memcpy(&ble_rx_data[0],&p_evt->params.rx_data.p_data[3],p_evt->params.rx_data.length-3);
            }
            break;
            case 0x45:                  //D12 删除本用户数据
            {
                ble_rx_cmd=CMD_FILE_ERASE;
                memcpy(&ble_rx_data[0],&p_evt->params.rx_data.p_data[3],p_evt->params.rx_data.length-3);
            }
            default:
                break;
            }
        }
        case 0x44:                         //离线相关指令
        {
            switch (p_evt->params.rx_data.p_data[1])
            {
            case 0x3D:                  //D13 删除所有用户数据
            {
                ble_rx_cmd=CMD_USER_DELEAT_ALL_DATA;
            }
            break;
            case 0x3C:                  //D12 删除本用户数据
            {
                ble_rx_cmd=CMD_USER_DELEAT_USER_DATA;
            }
            break;
            case 0x3B:                  //D11 查询数据空间
            {
                ble_rx_cmd=CMD_USER_GET_SPEACE_INF;
            }
            break;
            case 0x3A:                  //D10 查询离线情况
            {
                ble_rx_cmd=CMD_USER_GET_UPLAOD_INF;
            }
            break;
            case 0x39:                  //D9 设置自动离线时间
            {
                ble_rx_cmd=CMD_USER_SET_AFL_TIME;
                ble_rx_data[0]=p_evt->params.rx_data.p_data[3];
            }
            break;
            case 0x38:                  //D8 设置自动离线开关
            {
                ble_rx_cmd=CMD_SYSTEM_AUTO_OFL_SWHICHE;
                ble_rx_data[0]=p_evt->params.rx_data.p_data[3];
            }
            break;
            case 0x37:                  //D7 查询主机时间
            {
                ble_rx_cmd=CMD_USER_GET_SYS_TIME;
            }
            break;
            case 0x36:                  //D6 设置主机时间
            {
                ble_rx_cmd=CMD_USER_SET_SYS_TIME;
                memcpy(&ble_rx_data[0],&p_evt->params.rx_data.p_data[3],14);
            }
            break;
            case 0x35:                 //D5 设置WIFI密码
            {
                ble_rx_cmd=CMD_USER_GET_WIFI_CTRECT;
            }
            break;
            case 0x34:                 //D4 设置WIFI密码
            {
                memcpy(&ble_rx_data[0],&p_evt->params.rx_data.p_data[3],17);
                ble_rx_cmd=CMD_USER_SET_WIFI_CTRECT;
            }
            break;
            case 0x33:                 //D3 获取WIFI名称
            {
                ble_rx_cmd=CMD_USER_GET_WIFI_NAME;
            }
            break;
            case 0x32:                 //D2 设置WIFI名称
            {
                memcpy(&ble_rx_data[0],&p_evt->params.rx_data.p_data[3],17);
                ble_rx_cmd=CMD_USER_SET_WIFI_NAME;
            }
            break;
            default:
                break;
            }
        }
        break;
        case 0x43:                         //用户相关指令
        {
            switch (p_evt->params.rx_data.p_data[1])
            {
            case 0x37:                 //C7 获取用户体征
            {
                ble_rx_cmd=CMD_USER_GET_INF;
            }
            break;
            case 0x36:                 //C6 设置用户体征
            {
                ble_rx_cmd=CMD_USER_SET_INF;
                memcpy(&ble_rx_data[0],&p_evt->params.rx_data.p_data[3],4);
            }
            break;
            case 0x35:               //C5 获取用户名称
            {
                ble_rx_cmd=CMD_USER_GET_NAME;
            }
            break;
            case 0x34:               //C4 设置用户名称
            {
                ble_rx_cmd=CMD_USER_SET_NAME;
                memcpy(&ble_rx_data[0],&p_evt->params.rx_data.p_data[3],17);
            }
            break;
            case 0x33:              //C3 获取用户ID
            {
                ble_rx_cmd=CMD_USER_GET_ID;
            }
            break;
            case 0x32:              //C2 设置用户ID
            {
                ble_rx_cmd=CMD_USER_SET_ID;
                memcpy(&ble_rx_data[0],&p_evt->params.rx_data.p_data[3],4);
            }
            break;
            default:
                break;
            }
        }
        break;
        case 0x42:                         //在线相关指令
        {
            switch (p_evt->params.rx_data.p_data[1])
            {
            case 0x38:                //B8  LED控制
            {
                ble_rx_cmd=CMD_SYSTEM_LED_CTR;
                memcpy(&ble_rx_data[0],&p_evt->params.rx_data.p_data[3],2);
            }
            break;
            case 0x37:                //B7 计步器步数
            {
                ble_rx_cmd=CMD_SYSTEM_GET_STEPS;
            }
            break;
            case 0x36:                //B6  计步器重置
            {
                ble_rx_cmd=CMD_SYSTEM_RESET_STEP;
            }
            break;
            case 0x33:                //B3  关机模式
            {
                ble_rx_cmd=CMD_SYSTEM_SLEEP_MODE;
            }
            break;
            case 0x32:                //B2  关机模式
            {
                ble_rx_cmd=CMD_SYSTEM_AIR_RESET;
            }
            break;
            case 0x31:                //B1  重启进入OTA
            {
                ble_rx_cmd=CMD_SYSTEM_OTA_MODE;
            }
            break;
            default:
                break;
            }
        }
        break;
        case 0x41:                         //系统相关指令
        {
            switch (p_evt->params.rx_data.p_data[1])
            {
            case 0x37:               //A7 用户广播设置
            {
                ble_rx_cmd=CMD_USER_ADV_BOOD_DATA;
                memcpy(&ble_rx_data[0],&p_evt->params.rx_data.p_data[3],17);
            }
            break;
            case 0x36:              //A6 广播超时设置
            {
                ble_rx_cmd=CMD_SYSTEM_ADVOUT_DELAY;
                ble_rx_data[0]= p_evt->params.rx_data.p_data[3];
            }
            break;
            case 0x35:              //A5 关机延时设置
            {
                ble_rx_cmd=CMD_SYSTEM_CLOSE_DELAY;
                ble_rx_data[0]= p_evt->params.rx_data.p_data[3];
            }
            break;
            case 0x34:               //A4 开机延时设置
            {
                ble_rx_cmd=CMD_SYSTEM_OPEN_DELAY;
                ble_rx_data[0]= p_evt->params.rx_data.p_data[3];
            }
            break;
            case 0x33:                //A3 ACC数据开关
            {
                ble_rx_cmd=CMD_SYSTEM_ACC_SWHICHE;
                ble_rx_data[0]= p_evt->params.rx_data.p_data[3];
            }
            break;
            case 0x32:                 //A2 ECG数据开关
            {
                ble_rx_cmd=CMD_SYSTEM_ECG_SWHICHE;
                ble_rx_data[0]= p_evt->params.rx_data.p_data[3];
            }
            break;
            case 0x31:                 //A1 主机状态查询
            {
                ble_rx_cmd=CMD_SYSTEM_CHECK_STATUS;
            }
            break;
            default:
                break;
            }
        }
        break;
        default:
            break;
        }
        // vTaskResume(m_bluetooth_thread);
    }
}

void ble_nus_sevices_init(void)
{
    uint32_t       err_code;
    ble_nus_init_t nus_init;
    memset(&nus_init, 0, sizeof(nus_init));
    nus_init.data_handler = nus_data_handler;
    err_code = ble_nus_init(&m_nus, &nus_init);
    APP_ERROR_CHECK(err_code);
}








void app_ble_cmd_process(void)
{
    unsigned short  step_counter;
    unsigned int  err_code;
    rtc_time_t com_tm;
    switch( ble_rx_cmd)                     //即使应答命令处理
    {
    case CMD_USER_SET_SYS_TIME:       //D6   设置系统时间
    {
        rtc_time_t app_time_seting;
        app_time_seting.tm_year=(ble_rx_data[0]-0x30)*1000+(ble_rx_data[1]-0x30)*100+(ble_rx_data[2]-0x30)*10+(ble_rx_data[3]-0x30);
        app_time_seting.tm_mon= (ble_rx_data[4]-0x30)*10+(ble_rx_data[5]-0x30);
        app_time_seting.tm_mday = (ble_rx_data[6]-0x30)*10+(ble_rx_data[7]-0x30);
        app_time_seting.tm_hour= (ble_rx_data[8]-0x30)*10+(ble_rx_data[9]-0x30);
        app_time_seting.tm_min = (ble_rx_data[10]-0x30)*10+(ble_rx_data[11]-0x30);
        app_time_seting.tm_sec= (ble_rx_data[12]-0x30)*10+(ble_rx_data[13]-0x30);
        app_times_setting(app_time_seting);
        send_len[0]=5;
        ble_tx_data[0]=0x44;
        ble_tx_data[1]=0x36;
        memcpy(&ble_tx_data[2],info_OK,3);
        ble_nus_cmd_send(&m_nus, ble_tx_data, send_len, m_nus_conn_handle);
        ble_rx_cmd=0;
    }
    break;
    case CMD_USER_GET_WIFI_CTRECT:    //D5 获取 WIFI密码
    {
        ble_rx_cmd=0;
    }
    break;
    case CMD_USER_GET_INF:              //C7 获取用户体征
    {
        ble_rx_cmd=0;
    }
    break;
    case CMD_USER_SET_INF:              //C6 设置用户体征
    {
        ble_rx_cmd=0;
    }
    break;
    case CMD_USER_GET_NAME:              //C5 获取用户名称
    {
        ble_rx_cmd=0;
    }
    break;
    case CMD_USER_SET_NAME:              //C4 设置用户名称
    {
        send_len[0]=5;
        ble_tx_data[0]=0x43;
        ble_tx_data[1]=0x34;
        memcpy(&ble_tx_data[2],info_OK,3);
        ble_nus_cmd_send(&m_nus, ble_tx_data, send_len, m_nus_conn_handle);
        ble_rx_cmd=0;
    }
    break;
    case CMD_USER_GET_ID:            //  C3 获取用户ID
    {
        ble_rx_cmd=0;
    }
    break;
    case CMD_USER_SET_ID:            //  C2 设置用户ID
    {
        ble_rx_cmd=0;
    }
    break;
    case CMD_USER_ADV_BOOD_DATA:       //A7 用户绑定设置
    {
        ble_rx_cmd=0;
    }
    break;
    case CMD_SYSTEM_ACC_SWHICHE:       //A3 ACC开关
    {
        send_len[0]=5;
        ble_tx_data[0]=0x41;
        ble_tx_data[1]=0x33;
        memcpy(&ble_tx_data[2],info_OK,3);
        ble_nus_cmd_send(&m_nus, ble_tx_data, send_len, m_nus_conn_handle);
        ble_rx_cmd=0;
    }
    break;
    case CMD_SYSTEM_ECG_SWHICHE:       //A2 ECG开关
    {
        send_len[0]=5;
        ble_tx_data[0]=0x41;
        ble_tx_data[1]=0x32;
        memcpy(&ble_tx_data[2],info_OK,3);
        ble_nus_cmd_send(&m_nus, ble_tx_data, send_len, m_nus_conn_handle);
        ble_rx_cmd=0;
    }
    break;
    case CMD_SYSTEM_CHECK_STATUS:     //A1 主机主机状态查询
    {
        send_len[0]=12;
        ble_tx_data[0]=0x41;
        ble_tx_data[1]=0x31;
        ble_tx_data[2]=0x2B;
        ble_tx_data[8]= sys_config.boot_key_delay;
        ble_tx_data[9]= sys_config.shutdown_key_delay;
        ble_tx_data[10]= sys_config.ble_adv_timeout;
        // ble_tx_data[11]= usr_config.bood_data[16];
        ble_nus_cmd_send(&m_nus, ble_tx_data, send_len, m_nus_conn_handle);
        ble_rx_cmd=0;
    }
    break;
    case CMD_SYSTEM_GET_STEPS:        //B7 计步器步数
    {
        //   LSM6DS3_X_Get_Step_Counter(&step_counter);
        send_len[0]=5;
        ble_tx_data[0]=0x42;
        ble_tx_data[1]=0x37;
        ble_tx_data[2]=0x2B;
        ble_tx_data[3]=step_counter>>8;
        ble_tx_data[4]=step_counter;
        err_code=ble_nus_cmd_send(&m_nus,ble_tx_data, send_len, m_nus_conn_handle);
        if(err_code)ble_nus_cmd_send(&m_nus,ble_tx_data, send_len, m_nus_conn_handle);    //尝试重发
        ble_rx_cmd=0;
    }
    break;
    case CMD_SYSTEM_RESET_STEP:      //B6  计步器重置
    {
        //   LSM6DS3_reset_Step_Counter();
        send_len[0]=5;
        ble_tx_data[0]=0x42;
        ble_tx_data[1]=0x36;
        memcpy(&ble_tx_data[2],info_OK,3);
        ble_nus_cmd_send(&m_nus, ble_tx_data, send_len, m_nus_conn_handle);
        ble_rx_cmd=0;
    }
    break;
    case CMD_SYSTEM_SLEEP_MODE:      //B3  睡眠模式
    {
        ble_rx_cmd=0;
    }
    break;
    case CMD_SYSTEM_AIR_RESET:      //B2  空中重启
    {
        NVIC_SystemReset();
        ble_rx_cmd=0;
    }
    break;
    case CMD_SYSTEM_OTA_MODE:        //B1  重启进入OTA
    {
        err_code=sd_power_gpregret_clr(0,0xFF);
        APP_ERROR_CHECK(err_code);
        err_code=sd_power_gpregret_set(0,BOOTLOADER_OTA_START);
        APP_ERROR_CHECK(err_code);
        NVIC_SystemReset();
        ble_rx_cmd=0;
    }
    break;
    default:
        break;
    }
}
void app_ble_send_process(void)
{
    ble_bas_battery_level_update(&m_bas,sys_status.current_percent,NULL);
}
