#include "app_power.h"
#include "FreeRTOS.h"
#include "app_config.h"
#include "ble_bas.h"
#include "bsp.h"
#include "bsp_adc.h"

#include "task.h"

#include "app_display.h"
#include "app_protocol.h"

#define D_BATTARY_PERCENT_100_VOLTAGE 4100
#define D_BATTARY_PERCENT_90_VOLTAGE 4020 // 0.08

#define D_BATTARY_PERCENT_80_VOLTAGE 3920 // 0.10
#define D_BATTARY_PERCENT_70_VOLTAGE 3870 // 0.08
#define D_BATTARY_PERCENT_60_VOLTAGE 3800 // 0.07

#define D_BATTARY_PERCENT_50_VOLTAGE 3760 // 0.04
#define D_BATTARY_PERCENT_40_VOLTAGE 3700 // 0.04
#define D_BATTARY_PERCENT_30_VOLTAGE 3600 // 0.10

#define D_BATTARY_PERCENT_20_VOLTAGE 3510 // 0.09
#define D_BATTARY_PERCENT_10_VOLTAGE 3440 // 0.07
#define D_BATTARY_PERCENT_00_VOLTAGE 3000 // 0.44


//?????¨²??
#define C_BATTARY_PERCENT_100_VOLTAGE 4200
#define C_BATTARY_PERCENT_90_VOLTAGE 4170 // 0.03

#define C_BATTARY_PERCENT_80_VOLTAGE 4150 // 0.02
#define C_BATTARY_PERCENT_70_VOLTAGE 4140 // 0.01
#define C_BATTARY_PERCENT_60_VOLTAGE 4120 // 0.02

#define C_BATTARY_PERCENT_50_VOLTAGE 4060 // 0.06
#define C_BATTARY_PERCENT_40_VOLTAGE 3980 // 0.08
#define C_BATTARY_PERCENT_30_VOLTAGE 3890 // 0.09

#define C_BATTARY_PERCENT_20_VOLTAGE 3760 // 0.13
#define C_BATTARY_PERCENT_10_VOLTAGE 3660 // 0.10
#define C_BATTARY_PERCENT_00_VOLTAGE 3000 // 0.66



unsigned char bat_power_status = 1;
unsigned char bat_power_percent;
unsigned char last_bat_power_percent;
int battary_voltage; //????
float bat_voltage;
int battary_voltage_err; //????
int adc_times=10;
int power_run_times;
//int bat_voltage_adc=0;
static void battary_status_update(void)
{
//    if (nrf_gpio_pin_read(BATTARY_CHAG_PIN))
//        sys_status.charger_status = 0;
//    else
//        sys_status.charger_status = 1;
    if (nrf_gpio_pin_read(BATTARY_5V_PIN))
    {
        sys_status.usb_5v_status = 1;
        adc_times=1;
    }
    else
    {
        sys_status.usb_5v_status = 0;
        adc_times=10;
    }
}
static void calculate_battary_percent(void)
{
    int power_percent;
    if (bat_voltage_adc != 0)
    {
        bat_voltage = bat_voltage_adc / 650.000; // 6.0v - 4096  full scale  v_adc=0.4v_batary  1/6.1¡¤???
        bat_voltage_adc = 0;
        battary_voltage = (int)(bat_voltage * 1000.0f); //??????????%5
        if (sys_status.charger_status == 0)
        {
            if (battary_voltage >= D_BATTARY_PERCENT_90_VOLTAGE)
            {
                power_percent = 90 + (battary_voltage - D_BATTARY_PERCENT_90_VOLTAGE) / 8;
            }
            else if (battary_voltage >= D_BATTARY_PERCENT_80_VOLTAGE)
            {
                power_percent = 80 + (battary_voltage - D_BATTARY_PERCENT_80_VOLTAGE) / 10;
            }
            else if (battary_voltage >= D_BATTARY_PERCENT_70_VOLTAGE)
            {
                power_percent = 70 + (battary_voltage - D_BATTARY_PERCENT_70_VOLTAGE) / 8;
            }
            else if (battary_voltage >= D_BATTARY_PERCENT_60_VOLTAGE)
            {
                power_percent = 60 + (battary_voltage - D_BATTARY_PERCENT_60_VOLTAGE) / 7;
            }
            else if (battary_voltage >= D_BATTARY_PERCENT_50_VOLTAGE)
            {
                power_percent = 50 + (battary_voltage - D_BATTARY_PERCENT_50_VOLTAGE) / 4;
            }
            else if (battary_voltage >= D_BATTARY_PERCENT_40_VOLTAGE)
            {
                power_percent = 40 + (battary_voltage - D_BATTARY_PERCENT_40_VOLTAGE) / 4;
            }
            else if (battary_voltage >= D_BATTARY_PERCENT_30_VOLTAGE)
            {
                power_percent = 30 + (battary_voltage - D_BATTARY_PERCENT_30_VOLTAGE) / 10;
            }
            else if (battary_voltage >= D_BATTARY_PERCENT_20_VOLTAGE)
            {
                power_percent = 20 + (battary_voltage - D_BATTARY_PERCENT_20_VOLTAGE) / 9;
            }
            else if (battary_voltage >= D_BATTARY_PERCENT_10_VOLTAGE)
            {
                power_percent = 10 + (battary_voltage - D_BATTARY_PERCENT_10_VOLTAGE) / 7;
            }
            else if (battary_voltage >= D_BATTARY_PERCENT_00_VOLTAGE)
            {
                power_percent = 0 + (battary_voltage - D_BATTARY_PERCENT_00_VOLTAGE) / 44;
            }
            else
                power_percent = 0;
        }
        else if (sys_status.charger_status == 1)
        {
            if (battary_voltage >= C_BATTARY_PERCENT_90_VOLTAGE)
            {
                power_percent = 90 + (battary_voltage - C_BATTARY_PERCENT_90_VOLTAGE) / 3;
            }
            else if (battary_voltage >= C_BATTARY_PERCENT_80_VOLTAGE)
            {
                power_percent = 80 + (battary_voltage - C_BATTARY_PERCENT_80_VOLTAGE) / 2;
            }
            else if (battary_voltage >= C_BATTARY_PERCENT_70_VOLTAGE)
            {
                power_percent = 70 + (battary_voltage - C_BATTARY_PERCENT_70_VOLTAGE) / 1;
            }
            else if (battary_voltage >= C_BATTARY_PERCENT_60_VOLTAGE)
            {
                power_percent = 60 + (battary_voltage - C_BATTARY_PERCENT_60_VOLTAGE) / 2;
            }
            else if (battary_voltage >= C_BATTARY_PERCENT_50_VOLTAGE)
            {
                power_percent = 50 + (battary_voltage - C_BATTARY_PERCENT_50_VOLTAGE) / 6;
            }
            else if (battary_voltage >= C_BATTARY_PERCENT_40_VOLTAGE)
            {
                power_percent = 40 + (battary_voltage - C_BATTARY_PERCENT_40_VOLTAGE) / 8;
            }
            else if (battary_voltage >= C_BATTARY_PERCENT_30_VOLTAGE)
            {
                power_percent = 30 + (battary_voltage - C_BATTARY_PERCENT_30_VOLTAGE) / 9;
            }
            else if (bat_voltage >= C_BATTARY_PERCENT_20_VOLTAGE)
            {
                power_percent = 20 + (battary_voltage - C_BATTARY_PERCENT_20_VOLTAGE) / 13;
            }
            else if (battary_voltage >= C_BATTARY_PERCENT_10_VOLTAGE)
            {
                power_percent = 10 + (bat_voltage - C_BATTARY_PERCENT_10_VOLTAGE) / 10;
            }
            else if (battary_voltage >= C_BATTARY_PERCENT_00_VOLTAGE)
            {
                power_percent = 0 + (battary_voltage - C_BATTARY_PERCENT_00_VOLTAGE) / 66;
            }
            else
                power_percent = 0;
        }
        if (power_percent > 100)
            power_percent = 100;
        //          if( sys_status.usb_5v_status==0)
        //          {
        //
        sys_status.current_percent=power_percent;
        //           else sys_status.battary_low=0;
        //
        //          }
    }
}


void app_power_init(void)
{
    unsigned char usb_power_status;
    nrf_gpio_cfg_input(BATTARY_5V_PIN, NRF_GPIO_PIN_NOPULL);
    nrf_gpio_cfg_output(SYSTEM_POWER_EN);
    nrf_gpio_pin_set(SYSTEM_POWER_EN);
//    usb_power_status = nrf_gpio_pin_read(BATTARY_CHAG_PIN);
//    if (usb_power_status == 1)
//        last_bat_power_percent = 0;
//    else
//        last_bat_power_percent = 100;
}


void app_power_preass(void)
{
    power_run_times++;
    battary_status_update();
    if(power_run_times%adc_times==0)
    {
        calculate_battary_percent();
        if(sys_status.bluetooth_status==2)
            app_ble_send_process();
    }
    Time_Call(&sys_status.sys_times);
 
}