
#include "ble_bas.h"
#include "bsp.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include <nrfx_gpiote.h>
#include "FreeRTOS.h"
#include "task.h"
#include "app_system.h"
#include "app_config.h"
#include "app_sensor.h"
#include "app_times.h"
#include "app_display.h"
#include "app_power.h"
#include "app_protocol.h"

static TaskHandle_t m_system_thread =NULL;  /**< Definition of mode thread. */
static TaskHandle_t m_sensors_thread =NULL; /**< Definition of power thread. */
static TaskHandle_t m_display_thread =NULL;  /**< Definition of cmd thread. */
TaskHandle_t GUI_thread =NULL;  /**< Definition of mode thread. */
TaskHandle_t Tim_thread =NULL;  /**< Definition of mode thread. */








void app_system_thread(void * ass)
{
    UNUSED_PARAMETER(ass);    //  FS2_init();
    while (1)
    {
		
		lv_timer_handler();
		
        // app_sysmode_process();
        // app_ble_cmd_process();
        app_power_preass();
        vTaskDelay(1023); // 1000ms
    }
}

void app_system_init(void)
{
    app_config_init();
    app_times_init();
    app_power_init();
    //      建立系统应用  蓝牙 电源 显示
    if (xTaskCreate(app_sensor_thread, "ser", 100, NULL, 3, &m_sensors_thread) == 1)
    {
        sys_status.app_task_sensor = 1;
    }
//    if (xTaskCreate(app_display_thread, "dis", 1000, NULL, 2, &m_display_thread) == 1)
//    {
//        sys_status.app_task_display = 1;
//    }
    if (xTaskCreate(app_system_thread, "sys", 300, NULL, 1, &m_system_thread) == 1)
    {
        sys_status.app_task_bluetooth = 1;
    }
    // 负责
}