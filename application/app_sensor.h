#ifndef __APP_SENSOR_H
#define __APP_SENSOR_H
#include "FreeRTOS.h"
#include "semphr.h"

extern int bat_adc;
extern int full_adc;
extern int fr24_adc;
extern int hall_adc;

extern SemaphoreHandle_t  sensor_task_Semaphore;

void send_semapthore_from_isr(void);
int app_sensors_data_poll(short *data);
void app_sensor_thread(void * ass);
void app_sensor_24g_ant_mode(int mode);
#endif