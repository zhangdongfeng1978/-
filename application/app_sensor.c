#include "app_sensor.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "bsp_adc.h"
#include "app_config.h"
#include "app_display.h"
#include "bsp.h"

int current_show_mode;


extern void cdc_acm_user_data_send(unsigned char * data, int len);

QueueHandle_t sensor_data_xQueue;

static short sensor_data[3];
static int sensor_data_filter[4];
static int sample_times=0;
static short radio_data[5];
static int radio_data_index=0;

short fr_buff[192];
int fr_buff_index;
extern short frfull_data_buffer[100];
extern short fr24g_data_buffer[100];
int full_adc;
int fr24_adc;
int hall_adc;
int bat_adc;

static unsigned char cdc_send_data[10];
SemaphoreHandle_t  sensor_task_Semaphore = NULL;
static BaseType_t xHigherPriorityTaskWoken;





void send_semapthore_from_isr(void)
{
    xHigherPriorityTaskWoken=pdFALSE;
    xSemaphoreGiveFromISR(sensor_task_Semaphore,&xHigherPriorityTaskWoken);
}




static void app_sensor_init()
{
    nrf_gpio_cfg_output(FR_POWER_EN);
    nrf_gpio_pin_set(FR_POWER_EN);
    nrf_gpio_cfg_output(FR24_CTR1_EN);
    nrf_gpio_pin_clear(FR24_CTR1_EN);
    nrf_gpio_cfg_output(FR24_CTR2_EN);
    nrf_gpio_pin_set(FR24_CTR2_EN);
    //connect to fr\\rf2 2.4g
    sensor_data_xQueue= xQueueCreate(20, 6);   //250*0.2 =
    sensor_task_Semaphore = xSemaphoreCreateBinary();
}
static void app_sensors_data_push(short *data)//
{
    long err= xQueueSend(sensor_data_xQueue, data, NULL);
    //if(err!= 1)  acc_overflow++;
}
int app_sensors_data_poll(short *data)
{
    if (xQueueReceive(sensor_data_xQueue, data, 0) == pdTRUE)
        return 1;
    else
        return -1;
}
void app_sensor_24g_ant_mode(int mode)
{
    if(mode==0)  //2.4g
    {
        nrf_gpio_pin_clear(FR24_CTR1_EN);
        nrf_gpio_pin_set(FR24_CTR2_EN);
    }
    else  //ble
    {
        nrf_gpio_pin_set(FR24_CTR1_EN);
        nrf_gpio_pin_clear(FR24_CTR2_EN);
    }
}
void app_sensor_thread(void * ass)  //100hz
{
    UNUSED_PARAMETER(ass);
    app_sensor_init();
    bsp_saadc_init();
    while (1)
    {
        xSemaphoreTake( sensor_task_Semaphore, portMAX_DELAY );
        fr_buff[fr_buff_index++]=frfull_data_buffer[80];
        if(fr_buff_index>=10)
        {
            full_adc=frfull_data_buffer[80];
            fr24_adc=fr24g_data_buffer[80];
            bat_adc=bat_voltage_adc;
            hall_adc =hall_voltage_adc;
            fr_buff_index=0;
            if( current_show_mode ==1)
               ; //fullband_windosw_update();
            else if(  current_show_mode==2)
                ;//wifi_windosw_update();
            else if(  current_show_mode==3)
                ;//hall_windosw_update();
						
						else if( current_show_mode==5)
						{
							cdc_send_data[0]=full_adc&0xFF;
							cdc_send_data[1]=full_adc>>8;
							cdc_send_data[2]=fr24_adc&0xFF;
							cdc_send_data[3]=fr24_adc>>8;
							cdc_acm_user_data_send(cdc_send_data,4);
						}
        }
//			 short max_in100=0;
//			    for(int i=0;i<100;i++)
//					{
//					   if(max_in100<need_decode_data_buffer[i]) //radio
//							 max_in100=need_decode_data_buffer[i];
//					}
//			   radio_data[sample_times]= max_in100;
//        sensor_data_filter[1]+=irpdl_voltage_adc;//irpdl
//        sensor_data_filter[2]+=hall_voltage_adc; //hall
//		  	sensor_data_filter[3]+=batty_voltage_adc; //bettary
//        sample_times++;
//        if(sample_times>=5)
//        {
//            sample_times=0;
//            //sensor_data[0]=sensor_data_filter[0]/5-sys_config.radio_offset;
//					  short max_in500=0;
//					  for(int i=0;i<5;i++)
//					{
//					   if(max_in500<radio_data[i])
//							 max_in500=radio_data[i];
//					}
//					  sensor_data[0]=max_in500;
//            sensor_data[1]=sensor_data_filter[1]/5-sys_config.irpdl_offset;
//            sensor_data[2]=sensor_data_filter[2]/5-sys_config.hall_offset;
//					  batty_voltage_avg=sensor_data_filter[3]/5;
//            app_sensors_data_push(sensor_data); //20hz
//            sensor_data_filter[0]=0;
//            sensor_data_filter[1]=0;
//            sensor_data_filter[2]=0;
//			      sensor_data_filter[3]=0;
//        }
    }
}