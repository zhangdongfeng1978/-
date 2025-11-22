#ifndef BSP_ADC_H
#define BSP_ADC_H


extern int saadc_event_handler_flag;


extern short bat_voltage_adc;
extern short hall_voltage_adc;
void bsp_saadc_init(void);
void bsp_adc_wakeup_prepare(void);
void bsp_adc_sleep_prepare(void);
void bsp_adc_sample_start(void);
#endif
