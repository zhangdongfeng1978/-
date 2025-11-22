#include "bsp_adc.h"
#include "nrfx_saadc.h"
#include "bsp.h"
#include "nrfx_timer.h"
#include "nrfx_ppi.h"
#include "app_sensor.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "FreeRTOS.h"
#include "timers.h"

static const nrfx_timer_t AD8232_clok = NRFX_TIMER_INSTANCE(2);
nrf_ppi_channel_t ppi_channel1;

static nrf_saadc_channel_config_t FRFULL_channel0_config = NRFX_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN0); /**< Channel instance. Default configuration used. */
static nrf_saadc_channel_config_t FR24G_channel5_config = NRFX_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN7);  //5 -> 7
static nrf_saadc_channel_config_t HALL_channel6_config = NRFX_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN2);   //6 -> 2
static nrf_saadc_channel_config_t BAT_channel7_config = NRFX_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN5); 	 //7 -> 5
extern  void saadc_event_handler(nrfx_saadc_evt_t const * p_event);

nrf_saadc_value_t  adc_result[4];
short bat_voltage_adc;
short hall_voltage_adc;

short frfull_data_buffer[100];
short fr24g_data_buffer[100];
short hall_data_buffer[100];
short bat_data_buffer[100];

int adc_run_times=0;
short need_decode_data_buffer[100];


static void timer_handler(nrf_timer_event_t event_type, void * p_context)
{
}

static void saadc_sampling_event_init(void)
{
    ret_code_t err_code;
    uint32_t time_ticks;
    //Configure TIMER_LED for generating simple light effect - leds on board will invert his state one after the other.
    nrfx_timer_config_t timer_cfg = NRFX_TIMER_DEFAULT_CONFIG;
    timer_cfg.bit_width = NRF_TIMER_BIT_WIDTH_32;
    timer_cfg.frequency=NRF_TIMER_FREQ_250kHz;//NRF_TIMER_FREQ_31250Hz
    err_code = nrfx_timer_init(&AD8232_clok, &timer_cfg, timer_handler);
    APP_ERROR_CHECK(err_code);
    time_ticks=24;
    //the timer frequency is  31250 /250 125  10khz  100uS
    nrfx_timer_extended_compare(&AD8232_clok, NRF_TIMER_CC_CHANNEL0, time_ticks, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, false);
    nrfx_timer_enable(&AD8232_clok);
		
    uint32_t timer_compare_event_addr = nrfx_timer_event_address_get(&AD8232_clok, NRF_TIMER_EVENT_COMPARE0);
    uint32_t saadc_sample_task_addr   = nrfx_saadc_sample_task_get();
    /* setup ppi channel so that timer compare event is triggering sample task in SAADC */
    err_code = nrfx_ppi_channel_alloc(&ppi_channel1);
    APP_ERROR_CHECK(err_code);
    err_code = nrfx_ppi_channel_assign(ppi_channel1, timer_compare_event_addr, saadc_sample_task_addr);
    APP_ERROR_CHECK(err_code);
}


static void saadc_sampling_event_enable(void)
{
    ret_code_t err_code = nrfx_ppi_channel_enable(ppi_channel1);
    APP_ERROR_CHECK(err_code);
}

extern void device_adc_init(void);
void bsp_saadc_init(void)
{
    ret_code_t err_code;
		nrfx_saadc_config_t config = NRFX_SAADC_DEFAULT_CONFIG;
		config.oversample = NRF_SAADC_OVERSAMPLE_DISABLED;
		config.resolution = NRF_SAADC_RESOLUTION_12BIT;  // 12位分辨率
    nrfx_saadc_init(&config, saadc_event_handler);
		device_adc_init();
    saadc_sampling_event_init();
    saadc_sampling_event_enable();
}

nrf_saadc_value_t ADC_IR;
nrf_saadc_channel_config_t ch0_cfg =NRFX_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN0);
void ADC_Once_IR(void)
{
		ret_code_t err_code;
    //ch0_cfg =NRFX_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN0);
    ch0_cfg.gain=NRF_SAADC_GAIN1_6;
		ch0_cfg.pin_p      = NRF_SAADC_INPUT_AIN0,
		ch0_cfg.reference = NRF_SAADC_REFERENCE_INTERNAL;  // 0.6V参考
    nrfx_saadc_channel_init(0,&ch0_cfg);
		//err_code = nrfx_saadc_buffer_convert(&ADC_IR, 0);
		//APP_ERROR_CHECK(err_code);
		err_code = nrfx_saadc_sample_convert(0,&ADC_IR);
		APP_ERROR_CHECK(err_code);
		nrfx_saadc_channel_uninit(0);
}
 
 
nrf_saadc_value_t ADC_Hall;
nrf_saadc_channel_config_t ch1_cfg =NRFX_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN2);
void ADC_Once_Hall(void)
{
		ret_code_t err_code;
		//nrf_saadc_channel_config_t ch1_cfg =NRFX_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN2);
    ch1_cfg.gain=NRF_SAADC_GAIN1_6;
		ch1_cfg.pin_p      = NRF_SAADC_INPUT_AIN2,
		ch1_cfg.reference = NRF_SAADC_REFERENCE_INTERNAL;  // 0.6V参考
    nrfx_saadc_channel_init(0,&ch1_cfg);
		//err_code = nrfx_saadc_buffer_convert(&ADC_Hall, 1);
		//APP_ERROR_CHECK(err_code);
		err_code = nrfx_saadc_sample_convert(0,&ADC_Hall);  
		APP_ERROR_CHECK(err_code);
		nrfx_saadc_channel_uninit(0);
}



/*
			(3.15 - 2.55) / 5 = 0.12

			//3.15@4.2V
			3.03 + 0.12 = 3.15
			2.91 + 0.12 = 3.03
			2.79 + 0.12 = 2.91
			2.67 + 0.12 = 2.79
			2.55 + 0.12 = 2.67
			//2.55@3.4V

			参考电压：3.6
			分辨率：  12bit
					
					电压值		   ADC值
			5档： 3.03 - 3.15	3447 - 3584
			4档： 2.91 - 3.03	3310 - 3447
			3档： 2.79 - 2.91	3174 - 3310
			2档： 2.67 - 2.79	3038 - 3174
			1档： 2.55 - 2.67	2901 - 3038
*/
nrf_saadc_value_t ADC_Batty;
nrf_saadc_channel_config_t ch2_cfg =NRFX_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN5);
void ADC_Once_Batty(void)
{
		ret_code_t err_code;
		//nrf_saadc_channel_config_t ch2_cfg =NRFX_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN5);
    ch2_cfg.gain=NRF_SAADC_GAIN1_6;
		ch2_cfg.pin_p      = NRF_SAADC_INPUT_AIN5,
		ch2_cfg.reference = NRF_SAADC_REFERENCE_INTERNAL;  // 0.6V参考
    nrfx_saadc_channel_init(0,&ch2_cfg); 
		//err_code =nrfx_saadc_buffer_convert(&ADC_Batty, 2);
		//APP_ERROR_CHECK(err_code);
		err_code = nrfx_saadc_sample_convert(0,&ADC_Batty);
		APP_ERROR_CHECK(err_code);
		nrfx_saadc_channel_uninit(0);
}


nrf_saadc_value_t ADC_Wless;
nrf_saadc_channel_config_t ch3_cfg =NRFX_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN7); 
void ADC_Once_Wless(void)
{
		ret_code_t err_code;
		//nrf_saadc_channel_config_t ch3_cfg =NRFX_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN7); 
    ch3_cfg.gain=NRF_SAADC_GAIN1_6;
		ch3_cfg.pin_p      = NRF_SAADC_INPUT_AIN7,
		ch3_cfg.reference = NRF_SAADC_REFERENCE_INTERNAL;  // 0.6V参考
    nrfx_saadc_channel_init(0,&ch3_cfg);
		//err_code = nrfx_saadc_buffer_convert(&ADC_Wless, 3);
		//APP_ERROR_CHECK(err_code);
		err_code = nrfx_saadc_sample_convert(0,&ADC_Wless);
		APP_ERROR_CHECK(err_code);
		nrfx_saadc_channel_uninit(0);
}

nrf_saadc_value_t  adc_result[4];
void device_adc_init(void)
{
	ret_code_t err_code;
	
	
	#if 0
	ch0_cfg.gain=NRF_SAADC_GAIN1_6;
	ch0_cfg.pin_p      = NRF_SAADC_INPUT_AIN0,	//IR
	ch0_cfg.reference = NRF_SAADC_REFERENCE_INTERNAL;  // 0.6V参考
	err_code = nrfx_saadc_channel_init(0,&ch0_cfg);
	APP_ERROR_CHECK(err_code);
		
	ch1_cfg.gain=NRF_SAADC_GAIN1_6;
	ch1_cfg.pin_p      = NRF_SAADC_INPUT_AIN2, //Hall
	ch1_cfg.reference = NRF_SAADC_REFERENCE_INTERNAL;  // 0.6V参考
	err_code = nrfx_saadc_channel_init(1,&ch1_cfg);
	APP_ERROR_CHECK(err_code);
	
	ch2_cfg.gain=NRF_SAADC_GAIN1_6;
	ch2_cfg.pin_p      = NRF_SAADC_INPUT_AIN5, //Batty
	ch2_cfg.reference = NRF_SAADC_REFERENCE_INTERNAL;  // 0.6V参考
	err_code = nrfx_saadc_channel_init(2,&ch2_cfg); 
	APP_ERROR_CHECK(err_code);
	
	ch3_cfg.gain=NRF_SAADC_GAIN1_6;
	ch3_cfg.pin_p      = NRF_SAADC_INPUT_AIN7, //Wless
	ch3_cfg.reference = NRF_SAADC_REFERENCE_INTERNAL;  // 0.6V参考
	err_code = nrfx_saadc_channel_init(3,&ch3_cfg);
	APP_ERROR_CHECK(err_code);
	
	err_code = nrfx_saadc_buffer_convert(adc_result, 4*2);
	APP_ERROR_CHECK(err_code);
	
	#endif
	
	ch3_cfg.gain=NRF_SAADC_GAIN1_6;
	ch3_cfg.pin_p      = NRF_SAADC_INPUT_AIN7, //Wless
	ch3_cfg.reference = NRF_SAADC_REFERENCE_INTERNAL;  // 0.6V参考
	err_code = nrfx_saadc_channel_init(3,&ch3_cfg);
	APP_ERROR_CHECK(err_code);
	
	err_code = nrfx_saadc_buffer_convert(adc_result, 2);
	APP_ERROR_CHECK(err_code);
	
}

void device_adc_init_dinit(int flag)
{
	if(flag){
		ret_code_t err_code;
		ch0_cfg.gain=NRF_SAADC_GAIN1_6;
		ch0_cfg.pin_p      = NRF_SAADC_INPUT_AIN0,	//IR
		ch0_cfg.reference = NRF_SAADC_REFERENCE_INTERNAL;  // 0.6V参考
		err_code = nrfx_saadc_channel_init(0,&ch0_cfg);
		APP_ERROR_CHECK(err_code);
			
		ch1_cfg.gain=NRF_SAADC_GAIN1_6;
		ch1_cfg.pin_p      = NRF_SAADC_INPUT_AIN2, //Hall
		ch1_cfg.reference = NRF_SAADC_REFERENCE_INTERNAL;  // 0.6V参考
		err_code = nrfx_saadc_channel_init(1,&ch1_cfg);
		APP_ERROR_CHECK(err_code);
		
		ch2_cfg.gain=NRF_SAADC_GAIN1_6;
		ch2_cfg.pin_p      = NRF_SAADC_INPUT_AIN5, //Batty
		ch2_cfg.reference = NRF_SAADC_REFERENCE_INTERNAL;  // 0.6V参考
		err_code = nrfx_saadc_channel_init(2,&ch2_cfg); 
		APP_ERROR_CHECK(err_code);
		
		ch3_cfg.gain=NRF_SAADC_GAIN1_6;
		ch3_cfg.pin_p      = NRF_SAADC_INPUT_AIN7, //Wless
		ch3_cfg.reference = NRF_SAADC_REFERENCE_INTERNAL;  // 0.6V参考
		err_code = nrfx_saadc_channel_init(3,&ch3_cfg);
		APP_ERROR_CHECK(err_code);
	}else{
		ret_code_t err_code;
		err_code = nrfx_saadc_channel_uninit(0);
		APP_ERROR_CHECK(err_code);

		err_code = nrfx_saadc_channel_uninit(1);
		APP_ERROR_CHECK(err_code);

		err_code = nrfx_saadc_channel_uninit(2);
		APP_ERROR_CHECK(err_code);
		
		err_code = nrfx_saadc_channel_uninit(3);
		APP_ERROR_CHECK(err_code);
		
	}
}

nrf_saadc_value_t  adc_result_;
int saadc_event_handler_flag =0;

void beeper_start(void);
void beeper_stop(void);

int beep_cout =0;  //16

int  BEEP_ON_OFF_NUM = 800;
void saadc_event_handler(nrfx_saadc_evt_t const * p_event)
{
		 //beep
		 extern int sound_flag;
		 static int lock_flag =0;
		 if(sound_flag){
			static int sound_num =0;
			if(beep_cout){
//				if(beep_cout>0 && beep_cout<=2){ if(lock_flag==0) BEEP_ON_OFF_NUM = 1000;
//				}else if(beep_cout>2 && beep_cout<=4){ if(lock_flag==0) BEEP_ON_OFF_NUM = 900;
//				}else if(beep_cout>4 && beep_cout<=6){ if(lock_flag==0)  BEEP_ON_OFF_NUM = 800;
//				}else if(beep_cout>6 && beep_cout<=8){ if(lock_flag==0)    BEEP_ON_OFF_NUM = 700;
//				}else if(beep_cout>8 && beep_cout<=10){ if(lock_flag==0)   BEEP_ON_OFF_NUM = 600;
//				}else if(beep_cout>10 && beep_cout<=12){ if(lock_flag==0)  BEEP_ON_OFF_NUM = 500;
//				}else if(beep_cout>12 && beep_cout<=14){ if(lock_flag==0)   BEEP_ON_OFF_NUM = 400;
//				}else if(beep_cout>14 && beep_cout<=16){ if(lock_flag==0)   BEEP_ON_OFF_NUM = 300;}
				
//				if(beep_cout==1){				if(lock_flag==0) sound_num=0; BEEP_ON_OFF_NUM = 850;
//				}else if(beep_cout==2){	if(lock_flag==0) sound_num=0; BEEP_ON_OFF_NUM = 800;
//				}else if(beep_cout==3){	if(lock_flag==0) sound_num=0; BEEP_ON_OFF_NUM = 750;
//				}else if(beep_cout==4){	if(lock_flag==0) sound_num=0; BEEP_ON_OFF_NUM = 700;
//				}else if(beep_cout==5){	if(lock_flag==0) sound_num=0; BEEP_ON_OFF_NUM = 650;
//				}else if(beep_cout==6){	if(lock_flag==0) sound_num=0; BEEP_ON_OFF_NUM = 600;
//				}else if(beep_cout==7){	if(lock_flag==0) sound_num=0; BEEP_ON_OFF_NUM = 550;
//				}else if(beep_cout==8){ if(lock_flag==0) sound_num=0; BEEP_ON_OFF_NUM = 500;
//				}else if(beep_cout==9){	if(lock_flag==0) sound_num=0; BEEP_ON_OFF_NUM = 450;
//				}else if(beep_cout==10){if(lock_flag==0) sound_num=0; BEEP_ON_OFF_NUM = 400;
//				}else if(beep_cout==11){if(lock_flag==0) sound_num=0; BEEP_ON_OFF_NUM = 350;
//				}else if(beep_cout==12){if(lock_flag==0) sound_num=0; BEEP_ON_OFF_NUM = 300;
//				}else if(beep_cout==13){if(lock_flag==0) sound_num=0; BEEP_ON_OFF_NUM = 250;
//				}else if(beep_cout==14){if(lock_flag==0) sound_num=0; BEEP_ON_OFF_NUM = 200;}


				if(beep_cout==1){				if(lock_flag==0)  BEEP_ON_OFF_NUM = 850;
				}else if(beep_cout==2){	if(lock_flag==0)  BEEP_ON_OFF_NUM = 800;
				}else if(beep_cout==3){	if(lock_flag==0)  BEEP_ON_OFF_NUM = 750;
				}else if(beep_cout==4){	if(lock_flag==0)  BEEP_ON_OFF_NUM = 700;
				}else if(beep_cout==5){	if(lock_flag==0)  BEEP_ON_OFF_NUM = 650;
				}else if(beep_cout==6){	if(lock_flag==0)  BEEP_ON_OFF_NUM = 600;
				}else if(beep_cout==7){	if(lock_flag==0)  BEEP_ON_OFF_NUM = 550;
				}else if(beep_cout==8){ if(lock_flag==0)  BEEP_ON_OFF_NUM = 500;
				}else if(beep_cout==9){	if(lock_flag==0)  BEEP_ON_OFF_NUM = 450;
				}else if(beep_cout==10){if(lock_flag==0)  BEEP_ON_OFF_NUM = 400;
				}else if(beep_cout==11){if(lock_flag==0)  BEEP_ON_OFF_NUM = 350;
				}else if(beep_cout==12){if(lock_flag==0)  BEEP_ON_OFF_NUM = 300;
				}else if(beep_cout==13){if(lock_flag==0)  BEEP_ON_OFF_NUM = 250;
				}else if(beep_cout==14){if(lock_flag==0)  BEEP_ON_OFF_NUM = 200;}


				sound_num ++;
				if(sound_num == BEEP_ON_OFF_NUM){
					if(lock_flag ==0){
						beeper_start();
						lock_flag =1;
					}
				}else if(sound_num >= BEEP_ON_OFF_NUM * 2){
					if(lock_flag ==1){
						beeper_stop();
						lock_flag =0;
					}
					sound_num =0;
				}
					

//				if(sound_num < BEEP_ON_OFF_NUM){
//					if(lock_flag ==0){
//						beeper_start();
//						lock_flag =1;
//					}
//				}else if(sound_num > BEEP_ON_OFF_NUM   &&  sound_num < 2 * BEEP_ON_OFF_NUM){//}else if(sound_num > BEEP_ON_OFF_NUM * 2){
//					if(lock_flag ==1){
//						beeper_stop();
//						//sound_num =0;
//						lock_flag =0;
//					}
//				}else if(sound_num > 2 * BEEP_ON_OFF_NUM){
//					if(lock_flag ==1){
//						beeper_stop();
//						lock_flag =0;
//					}
//					sound_num =0;
//				}
			}else{
				if(lock_flag){
					lock_flag =0;
					beeper_stop();
				}
				sound_num =0;
			}
		}
		
		
		
		ret_code_t err_code;
    if(p_event->type == NRFX_SAADC_EVT_DONE)
    {
        if(adc_run_times<100){
				
				fr24g_data_buffer[adc_run_times]=adc_result_;
				
				memset(&adc_result_, 0, sizeof(adc_result_));
				ch0_cfg.gain=NRF_SAADC_GAIN1_6;
				ch0_cfg.pin_p      = NRF_SAADC_INPUT_AIN0,	//IR
				ch0_cfg.reference = NRF_SAADC_REFERENCE_INTERNAL;  // 0.6V参考
				err_code = nrfx_saadc_channel_init(0,&ch0_cfg);
				APP_ERROR_CHECK(err_code);
				nrfx_saadc_sample_convert(0, &adc_result_);
				frfull_data_buffer[adc_run_times]=adc_result_;
					
				#if 0	
				memset(&adc_result_, 0, sizeof(adc_result_));
				ch1_cfg.gain=NRF_SAADC_GAIN1_6;
				ch1_cfg.pin_p      = NRF_SAADC_INPUT_AIN2, //Hall
				ch1_cfg.reference = NRF_SAADC_REFERENCE_INTERNAL;  // 0.6V参考
				err_code = nrfx_saadc_channel_init(1,&ch1_cfg);
				APP_ERROR_CHECK(err_code);
				nrfx_saadc_sample_convert(1,&adc_result_);
				hall_data_buffer[adc_run_times]=adc_result_;
				#endif
				
				memset(&adc_result_, 0, sizeof(adc_result_));
				ch2_cfg.gain=NRF_SAADC_GAIN1_6;
				ch2_cfg.pin_p      = NRF_SAADC_INPUT_AIN5, //Batty
				ch2_cfg.reference = NRF_SAADC_REFERENCE_INTERNAL;  // 0.6V参考
				err_code = nrfx_saadc_channel_init(2,&ch2_cfg); 
				APP_ERROR_CHECK(err_code);
				nrfx_saadc_sample_convert(2,&adc_result_);
				bat_data_buffer[adc_run_times]=adc_result_;
							
							
				memset(&adc_result_, 0, sizeof(adc_result_));
				ch3_cfg.gain=NRF_SAADC_GAIN1_6;
				ch3_cfg.pin_p      = NRF_SAADC_INPUT_AIN7, //Wless
				ch3_cfg.reference = NRF_SAADC_REFERENCE_INTERNAL;  // 0.6V参考
				err_code = nrfx_saadc_channel_init(3,&ch3_cfg);
				APP_ERROR_CHECK(err_code);
				
				err_code = nrfx_saadc_buffer_convert(&adc_result_, 2);
				APP_ERROR_CHECK(err_code);

				
				adc_run_times++;
				
				
				#if 0
            frfull_data_buffer[adc_run_times]=adc_result[0];
						hall_data_buffer[adc_run_times]=adc_result[1];
						bat_data_buffer[adc_run_times]=adc_result[2];
						fr24g_data_buffer[adc_run_times]=adc_result[3];
					  ret_code_t err_code = nrfx_saadc_buffer_convert(adc_result, 4*2);
						APP_ERROR_CHECK(err_code);
						adc_run_times++;
				#endif
						
						
						
        }else{
				
				
				#if 0
						adc_run_times=0;
						saadc_event_handler_flag =1;            
						frfull_data_buffer[adc_run_times]=adc_result[0];
						hall_data_buffer[adc_run_times]=adc_result[1];
						bat_data_buffer[adc_run_times]=adc_result[2];
						fr24g_data_buffer[adc_run_times]=adc_result[3];
							
					   ret_code_t err_code = nrfx_saadc_buffer_convert(adc_result, 4*2);
						 APP_ERROR_CHECK(err_code);
             //frfull_data_buffer[adc_run_times]=adc_result[0];
				     //fr24g_data_buffer[adc_run_times++]=adc_result[3];
						 
							//hall_voltage_adc=adc_result[2];
					    //bat_voltage_adc=adc_result[3];
				#endif			



					
						adc_run_times=0;
						saadc_event_handler_flag =1;  
					
						//fr24g_data_buffer[adc_run_times]=adc_result_;
						memset(&adc_result_, 0, sizeof(adc_result_));
						ch3_cfg.gain=NRF_SAADC_GAIN1_6;
						ch3_cfg.pin_p      = NRF_SAADC_INPUT_AIN7, //Wless
						ch3_cfg.reference = NRF_SAADC_REFERENCE_INTERNAL;  // 0.6V参考
						err_code = nrfx_saadc_channel_init(3,&ch3_cfg);
						APP_ERROR_CHECK(err_code);
						
						err_code = nrfx_saadc_buffer_convert(&adc_result_, 2);
						APP_ERROR_CHECK(err_code);




							

				}
    }
}


static int ADC_Wless_arr[100] ={0};
static int ADC_Wless_num =0;
static nrf_saadc_value_t ADC_Wless_t =0;
void device_adc_deal(void)
{
	
	ret_code_t err_code;
	
//	nrfx_saadc_channel_init(0,&ch0_cfg);
//	err_code = nrfx_saadc_sample_convert(0,&ADC_IR);
//	APP_ERROR_CHECK(err_code);
//	nrfx_saadc_channel_uninit(0);


//	nrfx_saadc_channel_init(0,&ch1_cfg);
//	err_code = nrfx_saadc_sample_convert(0,&ADC_Hall);  
//	APP_ERROR_CHECK(err_code);
//	nrfx_saadc_channel_uninit(0);


//	nrfx_saadc_channel_init(0,&ch2_cfg); 
//	err_code = nrfx_saadc_sample_convert(0,&ADC_Batty);
//	APP_ERROR_CHECK(err_code);
//	nrfx_saadc_channel_uninit(0);


//	nrfx_saadc_channel_init(0,&ch3_cfg);
//	err_code = nrfx_saadc_sample_convert(0,&ADC_Wless_t);
//	APP_ERROR_CHECK(err_code);
//	nrfx_saadc_channel_uninit(0);


//	if(ADC_Wless_num<100){
//		//ADC_Wless_arr[ADC_Wless_num++] =ADC_Wless_t;
//	}else{
////	;
////		ADC_Wless_num =0;
//	}
	

	
//	if(ADC_Wless_num<100){
//		ADC_Wless_arr[ADC_Wless_num++] =ADC_Wless_t;
//	}else{
//		ADC_Wless_num =0;
////		int max_value =0;
////		for(int i=0; i<100; i++){
////			if(max_value<ADC_Wless_arr[i])
////				max_value=ADC_Wless_arr[i];
////			}
////			memset(ADC_Wless_arr, 0, sizeof(ADC_Wless_arr));
////			ADC_Wless =max_value;
//	}
	
	
	
	nrfx_saadc_channel_init(0,&ch3_cfg);
	nrfx_saadc_buffer_convert(&ADC_Wless, 1);
	APP_ERROR_CHECK(err_code);
	
	
	
	
	while(saadc_event_handler_flag ==0){
			vTaskDelay(10);
	}
	saadc_event_handler_flag =0;
	
	
//	nrfx_saadc_channel_uninit(0);

	
	
	
	
	

}




