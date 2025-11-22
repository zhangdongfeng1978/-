#include "misc.h"
#include "nrfx_timer.h"
#include "sdk_config.h"
#include "nrfx.h"
#include "nrfx_timer.h"
#include "nrfx_rtc.h"
#include "app_ui.h"

uint32_t Flash_Data =0;

void Misc_Init(void)
{
	MOTOR_Init();
	IR_Send_Init();
	pwm_init();
	batt_sta_init();
	Key_Init();
	BAT_POW_Init();
	LCD_Init();
	AD8314_Init();
	AD8314_Int_disable();
	hardware_timer_init(100);
	//storage_init();
	//Flash_Data =0;
	//read_flash(&Flash_Data);
	
	//load_data_from_flash(&Flash_Data);
//	sound_flag = Flash_Data & 0x01;
//	move_flag  = Flash_Data>>1 & 0x01;
}


/* 震动马达 */
void MOTOR_Init(void)
{
	nrf_gpio_cfg_output(MOTOR);
}

void MOTOR_Set(int pow)
{
	 if(pow ==0){
			nrf_gpio_pin_write(MOTOR, 0);
	 }else if(pow ==1){
			nrf_gpio_pin_write(MOTOR, 1);
	 }
}


void MOTOR_start_tim(int t) 
{
	if(move_flag){
		MOTOR_Set(1);
		nrf_delay_ms(t);
		MOTOR_Set(0);
	}
}


void MOTOR_start(void)
{
	if(move_flag){
		MOTOR_Set(1);
	}
}


void MOTOR_stop(void)
{
	MOTOR_Set(0);
}



/* 红外发射 */
void IR_Send_Init(void)
{
		nrf_gpio_cfg_output(IR_2);
		nrf_gpio_cfg_output(IR_3);
}

void IR_Send_Set(int pow)
{
	 if(pow ==0){
			nrf_gpio_pin_write(IR_2, 0);
			nrf_gpio_pin_write(IR_3, 0);
	 }else if(pow ==1){
			nrf_gpio_pin_write(IR_2, 1);
			nrf_gpio_pin_write(IR_3, 1);
	 }
}


/* 蜂鸣器 */
static nrfx_pwm_t m_pwm = PWM_INST_IDX;
static nrf_pwm_values_individual_t m_seq_values; // PWM值存储
static nrf_pwm_sequence_t const m_seq = {
    .values.p_individual = &m_seq_values,
    .length = NRF_PWM_VALUES_LENGTH(m_seq_values),
    .repeats = 0,
    .end_delay = 0
};

void pwm_handler(nrfx_pwm_evt_type_t event_type) //PWM事件处理函数
{
    //这里可以处理PWM事件
}

void pwm_init(void)
{
    nrfx_pwm_config_t config = {
        .output_pins = {
            BUZZER,          //通道0输出
            NRFX_PWM_PIN_NOT_USED,
            NRFX_PWM_PIN_NOT_USED,
            NRFX_PWM_PIN_NOT_USED
        },
        .irq_priority = NRFX_PWM_DEFAULT_CONFIG_IRQ_PRIORITY,
        .base_clock = NRF_PWM_CLK_16MHz,
        .count_mode = NRF_PWM_MODE_UP,
        .top_value = PWM_TOP_VALUE,
        .load_mode = NRF_PWM_LOAD_INDIVIDUAL,
        .step_mode = NRF_PWM_STEP_AUTO
    };
    //初始化PWM驱动
    nrfx_err_t err_code = nrfx_pwm_init(&m_pwm, &config, pwm_handler);
    APP_ERROR_CHECK(err_code);
    //设置占空比 (50%)
    m_seq_values.channel_0 = PWM_PULSE_WIDTH;
}

void beeper_start(void)
{
	if(sound_flag){
    nrfx_pwm_complex_playback(&m_pwm,
                              &m_seq,    // 主序列
                              &m_seq,    // 下一个序列（相同）
                              0,         // 循环次数 (0=无限循环)
                              NRFX_PWM_FLAG_STOP | NRFX_PWM_FLAG_LOOP
															);         // 起始延迟
															
	}
}
void beeper_stop(void)
{
	if(sound_flag){
    nrfx_pwm_stop(&m_pwm, true);
	}
}


void beeper_start_tim(int t) 
{
	if(t==0) return;
	if(sound_flag){
		beeper_start();
		nrf_delay_ms(t);
		beeper_stop();
	}
}

/*  红外 */
#define TIMER_INTERVAL_MS  50 //50ms   //100 //500  // 0.5秒定时
// 定时器实例配置
static const nrfx_timer_t m_timer = NRFX_TIMER_INSTANCE(1); //使用TIMER1
// 定时器中断处理
int IR_flag_mode =0;
void timer_handler(nrf_timer_event_t event_type, void *p_context) {
    if(event_type == NRF_TIMER_EVENT_COMPARE0){
			static int flag_num =0;
			flag_num++;
			if(IR_flag_mode ==0){
				nrf_gpio_pin_write(IR_2, 0);
				nrf_gpio_pin_write(IR_3, 0);
			}else if(IR_flag_mode ==1){   //常亮
				nrf_gpio_pin_write(IR_2, 1);
				nrf_gpio_pin_write(IR_3, 1);
			}else if(IR_flag_mode ==2){		//0.3s
				if(flag_num<=3){
					nrf_gpio_pin_write(IR_2, 1);
					nrf_gpio_pin_write(IR_3, 1);
				}else if(flag_num>=3 && flag_num<=6){
					nrf_gpio_pin_write(IR_2, 0);
					nrf_gpio_pin_write(IR_3, 0);
				}else{
					flag_num =0;
				}
			}else if(IR_flag_mode ==3){		//0.2s
				if(flag_num<=2){
					nrf_gpio_pin_write(IR_2, 1);
					nrf_gpio_pin_write(IR_3, 1);
				}else if(flag_num>=2 && flag_num<=4){
					nrf_gpio_pin_write(IR_2, 0);
					nrf_gpio_pin_write(IR_3, 0);
				}else{
					flag_num =0;
				}
			}else if(IR_flag_mode ==4){		//0.1s
				if(flag_num<=1){
					nrf_gpio_pin_write(IR_2, 1);
					nrf_gpio_pin_write(IR_3, 1);
				}else if(flag_num>=1 && flag_num<=2){
					nrf_gpio_pin_write(IR_2, 0);
					nrf_gpio_pin_write(IR_3, 0);
				}else{
					flag_num =0;
				}
			}
    }
}

// 初始化硬件定时器
void hardware_timer_init(int TIMER_MS){
    nrfx_timer_config_t timer_cfg = {
        .frequency = NRF_TIMER_FREQ_16MHz,  // 16MHz时钟
        .mode = NRF_TIMER_MODE_TIMER,
        .bit_width = NRF_TIMER_BIT_WIDTH_32, // 32位模式
        .interrupt_priority = 7,
        .p_context = NULL
    };
    
    // 初始化定时器
    ret_code_t err_code = nrfx_timer_init(&m_timer, &timer_cfg, timer_handler);
    APP_ERROR_CHECK(err_code);
    
    // 设置定时器比较值 (0.5秒)
		uint32_t ticks = nrfx_timer_ms_to_ticks(&m_timer, TIMER_INTERVAL_MS);
    nrfx_timer_extended_compare(&m_timer,
                              NRF_TIMER_CC_CHANNEL0,
                              ticks,
                              NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK,
                              true); // 启用中断
}

static bool hardware_timer_flag =0;
// 启动定时器
void hardware_timer_start(void){
		hardware_timer_flag =1;
    nrfx_timer_enable(&m_timer);
    nrfx_timer_clear(&m_timer);
    nrfx_timer_resume(&m_timer);
}

// 停止定时器
void hardware_timer_stop(void){
		if(hardware_timer_flag){
			hardware_timer_flag =0;
			nrfx_timer_pause(&m_timer);
			nrfx_timer_disable(&m_timer);
		}
}


/* 充电芯片 */
void batt_sta_init(void)  //GPIO_PIN_CNF_PULL_Disabled
{
	nrf_gpio_cfg_input(MCU_CHRG_,GPIO_PIN_CNF_PULL_Pullup);
	nrf_gpio_cfg_input(MCU_STDBY_,GPIO_PIN_CNF_PULL_Pullup);
	nrf_gpio_cfg_input(MCU_USB_STA_,GPIO_PIN_CNF_PULL_Disabled);
}

unsigned int batt_sta_update(void)
{
		int rel =0;  //1:正在充电  2:充电完成
		static int MCU_CHRG__time =0;
		static int MCU_CHRG__time_ =0;
		static int MCU_CHRG__flag =0;
		static int MCU_STDBY_time =0;
		static int MCU_STDBY_time_ =0;
		static int MCU_STDBY_flag =0;
//    if(nrf_gpio_pin_read(MCU_CHRG_)==0){	//正在充电
//				rel =1;
//    }else{															  //充电完成/未充电
//				rel =2;
//    }
		#if 1
    if(nrf_gpio_pin_read(MCU_CHRG_)==0){   //0：正在充电
				MCU_CHRG__time_ =0;
        MCU_CHRG__time++;
        if(MCU_CHRG__time==3){
						MCU_CHRG__time =0;
						MCU_CHRG__flag =0;
        }
    }else{																 //1：充电完成/没有接入电源
        MCU_CHRG__time=0;
				MCU_CHRG__time_++;
        if(MCU_CHRG__time_==3){
						MCU_CHRG__flag =1;
        }
    }
		
		
    if(nrf_gpio_pin_read(MCU_STDBY_)==0){		//充电完成
				MCU_STDBY_time_ =0;
        MCU_STDBY_time++;
        if(MCU_STDBY_time==3){
						MCU_STDBY_time =0;
						MCU_STDBY_flag =0;
        }
    }else{																	//正在充电/没有接入电源
        MCU_STDBY_time=0;
        MCU_STDBY_time_++;
        if(MCU_STDBY_time_==3){
						MCU_STDBY_flag =1;
        }
    }
		if(MCU_CHRG__flag == 0  &&  MCU_STDBY_flag == 1){				//正在充电
			rel =1;
		}else if(MCU_CHRG__flag == 1  &&  MCU_STDBY_flag == 0){	//充电完成
			rel =2;
		}else if(MCU_CHRG__flag == 1  &&  MCU_STDBY_flag == 1){	//未接输入电源
			rel =3;
		}
		#endif
		
		#if 0
		static int MCU_sta_time_ing =0;
		static int MCU_sta_time_no =0;
    if(nrf_gpio_pin_read(MCU_USB_STA_)){   //1：正在充电
				MCU_sta_time_no =0;
        MCU_sta_time_ing++;
        if(MCU_sta_time_ing==3){
						MCU_sta_time_ing =0;
						rel =1;
        }
    }else{																//0：充电完成/没有接入电源
        MCU_sta_time_ing=0;
				MCU_sta_time_no++;
        if(MCU_sta_time_no==3){
						MCU_sta_time_no =0;
						rel =3;
        }
    }
		#endif
		
		return rel;
}


#if 0
enum HALL_BIG_STEP{
	HALL_BIG_STEP17 = 3755,
	HALL_BIG_STEP16 = 3641,
	HALL_BIG_STEP15 = 3527,
	HALL_BIG_STEP14 = 3413,
	HALL_BIG_STEP13 = 3300,
	HALL_BIG_STEP12 = 3186,
	HALL_BIG_STEP11 = 3072,
	HALL_BIG_STEP10 = 2958,
	HALL_BIG_STEP9  = 2844,
	HALL_BIG_STEP8  = 2731,
	HALL_BIG_STEP7  = 2617,
	HALL_BIG_STEP6  = 2503,
	HALL_BIG_STEP5  = 2389,
	HALL_BIG_STEP4  = 2276,
	HALL_BIG_STEP3  = 2162,
	HALL_BIG_STEP2  = 2048,
	HALL_BIG_STEP1  = 1934
};

enum HALL_SMA_STEP{
	HALL_SMA_STEP17 = 0,
	HALL_SMA_STEP16 = 114,
	HALL_SMA_STEP15 = 228,
	HALL_SMA_STEP14 = 341,
	HALL_SMA_STEP13 = 455,
	HALL_SMA_STEP12 = 569,
	HALL_SMA_STEP11 = 682,
	HALL_SMA_STEP10 = 796,
	HALL_SMA_STEP9  = 910,
	HALL_SMA_STEP8  = 1024,
	HALL_SMA_STEP7  = 1138,
	HALL_SMA_STEP6  = 1252,
	HALL_SMA_STEP5  = 1365,
	HALL_SMA_STEP4  = 1479,
	HALL_SMA_STEP3  = 1593,
	HALL_SMA_STEP2  = 1707,
	HALL_SMA_STEP1  = 1820
};
#endif


#if 0

enum HALL_BIG_STEP{
	HALL_BIG_STEP17 = 3755,
	HALL_BIG_STEP16 = 3641,
	HALL_BIG_STEP15 = 3527,
	HALL_BIG_STEP14 = 3413,
	HALL_BIG_STEP13 = 3300,
	HALL_BIG_STEP12 = 3186,
	HALL_BIG_STEP11 = 3072,
	HALL_BIG_STEP10 = 2958,
	HALL_BIG_STEP9  = 2844,
	HALL_BIG_STEP8  = 2731,
	HALL_BIG_STEP7  = 2617,
	HALL_BIG_STEP6  = 2503,
	HALL_BIG_STEP5  = 2389,
	HALL_BIG_STEP4  = 2276,
	HALL_BIG_STEP3  = 2162,
	HALL_BIG_STEP2  = 2108,
	HALL_BIG_STEP1  = 2048//1934
};

//1850 - 3755     --->  1850 - 3755/3700


//1.65  --  1877


//0 - 1850       --->   1850 - 0/50

enum HALL_SMA_STEP{      
	HALL_SMA_STEP17 = 0,
	HALL_SMA_STEP16 = 114,
	HALL_SMA_STEP15 = 228,
	HALL_SMA_STEP14 = 341,
	HALL_SMA_STEP13 = 455,
	HALL_SMA_STEP12 = 569,
	HALL_SMA_STEP11 = 682,
	HALL_SMA_STEP10 = 796,
	HALL_SMA_STEP9  = 910,
	HALL_SMA_STEP8  = 1024,
	HALL_SMA_STEP7  = 1138,
	HALL_SMA_STEP6  = 1252,
	HALL_SMA_STEP5  = 1365,
	HALL_SMA_STEP4  = 1479,
	HALL_SMA_STEP3  = 1593,
	HALL_SMA_STEP2  = 1600,
	HALL_SMA_STEP1  = 1707
};

#endif


enum HALL_BIG_STEP{
	HALL_BIG_STEP17 = 32767,
	HALL_BIG_STEP16 = 28800,
	HALL_BIG_STEP15 = 26900,
	HALL_BIG_STEP14 = 25000,
	HALL_BIG_STEP13 = 23100,
	HALL_BIG_STEP12 = 21200,
	HALL_BIG_STEP11 = 19300,
	HALL_BIG_STEP10 = 17400,
	HALL_BIG_STEP9  = 15500,
	HALL_BIG_STEP8  = 13600,
	HALL_BIG_STEP7  = 11700,
	HALL_BIG_STEP6  = 9800,
	HALL_BIG_STEP5  = 7900,
	HALL_BIG_STEP4  = 6000,
	HALL_BIG_STEP3  = 4100,
	HALL_BIG_STEP2  = 2200,
	HALL_BIG_STEP1  = 1934//850//1934
};


int hall_sens[16] ={10, 20,  30,  40,  50,  60,  70,  80,  90,  100,  110,  120,  130,  140,  150,  180};

#if 0

int hall_data_deal(int value)
{
		int level =0;
		if((value>=HALL_BIG_STEP16-hall_sens[sensitivity_hall]) || (value<HALL_SMA_STEP16+hall_sens[sensitivity_hall])){
		//if((value>=HALL_BIG_STEP16-hall_sens[sensitivity_hall] && value<HALL_BIG_STEP17-hall_sens[sensitivity_hall]) || (value>=HALL_SMA_STEP17+hall_sens[sensitivity_hall] && value<HALL_SMA_STEP16+hall_sens[sensitivity_hall])){
			level =16;
		}else if((value>=HALL_BIG_STEP15-hall_sens[sensitivity_hall] && value<HALL_BIG_STEP16-hall_sens[sensitivity_hall]) || (value>=HALL_SMA_STEP16+hall_sens[sensitivity_hall] && value<HALL_SMA_STEP15+hall_sens[sensitivity_hall])){
			level =15;
		}else if((value>=HALL_BIG_STEP14-hall_sens[sensitivity_hall] && value<HALL_BIG_STEP15-hall_sens[sensitivity_hall]) || (value>=HALL_SMA_STEP15+hall_sens[sensitivity_hall] && value<HALL_SMA_STEP14+hall_sens[sensitivity_hall])){
			level =14;
		}else if((value>=HALL_BIG_STEP13-hall_sens[sensitivity_hall] && value<HALL_BIG_STEP14-hall_sens[sensitivity_hall]) || (value>=HALL_SMA_STEP14+hall_sens[sensitivity_hall] && value<HALL_SMA_STEP13+hall_sens[sensitivity_hall])){
			level =13;
		}else if((value>=HALL_BIG_STEP12-hall_sens[sensitivity_hall] && value<HALL_BIG_STEP13-hall_sens[sensitivity_hall]) || (value>=HALL_SMA_STEP13+hall_sens[sensitivity_hall] && value<HALL_SMA_STEP12+hall_sens[sensitivity_hall])){
			level =12;
		}else if((value>=HALL_BIG_STEP11-hall_sens[sensitivity_hall] && value<HALL_BIG_STEP12-hall_sens[sensitivity_hall]) || (value>=HALL_SMA_STEP12+hall_sens[sensitivity_hall] && value<HALL_SMA_STEP11+hall_sens[sensitivity_hall])){
			level =11;
		}else if((value>=HALL_BIG_STEP10-hall_sens[sensitivity_hall] && value<HALL_BIG_STEP11-hall_sens[sensitivity_hall]) || (value>=HALL_SMA_STEP11+hall_sens[sensitivity_hall] && value<HALL_SMA_STEP10+hall_sens[sensitivity_hall])){
			level =10;
		}else if((value>=HALL_BIG_STEP9-hall_sens[sensitivity_hall] && value<HALL_BIG_STEP10-hall_sens[sensitivity_hall]) || (value>=HALL_SMA_STEP10+hall_sens[sensitivity_hall] && value<HALL_SMA_STEP9+hall_sens[sensitivity_hall])){
			level =9;
		}else if((value>=HALL_BIG_STEP8-hall_sens[sensitivity_hall] && value<HALL_BIG_STEP9-hall_sens[sensitivity_hall]) || (value>=HALL_SMA_STEP9+hall_sens[sensitivity_hall] && value<HALL_SMA_STEP8+hall_sens[sensitivity_hall])){
			level =8;
		}else if((value>=HALL_BIG_STEP7-hall_sens[sensitivity_hall] && value<HALL_BIG_STEP8-hall_sens[sensitivity_hall]) || (value>=HALL_SMA_STEP8+hall_sens[sensitivity_hall] && value<HALL_SMA_STEP7+hall_sens[sensitivity_hall])){
			level =7;
		}else if((value>=HALL_BIG_STEP6-hall_sens[sensitivity_hall] && value<HALL_BIG_STEP7-hall_sens[sensitivity_hall]) || (value>=HALL_SMA_STEP7+hall_sens[sensitivity_hall] && value<HALL_SMA_STEP6+hall_sens[sensitivity_hall])){
			level =6;
		}else if((value>=HALL_BIG_STEP5-hall_sens[sensitivity_hall] && value<HALL_BIG_STEP6-hall_sens[sensitivity_hall]) || (value>=HALL_SMA_STEP6+hall_sens[sensitivity_hall] && value<HALL_SMA_STEP5+hall_sens[sensitivity_hall])){
			level =5;
		}else if((value>=HALL_BIG_STEP4-hall_sens[sensitivity_hall] && value<HALL_BIG_STEP5-hall_sens[sensitivity_hall]) || (value>=HALL_SMA_STEP5+hall_sens[sensitivity_hall] && value<HALL_SMA_STEP4+hall_sens[sensitivity_hall])){
			level =4;
		}else if((value>=HALL_BIG_STEP3-hall_sens[sensitivity_hall] && value<HALL_BIG_STEP4-hall_sens[sensitivity_hall]) || (value>=HALL_SMA_STEP4+hall_sens[sensitivity_hall] && value<HALL_SMA_STEP3+hall_sens[sensitivity_hall])){
			level =3;
		}else if((value>=HALL_BIG_STEP2-hall_sens[sensitivity_hall] && value<HALL_BIG_STEP3-hall_sens[sensitivity_hall]) || (value>=HALL_SMA_STEP3+hall_sens[sensitivity_hall] && value<HALL_SMA_STEP2+hall_sens[sensitivity_hall])){
			level =2;//level =2;
		}else if((value>=HALL_BIG_STEP1-hall_sens[sensitivity_hall] && value<HALL_BIG_STEP2-hall_sens[sensitivity_hall]) || (value>=HALL_SMA_STEP2+hall_sens[sensitivity_hall] && value<HALL_SMA_STEP1+hall_sens[sensitivity_hall])){
			level =1;//level =1;
		}else{
			level =0;
		}
		return level;
}

#endif


int hall_data_deal(int value)
{
		int level =0;
		if(value>=HALL_BIG_STEP16-hall_sens[sensitivity_hall]){
		//if((value>=HALL_BIG_STEP16-hall_sens[sensitivity_hall] && value<HALL_BIG_STEP17-hall_sens[sensitivity_hall]) || (value>=HALL_SMA_STEP17+hall_sens[sensitivity_hall] && value<HALL_SMA_STEP16+hall_sens[sensitivity_hall])){
			level =16;
		}else if(value>=HALL_BIG_STEP15-hall_sens[sensitivity_hall] && value<HALL_BIG_STEP16-hall_sens[sensitivity_hall]){
			level =15;
		}else if(value>=HALL_BIG_STEP14-hall_sens[sensitivity_hall] && value<HALL_BIG_STEP15-hall_sens[sensitivity_hall]){
			level =14;
		}else if(value>=HALL_BIG_STEP13-hall_sens[sensitivity_hall] && value<HALL_BIG_STEP14-hall_sens[sensitivity_hall]){
			level =13;
		}else if(value>=HALL_BIG_STEP12-hall_sens[sensitivity_hall] && value<HALL_BIG_STEP13-hall_sens[sensitivity_hall]){
			level =12;
		}else if(value>=HALL_BIG_STEP11-hall_sens[sensitivity_hall] && value<HALL_BIG_STEP12-hall_sens[sensitivity_hall]){
			level =11;
		}else if(value>=HALL_BIG_STEP10-hall_sens[sensitivity_hall] && value<HALL_BIG_STEP11-hall_sens[sensitivity_hall]){
			level =10;
		}else if(value>=HALL_BIG_STEP9-hall_sens[sensitivity_hall] && value<HALL_BIG_STEP10-hall_sens[sensitivity_hall]){
			level =9;
		}else if(value>=HALL_BIG_STEP8-hall_sens[sensitivity_hall] && value<HALL_BIG_STEP9-hall_sens[sensitivity_hall]){
			level =8;
		}else if(value>=HALL_BIG_STEP7-hall_sens[sensitivity_hall] && value<HALL_BIG_STEP8-hall_sens[sensitivity_hall]){
			level =7;
		}else if(value>=HALL_BIG_STEP6-hall_sens[sensitivity_hall] && value<HALL_BIG_STEP7-hall_sens[sensitivity_hall]){
			level =6;
		}else if(value>=HALL_BIG_STEP5-hall_sens[sensitivity_hall] && value<HALL_BIG_STEP6-hall_sens[sensitivity_hall]){
			level =5;
		}else if(value>=HALL_BIG_STEP4-hall_sens[sensitivity_hall] && value<HALL_BIG_STEP5-hall_sens[sensitivity_hall]){
			level =4;
		}else if(value>=HALL_BIG_STEP3-hall_sens[sensitivity_hall] && value<HALL_BIG_STEP4-hall_sens[sensitivity_hall]){
			level =3;
		}else if(value>=HALL_BIG_STEP2-hall_sens[sensitivity_hall] && value<HALL_BIG_STEP3-hall_sens[sensitivity_hall]){
			level =2;//level =2;
		}else if(value>=HALL_BIG_STEP1-hall_sens[sensitivity_hall] && value<HALL_BIG_STEP2-hall_sens[sensitivity_hall]){
			level =1;//level =1;
		}else{
			level =0;
		}
		return level;
}






#if 0
enum IR_STEP {
	IR_STEP17 = 3755,
	IR_STEP16 = 3572,
	IR_STEP15 = 3300,
	IR_STEP14 = 3072,
	IR_STEP13 = 2844,
	IR_STEP12 = 2617,
	IR_STEP11 = 2389,
	IR_STEP10 = 2162,
	IR_STEP9  = 1934,
	IR_STEP8  = 1707,
	IR_STEP7  = 1479,
	IR_STEP6  = 1252,
	IR_STEP5  = 1024,
	IR_STEP4  = 796,
	IR_STEP3  = 569,
	IR_STEP2  = 341,
	IR_STEP1  = 114
};
#endif

#if 0
enum IR_STEP {   // 2617  --  3755   step: 67
	IR_STEP17 = 3755,
	IR_STEP16 = 3622,
	IR_STEP15 = 3555,
	IR_STEP14 = 3488,
	IR_STEP13 = 3421,
	IR_STEP12 = 3354,
	IR_STEP11 = 3287,
	IR_STEP10 = 3220,
	IR_STEP9  = 3153,
	IR_STEP8  = 3086,
	IR_STEP7  = 3019,
	IR_STEP6  = 2952,
	IR_STEP5  = 2885,
	IR_STEP4  = 2818,
	IR_STEP3  = 2751,
	IR_STEP2  = 2684,
	IR_STEP1  = 2617
};
#endif


#if 0
enum IR_STEP {   // 2617  --  3755   step: 67
	IR_STEP17 = 3755,
	IR_STEP16 = 3705,
	IR_STEP15 = 3658,
	IR_STEP14 = 3611,
	IR_STEP13 = 3564,
	IR_STEP12 = 3517,
	IR_STEP11 = 3470,
	IR_STEP10 = 3423,
	IR_STEP9  = 3376,
	IR_STEP8  = 3329,
	IR_STEP7  = 3282,
	IR_STEP6  = 3235,
	IR_STEP5  = 3188,
	IR_STEP4  = 3141,
	IR_STEP3  = 3094,
	IR_STEP2  = 3047,
	IR_STEP1  = 3000
};
#endif

#if 1
enum IR_STEP {   // 2617  --  3755   step: 67
	IR_STEP17 = 3755,
	IR_STEP16 = 3750,
	IR_STEP15 = 3700,
	IR_STEP14 = 3650,
	IR_STEP13 = 3600,
	IR_STEP12 = 3550,
	IR_STEP11 = 3500,
	IR_STEP10 = 3480,
	IR_STEP9  = 3460,
	IR_STEP8  = 3440,
	IR_STEP7  = 3420,
	IR_STEP6  = 3400,
	IR_STEP5  = 3380,
	IR_STEP4  = 3360,
	IR_STEP3  = 3340,
	IR_STEP2  = 3320,
	IR_STEP1  = 3300
};
#endif

#define sens_offe 1000
int ir_sens[16] ={0, 134+sens_offe,  201+sens_offe,  268+sens_offe,  335+sens_offe,  402+sens_offe,  469+sens_offe,  536+sens_offe,  603+sens_offe,  670+sens_offe,  737+sens_offe,  804+sens_offe,  871+sens_offe,  938+sens_offe,  1005+sens_offe,  1072+sens_offe};
int ir_data_deal(int value)
{
		int level =0;
		if(value>=IR_STEP16-ir_sens[sensitivity_ir]){ //if(value>=IR_STEP16-ir_sens[sensitivity_ir] && value<IR_STEP17){
			level =16;
		}else if(value>=IR_STEP15-ir_sens[sensitivity_ir] && value<IR_STEP16-ir_sens[sensitivity_ir]){
			level =15;
		}else if(value>=IR_STEP14-ir_sens[sensitivity_ir] && value<IR_STEP15-ir_sens[sensitivity_ir]){
			level =14;
		}else if(value>=IR_STEP13-ir_sens[sensitivity_ir] && value<IR_STEP14-ir_sens[sensitivity_ir]){
			level =13;
		}else if(value>=IR_STEP12-ir_sens[sensitivity_ir] && value<IR_STEP13-ir_sens[sensitivity_ir]){
			level =12;
		}else if(value>=IR_STEP11-ir_sens[sensitivity_ir] && value<IR_STEP12-ir_sens[sensitivity_ir]){
			level =11;
		}else if(value>=IR_STEP10-ir_sens[sensitivity_ir] && value<IR_STEP11-ir_sens[sensitivity_ir]){
			level =10;
		}else if(value>=IR_STEP9-ir_sens[sensitivity_ir] && value<IR_STEP10-ir_sens[sensitivity_ir]){
			level =9;
		}else if(value>=IR_STEP8-ir_sens[sensitivity_ir] && value<IR_STEP9-ir_sens[sensitivity_ir]){
			level =8;
		}else if(value>=IR_STEP7-ir_sens[sensitivity_ir] && value<IR_STEP8-ir_sens[sensitivity_ir]){
			level =7;
		}else if(value>=IR_STEP6-ir_sens[sensitivity_ir] && value<IR_STEP7-ir_sens[sensitivity_ir]){
			level =6;
		}else if(value>=IR_STEP5-ir_sens[sensitivity_ir] && value<IR_STEP6-ir_sens[sensitivity_ir]){
			level =5;
		}else if(value>=IR_STEP4-ir_sens[sensitivity_ir] && value<IR_STEP5-ir_sens[sensitivity_ir]){
			level =4;
		}else if(value>=IR_STEP3-ir_sens[sensitivity_ir] && value<IR_STEP4-ir_sens[sensitivity_ir]){
			level =3;
		}else if(value>=IR_STEP2-ir_sens[sensitivity_ir] && value<IR_STEP3-ir_sens[sensitivity_ir]){
			level =2;
		}else if(value>=IR_STEP1-ir_sens[sensitivity_ir] && value<IR_STEP2-ir_sens[sensitivity_ir]){
			level =1;//level =1;
		}else{
			level =0;
		}
		return level;
}
		
#if 0
enum WIFI_STEP {
	WIFI_STEP16 = 1360,
	WIFI_STEP15 = 1275,
	WIFI_STEP14 = 1190,
	WIFI_STEP13 = 1105,
	WIFI_STEP12 = 1020,
	WIFI_STEP11 = 935,
	WIFI_STEP10 = 850,
	WIFI_STEP9  = 765,
	WIFI_STEP8  = 680,
	WIFI_STEP7  = 595,
	WIFI_STEP6  = 510,
	WIFI_STEP5  = 425,
	WIFI_STEP4  = 340,
	WIFI_STEP3  = 255,
	WIFI_STEP2  = 170,
	WIFI_STEP1  = 85,
};
#endif

#if 0
enum WIFI_STEP {				// 0-1020   64
	WIFI_STEP16 = 1020,
	WIFI_STEP15 = 960,
	WIFI_STEP14 = 896,
	WIFI_STEP13 = 832,
	WIFI_STEP12 = 768,
	WIFI_STEP11 = 704,
	WIFI_STEP10 = 640,
	WIFI_STEP9  = 576,
	WIFI_STEP8  = 512,
	WIFI_STEP7  = 448,
	WIFI_STEP6  = 384,
	WIFI_STEP5  = 320,
	WIFI_STEP4  = 256,
	WIFI_STEP3  = 192,
	WIFI_STEP2  = 128,
	WIFI_STEP1  = 64,
};
#endif


#if 0
enum WIFI_STEP {				// 0-576   38
	WIFI_STEP16 = 576,
	WIFI_STEP15 = 532,
	WIFI_STEP14 = 494,
	WIFI_STEP13 = 456,
	WIFI_STEP12 = 418,
	WIFI_STEP11 = 380,
	WIFI_STEP10 = 342,
	WIFI_STEP9  = 304,
	WIFI_STEP8  = 266,
	WIFI_STEP7  = 228,
	WIFI_STEP6  = 190,
	WIFI_STEP5  = 152,
	WIFI_STEP4  = 114,
	WIFI_STEP3  = 76,
	WIFI_STEP2  = 38,
	WIFI_STEP1  = 0,
};
#endif

#if 0
//0.1V - 1.5V				
enum WIFI_STEP {				// 0-576   38
	WIFI_STEP17 = 576,
	WIFI_STEP16 = 532,
	WIFI_STEP15 = 494,
	WIFI_STEP14 = 456,
	WIFI_STEP13 = 418,
	WIFI_STEP12 = 380,
	WIFI_STEP11 = 342,
	WIFI_STEP10  = 304,
	WIFI_STEP9  = 266,
	WIFI_STEP8  = 228,
	WIFI_STEP7  = 190,
	WIFI_STEP6  = 152,
	WIFI_STEP5  = 130,
	WIFI_STEP4  = 125,
	WIFI_STEP3  = 123,
	WIFI_STEP2  = 121,
	WIFI_STEP1  = 120,
};
#endif

//0.1V - 1.5V				
enum WIFI_STEP {				// 0-576   38
	WIFI_STEP17 = 576,
	WIFI_STEP16 = 532,
	WIFI_STEP15 = 494,
	WIFI_STEP14 = 456,
	WIFI_STEP13 = 418,
	WIFI_STEP12 = 380,
	WIFI_STEP11 = 342,
	WIFI_STEP10  = 304,
	WIFI_STEP9  = 266,
	WIFI_STEP8  = 228,
	WIFI_STEP7  = 190,
	WIFI_STEP6  = 180,
	WIFI_STEP5  = 170,
	WIFI_STEP4  = 165,
	WIFI_STEP3  = 160,
	WIFI_STEP2  = 155,
	WIFI_STEP1  = 150,
};



int wifi_sens[16] ={0, 2,  4,  6,  8,  10,  12,  14,  16,  18,  20,  22,  24,  26,  28,  38};
short wifi_db =0;
int wifi_data_deal(int value)
{
#if 1

		int level =0;
		if(value>=WIFI_STEP16-wifi_sens[sensitivity_wless]){//if(value>=WIFI_STEP15-wifi_sens[sensitivity_wless] && value<WIFI_STEP16-wifi_sens[sensitivity_wless]){
			level =16;
		}else if(value>=WIFI_STEP15-wifi_sens[sensitivity_wless] && value<WIFI_STEP16-wifi_sens[sensitivity_wless]){
			level =15;
		}else if(value>=WIFI_STEP14-wifi_sens[sensitivity_wless] && value<WIFI_STEP15-wifi_sens[sensitivity_wless]){
			level =14;
		}else if(value>=WIFI_STEP13-wifi_sens[sensitivity_wless] && value<WIFI_STEP14-wifi_sens[sensitivity_wless]){
			level =13;
		}else if(value>=WIFI_STEP12-wifi_sens[sensitivity_wless] && value<WIFI_STEP13-wifi_sens[sensitivity_wless]){
			level =12;
		}else if(value>=WIFI_STEP11-wifi_sens[sensitivity_wless] && value<WIFI_STEP12-wifi_sens[sensitivity_wless]){
			level =11;
		}else if(value>=WIFI_STEP10-wifi_sens[sensitivity_wless] && value<WIFI_STEP11-wifi_sens[sensitivity_wless]){
			level =10;
		}else if(value>=WIFI_STEP9-wifi_sens[sensitivity_wless] && value<WIFI_STEP10-wifi_sens[sensitivity_wless]){
			level =9;
		}else if(value>=WIFI_STEP8-wifi_sens[sensitivity_wless] && value<WIFI_STEP9-wifi_sens[sensitivity_wless]){
			level =8;
		}else if(value>=WIFI_STEP7-wifi_sens[sensitivity_wless] && value<WIFI_STEP8-wifi_sens[sensitivity_wless]){
			level =7;
		}else if(value>=WIFI_STEP6-wifi_sens[sensitivity_wless] && value<WIFI_STEP7-wifi_sens[sensitivity_wless]){
			level =6;
		}else if(value>=WIFI_STEP5-wifi_sens[sensitivity_wless] && value<WIFI_STEP6-wifi_sens[sensitivity_wless]){
			level =5;
		}else if(value>=WIFI_STEP4-wifi_sens[sensitivity_wless] && value<WIFI_STEP5-wifi_sens[sensitivity_wless]){
			level =4;
		}else if(value>=WIFI_STEP3-wifi_sens[sensitivity_wless] && value<WIFI_STEP4-wifi_sens[sensitivity_wless]){
			level =3;
		}else if(value>=WIFI_STEP2-wifi_sens[sensitivity_wless] && value<WIFI_STEP3-wifi_sens[sensitivity_wless]){
			level =2;
		}else if(value>=WIFI_STEP1-wifi_sens[sensitivity_wless] && value<WIFI_STEP2-wifi_sens[sensitivity_wless]){
			level =1;
		}else{
			level =0;
		}
		return level;
#endif
	  //wifi_db=value*3600/4096/21.3;	
}
		

int key_lock1 =0;
int key_lock2 =0;
int key_lock3 =0;
/* 按键 */
void Key_Init(void)
{
	nrf_gpio_cfg_input(BUTTON_UP,NRF_GPIO_PIN_NOPULL);
	nrf_gpio_cfg_input(BUTTON_POW,NRF_GPIO_PIN_NOPULL);
	nrf_gpio_cfg_input(BUTTON_DOWN,NRF_GPIO_PIN_NOPULL);
//	nrf_gpio_cfg_output(MCU_POW);
//	nrf_gpio_pin_write(MCU_POW, 0);
}
int key1_press_time=0;
int key2_press_time=0;
int key3_press_time=0;


int press_time_n =30;  //30 40


extern int layer;
static int BUTTON_POW_flag =0;
int Device_POW_flag =1;
extern unsigned int auto_pow_off_num;
unsigned int key_update(void)
{

		if(layer ==1){
			press_time_n =40;
		}else{
			press_time_n =15;
		}

    int key=0;
    if(nrf_gpio_pin_read(BUTTON_UP)==0){
        key1_press_time++;
        if(key1_press_time>=press_time_n){
						key1_press_time =0;
						//if(key_lock1){
						//	key_lock1 =0;
							key=1;
						//}
        }
    }else{
				//key_lock1 =1;
        key1_press_time=0;
    }
		
		#if 0
    if(nrf_gpio_pin_read(BUTTON_POW)){
        key2_press_time++;
				BUTTON_POW_flag =1;
//      if(key2_press_time==3){
//					key=2;
//      }
    }else{
				if(BUTTON_POW_flag){
					BUTTON_POW_flag =0;
					//判断按键时长
					if(key2_press_time>=100){//长按
							key2_press_time =0;
							//任何界面-长按关机-关闭屏幕
							if(Device_POW_flag){
								//关机
								Device_POW_flag =0;
								MOTOR_start_tim(300);
								sound_flag =0;
								move_flag =0;
								hardware_timer_stop(); //关闭定时器
								IR_Send_Set(0);  //关闭红外
								LCD_PWM_Set(0);
								BAT_POW_Set(0);
							}else if(Device_POW_flag ==0){
								//开机
								Device_POW_flag =1;
								LCD_PWM_Set(1);
								sound_flag =1;
								move_flag =1;
								//软件复位
								NVIC_SystemReset();
							}
					}else{		//短按
							key=2;
					}
				}
				key2_press_time=0;
    }
		#endif
		
		static bool BUTTON_POW_Lock =0;
		static bool BUTTON_POW_Lock_ =1;
		if(nrf_gpio_pin_read(BUTTON_POW)){//if(nrf_gpio_pin_read(BUTTON_POW)){
				//判断按键时长
				BUTTON_POW_Lock =1;
				if(key2_press_time++>=350){//长按
						BUTTON_POW_Lock =0;
						key2_press_time =0;
						//任何界面-长按关机-关闭屏幕
						if(Device_POW_flag  && BUTTON_POW_Lock_ ==0){
							BUTTON_POW_Lock_ =1;
							//关机
							Device_POW_flag =0;
							MOTOR_start_tim(300);
							sound_flag =0;
							move_flag =0;
							hardware_timer_stop(); //关闭定时器
							IR_Send_Set(0);  //关闭红外
							LCD_PWM_Set(0);
							BAT_POW_Set(0);
						}else if(Device_POW_flag ==0  && BUTTON_POW_Lock_ ==0){
							BUTTON_POW_Lock_ =1;
							//开机
							Device_POW_flag =1;
							LCD_PWM_Set(1);
							sound_flag =1;
							move_flag =1;
							//软件复位
							NVIC_SystemReset();
						}
				}
		}else{
				BUTTON_POW_Lock_ =0;
				if(BUTTON_POW_Lock){
					BUTTON_POW_Lock =0;
					key=2;
				}
				key2_press_time =0;
		}
	
	
	
    if(nrf_gpio_pin_read(BUTTON_DOWN)==0){
        key3_press_time++;
        if(key3_press_time>=press_time_n){
						key3_press_time =0;
						//if(key_lock3){
						//	key_lock3 =0;
							key=3;
						//}
        }
    }else{
				//key_lock3 =1;
        key3_press_time=0;
    }
		if(key!=0){
				//清空自动关机计数
				auto_pow_off_num =0;
				if(AD8314_Int_disable() ==1){
					key =0;
				}
		}
		
		return key;
}


/* 电源 */
void BAT_POW_Init(void)
{
	nrf_gpio_cfg_output(BAT_POW);
}

void BAT_POW_Set(int pow)
{
	if(pow ==0){
		nrf_gpio_cfg_input(MCU_CHRG_,GPIO_PIN_CNF_PULL_Disabled);
		nrf_gpio_cfg_input(MCU_STDBY_,GPIO_PIN_CNF_PULL_Disabled);
	}
	nrf_gpio_pin_write(BAT_POW, pow);
}



/* LCD屏 */
void LCD_Init(void)
{
	nrf_gpio_cfg_output(LCD_PWR_EN);
	nrf_gpio_cfg_output(BACKLIGHT_PWM);
}

void LCD_PWM_Set(int pow)
{
	nrf_gpio_pin_write(BACKLIGHT_PWM, pow);
	
}



//网络
void Net_Init(void)
{
	nrf_gpio_cfg_output(RF_POWER_EN_PIN);
}


void Net_Init_Pow_Set(int pow)
{
	nrf_gpio_pin_write(RF_POWER_EN_PIN, pow);
}


//内部flash
#include "fds.h"
#include "nrf_fstorage.h"
#include "nrf_fstorage_sd.h"

static bool m_fds_initialized = false;
static bool m_fds_writed = false;
//初始化FDS
static void fds_evt_handler(fds_evt_t const * p_evt)
{
    switch (p_evt->id){
        case FDS_EVT_INIT:
            if(p_evt->result == NRF_SUCCESS){
                m_fds_initialized = true;
                ;//NRF_LOG_INFO("FDS initialized successfully");
            }
            break;
        case FDS_EVT_WRITE:
            if (p_evt->result == NRF_SUCCESS) {
								m_fds_writed = true;
                ;//NRF_LOG_INFO("Write successful");
            }
            break;
        case FDS_EVT_UPDATE:
            if (p_evt->result == NRF_SUCCESS) {
								m_fds_writed = true;
                ;//NRF_LOG_INFO("Write successful");
            }
            break;
        default:
            break;
    }
}


void storage_init(void)
{
    ret_code_t ret = fds_register(fds_evt_handler);
    APP_ERROR_CHECK(ret);

    ret = fds_init();
    APP_ERROR_CHECK(ret);

    // 等待 FDS 初始化完成
    while(!m_fds_initialized){
        sd_app_evt_wait();
    }
}


#define FILE_ID    0x1000    // 自定义文件 ID
#define RECORD_KEY 0x0001    // 自定义记录 Key

//写入数据到Flash
void save_data_to_flash(uint32_t data)  
{
    fds_record_desc_t record_desc;
    fds_record_t record ={
			// 数据存储结构
			record.file_id           = FILE_ID,
			record.key               = RECORD_KEY,
			record.data.p_data       = &data,
			record.data.length_words = 1, // 1 word = 4 bytes
		};
    ret_code_t ret = fds_record_write(&record_desc, &record);
    if (ret != NRF_SUCCESS) {
        ;//NRF_LOG_ERROR("Write failed: 0x%X", ret);
    }
}



//从Flash读取数据
bool load_data_from_flash(uint32_t *p_data)
{
    fds_record_desc_t record_desc;
    fds_find_token_t  ftok;
		memset(&ftok, 0x00, sizeof(fds_find_token_t));
    ret_code_t ret = fds_record_find(FILE_ID, RECORD_KEY, &record_desc, &ftok);
    if(ret != NRF_SUCCESS){
        ;//NRF_LOG_ERROR("Record not found (0x%X)", ret);
        return false;
    }
    fds_flash_record_t flash_record;
    ret = fds_record_open(&record_desc, &flash_record);
    if (ret != NRF_SUCCESS){
        ;//NRF_LOG_ERROR("Failed to open record (0x%X)", ret);
        return false;
    }
    // 读取数据
    *p_data = *(uint32_t *)flash_record.p_data;
		
		//memcpy(p_data, flash_record.p_data, sizeof(uint32_t));
		
    ret = fds_record_close(&record_desc);
    if (ret != NRF_SUCCESS){
        ;//NRF_LOG_ERROR("Failed to close record (0x%X)", ret);
				return false;
    }
    return true;
}



//flash
#include "nrf.h"
#include "nrf_nvmc.h"



#define FLASH_START_ADDR  0xFE000  // 示例：从 512KB 开始（避开应用程序）  0xFE000
#define FLASH_PAGE_SIZE   4096        // 4KB
//#define FLASH_DATA_ADDR   (FLASH_START_ADDR + FLASH_PAGE_SIZE)  // 存储到第二个扇区
#define FLASH_DATA_ADDR   (FLASH_START_ADDR)  // 存储到第二个扇区


extern void nrfx_nvmc_word_write(uint32_t addr, uint32_t value);
extern nrfx_err_t nrfx_nvmc_page_erase(uint32_t addr);



void flash_erase_page(uint32_t page_address) 
{
    // 确保地址对齐到 4KB
    if (page_address % FLASH_PAGE_SIZE != 0) {
        return;
    }
    // 禁用中断（避免擦除期间被打断）
    __disable_irq();
    //nrf_nvmc_page_erase(page_address);
		//nrfx_nvmc_page_erase(page_address);
    __enable_irq();
}

void flash_write_data(uint32_t address, void *data, uint32_t length) {
    if (length % 4 != 0) {
        //NRF_LOG_ERROR("数据长度必须是 4 的倍数");
        return;
    }
    uint32_t *p_data = (uint32_t *)data;
    uint32_t words_to_write = length / 4;
    // 禁用中断
    __disable_irq();
    for (uint32_t i = 0; i < words_to_write; i++) {
        //nrf_nvmc_write_word(address + (i * 4), p_data[i]);
				nrfx_nvmc_word_write(address + (i * 4), p_data[i]);
    }
    __enable_irq();
}


void flash_read_data(uint32_t address, void *data, uint32_t length) {
    memcpy(data, (void *)address, length);
}


void write_flash(uint32_t data)
{
    // 1. 擦除目标扇区
    flash_erase_page(FLASH_DATA_ADDR);
		//flash_write_data(FLASH_DATA_ADDR, &data, sizeof(data));
}



void read_flash(uint32_t *data)
{
	flash_read_data(FLASH_DATA_ADDR, &data, 4);
}



//加速度计
#include "nrf_drv_twi.h"
#include <nrf_drv_twi.h>
#include <nrf_delay.h>
#include <nrf_gpio.h>
#include <nrf_drv_gpiote.h>
#define SC7A20H_ADDR 0x18  // I2C 地址（SA0=0）

// SC7A20H 寄存器定义
#define CTRL_REG1     0x20
#define CTRL_REG3     0x22
#define INT1_CFG      0x30
#define INT1_THS      0x32
#define INT1_DURATION 0x33
#define INT1_SRC      0x31
#define WHO_AM_I_REG	0x0F  // 版本号寄存器

// I2C 实例
#define IIC_SCL	NRF_GPIO_PIN_MAP(1, 9)//IIC_SCL	NRF_GPIO_PIN_MAP(0, 12)//IIC_SCL	NRF_GPIO_PIN_MAP(1, 9)
#define IIC_SDA	NRF_GPIO_PIN_MAP(0, 8)//IIC_SDA	NRF_GPIO_PIN_MAP(1, 9)//IIC_SDA	NRF_GPIO_PIN_MAP(0, 8)
nrf_drv_twi_t twi = NRF_DRV_TWI_INSTANCE(0);

// 初始化 I2C
void i2c_init(void) {
    nrf_drv_twi_config_t config = {
        .scl = IIC_SCL,
        .sda = IIC_SDA,
        .frequency = NRF_DRV_TWI_FREQ_100K,
        .interrupt_priority = APP_IRQ_PRIORITY_LOWEST
    };
    nrf_drv_twi_init(&twi, &config, NULL, NULL);
    nrf_drv_twi_enable(&twi);
}

// 写入 SC7A20H 寄存器
void sc7a20h_write_reg(uint8_t reg, uint8_t value) {
    uint8_t data[2] = {reg, value};
    nrf_drv_twi_tx(&twi, SC7A20H_ADDR, data, 2, false);
}


// 读取 SC7A20H 寄存器
uint8_t sc7a20h_read_reg(uint8_t reg) {
    uint8_t value;
    nrf_drv_twi_tx(&twi, SC7A20H_ADDR, &reg, 1, true);
    nrf_drv_twi_rx(&twi, SC7A20H_ADDR, &value, 1);
    return value;
}

// 读取 SC7A20H 版本号
uint8_t sc7a20h_read_version(void) {
    uint8_t reg = WHO_AM_I_REG;
    uint8_t version =0;
    // 发送寄存器地址
    ret_code_t err_code = nrf_drv_twi_tx(&twi, SC7A20H_ADDR, &reg, 1, false);
    if (err_code != NRF_SUCCESS) {
        //NRF_LOG_ERROR("TX failed: 0x%x", err_code);
        return 0xFF;
    }
    // 读取数据
    err_code = nrf_drv_twi_rx(&twi, SC7A20H_ADDR, &version, 1);
    if (err_code != NRF_SUCCESS) {
        //NRF_LOG_ERROR("RX failed: 0x%x", err_code);
        return 0xFF;
    }
    return version;
}


void sc7a20h_init_interrupt(void) 
{
		// SC7A20H 寄存器定义
		#define SC7A20H_ADDR          0x18
		#define SC7A20H_REG_CTRL1     0x20
		#define SC7A20H_REG_CTRL2     0x21
		#define SC7A20H_REG_CTRL3     0x22
		#define SC7A20H_REG_CTRL4     0x23
		#define SC7A20H_REG_CTRL5     0x24
		#define SC7A20H_REG_INT1_CFG  0x30
		#define SC7A20H_REG_INT1_THS  0x32
		#define SC7A20H_REG_INT1_DUR  0x33
		#define SC7A20H_REG_INT1_SRC  0x31
		#define SC7A20H_REG_STATUS    0x27
		sc7a20h_write_reg(SC7A20H_REG_CTRL1, 0x57);
		sc7a20h_write_reg(SC7A20H_REG_CTRL3, 0x40);
		sc7a20h_write_reg(SC7A20H_REG_CTRL4, 0x88);
		sc7a20h_write_reg(SC7A20H_REG_INT1_CFG, 0x95);
		sc7a20h_write_reg(SC7A20H_REG_INT1_THS, 0x40);//sc7a20h_write_reg(SC7A20H_REG_INT1_THS, 0x27);//sc7a20h_write_reg(SC7A20H_REG_INT1_THS, 0x17);//sc7a20h_write_reg(SC7A20H_REG_INT1_THS, 0x20);//sc7a20h_write_reg(SC7A20H_REG_INT1_THS, 0x10);
		sc7a20h_write_reg(SC7A20H_REG_INT1_DUR, 0x00);
}


volatile bool anti_lost_mode = 0;
volatile bool polic_ing = 0;
#if 0
//按键中断处理
#define BUTTON_PIN 11
void button_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action) {
    // 简单防抖：延时 50ms
    nrf_delay_ms(50);
    if (nrf_gpio_pin_read(BUTTON_PIN) == 0) {
        anti_lost_mode = !anti_lost_mode;
        if (anti_lost_mode) {
            // 进入防丢失模式：启用加速度计中断
            sc7a20h_write_reg(CTRL_REG3, 0x40);
        } else {
            // 退出防丢失模式：禁用中断
            sc7a20h_write_reg(CTRL_REG3, 0x00);
        }
    }
}
 
void gpiote_init(void) {
    nrf_drv_gpiote_in_config_t config = GPIOTE_CONFIG_IN_SENSE_HITOLO(true);
    nrf_drv_gpiote_in_init(BUTTON_PIN, &config, button_handler);
    nrf_drv_gpiote_in_event_enable(BUTTON_PIN, true);
}
#endif


//加速度计中断处理
//#define ALARM_PIN NRF_GPIO_PIN_MAP(0,12)

void in_pin_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action) {
		//清除中断标志（读取 INT1_SRC 寄存器）
		sc7a20h_read_reg(INT1_SRC);
		if(anti_lost_mode){
		//触发报警（LED/蜂鸣器）
				#if 0
				static int flag =0;
				if(flag){
					flag =0;
					beeper_start();MOTOR_start();
				}else{
					flag =1;
					beeper_stop();MOTOR_stop();
				}
				#endif
				
				sc7a20h_write_reg(CTRL_REG3, 0x00); //add_20250911
				
				polic_ing =1;
				
				//报警声音频率调节-替换
				//beeper_start();
				
				MOTOR_start();
				//IR_Send_Set(1);
				
				IR_flag_mode =4;
				hardware_timer_start(); //打开定时器
				
				//显示报警图片
//				extern lv_obj_t * img_polic;
//				if(img_polic ==NULL){
//					img_polic =lv_img_create(lv_scr_act());
//					lv_img_set_src(img_polic, &polic);
//					lv_obj_align(img_polic, LV_ALIGN_TOP_MID, 0, 0);
//				}
				
				
    }
}

#define INT_PIN NRF_GPIO_PIN_MAP(0,6) //INT_PIN NRF_GPIO_PIN_MAP(0,8)//INT_PIN NRF_GPIO_PIN_MAP(0,12)
void accel_interrupt_init(void) {
	  ret_code_t err_code;
    // 初始化GPIOTE驱动
    err_code = nrf_drv_gpiote_init();
    APP_ERROR_CHECK(err_code);

    // 配置按钮引脚为中断输入，下降沿触发
    //nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_HITOLO(true);
    //in_config.pull = NRF_GPIO_PIN_PULLUP;


    //err_code = nrf_drv_gpiote_in_init(BUTTON_PIN, &in_config, button_handler);
    //APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_in_config_t config = GPIOTE_CONFIG_IN_SENSE_HITOLO(true);
    err_code = nrf_drv_gpiote_in_init(INT_PIN, &config, in_pin_handler); // INT1 引脚
		APP_ERROR_CHECK(err_code);
    nrf_drv_gpiote_in_event_enable(INT_PIN, true);
}

void AD8314_Init(void)
{
		i2c_init();
		accel_interrupt_init();
		sc7a20h_init_interrupt();
		//sc7a20h_write_reg(CTRL_REG3, 0x40);
		sc7a20h_read_version();
}

bool AD8314_sta =0;
void AD8314_Int_enable(void)
{
		AD8314_sta =1;
		sc7a20h_init_interrupt();
		anti_lost_mode =1;
		polic_ing =0;
		sc7a20h_write_reg(CTRL_REG3, 0x40);
		//关闭屏幕
		LCD_PWM_Set(0);
}

bool AD8314_Int_disable(void)
{
		if(polic_ing || anti_lost_mode){
			beeper_start_tim(10);
			polic_ing =0;
			sc7a20h_write_reg(CTRL_REG3, 0x00);
			sc7a20h_write_reg(CTRL_REG3, 0x00);
			MOTOR_stop();
			beeper_stop();
			IR_Send_Set(0);
			anti_lost_mode =0;
			LCD_PWM_Set(1);
			
			IR_flag_mode =0;
			hardware_timer_stop(); //关闭定时器
			AD8314_sta =0;
			return 1;
		}
		return 0;
}


bool AD8314_deep_disable(void)
{
		if(polic_ing){
			//beeper_start_tim(10);
			polic_ing =0;
			anti_lost_mode =0;
			MOTOR_stop();
			beeper_stop();
			IR_Send_Set(0);
			hardware_timer_stop(); //关闭定时器
			IR_flag_mode =0;
			sc7a20h_write_reg(CTRL_REG3, 0x00);
			sc7a20h_write_reg(CTRL_REG3, 0x00);
			nrf_delay_ms(1000);
			return 1;
		}
		return 0;
}



//磁力传感器 QMC5883P
#define QMC5883P_ADDR   0x2C  // I2C 地址（SA0=0）
#define QMC5883P_REG_CHIP_ID	0x00 //0x0 // 芯片ID寄存器
// QMC5883P 寄存器定义
#define QMC5883P_REG_DATA_OUT_X_LSB   0x01
#define QMC5883P_REG_DATA_OUT_X_MSB   0x02
#define QMC5883P_REG_DATA_OUT_Y_LSB   0x03
#define QMC5883P_REG_DATA_OUT_Y_MSB   0x04
#define QMC5883P_REG_DATA_OUT_Z_LSB   0x05
#define QMC5883P_REG_DATA_OUT_Z_MSB   0x06
#define QMC5883P_REG_STATUS           0x06
//#define QMC5883P_REG_TEMP_LSB         0x07
//#define QMC5883P_REG_TEMP_MSB         0x08
//#define QMC5883P_REG_CONTROL_1        0x09
//#define QMC5883P_REG_CONTROL_2        0x0A
//#define QMC5883P_REG_PERIOD           0x0B
//#define QMC5883P_REG_CHIP_ID          0x0D // 芯片ID寄存器

// 控制寄存器1配置
#define QMC5883P_MODE_STANDBY         0x00


#define QMC5883P_MODE_CONTINUOUS      0x03
#define QMC5883P_ODR_10HZ             0x00
#define QMC5883P_ODR_50HZ             0x04
#define QMC5883P_ODR_100HZ            0x08
#define QMC5883P_ODR_200HZ            0x0C
#define QMC5883P_RNG_2G               0x00
#define QMC5883P_RNG_8G               0x10
#define QMC5883P_OSR_512              0x00
#define QMC5883P_OSR_256              0x40
#define QMC5883P_OSR_128              0x80
#define QMC5883P_OSR_64               0xC0

// I2C 实例
#define QMC_IIC_SCL		NRF_GPIO_PIN_MAP(0,11)
#define QMC_IIC_SDA		NRF_GPIO_PIN_MAP(0,12)
//全局TWI实例
nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(1);
//初始化 I2C
void QMC_i2c_init(void) {
    nrf_drv_twi_config_t config = {
        .scl = QMC_IIC_SCL, 
        .sda = QMC_IIC_SDA,
        .frequency = NRF_DRV_TWI_FREQ_100K,
        .interrupt_priority = APP_IRQ_PRIORITY_LOWEST,
				//.hold_bus_uninit =1
    };
    nrf_drv_twi_init(&m_twi, &config, NULL, NULL);
    nrf_drv_twi_enable(&m_twi);
}

//向QMC5883P的寄存器写入一个字节
ret_code_t qmc5883p_write_byte(uint8_t reg, uint8_t data) {
    ret_code_t err_code;
    uint8_t buffer[2] = {reg, data};
    err_code = nrf_drv_twi_tx(&m_twi, QMC5883P_ADDR, buffer, 2, false);
    return err_code;
}

//从QMC5883P的寄存器读取一个字节
ret_code_t qmc5883p_read_byte(uint8_t reg, uint8_t *data) {
    ret_code_t err_code;
    //先写入要读取的寄存器地址
    err_code = nrf_drv_twi_tx(&m_twi, QMC5883P_ADDR, &reg, 1, false);
    if (err_code != NRF_SUCCESS) {
        return err_code;
    }
    //然后读取该寄存器的值
    err_code = nrf_drv_twi_rx(&m_twi, QMC5883P_ADDR, data, 1);
    return err_code;
}


// 从QMC5883P读取多个字节 (用于读取传感器数据)
ret_code_t qmc5883p_read_bytes(uint8_t start_reg, uint8_t *data, uint32_t length) {
    ret_code_t err_code;
    // 先写入要起始寄存器地址
    err_code = nrf_drv_twi_tx(&m_twi, QMC5883P_ADDR, &start_reg, 1, true);
    if (err_code != NRF_SUCCESS) {
        return err_code;
    }
    // 然后连续读取多个字节
    err_code = nrf_drv_twi_rx(&m_twi, QMC5883P_ADDR, data, length);
    return err_code;
}

// 读取并验证QMC5883P的芯片ID (版本信息)
void qmc5883p_get_chip_id(uint8_t *chip_id_t) {
    qmc5883p_read_byte(QMC5883P_REG_CHIP_ID, chip_id_t);
}

//初始化QMC5883P传感器
int qmc5883p_init(void) {

//		nrf_gpio_cfg_output(QMC_IIC_SCL);
//		nrf_gpio_cfg_output(QMC_IIC_SDA);
//		nrf_gpio_pin_write(QMC_IIC_SCL, 0);
//		nrf_gpio_pin_write(QMC_IIC_SDA, 1);
    ret_code_t err_code;
		
		QMC_i2c_init();
		
		#if 1
    //软件复位 (设置控制寄存器2的SOFT_RST位)
    err_code = qmc5883p_write_byte(0x0b, 0x00); //0x04);//0x0C);
    if (err_code != NRF_SUCCESS) {
        return err_code;
    }
    nrf_delay_ms(10); // 等待复位完成
    // 配置周期寄存器 (建议值0x01)
//    err_code = qmc5883p_write_byte(QMC5883P_REG_PERIOD, 0x01);
//    if (err_code != NRF_SUCCESS) {
//        return err_code;
//    }


    //配置控制寄存器1 (连续测量模式, 10Hz, 2G量程, OSR=256)
    uint8_t ctrl1_config = QMC5883P_MODE_CONTINUOUS | QMC5883P_ODR_10HZ;
    err_code = qmc5883p_write_byte(0x0a, ctrl1_config);
		
    return err_code;
		
		#endif

}
uint8_t qmc5883p_data[6];
// 读取磁力计原始数据 (X, Y, Z)
ret_code_t qmc5883p_read_mag_data(int16_t *mag_x, int16_t *mag_y, int16_t *mag_z){
    ret_code_t err_code;
    memset(qmc5883p_data, 0, sizeof(qmc5883p_data));
    // 读取6个数据寄存器 (XLSB, XMSB, YLSB, YMSB, ZLSB, ZMSB)
    err_code = qmc5883p_read_bytes(QMC5883P_REG_DATA_OUT_X_LSB, qmc5883p_data, sizeof(qmc5883p_data));
    if (err_code != NRF_SUCCESS){
        return err_code;
    }
    // 组合数据 (注意: QMC5883P通常是LSB在前，MSB在后)
    *mag_x = (int16_t)((qmc5883p_data[1] << 8) | qmc5883p_data[0]);
    *mag_y = (int16_t)((qmc5883p_data[3] << 8) | qmc5883p_data[2]);
    *mag_z = (int16_t)((qmc5883p_data[5] << 8) | qmc5883p_data[4]);
    return NRF_SUCCESS;
}
uint8_t chip_id;
int16_t mag_x, mag_y, mag_z;
void qmc5883p_fun(void)
{
	ret_code_t err_code;
	// 初始化QMC5883P
	qmc5883p_init();
	// 读取并检查芯片ID
	qmc5883p_get_chip_id(&chip_id);
	//nrf_delay_ms(1000); // 每秒读取一次
	#if 0
	while(1) {
			uint8_t data =0;
			qmc5883p_read_byte(9, &data);
			if(data & 0x01){
				
			
	
				// 读取磁力数据
				err_code = qmc5883p_read_mag_data(&mag_x, &mag_y, &mag_z);
				if (err_code == NRF_SUCCESS) {
						;//NRF_LOG_INFO("Mag X: %d, Y: %d, Z: %d", mag_x, mag_y, mag_z);
				} else {
						;//NRF_LOG_ERROR("Failed to read data: 0x%X", err_code);
				}
			
			}
			nrf_delay_ms(1000); // 每秒读取一次
	}
	#endif
	return;
}



















































