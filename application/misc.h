#ifndef __MISC_H
#define __MISC_H

#include <stdint.h>
#include "nrf_drv_spi.h"
#include "app_util_platform.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "boards.h"
#include "app_error.h"
#include <string.h>
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrfx_pwm.h"
#include "nrfx_saadc.h"


extern uint32_t Flash_Data;    //bit 0: 声音    bit 1: 震动    
extern nrf_saadc_value_t ADC_IR;
extern nrf_saadc_value_t ADC_Hall;
extern nrf_saadc_value_t ADC_Batty;
extern nrf_saadc_value_t ADC_Wless;
extern int Device_POW_flag;
extern bool AD8314_sta;

/* 震动马达 */
#define MOTOR						NRF_GPIO_PIN_MAP(1,13)
void MOTOR_Init(void);
void MOTOR_Set(int pow);
void MOTOR_start_tim(int t);
void MOTOR_start(void);
void MOTOR_stop(void);

/* 红外发射 */
#define IR_2          NRF_GPIO_PIN_MAP(0,24)
#define IR_3          NRF_GPIO_PIN_MAP(0,15)
void IR_Send_Init(void);
void IR_Send_Set(int pow);
void hardware_timer_init(int TIMER_MS);
void hardware_timer_start(void);
void hardware_timer_stop(void);
extern int IR_flag_mode;


/* 蜂鸣器 */
#define BUZZER          NRF_GPIO_PIN_MAP(1,15)
#define PWM_INST_IDX    NRFX_PWM_INSTANCE(3)  // 使用PWM实例0
//PWM配置参数计算
#define PWM_CLK_FREQ    NRF_PWM_CLK_16MHz     // 16MHz时钟
#define PWM_FREQ        4000 //2700                  // 2.7kHz目标频率
#define PWM_TOP_VALUE   (16000000 / PWM_FREQ) // TOP值 = 16M/2700 ≈ 5926
#define DUTY_CYCLE      50                    // 50%占空比
#define PWM_PULSE_WIDTH ((PWM_TOP_VALUE * DUTY_CYCLE) / 100) // 脉冲宽度计算
void pwm_init(void);
void beeper_start(void);
void beeper_stop(void);
void beeper_start_tim(int t);
/* 充电芯片 */
#define MCU_CHRG_				NRF_GPIO_PIN_MAP(0,10)
#define MCU_STDBY_			NRF_GPIO_PIN_MAP(0,9)
#define MCU_USB_STA_		NRF_GPIO_PIN_MAP(1,10)
void batt_sta_init(void);
unsigned int batt_sta_update(void);


/* 按键 */
#define BUTTON_DOWN	NRF_GPIO_PIN_MAP(1,11)			//上
#define BUTTON_POW	NRF_GPIO_PIN_MAP(1,12)			//中
#define	BUTTON_UP   NRF_GPIO_PIN_MAP(1,6)      	//下
#define MCU_POW   	NRF_GPIO_PIN_MAP(0,26)
void Key_Init(void);
unsigned int key_update(void);


/* 电源 */
#define BAT_POW		NRF_GPIO_PIN_MAP(0,26)
#define KEY_ON_OFF		NRF_GPIO_PIN_MAP(0,13)//NRF_GPIO_PIN_MAP(0,04)
void BAT_POW_Init(void);
void BAT_POW_Set(int pow);


/* LCD屏 */
#define LCD_PWR_EN					NRF_GPIO_PIN_MAP(0,1)//NRF_GPIO_PIN_MAP(0,01)//NRF_GPIO_PIN_MAP(0,31)
#define BACKLIGHT_PWM				NRF_GPIO_PIN_MAP(0,4)//NRF_GPIO_PIN_MAP(0,06)//NRF_GPIO_PIN_MAP(0,29)
void LCD_Init(void);
void LCD_PWM_Set(int pow);


//NET
#define RF_POWER_EN_PIN		NRF_GPIO_PIN_MAP(0,0)

void Misc_Init(void);

void Net_Init(void);
void Net_Init_Pow_Set(int pow);


//内部Flash
void storage_init(void);
void save_data_to_flash(uint32_t data);
bool load_data_from_flash(uint32_t *p_data);






//霍尔 1.65V - 3.3V		1877-3755
/*  
	3.2 - 3.3	3641 - 3755
	3.1 - 3.2	3527 - 3641
	3.0 - 3.1	3413 - 3527
	2.9 - 3.0	3300 - 3413
	2.8 - 2.9	3186 - 3300
	2.7 - 2.8	3072 - 3186
	2.6 - 2.7	2958 - 3072
	2.5 - 2.6	2844 - 2958
	2.4 - 2.5	2731 - 2844
	2.3 - 2.4	2617 - 2731
	2.2 - 2.3	2503 - 2617
	2.1 - 2.2	2389 - 2503
	2.0 - 2.1	2276 - 2389
	1.9 - 2.0	2162 - 2276
	1.8 - 1.9	2048 - 2162
	1.7 - 1.8	1934 - 2048
*/



/* 0V - 1.65V				0-1877
	1.5 - 1.6	1707 - 1820
	1.4 - 1.5	1593 - 1707
	1.3 - 1.4	1479 - 1593
	1.2 - 1.3	1365 - 1479
	1.1 - 1.2	1252 - 1365
	1 - 1.1		1138 - 1252
	0.9 - 1		1024 - 1138
	0.8 - 0.9	910 - 1024
	0.7 - 0.8	796 - 910
	0.6 - 0.7	682 - 796
	0.5 - 0.6	569 - 682
	0.4 - 0.5	455 - 569
	0.3 - 0.4	341 - 455
	0.2 - 0.3	228 - 341
	0.1 - 0.2	114 - 228
	0 - 0.1		0 - 114
*/

int hall_data_deal(int value);



//红外 0V-3.3v		0-3755
/*
	3.1 - 3.3	3572 - 3755
	2.9 - 3.1	3300 - 3572
	2.7 - 2.9	3072 - 3300
	2.5 - 2.7	2844 - 3072
	2.3 - 2.5	2617 - 2844
	2.1 - 2.3	2389 - 2617
	1.9 - 2.1	2162 - 2389
	1.7 - 1.9	1934 - 2162
	1.5 - 1.7	1707 - 1934
	1.3 - 1.5	1479 - 1707
	1.1 - 1.3	1252 - 1479
	0.9 - 1.1	1024 - 1252
	0.7 - 0.9	796 - 1024
	0.5 - 0.7	569 - 796
	0.3 - 0.5	341 - 569
	0.1 - 0.3	114 - 341
*/
int ir_data_deal(int value);



//wifi 0V-1.2V   0-1365
/*
	1.1 - 1.2	1251 - 1365
	1.0 - 1.1	1138 - 1251
	0.9 - 1.0	1024 - 1138
	0.8 - 0.9	910 - 1024
	0.7 - 0.8	796 - 910
	0.6 - 0.7	683 - 796
	0.5 - 0.6	569 - 683
	0.4 - 0.5	455 - 569
	0.3 - 0.4	341 - 455
	0.2 - 0.3	228 - 341
	0.1 - 0.2	113 - 228
	0 - 0.1		50 - 113
*/
int wifi_data_deal(int value);




void write_flash(uint32_t data);
void read_flash(uint32_t *data);

//加速度计
extern volatile bool anti_lost_mode;
extern volatile bool polic_ing;
void AD8314_Init(void);
void AD8314_Int_enable(void);
bool AD8314_Int_disable(void);
bool AD8314_deep_disable(void);


#endif

