#ifndef __APP_UI_H
#define __APP_UI_H

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
#include "app_power.h"
#include "app_lcd.h"
#include "pic.h"
#include "lv_port_disp.h"

extern int sensitivity_ir;		//红外灵敏度
extern int sensitivity_hall;  //强磁灵敏度
extern int sensitivity_wless;	//无线灵敏度
extern int sensitivity_tag;		//tag灵敏度

extern int curr_pos_set;
void Batt_Set_UI_2(int ing);
void Batt_Set_UI_1(void);

extern volatile int Police_Flag;
extern volatile int Police_Flag_Again;
extern int up_flag;
extern int layer;  //当前层级
extern int layer1;  //层级1
extern int layer2;	//层级2
extern int sound_flag;
extern int move_flag;

void Lvgl_Init(void);
int layer_ui_deal(int key);
void Gui_Pro(void);
void ffun(void);

void mon2_img_pro(int data, lv_obj_t *parent);
void Batt_Set_UI(int ing, int pow);






#endif

