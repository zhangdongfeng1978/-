#include "app_ui.h"
#include "misc.h"
#include "bsp_adc.h"
#include "app_dis_scan.h"
#include "pic.h"


#define SET_ID_POW  0

extern int beep_cout;


extern int scan_mode;
volatile int Police_Flag =0;
volatile int Police_Flag_Again =0;
extern uint64_t device_id_only;
static lv_obj_t * img_main1 =NULL; // = lv_img_create(lv_scr_act());
static lv_obj_t * img_main2 =NULL;
static lv_obj_t * img_main3 =NULL;
static lv_obj_t * img_main4 =NULL;
static lv_obj_t * img_main5 =NULL;
static lv_obj_t * img_main6 =NULL;
static lv_obj_t * img_main7 =NULL;
static lv_obj_t * img_main8 =NULL;

static lv_obj_t * img_main1_1 =NULL;
static lv_obj_t * img_main2_1 =NULL;
static lv_obj_t * img_main3_1 =NULL;
static lv_obj_t * img_main4_1 =NULL;
static lv_obj_t * img_main5_1 =NULL;
static lv_obj_t * img_main6_1 =NULL;
static lv_obj_t * img_main6_t =NULL;
static lv_obj_t * img_main6_t_1 =NULL;
static lv_obj_t * img_main6_t_2 =NULL;
static lv_obj_t * img_main6_1_1 =NULL;
static lv_obj_t * img_main6_1_1_1 =NULL;
static lv_obj_t * img_main6_1_2 =NULL;
static lv_obj_t * img_main6_1_2_1 =NULL;
static lv_obj_t * img_main6_1_3 =NULL;

static lv_obj_t * img_main7_1 =NULL;   static  lv_obj_t *black_rect =NULL;
static lv_obj_t * img_main8_1 =NULL;

static lv_obj_t * img_ir_1 =NULL;
static lv_obj_t * img_ir_2 =NULL;
static lv_obj_t * img_ir_3 =NULL;
static lv_obj_t * img_ir_4 =NULL;

static lv_obj_t * img_mon_1 =NULL;
static lv_obj_t * img_mon_2 =NULL;
static lv_obj_t * img_mon_3 =NULL;
static lv_obj_t * img_mon_4 =NULL;
static lv_obj_t * img_mon_5 =NULL;
static lv_obj_t * img_mon_6 =NULL;
static lv_obj_t * img_mon_7 =NULL;
static lv_obj_t * img_mon_8 =NULL;
static lv_obj_t * img_mon_9 =NULL;
static lv_obj_t * img_mon_10 =NULL;
static lv_obj_t * img_mon_11 =NULL;
static lv_obj_t * img_mon_12 =NULL;
static lv_obj_t * img_mon_13 =NULL;
static lv_obj_t * img_mon_14 =NULL;
static lv_obj_t * img_mon_15 =NULL;
static lv_obj_t * img_mon_16 =NULL;


static lv_obj_t * img_mon2_1 =NULL;
static lv_obj_t * img_mon2_2 =NULL;
static lv_obj_t * img_mon2_3 =NULL;
static lv_obj_t * img_mon2_4 =NULL;
static lv_obj_t * img_mon2_5 =NULL;
static lv_obj_t * img_mon2_6 =NULL;
static lv_obj_t * img_mon2_7 =NULL;
static lv_obj_t * img_mon2_8 =NULL;
static lv_obj_t * img_mon2_9 =NULL;
static lv_obj_t * img_mon2_10 =NULL;
static lv_obj_t * img_mon2_11 =NULL;
static lv_obj_t * img_mon2_12 =NULL;
static lv_obj_t * img_mon2_13 =NULL;
static lv_obj_t * img_mon2_14 =NULL;



static lv_obj_t * img_shan_red =NULL;
static lv_obj_t * img_shan_blu =NULL;
static lv_obj_t * img_dian1 =NULL;
static lv_obj_t * img_dian2 =NULL;
static lv_obj_t * img_dian3 =NULL;
static lv_obj_t * img_dian4 =NULL;
static lv_obj_t * img_dian5 =NULL;


int sensitivity_ir =1;		//红外灵敏度
int sensitivity_hall =1;  //强磁灵敏度
int sensitivity_wless =1;	//无线灵敏度
int sensitivity_tag =1;		//tag灵敏度

int curr_pos_set =0;

lv_obj_t * img_main =NULL;
void Lvgl_Init(void)
{
	//
	if(img_main ==NULL){
		img_main = lv_img_create(lv_scr_act());
		lv_obj_set_size(img_main, lv_disp_get_hor_res(NULL), lv_disp_get_ver_res(NULL));
		lv_img_set_src(img_main, &main_ui);
		lv_obj_center(img_main);
	}
	//Batt_Set_UI(1, 5);
	layer  =1;  //当前层级
	layer1 =1;  //层级1
	layer2 =0;	//层级2
}



void Batt_Set_UI_2(int ing)
{
		if(layer ==1){
			if(ing ==1){
				if(img_shan_blu && lv_obj_is_valid(img_shan_blu)){
					lv_obj_del(img_shan_blu); 
					img_shan_blu = NULL;
				}
				if(img_shan_red ==NULL){
					img_shan_red = lv_img_create(img_main);
					lv_img_set_src(img_shan_red, &shan_red);
					lv_obj_align(img_shan_red, LV_ALIGN_TOP_MID, 35, 3);
				}
			}else if(ing ==2){
				if(img_shan_red && lv_obj_is_valid(img_shan_red)){
					lv_obj_del(img_shan_red); 
					img_shan_red = NULL;
				}
				if(img_shan_blu ==NULL){
					img_shan_blu = lv_img_create(img_main);
					lv_img_set_src(img_shan_blu, &shan_blu);
					lv_obj_align(img_shan_blu, LV_ALIGN_TOP_MID, 35, 3);
				}
			}else{
				if(img_shan_red && lv_obj_is_valid(img_shan_red)){
					lv_obj_del(img_shan_red); 
					img_shan_red = NULL;
				}
				if(img_shan_blu && lv_obj_is_valid(img_shan_blu)){
					lv_obj_del(img_shan_blu); 
					img_shan_blu = NULL;
				}
			}
		}
}

extern uint8_t bettry_flag;
void Batt_Set_UI_1(void)
{
if(layer ==1){
		static int curr_pow1_num =0;
		static int curr_pow2_num =0;
		static int curr_pow3_num =0;
		static int curr_pow4_num =0;
		static int curr_pow5_num =0;
		static int curr_pow0_num =0;
		static int pow =0;
		if(ADC_Batty>=2901 && ADC_Batty<3038){
			curr_pow2_num =0;
			curr_pow3_num =0;
			curr_pow4_num =0;
			curr_pow5_num =0;
			curr_pow0_num =0;
			if(curr_pow1_num ++ >10){
				curr_pow1_num =0;
				bettry_flag =0;
				pow =1;
			}
		}else if(ADC_Batty>=3038 && ADC_Batty<3174){
			curr_pow1_num =0;
			curr_pow3_num =0;
			curr_pow4_num =0;
			curr_pow5_num =0;
			curr_pow0_num =0;
			if(curr_pow2_num ++ >10){
				curr_pow2_num =0;
				bettry_flag =0;
				pow =2;
			}
			
		}else if(ADC_Batty>=3174 && ADC_Batty<3310){
			curr_pow1_num =0;
			curr_pow2_num =0;
			curr_pow4_num =0;
			curr_pow5_num =0;
			curr_pow0_num =0;
			if(curr_pow3_num ++ >10){
				curr_pow3_num =0;
				bettry_flag =0;
				pow =3;
			}
		}else if(ADC_Batty>=3310 && ADC_Batty<3550){//}else if(ADC_Batty>=3310 && ADC_Batty<3500){//}else if(ADC_Batty>=3310 && ADC_Batty<3447){
			curr_pow1_num =0;
			curr_pow2_num =0;
			curr_pow3_num =0;
			curr_pow5_num =0;
			curr_pow0_num =0;
			if(curr_pow4_num ++ >10){
				curr_pow4_num =0;
				bettry_flag =0;
				pow =4;
			}
		}else if(ADC_Batty>=3550){//}else if(ADC_Batty>=3500){//}else if(ADC_Batty>=3447){
			curr_pow1_num =0;
			curr_pow2_num =0;
			curr_pow3_num =0;
			curr_pow4_num =0;
			curr_pow0_num =0;
			if(curr_pow5_num ++ >10){
				curr_pow5_num =0;
				bettry_flag =1;
				pow =5;
			}
		}else if(ADC_Batty<=2901){
			curr_pow1_num =0;
			curr_pow2_num =0;
			curr_pow3_num =0;
			curr_pow4_num =0;
			curr_pow5_num =0;
			//if(curr_pow0_num ++ >3){
			if(curr_pow0_num ++ >20){
				curr_pow0_num =0;
				pow =0;
				//电量过低 - 自动关机   //低于电池3.4V -- 分压3.4/4*3  = 2.55  ==> 2901
				Device_POW_flag =0;
				MOTOR_start_tim(300);
				sound_flag =0;
				move_flag =0;
				LCD_PWM_Set(0);
				BAT_POW_Set(0);
			}
		}
		switch(pow){
			case 0: 
			{
				if(img_dian1 && lv_obj_is_valid(img_dian1)){lv_obj_del(img_dian1); img_dian1 = NULL;}
				if(img_dian2 && lv_obj_is_valid(img_dian2)){lv_obj_del(img_dian2); img_dian2 = NULL;}
				if(img_dian3 && lv_obj_is_valid(img_dian3)){lv_obj_del(img_dian3); img_dian3 = NULL;}
				if(img_dian4 && lv_obj_is_valid(img_dian4)){lv_obj_del(img_dian4); img_dian4 = NULL;}
				if(img_dian5 && lv_obj_is_valid(img_dian5)){lv_obj_del(img_dian5); img_dian5 = NULL;}
				break;
			}
			case 1:  
			{
				if(img_dian1 && lv_obj_is_valid(img_dian1)){lv_obj_del(img_dian1); img_dian1 = NULL;}
				if(img_dian2 && lv_obj_is_valid(img_dian2)){lv_obj_del(img_dian2); img_dian2 = NULL;}
				if(img_dian3 && lv_obj_is_valid(img_dian3)){lv_obj_del(img_dian3); img_dian3 = NULL;}
				if(img_dian4 && lv_obj_is_valid(img_dian4)){lv_obj_del(img_dian4); img_dian4 = NULL;}
				if(img_dian5 == NULL){
					img_dian5 = lv_img_create(img_main);
					lv_img_set_src(img_dian5, &dian);
					lv_obj_align(img_dian5, LV_ALIGN_TOP_MID, 65, 5);
				}
				break;
			}
			case 2:  
			{
				if(img_dian1 && lv_obj_is_valid(img_dian1)){lv_obj_del(img_dian1); img_dian1 = NULL;}
				if(img_dian2 && lv_obj_is_valid(img_dian2)){lv_obj_del(img_dian2); img_dian2 = NULL;}
				if(img_dian3 && lv_obj_is_valid(img_dian3)){lv_obj_del(img_dian3); img_dian3 = NULL;}
				if(img_dian4 == NULL){
					img_dian4 = lv_img_create(img_main);
					lv_img_set_src(img_dian4, &dian);
					lv_obj_align(img_dian4, LV_ALIGN_TOP_MID, 61, 5);
				}
				if(img_dian5 == NULL){
					img_dian5 = lv_img_create(img_main);
					lv_img_set_src(img_dian5, &dian);
					lv_obj_align(img_dian5, LV_ALIGN_TOP_MID, 65, 5);
				}
				break;
			}
			case 3:  
			{
				if(img_dian1 && lv_obj_is_valid(img_dian1)){lv_obj_del(img_dian1); img_dian1 = NULL;}
				if(img_dian2 && lv_obj_is_valid(img_dian2)){lv_obj_del(img_dian2); img_dian2 = NULL;}
				if(img_dian3 == NULL){
					img_dian3 = lv_img_create(img_main);
					lv_img_set_src(img_dian3, &dian);
					lv_obj_align(img_dian3, LV_ALIGN_TOP_MID, 57, 5);
				}
				if(img_dian4 == NULL){
					img_dian4 = lv_img_create(img_main);
					lv_img_set_src(img_dian4, &dian);
					lv_obj_align(img_dian4, LV_ALIGN_TOP_MID, 61, 5);
				}
				if(img_dian5 == NULL){
					img_dian5 = lv_img_create(img_main);
					lv_img_set_src(img_dian5, &dian);
					lv_obj_align(img_dian5, LV_ALIGN_TOP_MID, 65, 5);
				}
				break;
			}
			case 4:  
			{
				if(img_dian1 && lv_obj_is_valid(img_dian1)){lv_obj_del(img_dian1); img_dian1 = NULL;}
				if(img_dian2 == NULL){
					img_dian2 = lv_img_create(img_main);
					lv_img_set_src(img_dian2, &dian);
					lv_obj_align(img_dian2, LV_ALIGN_TOP_MID, 53, 5);
				}
				if(img_dian3 == NULL){
					img_dian3 = lv_img_create(img_main);
					lv_img_set_src(img_dian3, &dian);
					lv_obj_align(img_dian3, LV_ALIGN_TOP_MID, 57, 5);
				}
				if(img_dian4 == NULL){
					img_dian4 = lv_img_create(img_main);
					lv_img_set_src(img_dian4, &dian);
					lv_obj_align(img_dian4, LV_ALIGN_TOP_MID, 61, 5);
				}
				if(img_dian5 == NULL){
					img_dian5 = lv_img_create(img_main);
					lv_img_set_src(img_dian5, &dian);
					lv_obj_align(img_dian5, LV_ALIGN_TOP_MID, 65, 5);
				}
				break;
			}
			case 5:  
			{
				if(img_dian1 == NULL){
					img_dian1 = lv_img_create(img_main);
					lv_img_set_src(img_dian1, &dian);
					lv_obj_align(img_dian1, LV_ALIGN_TOP_MID, 49, 5);
				}
				if(img_dian2 == NULL){
					img_dian2 = lv_img_create(img_main);
					lv_img_set_src(img_dian2, &dian);
					lv_obj_align(img_dian2, LV_ALIGN_TOP_MID, 53, 5);
				}
				if(img_dian3 == NULL){
					img_dian3 = lv_img_create(img_main);
					lv_img_set_src(img_dian3, &dian);
					lv_obj_align(img_dian3, LV_ALIGN_TOP_MID, 57, 5);
				}
				if(img_dian4 == NULL){
					img_dian4 = lv_img_create(img_main);
					lv_img_set_src(img_dian4, &dian);
					lv_obj_align(img_dian4, LV_ALIGN_TOP_MID, 61, 5);
				}
				if(img_dian5 == NULL){
					img_dian5 = lv_img_create(img_main);
					lv_img_set_src(img_dian5, &dian);
					lv_obj_align(img_dian5, LV_ALIGN_TOP_MID, 65, 5);
				}
				break;
			}
			default:break;
		}
	}
}

lv_obj_t * label  = NULL;
void ffun(void)
{
		LV_FONT_DECLARE(lv_chinese) /* ???????? */
		label = lv_label_create(lv_scr_act());
		//lv_obj_set_style_text_color(label, lv_color_hex(0x70DBFF), 0);
		lv_obj_set_style_text_color(label, lv_color_hex(0), 0);
		extern const lv_font_t chinese;
		lv_obj_set_style_text_font(label, &chinese, 0); 
		lv_label_set_text(label, "王 ");
}


void lv_del(void)
{
	if(img_main8 && lv_obj_is_valid(img_main8)) {
		lv_obj_del(img_main8);
		img_main8 = NULL; 
	}
	if (img_main7 && lv_obj_is_valid(img_main7)) {
		lv_obj_del(img_main7);
		img_main7 = NULL;
	}
	if (img_main6 && lv_obj_is_valid(img_main6)) {
		lv_obj_del(img_main6); 
		img_main6 = NULL; 
	}
	if (img_main5 && lv_obj_is_valid(img_main5)) {
		lv_obj_del(img_main5);
		img_main5 = NULL;
	}
	if (img_main4 && lv_obj_is_valid(img_main4)) {
		lv_obj_del(img_main4);
		img_main4 = NULL;
	}
	if (img_main3 && lv_obj_is_valid(img_main3)) {
		lv_obj_del(img_main3);
		img_main3 = NULL; 
	}
	if (img_main2 && lv_obj_is_valid(img_main2)) {
		lv_obj_del(img_main2);
		img_main2 = NULL;
	}
	if (img_main1 && lv_obj_is_valid(img_main1)) {
		lv_obj_del(img_main1);
		img_main1 = NULL;
	}
	if(img_ir_1 && lv_obj_is_valid(img_ir_1)){lv_obj_del(img_ir_1); img_ir_1 = NULL;}
	if(img_ir_2 && lv_obj_is_valid(img_ir_2)){lv_obj_del(img_ir_2); img_ir_2 = NULL;}
	if(img_ir_3 && lv_obj_is_valid(img_ir_3)){lv_obj_del(img_ir_3); img_ir_3 = NULL;}
	if(img_ir_4 && lv_obj_is_valid(img_ir_4)){lv_obj_del(img_ir_4); img_ir_4 = NULL;}

	if(img_mon_1 && lv_obj_is_valid(img_mon_1)){lv_obj_del(img_mon_1); img_mon_1 = NULL;}
	if(img_mon_2 && lv_obj_is_valid(img_mon_2)){lv_obj_del(img_mon_2); img_mon_2 = NULL;}
	if(img_mon_3 && lv_obj_is_valid(img_mon_3)){lv_obj_del(img_mon_3); img_mon_3 = NULL;}
	if(img_mon_4 && lv_obj_is_valid(img_mon_4)){lv_obj_del(img_mon_4); img_mon_4 = NULL;}
	if(img_mon_5 && lv_obj_is_valid(img_mon_5)){lv_obj_del(img_mon_5); img_mon_5 = NULL;}
	if(img_mon_6 && lv_obj_is_valid(img_mon_6)){lv_obj_del(img_mon_6); img_mon_6 = NULL;}
	if(img_mon_7 && lv_obj_is_valid(img_mon_7)){lv_obj_del(img_mon_7); img_mon_7 = NULL;}
	if(img_mon_8 && lv_obj_is_valid(img_mon_8)){lv_obj_del(img_mon_8); img_mon_8 = NULL;}
	if(img_mon_9 && lv_obj_is_valid(img_mon_9)){lv_obj_del(img_mon_9); img_mon_9 = NULL;}
	if(img_mon_10 && lv_obj_is_valid(img_mon_10)){lv_obj_del(img_mon_10); img_mon_10 = NULL;}
	if(img_mon_11 && lv_obj_is_valid(img_mon_11)){lv_obj_del(img_mon_11); img_mon_11 = NULL;}
	if(img_mon_12 && lv_obj_is_valid(img_mon_12)){lv_obj_del(img_mon_12); img_mon_12 = NULL;}
	if(img_mon_13 && lv_obj_is_valid(img_mon_13)){lv_obj_del(img_mon_13); img_mon_13 = NULL;}
	if(img_mon_14 && lv_obj_is_valid(img_mon_14)){lv_obj_del(img_mon_14); img_mon_14 = NULL;}
	if(img_mon_15 && lv_obj_is_valid(img_mon_15)){lv_obj_del(img_mon_15); img_mon_15 = NULL;}
	if(img_mon_16 && lv_obj_is_valid(img_mon_16)){lv_obj_del(img_mon_16); img_mon_16 = NULL;}
}


void mon2_img_pro(int data, lv_obj_t *parent)
{
	switch(data){
		case 0:
		{
			if(img_mon2_1 && lv_obj_is_valid(img_mon2_1)){lv_obj_del(img_mon2_1); img_mon2_1 = NULL;}
			if(img_mon2_2 && lv_obj_is_valid(img_mon2_2)){lv_obj_del(img_mon2_2); img_mon2_2 = NULL;}
			if(img_mon2_3 && lv_obj_is_valid(img_mon2_3)){lv_obj_del(img_mon2_3); img_mon2_3 = NULL;}
			if(img_mon2_4 && lv_obj_is_valid(img_mon2_4)){lv_obj_del(img_mon2_4); img_mon2_4 = NULL;}
			if(img_mon2_5 && lv_obj_is_valid(img_mon2_5)){lv_obj_del(img_mon2_5); img_mon2_5 = NULL;}
			if(img_mon2_6 && lv_obj_is_valid(img_mon2_6)){lv_obj_del(img_mon2_6); img_mon2_6 = NULL;}
			if(img_mon2_7 && lv_obj_is_valid(img_mon2_7)){lv_obj_del(img_mon2_7); img_mon2_7 = NULL;}
			if(img_mon2_8 && lv_obj_is_valid(img_mon2_8)){lv_obj_del(img_mon2_8); img_mon2_8 = NULL;}
			if(img_mon2_9 && lv_obj_is_valid(img_mon2_9)){lv_obj_del(img_mon2_9); img_mon2_9 = NULL;}
			if(img_mon2_10 && lv_obj_is_valid(img_mon2_10)){lv_obj_del(img_mon2_10); img_mon2_10 = NULL;}
			if(img_mon2_11 && lv_obj_is_valid(img_mon2_11)){lv_obj_del(img_mon2_11); img_mon2_11 = NULL;}
			if(img_mon2_12 && lv_obj_is_valid(img_mon2_12)){lv_obj_del(img_mon2_12); img_mon2_12 = NULL;}
			if(img_mon2_13 && lv_obj_is_valid(img_mon2_13)){lv_obj_del(img_mon2_13); img_mon2_13 = NULL;}
			if(img_mon2_14 && lv_obj_is_valid(img_mon2_14)){lv_obj_del(img_mon2_14); img_mon2_14 = NULL;}
			break;
		}
		case 1:
		{
			if(img_mon2_2 && lv_obj_is_valid(img_mon2_2)){lv_obj_del(img_mon2_2); img_mon2_2 = NULL;}
			if(img_mon2_3 && lv_obj_is_valid(img_mon2_3)){lv_obj_del(img_mon2_3); img_mon2_3 = NULL;}
			if(img_mon2_4 && lv_obj_is_valid(img_mon2_4)){lv_obj_del(img_mon2_4); img_mon2_4 = NULL;}
			if(img_mon2_5 && lv_obj_is_valid(img_mon2_5)){lv_obj_del(img_mon2_5); img_mon2_5 = NULL;}
			if(img_mon2_6 && lv_obj_is_valid(img_mon2_6)){lv_obj_del(img_mon2_6); img_mon2_6 = NULL;}
			if(img_mon2_7 && lv_obj_is_valid(img_mon2_7)){lv_obj_del(img_mon2_7); img_mon2_7 = NULL;}
			if(img_mon2_8 && lv_obj_is_valid(img_mon2_8)){lv_obj_del(img_mon2_8); img_mon2_8 = NULL;}
			if(img_mon2_9 && lv_obj_is_valid(img_mon2_9)){lv_obj_del(img_mon2_9); img_mon2_9 = NULL;}
			if(img_mon2_10 && lv_obj_is_valid(img_mon2_10)){lv_obj_del(img_mon2_10); img_mon2_10 = NULL;}
			if(img_mon2_11 && lv_obj_is_valid(img_mon2_11)){lv_obj_del(img_mon2_11); img_mon2_11 = NULL;}
			if(img_mon2_12 && lv_obj_is_valid(img_mon2_12)){lv_obj_del(img_mon2_12); img_mon2_12 = NULL;}
			if(img_mon2_13 && lv_obj_is_valid(img_mon2_13)){lv_obj_del(img_mon2_13); img_mon2_13 = NULL;}
			if(img_mon2_14 && lv_obj_is_valid(img_mon2_14)){lv_obj_del(img_mon2_14); img_mon2_14 = NULL;}
			if(img_mon2_1 == NULL){
				img_mon2_1 =lv_img_create(parent);
				lv_img_set_src(img_mon2_1, &mon_2);
				lv_obj_align(img_mon2_1, LV_ALIGN_TOP_MID, -38, 251);
			}
			break;
		}
		case 2:
		{
			if(img_mon2_3 && lv_obj_is_valid(img_mon2_3)){lv_obj_del(img_mon2_3); img_mon2_3 = NULL;}
			if(img_mon2_4 && lv_obj_is_valid(img_mon2_4)){lv_obj_del(img_mon2_4); img_mon2_4 = NULL;}
			if(img_mon2_5 && lv_obj_is_valid(img_mon2_5)){lv_obj_del(img_mon2_5); img_mon2_5 = NULL;}
			if(img_mon2_6 && lv_obj_is_valid(img_mon2_6)){lv_obj_del(img_mon2_6); img_mon2_6 = NULL;}
			if(img_mon2_7 && lv_obj_is_valid(img_mon2_7)){lv_obj_del(img_mon2_7); img_mon2_7 = NULL;}
			if(img_mon2_8 && lv_obj_is_valid(img_mon2_8)){lv_obj_del(img_mon2_8); img_mon2_8 = NULL;}
			if(img_mon2_9 && lv_obj_is_valid(img_mon2_9)){lv_obj_del(img_mon2_9); img_mon2_9 = NULL;}
			if(img_mon2_10 && lv_obj_is_valid(img_mon2_10)){lv_obj_del(img_mon2_10); img_mon2_10 = NULL;}
			if(img_mon2_11 && lv_obj_is_valid(img_mon2_11)){lv_obj_del(img_mon2_11); img_mon2_11 = NULL;}
			if(img_mon2_12 && lv_obj_is_valid(img_mon2_12)){lv_obj_del(img_mon2_12); img_mon2_12 = NULL;}
			if(img_mon2_13 && lv_obj_is_valid(img_mon2_13)){lv_obj_del(img_mon2_13); img_mon2_13 = NULL;}
			if(img_mon2_14 && lv_obj_is_valid(img_mon2_14)){lv_obj_del(img_mon2_14); img_mon2_14 = NULL;}
			if(img_mon2_1 == NULL){
				img_mon2_1 =lv_img_create(parent);
				lv_img_set_src(img_mon2_1, &mon_2);
				lv_obj_align(img_mon2_1, LV_ALIGN_TOP_MID, -38, 251);
			}
			if(img_mon2_2 == NULL){
				img_mon2_2 =lv_img_create(parent);
				lv_img_set_src(img_mon2_2, &mon_2);
				lv_obj_align(img_mon2_2, LV_ALIGN_TOP_MID, -32, 251);
			}
			break;
		}
		case 3:
		{
			if(img_mon2_4 && lv_obj_is_valid(img_mon2_4)){lv_obj_del(img_mon2_4); img_mon2_4 = NULL;}
			if(img_mon2_5 && lv_obj_is_valid(img_mon2_5)){lv_obj_del(img_mon2_5); img_mon2_5 = NULL;}
			if(img_mon2_6 && lv_obj_is_valid(img_mon2_6)){lv_obj_del(img_mon2_6); img_mon2_6 = NULL;}
			if(img_mon2_7 && lv_obj_is_valid(img_mon2_7)){lv_obj_del(img_mon2_7); img_mon2_7 = NULL;}
			if(img_mon2_8 && lv_obj_is_valid(img_mon2_8)){lv_obj_del(img_mon2_8); img_mon2_8 = NULL;}
			if(img_mon2_9 && lv_obj_is_valid(img_mon2_9)){lv_obj_del(img_mon2_9); img_mon2_9 = NULL;}
			if(img_mon2_10 && lv_obj_is_valid(img_mon2_10)){lv_obj_del(img_mon2_10); img_mon2_10 = NULL;}
			if(img_mon2_11 && lv_obj_is_valid(img_mon2_11)){lv_obj_del(img_mon2_11); img_mon2_11 = NULL;}
			if(img_mon2_12 && lv_obj_is_valid(img_mon2_12)){lv_obj_del(img_mon2_12); img_mon2_12 = NULL;}
			if(img_mon2_13 && lv_obj_is_valid(img_mon2_13)){lv_obj_del(img_mon2_13); img_mon2_13 = NULL;}
			if(img_mon2_14 && lv_obj_is_valid(img_mon2_14)){lv_obj_del(img_mon2_14); img_mon2_14 = NULL;}
			if(img_mon2_1 == NULL){
				img_mon2_1 =lv_img_create(parent);
				lv_img_set_src(img_mon2_1, &mon_2);
				lv_obj_align(img_mon2_1, LV_ALIGN_TOP_MID, -38, 251);
			}
			if(img_mon2_2 == NULL){
				img_mon2_2 =lv_img_create(parent);
				lv_img_set_src(img_mon2_2, &mon_2);
				lv_obj_align(img_mon2_2, LV_ALIGN_TOP_MID, -32, 251);
			}
			if(img_mon2_3 == NULL){
				img_mon2_3 =lv_img_create(parent);
				lv_img_set_src(img_mon2_3, &mon_2);
				lv_obj_align(img_mon2_3, LV_ALIGN_TOP_MID, -26, 251);
			}
			break;
		}
		case 4:
		{
			if(img_mon2_5 && lv_obj_is_valid(img_mon2_5)){lv_obj_del(img_mon2_5); img_mon2_5 = NULL;}
			if(img_mon2_6 && lv_obj_is_valid(img_mon2_6)){lv_obj_del(img_mon2_6); img_mon2_6 = NULL;}
			if(img_mon2_7 && lv_obj_is_valid(img_mon2_7)){lv_obj_del(img_mon2_7); img_mon2_7 = NULL;}
			if(img_mon2_8 && lv_obj_is_valid(img_mon2_8)){lv_obj_del(img_mon2_8); img_mon2_8 = NULL;}
			if(img_mon2_9 && lv_obj_is_valid(img_mon2_9)){lv_obj_del(img_mon2_9); img_mon2_9 = NULL;}
			if(img_mon2_10 && lv_obj_is_valid(img_mon2_10)){lv_obj_del(img_mon2_10); img_mon2_10 = NULL;}
			if(img_mon2_11 && lv_obj_is_valid(img_mon2_11)){lv_obj_del(img_mon2_11); img_mon2_11 = NULL;}
			if(img_mon2_12 && lv_obj_is_valid(img_mon2_12)){lv_obj_del(img_mon2_12); img_mon2_12 = NULL;}
			if(img_mon2_13 && lv_obj_is_valid(img_mon2_13)){lv_obj_del(img_mon2_13); img_mon2_13 = NULL;}
			if(img_mon2_14 && lv_obj_is_valid(img_mon2_14)){lv_obj_del(img_mon2_14); img_mon2_14 = NULL;}
			if(img_mon2_1 == NULL){
				img_mon2_1 =lv_img_create(parent);
				lv_img_set_src(img_mon2_1, &mon_2);
				lv_obj_align(img_mon2_1, LV_ALIGN_TOP_MID, -38, 251);
			}
			if(img_mon2_2 == NULL){
				img_mon2_2 =lv_img_create(parent);
				lv_img_set_src(img_mon2_2, &mon_2);
				lv_obj_align(img_mon2_2, LV_ALIGN_TOP_MID, -32, 251);
			}
			if(img_mon2_3 == NULL){
				img_mon2_3 =lv_img_create(parent);
				lv_img_set_src(img_mon2_3, &mon_2);
				lv_obj_align(img_mon2_3, LV_ALIGN_TOP_MID, -26, 251);
			}
			if(img_mon2_4 == NULL){
				img_mon2_4 =lv_img_create(parent);
				lv_img_set_src(img_mon2_4, &mon_2);
				lv_obj_align(img_mon2_4, LV_ALIGN_TOP_MID, -20, 251);
			}
			break;
		}
		case 5:
		{
			if(img_mon2_6 && lv_obj_is_valid(img_mon2_6)){lv_obj_del(img_mon2_6); img_mon2_6 = NULL;}
			if(img_mon2_7 && lv_obj_is_valid(img_mon2_7)){lv_obj_del(img_mon2_7); img_mon2_7 = NULL;}
			if(img_mon2_8 && lv_obj_is_valid(img_mon2_8)){lv_obj_del(img_mon2_8); img_mon2_8 = NULL;}
			if(img_mon2_9 && lv_obj_is_valid(img_mon2_9)){lv_obj_del(img_mon2_9); img_mon2_9 = NULL;}
			if(img_mon2_10 && lv_obj_is_valid(img_mon2_10)){lv_obj_del(img_mon2_10); img_mon2_10 = NULL;}
			if(img_mon2_11 && lv_obj_is_valid(img_mon2_11)){lv_obj_del(img_mon2_11); img_mon2_11 = NULL;}
			if(img_mon2_12 && lv_obj_is_valid(img_mon2_12)){lv_obj_del(img_mon2_12); img_mon2_12 = NULL;}
			if(img_mon2_13 && lv_obj_is_valid(img_mon2_13)){lv_obj_del(img_mon2_13); img_mon2_13 = NULL;}
			if(img_mon2_14 && lv_obj_is_valid(img_mon2_14)){lv_obj_del(img_mon2_14); img_mon2_14 = NULL;}
			if(img_mon2_1 == NULL){
				img_mon2_1 =lv_img_create(parent);
				lv_img_set_src(img_mon2_1, &mon_2);
				lv_obj_align(img_mon2_1, LV_ALIGN_TOP_MID, -38, 251);
			}
			if(img_mon2_2 == NULL){
				img_mon2_2 =lv_img_create(parent);
				lv_img_set_src(img_mon2_2, &mon_2);
				lv_obj_align(img_mon2_2, LV_ALIGN_TOP_MID, -32, 251);
			}
			if(img_mon2_3 == NULL){
				img_mon2_3 =lv_img_create(parent);
				lv_img_set_src(img_mon2_3, &mon_2);
				lv_obj_align(img_mon2_3, LV_ALIGN_TOP_MID, -26, 251);
			}
			if(img_mon2_4 == NULL){
				img_mon2_4 =lv_img_create(parent);
				lv_img_set_src(img_mon2_4, &mon_2);
				lv_obj_align(img_mon2_4, LV_ALIGN_TOP_MID, -20, 251);
			}
			if(img_mon2_5 == NULL){
				img_mon2_5 =lv_img_create(parent);
				lv_img_set_src(img_mon2_5, &mon_2);
				lv_obj_align(img_mon2_5, LV_ALIGN_TOP_MID, -14, 251);
			}
			break;
		}
		case 6:
		{
			if(img_mon2_7 && lv_obj_is_valid(img_mon2_7)){lv_obj_del(img_mon2_7); img_mon2_7 = NULL;}
			if(img_mon2_8 && lv_obj_is_valid(img_mon2_8)){lv_obj_del(img_mon2_8); img_mon2_8 = NULL;}
			if(img_mon2_9 && lv_obj_is_valid(img_mon2_9)){lv_obj_del(img_mon2_9); img_mon2_9 = NULL;}
			if(img_mon2_10 && lv_obj_is_valid(img_mon2_10)){lv_obj_del(img_mon2_10); img_mon2_10 = NULL;}
			if(img_mon2_11 && lv_obj_is_valid(img_mon2_11)){lv_obj_del(img_mon2_11); img_mon2_11 = NULL;}
			if(img_mon2_12 && lv_obj_is_valid(img_mon2_12)){lv_obj_del(img_mon2_12); img_mon2_12 = NULL;}
			if(img_mon2_13 && lv_obj_is_valid(img_mon2_13)){lv_obj_del(img_mon2_13); img_mon2_13 = NULL;}
			if(img_mon2_14 && lv_obj_is_valid(img_mon2_14)){lv_obj_del(img_mon2_14); img_mon2_14 = NULL;}
			if(img_mon2_1 == NULL){
				img_mon2_1 =lv_img_create(parent);
				lv_img_set_src(img_mon2_1, &mon_2);
				lv_obj_align(img_mon2_1, LV_ALIGN_TOP_MID, -38, 251);
			}
			if(img_mon2_2 == NULL){
				img_mon2_2 =lv_img_create(parent);
				lv_img_set_src(img_mon2_2, &mon_2);
				lv_obj_align(img_mon2_2, LV_ALIGN_TOP_MID, -32, 251);
			}
			if(img_mon2_3 == NULL){
				img_mon2_3 =lv_img_create(parent);
				lv_img_set_src(img_mon2_3, &mon_2);
				lv_obj_align(img_mon2_3, LV_ALIGN_TOP_MID, -26, 251);
			}
			if(img_mon2_4 == NULL){
				img_mon2_4 =lv_img_create(parent);
				lv_img_set_src(img_mon2_4, &mon_2);
				lv_obj_align(img_mon2_4, LV_ALIGN_TOP_MID, -20, 251);
			}
			if(img_mon2_5 == NULL){
				img_mon2_5 =lv_img_create(parent);
				lv_img_set_src(img_mon2_5, &mon_2);
				lv_obj_align(img_mon2_5, LV_ALIGN_TOP_MID, -14, 251);
			}
			if(img_mon2_6 == NULL){
				img_mon2_6 =lv_img_create(parent);
				lv_img_set_src(img_mon2_6, &mon_2);
				lv_obj_align(img_mon2_6, LV_ALIGN_TOP_MID, -8, 251);
			}
			break;
		}
		case 7:
		{
			if(img_mon2_8 && lv_obj_is_valid(img_mon2_8)){lv_obj_del(img_mon2_8); img_mon2_8 = NULL;}
			if(img_mon2_9 && lv_obj_is_valid(img_mon2_9)){lv_obj_del(img_mon2_9); img_mon2_9 = NULL;}
			if(img_mon2_10 && lv_obj_is_valid(img_mon2_10)){lv_obj_del(img_mon2_10); img_mon2_10 = NULL;}
			if(img_mon2_11 && lv_obj_is_valid(img_mon2_11)){lv_obj_del(img_mon2_11); img_mon2_11 = NULL;}
			if(img_mon2_12 && lv_obj_is_valid(img_mon2_12)){lv_obj_del(img_mon2_12); img_mon2_12 = NULL;}
			if(img_mon2_13 && lv_obj_is_valid(img_mon2_13)){lv_obj_del(img_mon2_13); img_mon2_13 = NULL;}
			if(img_mon2_14 && lv_obj_is_valid(img_mon2_14)){lv_obj_del(img_mon2_14); img_mon2_14 = NULL;}
			if(img_mon2_1 == NULL){
				img_mon2_1 =lv_img_create(parent);
				lv_img_set_src(img_mon2_1, &mon_2);
				lv_obj_align(img_mon2_1, LV_ALIGN_TOP_MID, -38, 251);
			}
			if(img_mon2_2 == NULL){
				img_mon2_2 =lv_img_create(parent);
				lv_img_set_src(img_mon2_2, &mon_2);
				lv_obj_align(img_mon2_2, LV_ALIGN_TOP_MID, -32, 251);
			}
			if(img_mon2_3 == NULL){
				img_mon2_3 =lv_img_create(parent);
				lv_img_set_src(img_mon2_3, &mon_2);
				lv_obj_align(img_mon2_3, LV_ALIGN_TOP_MID, -26, 251);
			}
			if(img_mon2_4 == NULL){
				img_mon2_4 =lv_img_create(parent);
				lv_img_set_src(img_mon2_4, &mon_2);
				lv_obj_align(img_mon2_4, LV_ALIGN_TOP_MID, -20, 251);
			}
			if(img_mon2_5 == NULL){
				img_mon2_5 =lv_img_create(parent);
				lv_img_set_src(img_mon2_5, &mon_2);
				lv_obj_align(img_mon2_5, LV_ALIGN_TOP_MID, -14, 251);
			}
			if(img_mon2_6 == NULL){
				img_mon2_6 =lv_img_create(parent);
				lv_img_set_src(img_mon2_6, &mon_2);
				lv_obj_align(img_mon2_6, LV_ALIGN_TOP_MID, -8, 251);
			}
			if(img_mon2_7 == NULL){
				img_mon2_7 =lv_img_create(parent);
				lv_img_set_src(img_mon2_7, &mon_2);
				lv_obj_align(img_mon2_7, LV_ALIGN_TOP_MID, -2, 251);
			}
			break;
		}
		case 8:
		{
			if(img_mon2_9 && lv_obj_is_valid(img_mon2_9)){lv_obj_del(img_mon2_9); img_mon2_9 = NULL;}
			if(img_mon2_10 && lv_obj_is_valid(img_mon2_10)){lv_obj_del(img_mon2_10); img_mon2_10 = NULL;}
			if(img_mon2_11 && lv_obj_is_valid(img_mon2_11)){lv_obj_del(img_mon2_11); img_mon2_11 = NULL;}
			if(img_mon2_12 && lv_obj_is_valid(img_mon2_12)){lv_obj_del(img_mon2_12); img_mon2_12 = NULL;}
			if(img_mon2_13 && lv_obj_is_valid(img_mon2_13)){lv_obj_del(img_mon2_13); img_mon2_13 = NULL;}
			if(img_mon2_14 && lv_obj_is_valid(img_mon2_14)){lv_obj_del(img_mon2_14); img_mon2_14 = NULL;}
			if(img_mon2_1 == NULL){
				img_mon2_1 =lv_img_create(parent);
				lv_img_set_src(img_mon2_1, &mon_2);
				lv_obj_align(img_mon2_1, LV_ALIGN_TOP_MID, -38, 251);
			}
			if(img_mon2_2 == NULL){
				img_mon2_2 =lv_img_create(parent);
				lv_img_set_src(img_mon2_2, &mon_2);
				lv_obj_align(img_mon2_2, LV_ALIGN_TOP_MID, -32, 251);
			}
			if(img_mon2_3 == NULL){
				img_mon2_3 =lv_img_create(parent);
				lv_img_set_src(img_mon2_3, &mon_2);
				lv_obj_align(img_mon2_3, LV_ALIGN_TOP_MID, -26, 251);
			}
			if(img_mon2_4 == NULL){
				img_mon2_4 =lv_img_create(parent);
				lv_img_set_src(img_mon2_4, &mon_2);
				lv_obj_align(img_mon2_4, LV_ALIGN_TOP_MID, -20, 251);
			}
			if(img_mon2_5 == NULL){
				img_mon2_5 =lv_img_create(parent);
				lv_img_set_src(img_mon2_5, &mon_2);
				lv_obj_align(img_mon2_5, LV_ALIGN_TOP_MID, -14, 251);
			}
			if(img_mon2_6 == NULL){
				img_mon2_6 =lv_img_create(parent);
				lv_img_set_src(img_mon2_6, &mon_2);
				lv_obj_align(img_mon2_6, LV_ALIGN_TOP_MID, -8, 251);
			}
			if(img_mon2_7 == NULL){
				img_mon2_7 =lv_img_create(parent);
				lv_img_set_src(img_mon2_7, &mon_2);
				lv_obj_align(img_mon2_7, LV_ALIGN_TOP_MID, -2, 251);
			}
			if(img_mon2_8 == NULL){
				img_mon2_8 =lv_img_create(parent);
				lv_img_set_src(img_mon2_8, &mon_2);
				lv_obj_align(img_mon2_8, LV_ALIGN_TOP_MID, 2+2, 251);
			}
			break;
		}
		case 9:
		{
			if(img_mon2_10 && lv_obj_is_valid(img_mon2_10)){lv_obj_del(img_mon2_10); img_mon2_10 = NULL;}
			if(img_mon2_11 && lv_obj_is_valid(img_mon2_11)){lv_obj_del(img_mon2_11); img_mon2_11 = NULL;}
			if(img_mon2_12 && lv_obj_is_valid(img_mon2_12)){lv_obj_del(img_mon2_12); img_mon2_12 = NULL;}
			if(img_mon2_13 && lv_obj_is_valid(img_mon2_13)){lv_obj_del(img_mon2_13); img_mon2_13 = NULL;}
			if(img_mon2_14 && lv_obj_is_valid(img_mon2_14)){lv_obj_del(img_mon2_14); img_mon2_14 = NULL;}
			if(img_mon2_1 == NULL){
				img_mon2_1 =lv_img_create(parent);
				lv_img_set_src(img_mon2_1, &mon_2);
				lv_obj_align(img_mon2_1, LV_ALIGN_TOP_MID, -38, 251);
			}
			if(img_mon2_2 == NULL){
				img_mon2_2 =lv_img_create(parent);
				lv_img_set_src(img_mon2_2, &mon_2);
				lv_obj_align(img_mon2_2, LV_ALIGN_TOP_MID, -32, 251);
			}
			if(img_mon2_3 == NULL){
				img_mon2_3 =lv_img_create(parent);
				lv_img_set_src(img_mon2_3, &mon_2);
				lv_obj_align(img_mon2_3, LV_ALIGN_TOP_MID, -26, 251);
			}
			if(img_mon2_4 == NULL){
				img_mon2_4 =lv_img_create(parent);
				lv_img_set_src(img_mon2_4, &mon_2);
				lv_obj_align(img_mon2_4, LV_ALIGN_TOP_MID, -20, 251);
			}
			if(img_mon2_5 == NULL){
				img_mon2_5 =lv_img_create(parent);
				lv_img_set_src(img_mon2_5, &mon_2);
				lv_obj_align(img_mon2_5, LV_ALIGN_TOP_MID, -14, 251);
			}
			if(img_mon2_6 == NULL){
				img_mon2_6 =lv_img_create(parent);
				lv_img_set_src(img_mon2_6, &mon_2);
				lv_obj_align(img_mon2_6, LV_ALIGN_TOP_MID, -8, 251);
			}
			if(img_mon2_7 == NULL){
				img_mon2_7 =lv_img_create(parent);
				lv_img_set_src(img_mon2_7, &mon_2);
				lv_obj_align(img_mon2_7, LV_ALIGN_TOP_MID, -2, 251);
			}
			if(img_mon2_8 == NULL){
				img_mon2_8 =lv_img_create(parent);
				lv_img_set_src(img_mon2_8, &mon_2);
				lv_obj_align(img_mon2_8, LV_ALIGN_TOP_MID, 2+2, 251);
			}
			if(img_mon2_9 == NULL){
				img_mon2_9 =lv_img_create(parent);
				lv_img_set_src(img_mon2_9, &mon_2);
				lv_obj_align(img_mon2_9, LV_ALIGN_TOP_MID, 8+2, 251);
			}
			break;
		}
		case 10:
		{
			if(img_mon2_11 && lv_obj_is_valid(img_mon2_11)){lv_obj_del(img_mon2_11); img_mon2_11 = NULL;}
			if(img_mon2_12 && lv_obj_is_valid(img_mon2_12)){lv_obj_del(img_mon2_12); img_mon2_12 = NULL;}
			if(img_mon2_13 && lv_obj_is_valid(img_mon2_13)){lv_obj_del(img_mon2_13); img_mon2_13 = NULL;}
			if(img_mon2_14 && lv_obj_is_valid(img_mon2_14)){lv_obj_del(img_mon2_14); img_mon2_14 = NULL;}
			if(img_mon2_1 == NULL){
				img_mon2_1 =lv_img_create(parent);
				lv_img_set_src(img_mon2_1, &mon_2);
				lv_obj_align(img_mon2_1, LV_ALIGN_TOP_MID, -38, 251);
			}
			if(img_mon2_2 == NULL){
				img_mon2_2 =lv_img_create(parent);
				lv_img_set_src(img_mon2_2, &mon_2);
				lv_obj_align(img_mon2_2, LV_ALIGN_TOP_MID, -32, 251);
			}
			if(img_mon2_3 == NULL){
				img_mon2_3 =lv_img_create(parent);
				lv_img_set_src(img_mon2_3, &mon_2);
				lv_obj_align(img_mon2_3, LV_ALIGN_TOP_MID, -26, 251);
			}
			if(img_mon2_4 == NULL){
				img_mon2_4 =lv_img_create(parent);
				lv_img_set_src(img_mon2_4, &mon_2);
				lv_obj_align(img_mon2_4, LV_ALIGN_TOP_MID, -20, 251);
			}
			if(img_mon2_5 == NULL){
				img_mon2_5 =lv_img_create(parent);
				lv_img_set_src(img_mon2_5, &mon_2);
				lv_obj_align(img_mon2_5, LV_ALIGN_TOP_MID, -14, 251);
			}
			if(img_mon2_6 == NULL){
				img_mon2_6 =lv_img_create(parent);
				lv_img_set_src(img_mon2_6, &mon_2);
				lv_obj_align(img_mon2_6, LV_ALIGN_TOP_MID, -8, 251);
			}
			if(img_mon2_7 == NULL){
				img_mon2_7 =lv_img_create(parent);
				lv_img_set_src(img_mon2_7, &mon_2);
				lv_obj_align(img_mon2_7, LV_ALIGN_TOP_MID, -2, 251);
			}
			if(img_mon2_8 == NULL){
				img_mon2_8 =lv_img_create(parent);
				lv_img_set_src(img_mon2_8, &mon_2);
				lv_obj_align(img_mon2_8, LV_ALIGN_TOP_MID, 2+2, 251);
			}
			if(img_mon2_9 == NULL){
				img_mon2_9 =lv_img_create(parent);
				lv_img_set_src(img_mon2_9, &mon_2);
				lv_obj_align(img_mon2_9, LV_ALIGN_TOP_MID, 8+2, 251);
			}
			if(img_mon2_10 == NULL){
				img_mon2_10 =lv_img_create(parent);
				lv_img_set_src(img_mon2_10, &mon_2);
				lv_obj_align(img_mon2_10, LV_ALIGN_TOP_MID, 14+2, 251);
			}
			break;
		}
		case 11:
		{
			if(img_mon2_12 && lv_obj_is_valid(img_mon2_12)){lv_obj_del(img_mon2_12); img_mon2_12 = NULL;}
			if(img_mon2_13 && lv_obj_is_valid(img_mon2_13)){lv_obj_del(img_mon2_13); img_mon2_13 = NULL;}
			if(img_mon2_14 && lv_obj_is_valid(img_mon2_14)){lv_obj_del(img_mon2_14); img_mon2_14 = NULL;}
			if(img_mon2_1 == NULL){
				img_mon2_1 =lv_img_create(parent);
				lv_img_set_src(img_mon2_1, &mon_2);
				lv_obj_align(img_mon2_1, LV_ALIGN_TOP_MID, -38, 251);
			}
			if(img_mon2_2 == NULL){
				img_mon2_2 =lv_img_create(parent);
				lv_img_set_src(img_mon2_2, &mon_2);
				lv_obj_align(img_mon2_2, LV_ALIGN_TOP_MID, -32, 251);
			}
			if(img_mon2_3 == NULL){
				img_mon2_3 =lv_img_create(parent);
				lv_img_set_src(img_mon2_3, &mon_2);
				lv_obj_align(img_mon2_3, LV_ALIGN_TOP_MID, -26, 251);
			}
			if(img_mon2_4 == NULL){
				img_mon2_4 =lv_img_create(parent);
				lv_img_set_src(img_mon2_4, &mon_2);
				lv_obj_align(img_mon2_4, LV_ALIGN_TOP_MID, -20, 251);
			}
			if(img_mon2_5 == NULL){
				img_mon2_5 =lv_img_create(parent);
				lv_img_set_src(img_mon2_5, &mon_2);
				lv_obj_align(img_mon2_5, LV_ALIGN_TOP_MID, -14, 251);
			}
			if(img_mon2_6 == NULL){
				img_mon2_6 =lv_img_create(parent);
				lv_img_set_src(img_mon2_6, &mon_2);
				lv_obj_align(img_mon2_6, LV_ALIGN_TOP_MID, -8, 251);
			}
			if(img_mon2_7 == NULL){
				img_mon2_7 =lv_img_create(parent);
				lv_img_set_src(img_mon2_7, &mon_2);
				lv_obj_align(img_mon2_7, LV_ALIGN_TOP_MID, -2, 251);
			}
			if(img_mon2_8 == NULL){
				img_mon2_8 =lv_img_create(parent);
				lv_img_set_src(img_mon2_8, &mon_2);
				lv_obj_align(img_mon2_8, LV_ALIGN_TOP_MID, 2+2, 251);
			}
			if(img_mon2_9 == NULL){
				img_mon2_9 =lv_img_create(parent);
				lv_img_set_src(img_mon2_9, &mon_2);
				lv_obj_align(img_mon2_9, LV_ALIGN_TOP_MID, 8+2, 251);
			}
			if(img_mon2_10 == NULL){
				img_mon2_10 =lv_img_create(parent);
				lv_img_set_src(img_mon2_10, &mon_2);
				lv_obj_align(img_mon2_10, LV_ALIGN_TOP_MID, 14+2, 251);
			}
			if(img_mon2_11 == NULL){
				img_mon2_11 =lv_img_create(parent);
				lv_img_set_src(img_mon2_11, &mon_2);
				lv_obj_align(img_mon2_11, LV_ALIGN_TOP_MID, 20+2, 251);
			}
			break;
		}
		case 12:
		{
			if(img_mon2_13 && lv_obj_is_valid(img_mon2_13)){lv_obj_del(img_mon2_13); img_mon2_13 = NULL;}
			if(img_mon2_14 && lv_obj_is_valid(img_mon2_14)){lv_obj_del(img_mon2_14); img_mon2_14 = NULL;}
			if(img_mon2_1 == NULL){
				img_mon2_1 =lv_img_create(parent);
				lv_img_set_src(img_mon2_1, &mon_2);
				lv_obj_align(img_mon2_1, LV_ALIGN_TOP_MID, -38, 251);
			}
			if(img_mon2_2 == NULL){
				img_mon2_2 =lv_img_create(parent);
				lv_img_set_src(img_mon2_2, &mon_2);
				lv_obj_align(img_mon2_2, LV_ALIGN_TOP_MID, -32, 251);
			}
			if(img_mon2_3 == NULL){
				img_mon2_3 =lv_img_create(parent);
				lv_img_set_src(img_mon2_3, &mon_2);
				lv_obj_align(img_mon2_3, LV_ALIGN_TOP_MID, -26, 251);
			}
			if(img_mon2_4 == NULL){
				img_mon2_4 =lv_img_create(parent);
				lv_img_set_src(img_mon2_4, &mon_2);
				lv_obj_align(img_mon2_4, LV_ALIGN_TOP_MID, -20, 251);
			}
			if(img_mon2_5 == NULL){
				img_mon2_5 =lv_img_create(parent);
				lv_img_set_src(img_mon2_5, &mon_2);
				lv_obj_align(img_mon2_5, LV_ALIGN_TOP_MID, -14, 251);
			}
			if(img_mon2_6 == NULL){
				img_mon2_6 =lv_img_create(parent);
				lv_img_set_src(img_mon2_6, &mon_2);
				lv_obj_align(img_mon2_6, LV_ALIGN_TOP_MID, -8, 251);
			}
			if(img_mon2_7 == NULL){
				img_mon2_7 =lv_img_create(parent);
				lv_img_set_src(img_mon2_7, &mon_2);
				lv_obj_align(img_mon2_7, LV_ALIGN_TOP_MID, -2, 251);
			}
			if(img_mon2_8 == NULL){
				img_mon2_8 =lv_img_create(parent);
				lv_img_set_src(img_mon2_8, &mon_2);
				lv_obj_align(img_mon2_8, LV_ALIGN_TOP_MID, 2+2, 251);
			}
			if(img_mon2_9 == NULL){
				img_mon2_9 =lv_img_create(parent);
				lv_img_set_src(img_mon2_9, &mon_2);
				lv_obj_align(img_mon2_9, LV_ALIGN_TOP_MID, 8+2, 251);
			}
			if(img_mon2_10 == NULL){
				img_mon2_10 =lv_img_create(parent);
				lv_img_set_src(img_mon2_10, &mon_2);
				lv_obj_align(img_mon2_10, LV_ALIGN_TOP_MID, 14+2, 251);
			}
			if(img_mon2_11 == NULL){
				img_mon2_11 =lv_img_create(parent);
				lv_img_set_src(img_mon2_11, &mon_2);
				lv_obj_align(img_mon2_11, LV_ALIGN_TOP_MID, 20+2, 251);
			}
			if(img_mon2_12 == NULL){
				img_mon2_12 =lv_img_create(parent);
				lv_img_set_src(img_mon2_12, &mon_2);
				lv_obj_align(img_mon2_12, LV_ALIGN_TOP_MID, 26+2, 251);
			}
			break;
		}
		case 13:
		{
			if(img_mon2_14 && lv_obj_is_valid(img_mon2_14)){lv_obj_del(img_mon2_14); img_mon2_14 = NULL;}
			if(img_mon2_1 == NULL){
				img_mon2_1 =lv_img_create(parent);
				lv_img_set_src(img_mon2_1, &mon_2);
				lv_obj_align(img_mon2_1, LV_ALIGN_TOP_MID, -38, 251);
			}
			if(img_mon2_2 == NULL){
				img_mon2_2 =lv_img_create(parent);
				lv_img_set_src(img_mon2_2, &mon_2);
				lv_obj_align(img_mon2_2, LV_ALIGN_TOP_MID, -32, 251);
			}
			if(img_mon2_3 == NULL){
				img_mon2_3 =lv_img_create(parent);
				lv_img_set_src(img_mon2_3, &mon_2);
				lv_obj_align(img_mon2_3, LV_ALIGN_TOP_MID, -26, 251);
			}
			if(img_mon2_4 == NULL){
				img_mon2_4 =lv_img_create(parent);
				lv_img_set_src(img_mon2_4, &mon_2);
				lv_obj_align(img_mon2_4, LV_ALIGN_TOP_MID, -20, 251);
			}
			if(img_mon2_5 == NULL){
				img_mon2_5 =lv_img_create(parent);
				lv_img_set_src(img_mon2_5, &mon_2);
				lv_obj_align(img_mon2_5, LV_ALIGN_TOP_MID, -14, 251);
			}
			if(img_mon2_6 == NULL){
				img_mon2_6 =lv_img_create(parent);
				lv_img_set_src(img_mon2_6, &mon_2);
				lv_obj_align(img_mon2_6, LV_ALIGN_TOP_MID, -8, 251);
			}
			if(img_mon2_7 == NULL){
				img_mon2_7 =lv_img_create(parent);
				lv_img_set_src(img_mon2_7, &mon_2);
				lv_obj_align(img_mon2_7, LV_ALIGN_TOP_MID, -2, 251);
			}
			if(img_mon2_8 == NULL){
				img_mon2_8 =lv_img_create(parent);
				lv_img_set_src(img_mon2_8, &mon_2);
				lv_obj_align(img_mon2_8, LV_ALIGN_TOP_MID, 2+2, 251);
			}
			if(img_mon2_9 == NULL){
				img_mon2_9 =lv_img_create(parent);
				lv_img_set_src(img_mon2_9, &mon_2);
				lv_obj_align(img_mon2_9, LV_ALIGN_TOP_MID, 8+2, 251);
			}
			if(img_mon2_10 == NULL){
				img_mon2_10 =lv_img_create(parent);
				lv_img_set_src(img_mon2_10, &mon_2);
				lv_obj_align(img_mon2_10, LV_ALIGN_TOP_MID, 14+2, 251);
			}
			if(img_mon2_11 == NULL){
				img_mon2_11 =lv_img_create(parent);
				lv_img_set_src(img_mon2_11, &mon_2);
				lv_obj_align(img_mon2_11, LV_ALIGN_TOP_MID, 20+2, 251);
			}
			if(img_mon2_12 == NULL){
				img_mon2_12 =lv_img_create(parent);
				lv_img_set_src(img_mon2_12, &mon_2);
				lv_obj_align(img_mon2_12, LV_ALIGN_TOP_MID, 26+2, 251);
			}
			if(img_mon2_13 == NULL){
				img_mon2_13 =lv_img_create(parent);
				lv_img_set_src(img_mon2_13, &mon_2);
				lv_obj_align(img_mon2_13, LV_ALIGN_TOP_MID, 32+2, 251);
			}
			break;
		}
		case 14:
		{
			if(img_mon2_1 == NULL){
				img_mon2_1 =lv_img_create(parent);
				lv_img_set_src(img_mon2_1, &mon_2);
				lv_obj_align(img_mon2_1, LV_ALIGN_TOP_MID, -38, 251);
			}
			if(img_mon2_2 == NULL){
				img_mon2_2 =lv_img_create(parent);
				lv_img_set_src(img_mon2_2, &mon_2);
				lv_obj_align(img_mon2_2, LV_ALIGN_TOP_MID, -32, 251);
			}
			if(img_mon2_3 == NULL){
				img_mon2_3 =lv_img_create(parent);
				lv_img_set_src(img_mon2_3, &mon_2);
				lv_obj_align(img_mon2_3, LV_ALIGN_TOP_MID, -26, 251);
			}
			if(img_mon2_4 == NULL){
				img_mon2_4 =lv_img_create(parent);
				lv_img_set_src(img_mon2_4, &mon_2);
				lv_obj_align(img_mon2_4, LV_ALIGN_TOP_MID, -20, 251);
			}
			if(img_mon2_5 == NULL){
				img_mon2_5 =lv_img_create(parent);
				lv_img_set_src(img_mon2_5, &mon_2);
				lv_obj_align(img_mon2_5, LV_ALIGN_TOP_MID, -14, 251);
			}
			if(img_mon2_6 == NULL){
				img_mon2_6 =lv_img_create(parent);
				lv_img_set_src(img_mon2_6, &mon_2);
				lv_obj_align(img_mon2_6, LV_ALIGN_TOP_MID, -8, 251);
			}
			if(img_mon2_7 == NULL){
				img_mon2_7 =lv_img_create(parent);
				lv_img_set_src(img_mon2_7, &mon_2);
				lv_obj_align(img_mon2_7, LV_ALIGN_TOP_MID, -2, 251);
			}
			if(img_mon2_8 == NULL){
				img_mon2_8 =lv_img_create(parent);
				lv_img_set_src(img_mon2_8, &mon_2);
				lv_obj_align(img_mon2_8, LV_ALIGN_TOP_MID, 2+2, 251);
			}
			if(img_mon2_9 == NULL){
				img_mon2_9 =lv_img_create(parent);
				lv_img_set_src(img_mon2_9, &mon_2);
				lv_obj_align(img_mon2_9, LV_ALIGN_TOP_MID, 8+2, 251);
			}
			if(img_mon2_10 == NULL){
				img_mon2_10 =lv_img_create(parent);
				lv_img_set_src(img_mon2_10, &mon_2);
				lv_obj_align(img_mon2_10, LV_ALIGN_TOP_MID, 14+2, 251);
			}
			if(img_mon2_11 == NULL){
				img_mon2_11 =lv_img_create(parent);
				lv_img_set_src(img_mon2_11, &mon_2);
				lv_obj_align(img_mon2_11, LV_ALIGN_TOP_MID, 20+2, 251);
			}
			if(img_mon2_12 == NULL){
				img_mon2_12 =lv_img_create(parent);
				lv_img_set_src(img_mon2_12, &mon_2);
				lv_obj_align(img_mon2_12, LV_ALIGN_TOP_MID, 26+2, 251);
			}
			if(img_mon2_13 == NULL){
				img_mon2_13 =lv_img_create(parent);
				lv_img_set_src(img_mon2_13, &mon_2);
				lv_obj_align(img_mon2_13, LV_ALIGN_TOP_MID, 32+2, 251);
			}
			if(img_mon2_14 == NULL){
				img_mon2_14 =lv_img_create(parent);
				lv_img_set_src(img_mon2_14, &mon_2);
				lv_obj_align(img_mon2_14, LV_ALIGN_TOP_MID, 38+2, 251);
			}
			break;
		}
	}
}



#define Stime			20
#define BEEP_TIME	 0 //10//5	 //10
#define BEEP_TIME_TAG_SERA 0
void mon_img_pro_t(int data, lv_obj_t *parent, const void *mon_1, bool flag)
{
	  switch(data){//switch(data){
		case 0:
		{
			if(img_mon_1 && lv_obj_is_valid(img_mon_1)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_1); img_mon_1 = NULL;}
			if(img_mon_2 && lv_obj_is_valid(img_mon_2)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_2); img_mon_2 = NULL;}
			if(img_mon_3 && lv_obj_is_valid(img_mon_3)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_3); img_mon_3 = NULL;}
			if(img_mon_4 && lv_obj_is_valid(img_mon_4)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_4); img_mon_4 = NULL;}
			if(img_mon_5 && lv_obj_is_valid(img_mon_5)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_5); img_mon_5 = NULL;}
			if(img_mon_6 && lv_obj_is_valid(img_mon_6)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_6); img_mon_6 = NULL;}
			if(img_mon_7 && lv_obj_is_valid(img_mon_7)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_7); img_mon_7 = NULL;}
			if(img_mon_8 && lv_obj_is_valid(img_mon_8)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_8); img_mon_8 = NULL;}
			if(img_mon_9 && lv_obj_is_valid(img_mon_9)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_9); img_mon_9 = NULL;}
			if(img_mon_10 && lv_obj_is_valid(img_mon_10)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_10); img_mon_10 = NULL;}
			if(img_mon_11 && lv_obj_is_valid(img_mon_11)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_11); img_mon_11 = NULL;}
			if(img_mon_12 && lv_obj_is_valid(img_mon_12)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_12); img_mon_12 = NULL;}
			if(img_mon_13 && lv_obj_is_valid(img_mon_13)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_13); img_mon_13 = NULL;}
			if(img_mon_14 && lv_obj_is_valid(img_mon_14)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_14); img_mon_14 = NULL;}
			if(img_mon_15 && lv_obj_is_valid(img_mon_15)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_15); img_mon_15 = NULL;}
			if(img_mon_16 && lv_obj_is_valid(img_mon_16)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_16); img_mon_16 = NULL;}
			break;
		}
		case 1:
		{
			if(img_mon_2 && lv_obj_is_valid(img_mon_2)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_2); img_mon_2 = NULL;}
			if(img_mon_3 && lv_obj_is_valid(img_mon_3)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_3); img_mon_3 = NULL;}
			if(img_mon_4 && lv_obj_is_valid(img_mon_4)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_4); img_mon_4 = NULL;}
			if(img_mon_5 && lv_obj_is_valid(img_mon_5)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_5); img_mon_5 = NULL;}
			if(img_mon_6 && lv_obj_is_valid(img_mon_6)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_6); img_mon_6 = NULL;}
			if(img_mon_7 && lv_obj_is_valid(img_mon_7)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_7); img_mon_7 = NULL;}
			if(img_mon_8 && lv_obj_is_valid(img_mon_8)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_8); img_mon_8 = NULL;}
			if(img_mon_9 && lv_obj_is_valid(img_mon_9)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_9); img_mon_9 = NULL;}
			if(img_mon_10 && lv_obj_is_valid(img_mon_10)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_10); img_mon_10 = NULL;}
			if(img_mon_11 && lv_obj_is_valid(img_mon_11)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_11); img_mon_11 = NULL;}
			if(img_mon_12 && lv_obj_is_valid(img_mon_12)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_12); img_mon_12 = NULL;}
			if(img_mon_13 && lv_obj_is_valid(img_mon_13)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_13); img_mon_13 = NULL;}
			if(img_mon_14 && lv_obj_is_valid(img_mon_14)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_14); img_mon_14 = NULL;}
			if(img_mon_15 && lv_obj_is_valid(img_mon_15)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_15); img_mon_15 = NULL;}
			if(img_mon_16 && lv_obj_is_valid(img_mon_16)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_16); img_mon_16 = NULL;}
			if(img_mon_1 == NULL){
				img_mon_1 =lv_img_create(parent);
				lv_img_set_src(img_mon_1, mon_1);
				lv_obj_align(img_mon_1, LV_ALIGN_TOP_MID, 0, 220);
			}
			break;
		}
		case 2:
		{
			if(img_mon_1 && lv_obj_is_valid(img_mon_1)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_1); img_mon_1 = NULL;}
			if(img_mon_3 && lv_obj_is_valid(img_mon_3)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_3); img_mon_3 = NULL;}
			if(img_mon_4 && lv_obj_is_valid(img_mon_4)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_4); img_mon_4 = NULL;}
			if(img_mon_5 && lv_obj_is_valid(img_mon_5)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_5); img_mon_5 = NULL;}
			if(img_mon_6 && lv_obj_is_valid(img_mon_6)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_6); img_mon_6 = NULL;}
			if(img_mon_7 && lv_obj_is_valid(img_mon_7)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_7); img_mon_7 = NULL;}
			if(img_mon_8 && lv_obj_is_valid(img_mon_8)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_8); img_mon_8 = NULL;}
			if(img_mon_9 && lv_obj_is_valid(img_mon_9)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_9); img_mon_9 = NULL;}
			if(img_mon_10 && lv_obj_is_valid(img_mon_10)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_10); img_mon_10 = NULL;}
			if(img_mon_11 && lv_obj_is_valid(img_mon_11)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_11); img_mon_11 = NULL;}
			if(img_mon_12 && lv_obj_is_valid(img_mon_12)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_12); img_mon_12 = NULL;}
			if(img_mon_13 && lv_obj_is_valid(img_mon_13)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_13); img_mon_13 = NULL;}
			if(img_mon_14 && lv_obj_is_valid(img_mon_14)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_14); img_mon_14 = NULL;}
			if(img_mon_15 && lv_obj_is_valid(img_mon_15)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_15); img_mon_15 = NULL;}
			if(img_mon_16 && lv_obj_is_valid(img_mon_16)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_16); img_mon_16 = NULL;}
			if(img_mon_2 == NULL){
				img_mon_2 =lv_img_create(parent);
				lv_img_set_src(img_mon_2, mon_1);
				lv_obj_align(img_mon_2, LV_ALIGN_TOP_MID, 0, 209);
			}
			break;
		}
		case 3:
		{
			if(img_mon_1 && lv_obj_is_valid(img_mon_1)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_1); img_mon_1 = NULL;}
			if(img_mon_2 && lv_obj_is_valid(img_mon_2)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_2); img_mon_2 = NULL;}
			if(img_mon_4 && lv_obj_is_valid(img_mon_4)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_4); img_mon_4 = NULL;}
			if(img_mon_5 && lv_obj_is_valid(img_mon_5)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_5); img_mon_5 = NULL;}
			if(img_mon_6 && lv_obj_is_valid(img_mon_6)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_6); img_mon_6 = NULL;}
			if(img_mon_7 && lv_obj_is_valid(img_mon_7)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_7); img_mon_7 = NULL;}
			if(img_mon_8 && lv_obj_is_valid(img_mon_8)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_8); img_mon_8 = NULL;}
			if(img_mon_9 && lv_obj_is_valid(img_mon_9)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_9); img_mon_9 = NULL;}
			if(img_mon_10 && lv_obj_is_valid(img_mon_10)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_10); img_mon_10 = NULL;}
			if(img_mon_11 && lv_obj_is_valid(img_mon_11)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_11); img_mon_11 = NULL;}
			if(img_mon_12 && lv_obj_is_valid(img_mon_12)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_12); img_mon_12 = NULL;}
			if(img_mon_13 && lv_obj_is_valid(img_mon_13)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_13); img_mon_13 = NULL;}
			if(img_mon_14 && lv_obj_is_valid(img_mon_14)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_14); img_mon_14 = NULL;}
			if(img_mon_15 && lv_obj_is_valid(img_mon_15)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_15); img_mon_15 = NULL;}
			if(img_mon_16 && lv_obj_is_valid(img_mon_16)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_16); img_mon_16 = NULL;}
			
			if(img_mon_3 == NULL){
				img_mon_3 =lv_img_create(parent);
				lv_img_set_src(img_mon_3, mon_1);
				lv_obj_align(img_mon_3, LV_ALIGN_TOP_MID, 0, 198);
			}
			
			break;
		}
		case 4:
		{
			if(img_mon_1 && lv_obj_is_valid(img_mon_1)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_1); img_mon_1 = NULL;}
			if(img_mon_2 && lv_obj_is_valid(img_mon_2)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_2); img_mon_2 = NULL;}
			if(img_mon_3 && lv_obj_is_valid(img_mon_3)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_3); img_mon_3 = NULL;}
			if(img_mon_5 && lv_obj_is_valid(img_mon_5)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_5); img_mon_5 = NULL;}
			if(img_mon_6 && lv_obj_is_valid(img_mon_6)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_6); img_mon_6 = NULL;}
			if(img_mon_7 && lv_obj_is_valid(img_mon_7)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_7); img_mon_7 = NULL;}
			if(img_mon_8 && lv_obj_is_valid(img_mon_8)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_8); img_mon_8 = NULL;}
			if(img_mon_9 && lv_obj_is_valid(img_mon_9)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_9); img_mon_9 = NULL;}
			if(img_mon_10 && lv_obj_is_valid(img_mon_10)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_10); img_mon_10 = NULL;}
			if(img_mon_11 && lv_obj_is_valid(img_mon_11)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_11); img_mon_11 = NULL;}
			if(img_mon_12 && lv_obj_is_valid(img_mon_12)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_12); img_mon_12 = NULL;}
			if(img_mon_13 && lv_obj_is_valid(img_mon_13)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_13); img_mon_13 = NULL;}
			if(img_mon_14 && lv_obj_is_valid(img_mon_14)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_14); img_mon_14 = NULL;}
			if(img_mon_15 && lv_obj_is_valid(img_mon_15)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_15); img_mon_15 = NULL;}
			if(img_mon_16 && lv_obj_is_valid(img_mon_16)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_16); img_mon_16 = NULL;}
			if(img_mon_4 == NULL){
				img_mon_4 =lv_img_create(parent);
				lv_img_set_src(img_mon_4, mon_1);
				lv_obj_align(img_mon_4, LV_ALIGN_TOP_MID, 0, 187);
			}
			
			
			break;
		}
		case 5:
		{
			if(img_mon_1 && lv_obj_is_valid(img_mon_1)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_1); img_mon_1 = NULL;}
			if(img_mon_2 && lv_obj_is_valid(img_mon_2)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_2); img_mon_2 = NULL;}
			if(img_mon_3 && lv_obj_is_valid(img_mon_3)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_3); img_mon_3 = NULL;}
			if(img_mon_4 && lv_obj_is_valid(img_mon_4)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_4); img_mon_4 = NULL;}
			if(img_mon_6 && lv_obj_is_valid(img_mon_6)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_6); img_mon_6 = NULL;}
			if(img_mon_7 && lv_obj_is_valid(img_mon_7)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_7); img_mon_7 = NULL;}
			if(img_mon_8 && lv_obj_is_valid(img_mon_8)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_8); img_mon_8 = NULL;}
			if(img_mon_9 && lv_obj_is_valid(img_mon_9)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_9); img_mon_9 = NULL;}
			if(img_mon_10 && lv_obj_is_valid(img_mon_10)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_10); img_mon_10 = NULL;}
			if(img_mon_11 && lv_obj_is_valid(img_mon_11)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_11); img_mon_11 = NULL;}
			if(img_mon_12 && lv_obj_is_valid(img_mon_12)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_12); img_mon_12 = NULL;}
			if(img_mon_13 && lv_obj_is_valid(img_mon_13)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_13); img_mon_13 = NULL;}
			if(img_mon_14 && lv_obj_is_valid(img_mon_14)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_14); img_mon_14 = NULL;}
			if(img_mon_15 && lv_obj_is_valid(img_mon_15)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_15); img_mon_15 = NULL;}
			if(img_mon_16 && lv_obj_is_valid(img_mon_16)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_16); img_mon_16 = NULL;}
			
			if(img_mon_5 == NULL){
				img_mon_5 =lv_img_create(parent);
				lv_img_set_src(img_mon_5, mon_1);
				lv_obj_align(img_mon_5, LV_ALIGN_TOP_MID, 0, 177);
			}
			
			break;
		}
		case 6:
		{
			if(img_mon_1 && lv_obj_is_valid(img_mon_1)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_1); img_mon_1 = NULL;}
			if(img_mon_2 && lv_obj_is_valid(img_mon_2)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_2); img_mon_2 = NULL;}
			if(img_mon_3 && lv_obj_is_valid(img_mon_3)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_3); img_mon_3 = NULL;}
			if(img_mon_4 && lv_obj_is_valid(img_mon_4)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_4); img_mon_4 = NULL;}
			if(img_mon_5 && lv_obj_is_valid(img_mon_5)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_5); img_mon_5 = NULL;}
			if(img_mon_7 && lv_obj_is_valid(img_mon_7)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_7); img_mon_7 = NULL;}
			if(img_mon_8 && lv_obj_is_valid(img_mon_8)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_8); img_mon_8 = NULL;}
			if(img_mon_9 && lv_obj_is_valid(img_mon_9)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_9); img_mon_9 = NULL;}
			if(img_mon_10 && lv_obj_is_valid(img_mon_10)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_10); img_mon_10 = NULL;}
			if(img_mon_11 && lv_obj_is_valid(img_mon_11)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_11); img_mon_11 = NULL;}
			if(img_mon_12 && lv_obj_is_valid(img_mon_12)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_12); img_mon_12 = NULL;}
			if(img_mon_13 && lv_obj_is_valid(img_mon_13)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_13); img_mon_13 = NULL;}
			if(img_mon_14 && lv_obj_is_valid(img_mon_14)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_14); img_mon_14 = NULL;}
			if(img_mon_15 && lv_obj_is_valid(img_mon_15)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_15); img_mon_15 = NULL;}
			if(img_mon_16 && lv_obj_is_valid(img_mon_16)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_16); img_mon_16 = NULL;}
			if(img_mon_6 == NULL){
				img_mon_6 =lv_img_create(parent);
				lv_img_set_src(img_mon_6, mon_1);
				lv_obj_align(img_mon_6, LV_ALIGN_TOP_MID, 0, 166);
			}
			
			break;
		}
		case 7:
		{
			if(img_mon_1 && lv_obj_is_valid(img_mon_1)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_1); img_mon_1 = NULL;}
			if(img_mon_2 && lv_obj_is_valid(img_mon_2)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_2); img_mon_2 = NULL;}
			if(img_mon_3 && lv_obj_is_valid(img_mon_3)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_3); img_mon_3 = NULL;}
			if(img_mon_4 && lv_obj_is_valid(img_mon_4)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_4); img_mon_4 = NULL;}
			if(img_mon_5 && lv_obj_is_valid(img_mon_5)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_5); img_mon_5 = NULL;}
			if(img_mon_6 && lv_obj_is_valid(img_mon_6)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_6); img_mon_6 = NULL;}
			if(img_mon_8 && lv_obj_is_valid(img_mon_8)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_8); img_mon_8 = NULL;}
			if(img_mon_9 && lv_obj_is_valid(img_mon_9)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_9); img_mon_9 = NULL;}
			if(img_mon_10 && lv_obj_is_valid(img_mon_10)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_10); img_mon_10 = NULL;}
			if(img_mon_11 && lv_obj_is_valid(img_mon_11)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_11); img_mon_11 = NULL;}
			if(img_mon_12 && lv_obj_is_valid(img_mon_12)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_12); img_mon_12 = NULL;}
			if(img_mon_13 && lv_obj_is_valid(img_mon_13)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_13); img_mon_13 = NULL;}
			if(img_mon_14 && lv_obj_is_valid(img_mon_14)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_14); img_mon_14 = NULL;}
			if(img_mon_15 && lv_obj_is_valid(img_mon_15)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_15); img_mon_15 = NULL;}
			if(img_mon_16 && lv_obj_is_valid(img_mon_16)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_16); img_mon_16 = NULL;}
			if(img_mon_7 == NULL){
				img_mon_7 =lv_img_create(parent);
				lv_img_set_src(img_mon_7, mon_1);
				lv_obj_align(img_mon_7, LV_ALIGN_TOP_MID, 0, 156);
			}
			
			
			break;
		}
		case 8:
		{
			if(img_mon_1 && lv_obj_is_valid(img_mon_1)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_1); img_mon_1 = NULL;}
			if(img_mon_2 && lv_obj_is_valid(img_mon_2)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_2); img_mon_2 = NULL;}
			if(img_mon_3 && lv_obj_is_valid(img_mon_3)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_3); img_mon_3 = NULL;}
			if(img_mon_4 && lv_obj_is_valid(img_mon_4)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_4); img_mon_4 = NULL;}
			if(img_mon_5 && lv_obj_is_valid(img_mon_5)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_5); img_mon_5 = NULL;}
			if(img_mon_6 && lv_obj_is_valid(img_mon_6)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_6); img_mon_6 = NULL;}
			if(img_mon_7 && lv_obj_is_valid(img_mon_7)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_7); img_mon_7 = NULL;}
			if(img_mon_9 && lv_obj_is_valid(img_mon_9)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_9); img_mon_9 = NULL;}
			if(img_mon_10 && lv_obj_is_valid(img_mon_10)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_10); img_mon_10 = NULL;}
			if(img_mon_11 && lv_obj_is_valid(img_mon_11)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_11); img_mon_11 = NULL;}
			if(img_mon_12 && lv_obj_is_valid(img_mon_12)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_12); img_mon_12 = NULL;}
			if(img_mon_13 && lv_obj_is_valid(img_mon_13)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_13); img_mon_13 = NULL;}
			if(img_mon_14 && lv_obj_is_valid(img_mon_14)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_14); img_mon_14 = NULL;}
			if(img_mon_15 && lv_obj_is_valid(img_mon_15)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_15); img_mon_15 = NULL;}
			if(img_mon_16 && lv_obj_is_valid(img_mon_16)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_16); img_mon_16 = NULL;}
			
			if(img_mon_8 == NULL){
				img_mon_8 =lv_img_create(parent);
				lv_img_set_src(img_mon_8, mon_1);
				lv_obj_align(img_mon_8, LV_ALIGN_TOP_MID, 0, 146);
			}
			
			break;
		}
		case 9:
		{
			if(img_mon_1 && lv_obj_is_valid(img_mon_1)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_1); img_mon_1 = NULL;}
			if(img_mon_2 && lv_obj_is_valid(img_mon_2)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_2); img_mon_2 = NULL;}
			if(img_mon_3 && lv_obj_is_valid(img_mon_3)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_3); img_mon_3 = NULL;}
			if(img_mon_4 && lv_obj_is_valid(img_mon_4)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_4); img_mon_4 = NULL;}
			if(img_mon_5 && lv_obj_is_valid(img_mon_5)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_5); img_mon_5 = NULL;}
			if(img_mon_6 && lv_obj_is_valid(img_mon_6)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_6); img_mon_6 = NULL;}
			if(img_mon_7 && lv_obj_is_valid(img_mon_7)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_7); img_mon_7 = NULL;}
			if(img_mon_8 && lv_obj_is_valid(img_mon_8)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_8); img_mon_8 = NULL;}
			if(img_mon_10 && lv_obj_is_valid(img_mon_10)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_10); img_mon_10 = NULL;}
			if(img_mon_11 && lv_obj_is_valid(img_mon_11)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_11); img_mon_11 = NULL;}
			if(img_mon_12 && lv_obj_is_valid(img_mon_12)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_12); img_mon_12 = NULL;}
			if(img_mon_13 && lv_obj_is_valid(img_mon_13)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_13); img_mon_13 = NULL;}
			if(img_mon_14 && lv_obj_is_valid(img_mon_14)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_14); img_mon_14 = NULL;}
			if(img_mon_15 && lv_obj_is_valid(img_mon_15)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_15); img_mon_15 = NULL;}
			if(img_mon_16 && lv_obj_is_valid(img_mon_16)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_16); img_mon_16 = NULL;}
			
			if(img_mon_9 == NULL){
				img_mon_9 =lv_img_create(parent);
				lv_img_set_src(img_mon_9, mon_1);
				lv_obj_align(img_mon_9, LV_ALIGN_TOP_MID, 0, 135);
			}
			
			break;
		}
		case 10:
		{
			if(img_mon_1 && lv_obj_is_valid(img_mon_1)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_1); img_mon_1 = NULL;}
			if(img_mon_2 && lv_obj_is_valid(img_mon_2)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_2); img_mon_2 = NULL;}
			if(img_mon_3 && lv_obj_is_valid(img_mon_3)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_3); img_mon_3 = NULL;}
			if(img_mon_4 && lv_obj_is_valid(img_mon_4)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_4); img_mon_4 = NULL;}
			if(img_mon_5 && lv_obj_is_valid(img_mon_5)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_5); img_mon_5 = NULL;}
			if(img_mon_6 && lv_obj_is_valid(img_mon_6)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_6); img_mon_6 = NULL;}
			if(img_mon_7 && lv_obj_is_valid(img_mon_7)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_7); img_mon_7 = NULL;}
			if(img_mon_8 && lv_obj_is_valid(img_mon_8)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_8); img_mon_8 = NULL;}
			if(img_mon_9 && lv_obj_is_valid(img_mon_9)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_9); img_mon_9 = NULL;}
			if(img_mon_11 && lv_obj_is_valid(img_mon_11)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_11); img_mon_11 = NULL;}
			if(img_mon_12 && lv_obj_is_valid(img_mon_12)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_12); img_mon_12 = NULL;}
			if(img_mon_13 && lv_obj_is_valid(img_mon_13)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_13); img_mon_13 = NULL;}
			if(img_mon_14 && lv_obj_is_valid(img_mon_14)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_14); img_mon_14 = NULL;}
			if(img_mon_15 && lv_obj_is_valid(img_mon_15)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_15); img_mon_15 = NULL;}
			if(img_mon_16 && lv_obj_is_valid(img_mon_16)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_16); img_mon_16 = NULL;}
			
			
			if(img_mon_10 == NULL){
				img_mon_10 =lv_img_create(parent);
				lv_img_set_src(img_mon_10, mon_1);
				lv_obj_align(img_mon_10, LV_ALIGN_TOP_MID, 0, 123);
			}
			break;
		}
		case 11:
		{
			if(img_mon_1 && lv_obj_is_valid(img_mon_1)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_1); img_mon_1 = NULL;}
			if(img_mon_2 && lv_obj_is_valid(img_mon_2)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_2); img_mon_2 = NULL;}
			if(img_mon_3 && lv_obj_is_valid(img_mon_3)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_3); img_mon_3 = NULL;}
			if(img_mon_4 && lv_obj_is_valid(img_mon_4)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_4); img_mon_4 = NULL;}
			if(img_mon_5 && lv_obj_is_valid(img_mon_5)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_5); img_mon_5 = NULL;}
			if(img_mon_6 && lv_obj_is_valid(img_mon_6)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_6); img_mon_6 = NULL;}
			if(img_mon_7 && lv_obj_is_valid(img_mon_7)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_7); img_mon_7 = NULL;}
			if(img_mon_8 && lv_obj_is_valid(img_mon_8)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_8); img_mon_8 = NULL;}
			if(img_mon_9 && lv_obj_is_valid(img_mon_9)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_9); img_mon_9 = NULL;}
			if(img_mon_10 && lv_obj_is_valid(img_mon_10)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_10); img_mon_10 = NULL;}
			if(img_mon_12 && lv_obj_is_valid(img_mon_12)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_12); img_mon_12 = NULL;}
			if(img_mon_13 && lv_obj_is_valid(img_mon_13)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_13); img_mon_13 = NULL;}
			if(img_mon_14 && lv_obj_is_valid(img_mon_14)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_14); img_mon_14 = NULL;}
			if(img_mon_15 && lv_obj_is_valid(img_mon_15)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_15); img_mon_15 = NULL;}
			if(img_mon_16 && lv_obj_is_valid(img_mon_16)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_16); img_mon_16 = NULL;}
			
			if(img_mon_11 == NULL){
				img_mon_11 =lv_img_create(parent);
				lv_img_set_src(img_mon_11, mon_1);
				lv_obj_align(img_mon_11, LV_ALIGN_TOP_MID, 0, 113);
			}
			
			break;
		}
		case 12:
		{
			if(img_mon_1 && lv_obj_is_valid(img_mon_1)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_1); img_mon_1 = NULL;}
			if(img_mon_2 && lv_obj_is_valid(img_mon_2)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_2); img_mon_2 = NULL;}
			if(img_mon_3 && lv_obj_is_valid(img_mon_3)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_3); img_mon_3 = NULL;}
			if(img_mon_4 && lv_obj_is_valid(img_mon_4)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_4); img_mon_4 = NULL;}
			if(img_mon_5 && lv_obj_is_valid(img_mon_5)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_5); img_mon_5 = NULL;}
			if(img_mon_6 && lv_obj_is_valid(img_mon_6)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_6); img_mon_6 = NULL;}
			if(img_mon_7 && lv_obj_is_valid(img_mon_7)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_7); img_mon_7 = NULL;}
			if(img_mon_8 && lv_obj_is_valid(img_mon_8)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_8); img_mon_8 = NULL;}
			if(img_mon_9 && lv_obj_is_valid(img_mon_9)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_9); img_mon_9 = NULL;}
			if(img_mon_10 && lv_obj_is_valid(img_mon_10)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_10); img_mon_10 = NULL;}
			if(img_mon_11 && lv_obj_is_valid(img_mon_11)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_11); img_mon_11 = NULL;}
			if(img_mon_13 && lv_obj_is_valid(img_mon_13)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_13); img_mon_13 = NULL;}
			if(img_mon_14 && lv_obj_is_valid(img_mon_14)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_14); img_mon_14 = NULL;}
			if(img_mon_15 && lv_obj_is_valid(img_mon_15)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_15); img_mon_15 = NULL;}
			if(img_mon_16 && lv_obj_is_valid(img_mon_16)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_16); img_mon_16 = NULL;}
			
			if(img_mon_12 == NULL){
				img_mon_12 =lv_img_create(parent);
				lv_img_set_src(img_mon_12, mon_1);
				lv_obj_align(img_mon_12, LV_ALIGN_TOP_MID, 0, 103);
			}
			break;
		}
		case 13:
		{
			if(img_mon_1 && lv_obj_is_valid(img_mon_1)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_1); img_mon_1 = NULL;}
			if(img_mon_2 && lv_obj_is_valid(img_mon_2)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_2); img_mon_2 = NULL;}
			if(img_mon_3 && lv_obj_is_valid(img_mon_3)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_3); img_mon_3 = NULL;}
			if(img_mon_4 && lv_obj_is_valid(img_mon_4)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_4); img_mon_4 = NULL;}
			if(img_mon_5 && lv_obj_is_valid(img_mon_5)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_5); img_mon_5 = NULL;}
			if(img_mon_6 && lv_obj_is_valid(img_mon_6)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_6); img_mon_6 = NULL;}
			if(img_mon_7 && lv_obj_is_valid(img_mon_7)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_7); img_mon_7 = NULL;}
			if(img_mon_8 && lv_obj_is_valid(img_mon_8)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_8); img_mon_8 = NULL;}
			if(img_mon_9 && lv_obj_is_valid(img_mon_9)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_9); img_mon_9 = NULL;}
			if(img_mon_10 && lv_obj_is_valid(img_mon_10)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_10); img_mon_10 = NULL;}
			if(img_mon_11 && lv_obj_is_valid(img_mon_11)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_11); img_mon_11 = NULL;}
			if(img_mon_12 && lv_obj_is_valid(img_mon_12)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_12); img_mon_12 = NULL;}
			if(img_mon_14 && lv_obj_is_valid(img_mon_14)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_14); img_mon_14 = NULL;}
			if(img_mon_15 && lv_obj_is_valid(img_mon_15)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_15); img_mon_15 = NULL;}
			if(img_mon_16 && lv_obj_is_valid(img_mon_16)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_16); img_mon_16 = NULL;}
			
			if(img_mon_13 == NULL){
				img_mon_13 =lv_img_create(parent);
				lv_img_set_src(img_mon_13, mon_1);
				lv_obj_align(img_mon_13, LV_ALIGN_TOP_MID, 0, 92);
			}
			
			
			break;
		}
		case 14:
		{
			if(img_mon_1 && lv_obj_is_valid(img_mon_1)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_1); img_mon_1 = NULL;}
			if(img_mon_2 && lv_obj_is_valid(img_mon_2)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_2); img_mon_2 = NULL;}
			if(img_mon_3 && lv_obj_is_valid(img_mon_3)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_3); img_mon_3 = NULL;}
			if(img_mon_4 && lv_obj_is_valid(img_mon_4)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_4); img_mon_4 = NULL;}
			if(img_mon_5 && lv_obj_is_valid(img_mon_5)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_5); img_mon_5 = NULL;}
			if(img_mon_6 && lv_obj_is_valid(img_mon_6)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_6); img_mon_6 = NULL;}
			if(img_mon_7 && lv_obj_is_valid(img_mon_7)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_7); img_mon_7 = NULL;}
			if(img_mon_8 && lv_obj_is_valid(img_mon_8)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_8); img_mon_8 = NULL;}
			if(img_mon_9 && lv_obj_is_valid(img_mon_9)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_9); img_mon_9 = NULL;}
			if(img_mon_10 && lv_obj_is_valid(img_mon_10)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_10); img_mon_10 = NULL;}
			if(img_mon_11 && lv_obj_is_valid(img_mon_11)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_11); img_mon_11 = NULL;}
			if(img_mon_12 && lv_obj_is_valid(img_mon_12)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_12); img_mon_12 = NULL;}
			if(img_mon_13 && lv_obj_is_valid(img_mon_13)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_13); img_mon_13 = NULL;}
			if(img_mon_15 && lv_obj_is_valid(img_mon_15)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_15); img_mon_15 = NULL;}
			if(img_mon_16 && lv_obj_is_valid(img_mon_16)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_16); img_mon_16 = NULL;}
			if(img_mon_14 == NULL){
				img_mon_14 =lv_img_create(parent);
				lv_img_set_src(img_mon_14, mon_1);
				lv_obj_align(img_mon_14, LV_ALIGN_TOP_MID, 0, 82);
			}
			
			
			break;
		}
		case 15:
		{
			if(img_mon_1 && lv_obj_is_valid(img_mon_1)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_1); img_mon_1 = NULL;}
			if(img_mon_2 && lv_obj_is_valid(img_mon_2)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_2); img_mon_2 = NULL;}
			if(img_mon_3 && lv_obj_is_valid(img_mon_3)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_3); img_mon_3 = NULL;}
			if(img_mon_4 && lv_obj_is_valid(img_mon_4)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_4); img_mon_4 = NULL;}
			if(img_mon_5 && lv_obj_is_valid(img_mon_5)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_5); img_mon_5 = NULL;}
			if(img_mon_6 && lv_obj_is_valid(img_mon_6)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_6); img_mon_6 = NULL;}
			if(img_mon_7 && lv_obj_is_valid(img_mon_7)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_7); img_mon_7 = NULL;}
			if(img_mon_8 && lv_obj_is_valid(img_mon_8)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_8); img_mon_8 = NULL;}
			if(img_mon_9 && lv_obj_is_valid(img_mon_9)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_9); img_mon_9 = NULL;}
			if(img_mon_10 && lv_obj_is_valid(img_mon_10)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_10); img_mon_10 = NULL;}
			if(img_mon_11 && lv_obj_is_valid(img_mon_11)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_11); img_mon_11 = NULL;}
			if(img_mon_12 && lv_obj_is_valid(img_mon_12)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_12); img_mon_12 = NULL;}
			if(img_mon_13 && lv_obj_is_valid(img_mon_13)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_13); img_mon_13 = NULL;}
			if(img_mon_14 && lv_obj_is_valid(img_mon_14)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_14); img_mon_14 = NULL;}
			if(img_mon_16 && lv_obj_is_valid(img_mon_16)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_16); img_mon_16 = NULL;}
			if(img_mon_15 == NULL){
				img_mon_15 =lv_img_create(parent);
				lv_img_set_src(img_mon_15, mon_1);
				lv_obj_align(img_mon_15, LV_ALIGN_TOP_MID, 0, 71);
			}
			
			
			break;
		}
		case 16:
		{
			if(img_mon_1 && lv_obj_is_valid(img_mon_1)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_1); img_mon_1 = NULL;}
			if(img_mon_2 && lv_obj_is_valid(img_mon_2)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_2); img_mon_2 = NULL;}
			if(img_mon_3 && lv_obj_is_valid(img_mon_3)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_3); img_mon_3 = NULL;}
			if(img_mon_4 && lv_obj_is_valid(img_mon_4)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_4); img_mon_4 = NULL;}
			if(img_mon_5 && lv_obj_is_valid(img_mon_5)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_5); img_mon_5 = NULL;}
			if(img_mon_6 && lv_obj_is_valid(img_mon_6)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_6); img_mon_6 = NULL;}
			if(img_mon_7 && lv_obj_is_valid(img_mon_7)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_7); img_mon_7 = NULL;}
			if(img_mon_8 && lv_obj_is_valid(img_mon_8)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_8); img_mon_8 = NULL;}
			if(img_mon_9 && lv_obj_is_valid(img_mon_9)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_9); img_mon_9 = NULL;}
			if(img_mon_10 && lv_obj_is_valid(img_mon_10)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_10); img_mon_10 = NULL;}
			if(img_mon_11 && lv_obj_is_valid(img_mon_11)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_11); img_mon_11 = NULL;}
			if(img_mon_12 && lv_obj_is_valid(img_mon_12)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_12); img_mon_12 = NULL;}
			if(img_mon_13 && lv_obj_is_valid(img_mon_13)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_13); img_mon_13 = NULL;}
			if(img_mon_14 && lv_obj_is_valid(img_mon_14)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_14); img_mon_14 = NULL;}
			if(img_mon_15 && lv_obj_is_valid(img_mon_15)){beeper_start_tim(BEEP_TIME_TAG_SERA);lv_obj_del(img_mon_15); img_mon_15 = NULL;}


			if(img_mon_16 == NULL){
				img_mon_16 =lv_img_create(parent);
				lv_img_set_src(img_mon_16, mon_1);
				lv_obj_align(img_mon_16, LV_ALIGN_TOP_MID, 0, 61);
			}

			break;
		}
		default:break;
		}
}



void mon_img_pro(int data, lv_obj_t *parent, const void *mon_1, bool flag,  bool flag_t, bool clean, bool beep_pow)
{
		static int stat_num =0;
		static int curr_data =0;
		
		static int curr_data_flag =0;
		if(curr_data_flag != data){
			if( curr_data_flag > data){
				curr_data_flag--;
				if(curr_data_flag<=0)curr_data_flag =0;
			}else if(curr_data_flag == data){
				;//return;
			}else if(curr_data_flag < data){
				curr_data_flag++;
				if(curr_data_flag>=16)curr_data_flag =16;
			}
		}
		if(clean){
			curr_data_flag =0;
		}
	  switch(curr_data_flag){//switch(data){
		case 0:
		{
			if(beep_pow)beep_cout =0;
			if(img_mon_1 && lv_obj_is_valid(img_mon_1)){beeper_start_tim(BEEP_TIME);lv_obj_del(img_mon_1); img_mon_1 = NULL;}
			if(img_mon_2 && lv_obj_is_valid(img_mon_2)){beeper_start_tim(BEEP_TIME);lv_obj_del(img_mon_2); img_mon_2 = NULL;}
			if(img_mon_3 && lv_obj_is_valid(img_mon_3)){beeper_start_tim(BEEP_TIME);lv_obj_del(img_mon_3); img_mon_3 = NULL;}
			if(img_mon_4 && lv_obj_is_valid(img_mon_4)){beeper_start_tim(BEEP_TIME);lv_obj_del(img_mon_4); img_mon_4 = NULL;}
			if(img_mon_5 && lv_obj_is_valid(img_mon_5)){beeper_start_tim(BEEP_TIME);lv_obj_del(img_mon_5); img_mon_5 = NULL;}
			if(img_mon_6 && lv_obj_is_valid(img_mon_6)){beeper_start_tim(BEEP_TIME);lv_obj_del(img_mon_6); img_mon_6 = NULL;}
			if(img_mon_7 && lv_obj_is_valid(img_mon_7)){beeper_start_tim(BEEP_TIME);lv_obj_del(img_mon_7); img_mon_7 = NULL;}
			if(img_mon_8 && lv_obj_is_valid(img_mon_8)){beeper_start_tim(BEEP_TIME);lv_obj_del(img_mon_8); img_mon_8 = NULL;}
			if(img_mon_9 && lv_obj_is_valid(img_mon_9)){beeper_start_tim(BEEP_TIME);lv_obj_del(img_mon_9); img_mon_9 = NULL;}
			if(img_mon_10 && lv_obj_is_valid(img_mon_10)){beeper_start_tim(BEEP_TIME);lv_obj_del(img_mon_10); img_mon_10 = NULL;}
			if(img_mon_11 && lv_obj_is_valid(img_mon_11)){beeper_start_tim(BEEP_TIME);lv_obj_del(img_mon_11); img_mon_11 = NULL;}
			if(img_mon_12 && lv_obj_is_valid(img_mon_12)){beeper_start_tim(BEEP_TIME);lv_obj_del(img_mon_12); img_mon_12 = NULL;}
			if(img_mon_13 && lv_obj_is_valid(img_mon_13)){beeper_start_tim(BEEP_TIME);lv_obj_del(img_mon_13); img_mon_13 = NULL;}
			if(img_mon_14 && lv_obj_is_valid(img_mon_14)){beeper_start_tim(BEEP_TIME);lv_obj_del(img_mon_14); img_mon_14 = NULL;}
			if(img_mon_15 && lv_obj_is_valid(img_mon_15)){beeper_start_tim(BEEP_TIME);lv_obj_del(img_mon_15); img_mon_15 = NULL;}
			if(img_mon_16 && lv_obj_is_valid(img_mon_16)){beeper_start_tim(BEEP_TIME);lv_obj_del(img_mon_16); img_mon_16 = NULL;}
			break;
		}
		case 1:
		{
			if(beep_pow)beep_cout =1;
			if(img_mon_2 && lv_obj_is_valid(img_mon_2)){beeper_start_tim(BEEP_TIME);lv_obj_del(img_mon_2); img_mon_2 = NULL;}
			if(img_mon_3 && lv_obj_is_valid(img_mon_3)){beeper_start_tim(BEEP_TIME);lv_obj_del(img_mon_3); img_mon_3 = NULL;}
			if(img_mon_4 && lv_obj_is_valid(img_mon_4)){beeper_start_tim(BEEP_TIME);lv_obj_del(img_mon_4); img_mon_4 = NULL;}
			if(img_mon_5 && lv_obj_is_valid(img_mon_5)){beeper_start_tim(BEEP_TIME);lv_obj_del(img_mon_5); img_mon_5 = NULL;}
			if(img_mon_6 && lv_obj_is_valid(img_mon_6)){beeper_start_tim(BEEP_TIME);lv_obj_del(img_mon_6); img_mon_6 = NULL;}
			if(img_mon_7 && lv_obj_is_valid(img_mon_7)){beeper_start_tim(BEEP_TIME);lv_obj_del(img_mon_7); img_mon_7 = NULL;}
			if(img_mon_8 && lv_obj_is_valid(img_mon_8)){beeper_start_tim(BEEP_TIME);lv_obj_del(img_mon_8); img_mon_8 = NULL;}
			if(img_mon_9 && lv_obj_is_valid(img_mon_9)){beeper_start_tim(BEEP_TIME);lv_obj_del(img_mon_9); img_mon_9 = NULL;}
			if(img_mon_10 && lv_obj_is_valid(img_mon_10)){beeper_start_tim(BEEP_TIME);lv_obj_del(img_mon_10); img_mon_10 = NULL;}
			if(img_mon_11 && lv_obj_is_valid(img_mon_11)){beeper_start_tim(BEEP_TIME);lv_obj_del(img_mon_11); img_mon_11 = NULL;}
			if(img_mon_12 && lv_obj_is_valid(img_mon_12)){beeper_start_tim(BEEP_TIME);lv_obj_del(img_mon_12); img_mon_12 = NULL;}
			if(img_mon_13 && lv_obj_is_valid(img_mon_13)){beeper_start_tim(BEEP_TIME);lv_obj_del(img_mon_13); img_mon_13 = NULL;}
			if(img_mon_14 && lv_obj_is_valid(img_mon_14)){beeper_start_tim(BEEP_TIME);lv_obj_del(img_mon_14); img_mon_14 = NULL;}
			if(img_mon_15 && lv_obj_is_valid(img_mon_15)){beeper_start_tim(BEEP_TIME);lv_obj_del(img_mon_15); img_mon_15 = NULL;}
			if(img_mon_16 && lv_obj_is_valid(img_mon_16)){beeper_start_tim(BEEP_TIME);lv_obj_del(img_mon_16); img_mon_16 = NULL;}
			
			if(flag_t ==0){
				if(curr_data != data){
					stat_num =0;
					curr_data =data;
				}
				
				if(stat_num ++ < Stime){
					if(img_mon_1 == NULL){
						if(flag)beeper_start_tim(BEEP_TIME);
						img_mon_1 =lv_img_create(parent);
						lv_img_set_src(img_mon_1, mon_1);
						lv_obj_align(img_mon_1, LV_ALIGN_TOP_MID, 0, 220);
					}
				}else if(stat_num > 2*Stime){
					stat_num =0;
					if(img_mon_1 && lv_obj_is_valid(img_mon_1)){lv_obj_del(img_mon_1); img_mon_1 = NULL;}
				}
			}else{
				if(img_mon_1 == NULL){
					if(flag)beeper_start_tim(BEEP_TIME);
					img_mon_1 =lv_img_create(parent);
					lv_img_set_src(img_mon_1, mon_1);
					lv_obj_align(img_mon_1, LV_ALIGN_TOP_MID, 0, 220);
				}
			}
			break;
		}
		case 2:
		{
			if(beep_pow)beep_cout =2;
			if(img_mon_3 && lv_obj_is_valid(img_mon_3)){beeper_start_tim(BEEP_TIME);lv_obj_del(img_mon_3); img_mon_3 = NULL;}
			if(img_mon_4 && lv_obj_is_valid(img_mon_4)){beeper_start_tim(BEEP_TIME);lv_obj_del(img_mon_4); img_mon_4 = NULL;}
			if(img_mon_5 && lv_obj_is_valid(img_mon_5)){beeper_start_tim(BEEP_TIME);lv_obj_del(img_mon_5); img_mon_5 = NULL;}
			if(img_mon_6 && lv_obj_is_valid(img_mon_6)){beeper_start_tim(BEEP_TIME);lv_obj_del(img_mon_6); img_mon_6 = NULL;}
			if(img_mon_7 && lv_obj_is_valid(img_mon_7)){beeper_start_tim(BEEP_TIME);lv_obj_del(img_mon_7); img_mon_7 = NULL;}
			if(img_mon_8 && lv_obj_is_valid(img_mon_8)){beeper_start_tim(BEEP_TIME);lv_obj_del(img_mon_8); img_mon_8 = NULL;}
			if(img_mon_9 && lv_obj_is_valid(img_mon_9)){beeper_start_tim(BEEP_TIME);lv_obj_del(img_mon_9); img_mon_9 = NULL;}
			if(img_mon_10 && lv_obj_is_valid(img_mon_10)){beeper_start_tim(BEEP_TIME);lv_obj_del(img_mon_10); img_mon_10 = NULL;}
			if(img_mon_11 && lv_obj_is_valid(img_mon_11)){beeper_start_tim(BEEP_TIME);lv_obj_del(img_mon_11); img_mon_11 = NULL;}
			if(img_mon_12 && lv_obj_is_valid(img_mon_12)){beeper_start_tim(BEEP_TIME);lv_obj_del(img_mon_12); img_mon_12 = NULL;}
			if(img_mon_13 && lv_obj_is_valid(img_mon_13)){beeper_start_tim(BEEP_TIME);lv_obj_del(img_mon_13); img_mon_13 = NULL;}
			if(img_mon_14 && lv_obj_is_valid(img_mon_14)){beeper_start_tim(BEEP_TIME);lv_obj_del(img_mon_14); img_mon_14 = NULL;}
			if(img_mon_15 && lv_obj_is_valid(img_mon_15)){beeper_start_tim(BEEP_TIME);lv_obj_del(img_mon_15); img_mon_15 = NULL;}
			if(img_mon_16 && lv_obj_is_valid(img_mon_16)){beeper_start_tim(BEEP_TIME);lv_obj_del(img_mon_16); img_mon_16 = NULL;}
			
			if(flag_t ==0){
				if(curr_data != data){
					stat_num =0;
					curr_data =data;
				}
				if(stat_num ++ < Stime){
					if(img_mon_2 == NULL){
						if(flag)beeper_start_tim(BEEP_TIME);
						img_mon_2 =lv_img_create(parent);
						lv_img_set_src(img_mon_2, mon_1);
						lv_obj_align(img_mon_2, LV_ALIGN_TOP_MID, 0, 209);
					}
				}else if(stat_num > 2*Stime){
					stat_num =0;
					if(img_mon_2 && lv_obj_is_valid(img_mon_2)){lv_obj_del(img_mon_2); img_mon_2 = NULL;}
				}
			}else{
					if(img_mon_2 == NULL){
						if(flag)beeper_start_tim(BEEP_TIME);
						img_mon_2 =lv_img_create(parent);
						lv_img_set_src(img_mon_2, mon_1);
						lv_obj_align(img_mon_2, LV_ALIGN_TOP_MID, 0, 209);
					}
			}
			
			if(img_mon_1 == NULL){
				img_mon_1 =lv_img_create(parent);
				lv_img_set_src(img_mon_1, mon_1);
				lv_obj_align(img_mon_1, LV_ALIGN_TOP_MID, 0, 220);
			}
			break;
		}
		case 3:
		{
			if(beep_pow)beep_cout =3;
			if(img_mon_4 && lv_obj_is_valid(img_mon_4)){beeper_start_tim(BEEP_TIME);lv_obj_del(img_mon_4); img_mon_4 = NULL;}
			if(img_mon_5 && lv_obj_is_valid(img_mon_5)){beeper_start_tim(BEEP_TIME);lv_obj_del(img_mon_5); img_mon_5 = NULL;}
			if(img_mon_6 && lv_obj_is_valid(img_mon_6)){beeper_start_tim(BEEP_TIME);lv_obj_del(img_mon_6); img_mon_6 = NULL;}
			if(img_mon_7 && lv_obj_is_valid(img_mon_7)){beeper_start_tim(BEEP_TIME);lv_obj_del(img_mon_7); img_mon_7 = NULL;}
			if(img_mon_8 && lv_obj_is_valid(img_mon_8)){beeper_start_tim(BEEP_TIME);lv_obj_del(img_mon_8); img_mon_8 = NULL;}
			if(img_mon_9 && lv_obj_is_valid(img_mon_9)){beeper_start_tim(BEEP_TIME);lv_obj_del(img_mon_9); img_mon_9 = NULL;}
			if(img_mon_10 && lv_obj_is_valid(img_mon_10)){beeper_start_tim(BEEP_TIME);lv_obj_del(img_mon_10); img_mon_10 = NULL;}
			if(img_mon_11 && lv_obj_is_valid(img_mon_11)){beeper_start_tim(BEEP_TIME);lv_obj_del(img_mon_11); img_mon_11 = NULL;}
			if(img_mon_12 && lv_obj_is_valid(img_mon_12)){beeper_start_tim(BEEP_TIME);lv_obj_del(img_mon_12); img_mon_12 = NULL;}
			if(img_mon_13 && lv_obj_is_valid(img_mon_13)){beeper_start_tim(BEEP_TIME);lv_obj_del(img_mon_13); img_mon_13 = NULL;}
			if(img_mon_14 && lv_obj_is_valid(img_mon_14)){beeper_start_tim(BEEP_TIME);lv_obj_del(img_mon_14); img_mon_14 = NULL;}
			if(img_mon_15 && lv_obj_is_valid(img_mon_15)){beeper_start_tim(BEEP_TIME);lv_obj_del(img_mon_15); img_mon_15 = NULL;}
			if(img_mon_16 && lv_obj_is_valid(img_mon_16)){beeper_start_tim(BEEP_TIME);lv_obj_del(img_mon_16); img_mon_16 = NULL;}
			
			
			if(flag_t ==0){
				if(curr_data != data){
					stat_num =0;
					curr_data =data;
				}
				
				if(stat_num ++ < Stime){
					if(img_mon_3 == NULL){
						if(flag)beeper_start_tim(BEEP_TIME);
						img_mon_3 =lv_img_create(parent);
						lv_img_set_src(img_mon_3, mon_1);
						lv_obj_align(img_mon_3, LV_ALIGN_TOP_MID, 0, 198);
					}
				}else if(stat_num > 2*Stime){
					stat_num =0;
					if(img_mon_3 && lv_obj_is_valid(img_mon_3)){lv_obj_del(img_mon_3); img_mon_3 = NULL;}
				}
			}else{
					if(img_mon_3 == NULL){
						if(flag)beeper_start_tim(BEEP_TIME);
						img_mon_3 =lv_img_create(parent);
						lv_img_set_src(img_mon_3, mon_1);
						lv_obj_align(img_mon_3, LV_ALIGN_TOP_MID, 0, 198);
					}
			}

			if(img_mon_2 == NULL){
				img_mon_2 =lv_img_create(parent);
				lv_img_set_src(img_mon_2, mon_1);
				lv_obj_align(img_mon_2, LV_ALIGN_TOP_MID, 0, 209);
			}
			if(img_mon_1 == NULL){
				img_mon_1 =lv_img_create(parent);
				lv_img_set_src(img_mon_1, mon_1);
				lv_obj_align(img_mon_1, LV_ALIGN_TOP_MID, 0, 220);
			}
			break;
		}
		case 4:
		{
			if(beep_pow)beep_cout =4;
			if(img_mon_5 && lv_obj_is_valid(img_mon_5)){beeper_start_tim(BEEP_TIME);lv_obj_del(img_mon_5); img_mon_5 = NULL;}
			if(img_mon_6 && lv_obj_is_valid(img_mon_6)){beeper_start_tim(BEEP_TIME);lv_obj_del(img_mon_6); img_mon_6 = NULL;}
			if(img_mon_7 && lv_obj_is_valid(img_mon_7)){beeper_start_tim(BEEP_TIME);lv_obj_del(img_mon_7); img_mon_7 = NULL;}
			if(img_mon_8 && lv_obj_is_valid(img_mon_8)){beeper_start_tim(BEEP_TIME);lv_obj_del(img_mon_8); img_mon_8 = NULL;}
			if(img_mon_9 && lv_obj_is_valid(img_mon_9)){beeper_start_tim(BEEP_TIME);lv_obj_del(img_mon_9); img_mon_9 = NULL;}
			if(img_mon_10 && lv_obj_is_valid(img_mon_10)){beeper_start_tim(BEEP_TIME);lv_obj_del(img_mon_10); img_mon_10 = NULL;}
			if(img_mon_11 && lv_obj_is_valid(img_mon_11)){beeper_start_tim(BEEP_TIME);lv_obj_del(img_mon_11); img_mon_11 = NULL;}
			if(img_mon_12 && lv_obj_is_valid(img_mon_12)){beeper_start_tim(BEEP_TIME);lv_obj_del(img_mon_12); img_mon_12 = NULL;}
			if(img_mon_13 && lv_obj_is_valid(img_mon_13)){beeper_start_tim(BEEP_TIME);lv_obj_del(img_mon_13); img_mon_13 = NULL;}
			if(img_mon_14 && lv_obj_is_valid(img_mon_14)){beeper_start_tim(BEEP_TIME);lv_obj_del(img_mon_14); img_mon_14 = NULL;}
			if(img_mon_15 && lv_obj_is_valid(img_mon_15)){beeper_start_tim(BEEP_TIME);lv_obj_del(img_mon_15); img_mon_15 = NULL;}
			if(img_mon_16 && lv_obj_is_valid(img_mon_16)){beeper_start_tim(BEEP_TIME);lv_obj_del(img_mon_16); img_mon_16 = NULL;}
			
			if(flag_t ==0){
				if(curr_data != data){
					stat_num =0;
					curr_data =data;
				}
				if(stat_num ++ < Stime){
					if(img_mon_4 == NULL){
						if(flag)beeper_start_tim(BEEP_TIME);
						img_mon_4 =lv_img_create(parent);
						lv_img_set_src(img_mon_4, mon_1);
						lv_obj_align(img_mon_4, LV_ALIGN_TOP_MID, 0, 187);
					}
				}else if(stat_num > 2*Stime){
					stat_num =0;
					if(img_mon_4 && lv_obj_is_valid(img_mon_4)){lv_obj_del(img_mon_4); img_mon_4 = NULL;}
				}
			}else{
					if(img_mon_4 == NULL){
						if(flag)beeper_start_tim(BEEP_TIME);
						img_mon_4 =lv_img_create(parent);
						lv_img_set_src(img_mon_4, mon_1);
						lv_obj_align(img_mon_4, LV_ALIGN_TOP_MID, 0, 187);
					}
			}
			
			if(img_mon_3 == NULL){
				img_mon_3 =lv_img_create(parent);
				lv_img_set_src(img_mon_3, mon_1);
				lv_obj_align(img_mon_3, LV_ALIGN_TOP_MID, 0, 198);
			}
			if(img_mon_2 == NULL){
				img_mon_2 =lv_img_create(parent);
				lv_img_set_src(img_mon_2, mon_1);
				lv_obj_align(img_mon_2, LV_ALIGN_TOP_MID, 0, 209);
			}
			if(img_mon_1 == NULL){
				img_mon_1 =lv_img_create(parent);
				lv_img_set_src(img_mon_1, mon_1);
				lv_obj_align(img_mon_1, LV_ALIGN_TOP_MID, 0, 220);
			}
			break;
		}
		case 5:
		{
			if(beep_pow)beep_cout =5;
			if(img_mon_6 && lv_obj_is_valid(img_mon_6)){beeper_start_tim(BEEP_TIME);lv_obj_del(img_mon_6); img_mon_6 = NULL;}
			if(img_mon_7 && lv_obj_is_valid(img_mon_7)){beeper_start_tim(BEEP_TIME);lv_obj_del(img_mon_7); img_mon_7 = NULL;}
			if(img_mon_8 && lv_obj_is_valid(img_mon_8)){beeper_start_tim(BEEP_TIME);lv_obj_del(img_mon_8); img_mon_8 = NULL;}
			if(img_mon_9 && lv_obj_is_valid(img_mon_9)){beeper_start_tim(BEEP_TIME);lv_obj_del(img_mon_9); img_mon_9 = NULL;}
			if(img_mon_10 && lv_obj_is_valid(img_mon_10)){beeper_start_tim(BEEP_TIME);lv_obj_del(img_mon_10); img_mon_10 = NULL;}
			if(img_mon_11 && lv_obj_is_valid(img_mon_11)){beeper_start_tim(BEEP_TIME);lv_obj_del(img_mon_11); img_mon_11 = NULL;}
			if(img_mon_12 && lv_obj_is_valid(img_mon_12)){beeper_start_tim(BEEP_TIME);lv_obj_del(img_mon_12); img_mon_12 = NULL;}
			if(img_mon_13 && lv_obj_is_valid(img_mon_13)){beeper_start_tim(BEEP_TIME);lv_obj_del(img_mon_13); img_mon_13 = NULL;}
			if(img_mon_14 && lv_obj_is_valid(img_mon_14)){beeper_start_tim(BEEP_TIME);lv_obj_del(img_mon_14); img_mon_14 = NULL;}
			if(img_mon_15 && lv_obj_is_valid(img_mon_15)){beeper_start_tim(BEEP_TIME);lv_obj_del(img_mon_15); img_mon_15 = NULL;}
			if(img_mon_16 && lv_obj_is_valid(img_mon_16)){beeper_start_tim(BEEP_TIME);lv_obj_del(img_mon_16); img_mon_16 = NULL;}
			
			if(flag_t ==0){
				if(curr_data != data){
					stat_num =0;
					curr_data =data;
				}
				if(stat_num++ < Stime){
					if(img_mon_5 == NULL){
						if(flag)beeper_start_tim(BEEP_TIME);
						img_mon_5 =lv_img_create(parent);
						lv_img_set_src(img_mon_5, mon_1);
						lv_obj_align(img_mon_5, LV_ALIGN_TOP_MID, 0, 177);
					}
				}else if(stat_num > 2*Stime){
					stat_num =0;
					if(img_mon_5 && lv_obj_is_valid(img_mon_5)){lv_obj_del(img_mon_5); img_mon_5 = NULL;}
				}
			}else{
					if(img_mon_5 == NULL){
						if(flag)beeper_start_tim(BEEP_TIME);
						img_mon_5 =lv_img_create(parent);
						lv_img_set_src(img_mon_5, mon_1);
						lv_obj_align(img_mon_5, LV_ALIGN_TOP_MID, 0, 177);
					}
			}
			
			if(img_mon_4 == NULL){
				img_mon_4 =lv_img_create(parent);
				lv_img_set_src(img_mon_4, mon_1);
				lv_obj_align(img_mon_4, LV_ALIGN_TOP_MID, 0, 187);
			}
			if(img_mon_3 == NULL){
				img_mon_3 =lv_img_create(parent);
				lv_img_set_src(img_mon_3, mon_1);
				lv_obj_align(img_mon_3, LV_ALIGN_TOP_MID, 0, 198);
			}
			if(img_mon_2 == NULL){
				img_mon_2 =lv_img_create(parent);
				lv_img_set_src(img_mon_2, mon_1);
				lv_obj_align(img_mon_2, LV_ALIGN_TOP_MID, 0, 209);
			}
			if(img_mon_1 == NULL){
				img_mon_1 =lv_img_create(parent);
				lv_img_set_src(img_mon_1, mon_1);
				lv_obj_align(img_mon_1, LV_ALIGN_TOP_MID, 0, 220);
			}
			break;
		}
		case 6:
		{
			if(beep_pow)beep_cout =6;
			if(img_mon_7 && lv_obj_is_valid(img_mon_7)){beeper_start_tim(BEEP_TIME);lv_obj_del(img_mon_7); img_mon_7 = NULL;}
			if(img_mon_8 && lv_obj_is_valid(img_mon_8)){beeper_start_tim(BEEP_TIME);lv_obj_del(img_mon_8); img_mon_8 = NULL;}
			if(img_mon_9 && lv_obj_is_valid(img_mon_9)){beeper_start_tim(BEEP_TIME);lv_obj_del(img_mon_9); img_mon_9 = NULL;}
			if(img_mon_10 && lv_obj_is_valid(img_mon_10)){beeper_start_tim(BEEP_TIME);lv_obj_del(img_mon_10); img_mon_10 = NULL;}
			if(img_mon_11 && lv_obj_is_valid(img_mon_11)){beeper_start_tim(BEEP_TIME);lv_obj_del(img_mon_11); img_mon_11 = NULL;}
			if(img_mon_12 && lv_obj_is_valid(img_mon_12)){beeper_start_tim(BEEP_TIME);lv_obj_del(img_mon_12); img_mon_12 = NULL;}
			if(img_mon_13 && lv_obj_is_valid(img_mon_13)){beeper_start_tim(BEEP_TIME);lv_obj_del(img_mon_13); img_mon_13 = NULL;}
			if(img_mon_14 && lv_obj_is_valid(img_mon_14)){beeper_start_tim(BEEP_TIME);lv_obj_del(img_mon_14); img_mon_14 = NULL;}
			if(img_mon_15 && lv_obj_is_valid(img_mon_15)){beeper_start_tim(BEEP_TIME);lv_obj_del(img_mon_15); img_mon_15 = NULL;}
			if(img_mon_16 && lv_obj_is_valid(img_mon_16)){beeper_start_tim(BEEP_TIME);lv_obj_del(img_mon_16); img_mon_16 = NULL;}
			
			if(flag_t ==0){
				if(curr_data != data){
					stat_num =0;
					curr_data =data;
				}
				
				if(stat_num ++ < Stime){
					if(img_mon_6 == NULL){
						if(flag)beeper_start_tim(BEEP_TIME);
						img_mon_6 =lv_img_create(parent);
						lv_img_set_src(img_mon_6, mon_1);
						lv_obj_align(img_mon_6, LV_ALIGN_TOP_MID, 0, 166);
					}
				}else if(stat_num > 2*Stime){
					stat_num =0;
					if(img_mon_6 && lv_obj_is_valid(img_mon_6)){lv_obj_del(img_mon_6); img_mon_6 = NULL;}
				}
			}else{
					if(img_mon_6 == NULL){
						if(flag)beeper_start_tim(BEEP_TIME);
						img_mon_6 =lv_img_create(parent);
						lv_img_set_src(img_mon_6, mon_1);
						lv_obj_align(img_mon_6, LV_ALIGN_TOP_MID, 0, 166);
					}
			}
			
			if(img_mon_5 == NULL){
				img_mon_5 =lv_img_create(parent);
				lv_img_set_src(img_mon_5, mon_1);
				lv_obj_align(img_mon_5, LV_ALIGN_TOP_MID, 0, 177);
			}
			if(img_mon_4 == NULL){
				img_mon_4 =lv_img_create(parent);
				lv_img_set_src(img_mon_4, mon_1);
				lv_obj_align(img_mon_4, LV_ALIGN_TOP_MID, 0, 187);
			}
			if(img_mon_3 == NULL){
				img_mon_3 =lv_img_create(parent);
				lv_img_set_src(img_mon_3, mon_1);
				lv_obj_align(img_mon_3, LV_ALIGN_TOP_MID, 0, 198);
			}
			if(img_mon_2 == NULL){
				img_mon_2 =lv_img_create(parent);
				lv_img_set_src(img_mon_2, mon_1);
				lv_obj_align(img_mon_2, LV_ALIGN_TOP_MID, 0, 209);
			}
			if(img_mon_1 == NULL){
				img_mon_1 =lv_img_create(parent);
				lv_img_set_src(img_mon_1, mon_1);
				lv_obj_align(img_mon_1, LV_ALIGN_TOP_MID, 0, 220);
			}
			break;
		}
	case 7:
		{
			if(beep_pow)beep_cout =7;
			if(img_mon_8 && lv_obj_is_valid(img_mon_8)){beeper_start_tim(BEEP_TIME);lv_obj_del(img_mon_8); img_mon_8 = NULL;}
			if(img_mon_9 && lv_obj_is_valid(img_mon_9)){beeper_start_tim(BEEP_TIME);lv_obj_del(img_mon_9); img_mon_9 = NULL;}
			if(img_mon_10 && lv_obj_is_valid(img_mon_10)){beeper_start_tim(BEEP_TIME);lv_obj_del(img_mon_10); img_mon_10 = NULL;}
			if(img_mon_11 && lv_obj_is_valid(img_mon_11)){beeper_start_tim(BEEP_TIME);lv_obj_del(img_mon_11); img_mon_11 = NULL;}
			if(img_mon_12 && lv_obj_is_valid(img_mon_12)){beeper_start_tim(BEEP_TIME);lv_obj_del(img_mon_12); img_mon_12 = NULL;}
			if(img_mon_13 && lv_obj_is_valid(img_mon_13)){beeper_start_tim(BEEP_TIME);lv_obj_del(img_mon_13); img_mon_13 = NULL;}
			if(img_mon_14 && lv_obj_is_valid(img_mon_14)){beeper_start_tim(BEEP_TIME);lv_obj_del(img_mon_14); img_mon_14 = NULL;}
			if(img_mon_15 && lv_obj_is_valid(img_mon_15)){beeper_start_tim(BEEP_TIME);lv_obj_del(img_mon_15); img_mon_15 = NULL;}
			if(img_mon_16 && lv_obj_is_valid(img_mon_16)){beeper_start_tim(BEEP_TIME);lv_obj_del(img_mon_16); img_mon_16 = NULL;}
			
			if(flag_t ==0){
				if(curr_data != data){
					stat_num =0;
					curr_data =data;
				}
				if(stat_num ++ < Stime){
					if(img_mon_7 == NULL){
						if(flag)beeper_start_tim(BEEP_TIME);
						img_mon_7 =lv_img_create(parent);
						lv_img_set_src(img_mon_7, mon_1);
						lv_obj_align(img_mon_7, LV_ALIGN_TOP_MID, 0, 156);
					}
				}else if(stat_num > 2*Stime){
					stat_num =0;
					if(img_mon_7 && lv_obj_is_valid(img_mon_7)){lv_obj_del(img_mon_7); img_mon_7 = NULL;}
				}
			}else{
					if(img_mon_7 == NULL){
						if(flag)beeper_start_tim(BEEP_TIME);
						img_mon_7 =lv_img_create(parent);
						lv_img_set_src(img_mon_7, mon_1);
						lv_obj_align(img_mon_7, LV_ALIGN_TOP_MID, 0, 156);
					}
			}
			
			if(img_mon_6 == NULL){
				img_mon_6 =lv_img_create(parent);
				lv_img_set_src(img_mon_6, mon_1);
				lv_obj_align(img_mon_6, LV_ALIGN_TOP_MID, 0, 166);
			}
			if(img_mon_5 == NULL){
				img_mon_5 =lv_img_create(parent);
				lv_img_set_src(img_mon_5, mon_1);
				lv_obj_align(img_mon_5, LV_ALIGN_TOP_MID, 0, 177);
			}
			if(img_mon_4 == NULL){
				img_mon_4 =lv_img_create(parent);
				lv_img_set_src(img_mon_4, mon_1);
				lv_obj_align(img_mon_4, LV_ALIGN_TOP_MID, 0, 187);
			}
			if(img_mon_3 == NULL){
				img_mon_3 =lv_img_create(parent);
				lv_img_set_src(img_mon_3, mon_1);
				lv_obj_align(img_mon_3, LV_ALIGN_TOP_MID, 0, 198);
			}
			if(img_mon_2 == NULL){
				img_mon_2 =lv_img_create(parent);
				lv_img_set_src(img_mon_2, mon_1);
				lv_obj_align(img_mon_2, LV_ALIGN_TOP_MID, 0, 209);
			}
			if(img_mon_1 == NULL){
				img_mon_1 =lv_img_create(parent);
				lv_img_set_src(img_mon_1, mon_1);
				lv_obj_align(img_mon_1, LV_ALIGN_TOP_MID, 0, 220);
			}
			break;
		}
	case 8:
		{
			if(beep_pow)beep_cout =8;
			if(img_mon_9 && lv_obj_is_valid(img_mon_9)){beeper_start_tim(BEEP_TIME);lv_obj_del(img_mon_9); img_mon_9 = NULL;}
			if(img_mon_10 && lv_obj_is_valid(img_mon_10)){beeper_start_tim(BEEP_TIME);lv_obj_del(img_mon_10); img_mon_10 = NULL;}
			if(img_mon_11 && lv_obj_is_valid(img_mon_11)){beeper_start_tim(BEEP_TIME);lv_obj_del(img_mon_11); img_mon_11 = NULL;}
			if(img_mon_12 && lv_obj_is_valid(img_mon_12)){beeper_start_tim(BEEP_TIME);lv_obj_del(img_mon_12); img_mon_12 = NULL;}
			if(img_mon_13 && lv_obj_is_valid(img_mon_13)){beeper_start_tim(BEEP_TIME);lv_obj_del(img_mon_13); img_mon_13 = NULL;}
			if(img_mon_14 && lv_obj_is_valid(img_mon_14)){beeper_start_tim(BEEP_TIME);lv_obj_del(img_mon_14); img_mon_14 = NULL;}
			if(img_mon_15 && lv_obj_is_valid(img_mon_15)){beeper_start_tim(BEEP_TIME);lv_obj_del(img_mon_15); img_mon_15 = NULL;}
			if(img_mon_16 && lv_obj_is_valid(img_mon_16)){beeper_start_tim(BEEP_TIME);lv_obj_del(img_mon_16); img_mon_16 = NULL;}
			if(flag_t ==0){
				if(curr_data != data){
					stat_num =0;
					curr_data =data;
				}
				if(stat_num ++ < Stime){
					if(img_mon_8 == NULL){
						if(flag)beeper_start_tim(BEEP_TIME);
						img_mon_8 =lv_img_create(parent);
						lv_img_set_src(img_mon_8, mon_1);
						lv_obj_align(img_mon_8, LV_ALIGN_TOP_MID, 0, 146);
					}
				}else if(stat_num > 2*Stime){
					stat_num =0;
					if(img_mon_8 && lv_obj_is_valid(img_mon_8)){lv_obj_del(img_mon_8); img_mon_8 = NULL;}
				}
			}else{
					if(img_mon_8 == NULL){
						if(flag)beeper_start_tim(BEEP_TIME);
						img_mon_8 =lv_img_create(parent);
						lv_img_set_src(img_mon_8, mon_1);
						lv_obj_align(img_mon_8, LV_ALIGN_TOP_MID, 0, 146);
					}
			}
			if(img_mon_7 == NULL){
				img_mon_7 =lv_img_create(parent);
				lv_img_set_src(img_mon_7, mon_1);
				lv_obj_align(img_mon_7, LV_ALIGN_TOP_MID, 0, 156);
			}
			if(img_mon_6 == NULL){
				img_mon_6 =lv_img_create(parent);
				lv_img_set_src(img_mon_6, mon_1);
				lv_obj_align(img_mon_6, LV_ALIGN_TOP_MID, 0, 166);
			}
			if(img_mon_5 == NULL){
				img_mon_5 =lv_img_create(parent);
				lv_img_set_src(img_mon_5, mon_1);
				lv_obj_align(img_mon_5, LV_ALIGN_TOP_MID, 0, 177);
			}
			if(img_mon_4 == NULL){
				img_mon_4 =lv_img_create(parent);
				lv_img_set_src(img_mon_4, mon_1);
				lv_obj_align(img_mon_4, LV_ALIGN_TOP_MID, 0, 187);
			}
			if(img_mon_3 == NULL){
				img_mon_3 =lv_img_create(parent);
				lv_img_set_src(img_mon_3, mon_1);
				lv_obj_align(img_mon_3, LV_ALIGN_TOP_MID, 0, 198);
			}
			if(img_mon_2 == NULL){
				img_mon_2 =lv_img_create(parent);
				lv_img_set_src(img_mon_2, mon_1);
				lv_obj_align(img_mon_2, LV_ALIGN_TOP_MID, 0, 209);
			}
			if(img_mon_1 == NULL){
				img_mon_1 =lv_img_create(parent);
				lv_img_set_src(img_mon_1, mon_1);
				lv_obj_align(img_mon_1, LV_ALIGN_TOP_MID, 0, 220);
			}
			break;
		}
	case 9:
		{
			if(beep_pow)beep_cout =9;
			if(img_mon_10 && lv_obj_is_valid(img_mon_10)){beeper_start_tim(BEEP_TIME);lv_obj_del(img_mon_10); img_mon_10 = NULL;}
			if(img_mon_11 && lv_obj_is_valid(img_mon_11)){beeper_start_tim(BEEP_TIME);lv_obj_del(img_mon_11); img_mon_11 = NULL;}
			if(img_mon_12 && lv_obj_is_valid(img_mon_12)){beeper_start_tim(BEEP_TIME);lv_obj_del(img_mon_12); img_mon_12 = NULL;}
			if(img_mon_13 && lv_obj_is_valid(img_mon_13)){beeper_start_tim(BEEP_TIME);lv_obj_del(img_mon_13); img_mon_13 = NULL;}
			if(img_mon_14 && lv_obj_is_valid(img_mon_14)){beeper_start_tim(BEEP_TIME);lv_obj_del(img_mon_14); img_mon_14 = NULL;}
			if(img_mon_15 && lv_obj_is_valid(img_mon_15)){beeper_start_tim(BEEP_TIME);lv_obj_del(img_mon_15); img_mon_15 = NULL;}
			if(img_mon_16 && lv_obj_is_valid(img_mon_16)){beeper_start_tim(BEEP_TIME);lv_obj_del(img_mon_16); img_mon_16 = NULL;}
			
			if(flag_t ==0){
				if(curr_data != data){
					stat_num =0;
					curr_data =data;
				}
				if(stat_num ++ < Stime){
					if(img_mon_9 == NULL){
						if(flag)beeper_start_tim(BEEP_TIME);
						img_mon_9 =lv_img_create(parent);
						lv_img_set_src(img_mon_9, mon_1);
						lv_obj_align(img_mon_9, LV_ALIGN_TOP_MID, 0, 135);
					}
				}else if(stat_num > 2*Stime){
					stat_num =0;
					if(img_mon_9 && lv_obj_is_valid(img_mon_9)){lv_obj_del(img_mon_9); img_mon_9 = NULL;}
				}
			}else{
					if(img_mon_9 == NULL){
						if(flag)beeper_start_tim(BEEP_TIME);
						img_mon_9 =lv_img_create(parent);
						lv_img_set_src(img_mon_9, mon_1);
						lv_obj_align(img_mon_9, LV_ALIGN_TOP_MID, 0, 135);
					}
			}
			
			if(img_mon_8 == NULL){
				img_mon_8 =lv_img_create(parent);
				lv_img_set_src(img_mon_8, mon_1);
				lv_obj_align(img_mon_8, LV_ALIGN_TOP_MID, 0, 146);
			}
			if(img_mon_7 == NULL){
				img_mon_7 =lv_img_create(parent);
				lv_img_set_src(img_mon_7, mon_1);
				lv_obj_align(img_mon_7, LV_ALIGN_TOP_MID, 0, 156);
			}
			if(img_mon_6 == NULL){
				img_mon_6 =lv_img_create(parent);
				lv_img_set_src(img_mon_6, mon_1);
				lv_obj_align(img_mon_6, LV_ALIGN_TOP_MID, 0, 166);
			}
			if(img_mon_5 == NULL){
				img_mon_5 =lv_img_create(parent);
				lv_img_set_src(img_mon_5, mon_1);
				lv_obj_align(img_mon_5, LV_ALIGN_TOP_MID, 0, 177);
			}
			if(img_mon_4 == NULL){
				img_mon_4 =lv_img_create(parent);
				lv_img_set_src(img_mon_4, mon_1);
				lv_obj_align(img_mon_4, LV_ALIGN_TOP_MID, 0, 187);
			}
			if(img_mon_3 == NULL){
				img_mon_3 =lv_img_create(parent);
				lv_img_set_src(img_mon_3, mon_1);
				lv_obj_align(img_mon_3, LV_ALIGN_TOP_MID, 0, 198);
			}
			if(img_mon_2 == NULL){
				img_mon_2 =lv_img_create(parent);
				lv_img_set_src(img_mon_2, mon_1);
				lv_obj_align(img_mon_2, LV_ALIGN_TOP_MID, 0, 209);
			}
			if(img_mon_1 == NULL){
				img_mon_1 =lv_img_create(parent);
				lv_img_set_src(img_mon_1, mon_1);
				lv_obj_align(img_mon_1, LV_ALIGN_TOP_MID, 0, 220);
			}
			break;
		}
	case 10:
		{
			if(beep_pow)beep_cout =10;
			if(img_mon_11 && lv_obj_is_valid(img_mon_11)){beeper_start_tim(BEEP_TIME);lv_obj_del(img_mon_11); img_mon_11 = NULL;}
			if(img_mon_12 && lv_obj_is_valid(img_mon_12)){beeper_start_tim(BEEP_TIME);lv_obj_del(img_mon_12); img_mon_12 = NULL;}
			if(img_mon_13 && lv_obj_is_valid(img_mon_13)){beeper_start_tim(BEEP_TIME);lv_obj_del(img_mon_13); img_mon_13 = NULL;}
			if(img_mon_14 && lv_obj_is_valid(img_mon_14)){beeper_start_tim(BEEP_TIME);lv_obj_del(img_mon_14); img_mon_14 = NULL;}
			if(img_mon_15 && lv_obj_is_valid(img_mon_15)){beeper_start_tim(BEEP_TIME);lv_obj_del(img_mon_15); img_mon_15 = NULL;}
			if(img_mon_16 && lv_obj_is_valid(img_mon_16)){beeper_start_tim(BEEP_TIME);lv_obj_del(img_mon_16); img_mon_16 = NULL;}
			
			if(flag_t ==0){
				if(curr_data != data){
					stat_num =0;
					curr_data =data;
				}
				if(stat_num ++ < Stime){
					if(img_mon_10 == NULL){
						if(flag)beeper_start_tim(BEEP_TIME);
						img_mon_10 =lv_img_create(parent);
						lv_img_set_src(img_mon_10, mon_1);
						lv_obj_align(img_mon_10, LV_ALIGN_TOP_MID, 0, 123);
					}
				}else if(stat_num > 2*Stime){
					stat_num =0;
					if(img_mon_10 && lv_obj_is_valid(img_mon_10)){lv_obj_del(img_mon_10); img_mon_10 = NULL;}
				}
			}else{
					if(img_mon_10 == NULL){
						if(flag)beeper_start_tim(BEEP_TIME);
						img_mon_10 =lv_img_create(parent);
						lv_img_set_src(img_mon_10, mon_1);
						lv_obj_align(img_mon_10, LV_ALIGN_TOP_MID, 0, 123);
					}
			}
				
				
			if(img_mon_9 == NULL){
				img_mon_9 =lv_img_create(parent);
				lv_img_set_src(img_mon_9, mon_1);
				lv_obj_align(img_mon_9, LV_ALIGN_TOP_MID, 0, 135);
			}
			if(img_mon_8 == NULL){
				img_mon_8 =lv_img_create(parent);
				lv_img_set_src(img_mon_8, mon_1);
				lv_obj_align(img_mon_8, LV_ALIGN_TOP_MID, 0, 146);
			}
			if(img_mon_7 == NULL){
				img_mon_7 =lv_img_create(parent);
				lv_img_set_src(img_mon_7, mon_1);
				lv_obj_align(img_mon_7, LV_ALIGN_TOP_MID, 0, 156);
			}
			if(img_mon_6 == NULL){
				img_mon_6 =lv_img_create(parent);
				lv_img_set_src(img_mon_6, mon_1);
				lv_obj_align(img_mon_6, LV_ALIGN_TOP_MID, 0, 166);
			}
			if(img_mon_5 == NULL){
				img_mon_5 =lv_img_create(parent);
				lv_img_set_src(img_mon_5, mon_1);
				lv_obj_align(img_mon_5, LV_ALIGN_TOP_MID, 0, 177);
			}
			if(img_mon_4 == NULL){
				img_mon_4 =lv_img_create(parent);
				lv_img_set_src(img_mon_4, mon_1);
				lv_obj_align(img_mon_4, LV_ALIGN_TOP_MID, 0, 187);
			}
			if(img_mon_3 == NULL){
				img_mon_3 =lv_img_create(parent);
				lv_img_set_src(img_mon_3, mon_1);
				lv_obj_align(img_mon_3, LV_ALIGN_TOP_MID, 0, 198);
			}
			if(img_mon_2 == NULL){
				img_mon_2 =lv_img_create(parent);
				lv_img_set_src(img_mon_2, mon_1);
				lv_obj_align(img_mon_2, LV_ALIGN_TOP_MID, 0, 209);
			}
			if(img_mon_1 == NULL){
				img_mon_1 =lv_img_create(parent);
				lv_img_set_src(img_mon_1, mon_1);
				lv_obj_align(img_mon_1, LV_ALIGN_TOP_MID, 0, 220);
			}
			break;
		}
	case 11:
		{
			if(beep_pow)beep_cout =11;
			if(img_mon_12 && lv_obj_is_valid(img_mon_12)){beeper_start_tim(BEEP_TIME);lv_obj_del(img_mon_12); img_mon_12 = NULL;}
			if(img_mon_13 && lv_obj_is_valid(img_mon_13)){beeper_start_tim(BEEP_TIME);lv_obj_del(img_mon_13); img_mon_13 = NULL;}
			if(img_mon_14 && lv_obj_is_valid(img_mon_14)){beeper_start_tim(BEEP_TIME);lv_obj_del(img_mon_14); img_mon_14 = NULL;}
			if(img_mon_15 && lv_obj_is_valid(img_mon_15)){beeper_start_tim(BEEP_TIME);lv_obj_del(img_mon_15); img_mon_15 = NULL;}
			if(img_mon_16 && lv_obj_is_valid(img_mon_16)){beeper_start_tim(BEEP_TIME);lv_obj_del(img_mon_16); img_mon_16 = NULL;}
			
			if(flag_t ==0){
				if(curr_data != data){
					stat_num =0;
					curr_data =data;
				}
				if(stat_num ++ < Stime){
					if(img_mon_11 == NULL){
						if(flag)beeper_start_tim(BEEP_TIME);
						img_mon_11 =lv_img_create(parent);
						lv_img_set_src(img_mon_11, mon_1);
						lv_obj_align(img_mon_11, LV_ALIGN_TOP_MID, 0, 113);
					}
				}else if(stat_num > 2*Stime){
					stat_num =0;
					if(img_mon_11 && lv_obj_is_valid(img_mon_11)){lv_obj_del(img_mon_11); img_mon_11 = NULL;}
				}
			}else{
					if(img_mon_11 == NULL){
						if(flag)beeper_start_tim(BEEP_TIME);
						img_mon_11 =lv_img_create(parent);
						lv_img_set_src(img_mon_11, mon_1);
						lv_obj_align(img_mon_11, LV_ALIGN_TOP_MID, 0, 113);
					}
			}
			
			if(img_mon_10 == NULL){
				img_mon_10 =lv_img_create(parent);
				lv_img_set_src(img_mon_10, mon_1);
				lv_obj_align(img_mon_10, LV_ALIGN_TOP_MID, 0, 123);
			}
			if(img_mon_9 == NULL){
				img_mon_9 =lv_img_create(parent);
				lv_img_set_src(img_mon_9, mon_1);
				lv_obj_align(img_mon_9, LV_ALIGN_TOP_MID, 0, 135);
			}
			if(img_mon_8 == NULL){
				img_mon_8 =lv_img_create(parent);
				lv_img_set_src(img_mon_8, mon_1);
				lv_obj_align(img_mon_8, LV_ALIGN_TOP_MID, 0, 146);
			}
			if(img_mon_7 == NULL){
				img_mon_7 =lv_img_create(parent);
				lv_img_set_src(img_mon_7, mon_1);
				lv_obj_align(img_mon_7, LV_ALIGN_TOP_MID, 0, 156);
			}
			if(img_mon_6 == NULL){
				img_mon_6 =lv_img_create(parent);
				lv_img_set_src(img_mon_6, mon_1);
				lv_obj_align(img_mon_6, LV_ALIGN_TOP_MID, 0, 166);
			}
			if(img_mon_5 == NULL){
				img_mon_5 =lv_img_create(parent);
				lv_img_set_src(img_mon_5, mon_1);
				lv_obj_align(img_mon_5, LV_ALIGN_TOP_MID, 0, 177);
			}
			if(img_mon_4 == NULL){
				img_mon_4 =lv_img_create(parent);
				lv_img_set_src(img_mon_4, mon_1);
				lv_obj_align(img_mon_4, LV_ALIGN_TOP_MID, 0, 187);
			}
			if(img_mon_3 == NULL){
				img_mon_3 =lv_img_create(parent);
				lv_img_set_src(img_mon_3, mon_1);
				lv_obj_align(img_mon_3, LV_ALIGN_TOP_MID, 0, 198);
			}
			if(img_mon_2 == NULL){
				img_mon_2 =lv_img_create(parent);
				lv_img_set_src(img_mon_2, mon_1);
				lv_obj_align(img_mon_2, LV_ALIGN_TOP_MID, 0, 209);
			}
			if(img_mon_1 == NULL){
				img_mon_1 =lv_img_create(parent);
				lv_img_set_src(img_mon_1, mon_1);
				lv_obj_align(img_mon_1, LV_ALIGN_TOP_MID, 0, 220);
			}
			break;
		}
	case 12:
		{
			if(beep_pow)beep_cout =12;
			if(img_mon_13 && lv_obj_is_valid(img_mon_13)){beeper_start_tim(BEEP_TIME);lv_obj_del(img_mon_13); img_mon_13 = NULL;}
			if(img_mon_14 && lv_obj_is_valid(img_mon_14)){beeper_start_tim(BEEP_TIME);lv_obj_del(img_mon_14); img_mon_14 = NULL;}
			if(img_mon_15 && lv_obj_is_valid(img_mon_15)){beeper_start_tim(BEEP_TIME);lv_obj_del(img_mon_15); img_mon_15 = NULL;}
			if(img_mon_16 && lv_obj_is_valid(img_mon_16)){beeper_start_tim(BEEP_TIME);lv_obj_del(img_mon_16); img_mon_16 = NULL;}
			
			if(flag_t ==0){
				if(curr_data != data){
					stat_num =0;
					curr_data =data;
				}
				if(stat_num ++ < Stime){
					if(img_mon_12 == NULL){
						if(flag)beeper_start_tim(BEEP_TIME);
						img_mon_12 =lv_img_create(parent);
						lv_img_set_src(img_mon_12, mon_1);
						lv_obj_align(img_mon_12, LV_ALIGN_TOP_MID, 0, 103);
					}
				}else if(stat_num > 2*Stime){
					stat_num =0;
					if(img_mon_12 && lv_obj_is_valid(img_mon_12)){lv_obj_del(img_mon_12); img_mon_12 = NULL;}
				}
			}else{
					if(img_mon_12 == NULL){
						if(flag)beeper_start_tim(BEEP_TIME);
						img_mon_12 =lv_img_create(parent);
						lv_img_set_src(img_mon_12, mon_1);
						lv_obj_align(img_mon_12, LV_ALIGN_TOP_MID, 0, 103);
					}
			}
			
			if(img_mon_11 == NULL){
				img_mon_11 =lv_img_create(parent);
				lv_img_set_src(img_mon_11, mon_1);
				lv_obj_align(img_mon_11, LV_ALIGN_TOP_MID, 0, 113);
			}
			if(img_mon_10 == NULL){
				img_mon_10 =lv_img_create(parent);
				lv_img_set_src(img_mon_10, mon_1);
				lv_obj_align(img_mon_10, LV_ALIGN_TOP_MID, 0, 123);
			}
			if(img_mon_9 == NULL){
				img_mon_9 =lv_img_create(parent);
				lv_img_set_src(img_mon_9, mon_1);
				lv_obj_align(img_mon_9, LV_ALIGN_TOP_MID, 0, 135);
			}
			if(img_mon_8 == NULL){
				img_mon_8 =lv_img_create(parent);
				lv_img_set_src(img_mon_8, mon_1);
				lv_obj_align(img_mon_8, LV_ALIGN_TOP_MID, 0, 146);
			}
			if(img_mon_7 == NULL){
				img_mon_7 =lv_img_create(parent);
				lv_img_set_src(img_mon_7, mon_1);
				lv_obj_align(img_mon_7, LV_ALIGN_TOP_MID, 0, 156);
			}
			if(img_mon_6 == NULL){
				img_mon_6 =lv_img_create(parent);
				lv_img_set_src(img_mon_6, mon_1);
				lv_obj_align(img_mon_6, LV_ALIGN_TOP_MID, 0, 166);
			}
			if(img_mon_5 == NULL){
				img_mon_5 =lv_img_create(parent);
				lv_img_set_src(img_mon_5, mon_1);
				lv_obj_align(img_mon_5, LV_ALIGN_TOP_MID, 0, 177);
			}
			if(img_mon_4 == NULL){
				img_mon_4 =lv_img_create(parent);
				lv_img_set_src(img_mon_4, mon_1);
				lv_obj_align(img_mon_4, LV_ALIGN_TOP_MID, 0, 187);
			}
			if(img_mon_3 == NULL){
				img_mon_3 =lv_img_create(parent);
				lv_img_set_src(img_mon_3, mon_1);
				lv_obj_align(img_mon_3, LV_ALIGN_TOP_MID, 0, 198);
			}
			if(img_mon_2 == NULL){
				img_mon_2 =lv_img_create(parent);
				lv_img_set_src(img_mon_2, mon_1);
				lv_obj_align(img_mon_2, LV_ALIGN_TOP_MID, 0, 209);
			}
			if(img_mon_1 == NULL){
				img_mon_1 =lv_img_create(parent);
				lv_img_set_src(img_mon_1, mon_1);
				lv_obj_align(img_mon_1, LV_ALIGN_TOP_MID, 0, 220);
			}
			break;
		}
	case 13:
		{
			if(beep_pow)beep_cout =13;
			if(img_mon_14 && lv_obj_is_valid(img_mon_14)){beeper_start_tim(BEEP_TIME);lv_obj_del(img_mon_14); img_mon_14 = NULL;}
			if(img_mon_15 && lv_obj_is_valid(img_mon_15)){beeper_start_tim(BEEP_TIME);lv_obj_del(img_mon_15); img_mon_15 = NULL;}
			if(img_mon_16 && lv_obj_is_valid(img_mon_16)){beeper_start_tim(BEEP_TIME);lv_obj_del(img_mon_16); img_mon_16 = NULL;}
			
			if(curr_data != data){
				stat_num =0;
				curr_data =data;
			}
			
			if(stat_num ++ < Stime){
				if(img_mon_13 == NULL){
					if(flag)beeper_start_tim(BEEP_TIME);
					img_mon_13 =lv_img_create(parent);
					lv_img_set_src(img_mon_13, mon_1);
					lv_obj_align(img_mon_13, LV_ALIGN_TOP_MID, 0, 92);
				}
			}else if(stat_num > 2*Stime){
				stat_num =0;
				if(img_mon_13 && lv_obj_is_valid(img_mon_13)){lv_obj_del(img_mon_13); img_mon_13 = NULL;}
			}
			
			if(img_mon_12 == NULL){
				img_mon_12 =lv_img_create(parent);
				lv_img_set_src(img_mon_12, mon_1);
				lv_obj_align(img_mon_12, LV_ALIGN_TOP_MID, 0, 103);
			}
			
			if(img_mon_11 == NULL){
				img_mon_11 =lv_img_create(parent);
				lv_img_set_src(img_mon_11, mon_1);
				lv_obj_align(img_mon_11, LV_ALIGN_TOP_MID, 0, 113);
			}
			if(img_mon_10 == NULL){
				img_mon_10 =lv_img_create(parent);
				lv_img_set_src(img_mon_10, mon_1);
				lv_obj_align(img_mon_10, LV_ALIGN_TOP_MID, 0, 123);
			}
			if(img_mon_9 == NULL){
				img_mon_9 =lv_img_create(parent);
				lv_img_set_src(img_mon_9, mon_1);
				lv_obj_align(img_mon_9, LV_ALIGN_TOP_MID, 0, 135);
			}
			if(img_mon_8 == NULL){
				img_mon_8 =lv_img_create(parent);
				lv_img_set_src(img_mon_8, mon_1);
				lv_obj_align(img_mon_8, LV_ALIGN_TOP_MID, 0, 146);
			}
			if(img_mon_7 == NULL){
				img_mon_7 =lv_img_create(parent);
				lv_img_set_src(img_mon_7, mon_1);
				lv_obj_align(img_mon_7, LV_ALIGN_TOP_MID, 0, 156);
			}
			if(img_mon_6 == NULL){
				img_mon_6 =lv_img_create(parent);
				lv_img_set_src(img_mon_6, mon_1);
				lv_obj_align(img_mon_6, LV_ALIGN_TOP_MID, 0, 166);
			}
			if(img_mon_5 == NULL){
				img_mon_5 =lv_img_create(parent);
				lv_img_set_src(img_mon_5, mon_1);
				lv_obj_align(img_mon_5, LV_ALIGN_TOP_MID, 0, 177);
			}
			if(img_mon_4 == NULL){
				img_mon_4 =lv_img_create(parent);
				lv_img_set_src(img_mon_4, mon_1);
				lv_obj_align(img_mon_4, LV_ALIGN_TOP_MID, 0, 187);
			}
			if(img_mon_3 == NULL){
				img_mon_3 =lv_img_create(parent);
				lv_img_set_src(img_mon_3, mon_1);
				lv_obj_align(img_mon_3, LV_ALIGN_TOP_MID, 0, 198);
			}
			if(img_mon_2 == NULL){
				img_mon_2 =lv_img_create(parent);
				lv_img_set_src(img_mon_2, mon_1);
				lv_obj_align(img_mon_2, LV_ALIGN_TOP_MID, 0, 209);
			}
			if(img_mon_1 == NULL){
				img_mon_1 =lv_img_create(parent);
				lv_img_set_src(img_mon_1, mon_1);
				lv_obj_align(img_mon_1, LV_ALIGN_TOP_MID, 0, 220);
			}
			break;
		}
	case 14:
		{
			if(beep_pow)beep_cout =14;
			if(img_mon_15 && lv_obj_is_valid(img_mon_15)){lv_obj_del(img_mon_15); img_mon_15 = NULL;}
			if(img_mon_16 && lv_obj_is_valid(img_mon_16)){lv_obj_del(img_mon_16); img_mon_16 = NULL;}
			
			if(flag_t ==0){
			
				if(curr_data != data){
					stat_num =0;
					curr_data =data;
				}
				
				if(stat_num ++ < Stime){
					if(img_mon_14 == NULL){
						if(flag)beeper_start_tim(BEEP_TIME);
						img_mon_14 =lv_img_create(parent);
						lv_img_set_src(img_mon_14, mon_1);
						lv_obj_align(img_mon_14, LV_ALIGN_TOP_MID, 0, 82);
					}
				}else if(stat_num > 2*Stime){
					stat_num =0;
					if(img_mon_14 && lv_obj_is_valid(img_mon_14)){lv_obj_del(img_mon_14); img_mon_14 = NULL;}
				}
			}else{
					if(img_mon_14 == NULL){
						if(flag)beeper_start_tim(BEEP_TIME);
						img_mon_14 =lv_img_create(parent);
						lv_img_set_src(img_mon_14, mon_1);
						lv_obj_align(img_mon_14, LV_ALIGN_TOP_MID, 0, 82);
					}
			}
			
			if(img_mon_13 == NULL){
				img_mon_13 =lv_img_create(parent);
				lv_img_set_src(img_mon_13, mon_1);
				lv_obj_align(img_mon_13, LV_ALIGN_TOP_MID, 0, 92);
			}
			
			if(img_mon_12 == NULL){
				img_mon_12 =lv_img_create(parent);
				lv_img_set_src(img_mon_12, mon_1);
				lv_obj_align(img_mon_12, LV_ALIGN_TOP_MID, 0, 103);
			}
			
			if(img_mon_11 == NULL){
				img_mon_11 =lv_img_create(parent);
				lv_img_set_src(img_mon_11, mon_1);
				lv_obj_align(img_mon_11, LV_ALIGN_TOP_MID, 0, 113);
			}
			if(img_mon_10 == NULL){
				img_mon_10 =lv_img_create(parent);
				lv_img_set_src(img_mon_10, mon_1);
				lv_obj_align(img_mon_10, LV_ALIGN_TOP_MID, 0, 123);
			}
			if(img_mon_9 == NULL){
				img_mon_9 =lv_img_create(parent);
				lv_img_set_src(img_mon_9, mon_1);
				lv_obj_align(img_mon_9, LV_ALIGN_TOP_MID, 0, 135);
			}
			if(img_mon_8 == NULL){
				img_mon_8 =lv_img_create(parent);
				lv_img_set_src(img_mon_8, mon_1);
				lv_obj_align(img_mon_8, LV_ALIGN_TOP_MID, 0, 146);
			}
			if(img_mon_7 == NULL){
				img_mon_7 =lv_img_create(parent);
				lv_img_set_src(img_mon_7, mon_1);
				lv_obj_align(img_mon_7, LV_ALIGN_TOP_MID, 0, 156);
			}
			if(img_mon_6 == NULL){
				img_mon_6 =lv_img_create(parent);
				lv_img_set_src(img_mon_6, mon_1);
				lv_obj_align(img_mon_6, LV_ALIGN_TOP_MID, 0, 166);
			}
			if(img_mon_5 == NULL){
				img_mon_5 =lv_img_create(parent);
				lv_img_set_src(img_mon_5, mon_1);
				lv_obj_align(img_mon_5, LV_ALIGN_TOP_MID, 0, 177);
			}
			if(img_mon_4 == NULL){
				img_mon_4 =lv_img_create(parent);
				lv_img_set_src(img_mon_4, mon_1);
				lv_obj_align(img_mon_4, LV_ALIGN_TOP_MID, 0, 187);
			}
			if(img_mon_3 == NULL){
				img_mon_3 =lv_img_create(parent);
				lv_img_set_src(img_mon_3, mon_1);
				lv_obj_align(img_mon_3, LV_ALIGN_TOP_MID, 0, 198);
			}
			if(img_mon_2 == NULL){
				img_mon_2 =lv_img_create(parent);
				lv_img_set_src(img_mon_2, mon_1);
				lv_obj_align(img_mon_2, LV_ALIGN_TOP_MID, 0, 209);
			}
			if(img_mon_1 == NULL){
				img_mon_1 =lv_img_create(parent);
				lv_img_set_src(img_mon_1, mon_1);
				lv_obj_align(img_mon_1, LV_ALIGN_TOP_MID, 0, 220);
			}
			break;
		}
	case 15:
		{
			if(beep_pow)beep_cout =15;
			if(img_mon_16 && lv_obj_is_valid(img_mon_16)){beeper_start_tim(BEEP_TIME);lv_obj_del(img_mon_16); img_mon_16 = NULL;}
			
			if(flag_t ==0){
				if(curr_data != data){
					stat_num =0;
					curr_data =data;
				}
				if(stat_num ++ < Stime){
					if(img_mon_15 == NULL){
						if(flag)beeper_start_tim(BEEP_TIME);
						img_mon_15 =lv_img_create(parent);
						lv_img_set_src(img_mon_15, mon_1);
						lv_obj_align(img_mon_15, LV_ALIGN_TOP_MID, 0, 71);
					}
				}else if(stat_num > 2*Stime){
					stat_num =0;
					if(img_mon_15 && lv_obj_is_valid(img_mon_15)){lv_obj_del(img_mon_15); img_mon_15 = NULL;}
				}
			}else{
					if(img_mon_15 == NULL){
						if(flag)beeper_start_tim(BEEP_TIME);
						img_mon_15 =lv_img_create(parent);
						lv_img_set_src(img_mon_15, mon_1);
						lv_obj_align(img_mon_15, LV_ALIGN_TOP_MID, 0, 71);
					}
			}
				
			if(img_mon_14 == NULL){
				img_mon_14 =lv_img_create(parent);
				lv_img_set_src(img_mon_14, mon_1);
				lv_obj_align(img_mon_14, LV_ALIGN_TOP_MID, 0, 82);
			}
			
			if(img_mon_13 == NULL){
				img_mon_13 =lv_img_create(parent);
				lv_img_set_src(img_mon_13, mon_1);
				lv_obj_align(img_mon_13, LV_ALIGN_TOP_MID, 0, 92);
			}
			
			if(img_mon_12 == NULL){
				img_mon_12 =lv_img_create(parent);
				lv_img_set_src(img_mon_12, mon_1);
				lv_obj_align(img_mon_12, LV_ALIGN_TOP_MID, 0, 103);
			}
			
			if(img_mon_11 == NULL){
				img_mon_11 =lv_img_create(parent);
				lv_img_set_src(img_mon_11, mon_1);
				lv_obj_align(img_mon_11, LV_ALIGN_TOP_MID, 0, 113);
			}
			if(img_mon_10 == NULL){
				img_mon_10 =lv_img_create(parent);
				lv_img_set_src(img_mon_10, mon_1);
				lv_obj_align(img_mon_10, LV_ALIGN_TOP_MID, 0, 123);
			}
			if(img_mon_9 == NULL){
				img_mon_9 =lv_img_create(parent);
				lv_img_set_src(img_mon_9, mon_1);
				lv_obj_align(img_mon_9, LV_ALIGN_TOP_MID, 0, 135);
			}
			if(img_mon_8 == NULL){
				img_mon_8 =lv_img_create(parent);
				lv_img_set_src(img_mon_8, mon_1);
				lv_obj_align(img_mon_8, LV_ALIGN_TOP_MID, 0, 146);
			}
			if(img_mon_7 == NULL){
				img_mon_7 =lv_img_create(parent);
				lv_img_set_src(img_mon_7, mon_1);
				lv_obj_align(img_mon_7, LV_ALIGN_TOP_MID, 0, 156);
			}
			if(img_mon_6 == NULL){
				img_mon_6 =lv_img_create(parent);
				lv_img_set_src(img_mon_6, mon_1);
				lv_obj_align(img_mon_6, LV_ALIGN_TOP_MID, 0, 166);
			}
			if(img_mon_5 == NULL){
				img_mon_5 =lv_img_create(parent);
				lv_img_set_src(img_mon_5, mon_1);
				lv_obj_align(img_mon_5, LV_ALIGN_TOP_MID, 0, 177);
			}
			if(img_mon_4 == NULL){
				img_mon_4 =lv_img_create(parent);
				lv_img_set_src(img_mon_4, mon_1);
				lv_obj_align(img_mon_4, LV_ALIGN_TOP_MID, 0, 187);
			}
			if(img_mon_3 == NULL){
				img_mon_3 =lv_img_create(parent);
				lv_img_set_src(img_mon_3, mon_1);
				lv_obj_align(img_mon_3, LV_ALIGN_TOP_MID, 0, 198);
			}
			if(img_mon_2 == NULL){
				img_mon_2 =lv_img_create(parent);
				lv_img_set_src(img_mon_2, mon_1);
				lv_obj_align(img_mon_2, LV_ALIGN_TOP_MID, 0, 209);
			}
			if(img_mon_1 == NULL){
				img_mon_1 =lv_img_create(parent);
				lv_img_set_src(img_mon_1, mon_1);
				lv_obj_align(img_mon_1, LV_ALIGN_TOP_MID, 0, 220);
			}
			break;
		}
	case 16:
		{
			if(beep_pow)beep_cout =16;
			if(flag_t ==0){
					if(curr_data != data){
						stat_num =0;
						curr_data =data;
					}
					
					if(stat_num ++ < Stime){
						if(img_mon_16 == NULL){
							if(flag)beeper_start_tim(BEEP_TIME);
							img_mon_16 =lv_img_create(parent);
							lv_img_set_src(img_mon_16, mon_1);
							lv_obj_align(img_mon_16, LV_ALIGN_TOP_MID, 0, 61);
						}
					}else if(stat_num > 2*Stime){
						stat_num =0;
						if(img_mon_16 && lv_obj_is_valid(img_mon_16)){lv_obj_del(img_mon_16); img_mon_16 = NULL;}
					}
			}else{
					if(img_mon_16 == NULL){
						if(flag)beeper_start_tim(BEEP_TIME);
						img_mon_16 =lv_img_create(parent);
						lv_img_set_src(img_mon_16, mon_1);
						lv_obj_align(img_mon_16, LV_ALIGN_TOP_MID, 0, 61);
					}
			}
			
			if(img_mon_15 == NULL){
				img_mon_15 =lv_img_create(parent);
				lv_img_set_src(img_mon_15, mon_1);
				lv_obj_align(img_mon_15, LV_ALIGN_TOP_MID, 0, 71);
			}
			
			if(img_mon_14 == NULL){
				img_mon_14 =lv_img_create(parent);
				lv_img_set_src(img_mon_14, mon_1);
				lv_obj_align(img_mon_14, LV_ALIGN_TOP_MID, 0, 82);
			}
			
			if(img_mon_13 == NULL){
				img_mon_13 =lv_img_create(parent);
				lv_img_set_src(img_mon_13, mon_1);
				lv_obj_align(img_mon_13, LV_ALIGN_TOP_MID, 0, 92);
			}
			
			if(img_mon_12 == NULL){
				img_mon_12 =lv_img_create(parent);
				lv_img_set_src(img_mon_12, mon_1);
				lv_obj_align(img_mon_12, LV_ALIGN_TOP_MID, 0, 103);
			}
			
			if(img_mon_11 == NULL){
				img_mon_11 =lv_img_create(parent);
				lv_img_set_src(img_mon_11, mon_1);
				lv_obj_align(img_mon_11, LV_ALIGN_TOP_MID, 0, 113);
			}
			if(img_mon_10 == NULL){
				img_mon_10 =lv_img_create(parent);
				lv_img_set_src(img_mon_10, mon_1);
				lv_obj_align(img_mon_10, LV_ALIGN_TOP_MID, 0, 123);
			}
			if(img_mon_9 == NULL){
				img_mon_9 =lv_img_create(parent);
				lv_img_set_src(img_mon_9, mon_1);
				lv_obj_align(img_mon_9, LV_ALIGN_TOP_MID, 0, 135);
			}
			if(img_mon_8 == NULL){
				img_mon_8 =lv_img_create(parent);
				lv_img_set_src(img_mon_8, mon_1);
				lv_obj_align(img_mon_8, LV_ALIGN_TOP_MID, 0, 146);
			}
			if(img_mon_7 == NULL){
				img_mon_7 =lv_img_create(parent);
				lv_img_set_src(img_mon_7, mon_1);
				lv_obj_align(img_mon_7, LV_ALIGN_TOP_MID, 0, 156);
			}
			if(img_mon_6 == NULL){
				img_mon_6 =lv_img_create(parent);
				lv_img_set_src(img_mon_6, mon_1);
				lv_obj_align(img_mon_6, LV_ALIGN_TOP_MID, 0, 166);
			}
			if(img_mon_5 == NULL){
				img_mon_5 =lv_img_create(parent);
				lv_img_set_src(img_mon_5, mon_1);
				lv_obj_align(img_mon_5, LV_ALIGN_TOP_MID, 0, 177);
			}
			if(img_mon_4 == NULL){
				img_mon_4 =lv_img_create(parent);
				lv_img_set_src(img_mon_4, mon_1);
				lv_obj_align(img_mon_4, LV_ALIGN_TOP_MID, 0, 187);
			}
			if(img_mon_3 == NULL){
				img_mon_3 =lv_img_create(parent);
				lv_img_set_src(img_mon_3, mon_1);
				lv_obj_align(img_mon_3, LV_ALIGN_TOP_MID, 0, 198);
			}
			if(img_mon_2 == NULL){
				img_mon_2 =lv_img_create(parent);
				lv_img_set_src(img_mon_2, mon_1);
				lv_obj_align(img_mon_2, LV_ALIGN_TOP_MID, 0, 209);
			}
			if(img_mon_1 == NULL){
				img_mon_1 =lv_img_create(parent);
				lv_img_set_src(img_mon_1, mon_1);
				lv_obj_align(img_mon_1, LV_ALIGN_TOP_MID, 0, 220);
			}
			break;
		}
		default:break;
	}
}

lv_obj_t * img_main7_1_t =NULL;

int sound_flag =1;
int move_flag =1;

int layer  =1;  //当前层级
int layer1 =1;  //层级1 下 序列号
int layer2 =0;	//层级2 下 序列号
int layer_ui_deal(int key)
{
  if(key && Device_POW_flag){
		beeper_start_tim(10);
	  //MOTOR_start_tim(20);
		if(Police_Flag){
			Police_Flag =0;
			Police_Flag_Again =1;
			return -1;
		}
	
		up_flag =1;
		//当前UI层级
		switch(layer){
			case 1:
				if(key==1){				//当前在第一层级
					//层级1递增
					if(layer1++ >= 8) layer1 =1;
				}else if(key==2){	//当前在第二层级
					layer =2;  //进入第二层级
					layer2 =1;
					if(layer1==1){
						//打开定时器
						hardware_timer_start();
					}
				}else if(key==3){
					//层级1递减
					if(layer1-- <= 1) layer1 =8;
				}
				break;
			case 2:
				if(layer1 == 1){
					//红外挡位调节
					if(key==1){
						//层级2递增
						if(layer2-- <= 1)layer2 =1;//if(layer2++ >= 5)layer2 =5;
					}else if(key==2){
						//设置片选
						layer =1;  //进入第一层级
						layer2 =0;
						hardware_timer_stop(); //关闭定时器
						IR_Send_Set(0);  //关闭红外
					}else if(key==3){
						//层级2递减
						if(layer2++ >= 5)layer2 =5;//if(layer2-- <= 1)layer2 =1;
					}
				}else if(layer1 == 8){
					//设置界面
					if(key==1){
						//层级2递增
						if(layer2++ >= 3) layer2 =1;
					}else if(key==2){
						if(layer2==1){	//声音
							if(sound_flag){
								sound_flag =0;
							}else{
								sound_flag =1;
							}
						}
						if(layer2==2){	//震动
							if(move_flag){
								move_flag =0;
							}else{
								move_flag =1;
							}
						}
						if(layer2==3){
							//设置片选
							layer =1;  //进入第一层级
							layer2 =0;
							
							//将数据保存内部flash
							//if(sound_flag) Flash_Data |= (1<<0);
							//else Flash_Data &= ~(1<<0);
							//if(move_flag) Flash_Data |= (1<<1);
							//else Flash_Data &= ~(1<<1);
							//save_data_to_flash(Flash_Data);
							//write_flash(Flash_Data);
						}
					}else if(key==3){
						//层级2递减
						if(layer2-- <= 1) layer2 =3;
					}
				}else if(layer1 == 2){
					//夜视
					if(key==1){
						//层级2递增
						if(sensitivity_ir-- <= 1)sensitivity_ir =1;//if(layer2++ >= 5)layer2 =5;
					}else if(key==2){
						sensitivity_ir =1;
						layer =1;  //进入第一层级
						layer2 =0;
					}else if(key==3){
						//层级2递减
						if(sensitivity_ir++ >= 14)sensitivity_ir =14;//if(layer2-- <= 1)layer2 =1;
					}
					
					
				}else if(layer1 == 3){
					//无线
					if(key==1){
						//层级2递增
						if(sensitivity_wless-- <= 1)sensitivity_wless =1;//if(layer2++ >= 5)layer2 =5;
					}else if(key==2){
						sensitivity_wless =1;
						layer =1;  //进入第一层级
						layer2 =0;
					}else if(key==3){
						//层级2递减
						if(sensitivity_wless++ >= 14)sensitivity_wless =14;//if(layer2-- <= 1)layer2 =1;
					}
					
				}else if(layer1 == 4){
					//WIFI
					if(key==1){
						//层级2递增
						if(sensitivity_wless-- <= 1)sensitivity_wless =1;//if(layer2++ >= 5)layer2 =5;
					}else if(key==2){
						sensitivity_wless =1;
						layer =1;  //进入第一层级
						layer2 =0;
					}else if(key==3){
						//层级2递减
						if(sensitivity_wless++ >= 14)sensitivity_wless =14;//if(layer2-- <= 1)layer2 =1;
					}
					
				}else if(layer1 == 6){
					//强磁
					if(key==1){
						//层级2递增
						if(sensitivity_hall-- <= 1)sensitivity_hall =1;//if(layer2++ >= 5)layer2 =5;
					}else if(key==2){
						sensitivity_hall =1;
						layer =1;  //进入第一层级
						layer2 =0;
					}else if(key==3){
						//层级2递减
						if(sensitivity_hall++ >= 14)sensitivity_hall =14;//if(layer2-- <= 1)layer2 =1;
					}
					
				}else{
					if(key==2){
						layer =1;  //进入第一层级
						layer2 =0;
					}
				}
				break;
			default:
				break;
		}
		key =0;
	}
	return 0;
}


int up_flag =1;
static int ir_data =1;
static int mpn_data =1;
int tag_data =0;
void Gui_Pro(void)
{
//判断是否进入报警休眠
if(Police_Flag ==0){
		switch(layer1){ //当前在所在层级1序号
			case 1:
			{
				static int layer2_flag =0;
				if(up_flag){
					up_flag =0;
					lv_del();
					if(layer==1){	//
						img_main1 = lv_img_create(img_main);
						lv_img_set_src(img_main1, &main_ui1);
						lv_obj_align(img_main1, LV_ALIGN_TOP_MID, 0, 20);
					}
					if(layer2){	// 层级1 - 层级1序号1 - 层级2序号置一
							if(img_main1_1==NULL){
								layer2_flag =1;
								layer2 =2;
								img_main1_1 =lv_img_create(img_main);
								lv_obj_set_size(img_main1_1, lv_disp_get_hor_res(NULL), lv_disp_get_ver_res(NULL));
								lv_img_set_src(img_main1_1, &mon);
								lv_obj_align(img_main1_1, LV_ALIGN_TOP_MID, 0, 0);
							}
					}else{   //返回到层级1，且层级二置零
						 if(layer2_flag){
								layer2_flag =0;
								if(img_main1_1 && lv_obj_is_valid(img_main1_1)){
									mon_img_pro(0, img_main1_1, &red, 0, 0, 1, 0);
									lv_obj_del(img_main1_1);
									img_main1_1 = NULL;
								}
						 }
					}
				}
				if(layer2_flag){
					switch(layer2){
						case 1:
						{
							mon_img_pro(0, img_main1_1, &red, 0, 1, 0, 0);
							IR_flag_mode =0;
							break;
						}
						case 2:
						{
							mon_img_pro(4, img_main1_1, &red, 0, 1, 0, 0);
							IR_flag_mode =1;
							break;
						}
						case 3:
						{
							mon_img_pro(8, img_main1_1, &red, 0, 1, 0, 0);
							IR_flag_mode =2;
							break;
						}
						case 4:
						{
							mon_img_pro(12, img_main1_1, &red, 0, 1, 0, 0);
							IR_flag_mode =3;
							break;
						}
						case 5:
						{
							mon_img_pro(16, img_main1_1, &red, 0, 1, 0, 0);
							IR_flag_mode =4;
							break;
						}
						default:break;
					}
				}
				break;
			}
			case 2:  //夜视
			{
				static int layer2_flag =0;
				if(up_flag){
					up_flag =0;
					lv_del();
					if(layer==1){
						img_main2 = lv_img_create(img_main);
						lv_img_set_src(img_main2, &main_ui2);
						lv_obj_align(img_main2, LV_ALIGN_TOP_MID, 0, 60+1-4);
					}
					if(layer2){
						if(img_main2_1==NULL){
							layer2_flag =1;
							img_main2_1 =lv_img_create(img_main);
							lv_obj_set_size(img_main2_1, lv_disp_get_hor_res(NULL), lv_disp_get_ver_res(NULL));
							lv_img_set_src(img_main2_1, &mon);
							lv_obj_align(img_main2_1, LV_ALIGN_TOP_MID, 0, 0);
							curr_pos_set =2;
						}
					}else{
						 if(layer2_flag){
								layer2_flag =0;
								if(img_main2_1 && lv_obj_is_valid(img_main2_1)){
								  //关闭震动
									MOTOR_stop();
									mon_img_pro(0, img_main2_1, &mon_1, 0, 0, 1, 1);
									mon2_img_pro(0, img_main2_1);
									lv_obj_del(img_main2_1);
									img_main2_1 = NULL;
								}
								curr_pos_set =0;
						 }
					}
				}
				if(layer2_flag ==1){
					int ir_data_t =ir_data_deal(ADC_IR);
					if(ir_data_t>0)MOTOR_start();
					else MOTOR_stop();
					mon_img_pro(ir_data_t, img_main2_1, &mon_1, 1, 0, 0, 1);
					static int sensitivity_ir_t =0;
					if(sensitivity_ir_t != sensitivity_ir){
						sensitivity_ir_t =sensitivity_ir;
						mon2_img_pro(sensitivity_ir, img_main2_1);
					}
				}
				break;
			}
			case 3: //无线
			{
				static int layer2_flag =0;
				if(up_flag){
					up_flag =0;
					lv_del();
					if(layer==1){
						img_main3 = lv_img_create(img_main);
						lv_img_set_src(img_main3, &main_ui3);
						lv_obj_align(img_main3, LV_ALIGN_TOP_MID, 0, 105-11);
					}
					if(layer2){
						if(img_main3_1==NULL){
							layer2_flag =1;
							img_main3_1 =lv_img_create(img_main);
							lv_obj_set_size(img_main3_1, lv_disp_get_hor_res(NULL), lv_disp_get_ver_res(NULL));
							lv_img_set_src(img_main3_1, &mon);
							lv_obj_align(img_main3_1, LV_ALIGN_TOP_MID, 0, 0);
							curr_pos_set =3;
						}
					}else{
						 if(layer2_flag){
								layer2_flag =0;
								if(img_main3_1 && lv_obj_is_valid(img_main3_1)){
									//关闭震动
									MOTOR_stop();
									mon_img_pro(wifi_data_deal(ADC_Wless), img_main3_1, &mon_1, 1, 0, 1, 1);
									mon2_img_pro(0, img_main3_1);
									lv_obj_del(img_main3_1);
									img_main3_1 = NULL;
								}
								curr_pos_set =0;
						 }
					}
				}
				if(layer2_flag ==1){
					int wifi_data_t =wifi_data_deal(ADC_Wless);
					if(wifi_data_t>0)MOTOR_start();
					else MOTOR_stop();
					mon_img_pro(wifi_data_t, img_main3_1, &mon_1, 1, 0, 0, 1);
					
					static int sensitivity_wless_t =0;
					if(sensitivity_wless_t != sensitivity_wless){
						sensitivity_wless_t =sensitivity_wless;
						mon2_img_pro(sensitivity_wless, img_main3_1);
					}
				}
				break;
			}
			
			case 4:  //Wifi
			{
				static int layer2_flag =0;
				if(up_flag){
					up_flag =0;
					lv_del();
					if(layer==1){
						img_main8 = lv_img_create(img_main);
						lv_img_set_src(img_main8, &main_ui4);
						lv_obj_align(img_main8, LV_ALIGN_TOP_MID, 0, 140+6-14); 
					}
					if(layer2){
						if(img_main8_1==NULL){
							layer2_flag =1;
							img_main8_1 =lv_img_create(img_main);
							lv_obj_set_size(img_main8_1, lv_disp_get_hor_res(NULL), lv_disp_get_ver_res(NULL));
							lv_img_set_src(img_main8_1, &mon);
							lv_obj_align(img_main8_1, LV_ALIGN_TOP_MID, 0, 0);
							curr_pos_set =4;
						}
					}else{
						 if(layer2_flag){
								layer2_flag =0;
								if(img_main8_1 && lv_obj_is_valid(img_main8_1)){
									//关闭震动
									MOTOR_stop();
									mon_img_pro(wifi_data_deal(ADC_Wless), img_main8_1, &mon_1, 1, 0, 1, 1);
									mon2_img_pro(0, img_main8_1);
									lv_obj_del(img_main8_1);
									img_main8_1 = NULL;
								}
								curr_pos_set =0;
						 }
					}
				}
				if(layer2_flag ==1){
					int wifi_data_t =wifi_data_deal(ADC_Wless-100);
					if(wifi_data_t>0)MOTOR_start();
					else MOTOR_stop();
					mon_img_pro(wifi_data_t, img_main8_1, &mon_1, 1, 0, 0, 1);
					
					static int sensitivity_wless_t =0;
					if(sensitivity_wless_t != sensitivity_wless){
						sensitivity_wless_t =sensitivity_wless;
						mon2_img_pro(sensitivity_wless, img_main8_1);
					}
				}
				break;
			}
			
			
			case 5:		//tag
			{
				static int layer2_flag =0;
				if(up_flag){
					up_flag =0;
					lv_del();
					if(layer==1){
						img_main4 = lv_img_create(img_main);
						lv_img_set_src(img_main4, &main_ui5);
						lv_obj_align(img_main4, LV_ALIGN_TOP_MID, 0, 190-2-19);
					}
					if(layer2){
						if(img_main4_1==NULL){
							layer2_flag =1;
							img_main4_1 =lv_img_create(img_main);
							lv_obj_set_size(img_main4_1, lv_disp_get_hor_res(NULL), lv_disp_get_ver_res(NULL));
							lv_img_set_src(img_main4_1, &mon);
							lv_obj_align(img_main4_1, LV_ALIGN_TOP_MID, 0, 0);
							curr_pos_set =5;
							scan_mode =1;
							scan_start();
						}
					}else{
						 if(layer2_flag){
								layer2_flag =0;
								if(img_main4_1 && lv_obj_is_valid(img_main4_1)){
									//关闭震动
									MOTOR_stop();
									mon_img_pro(tag_data, img_main4_1, &mon_1, 1, 0, 1, 1);
									mon2_img_pro(0, img_main4_1);
									lv_obj_del(img_main4_1);
									img_main4_1 = NULL;
								}
								curr_pos_set =0;
								scan_stop();
						 }
					}
				}
				if(layer2_flag ==1){
				#if 1
					static int scan_flag =0;
					if(scan_mode ==1){
						scan_flag =1;
						static int tag_data_t =0;
						static bool flag =1;
						if(flag){
							if(tag_data_t++ >16){
								flag =0;
							}
						}else if(flag ==0){
							if(tag_data_t-- <0){
								flag =1;
							}
						}
						mon_img_pro_t(tag_data_t, img_main4_1, &red, 1);
					}else if(scan_mode ==2){
						if(scan_flag){
							scan_flag =0;
							mon_img_pro_t(0, img_main4_1, &red, 1);
						}
						if(tag_data<=0) tag_data =0;//tag_data =1;
						if(tag_data){
							MOTOR_start();
						}else{
							MOTOR_stop();
						}
						mon_img_pro(tag_data, img_main4_1, &mon_1, 1, 0, 0, 1);
					}
					#endif
					
					
					//mon_img_pro(tag_data, img_main4_1, &mon_1, 1);
					
					
					
//					static int sensitivity_tag_t =0;
//					if(sensitivity_tag_t != sensitivity_tag){
//						sensitivity_tag_t =sensitivity_tag;
//						mon2_img_pro(sensitivity_tag, img_main4_1);
//					}
					
					
				}
				break;
			}
			case 6:  //强磁
			{
				static int layer2_flag =0;
				if(up_flag){
					up_flag =0;
					lv_del();
					if(layer==1){
						img_main5 = lv_img_create(img_main);
						lv_img_set_src(img_main5, &main_ui6);
						lv_obj_align(img_main5, LV_ALIGN_TOP_MID, 0, 190+16);
					}
					if(layer2){
						if(img_main5_1==NULL){
							layer2_flag =1;
							img_main5_1 =lv_img_create(img_main);
							lv_obj_set_size(img_main5_1, lv_disp_get_hor_res(NULL), lv_disp_get_ver_res(NULL));
							lv_img_set_src(img_main5_1, &mon);
							//lv_obj_center(img_main5_1);
							lv_obj_align(img_main5_1, LV_ALIGN_TOP_MID, 0, 0);
							curr_pos_set =6;
						}
					}else{
						 if(layer2_flag){
								layer2_flag =0;
								if(img_main5_1 && lv_obj_is_valid(img_main5_1)){
									//关闭震动
									MOTOR_stop();
									mon_img_pro(0, img_main5_1, &mon_1, 0, 0, 1, 1);
									mon2_img_pro(0, img_main5_1);
									lv_obj_del(img_main5_1);
									img_main5_1 = NULL;
								}
								curr_pos_set =0;
						 }
					}
				}
				if(layer2_flag ==1){
					//mon_img_pro(mpn_data, img_main5_1, &mon_1);
					int hall_data_t =hall_data_deal(ADC_Hall);
					if(hall_data_t>0)MOTOR_start();
					else MOTOR_stop();
					mon_img_pro(hall_data_t, img_main5_1, &mon_1, 1, 0, 0, 1);
					
					static int sensitivity_hall_t =0;
					if(sensitivity_hall_t != sensitivity_hall){
						sensitivity_hall_t =sensitivity_hall;
						mon2_img_pro(sensitivity_hall, img_main5_1);
					}
					
				}
				break;
			}


			case 8:   //设置
			{
				if(up_flag){
					up_flag =0;		
					lv_del();
					if(layer==1){
						img_main6 = lv_img_create(img_main);
						lv_img_set_src(img_main6, &main_ui8);
						//lv_obj_align(img_main6, LV_ALIGN_TOP_MID, 0, 231);
						lv_obj_align(img_main6, LV_ALIGN_TOP_MID, 0, 273+8);
					}
					static int layer2_flag =0;
					if(layer2){
						if(img_main6_1==NULL){
							layer2_flag =1;
							img_main6_1 =lv_img_create(img_main);
							lv_obj_set_size(img_main6_1, lv_disp_get_hor_res(NULL), lv_disp_get_ver_res(NULL));
							lv_img_set_src(img_main6_1, &set);
							lv_obj_align(img_main6_1, LV_ALIGN_TOP_MID, 0, 0);
							
							//设备ID
							if(img_main6_t==NULL){
								#if SET_ID_POW
								LV_FONT_DECLARE(lv_chinese)
								img_main6_t = lv_label_create(img_main6_1);
								lv_obj_set_style_text_color(img_main6_t, lv_color_hex(0x70DBFF), 0);
								extern const lv_font_t chinese_1;
								lv_obj_set_style_text_font(img_main6_t, &chinese_1, 0); 
								lv_label_set_text(img_main6_t, "设备ID:");
								lv_obj_align(img_main6_t, LV_ALIGN_TOP_MID, 0, 250+25);
								
								#if 0
								//
								char buffer[4];
								char *device_id_only_p =(char *)&device_id_only;
								sprintf(buffer, "%02X-%02X-%02X-%02X", device_id_only_p[0],device_id_only_p[1],device_id_only_p[2],device_id_only_p[3]); // 使用大写字母格式化
								img_main6_t_1 = lv_label_create(img_main6_1);
								lv_obj_set_style_text_color(img_main6_t_1, lv_color_hex(0x70DBFF), 0);
								extern const lv_font_t chinese_1;
								lv_obj_set_style_text_font(img_main6_t_1, &chinese_1, 0); 
								lv_label_set_text(img_main6_t_1, buffer);
								lv_obj_align(img_main6_t_1, LV_ALIGN_TOP_MID, -20, 270);
								//
								sprintf(buffer, "%02X-%02X-%02X-%02X", device_id_only_p[4],device_id_only_p[5],device_id_only_p[6],device_id_only_p[7]); // 使用大写字母格式化
								img_main6_t_1 = lv_label_create(img_main6_1);
								lv_obj_set_style_text_color(img_main6_t_1, lv_color_hex(0x70DBFF), 0);
								extern const lv_font_t chinese_1;
								lv_obj_set_style_text_font(img_main6_t_1, &chinese_1, 0); 
								lv_label_set_text(img_main6_t_1, buffer);
								lv_obj_align(img_main6_t_1, LV_ALIGN_TOP_MID, 20, 270);
								#endif
								
								static char buffer[25];
								char *device_id_only_p =(char *)&device_id_only;
								sprintf(buffer, "%02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X", device_id_only_p[0],device_id_only_p[1],device_id_only_p[2],device_id_only_p[3],device_id_only_p[4],device_id_only_p[5],device_id_only_p[6],device_id_only_p[7]); // 使用大写字母格式化
								img_main6_t_1 = lv_label_create(img_main6_1);
								lv_obj_set_style_text_color(img_main6_t_1, lv_color_hex(0x70DBFF), 0);
								extern const lv_font_t chinese_1;
								lv_obj_set_style_text_font(img_main6_t_1, &chinese_1, 0); 
								lv_label_set_text(img_main6_t_1, buffer);
								lv_obj_align(img_main6_t_1, LV_ALIGN_TOP_MID, 0, 270+25);
								#endif
								
							}
							
							
							
							 if(sound_flag){
									if(img_main6_1_1_1 ==NULL){
										img_main6_1_1_1 =lv_img_create(img_main6_1);
										lv_img_set_src(img_main6_1_1_1, &key);
										lv_obj_align(img_main6_1_1_1, LV_ALIGN_TOP_MID, 56, 73);
									}
							 }else{
									if(img_main6_1_1_1 !=NULL){
										lv_obj_del(img_main6_1_1_1);
										img_main6_1_1_1 = NULL;
									}
							 }
							
							
							 if(move_flag){
									if(img_main6_1_2_1 ==NULL){
										img_main6_1_2_1 =lv_img_create(img_main6_1);
										lv_img_set_src(img_main6_1_2_1, &key);
										lv_obj_align(img_main6_1_2_1, LV_ALIGN_TOP_MID, 56, 121);
									}
							 }else{
									if(img_main6_1_2_1 !=NULL){
										lv_obj_del(img_main6_1_2_1);
										img_main6_1_2_1 = NULL;
									}
							 }
							
						}
						if(layer2 ==1){
							 if(img_main6_1_2 !=NULL){
									lv_obj_del(img_main6_1_2);
									img_main6_1_2 = NULL;
								}
							 if(img_main6_1_3 !=NULL){
									lv_obj_del(img_main6_1_3);
									img_main6_1_3 = NULL;
								}
							 if(img_main6_1_1 ==NULL){
									img_main6_1_1 =lv_img_create(img_main6_1);
									lv_img_set_src(img_main6_1_1, &set_1);
									lv_obj_align(img_main6_1_1, LV_ALIGN_TOP_MID, 0, 60);
							 }
							 
							 if(sound_flag){
									if(img_main6_1_1_1 ==NULL){
										img_main6_1_1_1 =lv_img_create(img_main6_1);
										lv_img_set_src(img_main6_1_1_1, &key);
										lv_obj_align(img_main6_1_1_1, LV_ALIGN_TOP_MID, 56, 73);
									}
							 }else{
									if(img_main6_1_1_1 !=NULL){
										lv_obj_del(img_main6_1_1_1);
										img_main6_1_1_1 = NULL;
									}
							 }
						}
						if(layer2 ==2){
							 if(img_main6_1_1 !=NULL){
									lv_obj_del(img_main6_1_1);
									img_main6_1_1 = NULL;
								}
							 if(img_main6_1_3 !=NULL){
									lv_obj_del(img_main6_1_3);
									img_main6_1_3 = NULL;
							 }
							 if(img_main6_1_2 ==NULL){
									img_main6_1_2 =lv_img_create(img_main6_1);
									lv_img_set_src(img_main6_1_2, &set_1);
									lv_obj_align(img_main6_1_2, LV_ALIGN_TOP_MID, 0, 110);
							 }
							 
							 if(move_flag){
									if(img_main6_1_2_1 ==NULL){
										img_main6_1_2_1 =lv_img_create(img_main6_1);
										lv_img_set_src(img_main6_1_2_1, &key);
										lv_obj_align(img_main6_1_2_1, LV_ALIGN_TOP_MID, 56, 121);
									}
							 }else{
									if(img_main6_1_2_1 !=NULL){
										lv_obj_del(img_main6_1_2_1);
										img_main6_1_2_1 = NULL;
									}
							 }
						}
						if(layer2 ==3){
							 if(img_main6_1_1 !=NULL){
									lv_obj_del(img_main6_1_1);
									img_main6_1_1 = NULL;
								}
							 if(img_main6_1_2 !=NULL){
									lv_obj_del(img_main6_1_2);
									img_main6_1_2 = NULL;
								}
							 if(img_main6_1_3 ==NULL){
									img_main6_1_3 =lv_img_create(img_main6_1);
									lv_img_set_src(img_main6_1_3, &set_1);
									lv_obj_align(img_main6_1_3, LV_ALIGN_TOP_MID, 0, 155);
							 }
						}
					}else{
						 if(layer2_flag){
								layer2_flag =0;
								if(img_main6_1 && lv_obj_is_valid(img_main6_1)) {
									lv_obj_del(img_main6_1);
									img_main6_1 = NULL;
									img_main6_1_1= NULL;
									img_main6_1_2= NULL;
									img_main6_1_3= NULL;
									img_main6_1_1_1= NULL;
									img_main6_1_2_1= NULL;
									img_main6_t =NULL;
								}
						 }
					}
				break;
			}
			}
			case 7:  //震动报警
			{
				static int time_t =0;
				static int layer2_flag =0;
				if(up_flag){
					up_flag =0;
					lv_del();
					if(layer==1){
						img_main7 = lv_img_create(img_main);
						lv_img_set_src(img_main7, &main_ui7);
						//lv_obj_align(img_main7, LV_ALIGN_TOP_MID, 0, 273);
						lv_obj_align(img_main7, LV_ALIGN_TOP_MID, 0, 231+13);
					}
					if(layer2){
						if(img_main7_1==NULL){
							if(img_main7_1_t ==NULL){
								layer2_flag =1;
							
								// 创建覆盖层（以图片为父对象）
								img_main7_1_t = lv_obj_create(img_main);
								
								// 设置覆盖层大小与图片一致
								//lv_obj_set_size(img_main7_1_t, lv_obj_get_width(img_main), lv_obj_get_height(img_main));
								//lv_obj_align(img_main7_1_t, LV_ALIGN_CENTER, 0, 0);  // 居中对齐
								lv_obj_set_size(img_main7_1_t, 170, 320);
								lv_obj_align(img_main7_1_t, LV_ALIGN_TOP_MID, 0, 0);  // 居中对齐
								
								// 设置纯黑色背景（不透明）
								lv_obj_set_style_bg_color(img_main7_1_t, lv_color_black(), 0);
								lv_obj_set_style_bg_opa(img_main7_1_t, LV_OPA_100, 0);  // 100% 不透明
								
								// 确保覆盖层在最上层
								lv_obj_move_foreground(img_main7_1_t);
							
								LV_FONT_DECLARE(lv_chinese)
								img_main7_1 = lv_label_create(img_main7_1_t);
								lv_obj_set_style_text_color(img_main7_1, lv_color_hex(0x70DBFF), 0);
								extern const lv_font_t chinese;
								lv_obj_set_style_text_font(img_main7_1, &chinese, 0); 

								//lv_label_set_text(img_main7_1, "7S后开始检查 ");
								lv_label_set_text(img_main7_1, "7S后开始侦测 ");
								//lv_obj_align(img_main7_1, LV_ALIGN_TOP_MID, 0, 160);
								lv_obj_align(img_main7_1, LV_ALIGN_TOP_MID, 0, 100);
								time_t =1400;
							}
						}
					}else{
						 if(layer2_flag){
								layer2_flag =0;
								if(img_main7_1 && lv_obj_is_valid(img_main7_1)) {
									lv_obj_del(img_main7_1);
									img_main7_1 = NULL;
								}
								if(img_main7_1_t && lv_obj_is_valid(img_main7_1_t)) {
									lv_obj_del(img_main7_1_t);
									img_main7_1_t = NULL;
								}
								//AD8314_Int_enable();
						 }
					}
				}else{
					//倒计时7S后开始进入报警
					if(layer2_flag){
						if(time_t-- ==0){
								if(img_main7_1 && lv_obj_is_valid(img_main7_1)) {
									lv_obj_del(img_main7_1);
									img_main7_1 = NULL;
								}
								if(img_main7_1_t && lv_obj_is_valid(img_main7_1_t)) {
									lv_obj_del(img_main7_1_t);
									img_main7_1_t = NULL;
								}
								layer2 =0;
								layer1 =1;
								layer =1;
								up_flag =1;
								//打开报警
								AD8314_Int_enable();
						}
					}
				}
				break;
			}
			
			
			
			
			
			
			
			
			
			
			
			
			
			default:break;
		}
	}
}


