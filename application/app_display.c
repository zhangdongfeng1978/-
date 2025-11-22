#include "app_display.h"
#include "app_config.h"
#include "bsp.h"
#include "FreeRTOS.h"
#include "task.h"
#include "app_system.h"
#include "app_times.h"


GUI_HWIN home_w;

int current_show_mode=0;



int time_int_second;
int desk_index=0;
int gui_run_times=0;
int gui_action_times=0;
int lcd_backlight_status=0;


static const GUI_POINT big_numbers[8][6]=
{

    5,4, 10,0, 31,0, 35,4,  29,10, 11,10,
    35,5, 40,9, 40,33, 35,37, 30,32, 30,11,
    35,38, 40,40,  40,66,  35,70, 30,64, 30,43,
    5,70, 11,65, 29,65, 35,70, 30,75, 9,75,
    5,37, 10,42, 10,64, 5,70,  0,66,  0,42,
    0,10, 5,5, 10,11, 10,32, 5,37, 0,33,
    5,37, 11,32, 29,32, 34,37,  29,42,  10,42,
};




const GUI_POINT ANT12_log[6]= {0,0,10,0,5,7,5,14,5,7,0,0};

static const unsigned char dig_code[10]= {0xc0, 0xf9, 0xa4, 0xb0, 0x99, 0x92, 0x82, 0xf8, 0x80, 0x90};


static void draw_digtal_number(unsigned short x,unsigned short y,unsigned char num)
{
    unsigned char code= dig_code[num];
    for(int i=0; i<8; i++)
    {
        if(((code>>i)&0x01)==0)  GUI_FillPolygon(big_numbers[i],6,x,y);
    }
}




static void draw_welcom_desktop(void)
{
    GUI_SetBkColor(GUI_WHITE);
    GUI_Clear();
    GUI_SetColor(GUI_BLACK);
    GUI_SetFont(&GUI_FontHZ24);
    GUI_DispStringHCenterAt("welcome",78,40);
//    unsigned char h_d,h_n,s_d,s_n,m_d,m_n;
//    h_d=sys_status.sys_times.tm_hour/10;
//    h_n=sys_status.sys_times.tm_hour%10;
//    s_d=sys_status.sys_times.tm_min/10;
//    s_n=sys_status.sys_times.tm_min%10;
//    m_n=sys_status.sys_times.tm_sec%10;
//    m_d=sys_status.sys_times.tm_sec/10;
//    draw_digtal_number(44,18,m_d);
//    draw_digtal_number(100,18,m_n);
    //  draw_digtal_number(24,88,s_d);
    //  draw_digtal_number(80,88,s_n);
}
static void cb_home_Win(WM_MESSAGE *pMsg)
{
    int *key_press;
    switch (pMsg->MsgId)
    {
    case WM_PAINT:
        draw_welcom_desktop();
        LCD_L0_Refresh();
        WM_CreateTimer(home_w,4,500,1);
        break;
    case WM_KEY:
        key_press=(int *)pMsg->Data.p;
        switch(*key_press)
        {
        case WM_USER_KEY_UP:
            WM_SelectWindow(home_w);
            break;
        case WM_USER_KEY_DOWN:
            WM_SelectWindow(home_w);
            break;
        case WM_USER_KEY_ENTER:
					   display_set_buzzer(1);
            draw_fullband_windosw(home_w);
            break;
        case WM_USER_KEY_BACK:
            WM_SelectWindow(home_w);
            LCD_L0_Refresh();
            break;
        case WM_USER_KEY_BACK_LONG:
            if(lcd_backlight_status==0)
            {
                lcd_backlight_ctr(1);
                lcd_backlight_status=1;
            }
            else
            {
                lcd_backlight_ctr(0);
                lcd_backlight_status=0;
            }
            break;
        default:
            break;
        }
        break;
    case	WM_TIMER:
        WM_DeleteTimer(home_w,4);
        leds_display_mode(1);
        draw_fullband_windosw(home_w);
        break;
    default:
        WM_DefaultProc(pMsg);
    }
}


static void leds_display_init(void)
{
    nrf_gpio_cfg_output(LED_1);
    nrf_gpio_pin_clear(LED_1);
    nrf_gpio_cfg_output(LED_2);
    nrf_gpio_pin_set(LED_2);
    nrf_gpio_cfg_output(LED_3);
    nrf_gpio_pin_set(LED_3);
    nrf_gpio_cfg_output(LED_4);
    nrf_gpio_pin_set(LED_4);
    nrf_gpio_cfg_output(LED_5);
    nrf_gpio_pin_set(LED_5);
    nrf_gpio_cfg_output(LED_6);
    nrf_gpio_pin_set(LED_6);
    nrf_gpio_cfg_output(LED_7);
    nrf_gpio_pin_set(LED_7);
	
	    nrf_gpio_cfg_output(SYSTEM_BUZZER_EN);
    nrf_gpio_pin_set(SYSTEM_BUZZER_EN);
	
	
		    nrf_gpio_cfg_output(SYSTEM_VOICE_EN);
    nrf_gpio_pin_set(SYSTEM_VOICE_EN);
}
void leds_display_mode(unsigned char mode)
{
    nrf_gpio_pin_set(LED_2);
    nrf_gpio_pin_set(LED_3);
    nrf_gpio_pin_set(LED_4);
    nrf_gpio_pin_set(LED_5);
    nrf_gpio_pin_set(LED_6);
    nrf_gpio_pin_set(LED_7);
    if(mode==1)
        nrf_gpio_pin_clear(LED_2);
    else	if(mode==2)
        nrf_gpio_pin_clear(LED_3);
    else	if(mode==3)
        nrf_gpio_pin_clear(LED_4);
    else	if(mode==4)
        nrf_gpio_pin_clear(LED_5);
    else	if(mode==5)
        nrf_gpio_pin_clear(LED_6);
}
void display_set_vioce(unsigned char enable)
{
    if(enable)
    {
        nrf_gpio_pin_set(SYSTEM_VOICE_EN);
    }
    else
    {
        nrf_gpio_pin_clear(SYSTEM_VOICE_EN);
    }
}
void display_set_buzzer(unsigned char enable)
{
    if(enable)
    {
        nrf_gpio_pin_set(SYSTEM_BUZZER_EN);
    }
    else
    {
        nrf_gpio_pin_clear(SYSTEM_BUZZER_EN);
    }
}
 
void leds_display_set_indicator(unsigned char enable)
{
    if(enable)
    {
        nrf_gpio_pin_clear(LED_1);
    }
    else
    {
        nrf_gpio_pin_set(LED_1);
    }
}
void leds_display_set_mute(unsigned char enable)
{
    if(enable)
    {
        nrf_gpio_pin_clear(LED_7);
    }
    else
    {
        nrf_gpio_pin_set(LED_7);
    }
}

static void app_display_init(void)
{
    leds_display_init();
    GUI_Init();
    home_w=WM_GetDesktopWindow();
    WM_SelectWindow(home_w);
    WM_SetCallback(home_w,cb_home_Win);
    WM_SetFocus(home_w);
    leds_display_mode(0);
    current_show_mode=0;
	  display_set_buzzer(0);
}



void app_display_thread(void *ass)
{
    UNUSED_PARAMETER(ass);
    app_display_init();
    while (1)
    {
        GUI_Exec();
        gui_run_times++;
        GUI_WaitKey();
    }
}