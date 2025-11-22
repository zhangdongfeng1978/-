#include "app_config.h"
#include "bsp.h"
#include "app_display.h"
#include "FreeRTOS.h"
#include "task.h"
#include "app_times.h"

//rtc_time_t tody_tm;
GUI_HWIN drones_w;


static void cb_drones_Win(WM_MESSAGE *pMsg)
{
    GUI_HWIN currt_w;
    GUI_HWIN next_w;
    int *key_press;
    int i;
    switch (pMsg->MsgId)
    {
    case WM_PAINT:
        LCD_L0_set_ram_buff();
        draw_sys_status_windosw(drones_w,"drones");
        GUI_SetBkColor(GUI_WHITE);
        GUI_ClearRect(0,16,143,167);
        LCD_L0_Refresh();
        LCD_L0_animation_Refresh(3,1);
        vTaskDelay(60); // 1000ms
        LCD_L0_animation_Refresh(3,2);
        vTaskDelay(50); // 1000ms
        LCD_L0_animation_Refresh(3,3);
        vTaskDelay(35); // 1000ms
        LCD_L0_animation_Refresh(3,4);
        vTaskDelay(20); // 1000ms
        LCD_L0_animation_Refresh(3,5);
        vTaskDelay(15); // 1000ms
        LCD_L0_animation_Refresh(3,6);
        sys_status.gui_update_delay=256;
        break;
    case WM_KEY:
        key_press=(int *)pMsg->Data.p;
        switch(*key_press)
        {
        case WM_USER_KEY_UP:
            WM_SelectWindow(drones_w);
            break;
        case WM_USER_KEY_DOWN:
            WM_SelectWindow(drones_w);
            break;
        case WM_USER_KEY_BACK:
            sys_status.status_bar_wind_handel=0;
            next_w=WM_GetParent(drones_w);
            WM_DeleteWindow(sys_status.status_bar_wind_handel);
            WM_DeleteWindow(drones_w);
            WM_SelectWindow(next_w);
            WM_SetFocus(next_w);
            break;
        }
        break;
    default:
        WM_DefaultProc(pMsg);
    }
}

void draw_drones_windosw(	GUI_HWIN Parent_w)
{
    drones_w = WM_CreateWindowAsChild(0, 0, 144, 168, Parent_w, WM_CF_SHOW, cb_drones_Win, 0);
    WM_SelectWindow(drones_w);
    WM_SetFocus(drones_w);
}

