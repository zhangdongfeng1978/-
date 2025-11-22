#include "app_display.h"
#include "app_config.h"
int input_index=0;
int last_input_index=0;
GUI_HWIN input_w;
WM_MESSAGE soft_key_msg;
int key_bord=0;
int input_key_vlue;
int input_times=0;
static const char *key_board_text[2][30] =
{

    "q",
    "w",
    "e",
    "r",
    "t",
    "y",
    "u",
    "i",
    "o",
    "p",
    " ", //CAPSLAOCK
    "a",
    "s",
    "d",
    "f",
    "g",
    "h",
    "j",
    "k",
    "l",
    " ",  //ENTER
    "z",
    "x",
    "c",
    "v",
    "b",
    "n",
    "m",
    ".",
    " ",  //DELETE
    "Q",
    "W",
    "E",
    "R",
    "T",
    "Y",
    "U",
    "I",
    "O",
    "P",
    " ", //CAPSLAOCK
    "A",
    "S",
    "D",
    "F",
    "G",
    "H",
    "J",
    "K",
    "L",
    " ",  //ENTER
    "Z",
    "X",
    "C",
    "V",
    "B",
    "N",
    "M",
    ".",
    " ",  //DELETE
};




static void redraw_key_status(int key_index)
{
    WM_SelectWindow(input_w);
    GUI_SetBkColor(GUI_BLACK);
    GUI_SetColor(GUI_WHITE);
    GUI_FillChamferRect(4+key_index%10*33,4+(key_index/10)*33,34+key_index%10*33,34+(key_index/10)*33,2);
    GUI_SetBkColor(GUI_WHITE);
    GUI_SetColor(GUI_BLACK);
    if(key_index<10)
        GUI_DispDecAt(key_index,24+key_index%10*33,18+(key_index/10)*33,1);
    GUI_DispStringHCenterAt(key_board_text[key_bord][key_index],19+key_index%10*33,10+(key_index/10)*33);
}
static void redraw_key_status_Invert(int key_index)
{
    WM_SelectWindow(input_w);
    GUI_InvertChamferRect(5+key_index%10*33,5+(key_index/10)*33,33+key_index%10*33,33+(key_index/10)*33,2);
}
int last_input;
static void cb_input_Win(WM_MESSAGE *pMsg)
{
    GUI_HWIN currt_w;
    GUI_HWIN next_w;
    int *key_press;
    int i;
    switch (pMsg->MsgId)
    {
    case WM_PAINT:
        GUI_SetColor(GUI_WHITE);
        GUI_FillChamferRect(0,0,24,168,3);  //336 136
        GUI_SetBkColor(GUI_WHITE);
        GUI_SetColor(GUI_BLACK);
//        input_index=0;
//        last_input_index=0;
//        //  redraw_key_status(input_index);
//        for (int i=0; i<30; i++)
//            redraw_key_status(i);
//
        GUI_SetFont(&GUI_Font16_ASCII);
        for (int i=0; i<10; i++)
            GUI_DispCharAt(*key_board_text[0][i],10,i*16+5);  //dir
        GUI_InvertChamferRect(2,input_index*16+5,22,input_index*16+21,2);
        last_input=input_index;
        LCD_L0_Refresh();
        break;
    case WM_KEY:
        key_press=(int *)pMsg->Data.p;
        switch(*key_press)
        {
        case WM_USER_KEY_UP:
//redraw_key_status_Invert(input_index);
            if(input_index>0)
            {
                input_index--;
            }
            else
            {
                input_index=29;
            }
            GUI_InvertChamferRect(2,last_input*16+5,22,last_input*16+21,2);
            GUI_InvertChamferRect(2,input_index*16+5,22,input_index*16+21,2);
            last_input=input_index;
            //   redraw_key_status_Invert(input_index);
            LCD_L0_Refresh();
            break;
        case WM_USER_KEY_DOWN:
            //   redraw_key_status_Invert(input_index);
            if(input_index<29)
            {
                input_index++;
            }
            else
            {
                input_index=0;
            }
            GUI_InvertChamferRect(2,last_input*16+5,22,last_input*16+21,2);
            GUI_InvertChamferRect(2,input_index*16+5,22,input_index*16+21,2);
            last_input=input_index;
//redraw_key_status_Invert(input_index);
            LCD_L0_Refresh();
            break;
        case WM_USER_KEY_ENTER:
            if(input_index==10)
            {
                if(key_bord==0)
                    key_bord=1;
                else
                    key_bord=0;
                for (int i=0; i<30; i++)
                    redraw_key_status(i);
                redraw_key_status_Invert(input_index);
                LCD_L0_Refresh();
            }
            else   //·¢ËÍ¼üÖµ
            {
                soft_key_msg.MsgId=WM_SOFT_KEY_MSGID;
                soft_key_msg.Data.p=&key_board_text[key_bord][input_index];
                WM_SendToParent(input_w,&soft_key_msg);
            }
            LCD_L0_Refresh();
            break;
        case WM_USER_KEY_UP_LONG:
            redraw_key_status_Invert(input_index);
            if(input_index>=10)
            {
                input_index=input_index-10;
            }
            redraw_key_status_Invert(input_index);
            LCD_L0_Refresh();
            break;
        case WM_USER_KEY_DOWN_LONG:
            redraw_key_status_Invert(input_index);
            if(input_index<20)
            {
                input_index=input_index+10;
            }
            redraw_key_status_Invert(input_index);
            LCD_L0_Refresh();
            break;
        case WM_USER_KEY_BACK:
            sys_status.status_bar_wind_handel=0;
            next_w=WM_GetParent(input_w);
            WM_DeleteWindow(sys_status.status_bar_wind_handel);
            WM_DeleteWindow(input_w);
            WM_SelectWindow(next_w);
            WM_SetFocus(next_w);
            break;
        }
        break;
    default:
        WM_DefaultProc(pMsg);
    }
}

void draw_input_window(GUI_HWIN Parent_hwin)
{
    input_w = WM_CreateWindowAsChild(120,0, 24, 168, Parent_hwin, WM_CF_SHOW, cb_input_Win, 0);
    WM_SelectWindow(input_w);
    WM_SetFocus(input_w);
}
