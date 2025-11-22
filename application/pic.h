#ifndef __PIC_H
#define __PIC_H

#include <stdint.h>
#define LV_LVGL_H_INCLUDE_SIMPLE

#ifdef __has_include
    #if __has_include("lvgl.h")
        #ifndef LV_LVGL_H_INCLUDE_SIMPLE
            #define LV_LVGL_H_INCLUDE_SIMPLE
        #endif
    #endif
#endif

#if defined(LV_LVGL_H_INCLUDE_SIMPLE)
    #include "lvgl.h"
#else
    #include "lvgl/lvgl.h"
#endif

extern const lv_img_dsc_t main_ui;
extern const lv_img_dsc_t main_ui1;
extern const lv_img_dsc_t main_ui2;
extern const lv_img_dsc_t main_ui3;
extern const lv_img_dsc_t main_ui4;
extern const lv_img_dsc_t main_ui5;
extern const lv_img_dsc_t main_ui6;
extern const lv_img_dsc_t main_ui7;
extern const lv_img_dsc_t main_ui8;


extern const lv_img_dsc_t mon;
extern const lv_img_dsc_t mon_1;
extern const lv_img_dsc_t mon_2;


extern const lv_img_dsc_t set;
//extern const lv_img_dsc_t polic;
extern const lv_img_dsc_t set_tim;

extern const lv_img_dsc_t shan_red;
extern const lv_img_dsc_t shan_blu;
extern const lv_img_dsc_t dian;
extern const lv_img_dsc_t key;
extern const lv_img_dsc_t set_1;
extern const lv_img_dsc_t logo;
extern const lv_img_dsc_t red;

#endif













