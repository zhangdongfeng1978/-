#ifndef __APP_LCD_H
#define __APP_LCD_H

#include <stdint.h>

//#define SCL		NRF_GPIO_PIN_MAP(1,10)
//#define SDI		NRF_GPIO_PIN_MAP(1,13)
//#define CS		NRF_GPIO_PIN_MAP(1,14)

//#define RST_PIN    NRF_GPIO_PIN_MAP(1,12)   //¸´Î»Òý½Å
//#define DC_PIN     NRF_GPIO_PIN_MAP(1,11)   //DCÒý½Å

//#define SCL		NRF_GPIO_PIN_MAP(0,19)
//#define SDI		NRF_GPIO_PIN_MAP(0,20)
//#define CS		NRF_GPIO_PIN_MAP(0,17)

//#define RST_PIN    NRF_GPIO_PIN_MAP(0,02)
//#define DC_PIN     NRF_GPIO_PIN_MAP(0,22)


//#define LCD_PWR_EN			NRF_GPIO_PIN_MAP(0,31)

//#define BACKLIGHT_PWM		NRF_GPIO_PIN_MAP(0,29)



#define SCL_		NRF_GPIO_PIN_MAP(0,19)
#define SDI_		NRF_GPIO_PIN_MAP(0,20)
#define CS		NRF_GPIO_PIN_MAP(0,17)

#define RST_PIN    NRF_GPIO_PIN_MAP(1,00)
#define DC_PIN     NRF_GPIO_PIN_MAP(0,22)


//#define LCD_PWR_EN			NRF_GPIO_PIN_MAP(0,01)

//#define BACKLIGHT_PWM		NRF_GPIO_PIN_MAP(0,06)




void Tft_Init(void);
void Set_Clocr1(void);
void Tft_Color_Fill(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t *colour);
void Tft_fill_area_screen(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color);
void Lvgl_tft_fill_area_screen(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t* color);


#endif



