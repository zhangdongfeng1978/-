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
#include "app_lcd.h"

// SPI 实例
static const nrfx_spim_t spim = NRFX_SPIM_INSTANCE(3);//NRFX_SPIM_INSTANCE(0);

void lcd_gpio_init(void) {
    nrf_gpio_cfg_output(DC_PIN);
    nrf_gpio_cfg_output(RST_PIN);
		nrf_gpio_cfg_output(CS);
		nrf_gpio_pin_clear(CS);
}

void st7789_write_command(uint8_t cmd) {
    nrf_gpio_pin_write(DC_PIN, 0);          // DC=0 写命令
    nrfx_spim_xfer_desc_t desc = NRFX_SPIM_XFER_TX(&cmd, 1);
    nrfx_spim_xfer(&spim, &desc, 0);
}

void st7789_write_data(uint8_t data){
    nrf_gpio_pin_write(DC_PIN, 1);          // DC=1 写数据
    nrfx_spim_xfer_desc_t desc = NRFX_SPIM_XFER_TX(&data, 1);
    nrfx_spim_xfer(&spim, &desc, 0);
}

// 发送多个数据
void tft_write_data_buf(uint8_t *data, uint16_t len) {
    nrf_gpio_pin_write(DC_PIN, 1);          // DC=1 写数据
		nrfx_spim_xfer_desc_t desc = NRFX_SPIM_XFER_TX(&data, len);
		nrfx_spim_xfer(&spim, &desc, 0);
}


// 设置显示区域（全屏）
void tft_set_window(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2) {
		st7789_write_command(0x2A);  // Column Address Set
		st7789_write_data((x1 >> 8) & 0xFF);
		st7789_write_data((x1+0x23) & 0xFF);
		st7789_write_data((x2 >> 8) & 0xFF);
		st7789_write_data((x2+0x23) & 0xFF);
		//st7789_write_command(0x2c);	//储存器写
	
		st7789_write_command(0x2B);  // Column Address Set
		st7789_write_data((y1 >> 8) & 0xFF);
		st7789_write_data(y1 & 0xFF);
		st7789_write_data((y2 >> 8) & 0xFF);
		st7789_write_data(y2 & 0xFF);
    st7789_write_command(0x2C);	//储存器写
}

//170*320*2
void st7789_fill_screen(uint16_t color) {
    st7789_write_command(0x2C);                 //写显存命令
    nrf_gpio_pin_write(DC_PIN, 1);
    uint8_t buffer[340];												//单行缓冲区  170 * 2 = 340
		memset(buffer, 0, 340);
		uint8_t lo = (uint8_t)((color>>8) & 0xff);
		uint8_t hi = (uint8_t)(color & 0xFF);
    for (int i = 0; i < 340; i+=2){							// 填充单行 (170像素=340字节)
        buffer[i] =lo;
        buffer[i + 1] = hi;
    }
		nrfx_spim_xfer_desc_t desc = NRFX_SPIM_XFER_TX(buffer, 340);
		for (int y = 0; y < 320; y++) {         //遍历320行
			nrfx_spim_xfer(&spim, &desc, 0);
		}
}

void Tft_fill_area_screen(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color)
{
		tft_set_window(x1, y1, x2, y2);
    st7789_write_command(0x2C);                 //写显存命令
    nrf_gpio_pin_write(DC_PIN, 1);
	
		uint8_t buffer[340];												//单行缓冲区 最大 170 * 2 = 340
		memset(buffer, 0, 340);
		uint8_t lo = (uint8_t)((color>>8) & 0xff);
		uint8_t hi = (uint8_t)(color & 0xFF);
		uint16_t row = (x2-x1+1)*2;
    for (int i = 0; i < row; i+=2){							// 填充单行 (170像素=340字节)
        buffer[i] =lo;
        buffer[i + 1] = hi;
    }
		uint16_t column =(y2-y1+1);
		nrfx_spim_xfer_desc_t desc = NRFX_SPIM_XFER_TX(buffer, row);
		for (int y = 0; y < column; y++) {
			nrfx_spim_xfer(&spim, &desc, 0);
		}
	
//    uint8_t buffer[340];												//单行缓冲区  170 * 2 = 340
//		memset(buffer, 0, 340);
//		uint8_t lo = (uint8_t)((color>>8) & 0xff);
//		uint8_t hi = (uint8_t)(color & 0xFF);
//    for (int i = 0; i < 340; i+=2){							// 填充单行 (170像素=340字节)
//        buffer[i] =lo;
//        buffer[i + 1] = hi;
//    }
//		nrfx_spim_xfer_desc_t desc = NRFX_SPIM_XFER_TX(buffer, 340);
//		for (int y = 0; y < 320; y++) {         //遍历320行
//			nrfx_spim_xfer(&spim, &desc, 0);
//		}
}


void Lvgl_tft_fill_area_screen(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t* color)
{
		tft_set_window(x1, y1, x2, y2);
    st7789_write_command(0x2C);                 //写显存命令
    nrf_gpio_pin_write(DC_PIN, 1);
	
		uint8_t buffer[340 * 80];//uint8_t buffer[340 * 160];//uint8_t buffer[340 * 320]; //uint8_t buffer[340];												//单行缓冲区 最大 170 * 2 = 340
		memset(buffer,0,sizeof(buffer));

		uint16_t row = (x2-x1+1);			//行数据
		uint16_t column =(y2-y1+1);		//列数据
	
	
		#if 1
		//170 * 320
		for (int y = 0, j=0; y < column; y++) {         							//遍历320行
			for(int i = 0 ; i < row; i++){							//填充单行 (170像素=340字节)
					buffer[j] = (uint8_t)(( color[y*row+i] >>8) & 0xFF);
					buffer[j + 1] = (uint8_t)( color[y*row+i] & 0xFF);
					j+=2;
			}
		}
		nrfx_spim_xfer_desc_t desc = NRFX_SPIM_XFER_TX(buffer, row*2*column);
		nrfx_spim_xfer(&spim, &desc, 0);
		memset(buffer,0,sizeof(buffer));
		#endif
		
		
		
		
	
#if 0
		for (int y = 0; y < column; y++) {         							//遍历320行
			for(int i = 0, j=0; i < row; i++, j+=2){							//填充单行 (170像素=340字节)
					buffer[j] = (uint8_t)(( color[y*row+i] >>8) & 0xFF);
					buffer[j + 1] = (uint8_t)( color[y*row+i] & 0xFF);
			}
			nrfx_spim_xfer_desc_t desc = NRFX_SPIM_XFER_TX(buffer, row*2);
			nrfx_spim_xfer(&spim, &desc, 0);
			memset(buffer,0,sizeof(buffer));
		}
#endif
		
		
		
#if 0
		uint16_t *color_t =color;
		for (int y = 0; y < column; y++) {	//遍历320行
			//for(int i = 0; i < row*2; i++){	//填充单行 (170像素=340字节)
			for(int i = 0; i < row; i++){	//填充单行 (170像素=340字节)
					//buffer[i] = (uint8_t)(( color[column*column+i] >>8) & 0xFF);
					//buffer[i + 1] = (uint8_t)( color[column*column+i] & 0xFF);
				
				//	buffer[i] = (uint8_t)(( color[y*row+i] >>8) & 0xFF);
				//	buffer[i + 1] = (uint8_t)( color[y*row+i] & 0xFF);
				
				
//					nrfx_spim_xfer_desc_t desc = NRFX_SPIM_XFER_TX(color_t, 1);
//					nrfx_spim_xfer(&spim, &desc, 0);
//					color_t++;
						uint16_t  data = *color_t;
						uint8_t  data1[2]={0};
						data1[0] =(uint8_t)((data>>8) & 0xFF);
						data1[1] = (uint8_t)(data & 0xFF);
						nrfx_spim_xfer_desc_t desc = NRFX_SPIM_XFER_TX(data1, 2);
						nrfx_spim_xfer(&spim, &desc, 0);
						color_t++;
			}

		}
#endif

		#if 0  //成功
		uint16_t *color_t =color;
		for (int y = 0; y < column; y++) {	//遍历320行
			//for(int i = 0; i < row*2; i++){	//填充单行 (170像素=340字节)
			for(int i = 0; i < row; i++){	//填充单行 (170像素=340字节)
					//buffer[i] = (uint8_t)(( color[column*column+i] >>8) & 0xFF);
					//buffer[i + 1] = (uint8_t)( color[column*column+i] & 0xFF);
				
				//	buffer[i] = (uint8_t)(( color[y*row+i] >>8) & 0xFF);
				//	buffer[i + 1] = (uint8_t)( color[y*row+i] & 0xFF);
				
				
//					nrfx_spim_xfer_desc_t desc = NRFX_SPIM_XFER_TX(color_t, 1);
//					nrfx_spim_xfer(&spim, &desc, 0);
//					color_t++;
				
						uint16_t  data = *color_t;
						uint8_t  data1 =(uint8_t)((data>>8) & 0xff);
						uint8_t  data2 = (uint8_t)(data & 0xFF);
						nrfx_spim_xfer_desc_t desc = NRFX_SPIM_XFER_TX(&data1, 1);
						nrfx_spim_xfer(&spim, &desc, 0);
						nrfx_spim_xfer_desc_t desc1 = NRFX_SPIM_XFER_TX(&data2, 1);
						nrfx_spim_xfer(&spim, &desc1, 0);
						color_t++;
			}
		}
		#endif

}


void lcd_spi_init(void) {
    nrfx_spim_config_t config = NRFX_SPIM_DEFAULT_CONFIG;
		config.mode = NRF_SPIM_MODE_0;  //模式
    config.frequency = NRF_SPIM_FREQ_32M;//NRF_SPIM_FREQ_32M;  // 8MHz SPI时钟
    config.mosi_pin = SDI_ ;           // 设置MOSI引脚
    config.sck_pin = SCL_ ;             // 设置SCK引脚
		config.ss_pin = CS ;               //硬件CS
    APP_ERROR_CHECK(nrfx_spim_init(&spim, &config, NULL, NULL));
}

void Tft_Init(void)
{
		lcd_gpio_init();
		lcd_spi_init();
    // 硬件复位
    nrf_gpio_pin_write(RST_PIN, 0);
    nrf_delay_ms(10);
    nrf_gpio_pin_write(RST_PIN, 1);
    nrf_delay_ms(120);

		st7789_write_command(0x11);     
		nrf_delay_ms(120);                

		st7789_write_command(0x36);     
		st7789_write_data(0x00);

		st7789_write_command(0x3A);     
		st7789_write_data(0x05);   

		st7789_write_command(0xB2);     
		st7789_write_data(0x0B);   
		st7789_write_data(0x0B);   
		st7789_write_data(0x00);   
		st7789_write_data(0x33);   
		st7789_write_data(0x33);   

		st7789_write_command(0xB7);     
		st7789_write_data(0x75);   

		st7789_write_command(0xBB);     
		st7789_write_data(0x28);   

		st7789_write_command(0xC0);     
		st7789_write_data(0x2C);   

		st7789_write_command(0xC2);     
		st7789_write_data(0x01);   

		st7789_write_command(0xC3);     
		st7789_write_data(0x1F);    

		st7789_write_command(0xC6);     
		st7789_write_data(0x13);    

		st7789_write_command(0xD0);     
		st7789_write_data(0xA7);    

		st7789_write_command(0xD0);     
		st7789_write_data(0xA4);   
		st7789_write_data(0xA1);   

		st7789_write_command(0xD6);     
		st7789_write_data(0xA1);  

		st7789_write_command(0xE0);     
		st7789_write_data(0xF0);
		st7789_write_data(0x05);
		st7789_write_data(0x0A);
		st7789_write_data(0x06);
		st7789_write_data(0x06);
		st7789_write_data(0x03);
		st7789_write_data(0x2B);
		st7789_write_data(0x32);
		st7789_write_data(0x43);
		st7789_write_data(0x36);
		st7789_write_data(0x11);
		st7789_write_data(0x10);
		st7789_write_data(0x2B);
		st7789_write_data(0x32);

		st7789_write_command(0xE1);
		st7789_write_data(0xF0);
		st7789_write_data(0x08);
		st7789_write_data(0x0C);
		st7789_write_data(0x0B);
		st7789_write_data(0x09);
		st7789_write_data(0x24);
		st7789_write_data(0x2B);
		st7789_write_data(0x22);
		st7789_write_data(0x43);
		st7789_write_data(0x38);
		st7789_write_data(0x15);
		st7789_write_data(0x16);
		st7789_write_data(0x2F);
		st7789_write_data(0x37);
		st7789_write_command(0x21);     
		st7789_write_command(0x29);	//开启显示
		st7789_write_command(0x2C);
		nrf_delay_ms(120);
}

void Set_Clocr1(void)
{
	#if 0
		//170 * 320
		tft_set_window(0,0, 170-1, 320-1);
		//st7789_fill_screen(0xF800);  	//红 R
		//st7789_fill_screen(0x07E0);	//绿 G
		st7789_fill_screen(0x001F);	//蓝 B
	#else
//		tft_set_window(0,0, 85-1, 160-1);
//		st7789_fill_screen(0xF800);  //红色 (RGB565: 0xF800)
//		tft_set_window(85-1,0, 170-1, 160-1);
//		st7789_fill_screen(0x07E0);	 //绿色 (RGB565: 0x07E0)
//		tft_set_window(0,160-1, 170-1, 320-1);
//		st7789_fill_screen(0x001F);  //蓝色 (RGB565: 0x001F)
	
	
		Tft_fill_area_screen(0,0, 84, 159,0xF800);
		Tft_fill_area_screen(85,0, 169, 159, 0x07E0);
		Tft_fill_area_screen(0,160, 169, 319,0x001F);
		
	#endif
	
	
}

/*
    st7789_write_command(0x2C);                 //写显存命令
    nrf_gpio_pin_write(DC_PIN, 1);
    uint8_t buffer[340];												//单行缓冲区  170 * 2 = 340
		memset(buffer, 0, 340);
    uint16_t *buf_ptr = (uint16_t*)buffer;
    for (int i = 0; i < 170; i++) {							// 填充单行 (170像素=340字节)
        buf_ptr[i] =color;
    }
    nrfx_spim_xfer_desc_t desc = NRFX_SPIM_XFER_TX(buffer, 340);
    for (int y = 0; y < 320; y++) {         //遍历320行
      nrfx_spim_xfer(&spim, &desc, 0);
    }
*/


void Tft_Color_Fill(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t *colour)
{
	
//Lvgl_tft_fill_area_screen(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t* color);
	
	Lvgl_tft_fill_area_screen( x1,  y1,  x2,  y2, colour);

	
	#if 0
    uint8_t buffer[340];												//单行缓冲区  170 * 2 = 340
		memset(buffer, 0, 340);
		uint8_t lo = (uint8_t)((color>>8) & 0xff);
		uint8_t hi = (uint8_t)(color & 0xFF);
    for (int i = 0; i < 340; i+=2){							// 填充单行 (170像素=340字节)
        buffer[i] =lo;
        buffer[i + 1] = hi;
    }
		nrfx_spim_xfer_desc_t desc = NRFX_SPIM_XFER_TX(buffer, 340);
		for (int y = 0; y < 320; y++) {         //遍历320行
			nrfx_spim_xfer(&spim, &desc, 0);
		}
	#endif
	
}



















