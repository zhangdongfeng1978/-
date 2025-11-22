#include "app_config.h"
#include "nrf_fstorage_sd.h"
#include <string.h>





system_config_t  sys_config;   //系统配置信息--开机延时/关机延时/广播延时
system_status_t  sys_status;

volatile  unsigned char  save_adv_name_flag=0;
volatile  unsigned char  save_system_config_flag=0;


static void callback(nrf_fstorage_evt_t * p_evt);

NRF_FSTORAGE_DEF(nrf_fstorage_t my_instance) =
{
    .evt_handler    = callback,
    .start_addr     = FLASH_START_ADDR,
    .end_addr       = FLASH_END_ADDR, //20KB
};


static void callback(nrf_fstorage_evt_t * p_evt)
{
    uint32_t  save_addr;
    if (p_evt->id==NRF_FSTORAGE_EVT_WRITE_RESULT) //p_evt->result==0 &&
    {
        save_system_config_flag=true;
    }
}

void app_test_init_book_setting(void)
{
//    strcpy(m_reader.books[0].book_name,"test");
//	m_reader.books[0].code_type=8;
//	m_reader.books[0].storage_start_addr=0;
//	m_reader.books[0].storage_size=3;
//	m_reader.books[0].mune_index=0;
//	m_reader.setting.dis_char=1;
//	m_reader.setting.dis_char=1;
//	m_reader.setting.dis_row=4;
//	m_reader.setting.font_color=1;
//	m_reader.setting.font_size=16;
//	m_reader.setting.font_type=1;
//	m_reader.setting.have_status_bar=1;
//	m_reader.setting.keep_last_row=1;
//	m_reader.setting.next_page_speed=0;
//	m_reader.setting.reading_page_addr=0;
//	m_reader.setting.last_reading_book=0;
}
void app_config_init(void)
{
    nrf_fstorage_init(&my_instance,  &nrf_fstorage_sd, NULL );
    app_config_loard();
}


void app_book_setting(void)
{
//    app_test_init_book_setting();
//    save_books_config_to_flash();
}


void save_system_config_to_flash(void) //77000
{
    uint32_t  timeout=1000000;
    save_system_config_flag=false;
//    memcpy(&flash_read_write_buf, &sys_config,sizeof(system_config_t));
    nrf_fstorage_erase( &my_instance,FLASH_START_ADDR, 1,NULL);
    while(save_system_config_flag==false&&timeout>10000)timeout--;
    save_system_config_flag=false;
    timeout=1000000;
//    nrf_fstorage_write(&my_instance,FLASH_START_ADDR+0x1000, (const uint32_t *)flash_read_write_buf, 12,NULL);
    nrf_fstorage_write(&my_instance,FLASH_START_ADDR, (const uint32_t *)&sys_config,sizeof(system_config_t),NULL);
    while(save_system_config_flag==false&&timeout>10000)timeout--;
}


void save_books_config_to_flash(void)
{
//    nrf_fstorage_write(&my_instance,FLASH_START_ADDR, (const uint32_t *)&m_reader, sizeof(reader_status_t),NULL);
//    while(save_user_config_flag==false&&timeout>10000)timeout--;
}

void read_books_config_from_flash(void)
{
//nrf_fstorage_read(&my_instance, FLASH_START_ADDR,  &m_reader,   sizeof(reader_status_t) );
}

void app_config_loard(void)
{
    uint32_t       err_code;
    int test_err;
    err_code = nrf_fstorage_read(&my_instance, FLASH_START_ADDR,  &sys_config,   sizeof(system_config_t) );
    if(err_code == 0 )
    {
        test_err=2;
    }
    sys_config.user.langvage=0;
}


