#ifndef __APP_CONFIG_H
#define __APP_CONFIG_H
#include "app_times.h"


#define FLASH_START_ADDR                0x76000                                    //76000-78000    8KB 4KB/PAGE*2   /0-23000  140KB SOFTDEVICE   /23000-73000  320KB  APP                  
#define FLASH_END_ADDR                  0x78000                                    //78000-80000    32KB BOOTLOADER    


typedef struct
{
    unsigned char save_id_min;
    unsigned char save_id_max;
    unsigned char save_id_numb;
    unsigned char save_id_list[250];
} save_data_status_t;

typedef struct
{

    int  user_id;
    unsigned char user_name[16];
    unsigned char user_sex;
    unsigned char user_age;
    unsigned char user_height;
    unsigned char user_weight;
    unsigned char user_heart_rate_low;
    unsigned char user_heart_rate_mid;
    unsigned char auto_offline;
    unsigned char auto_offline_time;
    unsigned char boot_key_no_check;
    unsigned char bood_data[20];
    unsigned char wifi_name[36];
    unsigned char wifi_password[36];
    unsigned char animation;
    unsigned char wacth_face;
    unsigned char langvage;
    unsigned char others;
    save_data_status_t save_status;


} user_config_t;

typedef struct
{
    unsigned char boot_key_delay;
    unsigned char reset_key_delay;
    unsigned char  ble_adv_timeout;
    unsigned char  shutdown_key_delay;
    user_config_t  user;

} system_config_t;



typedef struct
{
    int   user_id;                             //当前用户ID
    unsigned char  save_id;                    //每个用户最大记录条数250条，最多10个用户，在设定用户时，检测储存空间，有几个用户数据，然后分配记录ID  3分钟以下数据无效
    unsigned int   start_time;
    unsigned short total_packages ;
    unsigned short current_package;
    unsigned short send_sector;
} upload_task_t;

typedef struct
{
    unsigned int   user_id;                    //当前用户ID
    unsigned char  save_id;                    //每个用户最大记录条数250条，最多10个用户，在设定用户时，检测储存空间，有几个用户数据，然后分配记录ID  3分钟以下数据无效
    unsigned short save_sector;


    unsigned char  max_save_id;            //已存在的最大ID
    unsigned short free_sectors;           //空闲扇区
    unsigned short free_sector_start;     //空闲扇区开始位置


} save_task_t;

typedef struct
{
    unsigned char  result;
    unsigned int   user_id[10];
    unsigned char  notes[25];

    unsigned short total_packages;
    unsigned short total_times;
    unsigned int complete_time;
} last_upload_status_t;







typedef struct
{
    volatile unsigned int   run_times;

    volatile unsigned char  led_task_status;

    volatile unsigned char  app_task_power;
    volatile unsigned char  app_task_bluetooth;
    volatile unsigned char  app_task_display;
    volatile unsigned char  app_task_sensor;

    volatile unsigned char current_percent;
    volatile unsigned char 	charger_status;
    volatile unsigned char usb_5v_status;
    volatile unsigned char  ble_tag_status;
    volatile unsigned char  bluetooth_status;

    volatile short  status_bar_wind_handel;
    volatile unsigned short  gui_update_delay;

    volatile unsigned char  notify_numbers;
    volatile unsigned char  notify_app;
    volatile unsigned char  notify_type;
    rtc_time_t       sys_times;

} system_status_t;


typedef struct
{
    unsigned char  font_size;
    unsigned char  font_type;

    unsigned char  dis_char;         //字间距
    unsigned char  dis_row;          //行间距

    unsigned char  font_color;          //反色

    unsigned char  have_status_bar;          //状态栏 /全屏     标题-时间- 消息-    reader-bookname--进度条        meassage--- time-- ble-battary

    unsigned char  next_page_mode;          //翻页模式 --手动--滚屏--自动
    unsigned char  next_page_speed;          //翻页速度

    unsigned char  keep_last_row;            //保留一行

    unsigned int   last_reading_book;   //最后一次阅读记录
    unsigned int   reading_page_addr;

} read_setting_t;

typedef struct
{
    unsigned char   mune_index;     //序号
    char   book_name[20];   //名称
    unsigned int    storage_size;	   //文件大小
    unsigned int    storage_start_addr;  //储存开始地址（扇区）
    unsigned int    storage_end_addr;    //储存结束地址（扇区）
    unsigned int    code_type;          //编码格式

    unsigned char    read_encipher;      //加密
    unsigned char    encipher_key[6];      //密码


    unsigned int    last_reading_addr;


    unsigned int    reading_addr_maker;
    char    reading_maker_text[20];


} book_information_t;


typedef struct
{
    book_information_t  books[10];  //最多10本书

    read_setting_t     setting;
} reader_status_t;

extern volatile  unsigned char  save_adv_name_flag;
extern volatile  unsigned char  save_system_config_flag;
extern volatile  unsigned char  save_user_config_flag;

extern reader_status_t  m_reader;
extern system_config_t  sys_config;
extern user_config_t    usr_config;
extern system_status_t  sys_status;

void save_books_config_to_flash(void);

void read_books_config_from_flash(void);

void app_config_init(void);
void app_config_loard(void);
void save_system_config_to_flash(void);
void save_user_config_to_flash(void);
void app_book_setting(void);

extern int current_show_mode;


#endif



