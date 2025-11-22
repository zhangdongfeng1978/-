#ifndef __APP_TIMES_H
#define __APP_TIMES_H

typedef struct
{
    int year;
    int month;
    int day;
    int reserved;
} Mydate;


typedef struct
{
    int tm_sec;
    int tm_min;
    int tm_hour;
    int tm_mday;
    int tm_mon;
    int tm_year;
    int tm_wday;
} rtc_time_t;

typedef struct
{
    unsigned char n_day;
    unsigned char type[7];
    unsigned char max_temp[7];
    unsigned char min_temp[7];
    unsigned char city_name[15];

} weather_data_t;


//typedef struct
//{
//    unsigned char ble;
//	unsigned char sport;
//	unsigned char notify;   //如果有任意推送打开，该项为开
//    unsigned char alarm;
//    unsigned char open_hour;
//    unsigned char open_min;
//    unsigned char close_hour;
//    unsigned char close_min;
//    unsigned char ancs_call;
//    unsigned char ancs_message;
//    unsigned char ancs_wechat;
//    unsigned char ancs_qq;
//    unsigned char ancs_other;
//
//}remind_data_t;


extern unsigned char alarm_staus;
//extern remind_data_t  m_remind;
extern weather_data_t  m_weather;
void rtc_alram_timeout_handler( unsigned int ch);
void int_to_tm(unsigned int tim, rtc_time_t * tm);
unsigned int tm_to_int(rtc_time_t *tm);
void rtc_overflow_times_setting(void);
void app_times_setting(rtc_time_t m_set_tm);
void Time_Call(rtc_time_t *tm);
void Time_Call_Ndays(char ndays,rtc_time_t *tm_n);   //计算当前时间gen天日期
void app_alarm_setting(rtc_time_t alarm_set_tm,unsigned char mask,unsigned char cc_chanel);
unsigned char CaculateMonDays(rtc_time_t * tm);
unsigned char CaculateWeekDay(rtc_time_t * tm);
unsigned char CaculateWeeksInYear(rtc_time_t * tm);
//转阳历
Mydate toSolar(Mydate lunar);
//转农历
Mydate toLunar(Mydate solar);

void app_times_init(void);

//void stop_timer_start(void);
//void stop_timer_stop(void);
//void stop_timer_pause(void);
//void stop_timer_continue(void);
//int stop_timer_get(void);

//
//void dwon_timer_start(void);
//void dwon_timer_stop(void);
//void dwon_timer_pause(void);
//void dwon_timer_continue(void);
//int dwon_timer_get(void);
#endif



