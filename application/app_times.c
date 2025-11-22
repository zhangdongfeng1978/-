#include "app_times.h"
#include "bsp_rtc.h"


#include "app_config.h"
#include "nrf_drv_timer.h"
#define FEBRUARY		2
#define	STARTOFTIME		1970
#define SECDAY			86400L
#define SECYR			(SECDAY * 365)
#define	leapyear(year)		(((year) % 4 == 0&&(year) % 100 != 0)||(year) % 400 == 0)
#define	days_in_year(a) 	(leapyear(a) ? 366 : 365)
#define	days_in_month(a) 	(month_days[(a) - 1])



static unsigned int last_seting_times =  0;
rtc_time_t last_seting;
weather_data_t  m_weather;
//remind_data_t  m_remind;
int stop_timer_count;
int dwon_timer_count;
unsigned char alarm_staus;


const unsigned int lunar200y[199] =
{
    0x04AE53, 0x0A5748, 0x5526BD, 0x0D2650, 0x0D9544, 0x46AAB9, 0x056A4D, 0x09AD42, 0x24AEB6, 0x04AE4A,/*1901-1910*/
    0x6A4DBE, 0x0A4D52, 0x0D2546, 0x5D52BA, 0x0B544E, 0x0D6A43, 0x296D37, 0x095B4B, 0x749BC1, 0x049754,/*1911-1920*/
    0x0A4B48, 0x5B25BC, 0x06A550, 0x06D445, 0x4ADAB8, 0x02B64D, 0x095742, 0x2497B7, 0x04974A, 0x664B3E,/*1921-1930*/
    0x0D4A51, 0x0EA546, 0x56D4BA, 0x05AD4E, 0x02B644, 0x393738, 0x092E4B, 0x7C96BF, 0x0C9553, 0x0D4A48,/*1931-1940*/
    0x6DA53B, 0x0B554F, 0x056A45, 0x4AADB9, 0x025D4D, 0x092D42, 0x2C95B6, 0x0A954A, 0x7B4ABD, 0x06CA51,/*1941-1950*/
    0x0B5546, 0x555ABB, 0x04DA4E, 0x0A5B43, 0x352BB8, 0x052B4C, 0x8A953F, 0x0E9552, 0x06AA48, 0x6AD53C,/*1951-1960*/
    0x0AB54F, 0x04B645, 0x4A5739, 0x0A574D, 0x052642, 0x3E9335, 0x0D9549, 0x75AABE, 0x056A51, 0x096D46,/*1961-1970*/
    0x54AEBB, 0x04AD4F, 0x0A4D43, 0x4D26B7, 0x0D254B, 0x8D52BF, 0x0B5452, 0x0B6A47, 0x696D3C, 0x095B50,/*1971-1980*/
    0x049B45, 0x4A4BB9, 0x0A4B4D, 0xAB25C2, 0x06A554, 0x06D449, 0x6ADA3D, 0x0AB651, 0x093746, 0x5497BB,/*1981-1990*/
    0x04974F, 0x064B44, 0x36A537, 0x0EA54A, 0x86B2BF, 0x05AC53, 0x0AB647, 0x5936BC, 0x092E50, 0x0C9645,/*1991-2000*/
    0x4D4AB8, 0x0D4A4C, 0x0DA541, 0x25AAB6, 0x056A49, 0x7AADBD, 0x025D52, 0x092D47, 0x5C95BA, 0x0A954E,/*2001-2010*/
    0x0B4A43, 0x4B5537, 0x0AD54A, 0x955ABF, 0x04BA53, 0x0A5B48, 0x652BBC, 0x052B50, 0x0A9345, 0x474AB9,/*2011-2020*/
    0x06AA4C, 0x0AD541, 0x24DAB6, 0x04B64A, 0x69573D, 0x0A4E51, 0x0D2646, 0x5E933A, 0x0D534D, 0x05AA43,/*2021-2030*/
    0x36B537, 0x096D4B, 0xB4AEBF, 0x04AD53, 0x0A4D48, 0x6D25BC, 0x0D254F, 0x0D5244, 0x5DAA38, 0x0B5A4C,/*2031-2040*/
    0x056D41, 0x24ADB6, 0x049B4A, 0x7A4BBE, 0x0A4B51, 0x0AA546, 0x5B52BA, 0x06D24E, 0x0ADA42, 0x355B37,/*2041-2050*/
    0x09374B, 0x8497C1, 0x049753, 0x064B48, 0x66A53C, 0x0EA54F, 0x06B244, 0x4AB638, 0x0AAE4C, 0x092E42,/*2051-2060*/
    0x3C9735, 0x0C9649, 0x7D4ABD, 0x0D4A51, 0x0DA545, 0x55AABA, 0x056A4E, 0x0A6D43, 0x452EB7, 0x052D4B,/*2061-2070*/
    0x8A95BF, 0x0A9553, 0x0B4A47, 0x6B553B, 0x0AD54F, 0x055A45, 0x4A5D38, 0x0A5B4C, 0x052B42, 0x3A93B6,/*2071-2080*/
    0x069349, 0x7729BD, 0x06AA51, 0x0AD546, 0x54DABA, 0x04B64E, 0x0A5743, 0x452738, 0x0D264A, 0x8E933E,/*2081-2090*/
    0x0D5252, 0x0DAA47, 0x66B53B, 0x056D4F, 0x04AE45, 0x4A4EB9, 0x0A4D4C, 0x0D1541, 0x2D92B5          /*2091-2099*/
};

static int month_days[12] = {	31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };
static int monthTotal[13] = { 0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334, 365 };

unsigned char CaculateMonDays(rtc_time_t * tm)
{
    unsigned char days;
    if (leapyear(tm->tm_year)&&tm->tm_mon==2)
    {
        days= 29;
    }
    else days = days_in_month(tm->tm_mon);
    return days;
}
unsigned char Caculate_n_days_in_year(rtc_time_t * tm)
{
    unsigned char i, indays;
    int MonthOffset[] = { 0,31,59,90,120,151,181,212,243,273,304,334 };
    indays=tm->tm_mday;
    for(i=tm->tm_mon; i>0; i--)
    {
        indays+= MonthOffset[i];
    }
    if (leapyear(tm->tm_year))
    {
        indays += 1;
    }
    return indays;
}
unsigned char CaculateWeekDay(rtc_time_t * tm)
{
    unsigned char weekday;
    int leapsToDate;
    int lastYear;
    int day;
    int MonthOffset[] = { 0,31,59,90,120,151,181,212,243,273,304,334 };
    lastYear=tm->tm_year-1;
    /*计算从公元元年到计数的前一年之中一共经历了多少个闰年*/
    leapsToDate = lastYear/4 - lastYear/100 + lastYear/400;
    /*如若计数的这一年为闰年，且计数的月份在2月之后，则日数加1，否则不加1*/
    if((tm->tm_year%4==0) &&
            ((tm->tm_year%100!=0) || (tm->tm_year%400==0)) &&
            (tm->tm_mon>2))
    {
        /*
         * We are past Feb. 29 in a leap year
         */
        day=1;
    }
    else
    {
        day=0;
    }
    day += lastYear*365 + leapsToDate + MonthOffset[tm->tm_mon-1] + tm->tm_mday; /*计算从公元元年元旦到计数日期一共有多少天*/
    weekday=day%7;
    return weekday;
}

unsigned char CaculateWeeksInYear(rtc_time_t * tm)
{
    unsigned char weekday,ndays,weeks;
    unsigned int int_t1,int_t2;
    rtc_time_t temp_s;
    int_t1=tm_to_int(tm);
    temp_s.tm_year=tm->tm_year;
    temp_s.tm_mon=1;
    temp_s.tm_mday=1;
    temp_s.tm_hour=tm->tm_hour;
    temp_s.tm_min=tm->tm_min;
    temp_s.tm_sec=tm->tm_sec;
    int_t2 = tm_to_int(&temp_s);
    weekday = CaculateWeekDay(&temp_s);
    if(weekday>3)
    {
        ndays=7-weekday;
        int_t2 = int_t2+ndays*SECDAY;
    }
    else
    {
        ndays = weekday;
        int_t2 = int_t2-ndays*SECDAY;
    }
    if(int_t1<int_t2)
    {
        weeks=0;
    }
    else
    {
        weeks =   (int_t1-int_t2)/(7*SECDAY)+1;
    }
    return weeks;
}


void int_to_tm(unsigned int tim, rtc_time_t * tm)
{
    register unsigned int    i;
    register long   hms, day;
    day = tim / SECDAY;			/* 有多少天 */
    hms = tim % SECDAY;			/* 今天的时间，单位s */
    /* Hours, minutes, seconds are easy */
    tm->tm_hour = hms / 3600;
    tm->tm_min = (hms % 3600) / 60;
    tm->tm_sec = (hms % 3600) % 60;
    /* Number of years in days */ /*算出当前年份，起始的计数年份为1970年*/
    for (i = STARTOFTIME; day >= days_in_year(i); i++)
    {
        day -= days_in_year(i);
    }
    tm->tm_year = i;
    if (leapyear(tm->tm_year))
    {
        days_in_month(FEBRUARY) = 29;
    }
    for (i = 1; day >= days_in_month(i); i++)
    {
        day -= days_in_month(i);
    }
    days_in_month(FEBRUARY) = 28;
    tm->tm_mon = i;
    tm->tm_mday = day + 1;
    tm->tm_wday=CaculateWeekDay(tm);
}

unsigned int tm_to_int(rtc_time_t *tm)
{
    rtc_time_t temp_s;
    temp_s.tm_year=tm->tm_year;
    temp_s.tm_mon=tm->tm_mon;
    temp_s.tm_mday=tm->tm_mday;
    temp_s.tm_hour=tm->tm_hour;
    temp_s.tm_min=tm->tm_min;
    temp_s.tm_sec=tm->tm_sec;
    if (0 >= (int) (temp_s.tm_mon -= 2))  	/* 1..12 -> 11,12,1..10 */
    {
        temp_s.tm_mon += 12;		/* Puts Feb last since it has leap day */
        temp_s.tm_year -= 1;
    }
    return (((
                 (unsigned int) (temp_s.tm_year/4 - temp_s.tm_year/100 + temp_s.tm_year/400 + 367*temp_s.tm_mon/12 + temp_s.tm_mday) +
                 temp_s.tm_year*365 - 719499
             )*24 + temp_s.tm_hour /* now have hours */
            )*60 + temp_s.tm_min /* now have minutes */
           )*60 + temp_s.tm_sec;//-8*60*60; /* finally seconds */
    /*Add by fire: -8*60*60 把输入的北京时间转换为标准时间，
    再写入计时器中，确保计时器的数据为标准的UNIX时间戳*/
}


void app_times_setting(rtc_time_t m_set_tm)
{
    unsigned int i,temp_cc[4],temp_last,temp_now;
    int temp_dif;
    temp_last=last_seting_times;  //上次基准时间
    temp_now= bsp_rtc_get_counter()/16;  //当前计数时间
    last_seting_times=tm_to_int(&m_set_tm)-temp_now; //设置时间-计数器时间 =基准时间
//    temp_dif=last_seting_times-temp_last;  //新基准时间-上次基准时间=调整时间段
//     temp_dif= temp_dif%86400;              //24小时 调整了多少分钟
//
//    for(i=0; i<4; i++)
//    {
//        temp_cc[i]= bsp_rtc_get_cc(i);
//        if( temp_cc[i]!=0)
//        {
//            temp_cc[i]= temp_cc[i]-temp_dif*16;
//            bsp_rtc_set_cc(i,temp_cc[i],1);
//        }
//    }
}


//void app_alarm_setting(rtc_time_t alarm_set_tm,unsigned char enable,unsigned char cc_chanel)
//{
//    unsigned int now_connter, now_temp,set_temp;
//	int diff;
//    now_connter= bsp_rtc_get_counter()/16;  //当前相对计数时间
//    now_temp=now_connter+last_seting_times;  //当前绝对计数值
//    set_temp=tm_to_int(&alarm_set_tm);  //设置绝对时间

//	diff=set_temp-now_temp;
//
//    if(diff<=0) diff=diff+SECDAY;


//    set_temp=(now_connter+diff)*16;

//    if(set_temp>16777215)set_temp=set_temp-16777215;


//    if(enable>0)
//    {
//        bsp_rtc_set_cc(cc_chanel,set_temp,1);
//    }
//    else
//        bsp_rtc_set_cc(cc_chanel,0,0);



//}

void rtc_overflow_times_setting(void)
{
    unsigned int i;
    last_seting_times=last_seting_times+1048575;  //更新计时器
}


//void rtc_alram_timeout_handler( unsigned int ch)
//{
//    if(ch==0)
//    {
//        display_notif_flag=1;
//        last_notif.notify_type=NOTIF_SYS_ALARM;
//        alarm_staus=1;

//        bsp_rtc_set_cc(ch,0,0);
//    }
//    else if
//    (ch==1)
//    {
//        display_notif_flag=1;
//        last_notif.notify_type=NOTIF_SYS_ALARM;
//        alarm_staus=2;
//        bsp_rtc_set_cc(ch,0,0);
//    }
//    else if
//    (ch==2)
//    {
//        display_notif_flag=1;
//        last_notif.notify_type=NOTIF_SYS_ALARM;
//        alarm_staus=3;
//        bsp_rtc_set_cc(ch,0,0);

//    }
//    else if
//    (ch==3)
//    {
//        display_notif_flag=1;
//        last_notif.notify_type=NOTIF_SYS_ALARM;
//        alarm_staus=4;
//        bsp_rtc_set_cc(ch,0,0);

//    }

//}
void Time_Call(rtc_time_t *tm)   //计算当前时间，每分钟一次
{
    unsigned int temp;
    temp= bsp_rtc_get_counter()/16;
    temp= last_seting_times+temp;
    int_to_tm(temp,tm);
}


void Time_Call_Ndays(char ndays,rtc_time_t *tm_n)   //计算当前时间gen天日期，
{
    unsigned int temp;
    temp= bsp_rtc_get_counter()/16;
    if(ndays<128)
    {
        temp= last_seting_times+temp+ndays*SECDAY;
    }
    else
    {
        temp= last_seting_times+temp-(256-ndays)*SECDAY;
    }
    int_to_tm(temp,tm_n);
}


//时间初始化函数，获取RTC2计数器值,加上1970，为当前值，
void app_times_init(void)
{
    bsp_rtc_init();
    last_seting.tm_year=2020;
    last_seting.tm_mon=8;
    last_seting.tm_mday=23;
    last_seting.tm_hour=23;
    last_seting.tm_min=59;
    last_seting.tm_sec=0;
    last_seting_times=tm_to_int(&last_seting);
    m_weather.n_day=6;
}





Mydate toSolar(Mydate lunar)
{
    int year = lunar.year,
        month = lunar.month,
        day = lunar.day;
    int byNow, xMonth, i;
    Mydate solar;
    byNow = (lunar200y[year - 1901] & 0x001F) - 1;
    if (((lunar200y[year - 1901] & 0x0060) >> 5) == 2)
        byNow += 31;
    for (i = 1; i < month; i++)
    {
        if ((lunar200y[year - 1901] & (0x80000 >> (i - 1))) == 0)
        {
            byNow += 29;
        }
        else
            byNow += 30;
    }
    byNow += day;
    xMonth = (lunar200y[year - 1901] & 0xf00000) >> 20;
    if (xMonth != 0)
    {
        if (month > xMonth
                || (month == xMonth && lunar.reserved == 1))
        {
            if ((lunar200y[year - 1901] & (0x80000 >> (month - 1))) == 0)
                byNow += 29;
            else
                byNow += 30;
        }
    }
    if (byNow > 366|| (year % 4 != 0 && byNow == 365))
    {
        year += 1;
        if (year % 4 == 0)
            byNow -= 366;
        else
            byNow -= 365;
    }
    for (i = 1; i <= 13; i++)
    {
        if (monthTotal[i] >= byNow)
        {
            month = i;
            break;
        }
    }
    solar.day = byNow - monthTotal[month - 1];
    solar.month = month;
    solar.year = year;
    return solar;
}

Mydate toLunar(Mydate solar)
{
    int year = solar.year,
        month = solar.month,
        day = solar.day;
    int bySpring, bySolar, daysPerMonth;
    int index, flag;
    Mydate lunar;
    //bySpring 记录春节离当年元旦的天数。
    //bySolar 记录阳历日离当年元旦的天数。
    if (((lunar200y[year - 1901] & 0x0060) >> 5) == 1)
        bySpring = (lunar200y[year - 1901] & 0x001F) - 1;
    else
        bySpring = (lunar200y[year - 1901] & 0x001F) - 1 + 31;
    bySolar = monthTotal[month - 1] + day - 1;
    if ((!(year % 4)) && (month > 2))
        bySolar++;
    //daysPerMonth记录大小月的天数 29 或30
    //index 记录从哪个月开始来计算。
    //flag 是用来对闰月的特殊处理。
    //判断阳历日在春节前还是春节后
    if (bySolar >= bySpring) //阳历日在春节后（含春节那天）
    {
        bySolar -= bySpring;
        month = 1;
        index = 1;
        flag = 0;
        if ((lunar200y[year - 1901] & (0x80000 >> (index - 1))) == 0)
            daysPerMonth = 29;
        else
            daysPerMonth = 30;
        while (bySolar >= daysPerMonth)
        {
            bySolar -= daysPerMonth;
            index++;
            if (month == ((lunar200y[year - 1901] & 0xF00000) >> 20))
            {
                flag = ~flag;
                if (flag == 0)
                    month++;
            }
            else
                month++;
            if ((lunar200y[year - 1901] & (0x80000 >> (index - 1))) == 0)
                daysPerMonth = 29;
            else
                daysPerMonth = 30;
        }
        day = bySolar + 1;
    }
    else  //阳历日在春节前
    {
        bySpring -= bySolar;
        year--;
        month = 12;
        if (((lunar200y[year - 1901] & 0xF00000) >> 20) == 0)
            index = 12;
        else
            index = 13;
        flag = 0;
        if ((lunar200y[year - 1901] & (0x80000 >> (index - 1))) == 0)
            daysPerMonth = 29;
        else
            daysPerMonth = 30;
        while (bySpring > daysPerMonth)
        {
            bySpring -= daysPerMonth;
            index--;
            if (flag == 0)
                month--;
            if (month == ((lunar200y[year - 1901] & 0xF00000) >> 20))
                flag = ~flag;
            if ((lunar200y[year - 1901] & (0x80000 >> (index - 1))) == 0)
                daysPerMonth = 29;
            else
                daysPerMonth = 30;
        }
        day = daysPerMonth - bySpring + 1;
    }
    lunar.day = day;
    lunar.month = month;
    lunar.year = year;
    if (month == ((lunar200y[year - 1901] & 0xF00000) >> 20))
        lunar.reserved = 1;
    else
        lunar.reserved = 0;
    return lunar;
}


static int stop_timer_start_count=1;
static int dwon_timer_start_count=1;
void timer_stop_event_handler(nrf_timer_event_t event_type, void* p_context)
{
    //uint32_t led_to_invert = ((i++) % LEDS_NUMBER);
    switch (event_type)
    {
    case NRF_TIMER_EVENT_COMPARE0:
        if(stop_timer_start_count)
            stop_timer_count++;
        break;
    default:
        //Do nothing.
        break;
    }
}

void timer_dwon_event_handler(nrf_timer_event_t event_type, void* p_context)
{
    //uint32_t led_to_invert = ((i++) % LEDS_NUMBER);
    switch (event_type)
    {
    case NRF_TIMER_EVENT_COMPARE0:
        if(dwon_timer_start_count)
            dwon_timer_count++;
        break;
    default:
        //Do nothing.
        break;
    }
}
//void stop_timer_start(void)
//{
//    uint32_t time_ms = 10; //Time(in miliseconds) between consecutive compare events.
//    uint32_t time_ticks;
//    uint32_t err_code = NRF_SUCCESS;
//    //Configure TIMER_LED for generating simple light effect - leds on board will invert his state one after the other.
//    nrf_drv_timer_config_t timer_cfg = NRF_DRV_TIMER_DEFAULT_CONFIG;


//    err_code = nrf_drv_timer_init(&STOP_TIMER, &timer_cfg, timer_stop_event_handler);
//    APP_ERROR_CHECK(err_code);

//    time_ticks = nrf_drv_timer_ms_to_ticks(&STOP_TIMER, time_ms);

//    nrf_drv_timer_extended_compare( &STOP_TIMER, NRF_TIMER_CC_CHANNEL0, time_ticks, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, true);

//    nrf_drv_timer_enable(&STOP_TIMER);
//    stop_timer_start_count=1;
//}
//void dwon_timer_start(void)
//{
//    uint32_t time_ms = 10; //Time(in miliseconds) between consecutive compare events.
//    uint32_t time_ticks;
//    uint32_t err_code = NRF_SUCCESS;
//    //Configure TIMER_LED for generating simple light effect - leds on board will invert his state one after the other.
//    nrf_drv_timer_config_t timer_cfg = NRF_DRV_TIMER_DEFAULT_CONFIG;


//    err_code = nrf_drv_timer_init(&DWON_TIMER, &timer_cfg, timer_dwon_event_handler);
//    APP_ERROR_CHECK(err_code);

//    time_ticks = nrf_drv_timer_ms_to_ticks(&DWON_TIMER, time_ms);

//    nrf_drv_timer_extended_compare( &DWON_TIMER, NRF_TIMER_CC_CHANNEL0, time_ticks, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, true);

//    nrf_drv_timer_enable(&DWON_TIMER);
//    dwon_timer_start_count=1;
//}
//void stop_timer_stop(void)
//{
//    nrf_drv_timer_uninit(&STOP_TIMER);
//    stop_timer_count=0;
//    stop_timer_start_count=0;
//}
//void dwon_timer_stop(void)
//{
//    nrf_drv_timer_uninit(&DWON_TIMER);
//    dwon_timer_count=0;
//    dwon_timer_start_count=0;
//}
//void stop_timer_pause(void)
//{
//    stop_timer_start_count=0;
//}
//void dwon_timer_pause(void)
//{
//    dwon_timer_start_count=0;
//}
//void stop_timer_continue(void)
//{
//    stop_timer_start_count=1;
//}
//void dwon_timer_continue(void)
//{
//    dwon_timer_start_count=1;
//}
//int stop_timer_get(void)
//{
//    return stop_timer_count;

//}

//int dwon_timer_get(void)
//{
//    return dwon_timer_count;

//}


