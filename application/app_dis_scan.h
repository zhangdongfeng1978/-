#ifndef __APP_GUI_DISPLAY_H
#define __APP_GUI_DISPLAY_H


typedef struct
{
    unsigned char mac_addr[6];
    unsigned char   primary_phy;
    unsigned char   secondary_phy;
    char   tx_power;
    signed char   rssi;
    unsigned char   ch_index;
    unsigned char   adv_data[32];
    unsigned short   adv_len;
} scan_devices_t;


extern scan_devices_t d_list[20];
extern scan_devices_t tag_list[7];
extern char devices_numbers;

extern void scan_start(void);
extern void fast_scan_start(void);
extern void scan_stop(void);
extern void connect_scan_Peripheral(unsigned char Peripheral_index);
extern void send_cmd_to_Peripheral(unsigned char *cmd_string,unsigned short string_len);
extern unsigned char drones_status[8];
extern unsigned char periph_numbers;
extern char scan_complete;
extern short ecg_buff[250];
extern int ecg_buff_index;
extern unsigned char heart_rate;
extern unsigned short hrv_ssd;
extern unsigned short hrv_rmssd;
extern unsigned char display_win;
extern unsigned int body_steps;
extern int ble_tag_devices_detection(void);
extern int ble_tag_rssi_detection(void);


#endif



