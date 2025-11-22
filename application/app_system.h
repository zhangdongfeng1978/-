#ifndef __APP_MODE_H
#define __APP_MODE_H


extern int manger_win_react;
extern unsigned char update_advdata_flag;
extern void advertising_start(void);
extern void advertising_stop(void);
extern void advertising_data_update(void);
extern void bluetooth_disconnect(void);
void app_system_init(void);
void app_system_ble_process(void);

#endif



