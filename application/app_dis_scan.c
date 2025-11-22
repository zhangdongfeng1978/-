#include "app_dis_scan.h"
#include "bsp.h"
#include "FreeRTOS.h"
#include "task.h"
#include "app_config.h"

#include "app_display.h"
#include <math.h>
#include <string.h>
#include "app_times.h"
#include "app_sensor.h"

char devices_numbers=0;
char scan_complete=0;
scan_devices_t d_list[20];

scan_devices_t tag_list[7];
int ble_tag_numbers=0;

int ble_tag_devices_detection(void)
{
    int is_tag=0;
    if(scan_complete!=1)return 0;

    for(int i=0; i<devices_numbers; i++ )
    {
        if(d_list[i].adv_len==21&&d_list[i].adv_data[1]==0x16&&d_list[i].adv_data[3]==0xFD)//HUAWEI
        {
            memcpy(&tag_list[is_tag].mac_addr[0],&d_list[i].mac_addr[0],6);
            d_list[is_tag].tx_power=d_list[i].tx_power;
            d_list[is_tag].rssi=d_list[i].rssi;
            is_tag++;
        }
        else if(d_list[i].adv_len==21&&d_list[i].adv_data[2]==0x8F)//xiaomi
        {
            memcpy(&tag_list[is_tag].mac_addr[0],&d_list[i].mac_addr[0],6);
            d_list[is_tag].tx_power=d_list[i].tx_power;
            d_list[is_tag].rssi=d_list[i].rssi;
            is_tag++;
        }
				else if(d_list[i].adv_len==31&&d_list[i].adv_data[2]==0x4C)//Findtag
        {
            memcpy(&tag_list[is_tag].mac_addr[0],&d_list[i].mac_addr[0],6);
            d_list[is_tag].tx_power=d_list[i].tx_power;
            d_list[is_tag].rssi=d_list[i].rssi;
            is_tag++;
        }
				else if(d_list[i].adv_len > 11 && d_list[i].adv_data[2] == 0x06 && d_list[i].adv_data[5] == 0xE0 && d_list[i].adv_data[6] == 0xFF &&\
					      d_list[i].adv_data[9] == 0x05 && d_list[i].adv_data[10] == 0x01)//itag
        {
            memcpy(&tag_list[is_tag].mac_addr[0],&d_list[i].mac_addr[0],6);
            d_list[is_tag].tx_power=d_list[i].tx_power;
            d_list[is_tag].rssi=d_list[i].rssi;
            is_tag++;
        }
        else if(d_list[i].adv_data[2]==0x44&&d_list[i].adv_data[3]==0xFD)//other
        {
            memcpy(&tag_list[is_tag].mac_addr[0],&d_list[i].mac_addr[0],6);

            d_list[is_tag].tx_power=d_list[i].tx_power;
            d_list[is_tag].rssi=d_list[i].rssi;
            is_tag++;
        }
    }
    ble_tag_numbers=is_tag;
    return is_tag;
}


int ble_tag_rssi_detection(void)
{
	int max_rssi=-90;
	int have_update=0;
	if(scan_complete!=1)return 0;
	for(int i=0; i<devices_numbers; i++ ){
		for(int j=0; j<ble_tag_numbers; j++){
			if(memcmp(&tag_list[j].mac_addr[0],&d_list[i].mac_addr[0],6)==0){
				tag_list[j].rssi=d_list[i].rssi;
				have_update=1;
			}
		}
	}
  if(have_update){
    for(int j=0; j<ble_tag_numbers; j++){
			if(max_rssi< tag_list[j].rssi){
				max_rssi=tag_list[j].rssi;;
			}
    }
	}else{
		max_rssi=0;
  }

	return max_rssi;
}







