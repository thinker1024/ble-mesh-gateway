#include "../../proj/tl_common.h"
#include "../../proj/mcu/watchdog_i.h"
#include "../../proj_lib/rf_drv.h"
#include "../../proj_lib/pm.h"
#include "../../proj_lib/light_ll/light_ll.h"
#include "../../proj_lib/light_ll/light_frame.h"
#include "../../proj_lib/ble_ll/service.h"
#include "../../proj_lib/ble_ll/gatt_server.h"
#include "../common/common.h"

FLASH_ADDRESS_EXTERN;

ll_adv_private_t adv_pri_data = {
	VENDOR_ID,
	VENDOR_ID,
	0,
};
extern u8	adv_private_data_len;

ll_adv_rsp_private_t adv_rsp_pri_data = {
	VENDOR_ID,
	VENDOR_ID,
	0,
	0x1234,
	0x01,
};

void vendor_rf_led_ota_ok(void){
	return;
}

void vendor_rf_led_ota_error(void){
	return;
}

void vendor_irq_timer1(void){
	return;
}

void vendor_proc_led(void){
	return;
}

void vendor_rf_link_data_callback (void *p){
	return;
}

void vendor_user_init(void){
	return;
}

void vendor_set_adv_data(void){
	// config adv data
	extern u8 *slave_p_mac;
	adv_pri_data.MacAddress = *(u32 *)slave_p_mac;
	rf_link_slave_set_adv_private_data((u8*)(&adv_pri_data), sizeof(ll_adv_private_t));
    #if ADV_UUID
	extern void rf_link_slave_set_adv_uuid_data (u8 * pdata, u8 data_len);
	rf_link_slave_set_adv_uuid_data(adv_uuid, sizeof(adv_uuid));
    #endif
    adv_rsp_pri_data.ProductUUID = LIGHT_MODE;

	// config adv rsp data
	adv_rsp_pri_data.MacAddress = *(u32 *)slave_p_mac;
	foreach(i, 16){
	    adv_rsp_pri_data.rsv[i] = i;
	}
}

void set_vendor_function(void){
	return;

	p_vendor_rf_led_ota_ok 			= &vendor_rf_led_ota_ok;
	p_vendor_rf_led_ota_error		= &vendor_rf_led_ota_error;
	p_vendor_irq_timer1				= &vendor_irq_timer1;
	p_vendor_proc_led 				= &vendor_proc_led;
	p_vendor_rf_link_data_callback	= &vendor_rf_link_data_callback;
	p_vendor_user_init				= &vendor_user_init;

	u8 devName[] = {DEVICE_NAME};
	extern u8 ble_g_devName_light[];
	memcpy(ble_g_devName_light, devName, strlen((char *)devName) > 13? 13:strlen((char *)devName));

	extern attribute_t gAttributes_vendor[];
	extern attribute_t *p_vendor_att;
	p_vendor_att = gAttributes_vendor;

	//init some parameters
    extern void software_version();
    software_version();
}

