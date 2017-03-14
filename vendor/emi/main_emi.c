/*
 * Main_emi.c
 *
 *  Created on: Sep 1, 2014
 *      Author: qcmao
 */
#include "../../proj/tl_common.h"
#include "../../proj/mcu/watchdog_i.h"
#include "../../proj_lib/rf_drv.h"
#include "../common/emi.h"
#include "../common/rf_frame.h"
#include "../common/common.h"

//0280510f
typedef struct {
	u32 dma_len;            //won't be a fixed number as previous, should adjust with the mouse package number

	u8  rf_len;
	u8	proto;
	u8	flow;
	u8	type;

//	u32 gid;		//pipe0 code,	used as sync code for control pipe in hamster

	u8	rssi;
	u8	per;
	u8	seq_no;
	u8	pno;
	u8 data[30]; //30 : 400us(1MBPS)

}rf_packet_emi_t;

rf_packet_emi_t pkt_km = {
		sizeof (rf_packet_emi_t) - 4,	//0x10=20-4,dma_len

		sizeof (rf_packet_emi_t) - 5,	//0x0f=20-5,rf_len
		RF_PROTO_BYTE,						// 0x51, proto
		PKT_FLOW_DIR,						// 0x80, kb data flow
		FRAME_TYPE_KEYBOARD,				// 0x02, type

//		U32_MAX,			// gid0

		0,					// rssi
		0,					// per
		0,					// seq_no
		1,					// number of frame
};

#define LIGHT_IO_ENABLE    0
#if (LIGHT_IO_ENABLE)
#define LIGHT_IO    GPIO_PB6
void light_init(){
	gpio_set_func(LIGHT_IO, AS_GPIO); 
	gpio_set_input_en(LIGHT_IO, 0);
	gpio_set_output_en(LIGHT_IO, 1); 
	gpio_write(LIGHT_IO, 1);
}
#endif

enum{
    EMI_FRQ_L = 0,
    EMI_FRQ_M = 1,
    EMI_FRQ_H = 2, 
    EMI_FRQ_MODE_MAX, 
};

enum{
    EMI_MODE_CARRIER = 0,
    EMI_MODE_CD = 1,
    EMI_MODE_RX = 2,
    EMI_MODE_TX = 3,
    EMI_MODE_MAX,    
};

void main_loop(void)
{
#if(PA_ENABLE)
    pa_init(0, 0);
#endif

#if (LIGHT_IO_ENABLE)
    light_init();
#endif

	//default chn: low chn  (reference tbl_test_mode_freq[4])
	//EMI_FRQ_L  =  2402Mhz
	//EMI_FRQ_M = 2442Mhz
	//EMI_FRQ_H =  2480Mhz
	static s8   test_chn_idx = EMI_FRQ_L;
	
	//default mode: EMI_MODE_CARRIER(carrier mode)
	//EMI_MODE_CARRIER - 0
	//EMI_MODE_CD - 1
	//EMI_MODE_RX - 2
	//EMI_MODE_TX - 3
    static u8   test_mode_sel = EMI_MODE_CARRIER;

	//default: chn - 0/low chn, mode - 0/carrier mode, power - 0/8dbm
	emi_process( 1 , test_chn_idx, test_mode_sel, (u8 *)&pkt_km, RF_POWER_8dBm );

	write_reg8(0x808000,test_chn_idx);
	write_reg8(0x808001,test_mode_sel);

	while(1){
		if((test_chn_idx != read_reg8(0x808000))||(test_mode_sel != read_reg8(0x808001))){
			if((0x00<=read_reg8(0x808000))&&(read_reg8(0x808000)<0x03)&&(0x00<=read_reg8(0x808001))&&(read_reg8(0x808001)<0x04)){
				emi_process( 1 , read_reg8(0x808000), read_reg8(0x808001), (u8 *)&pkt_km, read_reg8(0x808002) );
				test_chn_idx = read_reg8(0x808000);
				test_mode_sel = read_reg8(0x808001);
			}
		}{
            tx_duty_handle((u8 *)&pkt_km);
		}
	}
}
