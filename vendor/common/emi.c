/*
 * emi.c
 *
 *  Created on: Feb 14, 2014
 *      Author: xuzhen
 */
#include "../../proj/tl_common.h"
#include "../../proj/mcu/watchdog_i.h"
#include "../../proj_lib/rf_drv.h"
#include "emi.h"
#include "../common/common.h"

void rf_send_single_packet (void* addr);

u8 emi_var[8];

const u8	tbl_test_mode_channel[4] = {host_cmd_chn_l, host_cmd_chn_m, host_cmd_chn_h};
const u8	tbl_test_mode_sel[4] = {host_cmd_carrier, host_cmd_cd, host_cmd_rx, host_cmd_tx};
const u8	tbl_test_mode_freq[4] = {2, 42, 80};

#define MODE_TABBLE         0
#define MODE_FREQ           1

#define SET_FREQ_MODE       MODE_FREQ

#define FIFO_DEPTH			(1*16)
#define STATE0				0x1234
#define STATE1				0x5678
#define STATE2				0xabcd
#define STATE3				0xef01

typedef struct{
    u32 start;
#if (FIFO_DEPTH > 0x08)
    u32 data[(FIFO_DEPTH - 8)>>2];
#endif
    u32 end;
    u32 cd_feed[2];
} fifo_emi_t;

#if 0
#if	(MCU_CORE_TYPE && MCU_CORE_TYPE == MCU_CORE_8266)
#define		PKT_BUFF_SIZE		256
#else
#define		PKT_BUFF_SIZE		80
#endif
#endif
#if 0
extern unsigned char rf_rx_buff[];
fifo_emi_t *fifo_emi = rf_rx_buff;          //re-use rf_rx_buff, PKT_BUFF_SIZE*2 must larger than sizeof (fifo_emi_t)
#else
fifo_emi_t fifo_emi_buff;
fifo_emi_t *fifo_emi = &fifo_emi_buff;
#endif
int pnGen(int state)
{
	int feed = 0;
	feed = (state&0x4000) >> 1;
	state ^= feed;
	state <<= 1;
	state = (state&0xfffe) + ((state&0x8000)>>15);
	return state;
}
void emi_cd_data_gen( void ){
    fifo_emi->end = (fifo_emi->cd_feed[0]<<16) + fifo_emi->cd_feed[1];
    //advance PN generator
    fifo_emi->cd_feed[0] = pnGen(fifo_emi->cd_feed[0]);
    fifo_emi->cd_feed[1] = pnGen(fifo_emi->cd_feed[1]);
}

static inline void emi_fifo_init( void ){
    fifo_emi->start = FIFO_DEPTH + 1;
    fifo_emi->end = STATE3;
    fifo_emi->cd_feed[0] = STATE0;
    fifo_emi->cd_feed[1] = STATE1;
}

static u8 test_mode, test_mode_chn;

u32 tx_time;

void emi_process (u8 cmd, u8 chn_idx, u8 test_mode_sel, u8 *tx_pkt, u8 tx_power ) {
    static u8   flg_emi_init = 0;
    if( !flg_emi_init ){
        emi_init( tx_power );
        flg_emi_init = 1;
        #if (SET_FREQ_MODE == MODE_TABBLE)
        SetTxMode ( tbl_test_mode_channel[0], RF_CHN_TABLE | RF_SET_TX_MANAUL ); //rx mode may fail dongle cd
        #else
        SetTxMode ( tbl_test_mode_freq[0], RF_SET_TX_MANAUL ); //rx mode may fail dongle cd
        #endif
    }
    
    if (cmd) {  //test mode or channel changed
        #if (SET_FREQ_MODE == MODE_TABBLE)
        test_mode_chn = tbl_test_mode_channel[chn_idx];
        #else
        test_mode_chn = tbl_test_mode_freq[chn_idx];
        #endif
        test_mode = tbl_test_mode_sel[test_mode_sel];
        if (TEST_MODE_CARRIER) { //carry                
            emi_carrier_init ();
            //rf_set_channel (test_mode_chn, RF_CHN_TABLE | RF_SET_TX_MANAUL );
            #if (SET_FREQ_MODE == MODE_TABBLE)
            rf_set_channel (test_mode_chn, RF_CHN_TABLE );
            #else
            rf_set_channel (test_mode_chn, 0 );
            #endif
            rf_set_txmode();
            sleep_us(200);
            emi_carrier_generate();                  
        }
        else{
            emi_carrier_recovery();
        }

        if (TEST_MODE_CD) {      //carry + data
            emi_fifo_init();
            emi_cd_prepare();
            WaitUs(1000);                       //must delay
            //SetTxMode (test_mode_chn, RF_CHN_TABLE | RF_SET_TX_MANAUL);
            #if (SET_FREQ_MODE == MODE_TABBLE)
            rf_set_channel (test_mode_chn, RF_CHN_TABLE );
            #else
            rf_set_channel (test_mode_chn, 0 );
            #endif
            rf_set_txmode();
            sleep_us(200);
            emi_cd_init ( (u32)(&fifo_emi->start) );
        }
        else{
            emi_cd_recovery();
        }

        if (TEST_MODE_RX) {     //rx
            #if (SET_FREQ_MODE == MODE_TABBLE)
            SetRxMode (test_mode_chn, RF_CHN_TABLE);
            #else
            SetRxMode (test_mode_chn, 0);
            #endif
        }
        else if (TEST_MODE_TX) { //tx
            #if (SET_FREQ_MODE == MODE_TABBLE)
            SetTxMode (test_mode_chn, RF_CHN_TABLE);
            #else
            SetTxMode (test_mode_chn, 0);
            #endif
        }
        
#if(PA_ENABLE)
        if(TEST_MODE_RX){
            pa_txrx(PA_RX);
        }else{
            pa_txrx(PA_TX);
        }
#endif
    }

    if (TEST_MODE_TX) {
        #if (SET_FREQ_MODE == MODE_TABBLE)
        SetTxMode (test_mode_chn, RF_CHN_TABLE);
        #else
        SetTxMode (test_mode_chn, 0);
        #endif
        rf_send_single_packet(tx_pkt);
        tx_time = clock_time();
        //rf_send_packet (tx_pkt, 350, 0);
    }
    
    static u32 emi_tick;
    while (!clock_time_exceed (emi_tick, 8000)) {
        if (TEST_MODE_CD) {
            emi_cd_data_gen ();
        }
        if (TEST_MODE_RX) {
            //rf_recv_pkt ();
        }
    }
    emi_tick = clock_time ();
}

void tx_duty_handle(u8 *tx_pkt){
    if (TEST_MODE_TX){
        if(clock_time_exceed (tx_time, 600)){
            tx_time = clock_time ();
            rf_send_single_packet(tx_pkt); 
        }
    }
}

