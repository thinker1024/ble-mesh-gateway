#include "../../proj/tl_common.h"
#include "../../proj/mcu/watchdog_i.h"
#include "../../proj_lib/rf_drv.h"
#include "../../proj_lib/pm.h"
#include "../../proj_lib/ble_ll/ble_ll.h"
#include "../../proj_lib/light_ll/light_frame.h"
#include "../../proj/drivers/keyboard.h"
#include "../common/rf_frame.h"
#include "../common/tl_audio.h"
#include "trace.h"
#include "../common/common.h"

#ifndef			DEBUG_USB_MODE
#define			DEBUG_USB_MODE					0
#endif

#define			EP_BO							5
FLASH_ADDRESS_EXTERN;

void	ble_master_update_adv (u8 * p);
void rc_led_en (int en, int fre);
void airmouse_enable(int en);
void	ble_master_set_golden ();

rf_packet_att_data_t	ota_buffer[8];
u8 *	p_firmware = 0;
int		host_ota_start = 0;
u8		ota_hl = 0;

///////////////////////////////////////////////////////////////////////////
rf_custom_master_dat_t 	rf_custom_dat;
extern kb_data_t	kb_event;
u32					tick_kb;
int					sys_kb = 0;
int					sys_am = 0;
u8					mouse_button = 0;

int					conn_interval = 24;				// 6 * 1.25 = 7.5 ms
int					conn_timeout = 1000;			// 1 second
int					conn_start = 1;

/////////////////// rf interrupt handler ///////////////////////////
_attribute_ram_code_ void irq_handler(void)
{
	irq_ble_master_handler ();
}

void	ble_master_data_callback (u8 *p)
{
	static	u32 bdbg_data;
	bdbg_data++;

	//////////////////////////////////////
	int n = p[9] + 8;   // p[9]:rf_len;  8 = 2 + 3(crc) + 2(dc) + 1(rssi)
	reg_usb_ep8_dat = n;
	reg_usb_ep8_dat = 0;
	for (int i=0; i<p[9]+5; i++)
	{
		reg_usb_ep8_dat = p[10+i];
	}
	reg_usb_ep8_dat = p[0]; // rssi
	reg_usb_ep8_ctrl = BIT(7);
	///////////////////////////////////////
}

void ble_event_callback (u8 status, u8 *p, u8 rssi)
{
	static u32 bdbg_event;
	bdbg_event++;

	//////////////////////////////////////////////
	int n = 0;
	if (p)
	{
		n = *p++;
	}
	reg_usb_ep8_dat = n + 8;
	reg_usb_ep8_dat = status;
	for (int i=0; i<n+5; i++)
	{
		reg_usb_ep8_dat = *p++;
	}
	reg_usb_ep8_dat = rssi;
	reg_usb_ep8_ctrl = BIT(7);
	///////////////////////////////////////////////
}


//////////////////////////////////////////////////////////
//	USB interfuace BI/BO
//////////////////////////////////////////////////////////
u8	buff_command[64];
int host_write ()
{
	static u32 no_cmd;
	no_cmd++;
	int n = reg_usb_ep_ptr (EP_BO);
	reg_usb_ep_ptr(EP_BO) = 0;
	for (int i=0; i<n; i++)
	{
		buff_command[i] = reg_usb_ep_dat(EP_BO);
	}

	u8 cmd = buff_command[0];

	if (cmd == 0 && n > 0)		// data
	{
		ble_master_ll_data (buff_command + 1, n - 1);
	}
	else if (cmd == 1)	// set mac
	{
		ble_master_set_slave_mac (buff_command + 1);
	}
	else if (cmd == 2)	// start master
	{
		conn_start = 1;
	}
	else if (cmd == 3)	// stop master
	{
		conn_start = 0;
		ble_master_stop ();
	}
	else if (cmd == 4)	// set connection parameters
	{
		ble_set_debug_adv_channel (buff_command[1]);
		//ble_master_set_channel_mask (buff_command + 6);
		conn_interval = buff_command[2] + buff_command[3]*256;
		conn_timeout = buff_command[4] + buff_command[5]*256;
		ble_master_update_adv (buff_command + 2);
	}
	else if (cmd == 5)	// set ota handle and start
	{
		host_ota_start = 1;
		ota_hl = buff_command[1] ? buff_command[1] : 0x18;
	}
	else if (cmd == 6)	//
	{
		master_smp_func_init ();
	}
	else if (cmd == 7)	// set ota handle and start
	{
		host_ota_start = 1;
		ota_hl = 0x0a;
	}

	return 0;
}

void proc_host ()
{
	static u32	tick_bulk_out;
	//////////// host interface change  //////////////////////////////////
	if (reg_usb_irq & BIT(EP_BO)) {
		host_write ();
		reg_usb_irq = BIT(EP_BO);
		reg_usb_ep_ctrl(EP_BO) = BIT(0);
	}
	else if (reg_usb_ep_ctrl(EP_BO) & FLD_USB_EP_BUSY)
	{
		tick_bulk_out = clock_time ();
	}
	else if (clock_time_exceed (tick_bulk_out, 1000000))
	{
		reg_usb_ep_ctrl(EP_BO) = BIT(0);
	}

}

//////////////////////////////////////////////////////////
// debug mode
//////////////////////////////////////////////////////////
void sys_enable_led ()
{
	if (sys_kb && sys_am)
	{
		rc_led_en (1, 2);
	}
	else if (!sys_kb && !sys_am)
	{
		rc_led_en (0, 2);
	}
	else
	{
		rc_led_en (1, 4);
	}
}

void sys_enable_kb (int en)
{
	sys_kb = en;
	//gpio_write (GPIO_PC5, en);
	sys_enable_led ();
}

int debug_am;
void sys_enable_airmouse (int en)
{
	debug_am++;
	sys_am = en;
	airmouse_enable (en);
	sys_enable_led ();
}

/////////////////////////////////////////////////////////////
//	work with PC software
/////////////////////////////////////////////////////////////
void proc_suspend (void)
{
	static u32 tick_10ms;
	if (sys_kb || sys_am || !clock_time_exceed (tick_kb, 2000000))		// mic or airmouse on
	{
		while (!clock_time_exceed (tick_10ms, 10000));
		tick_10ms = clock_time ();
	}
	else
	{
		usb_dp_pullup_en (0);
		cpu_sleep_wakeup (1, PM_WAKEUP_PAD , 0) ;
	}
}

#define		reg_debug_cmd		REG_ADDR16(0x8008)
void	proc_debug () {
    return ;    // can not use 0x8004~0x800d
    
	u16	udat = reg_debug_cmd;
	u8	cmd = udat >> 8;
	u8  adr = udat;
	u8	dat;
	if (cmd == 0xff) {			//read command
		dat = analog_read (adr);
		reg_debug_cmd = dat | 0x5500;
	}
	else if (cmd <= 0x20 || (cmd>=0x80 && cmd <=0xc0)) {	//write command
		analog_write (cmd, adr);
		dat = analog_read (cmd);
		reg_debug_cmd = dat | 0x6600;
	}
	else if (cmd == 0xfe) {	//set channel mask
		//chn_mask_fix = adr;
		//reg_debug_cmd = adr | 0x6700;
	}
	else if (cmd == 0xfd) {	//set tx power
		rf_set_power_level_index (adr & 15);
		reg_debug_cmd = adr | 0x6800;
	}

}

void proc_ota ()
{
	if (! ble_master_status ())
	{
		host_ota_start = 0;
	}
	//ble_master_add_tx_packet;
	static u32 n_firmware = 0;
	static u32 ota_adr = 0;
	static u32	tick_page;
	static u32	tick_10s;
	if (host_ota_start == 1)
	{
		n_firmware = *(u32 *)(flash_adr_ota_master+0x18);
		if(n_firmware > 0x30000){
            host_ota_start = 0;
            ble_event_callback (FLG_SYS_LINK_LOST, 0, 0);
            return ;
		}else{
    		host_ota_start = 2;
    		ota_adr = 0;
    		tick_page = clock_time ();
    		tick_10s = 0;
		}
	}
	else if (host_ota_start == 2)
	{
		if(clock_time_exceed(tick_page, 10*1000*1000)){
			tick_page = clock_time ();
			tick_10s ++;
			if (tick_10s > 100)
			{
				host_ota_start = 0;
				return;
			}
		}
		int idx = (ota_adr >> 4) & 7;
		rf_packet_att_data_t *p = &ota_buffer[idx];

		int nlen = ota_adr < n_firmware ? 16 : 0;

		p->type = 2;
		p->l2cap = 7 + nlen;
		p->chanid = 0x04;
		p->att = ATT_OP_WRITE_CMD;
		p->hl = ota_hl;
		p->hh = 0x0;
		p->dat[0] = ota_adr>>4;
		p->dat[1] = ota_adr>>12;
		if(nlen == 16){
			memcpy(p->dat + 2, p_firmware + ota_adr, 16);
		}else{
			memset(p->dat + 2, 0, 16);
		}
		u16 crc = crc16(p->dat, 2+nlen);
		p->dat[nlen + 2] = crc;
		p->dat[nlen + 3] = crc >> 8;
		p->rf_len = p->l2cap + 4;
		p->dma_len = p->l2cap + 6;

		if (ble_master_add_tx_packet ((u32)p))
		{
            if(0 == ota_adr){
                sleep_us(300*1000);     // wait for slave to unprotect flash.
            }
		
			ota_adr += 16;
			if (nlen == 0)
			{
				host_ota_start = 0;
			}
		}
	}
}

extern u8 pair_login_ok;

int	rf_link_slave_data_ota(void *ph);

typedef void (*fp_rf_led_ota_ok)(void);
typedef void (*fp_rf_led_ota_error)(void);

u16 		                ota_pkt_cnt = 0;
fp_rf_led_ota_ok 			p_vendor_rf_led_ota_ok 			= 0;
fp_rf_led_ota_error			p_vendor_rf_led_ota_error 		= 0;

enum{
    CMD_UART_OTA_FW_VERSION = 0xff00,
	CMD_UART_OTA_START      = 0xff01,
	CMD_UART_OTA_END        = 0xff02,
	CMD_UART_OTA_CODE       = 0xff03,
};

typedef struct{
    u8  len;
    u16 sno;
    u8	code[16];    //size must 16 
    u16 crc;
}uart_pkt_ota_t;

uart_pkt_ota_t uart_pkt_ota_pc;
u8  uart_ota_pc_tx_busy;

#define     UART_OTA_RX_EN              0   // only 8267 can enable now

#define     UART_OTA_PC_TEST_EN         0
#if UART_OTA_PC_TEST_EN   // for PC demo
u8  uart_ota_pc_start = 0;

#define CMD_UART_OTA_START_LEN  (sizeof(uart_pkt_ota_pc.sno))
#define CMD_UART_OTA_END_LEN    (sizeof(uart_pkt_ota_pc.sno)+sizeof(uart_pkt_ota_pc.crc))
#define CMD_UART_OTA_CODE_LEN   (sizeof(uart_pkt_ota_t)- sizeof(uart_pkt_ota_pc.len))
#define UART_CODE_LEN           (sizeof(uart_pkt_ota_pc.code))

STATIC_ASSERT(UART_CODE_LEN == 16);

void proc_uart_ota_pc()  // demo for PC
{
    static u32 n_firmware = 0;
    static u32 ota_adr = 0;
    if(!uart_ota_pc_start || uart_ota_pc_tx_busy){
        return ;
    }
    
    uart_ota_pc_tx_busy = 1;
    uart_pkt_ota_t *p = &uart_pkt_ota_pc;
    if(1 == uart_ota_pc_start){
        n_firmware = *(u32 *)(flash_adr_ota_master+0x18);
        ota_adr = 0;
        uart_ota_pc_start = 2;
        p->len = CMD_UART_OTA_START_LEN;
        p->sno = CMD_UART_OTA_START;
    }else if(2 == uart_ota_pc_start){
        int nlen = ota_adr < n_firmware ? UART_CODE_LEN : 0;

    	if(nlen == UART_CODE_LEN){
    		memcpy(p->code, p_firmware + ota_adr, UART_CODE_LEN);
            p->len = CMD_UART_OTA_CODE_LEN;
    	}else{  // nlen == 0
    		memset(p->code, 0, UART_CODE_LEN);
            p->len = CMD_UART_OTA_END_LEN;
    	}
        p->sno = ota_adr / UART_CODE_LEN;
		u16 crc = crc16((u8 *)&p->sno, sizeof(uart_pkt_ota_pc.sno)+nlen);
	    p->code[0+nlen] = (u8)crc;
	    p->code[1+nlen] = crc >> 8;
	    
        if(0 == ota_adr){
            sleep_us(300*1000);     // wait for slave to unprotect flash.
        }
	
		ota_adr += UART_CODE_LEN;
		if (nlen == 0){
			uart_ota_pc_start = 0;
		}    	
    }        
}
#endif

#if UART_OTA_RX_EN
u8  uart_ota_rx_start = 0;
u32 uart_ota_rx_start_tick = 0;
uart_pkt_ota_t uart_pkt_ota_rx;
rf_packet_att_data_t    uart2pkt_att;

void proc_uart_ota_rx ()
{
    if(uart_ota_pc_tx_busy){
	    rf_packet_att_data_t *p_pkt_att = &uart2pkt_att;
        memcpy(&uart_pkt_ota_rx, &uart_pkt_ota_pc, sizeof(uart_pkt_ota_rx));
        memcpy(p_pkt_att->dat, &uart_pkt_ota_rx.sno, uart_pkt_ota_rx.len);
        uart_ota_pc_tx_busy = 0;
        
	    u16 sno2cmd =  p_pkt_att->dat[0] | (p_pkt_att->dat[1]<<8);
    	if(sno2cmd == CMD_UART_OTA_FW_VERSION){
    	    // reserve
    	}else if(sno2cmd == CMD_UART_OTA_START){
            pair_login_ok = 1;
            uart_ota_rx_start = 1;
            uart_ota_rx_start_tick = clock_time();
    	    // flash erase
    	}else if(sno2cmd == CMD_UART_OTA_END){
    	    // reserve
    	}else{              // CODE
        	if (uart_ota_rx_start){
            	p_pkt_att->type = 2;
            	p_pkt_att->l2cap = 3 + uart_pkt_ota_rx.len;
            	p_pkt_att->chanid = 0x04;
            	p_pkt_att->att = ATT_OP_WRITE_CMD;
            	p_pkt_att->hl = 0x18;
            	p_pkt_att->hh = 0x0;
            	p_pkt_att->rf_len = p_pkt_att->l2cap + 4;
            	p_pkt_att->dma_len = p_pkt_att->l2cap + 6;
                
                rf_link_slave_data_ota(p_pkt_att);
            }
    	}
	}
}
#endif

int dbg_am, dbg_kb;
void main_loop(void)
{
	static u32 dbg_st, dbg_m_loop;
	dbg_m_loop ++;

	//proc_debug ();

#if(STACK_CHECK_ENABLE)
    stack_check();
#endif

#if DEBUG_USB_MODE

	proc_host ();

	if (! ble_master_status () && conn_start)		// APP(master) emulation
	{
		dbg_st++;

		ble_master_start (
				conn_interval,					//6 * 1.25 = 7.5 ms interval
				conn_timeout,					// 1 second time out
				&rf_custom_dat,
				CLOCK_SYS_CLOCK_HZ/1000000		// 32M: 32 tick per us
				);
	}

    proc_ota ();                // update firmware  to other slave lights.
	if(!host_ota_start){
	    #if UART_OTA_PC_TEST_EN
        proc_uart_ota_pc();     // update firmware to itself (local) from local for test.
        #endif
        #if UART_OTA_RX_EN
        proc_uart_ota_rx();     // update firmware to itself (local) from UART.
        #endif
    }
#else

	int det_kb = proc_keyboard (1);
	if (det_kb)
	{
		mouse_button = kb_event.cnt == 1 && kb_event.keycode[0] == VK_LEFTB;
		if (kb_event.cnt || kb_event.ctrl_key)		//non empty or release key
		{
			tick_kb = clock_time ();
		}
	}

	if (sys_am || mouse_button)
	{
		proc_mouse (mouse_button);
	}

	if ((sys_kb || sys_am) && det_kb)
	{
		dbg_kb++;
		rf_custom_dat.ctype = CUSTOM_DATA_KEYBOARD;
		memcpy (rf_custom_dat.cmd, (void *)&kb_event, sizeof(kb_data_t));
		ble_master_write (&rf_custom_dat.ctype);
	}
	else if (!ble_master_command_busy () && sys_am && km_data_get(rf_custom_dat.cmd))
	{
		dbg_am++;
		rf_custom_dat.ctype = CUSTOM_DATA_MOUSE;
		ble_master_write (&rf_custom_dat.ctype);
	}

	if (!sys_am && !sys_kb)
	{
		ble_master_stop ();
	}
	else if (! ble_master_status ())		// APP(master) emulation
	{
		dbg_st++;

		ble_master_pairing_enable (1);

		ble_master_start (
				conn_interval,					//6 * 1.25 = 7.5 ms interval
				conn_timeout,					//1 second time out
				&rf_custom_dat,
				CLOCK_SYS_CLOCK_HZ/1000000		// 32M: 32 tick per us
				);
	}

	proc_suspend ();
#endif
}

////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////
const	u8	mac_default[] = {0, 0, 0, 0, 0, 0};

void  user_init(void)
{
	// for app ota   
    flash_get_id();

    erase_ota_data_handle();

	usb_log_init ();
	/////////// ID initialization for host control software //////////
	REG_ADDR8(0x74) = 0x53;
	REG_ADDR16(0x7e) = 0x82bd;
	REG_ADDR8(0x74) = 0x00;

	//enable I2C function, enable internal 10K pullup
	reg_gpio_pe_gpio &= ~ BIT(7);
	reg_gpio_pf_gpio &= ~ BIT(1);
	analog_write(20, analog_read(20) | (GPIO_PULL_UP_10K<<2) | (GPIO_PULL_UP_10K<<6));	//  CK, DI, pullup 10K
	// enable gyro power
	gpio_write(GPIO_PD7, 0);
	gpio_set_output_en(GPIO_PD7, 1);
	//airmouse_enable (1);

	/////////// rc initialization /////////////////////////////////////
	ble_master_init ();
	//master_smp_func_init ();

	/////////// enable USB device /////////////////////////////////////
	#if (MCU_CORE_TYPE == MCU_CORE_8267)
	usb_dp_pullup_en (0);    // fix 8267 A0 error:1.5K pull up
	#else
	usb_dp_pullup_en (1);
	#endif
	reg_usb_ep_ctrl(EP_BO) = BIT(0);

	/////////////// setup LED /////////////////////////////////////////
	gpio_set_func (GPIO_LED, AS_GPIO);
	#if (MCU_CORE_TYPE == MCU_CORE_8267)
	REG_ADDR8(0x5b1) = 0x0;     // set default  function Mux to PWM
	REG_ADDR8(0x5b4) |= 0x3;    // set default  function Mux to PWM for PE0/PE1
	#endif
	//reg_pwm_pol =  BIT(PWMID_LED);
	reg_pwm_clk = 255;			//clock by 256
	pwm_set (PWMID_LED, 0x2000, 0x1000);
	pwm_start (PWMID_LED);

	////////// set up wakeup source: driver pin of keyboard  //////////
	u8 pin[] = KB_DRIVE_PINS;
	for (int i=0; i<sizeof (pin); i++)
	{
		cpu_set_gpio_wakeup (pin[i], 1, 1);
	}
	////////// set up wakeup source: driver pin of keyboard  //////////
	rf_set_power_level_index (RF_POWER_8dBm);

	ble_set_debug_adv_channel (38);
	p_firmware = (u8 *)flash_adr_ota_master;

#if	DEBUG_USB_MODE
	ble_master_set_slave_mac (mac_default);
	ble_master_set_golden ();
#endif

#if(STACK_CHECK_ENABLE)
    stack_check_init();
#endif

}
