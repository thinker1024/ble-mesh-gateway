#include "../../proj/tl_common.h"
#include "../../proj_lib/light_ll/light_frame.h"
#include "../../proj_lib/ble_ll/blueLight.h"
#include "../../proj_lib/light_ll/light_ll.h"
#include "../../proj/mcu/watchdog_i.h"
#include "../../proj_lib/pm.h"
#include "common.h"
#include "../../proj/drivers/uart.h"
#include "../../proj_lib/ble_ll/att.h"

FLASH_ADDRESS_EXTERN;
/////////////// password encode sk initial  ///////////////////////////////////////////////////
u8	pair_config_pwd_encode_sk[17] = {MESH_PWD_ENCODE_SK};
u8  pair_config_pwd_encode_enable = 1;

/////////////// adv par define ///////////////////////////////////////////////////
u16 adv_interval2listen_interval = 4;           // unit: default is 40ms, setting by 40000 from rf_link_slave_init (40000);
u16 online_status_interval2listen_interval = 8; // unit: default is 40ms, setting by 40000 from rf_link_slave_init (40000);

#if PASSIVE_EN
/////////////// for passive switch ///////////////////////////////////////////////
u8  separate_ADVpkt = 1;					//if 1 send one adv packet in each interrupt
u8  mesh_chn_amount = 2;				//amount of sys_chn_listen
/////////////// listen chanel define ///////////////////////////////////////////////////

u8 sys_chn_listen[4] = {2, 12, 2, 12};	//8, 30, 52, 74
#else
/////////////// for passive switch ///////////////////////////////////////////////
u8  separate_ADVpkt = 0;					//if 1 send one adv packet in each interrupt
u8  mesh_chn_amount = 4;				//amount of sys_chn_listen
/////////////// listen chanel define ///////////////////////////////////////////////////

u8 sys_chn_listen[4] = {2, 12, 23, 34};	//8, 30, 52, 74
#endif
/////////////// mesh node define ////////////////////////////////////////////////////
mesh_node_st_t mesh_node_st[MESH_NODE_MAX_NUM];
status_record_t slave_status_record[MESH_NODE_MAX_NUM];
u16 slave_status_record_size = sizeof(slave_status_record);

u32	mesh_node_mask[MESH_NODE_MAX_NUM>>5];
u16 mesh_node_max_num = MESH_NODE_MAX_NUM;
u8 mesh_node_st_val_len = MESH_NODE_ST_VAL_LEN;
u8 mesh_node_st_par_len = MESH_NODE_ST_PAR_LEN;
u8 mesh_node_st_len = sizeof(mesh_node_st_t);

void mesh_node_buf_init ()
{
	for (int i=0; i<mesh_node_max_num; i++)
	{
	    memset(&mesh_node_st[i], 0, sizeof(mesh_node_st_t));
	}
	device_status_update();
}

u8	SW_Low_Power = 0;

STATIC_ASSERT((MESH_NODE_MAX_NUM <= 256) && ((MESH_NODE_MAX_NUM % 32) == 0));
STATIC_ASSERT((MESH_NODE_ST_VAL_LEN >= 4) && ((MESH_NODE_ST_VAL_LEN <= 10)));

///////////////////////////////////////////////////////////////////////////////////
#define TYPE_FIX_STEP               1
#define TYPE_FIX_TIME               2

#define STEP_TYPE                   TYPE_FIX_TIME // TYPE_FIX_STEP // 

#if(STEP_TYPE == TYPE_FIX_STEP)
#define LIGHT_ADJUST_STEP           (2)   //unit: lum step 1--100
#define LIGHT_ADJUST_INTERVAL       (2)   // unit :10ms;     min:20ms
#else
#define LIGHT_ADJUST_TIME           (100)   //unit: 10ms
#define LIGHT_ADJUST_INTERVAL       (2)   // unit :10ms;     min:20ms
#endif

extern void light_adjust_RGB_hw(u8 val_R, u8 val_G, u8 val_B, u8 lum);
void light_onoff_normal(u8 on);

extern u16 rgb_lumen_map[101];
extern u8 light_off;
extern u8 led_lum;
extern u8 led_val[6];

typedef struct{
    u32 time;
    s16 lum_temp;
    s16 lum_dst;
    u8 step;
    u8 step_mod;
    u8 remainder;
    u8 adjusting_flag;
}light_step_t;

static light_step_t light_step = {};

enum{
    LUM_UP = 0,
    LUM_DOWN,
};

void get_next_lum(u8 direction){    
    u32 temp = light_step.remainder + light_step.step_mod;
    light_step.remainder = (u8)temp;
    
    if(LUM_UP == direction){
        light_step.lum_temp += light_step.step;
        if(temp >= 0x100){
            light_step.lum_temp += 1;
        }
        if(light_step.lum_temp >= light_step.lum_dst){
            light_step.lum_temp = light_step.lum_dst;
            light_step.remainder = 0;
        }
    }else{
        light_step.lum_temp -= light_step.step;
        if(temp >= 0x100){
            light_step.lum_temp -= 1;
        }
        if(light_step.lum_temp <= light_step.lum_dst){
            light_step.lum_temp = light_step.lum_dst;
            light_step.remainder = 0;
        }
    }
}

void get_step(u8 direction){
    light_step.remainder = 0;       // reset
    #if(STEP_TYPE == TYPE_FIX_STEP)
    light_step.step = LIGHT_ADJUST_STEP;
    light_step.step_mod = 0;
    #else   // fix time
    if(LUM_UP == direction){
        light_step.step = (light_step.lum_dst - light_step.lum_temp)/(LIGHT_ADJUST_TIME / LIGHT_ADJUST_INTERVAL);
        light_step.step_mod = (((light_step.lum_dst - light_step.lum_temp)%(LIGHT_ADJUST_TIME / LIGHT_ADJUST_INTERVAL))*256)/(LIGHT_ADJUST_TIME / LIGHT_ADJUST_INTERVAL);
    }else{
        light_step.step = (light_step.lum_temp - light_step.lum_dst)/(LIGHT_ADJUST_TIME / LIGHT_ADJUST_INTERVAL);
        light_step.step_mod = (((light_step.lum_temp - light_step.lum_dst)%(LIGHT_ADJUST_TIME / LIGHT_ADJUST_INTERVAL))*256)/(LIGHT_ADJUST_TIME / LIGHT_ADJUST_INTERVAL);
    }
    #endif
}

void light_step_correct_mod(u16 *pwm_val, u8 lum){
    #if(STEP_TYPE == TYPE_FIX_TIME)
    int temp_pwm = light_step.remainder;
    
    if(light_step.adjusting_flag && (light_step.lum_dst != light_step.lum_temp)
   && ((lum > 0) && (lum < ARRAY_SIZE(rgb_lumen_map) -1))
   && (light_step.remainder)){
        if(light_step.lum_dst > light_step.lum_temp){
            temp_pwm = *pwm_val + (temp_pwm * (rgb_lumen_map[lum+1] - rgb_lumen_map[lum])) / 256;

            if(temp_pwm > U16_MAX){
                temp_pwm = U16_MAX;
            }
        }else{
            temp_pwm = *pwm_val - (temp_pwm * (rgb_lumen_map[lum] - rgb_lumen_map[lum-1])) / 256;
            if(temp_pwm < 0){
                temp_pwm = 0;
            }
        }

        *pwm_val = temp_pwm;
    }
    #endif
}

void light_onoff_step_init()
{
    //light_step.adjusting_flag = 0;
    memset((u8 *)(&light_step), 0, sizeof(light_step));
}

void light_onoff_step(u8 on){
    if(light_step.adjusting_flag){
        //return ;
    }

    u8 set_flag= 1;
    
    if(on){
        if(light_off){
            if(0 == light_step.adjusting_flag){
                light_step.lum_temp = 0;
            }
            light_step.lum_dst = led_lum;
            get_step(LUM_UP);
    	}else{
    	    set_flag = 0;
    	    light_onoff_normal(1); // make sure on. unnecessary.
    	}
        light_off = 0;
	}else{
        if(light_off){
    	    set_flag = 0;
    	    light_onoff_normal(0); // make sure off. unnecessary.
    	}else{
            if(0 == light_step.adjusting_flag){
                light_step.lum_temp = led_lum;
            }
            light_step.lum_dst = 0;
            get_step(LUM_DOWN);
    	}
        light_off = 1;    
	}
	
    light_step.adjusting_flag = set_flag;
    light_step.time = 0;
}

void light_onoff_step_timer(){
    if(light_step.adjusting_flag){
        if(0 == light_step.time){
            if(light_step.lum_dst != light_step.lum_temp){
                if(light_step.lum_temp < light_step.lum_dst){
                    get_next_lum(LUM_UP);
                }else{
                    get_next_lum(LUM_DOWN);
                }
                light_adjust_RGB_hw(led_val[0], led_val[1], led_val[2], light_step.lum_temp);                
            }else{
                light_step.adjusting_flag = 0;
                memset((u8 *)(&light_step), 0, sizeof(light_step));
            }
        }
        
        light_step.time++;
        if(light_step.time >= LIGHT_ADJUST_INTERVAL){
            light_step.time = 0;
        }
    }
}

u8 is_lum_invalid(u8 lum){
    #define LED_LUM_MIN         5
    if(lum < LED_LUM_MIN){
        return LED_LUM_MIN;
    }else{
        return 0;
    }
}

extern u8 rf_slave_ota_busy;
void pa_init(u8 tx_pin_level, u8 rx_pin_level)
{
#if(PA_ENABLE)
    gpio_set_func(PA_TXEN_PIN, AS_GPIO);
    gpio_set_input_en(PA_TXEN_PIN, 0);
    gpio_set_output_en(PA_TXEN_PIN, 1);
    gpio_write(PA_TXEN_PIN, tx_pin_level);
    
    gpio_set_func(PA_RXEN_PIN, AS_GPIO);
    gpio_set_input_en(PA_RXEN_PIN, 0);
    gpio_set_output_en(PA_RXEN_PIN, 1);
    gpio_write(PA_RXEN_PIN, tx_pin_level);
#endif    
}
void pa_txrx(u8 val)
{
#if(PA_ENABLE)
    if(val == PA_OFF/* || rf_slave_ota_busy*/){
        gpio_write(PA_TXEN_PIN, 0);
        gpio_write(PA_RXEN_PIN, 0);
    }else if(val == PA_TX){
        gpio_write(PA_RXEN_PIN, 0);
        gpio_write(PA_TXEN_PIN, 1);
    }else if(val == PA_RX){
        gpio_write(PA_TXEN_PIN, 0);
        gpio_write(PA_RXEN_PIN, 1);
    }
#endif
}

u8 iBeaconInterval = 0;
u8 beacon_with_mesh_adv = 0;// 0 means only send beacon adv pkt;  1 means send both of beacon pkt and mesh adv pkt
#if(IBEACON_ENABLE)
u8 eddystone_uid[31] = {
                    0x02, 0x01, 0x06,       // not connect
                    0x03, 0x03, 0xAA, 0xFE, // uuid
                    0x17, 0x16, 0xAA, 0xFE, // UID type's len is 0x17
                        0x00,               // UID type
                        0x08,               // tx power
                        0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, // NID
                        0x01, 0x02, 0x03, 0x04, 0x05, 0x06,                         // BID
                        0x00, 0x00          // RFU
                    };
u8 eddystone_url[31] = {
                    0x02, 0x01, 0x06,       // not connect
                    0x03, 0x03, 0xAA, 0xFE, // uuid
                    0x12, 0x16, 0xAA, 0xFE, // URL type's len is variable
                        0x10,               // URL type
                        0x08,               // tx power
                        0x00,               // URL Scheme 0x00-http://www.  0x01-https://www.  0x02-http://  0x03-https://
                        0x74, 0x65, 0x6c, 0x69, 0x6e, 0x6b, 0x2d, 0x73, 0x65, 0x6d, 0x69,// telink-semi
                        0x07,               // 0x07-.com  0x08-.org 0x09-.edu  0x0a-.net............
                    };

u8 eddystone_tlm[31] = {
                    0x02, 0x01, 0x06,       // not connect
                    0x03, 0x03, 0xAA, 0xFE, // uuid
                    0x11, 0x16, 0xAA, 0xFE, // TLM type's len is 0x11
                        0x20,               // TLM type
                        0x00,               // TLM version
                        0x00, 0x00,         // Battery voltage 1mV/bit
                        0x00, 0x80,         // Temperature
                        0x00, 0x00, 0x00, 0x00, // ADV_CNT
                        0x00, 0x00, 0x00, 0x00, // SEC_CNT unit:0.1s
                    };

u8 iBeaconData[31] = {0};
u8 beacon_len = 0;
extern rf_packet_adv_ind_module_t pkt_ibeacon;
extern u8* slave_p_mac;
void set_ibeacon_data(u8 *val, int n){
    memcpy (pkt_ibeacon.advA, slave_p_mac, 6);
    memcpy (pkt_ibeacon.data, val, n);
    pkt_ibeacon.dma_len = n + 8;
    pkt_ibeacon.rf_len = n + 6;
}
#endif

u8 flash_user_data[16] = {0};
u8 user_data_idx;					//always pointing to the next block address to be written
#define max_data_block 256			//256*16= 4096
u8 get_user_data_idx(void)
{
	int i;
	u8 data[16];
	u8 data1[16] = {0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
					0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff};
	user_data_idx = 0;
	for (i=0; i<max_data_block; i++)
	{
		flash_read_page((flash_adr_user_data + user_data_idx*16),16,(u8 *)(&data));
		if(!memcmp(data,data1,16))
			break;
		user_data_idx++;
	}
	if(max_data_block == i)
	{
		memcpy(flash_user_data,data,16);
		flash_erase_sector(flash_adr_user_data);
		user_data_idx = 0;
	}
	if(0 == user_data_idx)
	{
		flash_write_page(flash_adr_user_data,16,(u8 *)(&flash_user_data));	//reset
		user_data_idx ++;
	}
	return user_data_idx;
}

void get_flash_user_data(void)		//get data from flash
{
	get_user_data_idx();
	flash_read_page((flash_adr_user_data + (user_data_idx-1)*16),16,(u8 *)(&flash_user_data));
}



void store_user_data_2flash(void)
{
	get_user_data_idx();
	flash_write_page((flash_adr_user_data + user_data_idx*16),16,(u8 *)(&flash_user_data));
	user_data_idx ++;
}

int compare_user_data(void)
{
	u8 temp[16];
	flash_read_page((flash_adr_user_data + (user_data_idx-1)*16),16,(u8 *)(&temp));
	return memcmp(temp,flash_user_data,16);
}


void check_store_user_data(void)	//if flash_user_data changed store it
{
	get_user_data_idx();
	if(!compare_user_data())
		return;							//data not changed
	store_user_data_2flash();
}

#if ((__PROJECT_LIGHT_SWITCH__)         \
    ||(__PROJECT_LIGHT_8266__)          \
    ||(__PROJECT_8266_MESH_CONFIG__)    \
    ||(__PROJECT_LIGHT_8267__)          \
    || (__PROJECT_LIGHT_8267_UART__)	\
    ||(__PROJECT_LIGHT_NO_MESH__)          \
    ||(__PROJECT_MASTER_LIGHT_8267__)          \
    ||(__PROJECT_MOTIONSENSOR_8267__)   \
    ||(__PROJECT_LIGHT_SWITCH__)        \
    ||(__PROJECT_BLE_MASTER__)          \
    ||(__PROJECT_MONITOR_8266__))

// for ota
extern fp_rf_led_ota_ok 			p_vendor_rf_led_ota_ok;
extern fp_rf_led_ota_error			p_vendor_rf_led_ota_error;
#if (__PROJECT_LIGHT_SWITCH__ || __PROJECT_MOTIONSENSOR_8267__ || __PROJECT_LIGHT_NO_MESH__)
#define 	OTA_LED				GPIO_LED
#else
#define 	OTA_LED				PWM_R
#endif

void rf_led_ota_ok(void){
	if(p_vendor_rf_led_ota_ok){
		p_vendor_rf_led_ota_ok();
		return;
	}
	gpio_set_func(OTA_LED, AS_GPIO);
	gpio_set_output_en(OTA_LED, 1);
	static u8 led_onoff = 1;
	foreach(i, 6){
		gpio_write(OTA_LED, led_onoff);
		led_onoff = !led_onoff;
#if(MODULE_WATCHDOG_ENABLE)
        wd_clear();
#endif
		sleep_us(1000*1000);
	}
}
void rf_led_ota_error(void){
	if(p_vendor_rf_led_ota_error){
		p_vendor_rf_led_ota_error();
		return;
	}
	gpio_set_func(OTA_LED, AS_GPIO);
	gpio_set_output_en(OTA_LED, 1);
	static u8 led_onoff = 1;
	foreach(i, 60){
		gpio_write(OTA_LED, led_onoff);
		led_onoff = !led_onoff;
#if(MODULE_WATCHDOG_ENABLE)
        wd_clear();
#endif
		sleep_us(100*1000);
	}
}
#endif

///////////////  light_slave_tx_command call back /////////////////////////////////
void rf_link_data_callback (u8 *);
void light_slave_tx_command_callback (u8 *p){
    rf_link_data_callback(p);
}

///////////////  set BLE interval and timeout parameter /////////////////////////////////
/************
 *
 * int setup_ble_parameter_start(u16 delay, u16 interval_min, u16 interval_max, u16 timeout);
 *
 * delay   :  unit: one ble interval 
 * interval_min,interval_max:  if all 0,will keep the system parameter for android but not ios.   unit: 1.25ms; must longer than 20ms.
 * timeout:  if 0,will keep the system parameter.   unit: 10ms; must longer than 3second for steady connect.
 *
 * return 0 means setup parameters is valid.
 * return -1 means parameter of interval is invalid.
 * return -2 means parameter of timeout is invalid.
 *
 *
 * void rf_link_slave_connect_callback()
 * system will call this function when receive command of BLE connect request.
 */
void rf_link_slave_connect_callback(){
    #if 1
    #if 0 // for android to set a fixed interval
    setup_ble_parameter_start(1, 32, 32, 200);  // interval 32: means 40ms;   timeout 200: means 2000ms
    #else   // for ios
    /*IOS Note: 
          20 ms <= interval_min
          interval_min + 20 ms <= interval_max <= 2second
          timeout <= 6second
       */
    setup_ble_parameter_start(1, 16, 16+16, 200);  // interval 32: means 40ms;   timeout 200: means 2000ms
    #endif
    #endif
}

void cb_ble_slave_disconnect(){
}

void cb_pair_failed(){
}

// system will call p_cb_ble_slave_disconnect() when BLE disconnect.
cb_func_void_t	p_cb_ble_slave_disconnect = 0;  // cb_ble_slave_disconnect  //

// system will call p_cb_pair_failed() when login failed or set mesh name/password/ltk failed.
cb_func_void_t	p_cb_pair_failed = 0;   // cb_pair_failed   //

/************
 * u8 get_setup_ble_parameter_result()
 *
 * return 0 means setup parameters fail.
 * return 1 means setup parameters success.
 */
 
u8 get_setup_ble_parameter_result(){
    extern u8 update_ble_par_success_flag;
    return update_ble_par_success_flag;
}

////////////////// gate way /////////////////////////////////////
#if GATEWAY_EN
int rf_link_slave_data (rf_packet_ll_data_t *p, u32 t);

extern u8 	slave_link_connected;
extern u8   pair_login_ok;
extern u8   not_need_login;

typedef struct{
    u8 sno[3];
    u8 src[2];
    u8 dst[2];
    u8 op;
    u16 vendor_id;
    u8 par[10];
}app_cmd_value_t;

typedef struct{
	u32 dma_len;
	u8	type;
	u8  rf_len;
	u16	l2capLen;
	u16	chanId;
	u8  opcode;
	u8 handle;
	u8 handle1;
	app_cmd_value_t app_cmd_v;
	u8 rsv[10];
}rf_packet_ll_app_t;

u32 gateway_cmd_sno;

u8          mode_master = 0;

void rf_link_slave_data_app(rf_packet_ll_app_t *pkt_app)
{
    u8 r = irq_disable();
    if(!gateway_security){
        not_need_login = pair_login_ok = 1;
    }
    rf_link_slave_data((rf_packet_ll_data_t *)(pkt_app), 0);
    irq_restore(r);
}
/*///////////////////////////////////////////////////
int	ble_gateway_ll_data (u8 *p, int n);

p[0] : rf_packet_att_cmd_t.type
n : length of input buffer, max is 29
///////////////////////////////////////////////////*/
int	ble_gateway_ll_data (u8 *p, int n)
{
	if (1) {
	    rf_packet_ll_app_t  pkt_app_data = {};
	    memset (&pkt_app_data, 0, sizeof(pkt_app_data));
		memcpy (&pkt_app_data.type, p, n);
		pkt_app_data.dma_len = n;			// recalculate to make sure length is ok
		pkt_app_data.rf_len = n - 2;        // recalculate to make sure length is ok
		if (pkt_app_data.type < 3)
		{
			pkt_app_data.l2capLen = n - 6;  // recalculate to make sure length is ok
		}

        rf_link_slave_data_app(&pkt_app_data);
		return 1;
	}
	return 0;
}

#if 1    // for test
int gatway_tx_command(u8 cmd, u16 dst_adr, u8 *par, u8 par_len, u8 op){ // should call in fp_gateway_rx_proc();
    if(par_len > (is_cmd_long_par(cmd) ? 15 : 10)){
        return -1;
    }

    // packet 
	rf_packet_ll_app_t  pkt_app_data = {};
    memset(&pkt_app_data, 0, sizeof(pkt_app_data));
    pkt_app_data.type = 0x02;
    pkt_app_data.rf_len = 17 + par_len;
    pkt_app_data.dma_len = pkt_app_data.rf_len + 2;
    pkt_app_data.l2capLen = pkt_app_data.rf_len - 4;
    pkt_app_data.chanId = 0x04;
    pkt_app_data.opcode = op;
    pkt_app_data.handle= 0x15;
    pkt_app_data.handle1 = 0x00;
    
    gateway_cmd_sno++;
    memcpy(pkt_app_data.app_cmd_v.sno, &gateway_cmd_sno, 3);
    //memcpy(pkt_app_data.app_cmd_v.src, &device_address, 2);
    memcpy(pkt_app_data.app_cmd_v.dst, &dst_adr, 2);
    pkt_app_data.app_cmd_v.op = (cmd & 0x3F) | 0xC0;
    pkt_app_data.app_cmd_v.vendor_id = VENDOR_ID;
    memcpy(pkt_app_data.app_cmd_v.par, par, par_len);
    
    // send command
    rf_link_slave_data_app(&pkt_app_data);
    
    return 0;
}

// update firmware for itself
int gatway_local_OTA(u8 *par, u8 par_len){ // should call in fp_gateway_rx_proc();
    if(par_len > 20){
        return -1;
    }

    // packet 
	rf_packet_ll_app_t  pkt_app_data = {};
    memset(&pkt_app_data, 0, sizeof(pkt_app_data));
    pkt_app_data.type = 0x02;
    pkt_app_data.rf_len = 7 + par_len;
    pkt_app_data.dma_len = pkt_app_data.rf_len + 2;
    pkt_app_data.l2capLen = pkt_app_data.rf_len - 4;
    pkt_app_data.chanId = 0x04;
    pkt_app_data.opcode = ATT_OP_WRITE_CMD;
    pkt_app_data.handle= 0x18;
    pkt_app_data.handle1 = 0x00;
    
    memcpy(&pkt_app_data.app_cmd_v, par, par_len);
    
    // send command
    rf_link_slave_data_app(&pkt_app_data);
    
    return 0;
}

void gatway_tx_command_test(){
    u8 par[10] = {0};   // must init to zero;  max size of par is 10
    u16 dst_adr = 0xFFFF;
    u8 cmd = LGT_CMD_LIGHT_ONOFF;
    
    static u32 tx_cmd_cnt;
    tx_cmd_cnt++;
    if(tx_cmd_cnt & 1){
        par[0] = LIGHT_ON_PARAM;
    }else{
        par[0] = LIGHT_OFF_PARAM;
    }
    
    gatway_tx_command(cmd, dst_adr, par, sizeof(par), ATT_OP_WRITE_REQ);
}

///////////////  sample function of MASTER UART handle received data  /////////////////////////////////
/************
 * void    gateway_uart_host_handle(void *pkt_uart);
 *
 * pkt_uart: pointer to UART data
 *
 * MASTER UART will call this function when receive data from gateway.
 */
void    gateway_uart_host_handle(void *pkt_uart)
{
    rf_packet_ll_app_t *pkt_rsp = CONTAINER_OF(pkt_uart, rf_packet_ll_app_t, l2capLen);
    if(0x04 == pkt_rsp->chanId){   // LLID : 2
        if(ATT_OP_WRITE_RSP == pkt_rsp->opcode){
            //
        }else if(ATT_OP_READ_RSP == pkt_rsp->opcode){
            //
        }else if(ATT_OP_HANDLE_VALUE_NOTI == pkt_rsp->opcode){  // notify pkt
            //
        }
    }
}
#endif

// Note: check differrence IO of button for differrence PCB
void	proc_ui_gateway ()
{
    if(gateway_en && rf_slave_ota_busy){
        u8 r = irq_disable();   // must
        fp_gateway_rx_proc();   // handle ota data as soon as possible
        irq_restore(r);
    }

	static u32 tick;
	if (!clock_time_exceed (tick, 40000))
	{
		return;
	}
	tick = clock_time();
	
    if(!gateway_en){
        // when in gateway mode, proccess in irq_st_ble_rx(), so it just receive BLE command in irq_st_ble_rx().
        fp_gateway_rx_proc();
    }
    
	static u8 st = 0;
	u8 s = !gpio_read (SWITCH_MODE_BUTTON1) || !gpio_read(SWITCH_MODE_BUTTON2);
	if ((!st) & s)
	{
	    #if 0
	    gatway_tx_command_test();      // for test
	    #else
	    if(0 == gateway_mode_onoff(!gateway_en)){
            rf_link_light_event_callback(LGT_CMD_DEL_PAIR);
	    }
	    #endif
	}
	st = s;
}

const rf_packet_ll_init_t	pkt_gateway_init = {
		sizeof (rf_packet_ll_init_t) - 4,		// dma_len
		FLG_BLE_LIGHT_CONNECT_REQ,				// type
		sizeof (rf_packet_ll_init_t) - 6,		// rf_len
		{0xd0, 0xd1, 0xd2, 0xd3, 0xd4, 0xd5},	// scanA
		{0xe0, 0xe1, 0xe2, 0xe3, 0xe4, 0xe5},	// advA
		{0xd6, 0xbe, 0x89, 0x8e},				// access code
		{0x8d, 0x26, 0xd8},						// crcinit[3]
		0x02,									// wsize
		0x0005,									// woffset
		0x0020,									// interval: 32 * 1.25 ms = 40 ms
		0x0000,									// latency
		0x0032,									// timeout: 50*10 ms = 500ms
		{0xff, 0xff, 0xff, 0xff, 0x1f},			// chm[5]
		0xac,									// hop: initial channel - 12
};

#define     EP_BO           5

void	ble_master_data_callback (u8 *p)
{
	static	u32 bdbg_data;
	bdbg_data++;
	/*static */u8 buf[48];
	memset4(buf, 0, sizeof(buf));

	rf_packet_ll_app_t *pkt_rsp;
	if(mode_master){
	    pkt_rsp = (rf_packet_ll_app_t *)(p+4);
	}else{
	    pkt_rsp = (rf_packet_ll_app_t *)p;
	}

	u8 rf_len = pkt_rsp->rf_len;
	buf[0] = rf_len;                      // buf[0-3]: length of UART
	memcpy(buf+4, &pkt_rsp->l2capLen, sizeof(buf)-4);
	
#if __PROJECT_LIGHT_8267_UART__
	uart_Send((u8*)(buf));          // UART MASTER would call gateway_uart_host_handle() to handle this data
#else
	reg_usb_ep8_dat = rf_len + 8;   // length of USB data
	reg_usb_ep8_dat = 0;
	u8 *pdata = buf+4;              // same witch (&pkt_rsp->l2capLen)
	for (int i=0; i < rf_len + 5; i++) // 5 = DC offset (2 BYTE) + CRC (3BYTE)
	{
		reg_usb_ep8_dat = pdata[i];
	}
	if(mode_master){
	    reg_usb_ep8_dat = p[0];     //rssi
	}else{
	    reg_usb_ep8_dat = 0;        //rssi
	}
	reg_usb_ep8_ctrl = BIT(7);
#endif
}

/*
ble_event_callback() is only needed if it is  security mode between master and gateway.
*/
void ble_event_callback (u8 status, u8 *p, u8 rssi)
{
	static u32 bdbg_event;
	bdbg_event++;
#if __PROJECT_LIGHT_8267_UART__
#else
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
#endif
	///////////////////////////////////////////////
}


//////////////////////////////////////////////////////////
//	USB interfuace BI/BO
//////////////////////////////////////////////////////////
u8	buff_command[64];
int host_write_gateway ()
{
	static u32 no_cmd;
	no_cmd++;
	memset4(buff_command, 0, sizeof(buff_command));
#if __PROJECT_LIGHT_8267_UART__
    u32 gateway_proc(void);
	int n = gateway_proc();
#else
	int n = reg_usb_ep_ptr (EP_BO);
	reg_usb_ep_ptr(EP_BO) = 0;
	for (int i=0; i<n; i++)
	{
		buff_command[i] = reg_usb_ep_dat(EP_BO);
	}
#endif
	u8 cmd = buff_command[0];

	if (cmd == 0 && n > 0)		// data
	{
	    if(gateway_en){
		    ble_gateway_ll_data (buff_command + 1, n - 1);
		}
	}
	else if (cmd == 2)	// start master
	{
	    extern rf_packet_adv_ind_module_t	pkt_adv;
	    // report adv packet to PC, because master need MAC of gate way in security mode
	    ble_event_callback (FLG_SYS_DEVICE_FOUND, (u8 *)(&(pkt_adv.rf_len)), 0);
	}
	else if (cmd == 3)	// stop master
	{
	}
	return 0;
}

void host_init ()
{
	/////////// ID initialization for host control software //////////
	REG_ADDR8(0x74) = 0x53;
	REG_ADDR16(0x7e) = 0x82bd;
	REG_ADDR8(0x74) = 0x00;

	/////////// enable USB device /////////////////////////////////////
	#if (MCU_CORE_TYPE == MCU_CORE_8267)    // 
	gpio_setup_up_down_resistor(GPIO_DP, PM_PIN_PULLUP_10K);
	usb_dp_pullup_en (0);    // fix 8267 A0 error:1.5K pull up
    #else
	usb_dp_pullup_en (1);
	#endif
	reg_usb_ep_ctrl(EP_BO) = BIT(0);
}

void proc_host ()
{
	//////////// host interface change  //////////////////////////////////
#if __PROJECT_LIGHT_8267_UART__
	host_write_gateway();
#else
	static u32	tick_bulk_out;
	if (reg_usb_irq & BIT(EP_BO)) {
	    if(!mode_master){
	        host_write_gateway ();
	    }
	    #if MODE_MASTER_SLAVE
	    else{
		    host_write ();
		}
		#endif
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
#endif
}

void gateway_init(){
    gateway_cmd_sno = ((clock_time() + device_address) & 0xff) << 16;
    fp_gateway_tx_proc = &ble_master_data_callback;
    fp_gateway_rx_proc = &proc_host;
	host_init();
}

void gateway_set_login_flag(){
    if(gateway_en && (!gateway_security)){
        not_need_login = pair_login_ok = 1;
    }else{
        not_need_login = pair_login_ok = 0;
    }
}

int gateway_mode_onoff(u8 on){
    u8 r = irq_disable();
    int ret = 0;
    if(on){
        if(0 == gateway_en){
            if(0 == slave_link_connected){  // if connected by other phone
                gateway_en = 1;
                int rf_link_slave_connect (rf_packet_ll_init_t *p, u32 t);
                rf_link_slave_connect((rf_packet_ll_init_t *)&pkt_gateway_init, clock_time());
                slave_link_connected = 1;
                gateway_set_login_flag();
                ble_event_callback (FLG_SYS_LINK_CONNECTED, 0, 0);
            }else{
                extern u8 ble_conn_terminate;
                ble_conn_terminate = 1;
                ret = -1;  // wait for timeout
            }
        }
    }else{
        if(gateway_en){
            gateway_set_login_flag();
        }
        gateway_en = 0;     // will timeout and into adv mode
        ble_event_callback (FLG_SYS_LINK_LOST, 0, 0);
    }

    irq_restore(r);
    return ret;
}
#endif

int mesh_ota_slave_need_ota(u8 *params)
{
    int ret = 1;
    mesh_ota_pkt_start_command_t *p =  (mesh_ota_pkt_start_command_t *)(params+2);
    if(LIGHT_MODE == p->dev_mode){
        void get_fw_version(u8 *ver);
        u8 ver_myself[4];
        get_fw_version(ver_myself);
        if(p->version[1] < ver_myself[1]){
            ret = 0;
        }else if(p->version[1] == ver_myself[1]){
            if(p->version[3] <= ver_myself[3]){
                ret = 0;
            }
        }
    }else{
        ret = 0;
    }
    
    return ret;
}

/* 
for start mesh ota, user should call 
mesh_ota_master_start_firmware_from_20000() or mesh_ota_master_start_firmware_from_own();

Note: only gate way or some node in BLE connected can start mesh ota now.
*/
void mesh_ota_master_start_firmware_from_20000(u8 dev_mode)
{
    mesh_ota_master_start((u8 *)flash_adr_ota_master, *(u32 *)(flash_adr_ota_master+0x18), dev_mode);
}

void mesh_ota_master_start_firmware_from_own()
{
#if (MCU_CORE_TYPE == MCU_CORE_8267)
    u32 adr_fw = ota_program_offset ? 0 : flash_adr_light_new_fw;  // 8267 should use "ota_program_offset"
#else
    u32 adr_fw = 0;
#endif

    mesh_ota_master_start((u8 *)adr_fw, *(u32 *)(adr_fw+0x18), LIGHT_MODE);
}

////////////////// check stack /////////////////////////////////////
#if(STACK_CHECK_ENABLE)
extern u32 _start_bss_, _end_bss_;
#define STACK_CHECK_FLAG        (0x1324adbc)    // use a random data

void stack_check_init(){
    _start_bss_ = _end_bss_ = STACK_CHECK_FLAG;
}

void stack_check(){
	static u32 stack_check_tick;
	if (!clock_time_exceed (stack_check_tick, 1000*1000))
	{
		return;
	}
	stack_check_tick = clock_time();
	
    if((_start_bss_ != STACK_CHECK_FLAG)    // irq_stack
    || (_end_bss_ != STACK_CHECK_FLAG)){    // system_stack
        irq_disable();
        while(1){
            #if(MODULE_WATCHDOG_ENABLE)
    		wd_clear();
            #endif
            static u8 stack_overflow;
            stack_overflow++;
        }
    }
}
#endif

////////////////// ALARM and SCENE /////////////////////////////////////
#include "rtc.h"
#include "scene.h"

int is_bridge_task_busy(){
    return ((ALARM_EN && (is_alarm_poll_notify_busy()))
          || (SCENE_EN && (is_scene_poll_notify_busy())));
}

int is_tx_cmd_busy(){   // it must be used, if want enough bridges(default is BRIDGE_MAX_CNT 8).
    return (0 != mesh_user_cmd_idx);
}

////////////////// OTA  /////////////////////////////////////
u32 get_ota_erase_sectors()
{
    return ERASE_SECTORS_FOR_OTA;
}

int is_ota_area_valid(u32 adr){
	u8 buf[4] = {0};
	foreach(i, ERASE_SECTORS_FOR_OTA){
    	flash_read_page(adr + i*0x1000, 4, buf);
    	u32 tmp = buf[0] | (buf[1]<<8) |(buf[2]<<16) | (buf[3]<<24);
    	if(tmp != ONES_32){
            return 0;
    	}
    }
	return 1;
}

void erase_ota_data(u32 adr){
    flash_protect_OTA_data_erase();
    #if 1
    foreach(i, ERASE_SECTORS_FOR_OTA){
        flash_erase_sector(adr+(ERASE_SECTORS_FOR_OTA -1 - i)*0x1000);
    }
    #else
    //  Note: differrent size or type may use differrent command of block erase.
    STATIC_ASSERT((ERASE_SECTORS_FOR_OTA % 16) == 0);
    u32 block_cnt = ERASE_SECTORS_FOR_OTA/16;
    foreach(i, block_cnt){
        flash_erase_block(adr+(block_cnt -1 - i)*0x10000);
    }
    #endif
}

void erase_ota_data_handle(){
	// for app ota  
    #if (MCU_CORE_TYPE == MCU_CORE_8267)
    u32 adr_ota_data = ota_program_offset;  // 8267 should use "ota_program_offset"
    #else
    u32 adr_ota_data = flash_adr_light_new_fw;
    #endif
	if(0 == is_ota_area_valid(adr_ota_data)){
		erase_ota_data(adr_ota_data);
	}

    #if (MCU_CORE_TYPE == MCU_CORE_8267)
    flash_protect_8267_normal();    // must after erase_sector for OTA
    #else
    flash_protect_8266_normal();
    #endif
}
void light_node_status_change_cb(u8 *p, u8 new_node){
    mesh_node_st_val_t *p_data = (mesh_node_st_val_t*)p;
    extern u8  sync_time_enable;
    if(sync_time_enable){
        p_data->par[0] &= ~FLD_SYNCED;   //Note: bit7 of par[0] have been use internal
    }
    static u8 dev_addr = 0;
    if(new_node){
        static u8 dev_new_node = 0;
        dev_new_node++;
        dev_addr = p_data->dev_adr;
    }else{
        static u8 dev_old_node = 0;
        dev_old_node++;
        dev_addr = p_data->dev_adr;
    }
}

#if ADV_UUID
u8 adv_uuid_flag = 1;
#else
u8 adv_uuid_flag = 0;
#endif
u8 adv_uuid[4] = {0x03, 0x02, 0xAB, 0xCD};

// recover status before software reboot
void light_sw_reboot_callback(void){
#if ((__PROJECT_LIGHT_8266__)           \
    ||(__PROJECT_LIGHT_8267__)          \
    ||(__PROJECT_LIGHT_NO_MESH__))
    if(rf_slave_ota_busy || is_mesh_ota_slave_running()){
        analog_write (rega_light_off, light_off);
    }
#endif
}

/////////////////// mesh_node_filter  //////////////////////
#if 1
cb_mesh_node_filter_t	cb_mesh_node_filter = 0;
#else   // for test relay function  
int is_neighbor_light(u8 device_adr, u8 *p)
{
    rf_packet_att_cmd_t *pp = (rf_packet_att_cmd_t *)p;
    pp = pp;
    if((*(u8 *)0x60000 == device_adr) || (*(u8 *)0x60001 == device_adr)
     ||(*(u8 *)0x60002 == device_adr) || (*(u8 *)0x60003 == device_adr)
     ||(*(u8 *)0x60004 == device_adr) || (*(u8 *)0x60005 == device_adr)){
        // only receive some neighbor devices
        return 1;
    }
    return 0;
}

/*////////////////// mesh_node_filter  //////////////////////
int (*cb_mesh_node_filter_t) (u8 device_adr);
fuction      : use to test relay function  

device_adr : means this packet is sent from this device_adr
p              : point to this packet

return 0 : means skip this packet
return 1 : means accept this packet
*/

cb_mesh_node_filter_t	cb_mesh_node_filter = is_neighbor_light;
#endif

/////////////////// passive device  //////////////////////
u8 passive_en = PASSIVE_EN;
int get_command_type(u8 *p_att_value)
{
#if PASSIVE_EN
    rf_packet_att_value_t *p = (rf_packet_att_value_t*)(p_att_value);
    u8 op = p->val[0] & 0x3F;
    u8 *par = p->val+3;
    if(LGT_CMD_LIGHT_ONOFF == op){
        if(ON_OFF_FROM_PASSIVE_DEV == par[3]){
            return CMD_TYPE_PASSIVE_DEV;
        }else if(ON_OFF_FROM_PASSIVE_DEV_ALT == par[3]){
            return CMD_TYPE_PASSIVE_DEV_ALT;
        }
    }
#endif

    return CMD_TYPE_NORMAL;
}

// Note: par[8], par[9] of passive command have been used internal for sno2.
void set_command_type2alt(u8 *p_att_value)
{
#if PASSIVE_EN
    rf_packet_att_value_t *p = (rf_packet_att_value_t*)(p_att_value);
    u8 op = p->val[0] & 0x3F;
    u8 *par = p->val+3;
    if(LGT_CMD_LIGHT_ONOFF == op){
        if(ON_OFF_FROM_PASSIVE_DEV == par[3]){
            par[3] = ON_OFF_FROM_PASSIVE_DEV_ALT;
        }
    }
#endif
}

