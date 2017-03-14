#ifndef _RF_LIGHT_LINK_LAYER_H_
#define _RF_LIGHT_LINK_LAYER_H_

#include "../../proj/tl_common.h"

#define         VENDOR_ID                       0x0211

// op cmd 11xxxxxxzzzzzzzzzzzzzzzz z's=VENDOR_ID  xxxxxx=LGT_CMD_
#define         LGT_CMD_READ_SCENE              0x00//
#define         LGT_CMD_SCENE_RSP               0x01//
#define         LGT_CMD_NOTIFY_MESH			    0x02//use between mesh
#define		    LGT_CMD_SW_DATA				    0x03
#define		    LGT_CMD_SW_RSP				    0x04
#define		    LGT_CMD_TIME_SYNC				0x05//internal use
#define		    LGT_CMD_MESH_OTA_DATA			0x06//internal use
#define             CMD_OTA_FW_VERSION					0xff00          // must larger than 0xff00
#define             CMD_OTA_START						0xff01
#define             CMD_OTA_END							0xff02
#define             CMD_STOP_MESH_OTA					0xfffe
#define             CMD_START_MESH_OTA					0xffff          // use in rf_link_data_callback()

#define		    LGT_CMD_MESH_OTA_READ			0x07//internal use
#define		        PAR_READ_VER			        0x00//internal use
#define		        PAR_READ_END			        0x01//internal use
#define		        PAR_READ_MAP			        0x02//internal use
#define		        PAR_READ_CALI			        0x03//internal use
#define		        PAR_READ_PROGRESS			    0x04//internal use
#define		    LGT_CMD_MESH_OTA_READ_RSP		0x08//internal use

#define			LGT_CMD_LIGHT_ONOFF				0x10
#define			LGT_CMD_LIGHT_ON				0x10
#define			LGT_CMD_LIGHT_OFF				0x11//internal use

#define			LGT_CMD_LIGHT_SET				0x12//set lumen
#define			LGT_CMD_SWITCH_CONFIG	        0x13//internal use
#define         LGT_CMD_LIGHT_GRP_RSP1          0x14//get group rsp: 8groups low 1bytes
#define         LGT_CMD_LIGHT_GRP_RSP2          0x15//get group rsp: front 4groups 2bytes
#define         LGT_CMD_LIGHT_GRP_RSP3          0x16//get group rsp: behind 4groups 2bytes
#define 		LGT_CMD_LIGHT_CONFIG_GRP 		0x17//add or del group
#define			LGT_CMD_LUM_UP					0x18//internal use
#define			LGT_CMD_LUM_DOWN				0x19//internal use
#define 		LGT_CMD_LIGHT_READ_STATUS 		0x1a//get status req
#define 		LGT_CMD_LIGHT_STATUS 			0x1b//get status rsp
#define 		LGT_CMD_LIGHT_ONLINE 			0x1c//get online status req//internal use
#define         LGT_CMD_LIGHT_GRP_REQ           0x1d//get group req
#define			LGT_CMD_LEFT_KEY				0x1e//internal use
#define			LGT_CMD_RIGHT_KEY				0x1f//internal use
#define			LGT_CMD_CONFIG_DEV_ADDR         0x20//add device address
#define         LGT_CMD_DEV_ADDR_RSP            0x21//rsp
#define         LGT_CMD_SET_RGB_VALUE           0x22//params[0]:1=R 2=G 3=B 4=RGB params[1~3]:R/G/B value 0~255  params[0]:5=CT params[1]=value 0~100%
#define         LGT_CMD_KICK_OUT                0x23//
#define         LGT_CMD_SET_TIME                0x24//
#define         LGT_CMD_ALARM                   0x25//
#define         LGT_CMD_READ_ALARM              0x26//
#define         LGT_CMD_ALARM_RSP               0x27//
#define         LGT_CMD_GET_TIME                0x28//
#define         LGT_CMD_TIME_RSP                0x29//
#define         LGT_CMD_USER_NOTIFY_REQ         0x2a// 
#define         LGT_CMD_USER_NOTIFY_RSP         0x2b//
#define 		LGT_CMD_SET_SW_GRP				0x2c
#define         LGT_CMD_LIGHT_RC_SET_RGB        0x2d
#define         LGT_CMD_SET_SCENE               0x2e
#define         LGT_CMD_LOAD_SCENE              0x2f

//------jump to 0x00
//------0x30 ~ 0x3f for customer
#define         LGT_CMD_LIGHT_W                 0x30


//===================================================
#define 		AUTH_TIME						60		// 60s

/*
freq = CLOCK_SYS_CLOCK_HZ / (PMW_MAX_TICK)
*/
#define         PMW_MAX_TICK_BASE	            (255)   // must 255
#define         PMW_MAX_TICK_MULTI	            (209)   // 209: freq = 600Hz
#define         PMW_MAX_TICK		            (PMW_MAX_TICK_BASE*PMW_MAX_TICK_MULTI)  // must less or equal than (255*256)

#define			PAIR_INFO_LEN				    48

#if PANEL_ENABLE
#define 		SLEEP_ENABLE				    1
#define			IRQ_GPIO0_ENABLE			    1

#define			BRIDGE_MAX_CNT				    0
#else
#define 		SLEEP_ENABLE				    0
#define			IRQ_GPIO0_ENABLE			    0

#define			BRIDGE_MAX_CNT				    8
#endif

#ifdef IRQ_TIMER1_ENABLE
#undef IRQ_TIMER1_ENABLE
#endif

#define			IRQ_TIMER0_ENABLE  			    0
#define			IRQ_TIME0_INTERVAL			    250 //us

#define			IRQ_TIMER1_ENABLE  			    1
#define			IRQ_TIME1_INTERVAL			    10 //ms

#define         ONLINE_STATUS_TIMEOUT           3000 //ms
enum{
	MODE_SWITCH = 1,
	MODE_CONFIG = 2,
};

enum{
	GET_STATUS      = 0,
	GET_GROUP1      = 1,// return 8 group_address(low 1byte)
	GET_GROUP2      = 2,// return front 4 group_address
	GET_GROUP3      = 3,// return behind 4 group_address
	GET_DEV_ADDR    = 4,// return device address
	GET_ALARM       = 5,// return alarm info
	GET_TIME        = 6,// return time info
	GET_USER_NOTIFY = 7,// return user notify info
	GET_SCENE		= 8,	//
	GET_MESH_OTA	= 9,	//
};

#define     NODE_STATUS_VALID_FLAG    0xA5
#define     MAX_GROUP_NUM   8

extern u8 sw_flag;

extern u16  device_address;
extern u16  group_address[MAX_GROUP_NUM];

extern u8 			user_data[16];
extern u8 			user_data_len ;

extern u32 Connect_suspend;
extern u32 ADV_Suspend;
void BLE_low_power_handle(u8 mode, u32 key_scan_interval);


#define SNO_IS_INVALID(n) (n[0] == 0 && n[1] == 0 && n[2] == 0)

#define		    SYS_LINK_STOP_ON_RESPONSE			BIT(0)

#define		    TLIGHT_ID0			0x03050102
#define		    TLIGHT_ID1			0x0503c119

#define			LGT_CMD_PAIRING_LIGHT_OFF		0xc1
#define			LGT_CMD_PAIRING_LIGHT_ON		0xc2
#define			LGT_CMD_PAIRING_LIGHT_SEL		0xc3
#define			LGT_CMD_PAIRING_LIGHT_CONFIRM	0xc4
#define			LGT_CMD_SET_MESH_INFO           0xc5
#define			LGT_CMD_SET_DEV_ADDR            0xc6
#define			LGT_CMD_DEL_PAIR                0xc7
#define			LGT_CMD_PAIRING_LIGHT_RESPONSE	0xc9
#define			LGT_CMD_PAIRING_LIGHT_OTA_EN	0xce
#define			LGT_CMD_PAIRING_LIGHT_STOP		0xcf

#define         LIGHT_OFF_PARAM                 0x00
#define         LIGHT_ON_PARAM                  0x01
#define             ON_OFF_NORMAL                   0x00
#define             ON_OFF_FROM_OTA                 0x01
#define             ON_OFF_FROM_PASSIVE_DEV         0x02    // sent from passive device
#define             ON_OFF_FROM_PASSIVE_DEV_ALT     0x03    // tx command: alter from passive device command

#define         LIGHT_SYNC_REST_PARAM           0x02

#define         LIGHT_ADD_GRP_PARAM             0x01
#define         LIGHT_DEL_GRP_PARAM             0x00

void	light_set_tick_per_us (int tick);

void	irq_handler(void);
void	irq_light_master_handler ();
void	irq_light_slave_handler ();

void	rf_link_master_init ();
int		rf_link_master_start (u32 interval, u32 timeout, void *p_data, int mode);
int		rf_link_master_command (u8 * p);
void 	rf_link_master_stop ();
void *	rf_link_master_get_response ();
int		rf_link_master_status ();

void	rf_link_slave_init (u32 interval);
void	rf_link_slave_proc (void);
void 	rf_link_slave_set_adv (u8 * tbl_pkt_adv, int n);
void	rf_link_slave_set_adv_mesh_name (u8 * pname, u8 data_len);
void	rf_link_slave_set_adv_private_data (u8 * pdata, u8 data_len);

void	rf_link_set_debug_adv_channel (u8 chn);

void	rf_link_light_event_callback (u8 status);
void 	rf_link_data_callback (u8 *);
int 	rf_link_response_callback (u8 *);

void	rf_link_set_max_relay (u8 num);

void	mesh_security_enable (int en);
void    mesh_get_fw_version(void);
void    blt_stall_mcu(u32 tick_stall);
u8      is_ble_connecting();
void    vendor_id_init(u16 vendor_id);

// param st is 2bytes = lumen% + rsv(0xFF)  // rf pkt : device_address+sn+lumen+rsv;  
void ll_device_status_update (u8 *st_val_par, u8 len);
int setup_ble_parameter_start(u16 delay, u16 interval_min, u16 interval_max, u16 timeout);
int	rf_link_add_tx_packet (u32 p);
int is_tx_packet_empty_suspend();
int is_add_packet_buf_ready();
int	rf_link_slave_data_write (void *ph);
void mesh_send_alarm_time_ll ();
int mesh_send_user_command ();
void rf_link_set_max_bridge(int num);

_attribute_ram_code_ void    light_sw_reboot(void);

typedef void (*fp_rf_led_ota_ok)(void);
typedef void (*fp_rf_led_ota_error)(void);
typedef void (*fp_irq_timer1)(void);
typedef void (*fp_proc_led)(void);
typedef void (*fp_rf_link_data_callback)(void *pdata);
typedef void (*fp_user_init)(void);

extern fp_rf_led_ota_ok 			p_vendor_rf_led_ota_ok;
extern fp_rf_led_ota_error			p_vendor_rf_led_ota_error;
extern fp_irq_timer1 				p_vendor_irq_timer1;
extern fp_proc_led 					p_vendor_proc_led;
extern fp_rf_link_data_callback		p_vendor_rf_link_data_callback;
extern fp_user_init 				p_vendor_user_init;

typedef void (*gateway_tx_proc_t)(u8 *);
typedef void (*gateway_rx_proc_t)(void);

extern gateway_tx_proc_t            fp_gateway_tx_proc;
extern gateway_rx_proc_t            fp_gateway_rx_proc;

//////////////// sync time  //////////////////////////

enum{
	FLD_SYNCED =				BIT(7),
};

int is_master_sync_time();
int is_there_no_sync_device();
int sync_time_callback(u8 *params, u16 cmd_delayed_ms, u32 t1_interval, u8 *p_cmd);
void sync_time_irq_handle(u8 cmd_delay2t1_cycle, u32 sync_time_alters);
void sync_alter_callback(u8 high, u8 sync_flag, u8 sync_reset);
void sync_alter_action_callback(u8 high);
void sync_cmd_action_callback(u8 sync_reset);
void sync_time_reset();
void sync_time_en(int en);

int blt_packet_crc24 (unsigned char *p, int n, int crc);
u32 clock_time_exceed_lib(u32 ref, u32 span_us);

int mesh_push_user_command (int sno, u16 dst, u8 *p, u8 len);
int mesh_push_user_command_no_relay (int sno, u16 dst, u8 *p, u8 len);

typedef struct{
    u8 version[4];
    u8 dev_mode;
}mesh_ota_pkt_start_command_t;  // don't modify element in it

void mesh_ota_master_proc ();
int mesh_ota_slave_save_data(u8 *params);
void mesh_ota_master_start(u8 *adr, u32 len, u8 dev_mode);
int mesh_ota_slave_set_response(u8 *params, u8 type);
void mesh_ota_master_read_rsp_handle(u8 *rsp_val, u8 repeat_flag);
void mesh_ota_master_read_rsp_handle_user(u8 *p, u8 repeat_flag);
void mesh_ota_master_read_idle_cnt_handle();
void mesh_ota_master_read_idle_cnt_handle_user();
int is_mesh_ota_slave_running();
u8 mesh_ota_master_get_st();
int is_not_master_sending_ota_st();
void mesh_ota_master_cancle ();
int is_mesh_ota_master_no_enc(u8 *ps);
int mesh_ota_slave_need_ota(u8 *params);
void mesh_ota_slave_reboot_delay ();
void register_mesh_ota_master_ui (void *p);
int is_cmd_long_par(u8 op);

#endif
