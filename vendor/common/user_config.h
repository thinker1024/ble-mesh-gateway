
#pragma once

#if (__PROJECT_BLE_MASTER__)
	#include "../ble_master/app_config.h"
#elif (__PROJECT_BEACON_DETECTOR__)
	#include "../ble_dongle_beacon/app_config.h"
#elif (__PROJECT_LIGHT_8266__)
	#include "../light_8266/light.h"
#elif (__PROJECT_LIGHT_8267__)
	#include "../light_8267/light.h"
#elif (__PROJECT_LIGHT_8267_UART__)
	#include "../light_8267_uart/light.h"
#elif (__PROJECT_LIGHT_NO_MESH__)
	#include "../light_no_mesh/light.h"
#elif (__PROJECT_MASTER_LIGHT_8267__)
	#include "../master_light_8267/light.h"
#elif (__PROJECT_8266_MESH_CONFIG__)
	#include "../8266_mesh_config/app_config.h"
#elif (__PROJECT_8263_SWITCH__)
	#include "../8263_switch/app_config.h"
#elif (__PROJECT_MOTIONSENSOR_8267__)
	#include "../motionsensor_8267/app_config.h"
#elif (__PROJECT_LIGHT_SWITCH__)
	#include "../light_switch/light_switch.h"
#elif (__PROJECT_OTA_BOOT__)
	#include "../ota_boot/app_config.h"
#elif (__PROJECT_OTA_MASTER__)
	#include "../ota_master/app_config.h"
#elif (__PROJECT_MONITOR_8266__)
	#include "../monitor_8266/app_config.h"
#elif (__PROJECT_EMI__)
	#include "../emi/main_emi.h"
#else
	#include "user_config_common.h"
#endif

#define	FLASH_SECTOR_SIZE       (4096)

#define	FLASH_ADDRESS_DEFINE						\
		u32 flash_adr_mac;							\
		u32 flash_adr_pairing;						\
		u32 flash_adr_dev_grp_adr;			        \
		u32 flash_adr_lum;							\
		u32 flash_adr_ota_master;					\
		u32 flash_adr_reset_cnt;					\
		u32 flash_adr_alarm;					    \
		u32 flash_adr_scene;					    \
		u32 flash_adr_user_data;					    \
		u32 flash_adr_light_new_fw;
#define FLASH_ADDRESS_CONFIG								\
		flash_adr_mac 			= FLASH_ADR_MAC;			\
		flash_adr_pairing 		= FLASH_ADR_PAIRING;		\
		flash_adr_dev_grp_adr   = FLASH_ADR_DEV_GRP_ADR;    \
		flash_adr_lum 			= FLASH_ADR_LUM;			\
		flash_adr_ota_master 	= FLASH_ADR_OTA_NEW_FW;     \
		flash_adr_reset_cnt 	= FLASH_ADR_RESET_CNT;      \
		flash_adr_alarm 	    = FLASH_ADR_ALARM;          \
		flash_adr_scene 	    = FLASH_ADR_SCENE;          \
		flash_adr_user_data   = FLASH_ADR_USER_DATA;          \
        flash_adr_light_new_fw  = FLASH_ADR_LIGHT_NEW_FW;          
#define FLASH_ADDRESS_EXTERN						\
		extern u32 flash_adr_mac;					\
		extern u32 flash_adr_pairing;				\
		extern u32 flash_adr_dev_grp_adr;		    \
		extern u32 flash_adr_lum;					\
		extern u32 flash_adr_ota_master;            \
		extern u32 flash_adr_reset_cnt;             \
		extern u32 flash_adr_alarm;                 \
		extern u32 flash_adr_scene;                 \
		extern u32 flash_adr_user_data;                 \
		extern u32 flash_adr_light_new_fw;

#define			FLASH_ADR_MAC				0x76000
#define			FLASH_ADR_DC				0x76010
#define			FLASH_ADR_TP_LOW		    0x76011
#define			FLASH_ADR_TP_HIGH		    0x76012
#define			FLASH_ADR_PAIRING			0x77000
#define			FLASH_ADR_LUM				0x78000
#define			FLASH_ADR_DEV_GRP_ADR	    0x79000
#define			FLASH_ADR_RESET_CNT			0x7A000
#define			FLASH_ADR_ALARM			    0x7B000
#define			FLASH_ADR_SCENE			    0x7C000
#define			FLASH_ADR_USER_DATA		    0x7D000
//////vendor define here from 0x7D000 to 0x7FFFF...
//////vendor define here ...
#define			FLASH_ADR_PAR_MAX			0x80000

// begin with 0x1F000, resv address, don't change
#define			FLASH_ADR_OTA_BOOT			0x1F000
#define			FLASH_ADR_OTA_NEW_FW		0x20000
#define			FLASH_ADR_OTA_READY_FLAG	0x3F000		
#define         FLASH_ADR_LIGHT_NEW_FW      0x40000
#define         ERASE_SECTORS_FOR_OTA       32

// note !!!DEVICE_NAME max 13 bytes  
//#define DEVICE_NAME 'T', 'e', 'l', 'i', 'n', 'k', ' ', 't', 'L', 'i', 'g' ,'h', 't'
#define DEVICE_NAME 'I','N','E','S','A'

#ifndef ADV_UUID
#define ADV_UUID                            0
#endif
#ifndef PA_ENABLE
#define PA_ENABLE                           0
#endif

#if(PA_ENABLE)
#ifndef PA_TXEN_PIN
#define PA_TXEN_PIN                         GPIO_GP10
#endif

#ifndef PA_RXEN_PIN
#define PA_RXEN_PIN                         GPIO_GP11
#endif
#endif

#ifndef PASSIVE_EN
#define PASSIVE_EN                          0
#endif

enum{
    PA_TX   = 0,
    PA_RX   = 1,
    PA_OFF  = 2,
};

#define PAIR_VALID_FLAG						0xFA
//u8 advData[] = {0x02, 0x01, 0x05};
//0~max_mesh_name_len bytes  (strlen(advData) + strlen(MESH_NAME) + sizeof(ll_adv_private_t))<=31
#define MESH_NAME							"test"
//max 16 bytes
#define MESH_PWD							"1123"
//max 16 bytes
#define MESH_PWD_ENCODE_SK					"0123456789ABCDEF"
//max 16 bytes, random data from master
#define MESH_LTK							0xc0, 0xc1, 0xc2, 0xc3, 0xc4, 0xc5, 0xc6, 0xc7, 0xd8, 0xd9, 0xda, 0xdb, 0xdc, 0xdd, 0xde, 0xdf

#define OUT_OF_MESH						    "out_of_mesh"

#define IBEACON_ENABLE                      0
// unit: 100ms
#define IBEACON_INTERVAL                    10

#define	LIGHT_MODE_DIM	    1
#define	LIGHT_MODE_CCT	    2
#define	LIGHT_MODE_RGBW	    3
#define	LIGHT_MODE_RGB	    4
#define	LIGHT_MODE_CSLEEP	5

#define	LIGHT_MODE_SWITCH	        0x20

#define	LIGHT_MODE_MOTION_SENSOR	0x30

#if (__PROJECT_LIGHT_SWITCH__)
#define LIGHT_MODE		LIGHT_MODE_SWITCH
#elif (__PROJECT_MOTIONSENSOR_8267__)
#define LIGHT_MODE		LIGHT_MODE_MOTION_SENSOR
#else
#define LIGHT_MODE		LIGHT_MODE_RGB
#endif

#if(LIGHT_MODE == LIGHT_MODE_CCT)
#define RGB_MAP_MAX			51
#else   //if(LIGHT_MODE == LIGHT_MODE_RGB)
#define RGB_MAP_MAX			64
#endif

