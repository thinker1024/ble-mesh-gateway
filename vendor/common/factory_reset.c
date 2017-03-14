#include "../../proj/tl_common.h"
#include "common.h"

FLASH_ADDRESS_EXTERN;

//////////////////Factory Reset///////////////////////////////////////////////////////////////////////
extern void rf_led_ota_ok(void);
extern void light_sw_reboot(void);

static int adr_reset_cnt_idx = 0;
static int reset_cnt = 0;

#define SERIALS_CNT                     (5)   // must less than 7

u8 factory_reset_serials[SERIALS_CNT * 2]   = { 0, 3,    // [0]:must 0
                                                0, 3,    // [2]:must 0
                                                0, 3,    // [4]:must 0
                                                3, 30,
                                                3, 30};

#define RESET_CNT_RECOUNT_FLAG          0
#define RESET_FLAG                      0x80

void	reset_cnt_clean ()
{
	if (adr_reset_cnt_idx < 3840)
	{
		return;
	}
	flash_erase_sector (flash_adr_reset_cnt);
	adr_reset_cnt_idx = 0;
}

void write_reset_cnt (u8 cnt)
{
	flash_write_page (flash_adr_reset_cnt + adr_reset_cnt_idx, 1, (u8 *)(&cnt));
}

void clear_reset_cnt ()
{
    write_reset_cnt(RESET_CNT_RECOUNT_FLAG);
}

int reset_cnt_get_idx ()		//return 0 if unconfigured
{
	u8 *pf = (u8 *)flash_adr_reset_cnt;
	for (adr_reset_cnt_idx=0; adr_reset_cnt_idx<4096; adr_reset_cnt_idx++)
	{
	    u8 restcnt_bit = pf[adr_reset_cnt_idx];
		if (restcnt_bit != RESET_CNT_RECOUNT_FLAG)	//end
		{
        	if(((u8)(~(BIT(0)|BIT(1)|BIT(2)|BIT(3))) == restcnt_bit)  // the fourth not valid
        	 ||((u8)(~(BIT(0)|BIT(1)|BIT(2)|BIT(3)|BIT(4)|BIT(5))) == restcnt_bit)){  // the fifth not valid
                clear_reset_cnt();
            }else{
			    break;
			}
		}
	}

    reset_cnt_clean();
    
	return 1;
}

u8 get_reset_cnt_bit ()
{
	if (adr_reset_cnt_idx < 0)
	{
	    reset_cnt_clean();
		return 0;
	}
	return *((u8 *)(flash_adr_reset_cnt + adr_reset_cnt_idx));
}

void increase_reset_cnt ()
{
	u8 restcnt_bit = get_reset_cnt_bit();
	foreach(i,8){
        if(restcnt_bit & BIT(i)){
            if(i < 3){
                reset_cnt = i;
            }else if(i < 5){
                reset_cnt = 3;
            }else if(i < 7){
                reset_cnt = 4;
            }
            
            restcnt_bit &= ~(BIT(i));
            write_reset_cnt(restcnt_bit);
            break;
        }
	}
}

int factory_reset(){
	u8 r = irq_disable ();
    clear_reset_cnt();

	for(int i = 1; i < (FLASH_ADR_PAR_MAX - flash_adr_mac) / 4096; ++i){
		flash_erase_sector(flash_adr_mac + i*0x1000);
	}
    irq_restore(r);
	return 0; 
}

int factory_reset_handle ()
{
    reset_cnt_get_idx();   
    u8 restcnt_bit; 
    restcnt_bit = get_reset_cnt_bit();
	if(restcnt_bit == RESET_FLAG){
        irq_disable();
        factory_reset();
        rf_led_ota_ok();
	    light_sw_reboot();
	}else{
        increase_reset_cnt();
	}
	return 0;
}

int factory_reset_cnt_check ()
{
    static u8 clear_st = 3;
    static u32 reset_check_time;

	if(0 == clear_st) return 0;

	if(3 == clear_st){
        clear_st--;
        reset_check_time = factory_reset_serials[reset_cnt*2];
    }
    
	if((2 == clear_st) && clock_time_exceed(0, reset_check_time*1000*1000)){
	    clear_st--;
	    reset_check_time = factory_reset_serials[reset_cnt*2 + 1];
	    if(3 == reset_cnt || 4 == reset_cnt){
            increase_reset_cnt();
        }
	}
    
	if((1 == clear_st) && clock_time_exceed(0, reset_check_time*1000*1000)){
	    clear_st = 0;
        clear_reset_cnt();
	}
	
	return 0;
}

enum{
	KICKOUT_OUT_OF_MESH = 0,
	KICKOUT_DEFAULT_NAME,
	KICKOUT_MODE_MAX,
};

void kick_out(u8 par){
    factory_reset();

    if(KICKOUT_OUT_OF_MESH == par){
        u8 buff[16];
        memset (buff, 0, 16);
        extern u8 pair_config_mesh_ltk[];
        memcpy (buff, pair_config_mesh_ltk, 16);
        flash_write_page (flash_adr_pairing + 48, 16, buff);
        memset (buff, 0, 16);
        memcpy (buff, MESH_PWD, strlen(MESH_PWD) > 16 ? 16 : strlen(MESH_PWD));
	    encode_password(buff);
        flash_write_page (flash_adr_pairing + 32, 16, buff);
        memset (buff, 0, 16);
        memcpy (buff, OUT_OF_MESH, strlen(OUT_OF_MESH) > 16 ? 16 : strlen(OUT_OF_MESH));
        flash_write_page (flash_adr_pairing + 16, 16, buff);
        memset (buff, 0, 16);
        buff[0] = PAIR_VALID_FLAG;
        if(pair_config_pwd_encode_enable){
            buff[15] = PAIR_VALID_FLAG;
        }
        flash_write_page (flash_adr_pairing, 16, buff);
    }
    
    rf_led_ota_ok();
}

