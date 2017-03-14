
#include "../../proj/tl_common.h"

#include "../../proj/drivers/flash.h"
#include "../../vendor/common/user_config.h"

#include "../../proj_lib/pm.h"
#include "../../proj/drivers/flash.h"

NOTE("size of '.data','.text' and '.rodata' must be 0.");
NOTE("if not,please check auto loading size of flash in cstartup_8266.S and boot.link");
//because ota_boot use cstartup_8266.S but not cstartup_8266_RAM.S, and it run all in RAM.

_attribute_ram_code_ void irq_handler (void)
{

}

u8		buff[256];

_attribute_ram_code_ void ota_reboot(void){
	//REG_ADDR8(0x602) = 0x84;				//reboot, sometimes can not reboot from flash but RAM.
	REG_ADDR8(0x6f) = 0x20;  //reboot
	while (1);
}

FLASH_ADDRESS_DEFINE;
_attribute_ram_code_ int main (void) {
	FLASH_ADDRESS_CONFIG;
	//cpu_wakeup_init();

	//REG_ADDR8 (0x01) = 0xf5;
	//clock_init();
	REG_ADDR8(0x61) = 0x00;
	REG_ADDR8(0x64) = 0xff;
	REG_ADDR8(0x74f) = 0x01;		//enable system tick;
	irq_disable ();
 
#if 1

#if 0
	reg_master_spi_ctrl = FLD_MASTER_SPI_CS;
	sleep_us (1);
	reg_master_spi_ctrl = 0;
	reg_master_spi_data = FLASH_WRITE_ENABLE_CMD;
	while(reg_master_spi_ctrl & FLD_MASTER_SPI_BUSY);

	reg_master_spi_ctrl = FLD_MASTER_SPI_CS;
	sleep_us (1);
	reg_master_spi_ctrl = 0;
	reg_master_spi_data = 0x01;							//write status: un-protection
	while(reg_master_spi_ctrl & FLD_MASTER_SPI_BUSY);
	reg_master_spi_data = 0x0;
	while(reg_master_spi_ctrl & FLD_MASTER_SPI_BUSY);
	reg_master_spi_ctrl = FLD_MASTER_SPI_CS;
#endif

	flash_read_page (flash_adr_light_new_fw, 256, buff);
	int	n = *(u32 *)(buff + 0x18);

	for (int i=0; i<n; i+=256)
	{
		if ((i & 0xfff) == 0)
		{
			flash_erase_sector (i);
		}

		flash_read_page (flash_adr_light_new_fw + i, 256, buff);
		flash_write_page (i, 256, buff);
	}

	buff[0] = 0;

	//REG_ADDR8 (0x01) = 0xf6;

	flash_write_page (FLASH_ADR_OTA_READY_FLAG, 1, buff);	//clear OTA flag

#else
	gpio_write (GPIO_PC2, 1);
	gpio_set_output_en (GPIO_PC2, 1);
#endif
    
	ota_reboot();
}


