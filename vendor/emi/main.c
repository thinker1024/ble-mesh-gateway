
#include "../../proj/tl_common.h"

#if(!__PROJECT_PM_TEST__)

#include "../../proj/mcu/watchdog_i.h"
#include "../../vendor/common/user_config.h"

#include "../../proj_lib/rf_drv.h"
#include "../../proj_lib/pm.h"

extern void user_init();
void main_loop(void);

_attribute_ram_code_ void irq_handler(void)
{
	u32 src = reg_irq_src;
	src = src;
#if 0
	if(src & FLD_IRQ_TMR1_EN){
		irq_host_timer1();
		reg_tmr_sta = FLD_TMR_STA_TMR1;
	}
#endif

	u8  src_rf = reg_rf_irq_status;
	if(src_rf & FLD_RF_IRQ_RX){
		//irq_ll_rx();
	}

	if(src_rf & FLD_RF_IRQ_TX){
		//irq_ll_tx();
	}
}

FLASH_ADDRESS_DEFINE;
int main (void) {
	FLASH_ADDRESS_CONFIG;
	cpu_wakeup_init();

	clock_init();

	dma_init();

//	gpio_init();

//	irq_init();

//	usb_init();

	rf_drv_init(0);

//    user_init ();

    irq_disable();

	while (1) {
		main_loop ();
	}
}

#endif


