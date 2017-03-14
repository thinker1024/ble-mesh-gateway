
#include "../../proj/tl_common.h"

#include "../../proj/mcu/watchdog_i.h"
#include "../../vendor/common/user_config.h"

#include "../../proj_lib/rf_drv.h"
#include "../../proj_lib/pm.h"

extern void user_init();
extern void proc_button_ahead();
void main_loop(void);

FLASH_ADDRESS_DEFINE;
int main (void) {
	FLASH_ADDRESS_CONFIG;
	cpu_wakeup_init();

	usb_dp_pullup_en (1);

	clock_init();

	dma_init();

	gpio_init();

	irq_init();

//	usb_init();

	rf_drv_init(0);

    user_init ();

    //usb_log_init ();

    irq_enable();

	while (1) {
#if(MODULE_WATCHDOG_ENABLE)
		wd_clear();
#endif
		main_loop ();
	}
}


