
#include "../../proj/tl_common.h"
#include "../../proj_lib/rf_drv.h"

#if 0
void (*rf_rx_irq_handler)(void);
void (*rf_tx_irq_handler)(void);
void (*gpio0_user_irq_handler)(void);
void (*gpio_user_irq_handler)(void);
void (*timer_irq1_handler)(void);


_attribute_ram_code_ void irq_handler(void)
{

	u32 src = reg_irq_src;

#if (MODULE_RF_ENABLE)
	u8  src_rf = reg_rf_rx_status;
#endif

	if(IRQ_TIMER1_ENABLE && (src & FLD_IRQ_TMR1_EN)){
		if(timer_irq1_handler)
			timer_irq1_handler();
		reg_irq_src = FLD_IRQ_TMR1_EN;
	}

	// should use FLD_IRQ_GPIO_RISC2_EN for compatibility with 86xx/85xx
	if(IRQ_GPIO_ENABLE && (src & FLD_IRQ_GPIO_RISC0_EN)){
	#ifndef WIN32
		if(gpio_user_irq_handler)
			gpio_user_irq_handler();
	#endif
		reg_irq_src = FLD_IRQ_GPIO_RISC0_EN;
	}

#if(!APPLICATION_DONGLE)
	if(IRQ_GPIO0_ENABLE && (src & FLD_IRQ_GPIO_RISC2_EN)){
	#ifndef WIN32
		if(gpio0_user_irq_handler)
			gpio0_user_irq_handler();
	#endif
		reg_irq_src = FLD_IRQ_GPIO_RISC2_EN;
	}
#endif


#if (MODULE_RF_ENABLE)
	if(IRQ_RF_RTX_ENABLE && (src_rf & FLD_RF_RX_INTR)){
		if(rf_rx_irq_handler)
			rf_rx_irq_handler();
		reg_rf_rx_status = FLD_RF_RX_INTR;

	}

	if(IRQ_RF_RTX_ENABLE && (src_rf & FLD_RF_TX_INTR)){
		if(rf_tx_irq_handler)
			rf_tx_irq_handler();
		reg_rf_rx_status = FLD_RF_TX_INTR;
	}
#endif


}

#endif
