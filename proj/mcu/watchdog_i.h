#pragma once
#include "register.h"

//  watchdog use timer 2
//STATIC_ASSERT((WATCHDOG_INIT_TIMEOUT * CLOCK_SYS_CLOCK_1MS >> WATCHDOG_TIMEOUT_COEFF) > 0);
static inline void wd_set_interval(int ms)	//  in ms
{
	assert((ms * CLOCK_SYS_CLOCK_1MS >> WATCHDOG_TIMEOUT_COEFF) > 0);
	SET_FLD_V(reg_tmr_ctrl, FLD_TMR_WD_CAPT, (ms * CLOCK_SYS_CLOCK_1MS >> WATCHDOG_TIMEOUT_COEFF), FLD_TMR2_MODE, 0);
	reg_tmr2_tick = 0;
}

static inline void wd_start(void)
{
	SET_FLD(reg_tmr_ctrl, FLD_TMR_WD_EN);
}

static inline void wd_stop(void)
{
	CLR_FLD(reg_tmr_ctrl, FLD_TMR_WD_EN);
}

static inline void wd_clear(void)
{
	SET_FLD(reg_tmr_ctrl, FLD_CLR_WD);
}

