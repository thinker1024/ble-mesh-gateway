
#include "../config/user_config.h"
#include "../mcu/gpio.h"
#include "../mcu/register.h"

// use static inline, because, spi flash code must reside in memory..
// these code may be embedd in flash code
static inline void mspi_wait(void){
	while(reg_master_spi_ctrl & FLD_MASTER_SPI_BUSY)
		;
}

static inline void mspi_high(void){
	reg_master_spi_ctrl = FLD_MASTER_SPI_CS;
}

static inline void mspi_low(void){
	reg_master_spi_ctrl = 0;
}

static inline u8 mspi_get(void){
	return reg_master_spi_data;
}

static inline void mspi_write(u8 c){
	reg_master_spi_data = c;
}

static inline void mspi_ctrl_write(u8 c){
	reg_master_spi_ctrl = c;
}

static inline u8 mspi_read(void){
	mspi_write(0);		// dummy, issue clock 
	mspi_wait();
	return mspi_get();
}
void spi_init(void);
void spi_write(u8 addr, u8 dat);
u8 spi_read(u8 addr);


