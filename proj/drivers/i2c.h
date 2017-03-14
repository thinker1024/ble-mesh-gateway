
#pragma once

#include "../common/types.h"

void i2c_sim_init(void);
void i2c_sim_write(u8 id, u8 addr, u8 dat);
u8 i2c_sim_read(u8 id, u8 addr);
void i2c_sim_burst_read(u8 id, u8 addr, u8 *p, u8 n);
void i2c_sim_burst_write(u8 id, u8 addr,u8 *p,u8 n);

void i2c_init (void);
int i2c_burst_write(u8 id, u16 adr, u8 * buff, int len);
int i2c_burst_read(u8 id, u16 adr, u8 * buff, int len);
void i2c_write(u8 id, u16 adr, u8 dat);
u8 i2c_read(u8 id, u16 adr);
void i2c_slave_init(u8 irq_en);
void i2c_pin_initial(u32 gpio_sda,u32 gpio_scl);

extern u8 i2c_map_buf[128];

typedef enum {
	I2C_IRQ_NONE = 0,
	I2C_IRQ_HOST_READ_WRITE,
	I2C_IRQ_HOST_READ_ONLY,
}I2C_IrqSrc;

typedef enum {
	I2C_SLAVE_DMA = 0,
	I2C_SLAVE_MAP,
}I2C_SlaveMode;

typedef enum {
	I2C_IRQ_DISABLE = 0,
	I2C_IRQ_ENABLE,
}I2C_IrqStatus;

#define     i2c_slave_rev_irq_en()        write_reg8(0x640, read_reg8(0x640)|0x80)
#define     i2c_slave_rev_irq_dis()       write_reg8(0x640, read_reg8(0x640)&0x7f)
#define     i2c_slave_rev_irq_clear()     write_reg8(0x22,read_reg8(0x22)|0x02)

I2C_IrqSrc I2C_SlaveIrqGet(void);
void I2C_SlaveIrqClr(I2C_IrqSrc src);

