
#pragma once

#include "../common/types.h"

void spi_sim_init(u32 pin_clk, u32 pin_dat_out);
u8 spi_sim_read(u32 pin_clk, u32 pin_dat_in);
void spi_sim_write(u32 pin_clk, u32 pin_dat_out, u8 data);

void spi_init(void);
void spi_write(u8 addr, u8 dat);
u8 spi_read(u8 addr);


