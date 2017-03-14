
#pragma once

#include "../../proj/common/types.h"

void myi2c_sim_burst_read(u8 id, u32 addr,u8 *p,u32 n);
void myi2c_sim_burst_write(u8 id, u32 addr,u8 *p,u32 n);

