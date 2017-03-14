
#pragma once

#include "../common/types.h"

#define ETH_RX_PKT_BUFF_SIZE	(ETH_RX_PKT_BUFF_LEN * ETH_RX_PKT_BUFF_COUNT)

void eth_hw_init(void);
void eth_hw_send_pkt(u8 *pkt);
u16 eth_hw_read_phy_reg(u8 reg);
void eth_hw_write_phy_reg(u8 reg, u16 data);
