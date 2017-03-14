
#pragma once

#include "../common/types.h"

typedef struct eth_pkt_t
{   u32 pkt_len;
	u8 	dmac[6];
	u8 	smac[6];
	u16 type;
}eth_pkt_t; 
typedef eth_pkt_t* p_eth_pkt_t;

void eth_init(void);
void eth_pkt_recv_irq_handler(void);
void eth_send_pkt(u8 *pkt, u32 len);


