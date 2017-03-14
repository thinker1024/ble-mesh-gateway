/**************************************************************************************************
  Filename:       	uart.h
  Author:			junjun.xu@telink-semi.com
  Created Date:	2016/06/05
  
  Description:    This file contains the details of enum variables and functions used in the uart.c file

  
**************************************************************************************************/
#ifndef 	uart_H
#define 	uart_H

#define UART_DATA_LEN    156      // data max 252
typedef struct{
    u32 len;                      // data max 252
    u8 data[UART_DATA_LEN];
}uart_data_t;
STATIC_ASSERT((sizeof(uart_data_t) % 16) == 0);
uart_data_t T_txdata_user;
uart_data_t T_txdata_buf;      // not for user

uart_data_t T_rxdata_user;
uart_data_t T_rxdata_buf;      // data max 252, user must copy rxdata to other Ram,but not use directly
unsigned char uart_rx_true;

enum HARDWARECONTROL{
	CTSWODDPARITY = 0x0e,
	CTSWEVENPARITY = 0x06,
	CTSONLY = 0x02,
	ODDPARITY = 0x0C,
	EVENPARITY = 0x04,
	NOCONTROL = 0x00,
};
enum UARTIRQSOURCE{
	UARTRXIRQ,
	UARTTXIRQ,
	UARTNOIRQ,
};

enum{
	UARTRXIRQ_MASK  = BIT(0),
	UARTTXIRQ_MASK  = BIT(1),
	UARTIRQ_MASK    = UARTRXIRQ_MASK | UARTTXIRQ_MASK,
};

#define CLK32M_UART9600			uart_Init(237,13,1,1,NOCONTROL)
#define	CLK32M_UART115200		uart_Init(19,13,1,1,NOCONTROL)
#define	CLK16M_UART115200		uart_Init(9,13,1,1,NOCONTROL)
#define	CLK16M_UART9600			uart_Init(103,15,1,1,NOCONTROL)

//UART_TX/UART_RX gpio pin config
#define	   UART_GPIO_CFG_PA6_PA7()  do{\
										*(volatile unsigned char  *)0x800586 &= 0x3f;\
										*(volatile unsigned char  *)0x8005b0 |= 0x80;\
								    }while(0) 
#define	   UART_GPIO_CFG_PB2_PB3()  do{\
										*(volatile unsigned char  *)0x80058e &= 0xf3;\
										*(volatile unsigned char  *)0x8005b1 |= 0x0c;\
								    }while(0)  
#define	   UART_GPIO_CFG_PC2_PC3()  do{\
										*(volatile unsigned char  *)0x800596 &= 0xf3;\
										*(volatile unsigned char  *)0x8005b2 |= 0x0c;\
								    }while(0)  

#define UART_GPIO_8266              1

#define UART_GPIO_8267_PA6_PA7      1
#define UART_GPIO_8267_PC2_PC3      2
#define UART_GPIO_8267_PB2_PB3      3

void uart_io_init(unsigned char uart_io_sel);

/**********************************************************
*
*	@brief	reset uart module
*	
*	@param	none
*
*	@return	none
*/
extern void uart_Reset(void );


/**********************************************************
*	
*	@brief	clear error state of uart rx, maybe used when application detected UART not work
*
*	@parm	none
*
*	@return	'1' RX error flag rised and cleard success; '0' RX error flag not rised 
*
*/
unsigned char uart_ErrorCLR(void);


/*******************************************************
*
*	@brief	uart initiate, set uart clock divider, bitwidth and the uart work mode
*
*	@param	uartCLKdiv - uart clock divider
*			bwpc - bitwidth, should be set to larger than 2
*			en_rx_irq - '1' enable rx irq; '0' disable.
*			en_tx_irq - enable tx irq; '0' disable.
*			hdwC - enum variable of hardware control functions
*
*	@return	'1' set success; '0' set error probably bwpc smaller than 3.
*
*		BaudRate = sclk/((uartCLKdiv+1)*(bwpc+1))  
*		SYCLK = 16Mhz
		115200		9			13
		9600		103			15
*
*		SYCLK = 32Mhz
*		115200		19			13
		9600		237			13	
*/
extern unsigned char uart_Init(unsigned short uartCLKdiv, unsigned char bwpc,unsigned char en_rx_irq,unsigned char en_tx_irq,enum HARDWARECONTROL hdwC);



/********************************************************************************
*	@brief	uart send data function, this  function tell the DMA to get data from the RAM and start 
*			the DMA send function
*
*	@param	sendBuff - send data buffer
*
*	@return	'1' send success; '0' DMA busy
*/
extern unsigned char uart_Send(unsigned char* addr);
extern unsigned char uart_print(const char* p);
extern unsigned char uart_byte(u8 p);


/********************************************************************
*	
*	@brief	uart receive function, call this function to get the UART data
*
*	@param	userDataBuff - data buffer to store the uart data
*
*	@return	'0' rx error; 'rxLen' received data length
*/
//extern unsigned short uart_Rec(unsigned char* addr);

/******************************************************************************
*
*	@brief		get the uart IRQ source and clear the IRQ status, need to be called in the irq process function
*
*	@return		uart_irq_src- enum variable of uart IRQ source, 'UARTRXIRQ' or 'UARTTXIRQ'
*
*/
extern u8 uart_IRQSourceGet(void);

/****************************************************************************************
*
*	@brief	data receive buffer initiate function. DMA would move received uart data to the address space, uart packet length
*			needs to be no larger than (recBuffLen - 4).
*
*	@param	*recAddr:	receive buffer's address info.
*			recBuffLen:	receive buffer's length, the maximum uart packet length should be smaller than (recBuffLen - 4)
*
*	@return	none
*/

extern void uart_BuffInit(unsigned char *recAddr, unsigned short recBuffLen, unsigned char *txAddr);

void uart_clr_tx_busy_flag();

#endif

