/***********************************************************************
 * $Id::                                                               $
 *
 * Project:	uart: Simple UART echo for LPCXpresso 1700
 * File:	uarttest.c
 * Description:
 * 			LPCXpresso Baseboard uses pins mapped to UART3 for
 * 			its USB-to-UART bridge. This application simply echos
 * 			all characters received.
 *
 ***********************************************************************
 * Software that is described herein is for illustrative purposes only
 * which provides customers with programming information regarding the
 * products. This software is supplied "AS IS" without any warranties.
 * NXP Semiconductors assumes no responsibility or liability for the
 * use of the software, conveys no license or title under any patent,
 * copyright, or mask work right to the product. NXP Semiconductors
 * reserves the right to make changes in the software without
 * notification. NXP Semiconductors also make no representation or
 * warranty that such application will be suitable for the specified
 * use without further testing or modification.
 **********************************************************************/

/*****************************************************************************
 *   History
 *   2013.03.18  ver 1.02    Added support for UART2, not tested on LPCXpresso 1769
 *			     Added functions to Available, close, read and flush UART
 *			     Added function to determines best dividers to get a target clock rate
 *			     Based in LPC1769_FreeRTOS_Plus_Featured_Demo_002.zip
 *
 *   2010.07.01  ver 1.01    Added support for UART3, tested on LPCXpresso 1700
 *   2009.05.27  ver 1.00    Prelimnary version, first Release
 *
******************************************************************************/
#include "type.h"

#ifndef __UART_H 
#define __UART_H

#define IER_RBR		0x01
#define IER_THRE	0x02
#define IER_RLS		0x04

#define IIR_PEND	0x01
#define IIR_RLS		0x03
#define IIR_RDA		0x02
#define IIR_CTI		0x06
#define IIR_THRE	0x01

#define LSR_RDR		0x01
#define LSR_OE		0x02
#define LSR_PE		0x04
#define LSR_FE		0x08
#define LSR_BI		0x10
#define LSR_THRE	0x20
#define LSR_TEMT	0x40
#define LSR_RXFE	0x80

#define Overflow	"Buffer Overflow !!!!!\r\n"

// Accepted Error baud rate value (in percent unit) 
#define UART_ACCEPTED_BAUDRATE_ERROR	(3)			/*!< Acceptable UART baudrate error */

//#define UART_LCR_DLAB_EN		((uint8_t)(1<<7))    	/*!< UART Divisor Latches Access bit enable */
//#define UART_LCR_BITMASK		((uint8_t)(0xFF))		/*!< UART line control bit mask */

/*********************************************************************//**
 * Macro defines for Macro defines for UART Fractional divider register
 **********************************************************************/
#define UART_FDR_DIVADDVAL(n)	((uint32_t)(n&0x0F))		/**< Baud-rate generation pre-scaler divisor */
#define UART_FDR_MULVAL(n)		((uint32_t)((n<<4)&0xF0))	/**< Baud-rate pre-scaler multiplier value */
#define UART_FDR_BITMASK		((uint32_t)(0xFF))			/**< UART Fractional Divider register bit mask */

/*********************************************************************//**
 * Macro defines for Macro defines for UARTn Divisor Latch LSB register
 **********************************************************************/
#define UART_LOAD_DLL(div)	((div) & 0xFF)	/**< Macro for loading least significant halfs of divisors */
//#define UART_DLL_MASKBIT	((uint8_t)0xFF)	/*!< Divisor latch LSB bit mask */

/*********************************************************************//**
 * Macro defines for Macro defines for UARTn Divisor Latch MSB register
 **********************************************************************/
#define UART_LOAD_DLM(div)  (((div) >> 8) & 0xFF)	/**< Macro for loading most significant halfs of divisors */
//#define UART_DLM_MASKBIT	((uint8_t)0xFF)			/*!< Divisor latch MSB bit mask */

void UART0_IRQHandler( void );
void UART1_IRQHandler( void );
void UART2_IRQHandler( void );
void UART3_IRQHandler( void );

uint32_t UART_Init( uint8_t PortNum, uint32_t Baudrate );
uint32_t UART_Close( uint8_t PortNum);
uint32_t UART_Set_Divisors(uint8_t PortNum, uint32_t Baudrate);
uint32_t UART_Read(uint8_t PortNum, char *Text, char *response, uint32_t Time);

void UART_Write( uint8_t PortNum, uint8_t send);
void UART_Send( uint8_t PortNum, char *BufferPtr);
void UART_Flush (uint8_t PortNum);

void Delay_Ms(uint32_t ms);

#endif /* end __UART_H */
/*****************************************************************************
**                            End Of File
******************************************************************************/
