/***********************************************************************
 * $Id::                                                               $
 *
 * Project:	uart: Simple UART echo for LPCXpresso 1700
 * File:	uart.c
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
 *   2013.03.18  ver 1.02  Used NXP forum resources and sample code :
 *   			 Added support for UART2, not tested on LPCXpresso 1769
 *			     Added function to close UART
 *			     Added function to determines best dividers to get a target clock rate
 *			     Based in LPC1769_FreeRTOS_Plus_Featured_Demo_002.zip and SerialV3.zip
 *				 URL: http://www.lpcware.com/content/blog/lpc17xx-uart-simpler-way-calculate-baudrate-timming
 *
 *   2010.07.01  ver 1.01    Added support for UART3, tested on LPCXpresso 1700
 *   2009.05.27  ver 1.00    Prelimnary version, first Release
 *
******************************************************************************/
#include "chip.h"
#include "uart.h"
#include <string.h>
#include "config.h"

volatile uint32_t UART0Status, UART1Status, UART2Status, UART3Status;
volatile uint8_t UART0TxEmpty = 1, UART1TxEmpty = 1, UART2TxEmpty = 1, UART3TxEmpty=1;
volatile uint8_t UART0Buffer[BUFSIZE], UART1Buffer[BUFSIZE], UART2Buffer[BUFSIZE], UART3Buffer[BUFSIZE];
volatile uint32_t UART0Count = 0, UART1Count = 0, UART2Count = 0, UART3Count = 0;

typedef struct
{
  volatile uint32_t PINSEL0;
  volatile uint32_t PINSEL1;
  volatile uint32_t PINSEL2;
  volatile uint32_t PINSEL3;
  volatile uint32_t PINSEL4;
  volatile uint32_t PINSEL5;
  volatile uint32_t PINSEL6;
  volatile uint32_t PINSEL7;
  volatile uint32_t PINSEL8;
  volatile uint32_t PINSEL9;
  volatile uint32_t PINSEL10;
       uint32_t RESERVED0[5];
  volatile uint32_t PINMODE0;
  volatile uint32_t PINMODE1;
  volatile uint32_t PINMODE2;
  volatile uint32_t PINMODE3;
  volatile uint32_t PINMODE4;
  volatile uint32_t PINMODE5;
  volatile uint32_t PINMODE6;
  volatile uint32_t PINMODE7;
  volatile uint32_t PINMODE8;
  volatile uint32_t PINMODE9;
  volatile uint32_t PINMODE_OD0;
  volatile uint32_t PINMODE_OD1;
  volatile uint32_t PINMODE_OD2;
  volatile uint32_t PINMODE_OD3;
  volatile uint32_t PINMODE_OD4;
  volatile uint32_t I2CPADCFG;
} LPC_PINCON_TypeDef;

typedef struct
{
  volatile uint32_t FLASHCFG;
       uint32_t RESERVED0[31];
  volatile uint32_t PLL0CON;
  volatile uint32_t PLL0CFG;
  volatile const  uint32_t PLL0STAT;
  volatile  uint32_t PLL0FEED;
       uint32_t RESERVED1[4];
  volatile uint32_t PLL1CON;
  volatile uint32_t PLL1CFG;
  volatile const  uint32_t PLL1STAT;
  volatile uint32_t PLL1FEED;
       uint32_t RESERVED2[4];
  volatile uint32_t PCON;
  volatile uint32_t PCONP;
       uint32_t RESERVED3[15];
  volatile uint32_t CCLKCFG;
  volatile uint32_t USBCLKCFG;
  volatile uint32_t CLKSRCSEL;
  volatile uint32_t	CANSLEEPCLR;
  volatile uint32_t	CANWAKEFLAGS;
       uint32_t RESERVED4[10];
  volatile uint32_t EXTINT;
       uint32_t RESERVED5;
  volatile uint32_t EXTMODE;
  volatile uint32_t EXTPOLAR;
       uint32_t RESERVED6[12];
  volatile uint32_t RSID;
       uint32_t RESERVED7[7];
  volatile uint32_t SCS;
  volatile uint32_t IRCTRIM;
  volatile uint32_t PCLKSEL0;
  volatile uint32_t PCLKSEL1;
       uint32_t RESERVED8[4];
  volatile uint32_t USBIntSt;
  volatile uint32_t DMAREQSEL;
  volatile uint32_t CLKOUTCFG;
 } LPC_SC_TypeDef;

#define LPC_APB0_BASE         (0x40000000UL)
#define LPC_APB1_BASE         (0x40080000UL)

#define LPC_PINCON_BASE       (LPC_APB0_BASE + 0x2C000)
#define LPC_PINCON            ((LPC_PINCON_TypeDef    *) LPC_PINCON_BASE   )

#define LPC_SC_BASE           (LPC_APB1_BASE + 0x7C000)
#define LPC_SC                ((LPC_SC_TypeDef        *) LPC_SC_BASE       )



void Delay_Ms(uint32_t ms){
 	uint32_t i;
 	for(i=0; i<(((uint32_t)7140)*ms);i++); // loop 7140 approximate 1 millisecond
}



/*****************************************************************************
** Function name:		UART0_IRQHandler
**
** Descriptions:		UART0 interrupt handler
**
** parameters:			None
** Returned value:		None
** 
*****************************************************************************/
void UART0_IRQHandler (void) 
{
  uint8_t IIRValue, LSRValue;
  uint8_t Dummy = Dummy;
	
  IIRValue = LPC_UART0->IIR;
    
  IIRValue >>= 1;			/* skip pending bit in IIR */
  IIRValue &= 0x07;			/* check bit 1~3, interrupt identification */
  if ( IIRValue == IIR_RLS )		/* Receive Line Status */
  {
	LSRValue = LPC_UART0->LSR;
	/* Receive Line Status */
	if ( LSRValue & (LSR_OE|LSR_PE|LSR_FE|LSR_RXFE|LSR_BI) )
	{
	  /* There are errors or break interrupt */
	  /* Read LSR will clear the interrupt */
	  UART0Status = LSRValue;
	  Dummy = LPC_UART0->RBR;		/* Dummy read on RX to clear 
							interrupt, then bail out */
	  return;
	}
	if ( LSRValue & LSR_RDR )	/* Receive Data Ready */			
	{
		/* If no error on RLS, normal ready, save into the data buffer. */
		/* Note: read RBR will clear the interrupt */
	  UART0Buffer[UART0Count] = LPC_UART0->RBR;
	  UART0Count++;
	  if ( UART0Count == BUFSIZE )
	  {
		UART0Count = 0;		/* buffer overflow */
	  }
	}
  }
  else if ( IIRValue == IIR_RDA )	/* Receive Data Available */
  {
	/* Receive Data Available */
	UART0Buffer[UART0Count] = LPC_UART0->RBR;
	UART0Count++;
	if ( UART0Count == BUFSIZE )
	{
	  UART0Count = 0;		/* buffer overflow */
	}
  }
  else if ( IIRValue == IIR_CTI )	/* Character timeout indicator */
  {
	/* Character Time-out indicator */
	UART0Status |= 0x100;		/* Bit 9 as the CTI error */
  }
  else if ( IIRValue == IIR_THRE )	/* THRE, transmit holding register empty */
  {
	/* THRE interrupt */
	LSRValue = LPC_UART0->LSR;		/* Check status in the LSR to see if
									valid data in U0THR or not */
	if ( LSRValue & LSR_THRE )
	{
	  UART0TxEmpty = 1;
	}
	else
	{
	  UART0TxEmpty = 0;
	}
  }
}

/*****************************************************************************
** Function name:		UART1_IRQHandler
**
** Descriptions:		UART1 interrupt handler
**
** parameters:			None
** Returned value:		None
** 
*****************************************************************************/
void UART1_IRQHandler (void) 
{
  uint8_t IIRValue, LSRValue;
  uint8_t Dummy = Dummy;
	
  IIRValue = LPC_UART1->IIR;
    
  IIRValue >>= 1;			/* skip pending bit in IIR */
  IIRValue &= 0x07;			/* check bit 1~3, interrupt identification */
  if ( IIRValue == IIR_RLS )		/* Receive Line Status */
  {
	LSRValue = LPC_UART1->LSR;
	/* Receive Line Status */
	if ( LSRValue & (LSR_OE|LSR_PE|LSR_FE|LSR_RXFE|LSR_BI) )
	{
	  /* There are errors or break interrupt */
	  /* Read LSR will clear the interrupt */
	  UART1Status = LSRValue;
	  Dummy = LPC_UART1->RBR;		/* Dummy read on RX to clear 
								interrupt, then bail out */
	  return;
	}
	if ( LSRValue & LSR_RDR )	/* Receive Data Ready */			
	{
	  /* If no error on RLS, normal ready, save into the data buffer. */
	  /* Note: read RBR will clear the interrupt */
	  UART1Buffer[UART1Count] = LPC_UART1->RBR;
	  UART1Count++;
	  if ( UART1Count == BUFSIZE )
	  {
		UART1Count = 0;		/* buffer overflow */
	  }
	}
  }
  else if ( IIRValue == IIR_RDA )	/* Receive Data Available */
  {
	/* Receive Data Available */
	UART1Buffer[UART1Count] = LPC_UART1->RBR;
	UART1Count++;
	if ( UART1Count == BUFSIZE )
	{
		UART1Count = 0;		/* buffer overflow */
	}
  }
  else if ( IIRValue == IIR_CTI )	/* Character timeout indicator */
  {
	/* Character Time-out indicator */
	UART1Status |= 0x100;		/* Bit 9 as the CTI error */
  }
  else if ( IIRValue == IIR_THRE )	/* THRE, transmit holding register empty */
  {
	/* THRE interrupt */
	LSRValue = LPC_UART1->LSR;		/* Check status in the LSR to see if
								valid data in U0THR or not */
	if ( LSRValue & LSR_THRE )
	{
	  UART1TxEmpty = 1;
	}
	else
	{
	  UART1TxEmpty = 0;
	}
  }

}
/*****************************************************************************
** Function name:		UART2_IRQHandler
**
** Descriptions:		UART2 interrupt handler
**
** parameters:			None
** Returned value:		None
** 
*****************************************************************************/
void UART2_IRQHandler (void) 
{
  uint8_t IIRValue, LSRValue;
  uint8_t Dummy = Dummy;
	
  IIRValue = LPC_UART2->IIR;
    
  IIRValue >>= 1;			/* skip pending bit in IIR */
  IIRValue &= 0x07;			/* check bit 1~3, interrupt identification */
  if ( IIRValue == IIR_RLS )		/* Receive Line Status */
  {
	LSRValue = LPC_UART2->LSR;
	/* Receive Line Status */
	if ( LSRValue & (LSR_OE|LSR_PE|LSR_FE|LSR_RXFE|LSR_BI) )
	{
	  /* There are errors or break interrupt */
	  /* Read LSR will clear the interrupt */
	  UART2Status = LSRValue;
	  Dummy = LPC_UART2->RBR;		/* Dummy read on RX to clear 
								interrupt, then bail out */
	  return;
	}
	if ( LSRValue & LSR_RDR )	/* Receive Data Ready */			
	{
	  /* If no error on RLS, normal ready, save into the data buffer. */
	  /* Note: read RBR will clear the interrupt */
	  UART2Buffer[UART2Count] = LPC_UART2->RBR;
	  UART2Count++;
	  if ( UART2Count == BUFSIZE )
	  {
		UART2Count = 0;		/* buffer overflow */
	  }
	}
  }
  else if ( IIRValue == IIR_RDA )	/* Receive Data Available */
  {
	/* Receive Data Available */
	UART2Buffer[UART2Count] = LPC_UART2->RBR;
	UART2Count++;
	if ( UART2Count == BUFSIZE )
	{
		UART2Count = 0;		/* buffer overflow */
	}
  }
  else if ( IIRValue == IIR_CTI )	/* Character timeout indicator */
  {
	/* Character Time-out indicator */
	UART2Status |= 0x100;		/* Bit 9 as the CTI error */
  }
  else if ( IIRValue == IIR_THRE )	/* THRE, transmit holding register empty */
  {
	/* THRE interrupt */
	LSRValue = LPC_UART2->LSR;		/* Check status in the LSR to see if
								valid data in U0THR or not */
	if ( LSRValue & LSR_THRE )
	{
	  UART2TxEmpty = 1;
	}
	else
	{
	  UART2TxEmpty = 0;
	}
  }

}
/*****************************************************************************
** Function name:		UART3_IRQHandler
**
** Descriptions:		UART3 interrupt handler
**
** parameters:			None
** Returned value:		None
**
*****************************************************************************/
void UART3_IRQHandler (void)
{
  uint8_t IIRValue, LSRValue;
  uint8_t Dummy = Dummy;

  IIRValue = LPC_UART3->IIR;

  IIRValue >>= 1;			/* skip pending bit in IIR */
  IIRValue &= 0x07;			/* check bit 1~3, interrupt identification */
  if ( IIRValue == IIR_RLS )		/* Receive Line Status */
  {
	LSRValue = LPC_UART3->LSR;
	/* Receive Line Status */
	if ( LSRValue & (LSR_OE|LSR_PE|LSR_FE|LSR_RXFE|LSR_BI) )
	{
	  /* There are errors or break interrupt */
	  /* Read LSR will clear the interrupt */
	  UART3Status = LSRValue;
	  Dummy = LPC_UART3->RBR;		/* Dummy read on RX to clear
							interrupt, then bail out */
	  return;
	}
	if ( LSRValue & LSR_RDR )	/* Receive Data Ready */
	{
	  /* If no error on RLS, normal ready, save into the data buffer. */
	  /* Note: read RBR will clear the interrupt */
	  UART3Buffer[UART3Count] = LPC_UART3->RBR;
	  UART3Count++;
	  if ( UART3Count == BUFSIZE )
	  {
		UART3Count = 0;		/* buffer overflow */
	  }
	}
  }
  else if ( IIRValue == IIR_RDA )	/* Receive Data Available */
  {
	/* Receive Data Available */
	UART3Buffer[UART3Count] = LPC_UART3->RBR;
	UART3Count++;
	if ( UART3Count == BUFSIZE )
	{
		UART3Count = 0;		/* buffer overflow */
	}
  }
  else if ( IIRValue == IIR_CTI )	/* Character timeout indicator */
  {
	/* Character Time-out indicator */
	UART3Status |= 0x100;		/* Bit 9 as the CTI error */
  }
  else if ( IIRValue == IIR_THRE )	/* THRE, transmit holding register empty */
  {
	/* THRE interrupt */
	LSRValue = LPC_UART3->LSR;		/* Check status in the LSR to see if
									valid data in U0THR or not */
	if ( LSRValue & LSR_THRE )
	{
	  UART3TxEmpty = 1;
	}
	else
	{
	  UART3TxEmpty = 0;
	}
  }
}


/*****************************************************************************
** Function name:		UARTInit
**
** Descriptions:		Initialize UART port, setup pin select,
**						clock, parity, stop bits, FIFO, etc.
**
** parameters:			PortNum(0 or 1 or 2 or 3) and UART Baudrate
** Returned value:		true or false, return false only if the 
**						interrupt handler can't be installed to the 
**						VIC table
** 
*****************************************************************************/
uint32_t UART_Init( uint8_t PortNum, uint32_t Baudrate )
{
  if ( PortNum == 0 )
  {
    NVIC_DisableIRQ(UART0_IRQn);

	LPC_PINCON->PINSEL0 &= ~0x000000F0;
	LPC_PINCON->PINSEL0 |= 0x00000050;  /* RxD0 is P0.3 and TxD0 is P0.2 */

	UART_Set_Divisors(PortNum, Baudrate);

   	NVIC_EnableIRQ(UART0_IRQn);

    LPC_UART0->IER = IER_RBR | IER_THRE | IER_RLS;	/* Enable UART0 interrupt */
    return (TRUE);
  }
  else if ( PortNum == 1 )
  {
    NVIC_DisableIRQ(UART1_IRQn);

	LPC_PINCON->PINSEL4 &= ~0x0000000F;
	LPC_PINCON->PINSEL4 |= 0x0000000A;	/* Enable RxD1 P2.1, TxD1 P2.0 */

	UART_Set_Divisors(PortNum, Baudrate);

   	NVIC_EnableIRQ(UART1_IRQn);

    LPC_UART1->IER = IER_RBR | IER_THRE | IER_RLS;	/* Enable UART1 interrupt */
    return (TRUE);
  }
  else if ( PortNum == 2 )
  {

	  NVIC_DisableIRQ(UART2_IRQn);

	  //LPC_PINCON->PINSEL4 &= ~0x000F0000;  /*Pinsel4 Bits 16-19*/
	  //LPC_PINCON->PINSEL4 |=  0x000A0000;  /* RxD2 is P2.9 and TxD2 is P2.8, value 10*/
	  LPC_PINCON->PINSEL0 &= ~0x00F00000;  /*Pinsel0 Bits 20-23*/
	  LPC_PINCON->PINSEL0 |=  0x00500000;  /* RxD2 is P0.11 and TxD2 is P0.10, value 01*/

	  LPC_SC->PCONP |= 1<<24; //Enable PCUART2

	  UART_Set_Divisors(PortNum, Baudrate);

	  NVIC_EnableIRQ(UART2_IRQn);

	  LPC_UART2->IER = IER_RBR | IER_THRE | IER_RLS;	/* Enable UART2 interrupt */
	  return (TRUE);
  }
  else if ( PortNum == 3 )
  {

	  NVIC_DisableIRQ(UART3_IRQn);

	  LPC_PINCON->PINSEL0 &= ~0x0000000F;
	  LPC_PINCON->PINSEL0 |=  0x0000000A;  /* RxD3 is P0.1 and TxD3 is P0.0 */
	  LPC_SC->PCONP |= 1<<4 | 1<<25; //Enable PCUART3

	  UART_Set_Divisors(PortNum, Baudrate);

	  NVIC_EnableIRQ(UART3_IRQn);

	  LPC_UART3->IER = IER_RBR | IER_THRE | IER_RLS;	/* Enable UART3 interrupt */
	  return (TRUE);
  }
  return( FALSE );
}

/*****************************************************************************
** Function name:		UARTClose
**
** Descriptions:		Close the UART port, setup pin select,
**						clock, parity, stop bits, FIFO, etc.
**
** parameters:			PortNum(0 or 1 or 2 or 3)
** Returned value:		true or false, return false only if the
**						interrupt handler can't be uninstalled to the
**						VIC table
**
*****************************************************************************/
uint32_t UART_Close( uint8_t PortNum)
{

  if ( PortNum == 0 )
  {
    	LPC_PINCON->PINSEL0 &= ~0x000000F0;
        /* Disable the interrupt in the VIC and UART controllers */
        LPC_UART0->IER = 0;
        NVIC_DisableIRQ(UART0_IRQn);
	    return (TRUE);
  }
  else if ( PortNum == 1 )
  {
	  	LPC_PINCON->PINSEL4 &= ~0x0000000F;
        /* Disable the interrupt in the VIC and UART controllers */
        LPC_UART1->IER = 0;
        NVIC_DisableIRQ(UART1_IRQn);
    	return (TRUE);
  }
  else if ( PortNum == 2 )
  {
	  	LPC_PINCON->PINSEL0 &= ~0x00F00000;
	  	LPC_SC->PCONP &= ~1<<24; //Disable PCUART2
        /* Disable the interrupt in the VIC and UART controllers */
        LPC_UART2->IER = 0;
        NVIC_DisableIRQ(UART2_IRQn);
		return (TRUE);
  }
  else if ( PortNum == 3 )
  {
	  	LPC_PINCON->PINSEL0 &= ~0x0000000F;
	  	LPC_SC->PCONP &= ~(1<<4 | 1<<25); //Disable PCUART3
        /* Disable the interrupt in the VIC and UART controllers */
        LPC_UART3->IER = 0;
        NVIC_DisableIRQ(UART3_IRQn);
		return (TRUE);
  }
  return( FALSE ); 
}

/*****************************************************************************
** Function name:		UART_set_divisors
**
** Descriptions:		Determines best dividers to get a target clock rate
**
**
** parameters:			UARTx	Pointer to selected UART peripheral, should be:
** 	  					- LPC_UART0: UART0 peripheral
** 						- LPC_UART1: UART1 peripheral
** 						- LPC_UART2: UART2 peripheral
** 						- LPC_UART3: UART3 peripheral
** 						Baudrate Desired UART baud rate.
**
** Returned value:		true or false, return false only if
**						best_error aproximation
**
*****************************************************************************/
uint32_t UART_Set_Divisors(uint8_t PortNum, uint32_t Baudrate)
{
	uint32_t errorStatus = FALSE;
	uint32_t pclkdiv;
	uint32_t uClk;
	uint32_t d, m, bestd, bestm, tmp;
	uint64_t best_divisor, divisor;
	uint32_t current_error, best_error;
	uint32_t recalcbaud;

	switch ( PortNum )
	{
	  case 0:
			pclkdiv = (LPC_SC->PCLKSEL0 >> 6) & 0x03;
		break;
	  case 1:
			pclkdiv = (LPC_SC->PCLKSEL0 >> 8) & 0x03;
		break;
	  case 2:
			pclkdiv = (LPC_SC->PCLKSEL1 >> 16) & 0x03;
		break;
	  case 3:
		    pclkdiv = (LPC_SC->PCLKSEL1 >> 18) & 0x03;
		break;

	  default:
		  return errorStatus;
	}

	switch ( pclkdiv )
	{
	  case 0x00:
	  default:
		uClk = SystemCoreClock/4;
		break;
	  case 0x01:
		uClk = SystemCoreClock;
		break;
	  case 0x02:
		uClk = SystemCoreClock/2;
		break;
	  case 0x03:
		uClk = SystemCoreClock/8;
		break;
	}

	// In the Uart IP block, baud rate is calculated using FDR and DLL-DLM registers
	// The formula is :
	// BaudRate= uClk * (mulFracDiv/(mulFracDiv+dividerAddFracDiv) / (16 * (DLL)
	// It involves floating point calculations. That's the reason the formulae are adjusted with
	// Multiply and divide method.
	// The value of mulFracDiv and dividerAddFracDiv should comply to the following expressions:
	// 0 < mulFracDiv <= 15, 0 <= dividerAddFracDiv <= 15
	best_error = 0xFFFFFFFF; // Worst case
	bestd = 0;
	bestm = 0;
	best_divisor = 0;
	for (m = 1 ; m <= 15 ;m++)
	{
		for (d = 0 ; d < m ; d++)
		{
		  divisor = ((uint64_t)uClk<<28)*m/(Baudrate*(m+d));
		  current_error = divisor & 0xFFFFFFFF;
		  tmp = divisor>>32;
		  // Adjust error
		  if(current_error > ((uint32_t)1<<31)){
			current_error = -current_error;
			tmp++;
			}
		  if(tmp<1 || tmp>65536) // Out of range
		  continue;
		  if( current_error < best_error){
			best_error = current_error;
			best_divisor = tmp;
			bestd = d;
			bestm = m;
			if(best_error == 0) break;
			}
		} // end of inner for loop
		if (best_error == 0)
		  break;
	} // end of outer for loop

	if(best_divisor == 0) return FALSE; // can not find best match
	recalcbaud = (uClk>>4) * bestm/(best_divisor * (bestm + bestd));
	// reuse best_error to evaluate baud error
	if(Baudrate>recalcbaud) best_error = Baudrate - recalcbaud;
	else best_error = recalcbaud - Baudrate;
	best_error = best_error * 100 / Baudrate;

	if (best_error < UART_ACCEPTED_BAUDRATE_ERROR)
	{
		if ( PortNum == 0 )
		{
			LPC_UART0->LCR = 0x83;		// 8 bits, no Parity, 1 Stop bit
			LPC_UART0->DLM = UART_LOAD_DLM(best_divisor);
			LPC_UART0->DLL = UART_LOAD_DLL(best_divisor);
			// Then reset DLAB bit
			LPC_UART0->LCR = 0x03;		// DLAB = 0
			LPC_UART0->FDR = (UART_FDR_MULVAL(bestm) | UART_FDR_DIVADDVAL(bestd)) & UART_FDR_BITMASK;
			LPC_UART0->FCR = 0x07;		// Enable and reset TX and RX FIFO.
		}
		else if ( PortNum == 1 )
		{
			LPC_UART1->LCR = 0x83;		// 8 bits, no Parity, 1 Stop bit
			LPC_UART1->DLM = UART_LOAD_DLM(best_divisor);
			LPC_UART1->DLL = UART_LOAD_DLL(best_divisor);
			// Then reset DLAB bit
			LPC_UART1->LCR = 0x03;		// DLAB = 0
			LPC_UART1->FDR = (UART_FDR_MULVAL(bestm) | UART_FDR_DIVADDVAL(bestd)) & UART_FDR_BITMASK;
			LPC_UART1->FCR = 0x07;		// Enable and reset TX and RX FIFO.

		}
		else if ( PortNum == 2 )
		{
			LPC_UART2->LCR = 0x83;		// 8 bits, no Parity, 1 Stop bit
			LPC_UART2->DLM = UART_LOAD_DLM(best_divisor);
			LPC_UART2->DLL = UART_LOAD_DLL(best_divisor);
			// Then reset DLAB bit
			LPC_UART2->LCR = 0x03;		// DLAB = 0
			LPC_UART2->FDR = (UART_FDR_MULVAL(bestm) | UART_FDR_DIVADDVAL(bestd)) & UART_FDR_BITMASK;

			LPC_UART2->FCR = 0x07;		// Enable and reset TX and RX FIFO.
		}
		else if ( PortNum == 3 )
		{
			LPC_UART3->LCR = 0x83;		// 8 bits, no Parity, 1 Stop bit
			LPC_UART3->DLM = UART_LOAD_DLM(best_divisor);
			LPC_UART3->DLL = UART_LOAD_DLL(best_divisor);
			// Then reset DLAB bit
			LPC_UART3->LCR = 0x03;		// DLAB = 0
			LPC_UART3->FDR = (UART_FDR_MULVAL(bestm) | UART_FDR_DIVADDVAL(bestd)) & UART_FDR_BITMASK;
			LPC_UART3->FCR = 0x07;		// Enable and reset TX and RX FIFO.
		}
		errorStatus = TRUE;
	}
	return errorStatus;
}

/*****************************************************************************
** Function name:		UARTWrite
**
** Descriptions:		Send a byte of data to the UART
**
** parameters:			portNum(0,1,2 or 3), data byte
** Returned value:		None
**
*****************************************************************************/
void UART_Write( uint8_t PortNum, uint8_t send)
{
  if ( PortNum == 0 )
  {
	  /* THRE status, contain valid data */
	  while ( !(UART0TxEmpty & 0x01) );
	  LPC_UART0->THR = send;
	  UART0TxEmpty = 0;	/* not empty in the THR until it shifts out */
  }
  else if (PortNum == 1)
  {
	  /* THRE status, contain valid data */
	  while ( !(UART1TxEmpty & 0x01) );
	  LPC_UART1->THR = send;
	  UART1TxEmpty = 0;	/* not empty in the THR until it shifts out */
  }
  else if ( PortNum == 2 )
  {
	  /* THRE status, contain valid data */
	  while ( !(UART2TxEmpty & 0x01) );
	  LPC_UART2->THR = send;
	  UART2TxEmpty = 0;	/* not empty in the THR until it shifts out */
  }
  else if ( PortNum == 3 )
  {
	  /* THRE status, contain valid data */
	  while ( !(UART3TxEmpty & 0x01) );
	  LPC_UART3->THR = send;
	  UART3TxEmpty = 0;	/* not empty in the THR until it shifts out */
  }
  return;
}

/*****************************************************************************
** Function name:		UARTSend
**
** Descriptions:		Send a zero terminated string to the UART
**
** parameters:			portNum(0,1,2 or 3), buffer pointer
** Returned value:		None
**
*****************************************************************************/
void UART_Send( uint8_t PortNum, char *BufferPtr)
{
	/*
	while( *BufferPtr )
	{
		UART_Write( PortNum, *BufferPtr++);
	}*/

    while ( *BufferPtr != '\0' ) {
    	UART_Write( PortNum, *BufferPtr++);
    }
}

/*****************************************************************************
** Function name:		Serial_flush
**
** Descriptions:		Clears the RX FIFO ringbuffer
**
** parameters:			portNum(0,1,2 or 3)
** Returned value:		none
**
*****************************************************************************/
void UART_Flush (uint8_t PortNum)
{
	  if ( PortNum == 0 )
	  {
		  UART0Count = 0;
		  //UART0Buffer[0] = '\0';
		  memset((uint8_t *)UART0Buffer, 0x00, BUFSIZE);
	  }
	  if ( PortNum == 1 )
	  {
		  UART1Count = 0;
		  //UART1Buffer[0] = '\0';
		  memset((uint8_t *)UART1Buffer, 0x00, BUFSIZE);
	  }
	  if ( PortNum == 2 )
	  {
		  UART2Count = 0;
		  //UART2Buffer[0] = '\0';
		  memset((uint8_t *)UART2Buffer, 0x00, BUFSIZE);
	  }
	  if ( PortNum == 3 )
	  {
		  UART3Count = 0;
		  //UART3Buffer[0] = '\0';
		  memset((uint8_t *)UART3Buffer, 0x00, BUFSIZE);
	  }
}


/*****************************************************************************
** Function name:		UART_Read
**
** Descriptions:		Read the UART port waiting time parameter
**
** parameters:			UART Port, Pointer to Buffer and Time to wait in milliseconds
** Returned value:		Number of characters read
**
*****************************************************************************/


uint32_t UART_Read(uint8_t PortNum, char *Text, char *response, uint32_t Time)
{
	Text[0] = '\0';
	while (Time > 0)
	{
		if ((PortNum == 0) && (UART0Count != 0))
		{
			LPC_UART0->IER = IER_THRE | IER_RLS;				/* Disable RBR */
			if (strlen((char *)UART0Buffer) + strlen(Text) < GetSIZE)
			{
				strncat(Text, (char*)UART0Buffer, UART0Count);
			}
			else Text = (char *)Overflow;
			UART0Count = 0;
			LPC_UART0->IER = IER_THRE | IER_RLS | IER_RBR;		/* Re-enable RBR */
			if(strstr(Text, response) != NULL) Time = 10;

		}
		else if ((PortNum == 1) && (UART1Count != 0))
		{
			LPC_UART1->IER = IER_THRE | IER_RLS;				/* Disable RBR */
			if (strlen((char *)UART1Buffer) + strlen(Text) < GetSIZE)
			{
				strncat(Text, (char*)UART1Buffer, UART1Count);
			}
			else Text = (char *)Overflow;
			UART1Count = 0;
			LPC_UART1->IER = IER_THRE | IER_RLS | IER_RBR;		/* Re-enable RBR */
			if(strstr(Text, response) != NULL) Time = 10;

		}
		else if ((PortNum == 2) && (UART2Count != 0))
		{
			LPC_UART2->IER = IER_THRE | IER_RLS;				/* Disable RBR */
			if (strlen((char *)UART2Buffer) + strlen(Text) < GetSIZE)
			{
				strncat(Text, (char*)UART2Buffer, UART2Count);
			}
			else Text = (char *)Overflow;
			UART2Count = 0;
			LPC_UART2->IER = IER_THRE | IER_RLS | IER_RBR;		/* Re-enable RBR */
			if(strstr(Text, response) != NULL) Time = 10;

		}
		else if ((PortNum == 3) && (UART3Count != 0))
		{
			LPC_UART3->IER = IER_THRE | IER_RLS;				/* Disable RBR */
			if (strlen((char *)UART3Buffer) + strlen(Text) < GetSIZE)
			{
				strncat(Text, (char*)UART3Buffer, UART3Count);
			}
			else Text = (char *)Overflow;
			UART3Count = 0;
			LPC_UART3->IER = IER_THRE | IER_RLS | IER_RBR;		/* Re-enable RBR */
			if(strstr(Text, response) != NULL) Time = 10;

		}
		Delay_Ms(1);
		Time--;
	}
	UART_Flush(PortNum);

	return strlen(Text);
}




/******************************************************************************
**                            End Of File
******************************************************************************/
