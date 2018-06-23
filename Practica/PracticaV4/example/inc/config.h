/*
 * config.h
 *
 *  Created on: 25/08/2013
 *      Author: Xavier Vilajosana
 */

#ifndef CONFIG_H_
#define CONFIG_H_

/*********************************************************************
**
** Comentar i descomentar la linia que toqui per versió definitiva
**
*********************************************************************/
#define MODE_DEBUG
//#define MODE_NORMAL

/*********************************************************************
**
** Definicions Base per al codi de : uart.c, printf.c i wifly.c
**
*********************************************************************/
#define RedSIZE		64
#define LowSIZE		128
#define MinSIZE		256
#define TipSIZE		512
#define MaxSIZE		1024
#define SuperSIZE	2048

#define UART0		0
#define UART1		1
#define UART2		2
#define UART3		3

#define BR_96	 	9600
#define BR_192	 	19200
#define BR_384	 	38400
#define BR_576	 	57600
#define BR_1152	 	115200

#define Brute		0  // Used for Active Delay
#define Task		1  // User for Passive Delay

#define LockNONE	0  // Not used Block
#define LockTASK	1  // Used for Block with vTaskSuspend
#define LockMUTEX	2  // Used for Block with Semaphore (Normal -> Printf, Recursive -> Wifly)

#define HTTP		"80"
#define ALTER		"8080"
#define TELNET		"23"
#define OTHER		"20"

#define WEB1		"HTTP/1.0"
#define WEB2		"HTTP/1.1"

/*********************************************************************
**
** Parametres modificables segons les necessitats
**
*********************************************************************/
#define UART_DBG		(UART3)
#define UART_WF			(UART0)

#define BR_DBG			(BR_1152)
#define BR_WF			(BR_96)

#define DELAY			(Task)
#define LOCK			(LockMUTEX)

#define GetSIZE			(MinSIZE)
#define SendSIZE		(LowSIZE)
#define BUFSIZE 		(RedSIZE)  // Define buffer base for UART

#define WEB				(WEB2)
#define NetPORT			(HTTP)

#endif /* CONFIG_H_ */
