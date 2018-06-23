/*****************************************************************************
 *   type.h:  Type definition Header file for NXP LPC17xx Family 
 *   Microprocessors
 *
 *   Copyright(C) 2009, NXP Semiconductor
 *   All rights reserved.
 *
 *   History
 *   2009.05.25  ver 1.00    Prelimnary version, first Release
 *
******************************************************************************/
#include <stdint.h>

#ifndef __TYPE_H__
#define __TYPE_H__

#ifndef NULL
#define NULL    ((void *)0)
#endif

#ifndef FALSE
#define FALSE   (0)
#endif

#ifndef TRUE
#define TRUE    (1)
#endif


typedef enum {RESET_1 = 0, SET_1 = !RESET_1} FlagStatus_1, ITStatus_1;
typedef enum {DISABLE_1 = 0, ENABLE_1 = !DISABLE_1} FunctionalState_1;

#endif  /* __TYPE_H__ */
