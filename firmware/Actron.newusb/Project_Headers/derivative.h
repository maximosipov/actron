/*
 * Note: This file is recreated by the project wizard whenever the MCU is
 *       changed and should not be edited by hand
 */

#ifndef DERIVATIVE_H_
#define DERIVATIVE_H_

/* Include the derivative-specific header file */
#include <MK20D5.h>

#define __MK_xxx_H__
#define LITTLE_ENDIAN

/* USB driver specific definitions */
#define FLAG_SET(BitNumber, Register)        (Register |=(1<<BitNumber))
#define FLAG_CLR(BitNumber, Register)        (Register &=~(1<<BitNumber))
#define FLAG_CHK(BitNumber, Register)        (Register & (1<<BitNumber))

#define _USB     0

typedef uint8_t uint8;
typedef uint16_t uint16;
typedef uint32_t uint32;

#define TRUE	1
#define FALSE	0

#define DEBUG { GPIOB_PTOR |= (1<<16); }

#endif
