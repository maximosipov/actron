/*
 * Copyright (c) 2013, Maxim Osipov <maxim.osipov@gmail.com>
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#ifndef AFE44XX_H_
#define AFE44XX_H_

#include <stdio.h>
#include <types.h>

#define CONTROL0		0x00
#define LED2STC			0x01
#define LED2ENDC		0x02
#define LED2LEDSTC		0x03
#define LED2LEDENDC		0x04
#define ALED2STC		0x05
#define ALED2ENDC		0x06
#define LED1STC			0x07
#define LED1ENDC		0x08
#define LED1LEDSTC		0x09
#define LED1LEDENDC		0x0a
#define ALED1STC		0x0b
#define ALED1ENDC		0x0c
#define LED2CONVST		0x0d
#define LED2CONVEND		0x0e
#define ALED2CONVST		0x0f
#define ALED2CONVEND	0x10
#define LED1CONVST		0x11
#define LED1CONVEND		0x12
#define ALED1CONVST		0x13
#define ALED1CONVEND	0x14
#define ADCRSTCNT0		0x15
#define ADCRSTENDCT0	0x16
#define ADCRSTCNT1		0x17
#define ADCRSTENDCT1	0x18
#define ADCRSTCNT2		0x19
#define ADCRSTENDCT2	0x1a
#define ADCRSTCNT3		0x1b
#define ADCRSTENDCT3	0x1c
#define PRPCOUNT		0x1d
#define CONTROL1		0x1e
#define SPARE1			0x1f
#define TIAGAIN			0x20
#define TIA_AMB_GAIN	0x21
#define LEDCNTRL		0x22
#define CONTROL2		0x23
#define SPARE2			0x24
#define SPARE3			0x25
#define SPARE4			0x26
#define SPARE4			0x26
#define RESERVED1		0x27
#define RESERVED2		0x28
#define ALARM			0x29
#define LED2VAL			0x2a
#define ALED2VAL		0x2b
#define LED1VAL			0x2c
#define ALED1VAL		0x2d
#define LED2ABSVAL		0x2e
#define LED1ABSVAL		0x2f
#define DIAG			0x30


extern void afe44xx_init(void);
extern void afe44xx_write(uint8_t a, uint32_t d);
extern uint32_t afe44xx_read(uint8_t a);


#endif /* AFE44XX_H_ */
