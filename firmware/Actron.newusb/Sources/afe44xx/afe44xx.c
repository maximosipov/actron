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

#include "derivative.h"
#include "hidef.h"
#include <stdio.h>
#include "afe44xx.h"

extern void delay(int t);

volatile uint32_t tmp;

volatile uint32_t afe_icount = 0;

volatile int32_t red = 0;
volatile int32_t red_amb = 0;
volatile int32_t red_diff = 0;
volatile int32_t ir = 0;
volatile int32_t ir_amb = 0;
volatile int32_t ir_diff = 0;


void afe44xx_init(void)
{
	int i;

	/* Configure sampling cycle (1ms cycle with 10us dead time) */
#define CYCLE	4000
#define FRAME	(CYCLE/4)
#define DTIME	(40)
	afe44xx_write(CONTROL0, 	0x0);
	afe44xx_write(CONTROL1, 	0x0);
	afe44xx_write(CONTROL2, 	0x200);			/* Disable crystal */
	afe44xx_write(PRPCOUNT,		CYCLE);

	afe44xx_write(LED2STC, 		3*FRAME);
	afe44xx_write(LED2ENDC, 	4*FRAME-1);
	afe44xx_write(LED2LEDSTC, 	3*FRAME+DTIME);
	afe44xx_write(LED2LEDENDC, 	4*FRAME-DTIME);
	afe44xx_write(ALED2STC, 	0*FRAME+DTIME);
	afe44xx_write(ALED2ENDC, 	1*FRAME-DTIME);
	afe44xx_write(LED2CONVST, 	0*FRAME);
	afe44xx_write(LED2CONVEND, 	1*FRAME);
	afe44xx_write(ALED2CONVST, 	1*FRAME);
	afe44xx_write(ALED2CONVEND, 2*FRAME);

	afe44xx_write(LED1STC, 		1*FRAME);
	afe44xx_write(LED1ENDC, 	2*FRAME-1);
	afe44xx_write(LED1LEDSTC, 	1*FRAME+DTIME);
	afe44xx_write(LED1LEDENDC, 	2*FRAME-DTIME);
	afe44xx_write(ALED1STC, 	2*FRAME+DTIME);
	afe44xx_write(ALED1ENDC, 	3*FRAME-DTIME);
	afe44xx_write(LED1CONVST, 	2*FRAME);
	afe44xx_write(LED1CONVEND, 	3*FRAME);
	afe44xx_write(ALED1CONVST, 	3*FRAME);
	afe44xx_write(ALED1CONVEND, 4*FRAME);

	afe44xx_write(ADCRSTCNT0, 	0*FRAME);
	afe44xx_write(ADCRSTENDCT0, 0*FRAME);
	afe44xx_write(ADCRSTCNT1, 	1*FRAME);
	afe44xx_write(ADCRSTENDCT1, 1*FRAME);
	afe44xx_write(ADCRSTCNT2, 	2*FRAME);
	afe44xx_write(ADCRSTENDCT2, 2*FRAME);
	afe44xx_write(ADCRSTCNT3, 	3*FRAME);
	afe44xx_write(ADCRSTENDCT3, 3*FRAME);

//	afe44xx_write(CONTROL2, 	0x8200);		/* ADC bypass + Disable crystal */

	afe44xx_write(CONTROL1, 	0x103);			/* Timers ON, average 3 samples */
	afe44xx_write(CONTROL0, 	0x1);			/* Switch to READ mode */
	delay(1000);
}

void afe44xx_write(uint8_t a, uint32_t d)
{
	uint8_t d0 = d & 0xff;
	uint8_t d1 = (d >> 8) & 0xff;
	uint8_t d2 = (d >> 16) & 0xff;
	SPI0_SR = 0x90000000;	/* Clear TCF and EOQF */
	while (!(SPI0_SR & SPI_SR_TFFF_MASK));
	SPI0_PUSHR = SPI_PUSHR_CONT_MASK | SPI_PUSHR_PCS(0x1) | SPI_PUSHR_TXDATA(a);
	while (!(SPI0_SR & SPI_SR_TFFF_MASK));
	SPI0_PUSHR = SPI_PUSHR_CONT_MASK | SPI_PUSHR_PCS(0x1) | SPI_PUSHR_TXDATA(d2);
	while (!(SPI0_SR & SPI_SR_TFFF_MASK));
	SPI0_PUSHR = SPI_PUSHR_CONT_MASK | SPI_PUSHR_PCS(0x1) | SPI_PUSHR_TXDATA(d1);
	while (!(SPI0_SR & SPI_SR_TFFF_MASK));
	SPI0_PUSHR = SPI_PUSHR_EOQ_MASK | SPI_PUSHR_PCS(0x1) | SPI_PUSHR_TXDATA(d0);
	while (!(SPI0_SR & SPI_SR_TCF_MASK));
	SPI0_SR = 0x90000000;	/* Clear TCF and EOQF */
	delay(20);
}

uint32_t afe44xx_read(uint8_t a)
{
	uint32_t d = 0;
	/* read */
	SPI0_SR = 0x90000000;	/* Clear TCF and EOQF */
	while (!(SPI0_SR & SPI_SR_TFFF_MASK));
	SPI0_PUSHR = SPI_PUSHR_CONT_MASK | SPI_PUSHR_PCS(0x1) | SPI_PUSHR_TXDATA(a);
	while (!(SPI0_SR & SPI_SR_TFFF_MASK));
	SPI0_PUSHR = SPI_PUSHR_CONT_MASK | SPI_PUSHR_PCS(0x1) | SPI_PUSHR_TXDATA(0);
	while (!(SPI0_SR & SPI_SR_TFFF_MASK));
	d = SPI0_POPR;
	SPI0_PUSHR = SPI_PUSHR_CONT_MASK | SPI_PUSHR_PCS(0x1) | SPI_PUSHR_TXDATA(0);
	while (!(SPI0_SR & SPI_SR_TFFF_MASK));
	d = (d << 8) | SPI0_POPR;
	SPI0_PUSHR = SPI_PUSHR_EOQ_MASK | SPI_PUSHR_PCS(0x1) | SPI_PUSHR_TXDATA(0);
	while (!(SPI0_SR & SPI_SR_TCF_MASK));
	d = (d << 8) | SPI0_POPR;
	SPI0_SR = 0x90000000;	/* Clear TCF and EOQF */
	delay(20);

	return d;
}

/* AFE4490 conversion complete ISR */
void afe44xx_isr(void) {
	DisableInterrupts;
	afe_icount++;

	/* Read data  */
	red = afe44xx_read(LED2VAL);
	red_amb = afe44xx_read(ALED2VAL);
	red_diff = afe44xx_read(LED2ABSVAL);
	ir = afe44xx_read(LED1VAL);
	ir_amb = afe44xx_read(ALED1VAL);
	ir_diff = afe44xx_read(LED1ABSVAL);

	/* Clear any pending */
	PORTC_PCR7 |= PORT_PCR_ISF_MASK;
	NVICICPR1 |= (1 << 10);
	EnableInterrupts;
}
