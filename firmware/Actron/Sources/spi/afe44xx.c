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

#include <stdio.h>
#include "afe44xx.h"
#include "derivative.h"

extern void delay(int t);
void afe44xx_write(uint8_t a, uint32_t d);
uint32_t afe44xx_read(uint8_t a);

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

#define TIAGAIN			0x20
#define TIA_AMB_GAIN	0x21
#define LEDCNTRL		0x22
#define CONTROL2		0x23





#define ALARM			0x29
#define LED2VAL			0x2a
#define ALED2VAL		0x2b
#define LED1VAL			0x2c
#define ALED1VAL		0x2d
#define LED2ABSVAL		0x2e
#define LED1ABSVAL		0x2f
#define DIAG			0x30

volatile uint32_t tmp;
volatile uint32_t red;
volatile uint32_t red_a;
volatile uint32_t ir;
volatile uint32_t ir_a;

void afe44xx_init(void)
{
	int i;
	/* Enable modules */
	SIM_SCGC5 |= SIM_SCGC5_PORTC_MASK;
	SIM_SCGC5 |= SIM_SCGC5_PORTD_MASK;
	SIM_SCGC6 |= SIM_SCGC6_SPI0_MASK;

	/* Configure SPI0, 8MHz */
	PORTD_PCR0 = (PORTD_PCR0 & ~PORT_PCR_MUX_MASK) | PORT_PCR_MUX(0x02);
	PORTD_PCR1 = (PORTD_PCR1 & ~PORT_PCR_MUX_MASK) | PORT_PCR_MUX(0x02);
	PORTD_PCR2 = (PORTD_PCR2 & ~PORT_PCR_MUX_MASK) | PORT_PCR_MUX(0x02);
	PORTD_PCR3 = (PORTD_PCR3 & ~PORT_PCR_MUX_MASK) | PORT_PCR_MUX(0x02);
	SPI0_MCR = SPI_MCR_MSTR_MASK | SPI_MCR_PCSIS(0x1f);
	SPI0_CTAR0 = SPI_CTAR_FMSZ(0x7) | SPI_CTAR_PBR(0x1) | SPI_CTAR_BR(0x0)
		| SPI_CTAR_CSSCK(0x5) | SPI_CTAR_ASC(0x5) | SPI_CTAR_DT(0x7);
	SPI0_RSER = 0x0; /* disable all interrupts */

	/* Configure pins for digital signals and SPI */
	/* CLGOUT (I) */
	PORTC_PCR9 = (PORTC_PCR9 & ~PORT_PCR_MUX_MASK) | PORT_PCR_MUX(0x01);
	GPIOC_PDDR &= ~(GPIOC_PDDR | (1<<9));
	/* _RESET (O) */
	PORTC_PCR10 = (PORTC_PCR10 & ~PORT_PCR_MUX_MASK) | PORT_PCR_MUX(0x01);
	GPIOC_PDDR |= (1<<10);
	/* ADC_RDY (I, interrupt) */
	PORTC_PCR11 = PORT_PCR_MUX(0x01) | PORT_PCR_IRQC(0x9);
	GPIOC_PDDR &= ~(GPIOC_PDDR | (1<<11));
#if 0
	NVICICER2 |= (1 << 25);				/* Clear any pending */
	NVICISER2 |= (1 << 25);				/* Enable interrupts */
#endif
	/* PD_ALM (I) */
	PORTD_PCR4 = (PORTD_PCR4 & ~PORT_PCR_MUX_MASK) | PORT_PCR_MUX(0x01);
	GPIOD_PDDR &= ~(GPIOD_PDDR | (1<<4));
	/* LED_ALM (I) */
	PORTD_PCR5 = (PORTD_PCR5 & ~PORT_PCR_MUX_MASK) | PORT_PCR_MUX(0x01);
	GPIOD_PDDR &= ~(GPIOD_PDDR | (1<<5));
	/* DIAG_END (I) */
	PORTD_PCR6 = (PORTD_PCR6 & ~PORT_PCR_MUX_MASK) | PORT_PCR_MUX(0x01);
	GPIOD_PDDR &= ~(GPIOD_PDDR | (1<<6));
	/* _AFE_PDN (O) */
	PORTD_PCR7 = (PORTD_PCR7 & ~PORT_PCR_MUX_MASK) | PORT_PCR_MUX(0x01);
	GPIOD_PDDR |= (1<<7);

	/* 8MHz clock is is on pin 39 (PTA4), FTM0_CH1 (ALT3) */
	SIM_SCGC5 |= SIM_SCGC5_PORTA_MASK;
	SIM_SCGC6 |= SIM_SCGC6_FTM0_MASK;
	/* configure FTM clock and mode (up-counting, EPWM) */
	FTM0_SC = (0x1 << 3) | (0x0);		/* Up-counting, 48MHz */
	FTM0_MODE = (0x1 << 2) | (0x1);		/* All access enabled */
	FTM0_CONF = (0x3 << 6);				/* Timer active in BDM mode */
	FTM0_C1SC = (0x1 << 5) | (0x1 << 2);/* EPWM */
	FTM0_CNTIN = 0;						/* Count from 0 */
	FTM0_CNT = 0;						/* Load counter */
	FTM0_MOD = 5;						/* Counter to get to 8MHz */
	FTM0_C1V = 2;						/* 50% duty cycle */
	FTM0_MODE = 0;						/* All access disabled */
	PORTA_PCR4 = (PORTA_PCR4 & ~PORT_PCR_MUX_MASK) | PORT_PCR_MUX(0x03);

	/* Take chip out of reset */
	GPIOD_PDOR |= (1<<7);
	GPIOC_PDOR |= (1<<10);

	delay(100000);

	/* Configure sampling cycle (1ms cycle with 10us dead time) */
	afe44xx_write(LED2STC,		0x55);
	tmp = afe44xx_read(LED2STC);
#define CYCLE	4000
#define FRAME	(CYCLE/4)
#define DTIME	(40)
	afe44xx_write(CONTROL0, 	0x0);
	afe44xx_write(CONTROL1, 	0x0);
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

	afe44xx_write(CONTROL1, 	0x100);
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
}

uint32_t afe44xx_read(uint8_t a)
{
	uint32_t d = 0;
	/* enable read */
	afe44xx_write(CONTROL0, 0x1);
	/* read */
	afe44xx_write(a, 0x0);
	d = SPI0_POPR;
	return d;
}

void afe44xx_isr(void) {
	/* ADC conversion is ready, read data */
	red = afe44xx_read(LED2VAL);
	red_a = afe44xx_read(ALED2VAL);
	ir = afe44xx_read(LED1VAL);
	ir_a = afe44xx_read(ALED1VAL);
}
