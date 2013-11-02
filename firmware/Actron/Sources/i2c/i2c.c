/*
 * Copyright (c) 2011, Hedde Bosman <heddebosman@incas3.eu>
 *
 * I2C communication device drivers for mc1322x
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
 *
 */

#include <stdio.h>
#include "i2c.h"


void i2c_tx(uint8_t addr, uint8_t len, uint8_t *buf)
{
	int i = 0;

	/* Enable module, set write mode and generate start signal */
	I2C0_C1 = I2C_C1_IICEN_MASK | I2C_C1_MST_MASK | I2C_C1_TX_MASK ;

	if (~(I2C0_S & I2C_S_RXAK_MASK)) {
		/* Write out address of slave (TX) */
		I2C0_D = (addr & 0x7f) << 1;
		delay(1000);
		while(i < len) {
			/* Write next byte */
			I2C0_D = buf[i];
			delay(1000);
			/* Check arbitration */
			if (I2C0_S & I2C_S_ARBL_MASK) {
				I2C0_S &= ~I2C_S_ARBL_MASK;
				printf("*** ERROR I2C: Arbitration lost\n");
				break;
			}
			i++;
		}
	} else {
		printf("*** ERROR I2C: No ack received\n");
	}

	I2C0_C1 &= ~I2C_C1_MST_MASK;	/* Stop condition */
	I2C0_C1 &= ~I2C_C1_IICEN_MASK;	/* Disable module */
	delay(1000);
}


void i2c_rx(uint8_t addr, uint8_t len, uint8_t *buf)
{
	int i = 0;
	int tmp;

	/* Enable module, set write mode and generate start signal */
	I2C0_C1 = I2C_C1_IICEN_MASK | I2C_C1_MST_MASK | I2C_C1_TX_MASK;

	if (~(I2C0_S & I2C_S_RXAK_MASK)) {
		/* Write out address of slave (RX) */
		I2C0_D = (addr & 0x7f) << 1 | 0x1;
		delay(1000);
		tmp = I2C0_D;
		/* Change to RX mode */
		I2C0_C1 = I2C_C1_IICEN_MASK | I2C_C1_MST_MASK;
		while(i < len) {
			/* Receiving next-to-last byte, thus turn off auto-ack for stop condition */
			if (i == len-1) {
				I2C0_C1 |= I2C_C1_TXAK_MASK;
			}
			/* Read new byte */
			buf[i] = I2C0_D;
			delay(1000);
			/* Check arbitration */
			if ((I2C0_S & I2C_S_ARBL_MASK) && (i != len-1)) {
				I2C0_S &= ~I2C_S_ARBL_MASK;
				printf("*** ERROR I2C: Arbitration lost\n");
				break;
			}
			i++;
		}
	} else {
		printf("*** ERROR I2C: No ack received\n");
	}

	I2C0_C1 &= ~I2C_C1_MST_MASK;	/* Stop condition */
	I2C0_C1 &= ~I2C_C1_IICEN_MASK;	/* Disable module */
	delay(1000);
}


void i2c_isr (void)
{
}
