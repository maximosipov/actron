/*
 * Copyright (c) 2013, Maxim Osipov <maxim.osipov@gmail.com>
 */

#include "derivative.h"
#include "hidef.h"
#include <stdio.h>
#include "i2c.h"
#include "sht21.h"
#include "mma7660.h"

/* Sensor measurements, updated by PIT0 ISR */
extern volatile sht21_data_t sht21_data;
extern volatile mma7660_data_t mma7660_data;

volatile int pit0_step = 0;

void i2c_tx(uint8_t addr, uint8_t len, uint8_t *buf)
{
	int i = 0;

	/* Enable module, set write mode and generate start signal */
	I2C0_C1 = I2C_C1_IICEN_MASK | I2C_C1_TX_MASK;
	delay(100);
	I2C0_C1 |= I2C_C1_MST_MASK;		/* Start condition */

	if (~(I2C0_S & I2C_S_RXAK_MASK)) {
		/* Write out address of slave (TX) */
		I2C0_D = (addr & 0x7f) << 1;
		delay(300);
		while(i < len) {
			/* Write next byte */
			I2C0_D = buf[i];
			delay(300);
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
	delay(100);
	I2C0_C1 &= ~I2C_C1_IICEN_MASK;	/* Disable module */
}

/* WARNING: Last character is not read!!! */
void i2c_rx(uint8_t addr, uint8_t len, uint8_t *buf)
{
	int i = 0;
	int tmp;

	/* Enable module, set write mode and generate start signal */
	I2C0_C1 = I2C_C1_IICEN_MASK | I2C_C1_TX_MASK;
	delay(100);
	I2C0_C1 |= I2C_C1_MST_MASK;		/* Start condition */

	if (~(I2C0_S & I2C_S_RXAK_MASK)) {
		/* Write out address of slave (RX) */
		I2C0_D = (addr & 0x7f) << 1 | 0x1;
		delay(300);
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
			delay(300);
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
	delay(100);
	buf[i] = I2C0_D;
	I2C0_C1 &= ~I2C_C1_IICEN_MASK;	/* Disable module */
}


void i2c_isr (void)
{
}

/* 100ms ISR to schedule I2C transfers */
void pit0_isr(void)
{
	DisableInterrupts;

	/* Data read pipeline */
	switch (pit0_step) {
	case 0:
		sht21_temp_req();
		pit0_step = 1;
		break;
	case 1:
		sht21_data.temp = sht21_temp_resp();
		pit0_step = 2;
		break;
	case 2:
		sht21_hum_req();
		pit0_step = 3;
		break;
	case 3:
		sht21_data.hum = sht21_hum_resp();
		pit0_step = 4;
		sht21_data.count += 1;
		break;
	case 4:
		mma7660_acc_req();
		pit0_step = 5;
		break;
	case 5:
		mma7660_acc_resp();
		pit0_step = 0;
		mma7660_data.count += 1;
		break;
	default:
		pit0_step = 0;
		break;
	}

	/* Clear any pending */
    PIT_TFLG0 |= PIT_TFLG_TIF_MASK;
	EnableInterrupts;
}
