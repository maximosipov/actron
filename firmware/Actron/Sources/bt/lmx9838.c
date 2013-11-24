/*
 * Copyright (c) 2013, Maxim Osipov <maxim.osipov@gmail.com>
 */

#include "derivative.h"
#include "hidef.h"
#include <stdio.h>
#include "lmx9838.h"

uint8_t bt_rx_buf[128];
uint8_t bt_tx_buf[128];

uint16_t bt_rx_len = 0;
uint16_t bt_tx_len = 0;

static int connected = 0;

/* Initialize BT controller */
void lmx9838_init(void)
{
	int init_done = 0;
	char n3 = bt_rx_buf[3];
	char n4 = bt_rx_buf[4];
	uint16_t init_len = (uint16_t)n3<<8 & n4;
	while(!init_done) {
		if (bt_rx_buf[0] == 0x02 && bt_rx_buf[1] == 0x69 && bt_rx_buf[2] == 0x25 && init_len > 0) {
			if (bt_rx_buf[6 + init_len] == 0x03) {
				memset(bt_rx_buf, 0, sizeof(bt_rx_buf));
				bt_rx_len = 0;
				init_done = 1;
			}
		}
		n3 = bt_rx_buf[3];
		n4 = bt_rx_buf[4];
		init_len = (uint16_t)n4<<8 | n3;
		Watchdog_Reset();
	}
}


/* Handle application logic */
void lmx9838_task(void)
{
	if(!connected) {
		/* Wait for link ready */
		if (bt_rx_buf[0] == 0x02 && bt_rx_buf[1] == 0x69 && bt_rx_buf[2] == 0x0C && bt_rx_buf[13] == 0x03) {
			memset(bt_rx_buf, 0, sizeof(bt_rx_buf));
			bt_rx_len = 0;
			connected = 1;
		}
	} else {
		/* Do our dirty business */
	}
}


/* Connection check */
int lmx9838_connected(void)
{
	return connected;
}
