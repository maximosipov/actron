/*
 * Copyright (c) 2013, Maxim Osipov <maxim.osipov@gmail.com>
 */

#include "uart.h"


extern uint8_t bt_rx_buf[128];
extern uint8_t bt_tx_buf[128];

extern uint16_t bt_rx_len;
extern uint16_t bt_tx_len;

static uint16_t tx_pos = 0;

void uart0_tx_start(void)
{
    UART0_C2 |= UART_C2_TIE_MASK;                                                
}

void uart0_isr(void)
{
	uint8_t tmp;
	
	/* RX interrupt */
	if (UART0_S1 & UART_S1_RDRF_MASK) {
		if (bt_rx_len >= sizeof(bt_rx_buf)) {
			/* overflow */
			tmp = UART0_D;
			return;
		}
		bt_rx_buf[bt_rx_len++] = UART0_D;
	}
	
	/* TX interrupt */
	if (UART0_S1 & UART_S1_TDRE_MASK) {
		/* End of buffer */
		if (tx_pos == bt_tx_len) {
			bt_tx_len = 0;
			tx_pos = 0;
		}
		/* Nothing to send */
		if (bt_tx_len == 0) {
			/* disable TX interrupts */
			UART0_C2 &= ~UART_C2_TIE_MASK;                                                
			return;
		}
		UART0_D = bt_tx_buf[tx_pos++];
	}
}

