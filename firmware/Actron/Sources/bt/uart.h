/*
 * Copyright (c) 2013, Maxim Osipov <maxim.osipov@gmail.com>
 */

#ifndef UART_H
#define UART_H

#include <stdio.h>
#include "derivative.h"


void uart0_tx_start(void);
void uart0_isr(void);

#endif
