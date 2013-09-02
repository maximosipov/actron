/*
 * Copyright (C) 2011 by Matthias Ringwald
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holders nor the names of
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY MATTHIAS RINGWALD AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL MATTHIAS
 * RINGWALD OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
 * THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 */

/*
 *  hal_tick.c
 *
 *  Implementation for MSP430 Experimenter board using 250 ms ticks provided by Timer A1
 *
 */

#include <stdlib.h>
#include <btstack/hal_tick.h>
#include "derivative.h" /* include peripheral declarations */

static void dummy_handler(void) {};

static void (*tick_handler)(void) = &dummy_handler;

void hal_tick_init(void)
{
	/* TODO: Do we need it now? PIT0 to 100 ms */
	SIM_SCGC6 |= SIM_SCGC6_PIT_MASK;
    NVICICER2 |= (1 << 4);				/* Clear any pending */
    NVICISER2 |= (1 << 4);				/* Enable interrupts */
	PIT_MCR = PIT_MCR_FRZ_MASK;			/* enable module, stop timers in debug mode */
	PIT_LDVAL0 = 48000000 / 10;			/* initialize timer */
	PIT_TCTRL0 |= PIT_TCTRL_TIE_MASK | PIT_TCTRL_TEN_MASK;
}

void hal_tick_set_handler(void (*handler)(void))
{
    if (handler == NULL) {
        tick_handler = &dummy_handler;
        return;
    }
    tick_handler = handler;
}

int hal_tick_get_tick_period_in_ms(void)
{
    return 100;
}

void pit0_isr(void)
{
    (*tick_handler)();
    PIT_TFLG0 |= PIT_TFLG_TIF_MASK;
}
