/*
 * uart.c
 *
 *  Created on: 2 Nov 2012
 *      Author: Maxim Osipov
 */
#if 0
#include <types.h>
#include <UART.h>
#include "derivative.h"


UARTError InitializeUART(UARTBaudRate baudRate)
{
	/*
	SCGC1_SCI1 = 1;
	SOPT3_SCI1PS = 0;

	PTADD_PTADD1 = 1;
	PTADD_PTADD2 = 0;

	SCI1C1_LOOPS = 0;
	SCI1C1_SCISWAI = 0;
	SCI1C1_RSRC = 0;
	SCI1C1_M = 0;
	SCI1C1_WAKE = 0;
	SCI1C1_ILT = 0;
	SCI1C1_PE = 0;
	SCI1C1_PT = 0;

	SCI1C2_TIE = 0;
	SCI1C2_TCIE = 0;
	SCI1C2_RIE = 0;
	SCI1C2_ILIE = 0;
	SCI1C2_TE = 1;
	SCI1C2_RE = 1;
	SCI1C2_RWU = 0;
	SCI1C2_SBK = 0;

	SCI1C3_ORIE = 0;
	SCI1C3_NEIE = 0;
	SCI1C3_FEIE = 0;
	SCI1C3_PEIE = 0;

	SCI1BDH_LBKDIE = 0;
	SCI1BDH_RXEDGIE = 0;
	*/
	/* BaudRate = BUSCLK/(16xSBR), BUSCLK=MCGOUT/2=24M */
	/*
	SCI1BDH_SBR = 0;
	SCI1BDL = 13;
	*/
	
	return kUARTNoError;
}

UARTError WriteUARTN(const void* bytes, unsigned long length)
{
	/*
	unsigned long i = 0;
	while (i < length) {
		SCI1D = ((char*)bytes)[i];
		while(!SCI1S1_TC) ;
		i++;
	}
	*/
	return kUARTNoError;
}
#endif