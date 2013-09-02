/*
 * main implementation: use this 'C' sample to create your own application
 *
 */

#define PE_MCUINIT

#include <stdio.h>
#include <stdarg.h>
#include "derivative.h" /* include peripheral declarations */
#include "mpl115a2.h"
#include "sht21.h"
#include "mma7660.h"

void MCU_init(void);
void BT_init(void);
int usb_printf(const char * format, ...);


extern void BT_stack_init(void);
extern void BT_stack_task(void);

/* Must match virtual_com.h !!! */
#define  DATA_BUFF_SIZE     (64)
extern uint8_t g_curr_send_buf[DATA_BUFF_SIZE];
extern uint8_t g_send_size;

int main(void)
{
	volatile int counter = 0;
	volatile uint32_t tmp = 0;
	int hum;
	int temp;
	int pres;
	int accx, accy, accz;
	
	MCU_init(); /* call device initialization */
	
	/* Select USB clock source and enable clock */
    NVICICER1 |= (1 << 3);	/* Clear any pending interrupts on USB */
    NVICISER1 |= (1 << 3);	/* Enable interrupts from USB module */
	SIM_SOPT2 |= (uint32_t)0x00050000UL;
	SIM_SCGC4 |= (uint32_t)0x00040000UL;
    SIM_SOPT1 |= (uint32_t)0x80000000UL;

	/* Enable CLKOUT on PTC3 */
//	SIM_SCGC5 |= (uint32_t)0x0800UL;
//	PORTC_PCR3 = (uint32_t)0x0500UL;
//	tmp = SIM_SOPT2;
//	tmp |= 0x00C0UL;
//	tmp &= ~0x0020UL;
//	SIM_SOPT2 = tmp;
	
	/* Initialize the USB Test Application */
	TestApp_Init();
//	BT_stack_init();
//	mma7660_init();
//	afe44xx_init();

	for(;;) {
    	Watchdog_Reset();
    	TestApp_Task();
//    	BT_stack_task();

	   	counter++;
	   	if (counter == 1000) {
//			temp = sht21_temp();
	   	}
	   	if (counter == 2000) {
//			hum = sht21_humidity();
	   	}
	   	/* Breaks USB */
	   	if (counter == 3000) {
//			pres = mpl115a2_pressure();
	   	}
	   	if (counter == 4000) {
//	   	  mma7660_acc(&accx, &accy, &accz);
	   	}
	   	if (counter > 10000) {
	   		usb_printf("{ \"t\": %i.%i, \"h\": %i.%i }\r\n",
	   					temp/100, temp%100, hum/100, hum%100);

//				"{ \"t\": %i.%i, \"h\": %i.%i, \"x\": %s%i.%i, \"y\": %s%i.%i, \"z\": %s%i.%i }\r\n",
//				temp/100, temp%100, hum/100, hum%100,
//	   	        accx > 0 ? "" : "-", abs(accx/100), abs(accx%100),
//	   	        accy > 0 ? "" : "-", abs(accy/100), abs(accy%100),
//	   	        accz > 0 ? "" : "-", abs(accz/100), abs(accz%100));

//				"{ \"temp\": %i.%i, \"hum\": %i.%i, \"pres\": %i.%i }\r\n",
//				temp/100, temp%100, hum/100, hum%100, pres/100, pres%100);

	   		counter = 0;
	   	}
	}
	
	return 0;
}

void delay(int t)
{
    int delay_count = t;
    do {
       delay_count--;
       Watchdog_Reset();    /* Reset the COP */
    } while(delay_count);
}

int usb_printf(const char * format, ...)
{
	va_list va;
	va_start(va, format);
	if (g_send_size == 0) {
		g_send_size += (uint8_t)snprintf(
				(char*)g_curr_send_buf,
				DATA_BUFF_SIZE - g_send_size,
				format, va);
	}
	va_end(va);
}
