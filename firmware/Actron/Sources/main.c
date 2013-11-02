/*
 * main implementation: use this 'C' sample to create your own application
 *
 */

#define PE_MCUINIT

#include <stdio.h>
#include <stdarg.h>
#include "derivative.h"
#include "hidef.h"
#include "mpl115a2.h"
#include "sht21.h"
#include "mma7660.h"
#include "usb_cdc.h"
#include "afe44xx.h"

void MCU_init(void);
void debug_init(void);
void led_init(void);
void usb_init(void);
void bt_init(void);
void psox_init(void);
void i2c_init();

void delay(int t);
int usb_printf(const char * format, ...);
int uart_printf(const char * format, ...);

extern void BT_stack_init(void);
extern void BT_stack_task(void);

/* Sensor measurements, updated by ??? */
volatile uint32_t temp = 0;
volatile uint32_t hum = 0;
volatile uint32_t acc_x = 0;
volatile uint32_t acc_y = 0;
volatile uint32_t acc_z = 0;


/* AFE44xx measurements, updated by AFE ISR */
extern volatile afe44xx_data_t afe44xx_data;


int main(void)
{
	volatile int tmp = 0;

	MCU_init();
//	led_init();
//	debug_init();
	usb_init();
//	bt_init();
//	psox_init();
//	i2c_init();
//	mma7660_init();
	
	CDC_Init();
//	BT_stack_init();
//	afe44xx_init(4000);

	for(;;) {
    	Watchdog_Reset();
		CDC_Engine();
//		BT_stack_task();

//    	mma7760_acc(&acc_x, &acc_y, &acc_z);
//    	temp = sht21_temp();
//    	hum = sht21_humidity();

//    	DisableInterrupts;
//		uart_printf("%i,%i,%i,%i,%i,%i,%i,%i\r\n",
//				afe44xx_data.red_amb, afe44xx_data.red, afe44xx_data.ir,
//				temp, hum, acc_x, acc_y, acc_z);
//		EnableInterrupts;
		usb_printf("%i,%i,%i,%i,%i,%i,%i,%i\r\n",
				afe44xx_data.red_amb, afe44xx_data.red, afe44xx_data.ir,
				temp, hum, acc_x, acc_y, acc_z);
		delay(10000);
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


#define SEND_SIZE 128
int send_size = 0;
unsigned char send_buf[SEND_SIZE];

int usb_printf(const char * format, ...)
{
	va_list va;
	va_start(va, format);
	if (send_size == 0) {
		send_size += (uint8_t)vsnprintf(
				(char*)send_buf,
				SEND_SIZE - send_size,
				format, va);
	}
	va_end(va);
}


int count;
int uart_printf(const char * format, ...)
{
	va_list va;
	va_start(va, format);
	send_size = (uint8_t)vsnprintf(
				(char*)send_buf,
				SEND_SIZE,
				format, va);
	count = 0;
	while (count < send_size) {
		UART1_D = send_buf[count];
		count++;
		while (!(UART1_S1 & UART_S1_TC_MASK));
	}
	va_end(va);
}
