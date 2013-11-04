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

void led_on(void);
void led_off(void);
void led_toggle(void);
void delay(int t);
int usb_printf(const char * format, ...);
int uart_printf(const char * format, ...);

extern void BT_stack_init(void);
extern void BT_stack_task(void);

/* Sensor measurements, updated by ??? */
volatile int acc_x = 0;
volatile int acc_y = 0;
volatile int acc_z = 0;

/* AFE44xx measurements, updated by AFE ISR */
extern volatile afe44xx_data_t afe44xx_data;

/* SHT21 measurements, updated by PIT0 ISR */
extern volatile sht21_data_t sht21_data;


int main(void)
{
	volatile int tmp = 0;
	int loop = 0;
	int t1, t2, h1, h2;

	MCU_init();
	led_init();
//	debug_init();
	usb_init();
//	bt_init();
	psox_init();
	i2c_init();
	pit0_init();
//	mma7660_init();
	
	TestApp_Init();
//	BT_stack_init();
	afe44xx_init(4000);

	for(;;) {
    	Watchdog_Reset();
    	TestApp_Task();
//		BT_stack_task();
//    	mma7760_acc(&acc_x, &acc_y, &acc_z);

    	if (loop >= 1000) {
    		loop = 0;
			led_toggle();
			DisableInterrupts;
			t1 = sht21_data.temp / 100;
			t2 = abs(sht21_data.temp % 100);
			h1 = sht21_data.hum / 100;
			h2 = abs(sht21_data.hum % 100);
			usb_printf("%i,%i,%i,%i.%i,%i.%i,%i,%i,%i\r\n",
					afe44xx_data.red_amb, afe44xx_data.red, afe44xx_data.ir,
					t1, t2, h1, h2, acc_x, acc_y, acc_z);
			EnableInterrupts;
    	} else {
    		loop += 1;
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


/* Must match virtual_com.h !!! */
#define  DATA_BUFF_SIZE     (128)
extern uint8_t g_curr_send_buf[DATA_BUFF_SIZE];
extern uint8_t g_send_size;

int usb_printf(const char * format, ...)
{
	va_list va;
	va_start(va, format);
	g_send_size = (uint8_t)vsnprintf(
				(char*)g_curr_send_buf,
				DATA_BUFF_SIZE,
				format, va);
	va_end(va);
}

#if 0
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
#endif