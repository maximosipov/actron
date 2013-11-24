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
#include "lmx9838.h"

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
int bt_printf(const char * format, ...);

/* AFE44xx measurements, updated by AFE ISR */
extern volatile afe44xx_data_t afe44xx_data;

/* SHT21 measurements, updated by PIT0 ISR */
extern volatile sht21_data_t sht21_data;

/* MMA7760 measurements, updated by PIT0 ISR */
extern volatile mma7660_data_t mma7660_data;


int main(void)
{
	volatile int tmp = 0;
	int loop = 0;
	int t1, t2, h1, h2;
	int x1, x2, y1, y2, z1, z2;

	MCU_init();
	led_init();
//	debug_init();
	usb_init();
	bt_init();
	psox_init();
	i2c_init();
	pit0_init();
	mma7660_init();
	
	TestApp_Init();
	afe44xx_init(4000);
	lmx9838_init();

	for(;;) {
    	Watchdog_Reset();
    	TestApp_Task();
		lmx9838_task();

    	if (loop >= 100000) {
//    		uart0_tx(1, "Z");
    		loop = 0;
			led_toggle();
			DisableInterrupts;
			/* Prepare SHT21 data */
			t1 = sht21_data.temp / 100;
			t2 = abs(sht21_data.temp % 100);
			h1 = sht21_data.hum / 100;
			h2 = abs(sht21_data.hum % 100);
			/* Prepare MMA7660 data */
			x1 = mma7660_data.x / 98;
			x2 = abs(mma7660_data.x % 98);
			y1 = mma7660_data.y / 98;
			y2 = abs(mma7660_data.y % 98);
			z1 = mma7660_data.z / 98;
			z2 = abs(mma7660_data.z % 98);
			/* Print */
			usb_printf("%i,%i,%i,%i.%i,%i.%i,%i.%i,%i.%i,%i.%i\r\n",
					afe44xx_data.red_amb, afe44xx_data.red, afe44xx_data.ir,
					t1, t2, h1, h2,
					x1, x2, y1, y2, z1, z2);
			bt_printf("%i,%i,%i,%i.%i,%i.%i,%i.%i,%i.%i,%i.%i\r\n",
					afe44xx_data.red_amb, afe44xx_data.red, afe44xx_data.ir,
					t1, t2, h1, h2,
					x1, x2, y1, y2, z1, z2);
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


/* Must match lmx9838.c !!! */
extern uint8_t bt_tx_buf[128];
extern uint16_t bt_tx_len;
int bt_printf(const char * format, ...)
{
	va_list va;
	va_start(va, format);
	if (!lmx9838_connected())
		return;
	bt_tx_len = (uint8_t)vsnprintf(
				(char*)bt_tx_buf,
				128,
				format, va);
	uart0_tx_start();
	va_end(va);
}
