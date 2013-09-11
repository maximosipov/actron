/*
 * main implementation: use this 'C' sample to create your own application
 *
 */

#define PE_MCUINIT

#include <stdio.h>
#include <stdarg.h>
#include "derivative.h"
#include "mpl115a2.h"
#include "sht21.h"
#include "mma7660.h"
#include "usb_cdc.h"
#include "usb_reg.h"

void MCU_init(void);
void led_init(void);
void usb_init(void);
void bt_init(void);

int usb_printf(const char * format, ...);

/* BT definitions */
extern void BT_stack_init(void);
extern void BT_stack_task(void);

int main(void)
{
	volatile int counter = 0;
	volatile uint32_t tmp = 0;
	int hum;
	int temp;
	int pres;
	int accx, accy, accz;
	
	MCU_init();
	led_init();
	usb_init();
	bt_init();
	afe44xx_init();
//	mma7660_init();
	
//	CDC_Init();
//	BT_stack_init();

	for(;;) {
    	Watchdog_Reset();
//		CDC_Engine();
//		BT_stack_task();

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
//	if (g_send_size == 0) {
//		g_send_size += (uint8_t)snprintf(
//				(char*)g_curr_send_buf,
//				DATA_BUFF_SIZE - g_send_size,
//				format, va);
//	}
//	va_end(va);
}

void led_init(void)
{
	/* LED destroys debugging */
#if 0
	/* Low pin 28 (PTB1), GPIO (ALT1) */
	SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK;
	PORTB_PCR1 = (PORTB_PCR1 & ~PORT_PCR_MUX_MASK) | PORT_PCR_MUX(0x01);
	GPIOB_PDDR |= 0x02;
	GPIOB_PDOR &= ~0x02;
#endif
#if 0
	/* 32.768kHz clock is is on pin 28 (PTB1), FTM2_CH0 (ALT3) */
	SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK;
	SIM_SCGC6 |= SIM_SCGC6_FTM1_MASK;
	/* configure FTM clock and mode (up-counting, EPWM) */
	FTM1_SC = (0x1 << 3) | (0x0);		/* Up-counting, 48MHz */
	FTM1_MODE = (0x1 << 2) | (0x1);		/* All access enabled */
	FTM1_CONF = (0x3 << 6);				/* Timer active in BDM mode */
	FTM1_C1SC = (0x1 << 5) | (0x1 << 2);/* EPWM */
	FTM1_CNTIN = 0;						/* Count from 0 */
	FTM1_CNT = 0;						/* Load counter */
	FTM1_MOD = 1465;					/* Counter to get to 32.768kHz */
	FTM1_C1V = 1465-146;				/* 10% duty cycle */
	FTM1_MODE = 0;						/* All access disabled */
	/* switch PORTB pin to FTM */
	PORTB_PCR1 = (PORTB_PCR1 & ~PORT_PCR_MUX_MASK) | PORT_PCR_MUX(0x03);
#endif
}

void usb_init(void)
{
    USB_REG_SET_ENABLE;
    USB_REG_SET_STDBY_STOP;      
    USB_REG_SET_STDBY_VLPx;

	/* Select USB clock source and enable clock */
    NVICICER1 |= (1 << 3);	/* Clear any pending interrupts on USB */
    NVICISER1 |= (1 << 3);	/* Enable interrupts from USB module */
    SIM_SOPT2 |= SIM_SOPT2_PLLFLLSEL_MASK | SIM_SOPT2_USBSRC_MASK;  
//	SIM_CLKDIV2 &= ~(SIM_CLKDIV2_USBDIV_MASK | SIM_CLKDIV2_USBFRAC_MASK);    
	SIM_SCGC4 |= (uint32_t)0x00040000UL;	/* Enable clock to USB */
    SIM_SOPT1 |= (uint32_t)0x80000000UL;	/* Enable voltage regulator */
}

void bt_init(void)
{
	int i;

	SIM_SCGC5 |= SIM_SCGC5_PORTC_MASK;
	SIM_SCGC4 |= SIM_SCGC4_UART1_MASK;

	/* 115200 bps, 8N1 HW flow control */
	UART1_BDH = 0;
	UART1_BDL = 26;						/* 115200 bps */
	UART1_C1 = 0;
	UART1_C3 = 0;
	UART1_C4 = 0;						/* adjust baud rate? */
	UART1_C5 = 0; // No DMA for now UART_C5_TDMAS_MASK | UART_C5_RDMAS_MASK;
	UART1_PFIFO = UART_PFIFO_TXFE_MASK | UART_PFIFO_RXFE_MASK;
	UART1_TWFIFO = 4;					/* 4 (8 data words FIFO) */
	UART1_RWFIFO = 1;					/* 1 (8 data words FIFO) */
	UART1_MODEM = UART_MODEM_RXRTSE_MASK | UART_MODEM_TXCTSE_MASK;
	UART1_C2 = UART_C2_TIE_MASK | UART_C2_RIE_MASK | UART_C2_TE_MASK | UART_C2_RE_MASK;
    NVICICER0 |= (1 << 18);				/* Clear any pending */
    NVICISER0 |= (1 << 18);				/* Enable interrupts */

	/* switch PORTC pins to UART1 RTS/CTS/TX/RX (ALT3) */
	PORTC_PCR1 = (PORTC_PCR1 & ~PORT_PCR_MUX_MASK) | PORT_PCR_MUX(0x03);
	PORTC_PCR2 = (PORTC_PCR2 & ~PORT_PCR_MUX_MASK) | PORT_PCR_MUX(0x03);
	PORTC_PCR3 = (PORTC_PCR3 & ~PORT_PCR_MUX_MASK) | PORT_PCR_MUX(0x03);
	PORTC_PCR4 = (PORTC_PCR4 & ~PORT_PCR_MUX_MASK) | PORT_PCR_MUX(0x03);

	/* 32.768kHz clock is is on pin 41 (PTB18), FTM2_CH0 (ALT3) */
	SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK;
	SIM_SCGC6 |= SIM_SCGC6_FTM1_MASK;
	/* configure FTM clock and mode (up-counting, EPWM) */
	FTM1_SC = (0x1 << 3) | (0x0);		/* Up-counting, 48MHz */
	FTM1_MODE = (0x1 << 2) | (0x1);		/* All access enabled */
	FTM1_CONF = (0x3 << 6);				/* Timer active in BDM mode */
	FTM1_C0SC = (0x1 << 5) | (0x1 << 2);/* EPWM */
	FTM1_CNTIN = 0;						/* Count from 0 */
	FTM1_CNT = 0;						/* Load counter */
	FTM1_MOD = 1465;					/* Counter to get to 32.768kHz */
	FTM1_C0V = 732;						/* 50% duty cycle */
	FTM1_MODE = 0;						/* All access disabled */
	/* switch PORTB pin to FTM */
	PORTB_PCR0 = (PORTB_PCR0 & ~PORT_PCR_MUX_MASK) | PORT_PCR_MUX(0x03);
	
	/* _SHUTDN is pin 37 (PTB2, ALT1) */
	PORTC_PCR0 = (PORTC_PCR0 & ~PORT_PCR_MUX_MASK) | PORT_PCR_MUX(0x01);
	GPIOC_PDDR |= 0x01;
	GPIOC_PDOR |= 0x01;

    /* wait for Bluetooth to power up properly after providing 32khz clock */
    delay(1000);
}
