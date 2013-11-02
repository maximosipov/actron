/*
** ###################################################################
**     This code is generated by the Device Initialization Tool.
**     It is overwritten during code generation.
**     USER MODIFICATION ARE PRESERVED ONLY INSIDE INTERRUPT SERVICE ROUTINES
**     OR EXPLICITLY MARKED SECTIONS
**
**     Project     : DeviceInitialization
**     Processor   : MK50DX128CMB7
**     Version     : Component 01.000, Driver 01.03, CPU db: 3.00.000
**     Datasheet   : K50P144M72SF1RM Rev. 0, Nov 2011
**     Compiler    : CodeWarrior ARM C Compiler
**     Date/Time   : 2013-11-02, 17:51, # CodeGen: 0
**     Abstract    :
**
**     Contents    :
**         Function "MCU_init" initializes selected peripherals
**
**     Copyright : 1997 - 2012 Freescale Semiconductor, Inc. All Rights Reserved.
**     
**     http      : www.freescale.com
**     mail      : support@freescale.com
** ###################################################################
*/

/* MODULE MCUinit */

#define PE_MCUINIT

#include <MK50D7.h>                    /* I/O map for MK50DX128CMB7 */
#include "MCUinit.h"

typedef void (*const tIsrFunc)(void);
typedef struct {
  uint32_t * __ptr;
  tIsrFunc __fun[110];
} tVectorTable;

/* User declarations and definitions */
extern void USB_ISR();
extern void i2c_isr();
extern void pit0_isr();
extern void uart1_isr();
extern void afe44xx_isr();
/* End of user declarations and definitions */

/*
** ===================================================================
**     Method      :  Cpu_SetBASEPRI (component MK50DX256MB7)
**
**     Description :
**         This method sets the BASEPRI core register.
**         This method is internal. It is used by Processor Expert only.
** ===================================================================
*/
/*lint -save  -e586 -e950 Disable MISRA rule (2.1,1.1) checking. */
#ifdef _lint
  #define Cpu_SetBASEPRI(Level)  /* empty */
#else
asm void Cpu_SetBASEPRI(register uint32_t Level) {
  MSR BASEPRI,R0;
  MOV PC,LR
}
#endif
/*lint -restore Enable MISRA rule (2.1,1.1) checking. */
/*
** ===================================================================
**     Method      :  __init_hardware (component MK50DX256MB7)
**
**     Description :
**         Initialization code for CPU core and a clock source.
** ===================================================================
*/
extern uint32_t __vector_table[];
void __init_hardware(void)
{
	  /*** ### MK50DX128CLH7 "Cpu" init code ... ***/
	  /*** PE initialization code after reset ***/
	  SCB_VTOR = (uint32_t)__vector_table; /* Set the interrupt vector table position */
	  /* SIM_SCGC6: RTC=1 */
	  SIM_SCGC6 |= (uint32_t)0x20000000UL;                       
	  if ((RTC_CR & RTC_CR_OSCE_MASK) == 0u) { /* Only if the OSCILLATOR is not already enabled */
	    /* RTC_CR: SC2P=0,SC4P=0,SC8P=0,SC16P=0 */
	    RTC_CR &= (uint32_t)~0x3C00UL;                      
	    /* RTC_CR: OSCE=1 */
	    RTC_CR |= (uint32_t)0x0100UL;                       
	    /* RTC_CR: CLKO=0 */
	    RTC_CR &= (uint32_t)~0x0200UL;                      
	  }
	  /* Disable the WDOG module */
	  /* WDOG_UNLOCK: WDOGUNLOCK=0xC520 */
	  WDOG_UNLOCK = (uint16_t)0xC520U;     /* Key 1 */
	  /* WDOG_UNLOCK : WDOGUNLOCK=0xD928 */
	  WDOG_UNLOCK  = (uint16_t)0xD928U;    /* Key 2 */
	  /* WDOG_STCTRLH: DISTESTWDOG=0,BYTESEL=0,TESTSEL=0,TESTWDOG=0,WAITEN=1,STOPEN=1,DBGEN=0,ALLOWUPDATE=1,WINEN=0,IRQRSTEN=0,CLKSRC=1,WDOGEN=0 */
	  WDOG_STCTRLH = (uint16_t)0x01D2U;                  
	  /* System clock initialization */
	  /* SIM_SCGC5: PORTA=1 */
	  SIM_SCGC5 |= (uint32_t)0x0200UL;     /* Enable clock gate for ports to enable pin routing */
	  /* SIM_CLKDIV1: OUTDIV1=0,OUTDIV2=0,OUTDIV3=0,OUTDIV4=1 */
	  SIM_CLKDIV1 = (uint32_t)0x00010000UL; /* Update system prescalers */
	  /* SIM_SOPT2: PLLFLLSEL=0 */
	  SIM_SOPT2 &= (uint32_t)~0x00010000UL; /* Select FLL as a clock source for various peripherals */
	  /* SIM_SOPT1: OSC32KSEL=0 */
	  SIM_SOPT1 &= (uint32_t)~0x000C0000UL; /* System oscillator drives 32 kHz clock for various peripherals */

	  /* PORTA_PCR18: ISF=0,MUX=0 */
	  PORTA_PCR18 &= (uint32_t)~0x01000700UL;                      

	  /* PORTA_PCR19: ISF=0,MUX=0 */
	  PORTA_PCR19 &= (uint32_t)~0x01000700UL;                      
	  /* Switch to FBE Mode */
	  /* OSC_CR: ERCLKEN=0,EREFSTEN=0,SC2P=0,SC4P=1,SC8P=1,SC16P=0 */
	  OSC_CR = (uint8_t)0x06U;                             
	  /* MCG_C7: OSCSEL=0 */
	  MCG_C7 &= (uint8_t)~(uint8_t)0x01U;                           
	  /* MCG_C2: LOCRE0=0,RANGE0=2,HGO0=0,EREFS0=1,LP=0,IRCS=0 */
	  MCG_C2 = (uint8_t)0x24U;                             
	  /* MCG_C1: CLKS=2,FRDIV=5,IREFS=0,IRCLKEN=1,IREFSTEN=0 */
	  MCG_C1 = (uint8_t)0xAAU;                             
	  /* MCG_C4: DMX32=0,DRST_DRS=0 */
	  MCG_C4 &= (uint8_t)~(uint8_t)0xE0U;                           
	  /* MCG_C5: PLLCLKEN0=0,PLLSTEN0=0,PRDIV0=0x0B */
	  MCG_C5 = (uint8_t)0x0BU;                             
	  /* MCG_C6: LOLIE0=0,PLLS=0,CME0=0,VDIV0=0 */
	  MCG_C6 = (uint8_t)0x00U;                             
	  while((MCG_S & MCG_S_OSCINIT0_MASK) == 0x00U) { /* Check that the oscillator is running */
	  }
	  while((MCG_S & MCG_S_IREFST_MASK) != 0x00U) { /* Check that the source of the FLL reference clock is the external reference clock. */
	  }
	  while((MCG_S & 0x0CU) != 0x08U) {    /* Wait until external reference clock is selected as MCG output */
	  }
	  /* Switch to PBE Mode */
	  /* OSC_CR: ERCLKEN=0,EREFSTEN=0,SC2P=0,SC4P=1,SC8P=1,SC16P=0 */
	  OSC_CR = (uint8_t)0x06U;                             
	  /* MCG_C7: OSCSEL=0 */
	  MCG_C7 &= (uint8_t)~(uint8_t)0x01U;                           
	  /* MCG_C1: CLKS=2,FRDIV=5,IREFS=0,IRCLKEN=1,IREFSTEN=0 */
	  MCG_C1 = (uint8_t)0xAAU;                             
	  /* MCG_C2: LOCRE0=0,RANGE0=2,HGO0=0,EREFS0=1,LP=0,IRCS=0 */
	  MCG_C2 = (uint8_t)0x24U;                             
	  /* MCG_C5: PLLCLKEN0=0,PLLSTEN0=0,PRDIV0=0x0B */
	  MCG_C5 = (uint8_t)0x0BU;                             
	  /* MCG_C6: LOLIE0=0,PLLS=1,CME0=0,VDIV0=0 */
	  MCG_C6 = (uint8_t)0x40U;                             
	  while((MCG_S & 0x0CU) != 0x08U) {    /* Wait until external reference clock is selected as MCG output */
	  }
	  while((MCG_S & MCG_S_LOCK0_MASK) == 0x00U) { /* Wait until locked */
	  }
	  /* Switch to PEE Mode */
	  /* OSC_CR: ERCLKEN=0,EREFSTEN=0,SC2P=0,SC4P=1,SC8P=1,SC16P=0 */
	  OSC_CR = (uint8_t)0x06U; // do we need internal caps???
	  /* MCG_C7: OSCSEL=0 */
	  MCG_C7 &= (uint8_t)~(uint8_t)0x01U;
	  /* MCG_C1: CLKS=0,FRDIV=5,IREFS=0,IRCLKEN=1,IREFSTEN=0 */
	  MCG_C1 = (uint8_t)0x2AU;
	  /* MCG_C2: LOCRE0=0,RANGE0=2,HGO0=0,EREFS0=1,LP=0,IRCS=0 */
	  MCG_C2 = (uint8_t)0x24U;
	  /* MCG_C5: PLLCLKEN0=0,PLLSTEN0=0,PRDIV0=0x0B */
	  MCG_C5 = (uint8_t)0x0BU;
	  /* MCG_C6: LOLIE0=0,PLLS=1,CME0=0,VDIV0=0 */
	  MCG_C6 = (uint8_t)0x40U;                             
	  while((MCG_S & 0x0CU) != 0x0CU) {    /* Wait until output of the PLL is selected */
	  }
}

/*
** ===================================================================
**     Method      :  MCU_init (component MK50DX256MB7)
**
**     Description :
**         Device initialization code for selected peripherals.
** ===================================================================
*/
void MCU_init(void)
{
    /* Initialization of the SIM module */

	/* PORTA_PCR4: ISF=0,MUX=7 */
	PORTA_PCR4 = (uint32_t)((PORTA_PCR4 & (uint32_t)~0x01000000UL) | (uint32_t)0x0700UL);
		  /* Initialization of the RCM module */
	/* RCM_RPFW: RSTFLTSEL=0 */
	RCM_RPFW &= (uint8_t)~(uint8_t)0x1FU;                           
	/* RCM_RPFC: RSTFLTSS=0,RSTFLTSRW=0 */
	RCM_RPFC &= (uint8_t)~(uint8_t)0x07U;                           
	
		/* Initialization of the PMC module */
	/* PMC_LVDSC1: LVDACK=1,LVDIE=0,LVDRE=1,LVDV=0 */
	PMC_LVDSC1 = (uint8_t)((PMC_LVDSC1 & (uint8_t)~(uint8_t)0x23U) | (uint8_t)0x50U);
	/* PMC_LVDSC2: LVWACK=1,LVWIE=0,LVWV=0 */
	PMC_LVDSC2 = (uint8_t)((PMC_LVDSC2 & (uint8_t)~(uint8_t)0x23U) | (uint8_t)0x40U);
	/* PMC_REGSC: BGEN=0,ACKISO=0,BGBE=0 */
	PMC_REGSC &= (uint8_t)~(uint8_t)0x19U;                           
	/* SMC_PMPROT: AVLP=0,ALLS=0,AVLLS=0 */
	SMC_PMPROT = (uint8_t)0x00U;         /* Setup Power mode protection register */
	/* Common initialization of the CPU registers */
	/* NVICIP20: PRI20=0 */
	NVICIP20 = (uint8_t)0x00U;                             
	/* ### */
	Cpu_SetBASEPRI(0U);
} /* MCU_init */


void led_init(void)
{
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
	FTM1_C1V = 1465-15;					/* 1% duty cycle */
	FTM1_MODE = 0;						/* All access disabled */
	/* switch PORTB pin to FTM */
	PORTB_PCR1 = (PORTB_PCR1 & ~PORT_PCR_MUX_MASK) | PORT_PCR_MUX(0x03);
}


void usb_init(void)
{
	/* Select USB clock source and enable clock */
    NVICICER2 |= (1 << 9);	/* Clear any pending interrupts on USB */
    NVICISER2 |= (1 << 9);	/* Enable interrupts from USB module */
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
	UART1_MODEM = 0; //UART_MODEM_RXRTSE_MASK | UART_MODEM_TXCTSE_MASK;
	UART1_C2 = UART_C2_TIE_MASK | UART_C2_RIE_MASK | UART_C2_TE_MASK | UART_C2_RE_MASK;
    NVICICPR0 |= (1 << 18);				/* Clear any pending */
    //NVICISER0 |= (1 << 18);				/* Enable interrupts */

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
	GPIOC_PDOR &= ~0x01;
	delay(100);
	GPIOC_PDOR |= 0x01;

    /* wait for Bluetooth to power up properly after providing 32khz clock */
    delay(1000);
}


void psox_init(void)
{
	/* Enable modules */
	SIM_SCGC5 |= SIM_SCGC5_PORTC_MASK;
	SIM_SCGC5 |= SIM_SCGC5_PORTD_MASK;
	SIM_SCGC6 |= SIM_SCGC6_SPI0_MASK;

	/* Configure SPI0, 8MHz */
	PORTD_PCR0 = (PORTD_PCR0 & ~PORT_PCR_MUX_MASK) | PORT_PCR_MUX(0x02);
	PORTD_PCR1 = (PORTD_PCR1 & ~PORT_PCR_MUX_MASK) | PORT_PCR_MUX(0x02);
	PORTD_PCR2 = (PORTD_PCR2 & ~PORT_PCR_MUX_MASK) | PORT_PCR_MUX(0x02);
	PORTD_PCR3 = (PORTD_PCR3 & ~PORT_PCR_MUX_MASK) | PORT_PCR_MUX(0x02);
	SPI0_CTAR0 = SPI_CTAR_FMSZ(0x7) | SPI_CTAR_PBR(0x1) | SPI_CTAR_BR(0x0)
		| SPI_CTAR_CSSCK(0x5) | SPI_CTAR_ASC(0x5) | SPI_CTAR_DT(0x7);
	SPI0_RSER = 0x0; /* disable all interrupts */
	SPI0_MCR = SPI_MCR_MSTR_MASK | SPI_MCR_PCSIS(0x1f);

	/* Configure pins for digital signals and SPI */
	/* CLKOUT (I) */
	PORTC_PCR5 = (PORTC_PCR5 & ~PORT_PCR_MUX_MASK) | PORT_PCR_MUX(0x01);
	GPIOC_PDDR &= ~(GPIOC_PDDR | (1<<5));
	/* _RESET (O) */
	PORTC_PCR6 = (PORTC_PCR6 & ~PORT_PCR_MUX_MASK) | PORT_PCR_MUX(0x01);
	GPIOC_PDDR |= (1<<6);
	/* ADC_RDY (I, interrupt) */
	PORTC_PCR7 = PORT_PCR_MUX(0x01) | PORT_PCR_IRQC(0x9);
	GPIOC_PDDR &= ~(GPIOC_PDDR | (1<<7));
	NVICICPR1 |= (1 << 10);				/* Clear any pending */
	NVICISER1 |= (1 << 10);				/* Enable interrupts */
	/* PD_ALM (I) */
	PORTD_PCR4 = (PORTD_PCR4 & ~PORT_PCR_MUX_MASK) | PORT_PCR_MUX(0x01);
	GPIOD_PDDR &= ~(GPIOD_PDDR | (1<<4));
	/* LED_ALM (I) */
	PORTD_PCR5 = (PORTD_PCR5 & ~PORT_PCR_MUX_MASK) | PORT_PCR_MUX(0x01);
	GPIOD_PDDR &= ~(GPIOD_PDDR | (1<<5));
	/* DIAG_END (I) */
	PORTD_PCR6 = (PORTD_PCR6 & ~PORT_PCR_MUX_MASK) | PORT_PCR_MUX(0x01);
	GPIOD_PDDR &= ~(GPIOD_PDDR | (1<<6));
	/* _AFE_PDN (O) */
	PORTD_PCR7 = (PORTD_PCR7 & ~PORT_PCR_MUX_MASK) | PORT_PCR_MUX(0x01);
	GPIOD_PDDR |= (1<<7);

	/* 8MHz clock is is on pin 21 (PTA4), FTM0_CH1 (ALT3) */
	SIM_SCGC5 |= SIM_SCGC5_PORTA_MASK;
	SIM_SCGC6 |= SIM_SCGC6_FTM0_MASK;
	/* configure FTM clock and mode (up-counting, EPWM) */
	FTM0_SC = (0x1 << 3) | (0x0);		/* Up-counting, 48MHz */
	FTM0_MODE = (0x1 << 2) | (0x1);		/* All access enabled */
	FTM0_CONF = (0x3 << 6);				/* Timer active in BDM mode */
	FTM0_C1SC = (0x1 << 5) | (0x1 << 2);/* EPWM */
	FTM0_CNTIN = 0;						/* Count from 0 */
	FTM0_CNT = 0;						/* Load counter */
	FTM0_MOD = 5;						/* Counter to get to 8MHz */
	FTM0_C1V = 2;						/* 50% duty cycle */
	FTM0_MODE = 0;						/* All access disabled */
	PORTA_PCR4 = (PORTA_PCR4 & ~PORT_PCR_MUX_MASK) | PORT_PCR_MUX(0x03);

	/* Take chip out of reset */
	GPIOD_PDOR |= (1<<7);
	GPIOC_PDOR |= (1<<6);

	delay(1000);
}


void debug_init(void)
{
	/* PTB16 as debug */
	SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK;
	PORTB_PCR16 = (PORTC_PCR16 & ~PORT_PCR_MUX_MASK) | PORT_PCR_MUX(0x01);
	GPIOB_PDDR |= (1<<16);
	GPIOB_PDOR &= ~(1<<16);
}


void i2c_init(void)
{
	SIM_SCGC4 |= SIM_SCGC4_I2C0_MASK;
	SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK;
	PORTB_PCR2 = (PORTC_PCR2 & ~PORT_PCR_MUX_MASK) | PORT_PCR_MUX(0x02);
	PORTB_PCR3 = (PORTC_PCR3 & ~PORT_PCR_MUX_MASK) | PORT_PCR_MUX(0x02);

#if 0
    NVICICPR0 |= (1 << 24);	/* Clear any pending interrupts on I2C0 */
    NVICISER0 |= (1 << 24);	/* Enable interrupts from I2C0 module */
#endif

    I2C0_F = 0x27;	/* mult = 1, div = 480, clk = 100kHz */ 
    I2C0_A1 = 0x02; /* our slave address (not used; we're master) */
#if 0
    I2C0_C1 = 0x80; /* module-enable, auto-ack = on */
#endif
}


/*
** ===================================================================
**     Interrupt handler : isr_default
**
**     Description :
**         User interrupt service routine. 
**     Parameters  : None
**     Returns     : Nothing
** ===================================================================
*/
PE_ISR(isr_default)
{
  /* Write your interrupt code here ... */

}
/* end of isr_default */


/*
** ===================================================================
**     Interrupt handler : isrINT_NMI
**
**     Description :
**         User interrupt service routine. 
**     Parameters  : None
**     Returns     : Nothing
** ===================================================================
*/
PE_ISR(isrINT_NMI)
{
  /* Write your interrupt code here ... */

}
/* end of isrINT_NMI */



#ifdef __cplusplus
extern "C" {
#endif
extern uint32_t __SP_INIT[];
extern void __thumb_startup( void );
#ifdef __cplusplus
}
#endif

/* Interrupt vector table */
#ifndef UNASSIGNED_ISR
  #define UNASSIGNED_ISR isr_default   /* unassigned interrupt service routine */
#endif


/* Pragma to place the interrupt vector table on correct location  location defined in linker file. */
#pragma define_section vectortable ".vectortable" ".vectortable" ".vectortable" far_abs R
static __declspec(vectortable) tVectorTable __vect_table = { /* Interrupt vector table */
   __SP_INIT,                                              /* 0 (0x00000000) (prior: -) */
  {
   (tIsrFunc)&__thumb_startup,                             /* 1 (0x00000004) (prior: -) */
   (tIsrFunc)&isrINT_NMI,                                  /* 2 (0x00000008) (prior: -2) */
   (tIsrFunc)&UNASSIGNED_ISR,                              /* 3 (0x0000000C) (prior: -1) */
   (tIsrFunc)&UNASSIGNED_ISR,                              /* 4 (0x00000010) (prior: -) */
   (tIsrFunc)&UNASSIGNED_ISR,                              /* 5 (0x00000014) (prior: -) */
   (tIsrFunc)&UNASSIGNED_ISR,                              /* 6 (0x00000018) (prior: -) */
   (tIsrFunc)&UNASSIGNED_ISR,                              /* 7 (0x0000001C) (prior: -) */
   (tIsrFunc)&UNASSIGNED_ISR,                              /* 8 (0x00000020) (prior: -) */
   (tIsrFunc)&UNASSIGNED_ISR,                              /* 9 (0x00000024) (prior: -) */
   (tIsrFunc)&UNASSIGNED_ISR,                              /* 10 (0x00000028) (prior: -) */
   (tIsrFunc)&UNASSIGNED_ISR,                              /* 11 (0x0000002C) (prior: -) */
   (tIsrFunc)&UNASSIGNED_ISR,                              /* 12 (0x00000030) (prior: -) */
   (tIsrFunc)&UNASSIGNED_ISR,                              /* 13 (0x00000034) (prior: -) */
   (tIsrFunc)&UNASSIGNED_ISR,                              /* 14 (0x00000038) (prior: -) */
   (tIsrFunc)&UNASSIGNED_ISR,                              /* 15 (0x0000003C) (prior: -) */
   (tIsrFunc)&UNASSIGNED_ISR,                              /* 16 (0x00000040) (prior: -) */
   (tIsrFunc)&UNASSIGNED_ISR,                              /* 17 (0x00000044) (prior: -) */
   (tIsrFunc)&UNASSIGNED_ISR,                              /* 18 (0x00000048) (prior: -) */
   (tIsrFunc)&UNASSIGNED_ISR,                              /* 19 (0x0000004C) (prior: -) */
   (tIsrFunc)&UNASSIGNED_ISR,                              /* 20 (0x00000050) (prior: -) */
   (tIsrFunc)&UNASSIGNED_ISR,                              /* 21 (0x00000054) (prior: -) */
   (tIsrFunc)&UNASSIGNED_ISR,                              /* 22 (0x00000058) (prior: -) */
   (tIsrFunc)&UNASSIGNED_ISR,                              /* 23 (0x0000005C) (prior: -) */
   (tIsrFunc)&UNASSIGNED_ISR,                              /* 24 (0x00000060) (prior: -) */
   (tIsrFunc)&UNASSIGNED_ISR,                              /* 25 (0x00000064) (prior: -) */
   (tIsrFunc)&UNASSIGNED_ISR,                              /* 26 (0x00000068) (prior: -) */
   (tIsrFunc)&UNASSIGNED_ISR,                              /* 27 (0x0000006C) (prior: -) */
   (tIsrFunc)&UNASSIGNED_ISR,                              /* 28 (0x00000070) (prior: -) */
   (tIsrFunc)&UNASSIGNED_ISR,                              /* 29 (0x00000074) (prior: -) */
   (tIsrFunc)&UNASSIGNED_ISR,                              /* 30 (0x00000078) (prior: -) */
   (tIsrFunc)&UNASSIGNED_ISR,                              /* 31 (0x0000007C) (prior: -) */
   (tIsrFunc)&UNASSIGNED_ISR,                              /* 32 (0x00000080) (prior: -) */
   (tIsrFunc)&UNASSIGNED_ISR,                              /* 33 (0x00000084) (prior: -) */
   (tIsrFunc)&UNASSIGNED_ISR,                              /* 34 (0x00000088) (prior: -) */
   (tIsrFunc)&UNASSIGNED_ISR,                              /* 35 (0x0000008C) (prior: -) */
   (tIsrFunc)&UNASSIGNED_ISR,                              /* 36 (0x00000090) (prior: -) */
   (tIsrFunc)&UNASSIGNED_ISR,                              /* 37 (0x00000094) (prior: -) */
   (tIsrFunc)&UNASSIGNED_ISR,                              /* 38 (0x00000098) (prior: -) */
   (tIsrFunc)&UNASSIGNED_ISR,                              /* 39 (0x0000009C) (prior: -) */
   (tIsrFunc)&UNASSIGNED_ISR,                              /* 40 (0x000000A0) (prior: -) */
   (tIsrFunc)&UNASSIGNED_ISR,                              /* 41 (0x000000A4) (prior: -) */
   (tIsrFunc)&UNASSIGNED_ISR,                              /* 42 (0x000000A8) (prior: -) */
   (tIsrFunc)&UNASSIGNED_ISR,                              /* 43 (0x000000AC) (prior: -) */
   (tIsrFunc)&UNASSIGNED_ISR,                              /* 44 (0x000000B0) (prior: -) */
   (tIsrFunc)&UNASSIGNED_ISR,                              /* 45 (0x000000B4) (prior: -) */
   (tIsrFunc)&UNASSIGNED_ISR,                              /* 46 (0x000000B8) (prior: -) */
   (tIsrFunc)&UNASSIGNED_ISR,                              /* 47 (0x000000BC) (prior: -) */
   (tIsrFunc)&UNASSIGNED_ISR,                              /* 48 (0x000000C0) (prior: -) */
   (tIsrFunc)&UNASSIGNED_ISR,                              /* 49 (0x000000C4) (prior: -) */
   (tIsrFunc)&UNASSIGNED_ISR,                              /* 50 (0x000000C8) (prior: -) */
   (tIsrFunc)&UNASSIGNED_ISR,                              /* 51 (0x000000CC) (prior: -) */
   (tIsrFunc)&UNASSIGNED_ISR,                              /* 52 (0x000000D0) (prior: -) */
   (tIsrFunc)&UNASSIGNED_ISR,                              /* 53 (0x000000D4) (prior: -) */
   (tIsrFunc)&UNASSIGNED_ISR,                              /* 54 (0x000000D8) (prior: -) */
   (tIsrFunc)&UNASSIGNED_ISR,                              /* 55 (0x000000DC) (prior: -) */
   (tIsrFunc)&UNASSIGNED_ISR,                              /* 56 (0x000000E0) (prior: -) */
   (tIsrFunc)&UNASSIGNED_ISR,                              /* 57 (0x000000E4) (prior: -) */
   (tIsrFunc)&UNASSIGNED_ISR,                              /* 58 (0x000000E8) (prior: -) */
   (tIsrFunc)&UNASSIGNED_ISR,                              /* 59 (0x000000EC) (prior: -) */
   (tIsrFunc)&UNASSIGNED_ISR,                              /* 60 (0x000000F0) (prior: -) */
   (tIsrFunc)&UNASSIGNED_ISR,                              /* 61 (0x000000F4) (prior: -) */
   (tIsrFunc)&UNASSIGNED_ISR,                              /* 62 (0x000000F8) (prior: -) */
   (tIsrFunc)&UNASSIGNED_ISR,                              /* 63 (0x000000FC) (prior: -) */
   (tIsrFunc)&UNASSIGNED_ISR,                              /* 64 (0x00000100) (prior: -) */
   (tIsrFunc)&UNASSIGNED_ISR,                              /* 65 (0x00000104) (prior: -) */
   (tIsrFunc)&UNASSIGNED_ISR,                              /* 66 (0x00000108) (prior: -) */
   (tIsrFunc)&UNASSIGNED_ISR,                              /* 67 (0x0000010C) (prior: -) */
   (tIsrFunc)&UNASSIGNED_ISR,                              /* 68 (0x00000110) (prior: -) */
   (tIsrFunc)&UNASSIGNED_ISR,                              /* 69 (0x00000114) (prior: -) */
   (tIsrFunc)&UNASSIGNED_ISR,                              /* 70 (0x00000118) (prior: -) */
   (tIsrFunc)&UNASSIGNED_ISR,                              /* 71 (0x0000011C) (prior: -) */
   (tIsrFunc)&UNASSIGNED_ISR,                              /* 72 (0x00000120) (prior: -) */
   (tIsrFunc)&UNASSIGNED_ISR,                              /* 73 (0x00000124) (prior: -) */
   (tIsrFunc)&UNASSIGNED_ISR,                              /* 74 (0x00000128) (prior: -) */
   (tIsrFunc)&UNASSIGNED_ISR,                              /* 75 (0x0000012C) (prior: -) */
   (tIsrFunc)&UNASSIGNED_ISR,                              /* 76 (0x00000130) (prior: -) */
   (tIsrFunc)&UNASSIGNED_ISR,                              /* 77 (0x00000134) (prior: -) */
   (tIsrFunc)&UNASSIGNED_ISR,                              /* 78 (0x00000138) (prior: -) */
   (tIsrFunc)&UNASSIGNED_ISR,                              /* 79 (0x0000013C) (prior: -) */
   (tIsrFunc)&UNASSIGNED_ISR,                              /* 80 (0x00000140) (prior: -) */
   (tIsrFunc)&UNASSIGNED_ISR,                              /* 81 (0x00000144) (prior: -) */
   (tIsrFunc)&UNASSIGNED_ISR,                              /* 82 (0x00000148) (prior: -) */
   (tIsrFunc)&UNASSIGNED_ISR,                              /* 83 (0x0000014C) (prior: -) */
   (tIsrFunc)&UNASSIGNED_ISR,                              /* 84 (0x00000150) (prior: -) */
   (tIsrFunc)&UNASSIGNED_ISR,                              /* 85 (0x00000154) (prior: -) */
   (tIsrFunc)&UNASSIGNED_ISR,                              /* 86 (0x00000158) (prior: -) */
   (tIsrFunc)&UNASSIGNED_ISR,                              /* 87 (0x0000015C) (prior: -) */
   (tIsrFunc)&UNASSIGNED_ISR,                              /* 88 (0x00000160) (prior: -) */
   (tIsrFunc)&USB_ISR,                              /* 89 (0x00000164) (prior: -) */
   (tIsrFunc)&UNASSIGNED_ISR,                              /* 90 (0x00000168) (prior: -) */
   (tIsrFunc)&UNASSIGNED_ISR,                              /* 91 (0x0000016C) (prior: -) */
   (tIsrFunc)&UNASSIGNED_ISR,                              /* 92 (0x00000170) (prior: -) */
   (tIsrFunc)&UNASSIGNED_ISR,                              /* 93 (0x00000174) (prior: -) */
   (tIsrFunc)&UNASSIGNED_ISR,                              /* 94 (0x00000178) (prior: -) */
   (tIsrFunc)&UNASSIGNED_ISR,                              /* 95 (0x0000017C) (prior: -) */
   (tIsrFunc)&UNASSIGNED_ISR,                              /* 96 (0x00000180) (prior: -) */
   (tIsrFunc)&UNASSIGNED_ISR,                              /* 97 (0x00000184) (prior: -) */
   (tIsrFunc)&UNASSIGNED_ISR,                              /* 98 (0x00000188) (prior: -) */
   (tIsrFunc)&UNASSIGNED_ISR,                              /* 99 (0x0000018C) (prior: -) */
   (tIsrFunc)&UNASSIGNED_ISR,                              /* 100 (0x00000190) (prior: -) */
   (tIsrFunc)&UNASSIGNED_ISR,                              /* 101 (0x00000194) (prior: -) */
   (tIsrFunc)&UNASSIGNED_ISR,                              /* 102 (0x00000198) (prior: -) */
   (tIsrFunc)&UNASSIGNED_ISR,                              /* 103 (0x0000019C) (prior: -) */
   (tIsrFunc)&UNASSIGNED_ISR,                              /* 104 (0x000001A0) (prior: -) */
   (tIsrFunc)&UNASSIGNED_ISR,                              /* 105 (0x000001A4) (prior: -) */
   (tIsrFunc)&UNASSIGNED_ISR,                              /* 106 (0x000001A8) (prior: -) */
   (tIsrFunc)&UNASSIGNED_ISR,                              /* 107 (0x000001AC) (prior: -) */
   (tIsrFunc)&UNASSIGNED_ISR,                              /* 108 (0x000001B0) (prior: -) */
   (tIsrFunc)&UNASSIGNED_ISR,                              /* 109 (0x000001B4) (prior: -) */
   (tIsrFunc)&UNASSIGNED_ISR                               /* 110 (0x000001B8) (prior: -) */
  }
};



/* END MCUinit */

/*
** ###################################################################
**
**     This file was created by Processor Expert 5.3 [05.01]
**     for the Freescale Kinetis series of microcontrollers.
**
** ###################################################################
*/
