/** 
 * @file  hal_bt.c
 ***************************************************************************/

#include <stdint.h>
#include <btstack/hal_uart_dma.h>
#include "derivative.h" /* include peripheral declarations */

extern void delay(int);

void dummy_handler(void) {};

// RX state
static uint16_t  bytes_to_read = 0;
static uint8_t * rx_buffer_ptr = 0;

// TX state
static uint16_t  bytes_to_write = 0;
static uint8_t * tx_buffer_ptr = 0;

// handlers
static void (*rx_done_handler)(void) = dummy_handler;
static void (*tx_done_handler)(void) = dummy_handler;

/* Initializes UART1 and GPIO ports to communicate with the PAN1327 */
void hal_uart_dma_init(void)
{
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
    NVICICER1 |= (1 << 15);				/* Clear any pending */
    NVICISER1 |= (1 << 15);				/* Enable interrupts */

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
	PORTB_PCR18 = (PORTB_PCR18 & ~PORT_PCR_MUX_MASK) | PORT_PCR_MUX(0x03);
	
	/* _SHUTDN is pin 37 (PTB2, ALT1) */
	PORTB_PCR2 = (PORTB_PCR2 & ~PORT_PCR_MUX_MASK) | PORT_PCR_MUX(0x01);

    /* wait for Bluetooth to power up properly after providing 32khz clock */
    delay(1000);

    hal_uart_dma_set_baud(115200);
}

/* TODO */
int hal_uart_dma_set_baud(uint32_t baud)
{
    int result = 0;
    
    switch (baud){
        case 4000000:
        	/* TODO */
        	break;
        case 3000000:
        	/* TODO */
            break;
        case 2400000:
        	/* TODO */
            break;
        case 2000000:
        	/* TODO */
            break;
        case 1000000:
        	/* TODO */
            break;
        case 921600:
        	/* TODO */
            break;
        case 115200:
        	/* TODO */
            break;
        case 57600:
        	/* TODO */
            break;
        default:
            result = -1;
            break;
    }
    return result;
}

void hal_uart_dma_set_block_received(void(*the_block_handler)(void))
{
    rx_done_handler = the_block_handler;
}

void hal_uart_dma_set_block_sent(void(*the_block_handler)(void))
{
    tx_done_handler = the_block_handler;
}

/* Disables UART1 and clears the GPIO settings */
void hal_uart_dma_shutdown(void)
{
	/* TODO */
	UART1_C2 = 0;						
}

void hal_uart_dma_send_block(const uint8_t * data, uint16_t len)
{    
//    printf("HAL_TX: %u\n\r", len);
    
    /* disable TX interrupts */
	UART1_C2 &= ~UART_C2_TIE_MASK;						

    tx_buffer_ptr = (uint8_t *) data;
    bytes_to_write = len;

    /* enable TX interrupts */
	UART1_C2 |= UART_C2_TIE_MASK;						
}

/* int used to indicate a request for more new data */
void hal_uart_dma_receive_block(uint8_t *buffer, uint16_t len)
{
//    printf("HAL_RX: %u\n\r", len);

    /* disable RX interrupts */
	UART1_C2 &= ~UART_C2_RIE_MASK;						

    rx_buffer_ptr = buffer;
    bytes_to_read = len;
    
    /* enable RX interrupts */
	UART1_C2 |= UART_C2_RIE_MASK;						
}

void hal_uart_dma_set_sleep(uint8_t sleep)
{
    /* TODO */    
}

// block-wise "DMA" RX/TX UART driver
void uart1_isr(void)
{
	/* RX interrupt */
	if (UART1_S1 & UART_S1_RDRF_MASK) {
		if (bytes_to_read == 0) {
			/* disable RX interrupts */
			UART1_C2 &= ~UART_C2_RIE_MASK;
			return;
		}
		*rx_buffer_ptr = UART1_D;
		++rx_buffer_ptr;
		--bytes_to_read;
		if (bytes_to_read > 0) {
			return;
		}
		/* disable RX interrupts */
		UART1_C2 &= ~UART_C2_RIE_MASK;

		(*rx_done_handler)();
	}
	
	/* TX interrupt */
	if (UART1_S1 & UART_S1_TDRE_MASK) {
		if (bytes_to_write == 0){
			/* disable TX interrupts */
			UART1_C2 &= ~UART_C2_TIE_MASK;						
			return;
		}
		UART1_D = *tx_buffer_ptr;
		++tx_buffer_ptr;
		--bytes_to_write;
		
		if (bytes_to_write > 0) {
			return;
		}
		
		/* disable TX interrupts */
		UART1_C2 &= ~UART_C2_TIE_MASK;						

		(*tx_done_handler)();
	}
}
