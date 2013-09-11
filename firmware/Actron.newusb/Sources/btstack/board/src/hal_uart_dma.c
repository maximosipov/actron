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
