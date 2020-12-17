/*
 * uart.h
 *
 *  Created on: 26.06.2020
 *      Author: Piotr
 */

#ifndef UART_UART_H_
#define UART_UART_H_

#include "stm32f10x.h"
//#include "F103_lib.h"
//#include "../CLOCK/clock.h"
#include <stdlib.h>

#define UART_RXB	128		/* Size of Rx buffer */
#define UART_TXB	128		/* Size of Tx buffer */

void UART_Conf(uint32_t BaudRate);

#define UART_BAUD 115200	// define the speed of interest to us

#define UART_RX_BUF_SIZE 32	// we define a buffer of 32 bytes
#define UART_RX_BUF_MASK ( UART_RX_BUF_SIZE - 1)	// we define a mask for our buffer

#define UART_TX_BUF_SIZE 2 // we define a buffer of 16 bytes
#define UART_TX_BUF_MASK ( UART_TX_BUF_SIZE - 1)	// we define a mask for our buffer

extern volatile uint8_t ascii_line;

// declarations of public functions
int uart_getc(void);
void uart_putc( char data );
void uart_puts(char *s);
void uart_putint(int value, int radix);

char * uart_get_str(char * buf);

void UART_RX_STR_EVENT(char * rbuf);
void register_uart_str_rx_event_callback(void (*callback)(char * pBuf));


#endif /* UART_UART_H_ */
