/*
 * uart.c
 *
 *  Created on: 26.06.2020
 *      Author: Piotr
 */

#include "UART.h"




volatile uint8_t ascii_line;


// Definition of UART_RxBuf receiving buffer
volatile char UART_RxBuf[UART_RX_BUF_SIZE];
// Index definition which determine the amount of data in buffer
volatile uint8_t UART_RxHead; // Index which determine the "head of snake"
volatile uint8_t UART_RxTail; // Index which determine the "tail of snake"



// Definition of UART_TxBuf transmit buffer
volatile char UART_TxBuf[UART_TX_BUF_SIZE];
// Index definition which determine the amount of data in buffer
volatile uint8_t UART_TxHead; // Index which determine the "head of snake"
volatile uint8_t UART_TxTail; // Index which determine the "tail of snake"


// a pointer to a callback for the event UART_RX_STR_EVENT()
static void (*uart_rx_str_event_callback)(char * pBuf);


// function for registering a callback function in an event UART_RX_STR_EVENT()
void register_uart_str_rx_event_callback(void (*callback)(char * pBuf)) {
	uart_rx_str_event_callback = callback;
}


void UART_Conf(uint32_t BaudRate)
{

	USART_InitTypeDef USARTInit;
	GPIO_InitTypeDef GPIOInit;

	//*******************************************************************
	// Setting of microprocessor clocks
	//*******************************************************************
	RCC_APB2PeriphClockCmd(RCC_APB2ENR_USART1EN | RCC_APB2ENR_IOPAEN, ENABLE);

	//*******************************************************************
	// Setting of microprocessor pins for handling UART
	//*******************************************************************
	// Configuration PA9 as Tx
	GPIOInit.GPIO_Pin = GPIO_Pin_9;
	GPIOInit.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIOInit.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIOInit);

	// Configuration PA10 as Rx
	GPIOInit.GPIO_Pin = GPIO_Pin_10;
	GPIOInit.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIOInit);


	//*******************************************************************
	// Setting of UART communication
	//*******************************************************************

	USARTInit.USART_BaudRate = BaudRate;
	USARTInit.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USARTInit.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USARTInit.USART_Parity = USART_Parity_No;
	USARTInit.USART_StopBits = USART_StopBits_1;
	USARTInit.USART_WordLength = USART_WordLength_8b;

	USART_Init(USART1, &USARTInit);
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
	USART_Cmd(USART1, ENABLE);



	//*******************************************************************
	// Enabling interrupt vector for USART1
	//*******************************************************************
	//NVIC_EnableIRQ( USART1_IRQn);	// RM -> 205 page -> table "Vector table for other STM32F10xxx device" -> position 37

	NVIC_InitTypeDef NVIC_InitStructure;

  	// Enabling interrupt from USART1
  	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  	NVIC_Init(&NVIC_InitStructure);

}

//***********************************************************************************************
  __attribute__((interrupt)) void USART1_IRQHandler (void)
{

	  if(USART_GetFlagStatus(USART1, USART_IT_RXNE) != RESET)
	  {
		  register uint8_t tmp_head;
		  register char data;

		  data = (char)USART_ReceiveData(USART1);

		  // calculation a new indeks of "head snake"
		  tmp_head = ( UART_RxHead + 1) & UART_RX_BUF_MASK;

		  // check that the snake will not start to eat its own tail
		  if ( tmp_head == UART_RxTail )
		  {
			  // here we can handle the error caused in some way convenient for us
			  // trying to overwrite data in the buffer, it could get to a situation where
			  // our snake would start to eat its own tail
			  // one of the most obvious solutions is e.g. immediate
			  // reset the ascii_line variable or control the hardware line
			  // buffer busy
			  UART_RxHead = UART_RxTail;
		  } else
		  {
			  switch( data )
			  {
				  case 0:					// ignore byte = 0
				  case 10: break;			// ignore byte LF
				  case 13: ascii_line++;	// signal the presence of the next line in the buffer
				  default : UART_RxHead = tmp_head; UART_RxBuf[tmp_head] = data;

				  uart_putc(UART_RxBuf[tmp_head]);
			  }
		  }

		  USART_ClearFlag(USART1, USART_IT_RXNE);
	  }

	  if(USART_GetFlagStatus(USART1, USART_SR_TXE) != RESET)
	  {
		  // we check if the indexes are different
		  if ( UART_TxHead != UART_TxTail )
		  {
			  // we calculate and remember the new index of the snake's tail (it can align with the head)
			  UART_TxTail = (UART_TxTail + 1) & UART_TX_BUF_MASK;
			  // we return the byte downloaded from the buffer as the result of the function
			  #ifdef UART_DE_PORT
			  UART_DE_NADAWANIE;
			  #endif

			  USART_SendData(USART1, UART_TxBuf[UART_TxTail]);
			  while(USART_GetFlagStatus(USART1, USART_SR_TC) == RESET);
			  USART_ClearFlag(USART1, USART_SR_TC);
		  }
		  else
		  {
			// reset the interrupt flag that occurs when the buffer is empty
			  USART_ITConfig(USART1, USART_IT_TXE, DISABLE);
		  }
	  }

}
//***********************************************************************************************
  // An event to receive text string data from a circular buffer
  void UART_RX_STR_EVENT(char * rbuf)
  {
  	if( ascii_line ) {
  		if( uart_rx_str_event_callback ) {
  			uart_get_str( rbuf );
  			(*uart_rx_str_event_callback)( rbuf );
  		} else {
  			UART_RxHead = UART_RxTail;
  		}
  	}
  }

  //***********************************************************************************************
  // we define a function that adds one byte to the circular buffer
  void uart_putc( char data )
  {
  	uint8_t tmp_head;

  		tmp_head  = (UART_TxHead + 1) & UART_TX_BUF_MASK;

      // if there is no space in the circular buffer, the loop waits for subsequent characters
      while ( tmp_head == UART_TxTail ){}

      UART_TxBuf[tmp_head] = data;
      UART_TxHead = tmp_head;

      // initialize the interrupt that occurs when the buffer is empty, thanks
             // what the procedure will take care of later on sending data
             // interrupt handling
      USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
  }

  //***********************************************************************************************
  void uart_puts(char *s)		// sends string from RAM to UART
  {
    register char c;
    while ((c = *s++)) uart_putc(c);			// until you encounter 0 send a character
  }

  //***********************************************************************************************
  void uart_putint(int value, int radix)	// sends text to the serial port
  {
  	char string[17];				// buffer for the result of itoa function
  	itoa(value, string, radix);		// convert value to ASCII
  	uart_puts(string);				// send string to serial port
  }

  //***********************************************************************************************
  // we define a function that takes one byte from the circular buffer
  int uart_getc(void)
  {
  	int data = -1;
      // we check if the indexes are equal
      if ( UART_RxHead == UART_RxTail ) return data;

      // we calculate and remember the new index of the "snake's tail" (it may align with the head)
      UART_RxTail = (UART_RxTail + 1) & UART_RX_BUF_MASK;
      // we return the byte downloaded from the buffer as the result of the function
      data = UART_RxBuf[UART_RxTail];

      return data;
  }

  //***********************************************************************************************
  char * uart_get_str(char * buf)
  {
	  int c;
	  char * wsk = buf;
	  if( ascii_line )
	  {
		while( (c = uart_getc()) )
		{
			if( 13 == c || c < 0) break;
  			*buf++ = c;
  		}
  		*buf=0;
  		ascii_line--;
	  }
	  return wsk;
  }

