/*
 * This library enables the USART1 peripheral on the STM32 Discovery
 * Author: Dan Humeniuk
 * Version: 18 Apr 19
 */

#include "main.h"
#include "USART.h"

/**
	* Sets the alternate functions for GPIOA pins 9 and 10
  */
static void SET_ALT_FUNC_USART1(void)
{
	/* Set Alternate function for PA9 and PA 10 to USART1_TX and USART1_RX */
	GPIOA->AFR[1] &= ~((1 << 11) | (1 << 10) | (1 << 9) | (1 << 7) | (1 << 6) | (1 << 5));
	GPIOA->AFR[1] |= (1 << 8) | (1 << 4);
	
	/* Enable TX and RX */
	USART1->CR1 |= (1 << 3);
	USART1->CR1 |= (1 << 2);
}

/**
	* Initializes the GPIO pins for USART1, sets the alternate functions, and
  * enables the peripheral
  */
void USART1_INIT(int baud)
{
	/* Initializes GPIOA pins 9 and 10*/ 
	GPIOA->MODER |= (1 << 18)	 | (1 << 19) | (1 << 20) | (1 << 21);	
	GPIOA->OSPEEDR |= (1 << 18)	 | (1 << 19) | (1 << 20) | (1 << 21);
	GPIOA->PUPDR &= !((1 << 18)	 | (1 << 19) | (1 << 20) | (1 << 21));
	
	SET_ALT_FUNC_USART1();
	
	/* Set BAUD rate */
	USART1->BRR = SystemCoreClock/baud;
	
	/* Enable USART1 Peripheral */
	USART1->CR1 |= USART_CR1_UE;
}

/**
	* Transmits a single character via the USART TDR register
  */
void TRANSMIT_CHAR(char c)
{
	/* Waits until transmission is ready */
	while(!(USART1->ISR & 0x80))
	{
		/* Do Nothing */
	}
	
	USART1->TDR = c;
	
	return;
}

/**
	* Transmits a string with the use of the TRANSMIT_CHAR function
  */
void TRANSMIT_STR(char c[])
{
	int i = 0;
	while(c[i] != '\0')
	{
		HAL_Delay(1);
		TRANSMIT_CHAR(c[i]);
		i++;
	}
	TRANSMIT_CHAR('\0');
	
	return;
}

/**
	* Transmits a single byte via the USART TDR register
  */
void SEND_BYTE(int8_t byte)
{
	/* Waits until transmission is ready */
	while(!(USART1->ISR & 0x80))
	{
		/* Do Nothing */
	}
	
	USART1->TDR = byte;
	
	return;
}
