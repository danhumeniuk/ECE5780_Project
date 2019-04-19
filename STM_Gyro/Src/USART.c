#include "main.h"

void USART1_INIT(int baud)
{
	/* Initializes USART1 pins */ 

	
		
	USART1->BRR = HAL_RCC_GetHCLKFreq()/baud;
	
}

void SET_ALT_FUNC_USART1(void)
{
	/* Set Alternate function for PA9 and PA 10 to USART1_TX and USART1_RX */
	GPIOA->AFR[1] &= ~((1 << 11) | (1 << 10) | (1 << 9) | (1 << 7) | (1 << 6) | (1 << 5));
	GPIOA->AFR[1] |= (1 << 8) | (1 << 4);
	
	/* Enable TX and RX */
	USART1->CR1 |= (1 << 3);
	USART1->CR1 |= (1 << 2);
}

void USART1_Enable(void)
{
	/* Enable USART1 Peripheral */
	USART1->CR1 |= USART_CR1_UE;
}

/**
	* @brief Transmits a single character via the USART TDR register
	* @retval NONE
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
