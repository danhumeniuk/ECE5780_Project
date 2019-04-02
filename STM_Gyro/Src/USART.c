#include "main.h"

void usart_init(void)
{
	
}

/**
	* @brief Transmits a single character via the USART TDR register
	* @retval NONE
  */
void transmit_char(char c)
{
	/* Waits until transmission is ready */
	while(!(USART1->ISR & 0x80))
	{
		/* Do Nothing */
	}
	
	USART1->TDR &= ~(0xFF);
	USART1->TDR |= c;
	
	return;
}

void transmit_str(char c[])
{
	int i = 0;
	while(c[i] != '\0')
	{
		HAL_Delay(1);
		transmit_char(c[i]);
		i++;
	}
	return;
}
