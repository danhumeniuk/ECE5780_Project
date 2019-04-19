#include "main.h"

void init_LEDs(void)
{	
	/* Initialize Blue and Red LEDs */
	GPIOC->MODER |=	(1 << 14) | (1 << 12); //SET bits 14 and 12 bit for Pins 6-7
	
	GPIOC->MODER &=	~((1 << 15) | (1 << 13)); //Clear bits 15 and 13 for Pins 6-7
														
	GPIOC->OTYPER &=	~((1 << 7) | (1 << 6)); //Clears bits 7 and 6 for Pins 6-7
														
	GPIOC->OSPEEDR &= ~((1 << 14) | (1 << 12)); //SET bits 14 and 12 bit for Pins 6-7
														
	GPIOC->PUPDR &=	~((1 << 15) | (1 << 14 ) | (1 << 13) | (1 << 12)); //Clears bits 15 - 12
}
