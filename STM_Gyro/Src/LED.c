/*
 * This library enables LEDs
 * on the STM32
 * Author: Dan Humeniuk
 * Version: 18 Apr 19
 */

#include "main.h"

void init_blue_LED(void)
{	
	/* Initialize Blue and Red LEDs */
	GPIOC->MODER |=	(1 << 14);
	
	GPIOC->MODER &=	~(1 << 15); 
	
	GPIOC->OTYPER &=	~(1 << 7);
														
	GPIOC->OSPEEDR &= ~((1 << 14) | (1 << 12)); 
														
	GPIOC->PUPDR &=	~((1 << 15) | (1 << 14 )); 
}

void init_red_LED(void)
{
	GPIOC->MODER |=	(1 << 12);
	
	GPIOC->MODER &=	~(1 << 13); 
	
	GPIOC->OTYPER &=	~(1 << 6); 
	
	GPIOC->OSPEEDR &= ~(1 << 12);
	
	GPIOC->PUPDR &=	~((1 << 13) | (1 << 12)); 
}
