/*
 * These functions are to enable various clocks
 * on the STM32
 * Author: Dan Humeniuk
 * Version: 18 Apr 19
 */

#include "Clocks.h"
#include "main.h"

/*
 * Helper function that has the sole purpose of
 * enabling the GPIOA Clock
 */
void init_gpioa_clk(void)
{
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;	
}

/*
 * Helper function that has the sole purpose of
 * enabling the GPIOB Clock
 */
void init_gpiob_clk(void)
{
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
}

/*
 * Helper function that has the sole purpose of
 * enabling the GPIOC Clock
 */
void init_gpioc_clk(void)
{
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;	
}

/*
 * Helper function that has the sole purpose of
 * enabling the USART1 Clock
 */
void init_usart1_clk(void)
{
	RCC->APB2ENR |= RCC_APB2ENR_USART1EN;	
}

/*
 * Helper function that has the sole purpose of
 * enabling the I2C2 Clock
 */
void init_i2c2_clk(void)
{
	RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;
}
