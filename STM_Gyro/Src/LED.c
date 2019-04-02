#include "main.h"

void init_LEDs(void)
{	
	GPIO_InitTypeDef initLED = 
	{
		GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9,
		GPIO_MODE_OUTPUT_PP,
		GPIO_SPEED_FREQ_LOW,
		GPIO_NOPULL
	};
	HAL_GPIO_Init(GPIOC, &initLED);
}
