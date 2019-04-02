/* 
 * Functions for initializing the Gyroscope on the
 * STM32 Discovery
 */

#include "main.h"

void init_gyro(uint16_t I2C_pins[4], GPIO_TypeDef * GPIOs[4]);
  
void set_alternate_func_I2C2(void);

void set_I2C2_100kHz(void);
