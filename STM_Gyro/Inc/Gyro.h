/* 
 * Functions for initializing the Gyroscope on the
 * STM32 Discovery
 */

#include "main.h"

void init_gyro(uint16_t I2C_pins[4], GPIO_TypeDef * GPIOs[4]);
  
void set_alternate_func_I2C2(void);

void set_I2C2_100kHz(void);

int request_write(uint16_t slave_addr, uint16_t send, uint16_t buffer[], I2C_TypeDef * _I2C);

int request_read(uint16_t slave_addr, uint16_t send, I2C_TypeDef * _I2C);
