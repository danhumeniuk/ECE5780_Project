/* 
 * Functions for initializing the Gyroscope on the
 * STM32 Discovery
 */
#include "main.h"

void init_gyro(uint16_t I2C_pins[4], GPIO_TypeDef * GPIOs[4])
{
	/* Initializes I2C pins */ 
	GPIO_InitTypeDef I2C_SDA = 
	{
		I2C_pins[0],
		GPIO_MODE_AF_OD,
		GPIO_SPEED_FREQ_HIGH,
		GPIO_NOPULL
	};
	
	GPIO_InitTypeDef I2C_SCL = 
	{
		I2C_pins[1],
		GPIO_MODE_AF_OD,
		GPIO_SPEED_FREQ_HIGH,
		GPIO_NOPULL
	};
	
	GPIO_InitTypeDef SLAVER = 
	{
		I2C_pins[2],
		GPIO_MODE_OUTPUT_PP,
		GPIO_SPEED_FREQ_HIGH,
		GPIO_NOPULL
	};
	
	GPIO_InitTypeDef SPI_I2C_Pin = 
	{
		I2C_pins[3],
		GPIO_MODE_OUTPUT_PP,
		GPIO_SPEED_FREQ_HIGH,
		GPIO_NOPULL
	};
	
	/* Initialize Pins */
	HAL_GPIO_Init(GPIOs[0], &I2C_SDA);
	HAL_GPIO_Init(GPIOs[1], &I2C_SCL);
	HAL_GPIO_Init(GPIOs[2], &SLAVER);
	HAL_GPIO_Init(GPIOs[3], &SPI_I2C_Pin);
	
	/* Set Slaver and SPI/I2C pins to high to High */
	
	HAL_GPIO_WritePin(GPIOs[2], I2C_pins[2], GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOs[3], I2C_pins[3], GPIO_PIN_SET);
	
}

void set_alternate_func_I2C2(void)
{
	/* SET ALTERNATE FUNCTIONS FOR PB11 AND PB13 */
	/* PB11 AF1 - 0001 */
	/* Set bits for PB11 xxx1*/ 
	GPIOB->AFR[1] |= (1 << 12);
	
	/* Clear bits for PB11 000x */
	GPIOB->AFR[1] &= ~((1 << 15) | (1 << 14) | (1 << 13));
	
	/* PB13 AF5 - 0101 */
	/* Set bits for PB13 x1x1*/
	GPIOB->AFR[1] |= (0x5 << 20);
	
	/* Clear bits for PB13 0x0x */
	GPIOB->AFR[1] &= ~((1 << 23) | (1 << 21));
}

void set_I2C2_100kHz(void)
{
	/* SET TIMING REGISTER FOR I2C2 SET TO 100kHz */
	/* PRESC */
	I2C2->TIMINGR |= (0x1 << 28);
	/* SCLDEL */
	I2C2->TIMINGR |= (0X4 << 20);
	/* SDADEL */
  I2C2->TIMINGR |= (0x2 << 16);
	/* SCLH */
	I2C2->TIMINGR |= (0xF << 8);
	/* SCLL */
	I2C2->TIMINGR |= (0x13 << 0);
}

int request_write(uint16_t slave_addr, uint16_t send, uint16_t buffer[], I2C_TypeDef * _I2C)
{
	/* Clear the NBYTES and SADD */
	_I2C->CR2 &= ~((0x7F << 16) | (0x3FF << 0));
		
	/* Slave address */
	_I2C->CR2 |= ((slave_addr & 0x3FF) << 1);
		
	/* Set Bytes to Transmit */
	_I2C->CR2 |= ((send & 0xFF) << 16);
	
	/* Request write operation */
	_I2C->CR2 &= ~I2C_CR2_RD_WRN;
		
	/* Set the Start bit */
	_I2C->CR2 |= I2C_CR2_START;
	
	while(1)
	{
		/* Condition for NACK */
		if(_I2C->ISR & I2C_ISR_NACKF)
		{
			/* Glow Red DEBUGGING ONLY */
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
			return 0;
		}
		/* Condition for TXIS */
		else if(_I2C->ISR & I2C_ISR_TXIS)
		{
			break;
		}
	}
	
	/* Set TXDR reg with address */		
	int i = 0;
	while(i < send)
	{
		_I2C->TXDR = buffer[i];
		i++;

		if(i == send)
		{
			break;
		}
		while(1)
		{
			/* Condition for NACK */
			if(_I2C->ISR & I2C_ISR_NACKF)
			{
				return 0;
			}
			/* Condition for TXIS */
			else if(_I2C->ISR & I2C_ISR_TXIS)
			{
				break;
			}
		}
	}
	/* Wait for transfer to complete */
	while(!(_I2C->ISR & I2C_ISR_TC))
	{

	}
	return 1;
}

int request_read(uint16_t slave_addr, uint16_t send, I2C_TypeDef * _I2C)
{
	/* Clear the NBYTES and SADD */
	_I2C->CR2 &= ~((0x7F << 16) | (0x3FF << 0));
	
	/* Slave address */
	_I2C->CR2 |= (slave_addr << 1);
		
	/* Set Bytes to Transmit to 1 */
	_I2C->CR2 |= (send << 16);
	
	/* Request read operation */
	_I2C->CR2 |= I2C_CR2_RD_WRN;

	/* Set the Start bit */
	_I2C->CR2 |= I2C_CR2_START;
	
	while(1)
	{
		/* Condition for NACK */
		if(_I2C->ISR & I2C_ISR_NACKF)
		{
			/* Glow Red FOR DEBUGGING PUROPSES */
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
		}
		/* Condition for TXIS */
		else if(_I2C->ISR & I2C_ISR_RXNE)
		{
			break;
		}
	}
		
	while(!(_I2C->ISR & I2C_ISR_TC))
	{
	
	}
	
	return 1;
}
