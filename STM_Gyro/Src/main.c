/* Daniel Humeniuk, Kyle Price, Melvin Bosnjak, Jaden Simon */
/* Mini Project */

/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private define ------------------------------------------------------------*/

#define L3GD 0x6B

/* Private variables ---------------------------------------------------------*/
static uint16_t data_buff[2];

/* Private function prototypes -----------------------------------------------*/

void SystemClock_Config(void);

/* Private user code ---------------------------------------------------------*/

int request_write(uint16_t slave_addr, uint16_t send, uint16_t buffer[]);
int request_read(uint16_t slave_addr, uint16_t send);

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
	
	__HAL_RCC_I2C2_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	
	/* INITIALIZE GPIO PINS */
	
	/* Initializes I2C pins */ 
	GPIO_InitTypeDef I2C_SDA_SCL = 
	{
		GPIO_PIN_11 | GPIO_PIN_13,
		GPIO_MODE_AF_OD,
		GPIO_SPEED_FREQ_HIGH,
		GPIO_NOPULL
	};
	
	GPIO_InitTypeDef Pin_PB14 = 
	{
		GPIO_PIN_14,
		GPIO_MODE_OUTPUT_PP,
		GPIO_SPEED_FREQ_HIGH,
		GPIO_NOPULL
	};
	
	GPIO_InitTypeDef Pin_PC0 = 
	{
		GPIO_PIN_0,
		GPIO_MODE_OUTPUT_PP,
		GPIO_SPEED_FREQ_HIGH,
		GPIO_NOPULL
	};
	
	GPIO_InitTypeDef initLED = 
	{
		GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9,
		GPIO_MODE_OUTPUT_PP,
		GPIO_SPEED_FREQ_LOW,
		GPIO_NOPULL
	};
	
	HAL_GPIO_Init(GPIOB, &I2C_SDA_SCL);
	HAL_GPIO_Init(GPIOB, &Pin_PB14);
	HAL_GPIO_Init(GPIOC, &Pin_PC0);
	HAL_GPIO_Init(GPIOC, &initLED);
	
	/* Set PB13 and PC0 to High */
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);
	
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
	
	/* Set PE */
	I2C2->CR1 |= (1 << 0);
	
	/* Set TXDR reg with WHO_AM_I */
	data_buff[0] = 0x0F;
	
	request_write(L3GD, 0x1, data_buff);
	
	request_read(L3GD, 0x1);

	uint32_t who_am_i = I2C2->RXDR & (0xFF);	
	
	I2C2->CR2 |= I2C_CR2_STOP;
	
	/* Pulse blue LED to ensure proper address was sent back */
	if(who_am_i == 0xD4)
	{
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
		HAL_Delay(1000);
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_7);
	}
	
	
	/*******************************************************************/

	STARTOVER:
	
	/* Write to CTRL Reg 1, set to normal or sleep mode */
	data_buff[0] = 0x20;
	data_buff[1] = 0x0B;
	
	request_write(L3GD, 0x2, data_buff);
	
	I2C2->CR2 |= I2C_CR2_STOP;
	
	int16_t x_axis;
	int16_t y_axis;
	
	/* Infinite loop */	
  while (1)
  {
		HAL_Delay(100);
	
		//HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_9);

		data_buff[0] = 0x27;
		/* Request data from status register */
		if(!request_write(L3GD, 0x1, data_buff))
		{
			goto STARTOVER;
		}

		if(!request_read(L3GD, 0x1))
		{
			goto STARTOVER;
		}

		I2C2->CR2 |= I2C_CR2_STOP;
  
		uint16_t result = I2C2->RXDR & (0xFF);

		HAL_Delay(100);

		/* X-AXIS available */
		if(result & 0x2)
		{
			data_buff[0] = 0x28;

			request_write(L3GD, 0x1, data_buff);
			
			request_read(L3GD, 0x1);
			
			x_axis = I2C2->RXDR & (0xFF);
			HAL_Delay(100);
			
			data_buff[0] = 0x29;
			
			request_write(L3GD, 0x1, data_buff);
			request_read(L3GD, 0x1);
			
			x_axis |= ((I2C2->RXDR & 0xFF) << 8);

			I2C2->CR2 |= I2C_CR2_STOP;
			
			HAL_Delay(90);
		}
  
		/* Y-AXIS available */
		if(result & 0x1)
		{
			data_buff[0] = 0x2A;
			request_write(L3GD, 0x1, data_buff);
			
			request_read(L3GD, 0x1);
			y_axis = I2C2->RXDR & 0xFF;
			
			HAL_Delay(100);
			
			data_buff[0] = 0x2B;
			request_write(L3GD, 0x1, data_buff);
			
			request_read(L3GD, 0x1);
			y_axis |= ((I2C2->RXDR & 0xFF) << 8);
			I2C2->CR2 |= I2C_CR2_STOP;
		}

		/* Check for threshold values for y and x axis and toggle pins accordingly */
		if(y_axis > 1000)
		{
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);	
		}
		if(y_axis < -1000)
		{
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);	
		}
		if(x_axis > 1000)
		{
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);	
		}
		
		if(x_axis < -1000)
		{
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);	
		}
		
	}

}

/*
 * Helper function to request a send. Returns a 0 if request failed
 * If it returns 1, the TX register will be available. This function
 * does not set the stop condition
 */
int request_write(uint16_t slave_addr, uint16_t send, uint16_t buffer[])
{
	/* Clear the NBYTES and SADD */
	I2C2->CR2 &= ~((0x7F << 16) | (0x3FF << 0));
		
	/* Slave address */
	I2C2->CR2 |= ((slave_addr & 0x3FF) << 1);
		
	/* Set Bytes to Transmit */
	I2C2->CR2 |= ((send & 0xFF) << 16);
	
	/* Request write operation */
	I2C2->CR2 &= ~I2C_CR2_RD_WRN;
		
	/* Set the Start bit */
	I2C2->CR2 |= I2C_CR2_START;
	
	while(1)
	{
		/* Condition for NACK */
		if(I2C2->ISR & I2C_ISR_NACKF)
		{
			/* Glow Red DEBUGGING ONLY */
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
			return 0;
		}
		/* Condition for TXIS */
		else if(I2C2->ISR & I2C_ISR_TXIS)
		{
			break;
		}
	}
	
	/* Set TXDR reg with address */		
	int i = 0;
	while(i < send)
	{
		I2C2->TXDR = buffer[i];
		i++;

		if(i == send)
		{
			break;
		}
		while(1)
		{
			/* Condition for NACK */
			if(I2C2->ISR & I2C_ISR_NACKF)
			{
				/* Glow Red DEBUGGING ONLY */
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
				return 0;
			}
			/* Condition for TXIS */
			else if(I2C2->ISR & I2C_ISR_TXIS)
			{
				break;
			}
		}
	}
	/* Wait for transfer to complete */
	while(!(I2C2->ISR & I2C_ISR_TC))
	{

	}

	return 1;
}

int request_read(uint16_t slave_addr, uint16_t send)
{
	/* Clear the NBYTES and SADD */
	I2C2->CR2 &= ~((0x7F << 16) | (0x3FF << 0));
	
	/* Slave address */
	I2C2->CR2 |= (slave_addr << 1);
		
	/* Set Bytes to Transmit to 1 */
	I2C2->CR2 |= (send << 16);
	
	/* Request read operation */
	I2C2->CR2 |= I2C_CR2_RD_WRN;

	/* Set the Start bit */
	I2C2->CR2 |= I2C_CR2_START;
	
	while(1)
	{
		/* Condition for NACK */
		if(I2C2->ISR & I2C_ISR_NACKF)
		{
			/* Glow Red FOR DEBUGGING PUROPSES */
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
		}
		/* Condition for TXIS */
		else if(I2C2->ISR & I2C_ISR_RXNE)
		{
			break;
		}
	}
		
	while(!(I2C2->ISR & I2C_ISR_TC))
	{
	
	}
	
	return 1;
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}


/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
