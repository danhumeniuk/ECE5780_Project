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
#include "StringHelper.h"
#include "GyroInit.h"
#include "init_LED.h"

/* Private define ------------------------------------------------------------*/

#define L3GD 0x6B

/* Private variables ---------------------------------------------------------*/
static uint16_t data_buff[2];

/* Private functions -----------------------------------------------*/

int request_write_I2C2(uint16_t slave_addr, uint16_t send, uint16_t buffer[], I2C_TypeDef * _I2C);
int request_read_I2C2(uint16_t slave_addr, uint16_t send, I2C_TypeDef * _I2C);

void SystemClock_Config(void);

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
	init_LEDs();
	
	uint16_t gyro_pins[4] = {GPIO_PIN_11, GPIO_PIN_13, GPIO_PIN_14, GPIO_PIN_0};
	GPIO_TypeDef * gpios[4] = {GPIOB, GPIOB, GPIOB, GPIOC};
	
	init_gyro(gyro_pins, gpios);
	
	set_alternate_func_I2C2();

	set_I2C2_100kHz();
	
	/* Set PE */
	I2C2->CR1 |= (1 << 0);
	
	/*******************************************************************/

	STARTOVER:
	
	/* Write to CTRL Reg 1, set to normal or sleep mode */
	data_buff[0] = 0x20;
	data_buff[1] = 0x0B;
	
	request_write(L3GD, 0x2, data_buff, I2C2);
	
	I2C2->CR2 |= I2C_CR2_STOP;
	
	int16_t y_axis;
	
	/* Infinite loop */	
  while (1)
  {
		HAL_Delay(100);
	
		//HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_9);

		data_buff[0] = 0x27;
		/* Request data from status register */
		if(!request_write(L3GD, 0x1, data_buff, I2C2))
		{
			goto STARTOVER;
		}

		if(!request_read(L3GD, 0x1, I2C2))
		{
			goto STARTOVER;
		}

		I2C2->CR2 |= I2C_CR2_STOP;
  
		uint16_t result = I2C2->RXDR & (0xFF);

		HAL_Delay(100);

		/* Y-AXIS available */
		if(result & 0x1)
		{
			data_buff[0] = 0x2A;
			request_write(L3GD, 0x1, data_buff, I2C2);
			
			request_read(L3GD, 0x1, I2C2);
			y_axis = I2C2->RXDR & 0xFF;
			
			HAL_Delay(100);
			
			data_buff[0] = 0x2B;
			request_write(L3GD, 0x1, data_buff, I2C2);
			
			request_read(L3GD, 0x1, I2C2);
			y_axis |= ((I2C2->RXDR & 0xFF) << 8);
			I2C2->CR2 |= I2C_CR2_STOP;
		}
	
		if(y_axis > 500)
		{
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);
		}
		else if(y_axis < -500)
		{
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);
		}
	}
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
