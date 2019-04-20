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
#include "Gyro.h"
#include "LED.h"
#include "USART.h"
#include "Clocks.h"

/* Private define ------------------------------------------------------------*/

#define L3GD 0x6B					// Address of the gyroscope
#define GYRO_ODR 95 			// How quickly the L3GD20 updates
#define GYRO_SENS 8.75 		// The sensitivity in mdps/digit

/* Global variables ---------------------------------------------------------*/
uint16_t data_buff[2];		// Buffer for I2C communication
int32_t rotation = 0; 		// Rotation of the gyroscope
int16_t calibration = 0; 	// Calibration data computed at initialization

/* Private functions -----------------------------------------------*/

void SystemClock_Config(void);
int16_t Read_YAxis(void);
void Calibrate(void);

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
	
	init_gpioa_clk();
	init_gpiob_clk();
	init_gpioc_clk();
	init_usart1_clk();
	init_i2c2_clk();	
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
	
	/* INITIALIZE GPIO PINS */
	init_blue_LED();
	
	init_red_LED();
	
	uint16_t gyro_pins[4] = {GPIO_PIN_11, GPIO_PIN_13, GPIO_PIN_14, GPIO_PIN_0};
	
	GPIO_TypeDef * gpios[4] = {GPIOB, GPIOB, GPIOB, GPIOC};
	
	init_gyro(gyro_pins, gpios);
	
	set_alternate_func_I2C2();

	set_I2C2_100kHz();

	/* ENABLE USART1 */
	USART1_INIT(115200);
		
	/* Set PE */
	I2C2->CR1 |= (1 << 0);
	
	/*******************************************************************/
	
	/* Write to CTRL Reg 1, set to normal mode with Y-axis enabled (PD = 1, Yen = 1) */
	/* Also set bandwidth to reduce noise */
	data_buff[0] = 0x20;
	data_buff[1] = 0x3F;		
	request_write(L3GD, 0x2, data_buff, I2C2);
	I2C2->CR2 |= I2C_CR2_STOP;
	
	HAL_Delay(1); // Short pause to allow for lines to stabilize

	/* Write to CTRL Reg 3, enable I2_DRDY */
	data_buff[0] = 0x22;
	data_buff[1] = 0x08;
	request_write(L3GD, 0x2, data_buff, I2C2);
	I2C2->CR2 |= I2C_CR2_STOP;
	
	HAL_Delay(1);
	
	// Calibrate the gyroscope
	Calibrate();
	
	/* Setup EXTI for PC2 (INT2 from gyroscope is mapped to PC2) */
	SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI2_PC;
	EXTI->IMR |= EXTI_IMR_IM2; // PC2 -> EXTI channel 2
	EXTI->RTSR |= EXTI_RTSR_RT2; // Set rising trigger
	
	/* EXTI interrupt code */
	HAL_NVIC_SetPriority(EXTI2_3_IRQn, 0, 0); // Give it the highest priority
	HAL_NVIC_EnableIRQ(EXTI2_3_IRQn);	
	
	Read_YAxis(); // Flush the gyroscope
	
	while(1)
	{
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
	* Takes 100 samples over 1 second from the gyroscope to get a drift value
	*/
void Calibrate(void)
{
	int32_t sum = 0;
	
	for (int i = 0; i < 100; i++)
	{
		sum += Read_YAxis();
		HAL_Delay(10);
	}
	
	calibration = sum / 100;

	GPIOC->ODR |= GPIO_ODR_7; // Flashes the blue LED to show it is calibrated
}

/**
	* Reads the two registers containing the y-axis angular velocity of the L3GD20
	* Returns the resulting value.
	*/
int16_t Read_YAxis(void)
{
		int16_t y_axis = 0;	
	
		data_buff[0] = 0x2A;
		request_write(L3GD, 0x1, data_buff, I2C2);
		request_read(L3GD, 0x1, I2C2);
		y_axis = I2C2->RXDR & 0xFF;
		
		data_buff[0] = 0x2B;
		request_write(L3GD, 0x1, data_buff, I2C2);
		request_read(L3GD, 0x1, I2C2);
		y_axis |= ((I2C2->RXDR & 0xFF) << 8);
	
		I2C2->CR2 |= I2C_CR2_STOP;
	
		return y_axis;
}
/**
	* Handles interrupts from EXTI channel 2 and 3.
	* Since we only have channel 2 enabled, we will forgo checking if the interrupt
	* was caused by the third channel. Channel 2 is connected to INT2_DRDY of the 
	* L3GD20, meaning new data is ready to be read.
	*/
void EXTI2_3_IRQHandler(void)
{	 		
		int16_t y_speed = Read_YAxis(); // Get the new y-axis data
		y_speed -= calibration; // Offset it by the calibration data
		rotation += ((int32_t)y_speed << 16) * (GYRO_SENS / 1000 / GYRO_ODR); // Update the rotation
	
		// Read the upper 16 bits for the current rotation
		if((rotation >> 16) > 15)
		{
			GPIOC->ODR |= GPIO_ODR_6; // Turn on the red LED
			GPIOC->ODR &= ~GPIO_ODR_7; // Turn off the blue LED
		}
		else if((rotation >> 16) < -15)
		{
			GPIOC->ODR |= GPIO_ODR_7; // Turn on the blue LED
			GPIOC->ODR &= ~GPIO_ODR_6; // Turn off the red LED
		}
		else
		{
			GPIOC->ODR &= ~(GPIO_ODR_6 | GPIO_ODR_7); // Turn off both LEDs
		}
				
		// Send the current rotation over UART to the ESP8266
		SEND_BYTE((int8_t)(rotation >> 16));		
		EXTI->PR |= EXTI_PR_PR2; // Clear the pending interrupt flag		
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
