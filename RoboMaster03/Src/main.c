/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "string.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart7;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_UART7_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
char readBuf[1];
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_UART7_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_IT(&huart7,(uint8_t*)readBuf,1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle){
	switch(readBuf[0]){
		case 'r':
			HAL_GPIO_TogglePin(LD_RED_GPIO_Port,LD_RED_Pin);
			HAL_UART_Transmit(&huart7,(uint8_t*)"LED_RED!\r\n",strlen("LED_RED!\r\n"),HAL_MAX_DELAY);
			break;
		case 'g':
			HAL_GPIO_TogglePin(LD_GREEN_GPIO_Port,LD_GREEN_Pin);
			HAL_UART_Transmit(&huart7,(uint8_t*)"LED_GREEN!\r\n",strlen("LED_GREEN!\r\n"),HAL_MAX_DELAY);
			break;
		case '1':
			HAL_GPIO_TogglePin(LD1_GPIO_Port,LD1_Pin);
			HAL_UART_Transmit(&huart7,(uint8_t*)"LED1!\r\n",strlen("LED1!\r\n"),HAL_MAX_DELAY);
			break;
		case '2':
			HAL_GPIO_TogglePin(LD2_GPIO_Port,LD2_Pin);
			HAL_UART_Transmit(&huart7,(uint8_t*)"LED2!\r\n",strlen("LED2!\r\n"),HAL_MAX_DELAY);
			break;
		case '3':
			HAL_GPIO_TogglePin(LD3_GPIO_Port,LD3_Pin);
			HAL_UART_Transmit(&huart7,(uint8_t*)"LED3!\r\n",strlen("LED3!\r\n"),HAL_MAX_DELAY);
			break;
		case '4':
			HAL_GPIO_TogglePin(LD4_GPIO_Port,LD4_Pin);
			HAL_UART_Transmit(&huart7,(uint8_t*)"LED4!\r\n",strlen("LED4!\r\n"),HAL_MAX_DELAY);
			break;
		case '5':
			HAL_GPIO_TogglePin(LD5_GPIO_Port,LD5_Pin);
			HAL_UART_Transmit(&huart7,(uint8_t*)"LED5!\r\n",strlen("LED5!\r\n"),HAL_MAX_DELAY);
			break;
		case '6':
			HAL_GPIO_TogglePin(LD6_GPIO_Port,LD6_Pin);
			HAL_UART_Transmit(&huart7,(uint8_t*)"LED6!\r\n",strlen("LED6!\r\n"),HAL_MAX_DELAY);
			break;
		case '7':
			HAL_GPIO_TogglePin(LD7_GPIO_Port,LD7_Pin);
			HAL_UART_Transmit(&huart7,(uint8_t*)"LED7!\r\n",strlen("LED7!\r\n"),HAL_MAX_DELAY);
			break;
		case '8':
			HAL_GPIO_TogglePin(LD8_GPIO_Port,LD8_Pin);
			HAL_UART_Transmit(&huart7,(uint8_t*)"LED8!\r\n",strlen("LED8!\r\n"),HAL_MAX_DELAY);
			break;
	}
	HAL_UART_Receive_IT(&huart7,(uint8_t*)readBuf,1);
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief UART7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART7_Init(void)
{

  /* USER CODE BEGIN UART7_Init 0 */

  /* USER CODE END UART7_Init 0 */

  /* USER CODE BEGIN UART7_Init 1 */

  /* USER CODE END UART7_Init 1 */
  huart7.Instance = UART7;
  huart7.Init.BaudRate = 115200;
  huart7.Init.WordLength = UART_WORDLENGTH_8B;
  huart7.Init.StopBits = UART_STOPBITS_1;
  huart7.Init.Parity = UART_PARITY_NONE;
  huart7.Init.Mode = UART_MODE_TX_RX;
  huart7.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart7.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart7) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART7_Init 2 */

  /* USER CODE END UART7_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, LD8_Pin|LD7_Pin|LD6_Pin|LD5_Pin 
                          |LD4_Pin|LD3_Pin|LD2_Pin|LD1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD_RED_GPIO_Port, LD_RED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD_GREEN_GPIO_Port, LD_GREEN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LD8_Pin LD7_Pin LD6_Pin LD5_Pin 
                           LD4_Pin LD3_Pin LD2_Pin LD1_Pin */
  GPIO_InitStruct.Pin = LD8_Pin|LD7_Pin|LD6_Pin|LD5_Pin 
                          |LD4_Pin|LD3_Pin|LD2_Pin|LD1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : LD_RED_Pin */
  GPIO_InitStruct.Pin = LD_RED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD_RED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD_GREEN_Pin */
  GPIO_InitStruct.Pin = LD_GREEN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD_GREEN_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

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
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
