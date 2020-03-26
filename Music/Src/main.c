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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum{
	Do1L = 262,     ///*261.63Hz*/    3822us
	Re2L = 294,     ///*293.66Hz*/    3405us
	Mi3L = 330,     ///*329.63Hz*/    3034us
	Fa4L = 349,     ///*349.23Hz*/    2863us
	So5L = 392,     ///*392.00Hz*/    2551us
	La6L = 440,     ///*440.00Hz*/    2272us
	Si7L = 494,     ///*493.88Hz*/    2052us

	Do1M = 523,     ///*523.25Hz*/    1911us
	Re2M = 587,     ///*587.33Hz*/    1703us
	Mi3M = 659,     ///*659.26Hz*/    1517us
	Fa4M = 698,     ///*698.46Hz*/    1432us
	So5M = 784,     ///*784.00Hz*/    1276us
	La6M = 880,     ///*880.00Hz*/    1136us
	Si7M = 988,     ///*987.77Hz*/    1012us

	Do1H = 1047,     ///*1046.50Hz*/   956us
	Re2H = 1175,     ///*1174.66Hz*/   851us
	Mi3H = 1319,     ///*1318.51Hz*/   758us
	Fa4H = 1397,     ///*1396.91Hz*/   716us
	So5H = 1568,     ///*1567.98Hz*/   638us
	La6H = 1760,     ///*1760.00Hz*/   568us
	Si7H = 1976,     ///*1975.53Hz*/   506us

	Silent  = 0,
	Finish  = -1
} buzzer_freq_t;

buzzer_freq_t littleStar_freq[] = {
		Do1M, Do1M, So5M, So5M, La6M, La6M, So5M, Silent,
		Fa4M, Fa4M, Mi3M, Mi3M, Re2M, Re2M, Do1M, Silent,
		So5M, So5M, Fa4M, Fa4M, Mi3M, Mi3M, Re2M, Silent,
		So5M, So5M, Fa4M, Fa4M, Mi3M, Mi3M, Re2M, Silent,
		Do1M, Do1M, So5M, So5M, La6M, La6M, So5M, Silent,
		Fa4M, Fa4M, Mi3M, Mi3M, Re2M, Re2M, Do1M, Silent,
		Finish
};

float littleStar_durt[] = {
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1,
		1
};

buzzer_freq_t mechanicus_freq[] = {
		So5M, So5M, So5M, So5M, Fa4M, Mi3M, So5M, Do1H, Re2H,
		Mi3H, Mi3H, Mi3H, Re2H, Do1H, Do1H, Si7M,
		La6M, La6M, La6M, Si7M, Do1H, Si7M, Do1H, La6M,
		So5M, La6M, So5M, Mi3M, So5M, So5M, So5M,
		So5M, So5M, So5M, Fa4M, Mi3M, So5M, Do1H, Re2H,
		Mi3H, Mi3H, Mi3H, Re2H, Do1H, Do1H, Re2H, Re2H, Do1H, Si7M,
		Do1H, Silent, Silent, So5M, Fa4M, Mi3M, So5M, Do1H, Re2H,
		Mi3H, Do1H, Silent, La6M, Si7M, Do1H, Si7M, Do1H, La6M,
		So5M, Mi3M, Silent, So5M, Fa4M, Mi3M, So5M, Do1H, Re2H,
		Mi3H, Do1H, Do1H, Re2H, Re2H, Do1H, Si7M, Do1H, Silent, Silent,
		Finish
};

float mechanicus_durt[] = {
		0.5, 0.25, 0.25, 0.5, 0.25, 0.5, 0.25, 0.5, 0.25,
		0.5, 0.25, 0.5, 0.25, 1, 0.5, 0.25,
		0.5, 0.25, 0.5, 0.25, 0.5, 0.25, 0.5, 0.25,
		0.5, 0.25, 0.5, 0.25, 1, 0.5, 0.25,
		0.5, 0.25, 0.5, 0.25, 0.5, 0.25, 0.5, 0.25,
		0.5, 0.25, 0.5, 0.25, 1, 1, 1, 1, 1, 1,
		2, 0.5, 0.5, 1, 0.5, 0.5, 0.25, 0.5, 0.25,
		2, 1, 0.5, 1, 0.5, 0.5, 0.25, 0.5, 0.25,
		2, 1, 0.5, 1, 0.5, 0.5, 0.25, 0.5, 0.25,
		2, 1, 1, 1, 1, 1, 1, 2, 1, 1,
		1
};
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim12;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM12_Init(void);
/* USER CODE BEGIN PFP */
void buzzer_sing_tone(buzzer_freq_t freq);
void buzzer_sing_song(buzzer_freq_t *freq, float *durt);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_TIM12_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  	//buzzer_sing_song(littleStar_freq, littleStar_durt);
  	buzzer_sing_song(mechanicus_freq, mechanicus_durt);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM12 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM12_Init(void)
{

  /* USER CODE BEGIN TIM12_Init 0 */

  /* USER CODE END TIM12_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM12_Init 1 */

  /* USER CODE END TIM12_Init 1 */
  htim12.Instance = TIM12;
  htim12.Init.Prescaler = 83;
  htim12.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim12.Init.Period = 0;
  htim12.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim12.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim12) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim12, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim12) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim12, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM12_Init 2 */

  /* USER CODE END TIM12_Init 2 */
  HAL_TIM_MspPostInit(&htim12);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */
void buzzer_sing_tone(buzzer_freq_t freq){
	__HAL_TIM_SET_AUTORELOAD(&htim12, 1e6 / freq);
	__HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, 1e6 / ( freq * 2 ));
}

void buzzer_sing_song(buzzer_freq_t *freq, float *durt){
	int i = 0;
	while (freq[i] != Finish) {
		buzzer_sing_tone(freq[i]);
		HAL_Delay(600 * durt[i++]);
	}
	buzzer_sing_tone(Silent);
}
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
