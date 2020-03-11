/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LD8_Pin GPIO_PIN_8
#define LD8_GPIO_Port GPIOG
#define LD7_Pin GPIO_PIN_7
#define LD7_GPIO_Port GPIOG
#define LD6_Pin GPIO_PIN_6
#define LD6_GPIO_Port GPIOG
#define LD5_Pin GPIO_PIN_5
#define LD5_GPIO_Port GPIOG
#define LD4_Pin GPIO_PIN_4
#define LD4_GPIO_Port GPIOG
#define LD3_Pin GPIO_PIN_3
#define LD3_GPIO_Port GPIOG
#define LD2_Pin GPIO_PIN_2
#define LD2_GPIO_Port GPIOG
#define Button_Pin GPIO_PIN_2
#define Button_GPIO_Port GPIOB
#define LD1_Pin GPIO_PIN_1
#define LD1_GPIO_Port GPIOG
#define LD_RED_Pin GPIO_PIN_11
#define LD_RED_GPIO_Port GPIOE
#define LD_GREEN_Pin GPIO_PIN_14
#define LD_GREEN_GPIO_Port GPIOF
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
