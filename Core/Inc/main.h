/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "stm32f7xx_hal.h"

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
#define X_DIR_Pin GPIO_PIN_0
#define X_DIR_GPIO_Port GPIOC
#define Y_DIR_Pin GPIO_PIN_1
#define Y_DIR_GPIO_Port GPIOC
#define X_CLK_Pin GPIO_PIN_0
#define X_CLK_GPIO_Port GPIOA
#define Y_CLK_Pin GPIO_PIN_1
#define Y_CLK_GPIO_Port GPIOA
#define XYZ_ENABLE_Pin GPIO_PIN_4
#define XYZ_ENABLE_GPIO_Port GPIOA
#define BCD_ENABLE_Pin GPIO_PIN_5
#define BCD_ENABLE_GPIO_Port GPIOA
#define NOZZLE_CLK_Pin GPIO_PIN_6
#define NOZZLE_CLK_GPIO_Port GPIOA
#define NOZZLE_DIR_Pin GPIO_PIN_13
#define NOZZLE_DIR_GPIO_Port GPIOF
#define X_A_Pin GPIO_PIN_14
#define X_A_GPIO_Port GPIOE
#define X_B_Pin GPIO_PIN_15
#define X_B_GPIO_Port GPIOE
#define Y_A_Pin GPIO_PIN_12
#define Y_A_GPIO_Port GPIOB
#define Y_B_Pin GPIO_PIN_13
#define Y_B_GPIO_Port GPIOB
#define X_Lim_Pin GPIO_PIN_10
#define X_Lim_GPIO_Port GPIOC
#define Y_Lim_Pin GPIO_PIN_11
#define Y_Lim_GPIO_Port GPIOC
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
