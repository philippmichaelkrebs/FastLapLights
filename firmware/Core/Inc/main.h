/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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
#include "stm32c0xx_hal.h"

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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define START_LIGHT_ALIGNMENT_Pin GPIO_PIN_1
#define START_LIGHT_ALIGNMENT_GPIO_Port GPIOA
#define RACE_TRAINING_SESSION_SW_Pin GPIO_PIN_3
#define RACE_TRAINING_SESSION_SW_GPIO_Port GPIOA
#define TRACK_GREEN_Pin GPIO_PIN_6
#define TRACK_GREEN_GPIO_Port GPIOA
#define TRACK_RED_PWM_Pin GPIO_PIN_7
#define TRACK_RED_PWM_GPIO_Port GPIOA
#define IO_POWER_RAIL_DATA_FALL_Pin GPIO_PIN_8
#define IO_POWER_RAIL_DATA_FALL_GPIO_Port GPIOA
#define LIGHT_MODE_SJ_Pin GPIO_PIN_12
#define LIGHT_MODE_SJ_GPIO_Port GPIOA
#define IO_POWER_RAIL_DATA_RISE_Pin GPIO_PIN_6
#define IO_POWER_RAIL_DATA_RISE_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
