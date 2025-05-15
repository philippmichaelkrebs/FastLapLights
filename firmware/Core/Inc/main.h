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
#define PWM_START_LIGHTS_Pin GPIO_PIN_0
#define PWM_START_LIGHTS_GPIO_Port GPIOA
#define PWM_PIT_LANE_Pin GPIO_PIN_1
#define PWM_PIT_LANE_GPIO_Port GPIOA
#define IO_POWER_RAIL_INT_Pin GPIO_PIN_3
#define IO_POWER_RAIL_INT_GPIO_Port GPIOA
#define TRACK_LIGHTS_G_Pin GPIO_PIN_6
#define TRACK_LIGHTS_G_GPIO_Port GPIOA
#define TRACK_LIGHTS_R_Pin GPIO_PIN_7
#define TRACK_LIGHTS_R_GPIO_Port GPIOA
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
