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
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>   // bool 정의 포함
extern volatile bool sensingEnabled;   // 선언 (어디선가 정의돼 있다 알림)
extern volatile uint8_t times;
extern volatile uint8_t action;
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
#define imu_cs2_Pin GPIO_PIN_3
#define imu_cs2_GPIO_Port GPIOA
#define imu_cs1_Pin GPIO_PIN_4
#define imu_cs1_GPIO_Port GPIOA
#define imu_cs5_Pin GPIO_PIN_12
#define imu_cs5_GPIO_Port GPIOF
#define imu_cs3_Pin GPIO_PIN_12
#define imu_cs3_GPIO_Port GPIOD
#define imu_cs4_Pin GPIO_PIN_13
#define imu_cs4_GPIO_Port GPIOD
#define USER_BUTTON_1_Pin GPIO_PIN_14
#define USER_BUTTON_1_GPIO_Port GPIOD
#define USER_BUTTON_1_EXTI_IRQn EXTI15_10_IRQn
#define SPI1_FSYNC_Pin GPIO_PIN_12
#define SPI1_FSYNC_GPIO_Port GPIOG
#define SPI3_FSYNC_Pin GPIO_PIN_14
#define SPI3_FSYNC_GPIO_Port GPIOG
#define SPI4_FSYNC_Pin GPIO_PIN_15
#define SPI4_FSYNC_GPIO_Port GPIOG

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
