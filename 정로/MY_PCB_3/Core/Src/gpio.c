/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
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

/* Includes ------------------------------------------------------------------*/
#include "gpio.h"

/* USER CODE BEGIN 0 */
#include "main.h"
#include "tim.h"




/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */
// main.c 또는 gpio.c 안에 작성

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    static uint32_t lastTick = 0;
    uint32_t now = HAL_GetTick();

    // ★ 버튼 핀: PG4 → PD14 → 이제 PB13
    if (GPIO_Pin == USER_BUTTON_1_Pin)   // == GPIO_PIN_13
    {
        // 250ms 이내 재진입 방지
        if (now - lastTick < 250)
            return;
        lastTick = now;

        sensingEnabled = !sensingEnabled;

        if (sensingEnabled)
        {
            times++;
            action++;
        }
        __HAL_TIM_SET_COUNTER(&htim6, 0);
        HAL_TIM_Base_Start_IT(&htim6);
    }
}
/* USER CODE END 1 */

/** Configure pins
     PH0-OSC_IN (PH0)   ------> RCC_OSC_IN
     PA11   ------> UART4_RX
     PA12   ------> UART4_TX
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, imu_cs2_Pin|imu_cs1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(imu_cs5_GPIO_Port, imu_cs5_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, imu_cs3_Pin|imu_cs4_Pin|SPI1_FSYNC_Pin|SPI3_FSYNC_Pin
                          |SPI4_FSYNC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : imu_cs2_Pin imu_cs1_Pin */
  GPIO_InitStruct.Pin = imu_cs2_Pin|imu_cs1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : imu_cs5_Pin */
  GPIO_InitStruct.Pin = imu_cs5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(imu_cs5_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USER_BUTTON_1_Pin */
  GPIO_InitStruct.Pin = USER_BUTTON_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(USER_BUTTON_1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : imu_cs3_Pin imu_cs4_Pin SPI1_FSYNC_Pin SPI3_FSYNC_Pin
                           SPI4_FSYNC_Pin */
  GPIO_InitStruct.Pin = imu_cs3_Pin|imu_cs4_Pin|SPI1_FSYNC_Pin|SPI3_FSYNC_Pin
                          |SPI4_FSYNC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : PA11 PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_UART4;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */
