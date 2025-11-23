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
#include "main.h"
#include <stdio.h>
extern volatile uint8_t modelBusy;
extern volatile bool sensingEnabled;
extern volatile uint16_t frame_count;
extern volatile bool recording_done;




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

    if (GPIO_Pin != USER_BUTTON_1_Pin)
        return;

    // 1) 소프트 디바운스 (250ms 이내 재입력 무시)
    if (now - lastTick < 250)
        return;
    lastTick = now;

    // 2) 모델이 아직 돌고 있으면 버튼 무시 (안전장치)
    if (modelBusy) {
        printf("⚠ Model busy, button ignored\r\n");
        return;
    }

    // 3) 상태 토글: OFF→ON = 녹화 시작, ON→OFF = 녹화 종료
    if (!sensingEnabled)
    {
        // ▶ 녹화 시작
        sensingEnabled  = true;
        frame_count     = 0;      // 길이 T 카운트 0에서 시작
        recording_done  = false;  // ★ 새 세션이므로 깔끔하게 리셋

        printf("▶ Recording start\r\n");

        // TIM6 디바운스 안 쓸 거면 아래 두 줄은 지워도 됨
        // __HAL_TIM_SET_COUNTER(&htim6, 0);
        // HAL_TIM_Base_Start_IT(&htim6);
    }
    else
    {
        // ■ 녹화 종료
        sensingEnabled  = false;
        recording_done  = true;   // ★ imu_store가 이걸 보고 모델 호출

        printf("■ Recording stop (%u frames)\r\n", frame_count);

        // TIM6 안 쓸 거면 이 줄도 안 써도 됨
        // HAL_TIM_Base_Stop_IT(&htim6);
    }
}
/* USER CODE END 1 */

/** Configure pins
     PH0-OSC_IN (PH0)   ------> RCC_OSC_IN
     PC3_C   ------> SPI2_MOSI
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
  HAL_GPIO_WritePin(GPIOD, imu_cs3_Pin|imu_cs4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, SPI1_FSYNC_Pin|SPI3_FSYNC_Pin|SPI4_FSYNC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : imu_cs2_Pin imu_cs1_Pin */
  GPIO_InitStruct.Pin = imu_cs2_Pin|imu_cs1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : imu_cs5_Pin */
  GPIO_InitStruct.Pin = imu_cs5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(imu_cs5_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : imu_cs3_Pin imu_cs4_Pin */
  GPIO_InitStruct.Pin = imu_cs3_Pin|imu_cs4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : USER_BUTTON_1_Pin */
  GPIO_InitStruct.Pin = USER_BUTTON_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(USER_BUTTON_1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA11 PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_UART4;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI1_FSYNC_Pin SPI3_FSYNC_Pin SPI4_FSYNC_Pin */
  GPIO_InitStruct.Pin = SPI1_FSYNC_Pin|SPI3_FSYNC_Pin|SPI4_FSYNC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */
