/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    iwdg.c
  * @brief   This file provides code for the configuration
  *          of the IWDG instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "iwdg.h"

/* USER CODE BEGIN 0 */
#include <stdbool.h>
#include "stm32g4xx_hal_iwdg.h"
/* USER CODE END 0 */

IWDG_HandleTypeDef hiwdg;

/* IWDG init function */
void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */
  /* Deferred-start pattern — matches TI HAL_setupWatchdog() approach:
   *   1st call: main.c peripheral init → store params only, return early.
   *      IWDG does NOT start, so HAL_Delay(1000) and flash init can complete.
   *   2nd call: BearDriver_Main() just before for(;;) → falls through, starts IWDG.
   * CubeMX regeneration preserves this USER CODE block. */
  static bool s_iwdg_deferred = true;
  if (s_iwdg_deferred) {
    s_iwdg_deferred         = false;
    hiwdg.Instance          = IWDG;
    hiwdg.Init.Prescaler    = IWDG_PRESCALER_4;  /* Ti = 512 ms */
    hiwdg.Init.Window       = 4095;
    hiwdg.Init.Reload       = 4095;
    return;  /* skip HAL_IWDG_Init — IWDG not started yet */
  }
  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_4;
  hiwdg.Init.Window = 4095;
  hiwdg.Init.Reload = 4095;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */
  /* IWDG_PRESCALER_4: tick = LSI(32 kHz) / 4 = 8 kHz (0.125 ms/tick)
   * Ti = (4095+1) * 0.125 ms = 512 ms, Window mode disabled */
  /* USER CODE END IWDG_Init 2 */

}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
