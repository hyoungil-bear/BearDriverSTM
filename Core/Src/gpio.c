/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
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
#include "gpio.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(do_LED_Err_GPIO_Port, do_LED_Err_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, do_LED_Run_Pin|do_nSTBY_1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(do_nSTBY_2_GPIO_Port, do_nSTBY_2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : do_LED_Err_Pin */
  GPIO_InitStruct.Pin = do_LED_Err_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(do_LED_Err_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : do_LED_Run_Pin do_nSTBY_1_Pin */
  GPIO_InitStruct.Pin = do_LED_Run_Pin|do_nSTBY_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : di_FLAG_1_Pin */
  GPIO_InitStruct.Pin = di_FLAG_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(di_FLAG_1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : di_HALLA_1_Pin di_HALLB_1_Pin di_HALLC_1_Pin di_rev_B0_Pin
                           di_rev_B1_Pin di_rev_B2_Pin */
  GPIO_InitStruct.Pin = di_HALLA_1_Pin|di_HALLB_1_Pin|di_HALLC_1_Pin|di_rev_B0_Pin
                          |di_rev_B1_Pin|di_rev_B2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : di_nESTOP_IN_Pin di_HALLC_2_Pin di_HALLB_2_Pin di_HALLA_2_Pin */
  GPIO_InitStruct.Pin = di_nESTOP_IN_Pin|di_HALLC_2_Pin|di_HALLB_2_Pin|di_HALLA_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : do_nSTBY_2_Pin */
  GPIO_InitStruct.Pin = do_nSTBY_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(do_nSTBY_2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : di_FLAG_2_Pin */
  GPIO_InitStruct.Pin = di_FLAG_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(di_FLAG_2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : di_rev_B3_Pin */
  GPIO_InitStruct.Pin = di_rev_B3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(di_rev_B3_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */
