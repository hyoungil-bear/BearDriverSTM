/**
  ******************************************************************************
  * @file    traj.c
  * @author  Bear Robotics Motor Control Team
  * @brief   Trajectory generator implementation.
  ******************************************************************************
  * @attention
  *
  * This file implements trajectory generation adapted from TI InstaSPIN-FOC
  * for STM32G474. Provides smooth transitions between target values with
  * configurable acceleration limits and tracking gains.
  *
  * Original TI Copyright (c) 2012, Texas Instruments Incorporated
  * Adapted for STM32 by Bear Robotics
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "traj.h"
#include <string.h>

/** @addtogroup BearDriver
  * @{
  */

/** @addtogroup Trajectory
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Initialize trajectory generator
  * @param  pTraj: Pointer to trajectory handle
  * @retval None
  */
void Traj_Init(Traj_Handle_t *pTraj)
{
  if (pTraj == NULL) return;

  /* Clear structure */
  memset(pTraj, 0, sizeof(Traj_Handle_t));

  /* Set default values */
  pTraj->target_value = 0.0f;
  pTraj->int_value = 0.0f;
  pTraj->min_value = -1.0f;
  pTraj->max_value = 1.0f;
  pTraj->max_delta = 0.01f;
  pTraj->current_acc = 0.0f;
  pTraj->error = 0.0f;
  pTraj->K = 1.0f;  /* Unity gain (no smoothing) */
  pTraj->fs = 1000.0f;  /* 1 kHz default update rate */
}

/**
  * @brief  Run trajectory generator
  * @note   This function should be called periodically at the rate specified
  *         by the fs parameter. It generates smooth transitions from the
  *         current intermediate value toward the target value with
  *         acceleration limiting.
  *
  * @param  pTraj: Pointer to trajectory handle
  * @retval None
  */
void Traj_Run(Traj_Handle_t *pTraj)
{
  if (pTraj == NULL) return;

  /* Get current values */
  float target_value = pTraj->target_value;
  float int_value = pTraj->int_value;
  float K = pTraj->K;
  float max_delta = pTraj->max_delta;
  float min_value = pTraj->min_value;
  float max_value = pTraj->max_value;

  /* Calculate error with tracking gain */
  float error = (target_value - int_value) * K;

  /* Limit acceleration (delta) */
  float current_acc = MAX(MIN(error, max_delta), -max_delta);

  /* Update intermediate value */
  int_value += current_acc;

  /* Apply value constraints */
  int_value = MAX(MIN(int_value, max_value), min_value);

  /* Store updated values */
  pTraj->int_value = int_value;
  pTraj->current_acc = current_acc;
  pTraj->error = error;
}

/**
  * @}
  */

/**
  * @}
  */

/******************* (C) COPYRIGHT Bear Robotics *****END OF FILE****/
