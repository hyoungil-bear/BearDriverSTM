/**
  ******************************************************************************
  * @file    traj.h
  * @author  Bear Robotics Motor Control Team
  * @brief   Trajectory generator for smooth velocity/position control.
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

#ifndef TRAJ_H
#define TRAJ_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <math.h>

/** @addtogroup BearDriver
  * @{
  */

/** @addtogroup Trajectory
  * @{
  */

/* Exported types ------------------------------------------------------------*/

/**
  * @brief  Trajectory generator object structure
  * @note   Generates smooth transitions between target values with
  *         acceleration limiting and first-order tracking
  */
typedef struct
{
  float target_value;     /*!< Target value for the trajectory */
  float int_value;        /*!< Current intermediate value along the trajectory */
  float min_value;        /*!< Minimum value constraint */
  float max_value;        /*!< Maximum value constraint */
  float max_delta;        /*!< Maximum delta (acceleration) per update */
  float current_acc;      /*!< Current acceleration value */
  float error;            /*!< Tracking error (target - intermediate) */
  float K;                /*!< Smoothing tracking gain (0.0 to 1.0) */
  float fs;               /*!< Update frequency (Hz) */
} Traj_Handle_t;

/* Exported constants --------------------------------------------------------*/

/**
  * @brief  Default maximum derivative value for trajectory output
  * @note   This limits the rate of change. Adjust based on application.
  */
#define TRAJ_DEFAULT_MAX_DVALUE  1000.0f

/* Exported macro ------------------------------------------------------------*/

#ifndef MIN
#define MIN(A, B) ((A) < (B) ? (A) : (B))
#endif

#ifndef MAX
#define MAX(A, B) ((A) > (B) ? (A) : (B))
#endif

/* Exported functions prototypes ---------------------------------------------*/

/**
  * @brief  Initialize trajectory generator
  * @param  pTraj: Pointer to trajectory handle
  * @retval None
  */
void Traj_Init(Traj_Handle_t *pTraj);

/**
  * @brief  Run trajectory generator (call periodically)
  * @param  pTraj: Pointer to trajectory handle
  * @retval None
  */
void Traj_Run(Traj_Handle_t *pTraj);

/**
  * @brief  Get intermediate value (current trajectory output)
  * @param  pTraj: Pointer to trajectory handle
  * @retval Current intermediate value
  */
static inline float Traj_GetIntValue(Traj_Handle_t *pTraj)
{
  return pTraj->int_value;
}

/**
  * @brief  Set intermediate value
  * @param  pTraj: Pointer to trajectory handle
  * @param  value: Intermediate value to set
  * @retval None
  */
static inline void Traj_SetIntValue(Traj_Handle_t *pTraj, float value)
{
  pTraj->int_value = value;
}

/**
  * @brief  Get target value
  * @param  pTraj: Pointer to trajectory handle
  * @retval Target value
  */
static inline float Traj_GetTargetValue(Traj_Handle_t *pTraj)
{
  return pTraj->target_value;
}

/**
  * @brief  Set target value
  * @param  pTraj: Pointer to trajectory handle
  * @param  value: Target value to set
  * @retval None
  */
static inline void Traj_SetTargetValue(Traj_Handle_t *pTraj, float value)
{
  pTraj->target_value = value;
}

/**
  * @brief  Get maximum delta (acceleration limit)
  * @param  pTraj: Pointer to trajectory handle
  * @retval Maximum delta value
  */
static inline float Traj_GetMaxDelta(Traj_Handle_t *pTraj)
{
  return pTraj->max_delta;
}

/**
  * @brief  Set maximum delta (acceleration limit)
  * @param  pTraj: Pointer to trajectory handle
  * @param  value: Maximum delta value
  * @retval None
  */
static inline void Traj_SetMaxDelta(Traj_Handle_t *pTraj, float value)
{
  pTraj->max_delta = value;
}

/**
  * @brief  Get minimum value constraint
  * @param  pTraj: Pointer to trajectory handle
  * @retval Minimum value
  */
static inline float Traj_GetMinValue(Traj_Handle_t *pTraj)
{
  return pTraj->min_value;
}

/**
  * @brief  Set minimum value constraint
  * @param  pTraj: Pointer to trajectory handle
  * @param  value: Minimum value
  * @retval None
  */
static inline void Traj_SetMinValue(Traj_Handle_t *pTraj, float value)
{
  pTraj->min_value = value;
}

/**
  * @brief  Get maximum value constraint
  * @param  pTraj: Pointer to trajectory handle
  * @retval Maximum value
  */
static inline float Traj_GetMaxValue(Traj_Handle_t *pTraj)
{
  return pTraj->max_value;
}

/**
  * @brief  Set maximum value constraint
  * @param  pTraj: Pointer to trajectory handle
  * @param  value: Maximum value
  * @retval None
  */
static inline void Traj_SetMaxValue(Traj_Handle_t *pTraj, float value)
{
  pTraj->max_value = value;
}

/**
  * @brief  Get current acceleration
  * @param  pTraj: Pointer to trajectory handle
  * @retval Current acceleration value
  */
static inline float Traj_GetCurrentAcc(Traj_Handle_t *pTraj)
{
  return pTraj->current_acc;
}

/**
  * @brief  Set current acceleration
  * @param  pTraj: Pointer to trajectory handle
  * @param  value: Current acceleration value
  * @retval None
  */
static inline void Traj_SetCurrentAcc(Traj_Handle_t *pTraj, float value)
{
  pTraj->current_acc = value;
}

/**
  * @brief  Get tracking error
  * @param  pTraj: Pointer to trajectory handle
  * @retval Tracking error (target - intermediate)
  */
static inline float Traj_GetError(Traj_Handle_t *pTraj)
{
  return pTraj->error;
}

/**
  * @brief  Set tracking error
  * @param  pTraj: Pointer to trajectory handle
  * @param  value: Error value
  * @retval None
  */
static inline void Traj_SetError(Traj_Handle_t *pTraj, float value)
{
  pTraj->error = value;
}

/**
  * @brief  Get tracking gain K
  * @param  pTraj: Pointer to trajectory handle
  * @retval Tracking gain value (0.0 to 1.0)
  */
static inline float Traj_GetK(Traj_Handle_t *pTraj)
{
  return pTraj->K;
}

/**
  * @brief  Set tracking gain K
  * @param  pTraj: Pointer to trajectory handle
  * @param  value: Tracking gain (0.0 to 1.0, higher = more aggressive)
  * @retval None
  */
static inline void Traj_SetK(Traj_Handle_t *pTraj, float value)
{
  pTraj->K = value;
}

/**
  * @brief  Get update frequency
  * @param  pTraj: Pointer to trajectory handle
  * @retval Update frequency in Hz
  */
static inline float Traj_GetFs(Traj_Handle_t *pTraj)
{
  return pTraj->fs;
}

/**
  * @brief  Set update frequency
  * @param  pTraj: Pointer to trajectory handle
  * @param  value: Update frequency in Hz
  * @retval None
  */
static inline void Traj_SetFs(Traj_Handle_t *pTraj, float value)
{
  pTraj->fs = value;
}

/**
  * @brief  Get derivative value (rate of change)
  * @param  pTraj: Pointer to trajectory handle
  * @param  max_dvalue: Maximum derivative value limit
  * @retval Time derivative of output (clamped)
  */
static inline float Traj_GetDValue(Traj_Handle_t *pTraj, float max_dvalue)
{
  float dValue = pTraj->error * pTraj->fs;
  return MAX(MIN(dValue, max_dvalue), -max_dvalue);
}

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* TRAJ_H */

/******************* (C) COPYRIGHT Bear Robotics *****END OF FILE****/
