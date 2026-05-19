/**
  ******************************************************************************
  * @file    differential_drive_limiter_helper.h
  * @author  Bear Robotics Motor Control Team
  * @brief   Differential drive velocity limiter for robot kinematic constraints
  ******************************************************************************
  * @attention
  *
  * This module limits commanded wheel velocities based on robot kinematic
  * parameters (wheel radius, wheelbase) and maximum linear/angular velocity
  * constraints. Prevents commands that would exceed physical limits.
  *
  * Adapted from TI BearDriver for STM32G474.
  *
  ******************************************************************************
  */

#ifndef DIFFERENTIAL_DRIVE_LIMITER_HELPER_H
#define DIFFERENTIAL_DRIVE_LIMITER_HELPER_H

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @addtogroup BearDriver
  * @{
  */

/** @addtogroup DifferentialDriveLimiter
  * @{
  */

/* Exported types ------------------------------------------------------------*/

/**
  * @brief  Differential drive robot parameters
  */
typedef struct
{
  float wheel_radius_m;         /*!< Wheel radius (meters) */
  float wheel_base_radius_m;    /*!< Half of wheelbase distance (meters) */
  float linear_limit_m_s;       /*!< Maximum linear velocity (m/s) */
  float angular_limit_rad_s;    /*!< Maximum angular velocity (rad/s) */
} DifferentialDriveLimiter_Params_t;

/**
  * @brief  Wheel velocity command (KRPM)
  */
typedef struct
{
  float left;   /*!< Left wheel velocity (KRPM) */
  float right;  /*!< Right wheel velocity (KRPM) */
} DifferentialDriveLimiter_Command_t;

/**
  * @brief  Differential drive limiter instance
  */
typedef struct
{
  float wheel_base_radius_krpm_adjusted;  /*!< Precomputed wheelbase in KRPM units */
  float linear_limit_krpm;                /*!< Maximum linear velocity (KRPM) */
  float angular_limit_rad_s;              /*!< Maximum angular velocity (rad/s) */
} DifferentialDriveLimiter_t;

/* Exported constants --------------------------------------------------------*/

/* Default robot parameters (smallest radius robot for safety) */
#define DEFAULT_WHEEL_RADIUS_M          0.089f      /*!< 89mm wheel radius */
#define DEFAULT_WHEEL_BASE_RADIUS_M     0.3784f     /*!< 378.4mm wheelbase radius */
#define DEFAULT_LINEAR_LIMIT_M_S        1.4f        /*!< 1.4 m/s max linear velocity */
#define DEFAULT_ANGULAR_LIMIT_RAD_S     1.2f        /*!< 1.2 rad/s max angular velocity */

/* Exported macro ------------------------------------------------------------*/

/* Exported variables --------------------------------------------------------*/

extern DifferentialDriveLimiter_t gCmdLimiter;  /*!< Global command limiter instance */

/* Exported functions prototypes ---------------------------------------------*/

/**
  * @brief  Initialize differential drive limiter
  * @param  limiter: Pointer to limiter structure
  * @param  params: Robot kinematic parameters
  * @retval None
  */
void DifferentialDriveLimiter_Init(DifferentialDriveLimiter_t *limiter,
                                   const DifferentialDriveLimiter_Params_t *params);

/**
  * @brief  Set command limiter parameters (convenience wrapper for gCmdLimiter)
  * @param  params: Robot kinematic parameters
  * @retval None
  */
void set_cmd_limiter(const DifferentialDriveLimiter_Params_t *params);

/**
  * @brief  Limit wheel velocity command based on kinematic constraints
  * @param  limiter: Pointer to limiter structure
  * @param  requested: Requested wheel velocities (KRPM)
  * @param  limited: Output limited wheel velocities (KRPM)
  * @retval None
  */
void DifferentialDriveLimiter_Limit(const DifferentialDriveLimiter_t *limiter,
                                    const DifferentialDriveLimiter_Command_t *requested,
                                    DifferentialDriveLimiter_Command_t *limited);

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* DIFFERENTIAL_DRIVE_LIMITER_HELPER_H */

/******************* (C) COPYRIGHT Bear Robotics *****END OF FILE****/
