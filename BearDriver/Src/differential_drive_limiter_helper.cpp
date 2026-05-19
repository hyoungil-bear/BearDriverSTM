/**
  ******************************************************************************
  * @file    differential_drive_limiter_helper.cpp
  * @author  Bear Robotics Motor Control Team
  * @brief   Differential drive velocity limiter implementation
  ******************************************************************************
  * @attention
  *
  * Implements kinematic constraints for differential drive robots.
  * Limits wheel velocities based on maximum linear and angular velocities.
  *
  * Algorithm:
  * 1. Convert wheel velocities (KRPM) to robot velocities (v, omega)
  * 2. Check if linear or angular limits are exceeded
  * 3. If exceeded, scale down both wheel velocities proportionally
  * 4. Return scaled velocities
  *
  * Adapted from TI BearDriver for STM32G474.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "differential_drive_limiter_helper.h"
#include <math.h>
#include <float.h>

/** @addtogroup BearDriver
  * @{
  */

/** @addtogroup DifferentialDriveLimiter
  * @{
  */

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Global command limiter instance */
DifferentialDriveLimiter_t gCmdLimiter = {
  .wheel_base_radius_krpm_adjusted = FLT_MAX,
  .linear_limit_krpm = 0.0f,
  .angular_limit_rad_s = 0.0f
};

/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  Initialize differential drive limiter
  * @param  limiter: Pointer to limiter structure
  * @param  params: Robot kinematic parameters
  * @retval None
  */
void DifferentialDriveLimiter_Init(DifferentialDriveLimiter_t *limiter,
                                   const DifferentialDriveLimiter_Params_t *params)
{
  if (limiter == NULL || params == NULL) {
    return;
  }

  /* Convert linear limit from m/s to KRPM
   * v [m/s] = omega [rad/s] * r [m]
   * omega [rad/s] = v [m/s] / r [m]
   * RPM = omega [rad/s] * 60 / (2*pi)
   * KRPM = RPM / 1000
   *
   * linear_limit_krpm = linear_limit_m_s * 60 / (2 * pi * wheel_radius_m * 1000)
   */
  float linear_limit_krpm = params->linear_limit_m_s * 60.0f /
                            (2.0f * M_PI * params->wheel_radius_m * 1000.0f);

  /* Precompute wheelbase radius in KRPM units for internal calculations
   * This avoids repeated conversions involving wheel radius
   *
   * wheel_base_radius_krpm_adjusted = wheel_base_radius_m * 60 /
   *                                    (2 * pi * wheel_radius_m * 1000)
   */
  float wheel_base_radius_krpm_adjusted = params->wheel_base_radius_m * 60.0f /
                                          (2.0f * M_PI * params->wheel_radius_m * 1000.0f);

  /* Store computed parameters */
  limiter->wheel_base_radius_krpm_adjusted = wheel_base_radius_krpm_adjusted;
  limiter->linear_limit_krpm = linear_limit_krpm;
  limiter->angular_limit_rad_s = params->angular_limit_rad_s;
}

/**
  * @brief  Set command limiter parameters (convenience wrapper for gCmdLimiter)
  * @param  params: Robot kinematic parameters
  * @retval None
  */
void set_cmd_limiter(const DifferentialDriveLimiter_Params_t *params)
{
  DifferentialDriveLimiter_Init(&gCmdLimiter, params);
}

/**
  * @brief  Limit wheel velocity command based on kinematic constraints
  * @param  limiter: Pointer to limiter structure
  * @param  requested: Requested wheel velocities (KRPM)
  * @param  limited: Output limited wheel velocities (KRPM)
  * @retval None
  *
  * @note   Differential drive kinematics:
  *         v = (v_left + v_right) / 2      (linear velocity)
  *         omega = (v_right - v_left) / (2*L)  (angular velocity, L = wheelbase radius)
  *
  *         With KRPM units and precomputed adjustments:
  *         v_krpm = (v_left + v_right) / 2
  *         omega = (v_right - v_left) / (2 * wheel_base_radius_krpm_adjusted)
  */
void DifferentialDriveLimiter_Limit(const DifferentialDriveLimiter_t *limiter,
                                    const DifferentialDriveLimiter_Command_t *requested,
                                    DifferentialDriveLimiter_Command_t *limited)
{
  if (limiter == NULL || requested == NULL || limited == NULL) {
    return;
  }

  /* Default output is requested input (no limiting) */
  limited->left = requested->left;
  limited->right = requested->right;

  /* Calculate robot linear velocity (KRPM) */
  float v_krpm = (requested->left + requested->right) * 0.5f;

  /* Calculate robot angular velocity (rad/s)
   * omega = (v_right - v_left) / (2 * L)
   * where L is wheelbase radius in KRPM-adjusted units
   */
  float omega_rad_s = (requested->right - requested->left) /
                      (2.0f * limiter->wheel_base_radius_krpm_adjusted);

  /* Check linear velocity limit */
  float linear_scale = 1.0f;
  if (fabsf(v_krpm) > limiter->linear_limit_krpm && limiter->linear_limit_krpm > 0.0f) {
    linear_scale = limiter->linear_limit_krpm / fabsf(v_krpm);
  }

  /* Check angular velocity limit */
  float angular_scale = 1.0f;
  if (fabsf(omega_rad_s) > limiter->angular_limit_rad_s && limiter->angular_limit_rad_s > 0.0f) {
    angular_scale = limiter->angular_limit_rad_s / fabsf(omega_rad_s);
  }

  /* Use the most restrictive scale factor */
  float scale = (linear_scale < angular_scale) ? linear_scale : angular_scale;

  /* Apply scaling if limiting is needed */
  if (scale < 1.0f) {
    limited->left = requested->left * scale;
    limited->right = requested->right * scale;
  }
}

/**
  * @}
  */

/**
  * @}
  */

/******************* (C) COPYRIGHT Bear Robotics *****END OF FILE****/
