/**
  ******************************************************************************
  * @file    user_motor.cpp
  * @author  Bear Robotics Motor Control Team
  * @brief   User motor parameter initialization
  ******************************************************************************
  * @attention
  *
  * This file provides USER_Params initialization (USER_setParamsMtr) and
  * basic parameter validation (USER_checkForErrors).
  * Adapted from TI BearDriver for STM32G474.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "user_params.h"
#include <string.h>

// **************************************************************************
// USER_Params initialization (matches original TI naming)

void USER_setParamsMtr(USER_Params *p, int16_t hwVersion) {
  if (p == NULL) return;

  memset(p, 0, sizeof(USER_Params));

  /* Hardware + Motor identification */
  p->hwVersion          = hwVersion;
  p->motor_numPolePairs = USER_MOTOR_NUM_POLE_PAIRS;

  /* Motor electrical parameters */
  p->motor_Rs        = USER_MOTOR_Rs;
  p->motor_Ls_d      = USER_MOTOR_Ls_d;
  p->motor_Ls_q      = USER_MOTOR_Ls_q;
  p->torqueConstant  = USER_MOTOR_TORQUE_CONSTANT;

  /* Current limit */
  p->maxCurrent         = USER_MOTOR_MAX_CURRENT;

  /* Speed PID defaults [SI units]
   *   Kp  [A/RPM]      0.02 ~ 0.08(default) ~ 0.2
   *   Ki  [A/(RPM·s)]  0.5  ~ 2.0 (default) ~ 5.0     (Kis = Ki×Ts)
   *   Kd  [A·s/RPM]    0.0002 ~ 0.0008(default) ~ 0.002  (Kds = Kd/Ts)
   *   Kff [A/(RPM/tick)] 0.0 ~ 0.1 (default) ~ 0.5
   *   dN  [rad/s]       50(heavy) ~ 200(default) ~ 500(light)  */
  p->spdParams.Kp   = 0.08f;
  p->spdParams.Ki   = 2.0f;
  p->spdParams.Kd   = 0.0008f;
  p->spdParams.Kff  = 0.1f;
  p->spdParams.dN   = 200.0f;
  p->spdParams.Kout = 1.0f;

  /* PWM / Control timing */
  p->numPwmTicksPerIsrTick   = USER_NUM_PWM_TICKS_PER_ISR_TICK;
  p->numCtrlTicksPerSpeedTick = USER_NUM_CTRL_TICKS_PER_SPEED_TICK;
  p->pwmPeriod_usec          = USER_PWM_PERIOD_usec;
  p->ctrlFreq_Hz             = USER_CTRL_FREQ_Hz;
  p->ctrlPeriod_sec          = USER_CTRL_PERIOD_sec;
  p->maxVsMag_pu             = USER_MAX_VS_MAG_PU;

  /* Filter poles */
  p->offsetPole_rps        = USER_OFFSET_POLE_rps;
  p->voltageFilterPole_rps = USER_VOLTAGE_FILTER_POLE_rps;
}

// **************************************************************************
// Parameter validation for encoder FOC
// Returns first error found (TI used last-error-wins; first-error is more useful)
// On error, caller should halt the system — wrong params can damage hardware.

USER_ErrorCode_e USER_checkForErrors(const USER_Params *p) {
  if (p == NULL) return USER_ErrorCode_maxCurrent_Invalid;

  // Motor identification
  if (p->motor_numPolePairs == 0)
    return USER_ErrorCode_polePairs_Invalid;

  // Motor electrical — negative values indicate corrupted data
  if (p->motor_Rs < 0.0f)
    return USER_ErrorCode_motorRs_Invalid;
  if ((p->motor_Ls_d < 0.0f) || (p->motor_Ls_q < 0.0f))
    return USER_ErrorCode_motorLs_Invalid;

  // Current limit
  if (p->maxCurrent <= 0.0f)
    return USER_ErrorCode_maxCurrent_Invalid;

  // Control timing — zero/negative frequencies cause division by zero
  if (p->ctrlFreq_Hz <= 0.0f)
    return USER_ErrorCode_ctrlFreq_Invalid;
  if (p->pwmPeriod_usec <= 0.0f)
    return USER_ErrorCode_pwmPeriod_Invalid;

  // Voltage limit — must be within SVPWM linear range
  if ((p->maxVsMag_pu <= 0.0f) || (p->maxVsMag_pu > 1.0f))
    return USER_ErrorCode_maxVsMag_Invalid;

  // Filter poles — zero pole causes filter to never converge
  if (p->offsetPole_rps <= 0.0f)
    return USER_ErrorCode_offsetPole_Invalid;
  if (p->voltageFilterPole_rps <= 0.0f)
    return USER_ErrorCode_voltageFilterPole_Invalid;

  return USER_ErrorCode_NoError;
}

/******************* (C) COPYRIGHT Bear Robotics *****END OF FILE****/
