/**
  ******************************************************************************
  * @file    pid.cpp
  * @author  Bear Robotics Motor Control Team
  * @brief   PID controller implementation
  ******************************************************************************
  * @attention
  *
  * This file implements PID controller with anti-windup and derivative
  * filtering for motor control applications.
  * Adapted from TI BearDriver for STM32G474.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "pid.h"
#include <string.h>

/* Private function prototypes -----------------------------------------------*/
static float Filter_Run(PID_Filter_t *filter, float x);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Run first-order filter
  * @note   y[n] = b0*x[n] + b1*x[n-1] - a1*y[n-1]
  */
static float Filter_Run(PID_Filter_t *filter, float x)
{
  float y = filter->b0 * x + filter->b1 * filter->x1 - filter->a1 * filter->y1;
  filter->x1 = x;
  filter->y1 = y;
  return y;
}

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  Initialize PID controller
  * @note   Matches original TI PID_init(void*, size_t) signature
  */
PID_Handle PID_init(void *pMemory, const size_t numBytes)
{
  PID_Handle handle;

  if (numBytes < sizeof(PID_Obj)) {
    return (PID_Handle)NULL;
  }

  handle = (PID_Handle)pMemory;

  /* Set defaults */
  PID_setUi(handle, 0.0f);
  PID_setRefValue(handle, 0.0f);
  PID_setFbackValue(handle, 0.0f);
  PID_setLastError(handle, 0.0f);
  handle->I_windup = 0.0f;

  handle->Kp = 0.0f;
  handle->Ki = 0.0f;
  handle->Kd = 0.0f;
  handle->Kff = 0.0f;
  handle->dN = 0.0f;
  handle->K = 0.0f;
  handle->Kout = 1.0f;

  handle->Kis = 0.0f;
  handle->Kds = 0.0f;

  handle->Ud = 0.0f;

  handle->outMin = -1.0f;
  handle->outMax = 1.0f;
  handle->UiOutMin = -1.0f;
  handle->UiOutMax = 1.0f;
  handle->UiSatFlag = false;
  handle->UdOutMax = 0.0f;  /* 0 = disabled (backward compatible) */

  handle->ts = 0.0001f;  /* 100 us default */

  /* Initialize derivative filter */
  handle->filter.b0 = 0.0f;
  handle->filter.a1 = 0.0f;
  handle->filter.b1 = 0.0f;
  handle->filter.x1 = 0.0f;
  handle->filter.y1 = 0.0f;

  return handle;
}

/**
  * @brief  Run standard PID controller (P + I with saturation)
  * @note   Matches original TI PID_run
  */
void PID_run(PID_Handle handle, const float refValue,
             const float fbackValue, float *pOutValue)
{
  float Error;
  float Up, Ui;

  Error = refValue - fbackValue;

  Ui = handle->Ui;
  Up = handle->Kp * Error;
  Ui = PID_SAT(Ui + handle->Ki * Up, handle->UiOutMax, handle->UiOutMin);

  if ((Ui >= handle->UiOutMax) || (Ui <= handle->UiOutMin)) {
    handle->UiSatFlag = true;
  } else {
    handle->UiSatFlag = false;
  }

  handle->Ui = Ui;
  handle->refValue = refValue;
  handle->fbackValue = fbackValue;

  *pOutValue = PID_SAT(Up + Ui, handle->outMax, handle->outMin);
}

/**
  * @brief  Run PID controller for speed (with feedforward and derivative)
  * @note   Matches original TI PID_run_spd
  */
void PID_run_spd(PID_Handle handle, const float refValue,
                 const float fbackValue, float ff, float *pOutValue)
{
  float Error;
  float Up, Ui, Ud, Uff;

  /* calc error */
  Error = refValue - fbackValue;

  /* calc P term */
  Up = handle->Kp * Error;

  /* calc I term */
  Ui = handle->Ui;
  Ui = PID_SAT(handle->I_windup + Ui + handle->Kis * Error,
               handle->UiOutMax, handle->UiOutMin);

  if ((Ui >= handle->UiOutMax) || (Ui <= handle->UiOutMin)) {
    handle->UiSatFlag = true;
  } else {
    handle->UiSatFlag = false;
  }

  /* calc D term */
  Ud = 0.0f;
  float d = handle->Kds * (Error - handle->lastError);

  /* Input clamp: prevents filter state from diverging on large error spikes */
  if (handle->UdOutMax > 0.0f) {
    d = PID_SAT(d, handle->UdOutMax, -handle->UdOutMax);
  }

  if (handle->dN != 0.0f) {
    Ud = Filter_Run(&handle->filter, d);
  } else {
    Ud = d;
  }

  /* Output clamp: hard limit on D-term contribution to total output */
  if (handle->UdOutMax > 0.0f) {
    Ud = PID_SAT(Ud, handle->UdOutMax, -handle->UdOutMax);
  }

  /* calc FF term */
  Uff = handle->Kff * ff;

  /* save state */
  handle->Ui = Ui;
  handle->Ud = Ud;
  handle->refValue = refValue;
  handle->fbackValue = fbackValue;
  handle->lastError = Error;

  /* compute total output with output gain */
  float g = (Up + Ui + Ud + Uff) * handle->Kout;

  /* Saturate the output */
  *pOutValue = PID_SAT(g, handle->outMax, handle->outMin);

  /* Anti-windup backtracking */
  handle->I_windup = 0.0f;  // backtracking disabled — (*pOutValue - g) caused oscillation
}

/**
  * @brief  Run PID controller with setpoint weighting for speed
  * @note   Matches original TI PID_run_spd_spw
  */
void PID_run_spd_spw(PID_Handle handle, const float refValue,
                     const float fbackValue, const float b,
                     const float c, float *pOutValue)
{
  float Error_P;
  float Error;
  float Up, Ui, Uff;

  Error_P = b * refValue - fbackValue;
  Error = refValue - fbackValue;

  Uff = refValue * handle->Kff;

  Ui = handle->Ui;
  Up = handle->Kp * Error_P;

  Ui = PID_SAT(handle->I_windup + Ui + handle->Kis * Error,
               handle->outMax, handle->outMin);

  handle->Ui = Ui;
  handle->refValue = refValue;
  handle->fbackValue = fbackValue;

  float x = Up + Ui + Uff;

  *pOutValue = PID_SAT(x, handle->outMax, handle->outMin);

  handle->I_windup = *pOutValue - x;
}

/******************* (C) COPYRIGHT Bear Robotics *****END OF FILE****/
