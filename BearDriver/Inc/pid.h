/**
  ******************************************************************************
  * @file    pid.h
  * @author  Bear Robotics Motor Control Team
  * @brief   PID controller with anti-windup and derivative filtering
  ******************************************************************************
  * @attention
  *
  * This file provides PID controller implementation for motor control.
  * Adapted from TI BearDriver for STM32G474 with float math.
  * Original: Texas Instruments PID module (pid.h)
  *
  * Changes from TI original:
  *   - _iq fixed-point → float (STM32 Cortex-M4 FPU)
  *   - FILTER_FO module → inline first-order filter
  *   - Function implementations in pid.cpp (not inline)
  *
  ******************************************************************************
  */

#ifndef _PID_H_
#define _PID_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

/* Exported types ------------------------------------------------------------*/

/**
  * @brief  First-order filter for derivative filtering
  * @note   Replaces TI FILTER_FO module
  */
typedef struct
{
  float x1;      /*!< Previous input x[n-1] */
  float y1;      /*!< Previous output y[n-1] */
  float a1;      /*!< Denominator coefficient (recursive) */
  float b0;      /*!< Numerator coefficient for x[n] */
  float b1;      /*!< Numerator coefficient for x[n-1] */
} PID_Filter_t;

/**
  * @brief  PID controller object
  * @note   Matches original TI PID_Obj structure with float types
  */
typedef struct _PID_Obj_
{
  float Kp;            /*!< Proportional gain */
  float Ki;            /*!< Integral gain */
  float Kd;            /*!< Derivative gain */
  float Kff;           /*!< Feedforward gain */
  float dN;            /*!< Derivative filter pole (rad/s) */
  float K;             /*!< Smoothing tracking constant for anti-windup */
  float Kout;          /*!< Output loop gain */

  float Ui;            /*!< Integrator state */
  float Ud;            /*!< Derivative state (filtered) */

  float refValue;      /*!< Reference input value */
  float fbackValue;    /*!< Feedback input value */

  float lastError;     /*!< Previous error for derivative calculation */
  float outMin;        /*!< Minimum output limit */
  float outMax;        /*!< Maximum output limit */

  float UiOutMin;      /*!< Minimum integrator output */
  float UiOutMax;      /*!< Maximum integrator output */

  bool UiSatFlag;      /*!< Integrator saturation flag */

  float UdOutMax;      /*!< D-term output limit (0 = disabled) */

  float Kis;           /*!< Ki * ts (scaled integral gain) */
  float Kds;           /*!< Kd / ts (scaled derivative gain) */

  float I_windup;      /*!< Windup backtracking value for I term */

  float ts;            /*!< Sampling time (seconds) */

  PID_Filter_t filter; /*!< First-order derivative filter */

} PID_Obj;

/**
  * @brief  PID configuration structure for flash storage
  * @note   Matches original TI PID_CONFIG structure
  */
struct PID_CONFIG
{
  float Kp;            /*!< Proportional gain */
  float Ki;            /*!< Integral gain */
  float Kd;            /*!< Derivative gain */
  float Kff;           /*!< Feedforward gain */
  float dN;            /*!< Derivative filter pole */
  float K;             /*!< Shaping filter tracking gain */
  float Kout;          /*!< Output loop gain */
};

/**
  * @brief  PID handle type
  */
typedef struct _PID_Obj_ *PID_Handle;

/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

#define PID_SAT(x, max, min)  (((x) > (max)) ? (max) : (((x) < (min)) ? (min) : (x)))

/* Exported functions prototypes ---------------------------------------------*/

/**
  * @brief  Initialize PID controller
  * @note   Matches original TI PID_init signature
  * @param  pMemory: Pointer to PID_Obj memory
  * @param  numBytes: Size of allocated memory
  * @retval PID_Handle or NULL if memory too small
  */
extern PID_Handle PID_init(void *pMemory, const size_t numBytes);

/**
  * @brief  Run standard PID controller (P + I with saturation)
  * @param  handle: PID controller handle
  * @param  refValue: Reference setpoint
  * @param  fbackValue: Feedback measurement
  * @param  pOutValue: Pointer to store output value
  * @retval None
  */
void PID_run(PID_Handle handle, const float refValue,
             const float fbackValue, float *pOutValue);

/**
  * @brief  Run PID controller for speed (with feedforward and derivative)
  * @param  handle: PID controller handle
  * @param  refValue: Reference setpoint
  * @param  fbackValue: Feedback measurement
  * @param  ff: Feedforward term
  * @param  pOutValue: Pointer to store output value
  * @retval None
  */
void PID_run_spd(PID_Handle handle, const float refValue,
                 const float fbackValue, float ff, float *pOutValue);

/**
  * @brief  Run PID controller with setpoint weighting for speed
  * @param  handle: PID controller handle
  * @param  refValue: Reference setpoint
  * @param  fbackValue: Feedback measurement
  * @param  b: Setpoint weight for proportional term
  * @param  c: Setpoint weight (not used, no D term in this variant)
  * @param  pOutValue: Pointer to store output value
  * @retval None
  */
void PID_run_spd_spw(PID_Handle handle, const float refValue,
                     const float fbackValue, const float b,
                     const float c, float *pOutValue);

/* Inline getter/setter functions -------------------------------------------*/

static inline float PID_getFbackValue(PID_Handle handle) {
  return handle->fbackValue;
}

static inline float PID_getRefValue(PID_Handle handle) {
  return handle->refValue;
}

static inline float PID_getKp(PID_Handle handle) {
  return handle->Kp;
}

static inline float PID_getKi(PID_Handle handle) {
  return handle->Ki;
}

static inline float PID_getKd(PID_Handle handle) {
  return handle->Kd;
}

static inline float PID_getKff(PID_Handle handle) {
  return handle->Kff;
}

static inline float PID_getDN(PID_Handle handle) {
  return handle->dN;
}

static inline float PID_getK(PID_Handle handle) {
  return handle->K;
}

static inline float PID_getKout(PID_Handle handle) {
  return handle->Kout;
}

static inline float PID_getTS(PID_Handle handle) {
  return handle->ts;
}

static inline float PID_getUi(PID_Handle handle) {
  return handle->Ui;
}

static inline float PID_getLastError(PID_Handle handle) {
  return handle->lastError;
}

static inline float PID_getOutMax(PID_Handle handle) {
  return handle->outMax;
}

static inline float PID_getOutMin(PID_Handle handle) {
  return handle->outMin;
}

static inline bool PID_getUiSatFlag(PID_Handle handle) {
  return handle->UiSatFlag;
}

static inline void PID_getGains(PID_Handle handle, float *pKp, float *pKi,
                                float *pKd, float *pKff, float *pdN, float *pKout) {
  *pKp = handle->Kp;
  *pKi = handle->Ki;
  *pKd = handle->Kd;
  *pKff = handle->Kff;
  *pdN = handle->dN;
  *pKout = handle->Kout;
}

static inline void PID_getMinMax(PID_Handle handle, float *pOutMin, float *pOutMax) {
  *pOutMin = handle->outMin;
  *pOutMax = handle->outMax;
}

static inline void PID_getUiMinMax(PID_Handle handle, float *pOutMin, float *pOutMax) {
  *pOutMin = handle->UiOutMin;
  *pOutMax = handle->UiOutMax;
}

static inline void PID_setFbackValue(PID_Handle handle, const float fbackValue) {
  handle->fbackValue = fbackValue;
}

static inline void PID_setRefValue(PID_Handle handle, const float refValue) {
  handle->refValue = refValue;
}

static inline void PID_setKp(PID_Handle handle, const float Kp) {
  handle->Kp = Kp;
}

static inline void PID_setKff(PID_Handle handle, const float Kff) {
  handle->Kff = Kff;
}

static inline void PID_setKout(PID_Handle handle, const float gain) {
  handle->Kout = gain;
}

static inline void PID_setMinMax(PID_Handle handle, const float outMin, const float outMax) {
  handle->outMin = outMin;
  handle->outMax = outMax;
}

static inline void PID_setOutMax(PID_Handle handle, const float outMax) {
  handle->outMax = outMax;
}

static inline void PID_setOutMin(PID_Handle handle, const float outMin) {
  handle->outMin = outMin;
}

static inline void PID_setUiMinMax(PID_Handle handle, const float outMin, const float outMax) {
  handle->UiOutMin = outMin;
  handle->UiOutMax = outMax;
}

static inline void PID_setUdMax(PID_Handle handle, const float udMax) {
  handle->UdOutMax = udMax;
}

static inline float PID_getUdMax(PID_Handle handle) {
  return handle->UdOutMax;
}

static inline void PID_setUi(PID_Handle handle, const float Ui) {
  handle->Ui = Ui;
  handle->I_windup = 0.0f;
}

static inline void PID_setLastError(PID_Handle handle, const float lastError) {
  handle->lastError = lastError;
}

static inline void PID_setK(PID_Handle handle, const float K) {
  handle->K = K;
}

/**
  * @brief  Set integral gain (also updates Kis = Ki * ts)
  */
static inline void PID_setKi(PID_Handle handle, const float Ki) {
  handle->Ki = Ki;
  handle->Kis = Ki * handle->ts;
}

/**
  * @brief  Set derivative gain (updates scaled Kds = Kd / ts)
  */
static inline void PID_setKd(PID_Handle handle, const float Kd) {
  handle->Kd = Kd;
  handle->Kds = (handle->ts > 0.0f) ? (Kd / handle->ts) : 0.0f;
}

/**
  * @brief  Set derivative filter pole and update filter coefficients
  * @note   Uses bilinear transform with frequency warping
  * @param  handle: PID controller handle
  * @param  dN: Derivative filter pole (rad/s), 0 = no filter
  */
static inline void PID_setDN(PID_Handle handle, const float dN) {
  handle->dN = dN;

  if (dN > 0.0f && handle->ts > 0.0f) {
    float inv_Ts_over_2 = 2.0f / handle->ts;

    /* Correct frequency for bilinear transform frequency warping */
    float wN = inv_Ts_over_2 * tanf(dN * handle->ts / 2.0f);

    /* Setup the derivative feedback filter */
    handle->filter.b0 = wN / (wN + inv_Ts_over_2);           /* x[n] */
    handle->filter.a1 = (wN - inv_Ts_over_2) / (wN + inv_Ts_over_2); /* y[n-1] */
    handle->filter.b1 = handle->filter.b0;                    /* x[n-1] */
    handle->filter.x1 = 0.0f;
    handle->filter.y1 = 0.0f;
  }
}

/**
  * @brief  Set sample time (resets derivative filter and scaled gains)
  */
static inline void PID_setTS(PID_Handle handle, const float ts) {
  handle->ts = ts;
  PID_setDN(handle, handle->dN);
  PID_setKi(handle, handle->Ki);
  PID_setKd(handle, handle->Kd);
}

/**
  * @brief  Set all PID gains at once
  * @note   Matches original TI PID_setGains with 6 parameters
  */
static inline void PID_setGains(PID_Handle handle, const float Kp, const float Ki,
                                const float Kd, const float Kff, const float dN,
                                const float Kout) {
  handle->Kp = Kp;
  PID_setKi(handle, Ki);
  PID_setKd(handle, Kd);
  handle->Kff = Kff;
  handle->Kout = Kout;
  PID_setDN(handle, dN);
}

static inline void PID_setGain(PID_Handle handle, const float gain) {
  handle->Kout = gain;
}

#ifdef __cplusplus
}
#endif

#endif /* _PID_H_ */
