/**
  ******************************************************************************
  * @file    user_params.h
  * @author  Bear Robotics Motor Control Team
  * @brief   System-wide user parameter definitions
  ******************************************************************************
  * @attention
  *
  * This file provides system-wide control parameters, hardware constants,
  * and derived conversion factors.
  * Adapted from TI BearDriver for STM32G474.
  *
  ******************************************************************************
  */

#ifndef USER_PARAMS_H
#define USER_PARAMS_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
/*============================================================================
 *  Motor selection (selects parameter set from user_motor_database.h)
 *===========================================================================*/
#ifdef BUILD_SYS_ID
#define USER_MOTOR  MOTOR_NOT_IDENTIFIED
#else
#define USER_MOTOR  MOTOR_ZHONGLING_ZLLG50ASM200_4096  /*!< Default motor */
#endif

#include "user_motor_database.h"

/*============================================================================
 *  Base hardware constants
 *===========================================================================*/

/* Power stage */
#define RSHUNT                          0.005    /*!< Shunt resistance (Ohm) */
#define AMPLIFICATION_GAIN              12       /*!< Current sense amplifier gain */
#define VBUS_PARTITIONING_FACTOR        0.05897269564191779  /*!< Vbus resistor divider ratio */
#define ADC_REFERENCE_VOLTAGE           3.3      /*!< ADC reference voltage (V) */

/* MCU clock / Timer (STM32G4xx) */
#define ADV_TIM_CLK_MHz                 170      /*!< Advanced timer clock (MHz) */
#define TIM_CLOCK_DIVIDER               1        /*!< TIM clock prescaler (1 = no division) */

/* Drive settings */
#define PWM_FREQUENCY                   10000    /*!< PWM switching frequency (Hz) */
#define SW_DEADTIME_NS                  1000     /*!< Software dead-time (ns) */
#define REGULATION_EXECUTION_RATE       1        /*!< FOC execution rate in PWM cycles */

/*============================================================================
 *  Derived conversions (depend on base constants above)
 *===========================================================================*/

//! ADC per-unit (0~1) to Amperes: Vref / (Rshunt x Gain) = 3.3 / (0.005 x 12) = 55.0
#define ADC_TO_AMPS                     (ADC_REFERENCE_VOLTAGE / (RSHUNT * AMPLIFICATION_GAIN))

#define VBUS_ADC_TO_VOLT                (ADC_REFERENCE_VOLTAGE / VBUS_PARTITIONING_FACTOR)

#define PWM_PERIOD_CYCLES               (uint16_t)(((uint32_t)ADV_TIM_CLK_MHz * (uint32_t)1000000u \
                                         / ((uint32_t)(PWM_FREQUENCY))) & (uint16_t)0xFFFE)

#define REP_COUNTER                     (uint16_t)((REGULATION_EXECUTION_RATE * 2u) - 1u)

#define HTMIN                           1  /* CCR4 placeholder; overwritten at runtime */

/* Dead-time register value (STM32 TIMx_BDTR.DTG encoding) */
#define DEAD_TIME_COUNTS_1              (ADV_TIM_CLK_MHz * TIM_CLOCK_DIVIDER * SW_DEADTIME_NS / 1000uL)

#if (DEAD_TIME_COUNTS_1 <= 255)
#define DEAD_TIME_COUNTS                (uint16_t)DEAD_TIME_COUNTS_1
#elif (DEAD_TIME_COUNTS_1 <= 508)
#define DEAD_TIME_COUNTS                (uint16_t)(((ADV_TIM_CLK_MHz * TIM_CLOCK_DIVIDER * SW_DEADTIME_NS / 2) / 1000uL) + 128)
#elif (DEAD_TIME_COUNTS_1 <= 1008)
#define DEAD_TIME_COUNTS                (uint16_t)(((ADV_TIM_CLK_MHz * TIM_CLOCK_DIVIDER * SW_DEADTIME_NS / 8) / 1000uL) + 320)
#elif (DEAD_TIME_COUNTS_1 <= 2015)
#define DEAD_TIME_COUNTS                (uint16_t)(((ADV_TIM_CLK_MHz * TIM_CLOCK_DIVIDER * SW_DEADTIME_NS / 16) / 1000uL) + 384)
#else
#define DEAD_TIME_COUNTS                510
#endif

/*============================================================================
 *  Hardware version definitions
 *===========================================================================*/
#define MOTOR_HW_VERSION_1              1
#define MOTOR_HW_VERSION_2              2
#define MOTOR_HW_VERSION_3              3
#define MOTOR_HW_VERSION_15             15

/*============================================================================
 *  Control timing
 *===========================================================================*/
#define USER_NUM_PWM_TICKS_PER_ISR_TICK     1U         /*!< PWM to ISR decimation (1:1) */
#define USER_NUM_ISR_TICKS_PER_CTRL_TICK    1U         /*!< ISR to control decimation */
#define USER_NUM_CTRL_TICKS_PER_SPEED_TICK  10U        /*!< Control to speed decimation (10kHz/10 = 1kHz) */

#define USER_PWM_FREQ_kHz               (PWM_FREQUENCY / 1000.0f)
#define USER_PWM_PERIOD_usec            (1000.0f / USER_PWM_FREQ_kHz)

#define USER_CTRL_FREQ_Hz               ((USER_PWM_FREQ_kHz * 1000.0f) / \
                                         (float)(USER_NUM_PWM_TICKS_PER_ISR_TICK * \
                                                 USER_NUM_ISR_TICKS_PER_CTRL_TICK))
#define USER_CTRL_PERIOD_sec            (1.0f / USER_CTRL_FREQ_Hz)

/*============================================================================
 *  TI per-unit ↔ SI gain conversion (empirical mapping)
 *
 *  TI API default:  Kp_pu = 45,  Ki_pu = 300
 *  STM tuned (SI):  Kp_SI = 0.08, Ki_SI = 0.2
 *
 *  Conversion (separate for P and I):
 *    Kp_SI = Kp_pu * SPD_KP_PU_TO_SI      Ki_SI = Ki_pu * SPD_KI_PU_TO_SI
 *    Kp_pu = Kp_SI / SPD_KP_PU_TO_SI      Ki_pu = Ki_SI / SPD_KI_PU_TO_SI
 *===========================================================================*/
#define TI_REF_KP_PU        45.0f              /*!< TI reference Kp (per-unit) */
#define TI_REF_KI_PU        300.0f             /*!< TI reference Ki (per-unit) */
#define STM_REF_KP_SI       0.08f              /*!< STM tuned Kp [A/RPM] */
#define STM_REF_KI_SI       2.0f               /*!< STM tuned Ki [A/(RPM·s)] */

#define SPD_KP_PU_TO_SI     (STM_REF_KP_SI / TI_REF_KP_PU)   /* = 0.001778 */
#define SPD_KI_PU_TO_SI     (STM_REF_KI_SI / TI_REF_KI_PU)   /* = 0.000667 */

/*  Kd conversion (empirical):
 *    TI  Kd_pu = 0.05   →  STM Kd_SI = 0.0008
 *    SPD_KD_PU_TO_SI = 0.0008 / 0.05 = 0.016                       */
#define SPD_KD_PU_TO_SI     (0.0008f / 0.05f)

/*  Kff conversion (empirical):
 *    TI  Kff_pu = 4.5   →  STM Kff_SI = 0.1
 *    SPD_KFF_PU_TO_SI = 0.1 / 4.5 = 0.02222                        */
#define SPD_KFF_PU_TO_SI    (0.1f / 4.5f)

/*============================================================================
 *  Motor control limits
 *===========================================================================*/
#define USER_MAX_VS_MAG_PU                  0.5f       /*!< Max voltage vector magnitude (pu) */

/*============================================================================
 *  Filter parameters
 *===========================================================================*/
#define USER_VOLTAGE_FILTER_POLE_Hz         214.97f    /*!< Voltage filter pole (Hz) */
#define USER_OFFSET_POLE_rps            (20.0f)  /*!< Offset estimation pole, rad/s (TI default, do not change) */
#define USER_VOLTAGE_FILTER_POLE_rps    (USER_VOLTAGE_FILTER_POLE_Hz * 2.0f * M_PI)

/*============================================================================
 *  Stall detection thresholds
 *===========================================================================*/
#define USER_MOTOR_STALL_ZERO_SPEED_THRESHOLD      5.0f   /*!< 5 RPM threshold */
#define USER_MOTOR_STALL_NON_ZERO_SPEED_THRESHOLD  0.0f   /*!< 0 RPM threshold */
#define USER_MOTOR_STALL_CURRENT_RATIO             0.9f   /*!< 90% of max current */

/*============================================================================
 *  Encoder input-capture mode selector
 *
 *  LEGACY : per-edge CC interrupt fires handleCapture() each encoder edge.
 *           Prescaler 169 -> 1 MHz tick, 1 us resolution.
 *  POLLING: CC interrupt disabled; speed controller (updateValues) polls
 *           CCR1 each cycle. Prescaler 16 -> 10 MHz tick, 100 ns resolution.
 *           Only the 16-bit overflow (UIE, ~152 Hz) fires an ISR.
 *===========================================================================*/
#define ENCODER_CAPTURE_MODE_LEGACY         0
#define ENCODER_CAPTURE_MODE_POLLING        1

#ifndef ENCODER_CAPTURE_MODE
#define ENCODER_CAPTURE_MODE  ENCODER_CAPTURE_MODE_LEGACY
#endif

/*============================================================================
 *  Error codes (encoder FOC subset of TI USER_ErrorCode_e)
 *===========================================================================*/
typedef enum {
  USER_ErrorCode_NoError = 0,
  USER_ErrorCode_maxCurrent_Invalid,       /*!< maxCurrent <= 0 */
  USER_ErrorCode_polePairs_Invalid,        /*!< motor_numPolePairs == 0 */
  USER_ErrorCode_motorRs_Invalid,          /*!< motor_Rs < 0 */
  USER_ErrorCode_motorLs_Invalid,          /*!< motor_Ls_d or Ls_q < 0 */
  USER_ErrorCode_ctrlFreq_Invalid,         /*!< ctrlFreq_Hz <= 0 */
  USER_ErrorCode_pwmPeriod_Invalid,        /*!< pwmPeriod_usec <= 0 */
  USER_ErrorCode_maxVsMag_Invalid,         /*!< maxVsMag_pu <= 0 or > 1.0 */
  USER_ErrorCode_offsetPole_Invalid,       /*!< offsetPole_rps <= 0 */
  USER_ErrorCode_voltageFilterPole_Invalid,/*!< voltageFilterPole_rps <= 0 */
  USER_numErrorCodes
} USER_ErrorCode_e;

/*============================================================================
 *  Types
 *===========================================================================*/

/** Speed PID parameter structure (matches original TI USER_Spd_Params) */
typedef struct {
  float Kp;    /*!< Proportional gain */
  float Ki;    /*!< Integral gain */
  float Kd;    /*!< Derivative gain */
  float Kff;   /*!< Feed-forward gain */
  float dN;    /*!< Derivative filter pole */
  float Kout;  /*!< Output loop gain */
} USER_Spd_Params;

/**
  * User parameters structure.
  * 16-bit fields are paired to avoid alignment padding.
  */
typedef struct _USER_Params_ {
  /* Hardware + Motor identification */
  int16_t  hwVersion;                /*!< Hardware version from HW id bits */
  uint16_t motor_numPolePairs;       /*!< Number of pole pairs */

  /* Motor electrical parameters */
  float motor_Rs;                    /*!< Stator resistance, Ohm */
  float motor_Ls_d;                  /*!< Direct stator inductance, H */
  float motor_Ls_q;                  /*!< Quadrature stator inductance, H */
  float torqueConstant;              /*!< Torque constant (Nm/A) */

  /* Current limit */
  float maxCurrent;                  /*!< Battery OCP limit, A */

  /* PWM / Control timing */
  uint16_t numPwmTicksPerIsrTick;    /*!< PWM ticks per ISR tick */
  uint16_t numCtrlTicksPerSpeedTick; /*!< Ctrl ticks per speed tick */
  float pwmPeriod_usec;              /*!< PWM period, usec */
  float ctrlFreq_Hz;                 /*!< Controller frequency, Hz */
  float ctrlPeriod_sec;              /*!< Controller period, sec */
  float maxVsMag_pu;                 /*!< Maximum voltage vector magnitude, pu */

  /* Filter poles */
  float offsetPole_rps;              /*!< Offset estimation pole, rad/s */
  float voltageFilterPole_rps;       /*!< Analog voltage filter pole, rad/s */

  /* Speed PID (HW-version-dependent) */
  USER_Spd_Params spdParams;         /*!< Speed control loop parameters */
} USER_Params;

/*============================================================================
 *  Function prototypes
 *===========================================================================*/

void USER_setParamsMtr(USER_Params *pUserParams, int16_t hwVersion);
USER_ErrorCode_e USER_checkForErrors(const USER_Params *pUserParams);

#ifdef __cplusplus
}
#endif

#endif /* USER_PARAMS_H */
