/**
  ******************************************************************************
  * @file    user_motor_database.h
  * @author  Bear Robotics Motor Control Team
  * @brief   Motor electrical parameter definitions
  ******************************************************************************
  * @attention
  *
  * This file contains motor electrical parameters for different motor models.
  * Adapted from TI BearDriver for STM32G474.
  *
  ******************************************************************************
  */

#ifndef USER_MOTOR_DATABASE_H
#define USER_MOTOR_DATABASE_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>

/*============================================================================
 *  Motor type definitions
 *===========================================================================*/
#define MOTOR_NOT_IDENTIFIED                100U  /*!< Used for system identification */
#define MOTOR_ZHONGLING_ZLLG65ASM250        101U  /*!< Servi, Serviplus, Carti100 */
#define MOTOR_ZHONGLING_ZLLG65ASM250_L      102U  /*!< Low inductance variant */
#define MOTOR_ZHONGLING_ZLLG65ASM150_4096   103U  /*!< 4096 PPR encoder variant */
#define MOTOR_ZHONGLING_ZLLG65ASM250_4096   104U  /*!< Cart carrier POC */
#define MOTOR_ZHONGLING_ZLLG50ASM200_4096   105U  /*!< Servi-Q (default) */

/*============================================================================
 *  Motor-specific parameters
 *
 *  Only defines actually consumed at runtime are kept:
 *    NUM_POLE_PAIRS, Rs, Ls_d, Ls_q, TORQUE_CONSTANT,
 *    MAX_CURRENT, ENCODER_LINES, MAX_SPEED_RPM
 *===========================================================================*/

#if (USER_MOTOR == MOTOR_NOT_IDENTIFIED)
/* Unidentified motor - default parameters for identification */
#define USER_MOTOR_NUM_POLE_PAIRS           15U        /*!< Number of pole pairs */
#define USER_MOTOR_Rs                       0.0f       /*!< Phase resistance (Ohm) */
#define USER_MOTOR_Ls_d                     0.0f       /*!< D-axis inductance (H) */
#define USER_MOTOR_Ls_q                     0.0f       /*!< Q-axis inductance (H) */
#define USER_MOTOR_TORQUE_CONSTANT          0.0f       /*!< Torque constant (Nm/A) */
#define USER_MOTOR_MAX_CURRENT              30.0f      /*!< Maximum current limit (A) */
#define USER_MOTOR_ENCODER_LINES            0.0f       /*!< Encoder lines per revolution */
#define USER_MOTOR_MAX_SPEED_RPM            330.0f     /*!< Maximum speed (RPM) */

#elif (USER_MOTOR == MOTOR_ZHONGLING_ZLLG65ASM250_L)
/* ZLLG65ASM250_L - Low inductance variant */
#define USER_MOTOR_NUM_POLE_PAIRS           15U
#define USER_MOTOR_Rs                       0.255f
#define USER_MOTOR_Ls_d                     0.000697f
#define USER_MOTOR_Ls_q                     0.000697f
#define USER_MOTOR_TORQUE_CONSTANT          0.8791f
#define USER_MOTOR_MAX_CURRENT              30.0f
#define USER_MOTOR_ENCODER_LINES            1024.0f
#define USER_MOTOR_MAX_SPEED_RPM            330.0f

#elif (USER_MOTOR == MOTOR_ZHONGLING_ZLLG65ASM250)
/* ZLLG65ASM250 - Servi, Serviplus, Carti100 */
#define USER_MOTOR_NUM_POLE_PAIRS           15U
#define USER_MOTOR_Rs                       0.0691615716f
#define USER_MOTOR_Ls_d                     0.000145286713f
#define USER_MOTOR_Ls_q                     0.000145286713f
#define USER_MOTOR_TORQUE_CONSTANT          0.3424f
#define USER_MOTOR_MAX_CURRENT              30.0f
#define USER_MOTOR_ENCODER_LINES            1024.0f
#define USER_MOTOR_MAX_SPEED_RPM            330.0f

#elif (USER_MOTOR == MOTOR_ZHONGLING_ZLLG65ASM150_4096)
/* ZLLG65ASM150_4096 - 4096 PPR encoder variant */
#define USER_MOTOR_NUM_POLE_PAIRS           15U
#define USER_MOTOR_Rs                       0.302849978f
#define USER_MOTOR_Ls_d                     0.000892531539f
#define USER_MOTOR_Ls_q                     0.000892531539f
#define USER_MOTOR_TORQUE_CONSTANT          0.8791f
#define USER_MOTOR_MAX_CURRENT              30.0f
#define USER_MOTOR_ENCODER_LINES            4096.0f
#define USER_MOTOR_MAX_SPEED_RPM            330.0f

#elif (USER_MOTOR == MOTOR_ZHONGLING_ZLLG65ASM250_4096)
/* ZLLG65ASM250_4096 - Cart carrier POC */
#define USER_MOTOR_NUM_POLE_PAIRS           15U
#define USER_MOTOR_Rs                       0.32f
#define USER_MOTOR_Ls_d                     0.0007675f
#define USER_MOTOR_Ls_q                     0.0007675f
#define USER_MOTOR_TORQUE_CONSTANT          0.8791f
#define USER_MOTOR_MAX_CURRENT              24.0f
#define USER_MOTOR_ENCODER_LINES            4096.0f
#define USER_MOTOR_MAX_SPEED_RPM            330.0f

#elif (USER_MOTOR == MOTOR_ZHONGLING_ZLLG50ASM200_4096)
/* ZLLG50ASM200_4096 - Servi-Q (default) */
#define USER_MOTOR_NUM_POLE_PAIRS           10U
#define USER_MOTOR_Rs                       0.42f
#define USER_MOTOR_Ls_d                     0.00088f
#define USER_MOTOR_Ls_q                     0.00088f
#define USER_MOTOR_TORQUE_CONSTANT          0.5443f
#define USER_MOTOR_MAX_CURRENT              20.0f
#define USER_MOTOR_ENCODER_LINES            4096.0f
#define USER_MOTOR_MAX_SPEED_RPM            330.0f

#else
#error "No motor type specified — check USER_MOTOR in user_params.h"
#endif

#ifdef __cplusplus
}
#endif

#endif /* USER_MOTOR_DATABASE_H */
