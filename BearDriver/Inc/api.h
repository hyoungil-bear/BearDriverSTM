/**
  ******************************************************************************
  * @file    api.h
  * @author  Bear Robotics Motor Control Team
  * @brief   Motor control API for host communication protocol.
  ******************************************************************************
  * @attention
  *
  * This file implements the communication API adapted from TI BearDriver
  * for STM32G474. Provides register-based command/response interface for
  * dual motor control via UART/SPI with SLIP protocol.
  *
  ******************************************************************************
  */

#ifndef API_H
#define API_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>

/** @addtogroup BearDriver
  * @{
  */

/** @addtogroup API
  * @{
  */

/* Exported types ------------------------------------------------------------*/

/**
  * @brief  Packed structure attribute for GCC/ARM compilers
  */
#if defined(__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050)
  #define PACKED_ARM
  #define PACKED_GCC __attribute__((packed))
#elif defined(__CC_ARM)
  #define PACKED_ARM __packed
  #define PACKED_GCC
#elif defined(__GNUC__)
  #define PACKED_ARM
  #define PACKED_GCC __attribute__((packed))
#else
  #define PACKED_ARM
  #define PACKED_GCC
#endif

/**
  * @brief  API command type
  */
typedef enum
{
  API_CMD_WR = 0,        /*!< Write command from host (incoming data) */
  API_CMD_RD = 1,        /*!< Read request from host (incoming request for data) */
  API_CMD_RD_RESP = 2,   /*!< Read response to a read request (outgoing data) */
  API_CMD_WR_RD = 3,     /*!< Write command with read back request */
  API_CMD_END = 0x7FFF   /*!< Force enum to 16-bit */
} API_CMD_TYPE_e;

/**
  * @brief  API register addresses
  * @note   All args are 32-bit values, MSByte first order
  *         R = Read Only, W = Write Only, RW = Read/Write
  *         Each message has two args: arg1 for motor 1, arg2 for motor 2
  */
typedef enum
{
  /* System info registers */
  API_REG_VERSION = 0,              /*!< R: Firmware/Hardware version */
  API_REG_STATUS,                   /*!< R: Motor controller status */

  /* Tuning parameters */
  API_REG_ST_BW,                    /*!< RW: Velocity controller bandwidth (NOT USED) */
  API_REG_PID_Idq_P,                /*!< RW: Current controller PID P gain */
  API_REG_PID_Idq_I,                /*!< RW: Current controller PID I gain */

  /* Motor control registers */
  API_REG_TARGET_SPD_KRPM,          /*!< RW: Target velocity (float krpm) */
  API_REG_SPD_KRPM,                 /*!< R: Current velocity (float krpm) */
  API_REG_ENABLE_MOTOR,             /*!< RW: Motor enable (0=disabled, 1=enabled) */
  API_REG_ENABLE_OFFSET_CALCS,      /*!< W: Enable offset calculation */

  /* Telemetry */
  API_REG_ENCODER,                  /*!< R: Encoder position (rad) */
  API_REG_ENCODER_VEL,              /*!< R: Encoder velocity (krpm) */

  /* Speed PID parameters */
  API_REG_PID_SPEED_P,              /*!< RW: Speed controller PID P gain */
  API_REG_PID_SPEED_I,              /*!< RW: Speed controller PID I gain */
  API_REG_PID_SPEED_D,              /*!< RW: Speed controller PID D gain */
  API_REG_PID_SPEED_FF,             /*!< RW: Speed controller feed-forward gain */
  API_REG_PID_SPEED_Dn,             /*!< RW: Speed controller derivative filter pole */

  /* Control commands */
  API_REG_RESET_FAULT,              /*!< W: Reset fault state */
  API_REG_TRAJ_K,                   /*!< RW: Trajectory tracking gain */
  API_REG_TRAJ_MAX_DELTA,           /*!< RW: Trajectory max acceleration */
  API_REG_PID_SPEED_OUT,            /*!< RW: Speed controller output gain */

  /* System configuration */
  API_REG_HARDWARE_REV,             /*!< R: Hardware revision */
  API_REG_SET_TARGET_SPD_KRPM_GET_ENCODER_VEL,  /*!< RW/R: Combined register */
  API_REG_DISABLE_MOTORS_ON_BOOT,   /*!< RW: Disable motors on boot flag */
  API_REG_STALL_LOCK_ENABLE,        /*!< RW: Stall lock enable flag */

  API_REG_NUM,                      /*!< Total register count */
  API_REG_END = 0x7FFF              /*!< Force enum to 16-bit */
} API_REG_e;

/**
  * @brief  Base packet structure (host to motor)
  */
typedef PACKED_ARM struct
{
  API_CMD_TYPE_e rw;    /*!< Command type */
  API_REG_e reg;        /*!< Register address */
  uint32_t arg1;        /*!< Argument 1 (motor 1) */
  uint32_t arg2;        /*!< Argument 2 (motor 2) */
  uint16_t crc;         /*!< CRC checksum */
} PACKED_GCC BasePacket_t;

/**
  * @brief  Motor packet structure (motor to host)
  */
typedef PACKED_ARM struct
{
  API_CMD_TYPE_e rw;    /*!< Command type */
  API_REG_e reg;        /*!< Register address */
  uint32_t arg1;        /*!< Argument 1 (motor 1 data) */
  uint32_t arg2;        /*!< Argument 2 (motor 2 data) */
  uint32_t arg3;        /*!< Argument 3 (motor 1 status) */
  uint32_t arg4;        /*!< Argument 4 (motor 2 status) */
  uint32_t arg5;        /*!< Argument 5 (torque: motor2<<16 | motor1, units 0.1 Nm) */
  uint32_t arg6;        /*!< Argument 6 (temperature: motor2<<16 | motor1, units 0.1 degC) */
  uint16_t crc;         /*!< CRC checksum */
} PACKED_GCC MotorPacket_t;

/* Exported constants --------------------------------------------------------*/

/**
  * @brief  Communication interface selection
  */
#define API_USE_SCI     /*!< Use UART/RS485 with SLIP protocol */
// #define API_USE_SPI  /*!< Use SPI interface (alternative) */

/* Exported macro ------------------------------------------------------------*/

/**
  * @brief  Union for type-punning float <-> uint32_t conversion
  */
typedef union
{
  float f;
  uint32_t u;
} FloatUint32_t;

/**
  * @brief  Convert float to uint32_t for transmission
  */
static inline uint32_t API_FloatToUint32(float value)
{
  FloatUint32_t conv;
  conv.f = value;
  return conv.u;
}

/**
  * @brief  Convert uint32_t to float after reception
  */
static inline float API_Uint32ToFloat(uint32_t value)
{
  FloatUint32_t conv;
  conv.u = value;
  return conv.f;
}

/* Exported functions prototypes ---------------------------------------------*/

/**
  * @brief  Process command from host
  * @param  pIn: Pointer to received base packet
  * @param  pOut: Pointer to output motor packet
  * @retval true if response should be sent, false otherwise
  */
bool API_ProcessCommand(const BasePacket_t *pIn, MotorPacket_t *pOut);

/**
  * @brief  Process host communications (call from main loop)
  * @retval None
  */
void API_ProcessHostComs(void);

/**
  * @brief  Process debug console communications (call from main loop)
  * @retval None
  */
void API_ProcessDebugComs(void);

/**
  * @brief  Initialize API module
  * @retval None
  */
void API_Init(void);

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* API_H */

/******************* (C) COPYRIGHT Bear Robotics *****END OF FILE****/
