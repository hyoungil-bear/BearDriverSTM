/**
  ******************************************************************************
  * @file    sci_coms.h
  * @author  Bear Robotics Motor Control Team
  * @brief   Serial Communication Interface (UART) with SLIP encoding
  ******************************************************************************
  * @attention
  *
  * This file provides UART communication with SLIP encoding for API protocol.
  * Adapted from TI BearDriver for STM32G474.
  *
  ******************************************************************************
  */

#ifndef SCI_COMS_H
#define SCI_COMS_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include "api.h"

/** @addtogroup BearDriver
  * @{
  */

/** @addtogroup SCI_Coms
  * @{
  */

/* Exported types ------------------------------------------------------------*/

/**
  * @brief  UART device identifiers
  */
typedef enum
{
  SCI_A_FD = 0,    /*!< USART2 - Debug/STM bootloader (921600 baud) */
  SCI_B_FD = 1     /*!< USART3 - RS485 API (115200 baud) */
} SCI_Device_e;

/**
  * @brief  SLIP decoder state machine
  */
typedef enum
{
  SRX_IDLE = 0,    /*!< Idle, waiting for START */
  SRX_ESC,         /*!< Escape character received */
  SRX_CHAR         /*!< Receiving data characters */
} SLIP_RX_STATE;

/* Exported constants --------------------------------------------------------*/

/* Baud rates */
#define SCIA_BAUD_RATE      921600U   /*!< USB debug/STM bootloader */
#define SCIB_BAUD_RATE      115200U   /*!< RS485 API */

/* Buffer sizes (must be power of 2) */
#define SIZE_RX_BUFFER      128U      /*!< Receive buffer size */
#define SIZE_TX_BUFFER      128U      /*!< Transmit buffer size */

/* Buffer masks */
#define RX_BUFFER_LEN_MASK  0x7FU     /*!< RX buffer index mask */
#define TX_BUFFER_LEN_MASK  0x7FU     /*!< TX buffer index mask */

/* SLIP protocol characters */
#define SLIP_END            0xC0U     /*!< End of packet */
#define SLIP_START          0xC1U     /*!< Start of packet */
#define SLIP_ESC            0xDBU     /*!< Escape character */
#define SLIP_ESC_END        0xDCU     /*!< Escaped END */
#define SLIP_ESC_START      0xDEU     /*!< Escaped START */
#define SLIP_ESC_ESC        0xDDU     /*!< Escaped ESC */

/* Communication check timer values (in 10ms ticks) */
#define BASE_COM_CHECK_TIMER_10MS_RX     10U    /*!< 100ms normal operation */
#define BASE_COM_CHECK_TIMER_10MS_ERROR  500U   /*!< 5 second error recovery */

/* Exported macro ------------------------------------------------------------*/

/* Exported variables --------------------------------------------------------*/

/**
  * @brief  Base communication check timer (10ms ticks)
  */
extern volatile uint32_t base_com_check_timer_10ms;

/* Exported functions prototypes ---------------------------------------------*/

/**
  * @brief  Initialize SCI communication system
  * @retval None
  */
void SCI_Init(void);

/**
  * @brief  Initialize specific UART device
  * @param  dev: Device identifier (SCI_A_FD or SCI_B_FD)
  * @retval None
  */
void SCI_Device_Init(SCI_Device_e dev);

/**
  * @brief  Process host communications (call from main loop)
  * @note   Handles packet reception, decoding, and response
  * @retval None
  */
void SCI_ProcessHostComs(void);

/**
  * @brief  Read and decode a packet from UART
  * @param  dev: Device identifier
  * @param  prw: Pointer to store read/write command
  * @param  preg: Pointer to store register
  * @param  parg1: Pointer to store argument 1
  * @param  parg2: Pointer to store argument 2
  * @retval true if valid packet received, false otherwise
  */
bool SCI_ReadPacket(SCI_Device_e dev, API_CMD_TYPE_e *prw, API_REG_e *preg,
                    uint32_t *parg1, uint32_t *parg2);

/**
  * @brief  Send a packet via UART with SLIP encoding
  * @param  dev: Device identifier
  * @param  packet: Pointer to motor packet to send
  * @retval None
  */
void SCI_SendPacket(SCI_Device_e dev, const MotorPacket_t *packet);

/**
  * @brief  SLIP encode and transmit data buffer
  * @param  dev: Device identifier
  * @param  buf: Pointer to data buffer
  * @param  len: Length of data in bytes
  * @retval None
  */
void SCI_SlipEncode(SCI_Device_e dev, const uint8_t *buf, uint16_t len);

/**
  * @brief  SLIP decode incoming character
  * @param  dev: Device identifier
  * @param  c: Character to decode
  * @param  out_len: Pointer to store decoded packet length
  * @retval true if complete packet decoded, false otherwise
  */
bool SCI_SlipDecode(SCI_Device_e dev, uint8_t c, uint16_t *out_len);

/**
  * @brief  Check if UART transmit buffer is empty
  * @param  dev: Device identifier
  * @retval true if empty, false otherwise
  */
bool SCI_TxEmpty(SCI_Device_e dev);

/**
  * @brief  Check if receive data is available
  * @param  dev: Device identifier
  * @retval true if data available, false otherwise
  */
bool SCI_RxAvailable(SCI_Device_e dev);

/**
  * @brief  Get received character from buffer
  * @param  dev: Device identifier
  * @retval Character (0-255) or -1 if no data
  */
int16_t SCI_GetChar(SCI_Device_e dev);

/**
  * @brief  Put character into transmit buffer
  * @param  dev: Device identifier
  * @param  c: Character to send
  * @retval true if successful, false if buffer full
  */
bool SCI_PutChar(SCI_Device_e dev, uint8_t c);

/**
  * @brief  UART RX interrupt callback
  * @param  dev: Device identifier
  * @param  data: Received character
  * @retval None
  */
void SCI_RxCallback(SCI_Device_e dev, uint8_t data);

/**
  * @brief  UART TX interrupt callback
  * @param  dev: Device identifier
  * @retval None
  */
void SCI_TxCallback(SCI_Device_e dev);

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* SCI_COMS_H */

/******************* (C) COPYRIGHT Bear Robotics *****END OF FILE****/
