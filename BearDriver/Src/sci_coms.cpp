/**
  ******************************************************************************
  * @file    sci_coms.cpp
  * @author  Bear Robotics Motor Control Team
  * @brief   Serial Communication Interface implementation with SLIP encoding
  ******************************************************************************
  * @attention
  *
  * This file implements UART communication with SLIP protocol for API.
  * Adapted from TI BearDriver for STM32G474.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "sci_coms.h"
#include "main.h"
#include "usart.h"
#include "gpio.h"
#include "api.h"
#include "crc.h"
#include <string.h>

/** @addtogroup BearDriver
  * @{
  */

/** @addtogroup SCI_Coms
  * @{
  */

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/**
  * @brief  Receive ISR buffers (circular)
  */
static volatile uint16_t rx_isr_buffer[2][SIZE_RX_BUFFER];
static volatile uint16_t rx_isr_in_idx[2] = {0, 0};
static volatile uint16_t rx_isr_out_idx[2] = {0, 0};

/**
  * @brief  Transmit ISR buffers (circular)
  */
static volatile uint16_t tx_isr_buffer[2][SIZE_TX_BUFFER];
static volatile uint16_t tx_isr_in_idx[2] = {0, 0};
static volatile uint16_t tx_isr_out_idx[2] = {0, 0};

/**
  * @brief  SLIP decoding buffers
  */
static uint16_t rx_slip_buffer[2][SIZE_RX_BUFFER];
static uint16_t rx_slip_idx[2] = {0, 0};

/**
  * @brief  SLIP decoder state
  */
static SLIP_RX_STATE slip_state[2] = {SRX_IDLE, SRX_IDLE};

/**
  * @brief  Base communication check timer
  */
volatile uint32_t base_com_check_timer_10ms = 0;

/**
  * @brief  RS485 direction control
  */
static bool rs485_tx_active[2] = {false, false};

/* Private function prototypes -----------------------------------------------*/

static bool SCI_IsRxBufferEmpty(SCI_Device_e dev);
static bool SCI_IsRxBufferFull(SCI_Device_e dev);
static bool SCI_IsTxBufferEmpty(SCI_Device_e dev);
static bool SCI_IsTxBufferFull(SCI_Device_e dev);
static uint8_t SCI_ReadRxBuffer(SCI_Device_e dev);
static void SCI_WriteTxBuffer(SCI_Device_e dev, uint8_t c);
static UART_HandleTypeDef* SCI_GetUartHandle(SCI_Device_e dev);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Get UART handle for device
  * @param  dev: Device identifier
  * @retval Pointer to UART handle
  */
static UART_HandleTypeDef* SCI_GetUartHandle(SCI_Device_e dev)
{
  if (dev == SCI_A_FD) {
    return &huart2;  /* USART2 for debug */
  } else {
    return &huart3;  /* USART3 for RS485 */
  }
}

/**
  * @brief  Check if RX buffer is empty
  * @param  dev: Device identifier
  * @retval true if empty, false otherwise
  */
static bool SCI_IsRxBufferEmpty(SCI_Device_e dev)
{
  return (rx_isr_in_idx[dev] == rx_isr_out_idx[dev]);
}

/**
  * @brief  Check if RX buffer is full
  * @param  dev: Device identifier
  * @retval true if full, false otherwise
  */
static bool SCI_IsRxBufferFull(SCI_Device_e dev)
{
  return (((rx_isr_in_idx[dev] + 1) & RX_BUFFER_LEN_MASK) == rx_isr_out_idx[dev]);
}

/**
  * @brief  Check if TX buffer is empty
  * @param  dev: Device identifier
  * @retval true if empty, false otherwise
  */
static bool SCI_IsTxBufferEmpty(SCI_Device_e dev)
{
  return (tx_isr_in_idx[dev] == tx_isr_out_idx[dev]);
}

/**
  * @brief  Check if TX buffer is full
  * @param  dev: Device identifier
  * @retval true if full, false otherwise
  */
static bool SCI_IsTxBufferFull(SCI_Device_e dev)
{
  return (((tx_isr_in_idx[dev] + 1) & TX_BUFFER_LEN_MASK) == tx_isr_out_idx[dev]);
}

/**
  * @brief  Read character from RX buffer
  * @param  dev: Device identifier
  * @retval Character value
  */
static uint8_t SCI_ReadRxBuffer(SCI_Device_e dev)
{
  uint8_t c = (uint8_t)rx_isr_buffer[dev][rx_isr_out_idx[dev]];
  rx_isr_out_idx[dev] = (rx_isr_out_idx[dev] + 1) & RX_BUFFER_LEN_MASK;
  return c;
}

/**
  * @brief  Write character to TX buffer
  * @param  dev: Device identifier
  * @param  c: Character to write
  * @retval None
  */
static void SCI_WriteTxBuffer(SCI_Device_e dev, uint8_t c)
{
  if (SCI_IsTxBufferFull(dev)) {
    return;  /* Drop byte — buffer full; packet will be malformed, CRC rejects it */
  }

  tx_isr_buffer[dev][tx_isr_in_idx[dev]] = c;
  tx_isr_in_idx[dev] = (tx_isr_in_idx[dev] + 1) & TX_BUFFER_LEN_MASK;

  /* Enable TX interrupt */
  UART_HandleTypeDef *huart = SCI_GetUartHandle(dev);
  __HAL_UART_ENABLE_IT(huart, UART_IT_TXE);
}

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  Initialize SCI communication system
  * @retval None
  */
void SCI_Init(void)
{
  /* Initialize both devices */
  SCI_Device_Init(SCI_A_FD);
  SCI_Device_Init(SCI_B_FD);

  /* Initialize communication check timer */
  base_com_check_timer_10ms = 0;
}

/**
  * @brief  Initialize specific UART device
  * @param  dev: Device identifier
  * @retval None
  */
void SCI_Device_Init(SCI_Device_e dev)
{
  /* Clear buffers */
  memset((void*)&rx_isr_buffer[dev][0], 0, SIZE_RX_BUFFER * sizeof(uint16_t));
  memset((void*)&tx_isr_buffer[dev][0], 0, SIZE_TX_BUFFER * sizeof(uint16_t));
  memset(&rx_slip_buffer[dev][0], 0, SIZE_RX_BUFFER * sizeof(uint16_t));

  /* Reset indices */
  rx_isr_in_idx[dev] = 0;
  rx_isr_out_idx[dev] = 0;
  tx_isr_in_idx[dev] = 0;
  tx_isr_out_idx[dev] = 0;
  rx_slip_idx[dev] = 0;

  /* Reset SLIP decoder */
  slip_state[dev] = SRX_IDLE;

  /* Reset RS485 state */
  rs485_tx_active[dev] = false;

  /* Enable UART RX interrupt */
  UART_HandleTypeDef *huart = SCI_GetUartHandle(dev);
  __HAL_UART_ENABLE_IT(huart, UART_IT_RXNE);

  /* Set RS485 to receive mode if applicable */
  if (dev == SCI_B_FD) {
    /* TODO: Set RS485 DE pin to receive mode */
    /* HAL_GPIO_WritePin(RS485_DE_GPIO_Port, RS485_DE_Pin, GPIO_PIN_RESET); */
  }
}

/**
  * @brief  Process host communications
  * @retval None
  */
void SCI_ProcessHostComs(void)
{
  API_CMD_TYPE_e rw;
  API_REG_e reg;
  uint32_t arg1, arg2;

  /* Process RS485 (SCI_B) */
  SCI_Device_e dev = SCI_B_FD;

  /* RS485 DE is controlled automatically by USART3 hardware (AF7, HAL_RS485Ex_Init).
   * rs485_tx_active tracks TX state for RX suppression only. */
  if (rs485_tx_active[dev] && SCI_TxEmpty(dev)) {
    rs485_tx_active[dev] = false;
  }

  /* Check for incoming packet — skip while TX in progress (RS485 half-duplex) */
  if (!rs485_tx_active[dev] && SCI_ReadPacket(dev, &rw, &reg, &arg1, &arg2)) {
    BasePacket_t in_packet = {rw, reg, arg1, arg2, 0};
    MotorPacket_t out_packet = {};

    if (API_ProcessCommand(&in_packet, &out_packet)) {
      /* Reset communication check timer */
      base_com_check_timer_10ms = BASE_COM_CHECK_TIMER_10MS_RX;

      /* Send response if read command */
      if (rw == API_CMD_RD || rw == API_CMD_WR_RD) {
        /* Calculate CRC */
        out_packet.crc = CRC_Calculate((const uint8_t*)&out_packet,
                                       sizeof(MotorPacket_t) - sizeof(uint16_t));
        SCI_SendPacket(dev, &out_packet);
      }
    }
  }
}

/**
  * @brief  Read and decode a packet
  * @param  dev: Device identifier
  * @param  prw: Pointer to store read/write command
  * @param  preg: Pointer to store register
  * @param  parg1: Pointer to store argument 1
  * @param  parg2: Pointer to store argument 2
  * @retval true if valid packet received, false otherwise
  */
bool SCI_ReadPacket(SCI_Device_e dev, API_CMD_TYPE_e *prw, API_REG_e *preg,
                    uint32_t *parg1, uint32_t *parg2)
{
  while (!SCI_IsRxBufferEmpty(dev)) {
    uint8_t c = SCI_ReadRxBuffer(dev);
    uint16_t out_len = 0;

    if (SCI_SlipDecode(dev, c, &out_len)) {
      /* Complete packet received */
      if (out_len != sizeof(BasePacket_t)) {
        return false;  /* Invalid packet size */
      }

      BasePacket_t packet;
      memcpy(&packet, &rx_slip_buffer[dev][0], sizeof(BasePacket_t));

      /* Validate CRC */
      uint16_t calc_crc = CRC_Calculate((const uint8_t*)&packet,
                                        sizeof(BasePacket_t) - sizeof(uint16_t));
      if (calc_crc != packet.crc) {
        return false;  /* CRC mismatch */
      }

      /* Extract packet fields */
      *prw = packet.rw;
      *preg = packet.reg;
      *parg1 = packet.arg1;
      *parg2 = packet.arg2;

      return true;
    }
  }

  return false;
}

/**
  * @brief  Send a packet via UART
  * @param  dev: Device identifier
  * @param  packet: Pointer to motor packet
  * @retval None
  */
void SCI_SendPacket(SCI_Device_e dev, const MotorPacket_t *packet)
{
  /* DE is asserted automatically by USART3 hardware (PB14 = AF7 USART3_DE).
   * rs485_tx_active tracks TX in-progress for RX suppression. */
  if (dev == SCI_B_FD) {
    rs485_tx_active[dev] = true;
  }

  /* SLIP encode and transmit */
  SCI_SlipEncode(dev, (const uint8_t*)packet, sizeof(MotorPacket_t));
}

/**
  * @brief  SLIP encode and transmit data
  * @param  dev: Device identifier
  * @param  buf: Pointer to data buffer
  * @param  len: Length in bytes
  * @retval None
  */
void SCI_SlipEncode(SCI_Device_e dev, const uint8_t *buf, uint16_t len)
{
  /* Send START */
  SCI_WriteTxBuffer(dev, SLIP_START);

  /* Send data with escaping */
  for (uint16_t i = 0; i < len; i++) {
    uint8_t c = buf[i];

    if (c == SLIP_END) {
      SCI_WriteTxBuffer(dev, SLIP_ESC);
      SCI_WriteTxBuffer(dev, SLIP_ESC_END);
    } else if (c == SLIP_ESC) {
      SCI_WriteTxBuffer(dev, SLIP_ESC);
      SCI_WriteTxBuffer(dev, SLIP_ESC_ESC);
    } else if (c == SLIP_START) {
      SCI_WriteTxBuffer(dev, SLIP_ESC);
      SCI_WriteTxBuffer(dev, SLIP_ESC_START);
    } else {
      SCI_WriteTxBuffer(dev, c);
    }
  }

  /* Send END */
  SCI_WriteTxBuffer(dev, SLIP_END);
}

/**
  * @brief  SLIP decode incoming character
  * @param  dev: Device identifier
  * @param  c: Character to decode
  * @param  out_len: Pointer to store decoded length
  * @retval true if complete packet decoded, false otherwise
  */
bool SCI_SlipDecode(SCI_Device_e dev, uint8_t c, uint16_t *out_len)
{
  /* Check for buffer overflow */
  if (rx_slip_idx[dev] >= SIZE_RX_BUFFER) {
    slip_state[dev] = SRX_IDLE;
    rx_slip_idx[dev] = 0;
    return false;
  }

  switch (slip_state[dev]) {
    case SRX_IDLE:
      rx_slip_idx[dev] = 0;
      if (c == SLIP_START) {
        slip_state[dev] = SRX_CHAR;
      }
      break;

    case SRX_ESC:
      if (c == SLIP_ESC_ESC) {
        rx_slip_buffer[dev][rx_slip_idx[dev]++] = SLIP_ESC;
        slip_state[dev] = SRX_CHAR;
      } else if (c == SLIP_ESC_END) {
        rx_slip_buffer[dev][rx_slip_idx[dev]++] = SLIP_END;
        slip_state[dev] = SRX_CHAR;
      } else if (c == SLIP_ESC_START) {
        rx_slip_buffer[dev][rx_slip_idx[dev]++] = SLIP_START;
        slip_state[dev] = SRX_CHAR;
      } else {
        /* Invalid escape sequence */
        slip_state[dev] = SRX_IDLE;
      }
      break;

    case SRX_CHAR:
      if (c == SLIP_END) {
        /* Complete packet received */
        *out_len = rx_slip_idx[dev];
        rx_slip_idx[dev] = 0;
        slip_state[dev] = SRX_IDLE;
        return true;
      } else if (c == SLIP_ESC) {
        slip_state[dev] = SRX_ESC;
      } else if (c == SLIP_START) {
        /* Unexpected START - restart */
        slip_state[dev] = SRX_IDLE;
      } else {
        rx_slip_buffer[dev][rx_slip_idx[dev]++] = c;
      }
      break;
  }

  return false;
}

/**
  * @brief  Check if TX is empty
  * @param  dev: Device identifier
  * @retval true if empty, false otherwise
  */
bool SCI_TxEmpty(SCI_Device_e dev)
{
  UART_HandleTypeDef *huart = SCI_GetUartHandle(dev);
  /* Buffer empty AND shift register drained (TC = Transmission Complete) */
  return (SCI_IsTxBufferEmpty(dev) &&
          __HAL_UART_GET_FLAG(huart, UART_FLAG_TC));
}

/**
  * @brief  Check if RX data available
  * @param  dev: Device identifier
  * @retval true if data available, false otherwise
  */
bool SCI_RxAvailable(SCI_Device_e dev)
{
  return !SCI_IsRxBufferEmpty(dev);
}

/**
  * @brief  Get character from RX buffer
  * @param  dev: Device identifier
  * @retval Character or -1 if no data
  */
int16_t SCI_GetChar(SCI_Device_e dev)
{
  if (SCI_IsRxBufferEmpty(dev)) {
    return -1;
  }

  return (int16_t)SCI_ReadRxBuffer(dev);
}

/**
  * @brief  Put character into TX buffer
  * @param  dev: Device identifier
  * @param  c: Character to send
  * @retval true if successful, false if buffer full
  */
bool SCI_PutChar(SCI_Device_e dev, uint8_t c)
{
  if (SCI_IsTxBufferFull(dev)) {
    return false;
  }

  SCI_WriteTxBuffer(dev, c);
  return true;
}

/**
  * @brief  UART RX interrupt callback
  * @param  dev: Device identifier
  * @param  data: Received character
  * @retval None
  */
void SCI_RxCallback(SCI_Device_e dev, uint8_t data)
{
  if (!SCI_IsRxBufferFull(dev)) {
    rx_isr_buffer[dev][rx_isr_in_idx[dev]] = data;
    rx_isr_in_idx[dev] = (rx_isr_in_idx[dev] + 1) & RX_BUFFER_LEN_MASK;
  }
}

/**
  * @brief  UART TX interrupt callback
  * @param  dev: Device identifier
  * @retval None
  */
void SCI_TxCallback(SCI_Device_e dev)
{
  if (!SCI_IsTxBufferEmpty(dev)) {
    /* Send next character */
    uint8_t c = (uint8_t)tx_isr_buffer[dev][tx_isr_out_idx[dev]];
    tx_isr_out_idx[dev] = (tx_isr_out_idx[dev] + 1) & TX_BUFFER_LEN_MASK;

    UART_HandleTypeDef *huart = SCI_GetUartHandle(dev);
    huart->Instance->TDR = c;
  } else {
    /* Buffer empty - disable TX interrupt */
    UART_HandleTypeDef *huart = SCI_GetUartHandle(dev);
    __HAL_UART_DISABLE_IT(huart, UART_IT_TXE);
  }
}

/**
  * @}
  */

/**
  * @}
  */

/******************* (C) COPYRIGHT Bear Robotics *****END OF FILE****/
