/*
 * crc.h
 *
 * CRC-16 calculation for communication packets.
 * Adapted from TI BearDriver bear::compute_crc / bear::validate_crc
 * for STM32G474.
 */

#ifndef CRC_H_
#define CRC_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/**
  * @brief  Calculate CRC-16 over a data buffer
  * @param  data: Pointer to data buffer
  * @param  len: Length of data in bytes
  * @retval CRC-16 value
  */
static inline uint16_t CRC_Calculate(const uint8_t *data, uint16_t len)
{
  uint16_t crc = 0xFFFF;

  for (uint16_t i = 0; i < len; i++) {
    crc ^= (uint16_t)data[i] << 8;
    for (uint8_t j = 0; j < 8; j++) {
      if (crc & 0x8000) {
        crc = (crc << 1) ^ 0x1021;  /* CRC-16-CCITT polynomial */
      } else {
        crc <<= 1;
      }
    }
  }

  return crc;
}

/**
  * @brief  Validate CRC-16 of a data buffer
  * @param  data: Pointer to data buffer
  * @param  len: Length of data (excluding CRC field)
  * @param  expected_crc: Expected CRC value
  * @retval true if CRC matches, false otherwise
  */
static inline bool CRC_Validate(const uint8_t *data, uint16_t len,
                                uint16_t expected_crc)
{
  return (CRC_Calculate(data, len) == expected_crc);
}

#ifdef __cplusplus
}
#endif

#endif /* CRC_H_ */
