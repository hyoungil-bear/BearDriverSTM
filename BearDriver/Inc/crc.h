/*
 * crc.h
 *
 * XOR-based 16-bit CRC for communication packets.
 * Matches TI BearDriver bear::xor_crc<uint16_t> exactly.
 *
 * Algorithm: XOR all 16-bit little-endian words of the payload.
 *   init = 0, no polynomial, no reflection, no final XOR.
 */

#ifndef CRC_H_
#define CRC_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/**
  * @brief  Calculate XOR-16 checksum over a data buffer
  * @note   Matches TI BearDriver bear::xor_crc<uint16_t>(data, len):
  *           crc = word[0] ^ word[1] ^ ... ^ word[n-1]  (16-bit little-endian words)
  * @param  data: Pointer to data buffer
  * @param  len: Length of data in bytes
  * @retval XOR-16 checksum
  */
static inline uint16_t CRC_Calculate(const uint8_t *data, uint16_t len)
{
  uint16_t crc = 0;

  /* XOR all 16-bit little-endian words */
  for (uint16_t i = 0; i + 1U < len; i += 2U) {
    crc ^= (uint16_t)data[i] | ((uint16_t)data[i + 1U] << 8);
  }
  /* Trailing odd byte (if len is odd) */
  if (len & 1U) {
    crc ^= (uint16_t)data[len - 1U];
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
