/*
 * spi_flash.h
 *
 * FM25V02A SPI FRAM driver for STM32G474.
 * 256Kbit (32KB), no erase required, byte-addressable, 10^14 write endurance.
 * SPI Mode 0 (CPOL=0, CPHA=0), max SCK 20MHz.
 *
 * Hardware: SPI3 (PC10=SCK, PC11=MISO, PC12=MOSI), NSS=PA15 (hardware)
 *           WP=VDD (write protect disabled), HOLD=VDD (hold disabled)
 * Supports both software CS (GPIO) and hardware NSS (cs_port=nullptr).
 */

#ifndef SRC_SPI_FLASH_H_
#define SRC_SPI_FLASH_H_

#include <stdint.h>
#include "main.h"

class SPIFlash {
 public:
  /* Application-level validity tag — written alongside data to verify integrity.
   * kValid: entry present, kClear: entry deleted/blank (FRAM erased state). */
  struct MagicBits {
    enum {
      kValid = 0x36f2,
      kClear = 0xFFFF,
    };
  };

  /* Wraps any data type with a magic validity tag for flash storage. */
  template <typename T>
  struct FlashEntry {
    T data;
    uint16_t magic;  /* SPIFlash::MagicBits::kValid when valid, kClear when deleted */
  };

  struct Commands {
    enum {
      kWriteEnable  = 0x06,  /* Set Write Enable Latch (WEL) */
      kWriteDisable = 0x04,  /* Reset WEL */
      kReadStatus   = 0x05,  /* Read Status Register */
      kWriteStatus  = 0x01,  /* Write Status Register */
      kRead         = 0x03,  /* Read Memory */
      kWrite        = 0x02,  /* Write Memory */
      kReadID       = 0x9F,  /* Read Device ID */
      kSleep        = 0xB9,  /* Enter Sleep Mode */
      kWake         = 0xAB,  /* Exit Sleep Mode (any CS pulse) */
    };
  };

  struct Status {
    enum {
      kWEL  = (1 << 1),  /* Write Enable Latch */
      kBP0  = (1 << 2),  /* Block Protect 0 */
      kBP1  = (1 << 3),  /* Block Protect 1 */
      kWPEN = (1 << 7),  /* Write Protect Enable */
    };
  };

  enum {
    kCapacity  = 32768,   /* 32KB */
    kAddrMask  = 0x7FFF,  /* 15-bit address */
    kTimeoutMs = 10,
  };

  /* Template wrapper: write/read any struct to/from flash address */
  template <typename T>
  bool write(uint32_t addr, const T& data) {
    return write(addr, reinterpret_cast<const uint8_t*>(&data), sizeof(T));
  }

  template <typename T>
  bool read(uint32_t addr, T& data) {
    return read(addr, reinterpret_cast<uint8_t*>(&data), sizeof(T));
  }

  SPIFlash();
  ~SPIFlash();

  bool init(SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port, uint16_t cs_pin);

  bool write(uint32_t addr, const uint8_t* data, uint16_t len);
  bool read(uint32_t addr, uint8_t* data, uint16_t len);

  bool sleep(void);
  bool wake(void);

  bool readDeviceID(uint8_t *id, uint8_t len);  /* Verify: Cypress=0x7F C2 2800 */

 private:
  SPI_HandleTypeDef *hspi;
  GPIO_TypeDef *cs_port;
  uint16_t cs_pin;
  bool initialized;

  bool setWriteEnable(void);

  inline void select(void) {
    if (cs_port) {
      HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_RESET);
    }
    /* Hardware NSS: HAL_SPI_Transmit enables SPI -> NSS low automatically */
  }
  inline void deselect(void) {
    if (cs_port) {
      HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);
    } else if (hspi) {
      /* Hardware NSS: disable SPI -> NSS high */
      while (__HAL_SPI_GET_FLAG(hspi, SPI_FLAG_BSY)) {}
      __HAL_SPI_DISABLE(hspi);
    }
  }
};

extern SPIFlash spi_flash;

#endif /* SRC_SPI_FLASH_H_ */
