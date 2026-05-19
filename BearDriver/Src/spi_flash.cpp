/*
 * spi_flash.cpp
 *
 * FM25V02A SPI FRAM driver for STM32G474.
 * Rewritten from NOR Flash driver: no erase, no busy-wait, no page boundary.
 *
 * Write sequence : WREN → WRITE(addr, data) — WEL auto-clears after CS deassert
 * Read  sequence : READ(addr) → receive data
 * No busy polling required (FRAM write completes within SCK cycle)
 */

#include "spi_flash.h"
#include <string.h>

SPIFlash spi_flash;

SPIFlash::SPIFlash()
    : hspi(nullptr)
    , cs_port(nullptr)
    , cs_pin(0)
    , initialized(false)
{
}

SPIFlash::~SPIFlash()
{
}

/* --------------------------------------------------------------------------
 * init
 * Verify communication by reading Device ID.
 * FM25V02A JEDEC ID: 0x7F 0x7F 0x7F 0x7F 0x7F 0x7F 0xC2 0x22 0x08
 * Check manufacturer byte (0xC2 = Cypress) at index 6 after 6 continuation bytes.
 * Simplified: just check that response is not all-0x00 or all-0xFF.
 * -------------------------------------------------------------------------- */
bool SPIFlash::init(SPI_HandleTypeDef *hspi_in, GPIO_TypeDef *cs_port_in,
                    uint16_t cs_pin_in)
{
  if (hspi_in == nullptr) {
    return false;
  }

  hspi     = hspi_in;
  cs_port  = cs_port_in;
  cs_pin   = cs_pin_in;

  deselect();

  uint8_t id[9] = {0};
  bool ok = readDeviceID(id, sizeof(id));

  /* Validate: not all 0x00 or 0xFF, manufacturer byte (id[6]) = 0xC2 */
  if (ok && id[6] == 0xC2) {
    initialized = true;
    return true;
  }

  return false;
}

/* --------------------------------------------------------------------------
 * readDeviceID
 * FM25V02A returns 9 bytes: 6x 0x7F (JEDEC continuation) + 0xC2 + 0x22 + 0x08
 * -------------------------------------------------------------------------- */
bool SPIFlash::readDeviceID(uint8_t *id, uint8_t len)
{
  uint8_t cmd = Commands::kReadID;

  select();
  HAL_StatusTypeDef status = HAL_SPI_Transmit(hspi, &cmd, 1, kTimeoutMs);
  if (status == HAL_OK) {
    status = HAL_SPI_Receive(hspi, id, len, kTimeoutMs);
  }
  deselect();

  return (status == HAL_OK);
}

/* --------------------------------------------------------------------------
 * setWriteEnable
 * Must be called immediately before each write (WEL auto-clears after CS rise).
 * -------------------------------------------------------------------------- */
bool SPIFlash::setWriteEnable(void)
{
  uint8_t cmd = Commands::kWriteEnable;

  select();
  HAL_StatusTypeDef status = HAL_SPI_Transmit(hspi, &cmd, 1, kTimeoutMs);
  deselect();

  return (status == HAL_OK);
}

/* --------------------------------------------------------------------------
 * write
 * addr: 0x0000 – 0x7FFF (15-bit, 32KB)
 * len : number of bytes; no page boundary restriction (FRAM)
 * -------------------------------------------------------------------------- */
bool SPIFlash::write(uint32_t addr, const uint8_t *data, uint16_t len)
{
  if (!initialized || data == nullptr || len == 0) {
    return false;
  }
  if (addr + len > kCapacity) {
    return false;  /* Address out of range */
  }

  if (!setWriteEnable()) {
    return false;
  }

  uint8_t cmd_buf[3];
  cmd_buf[0] = Commands::kWrite;
  cmd_buf[1] = (addr >> 8) & 0xFF;  /* A[14:8] */
  cmd_buf[2] =  addr       & 0xFF;  /* A[7:0]  */

  select();
  HAL_StatusTypeDef status = HAL_SPI_Transmit(hspi, cmd_buf, 3, kTimeoutMs);
  if (status == HAL_OK) {
    /* HAL_SPI_Transmit takes non-const pData but does not modify it */
    status = HAL_SPI_Transmit(hspi, const_cast<uint8_t*>(data), len, kTimeoutMs);
  }
  deselect();
  /* WEL auto-clears on CS rise — no WRDI needed */

  return (status == HAL_OK);
}

/* --------------------------------------------------------------------------
 * read
 * addr: 0x0000 – 0x7FFF (15-bit, 32KB)
 * -------------------------------------------------------------------------- */
bool SPIFlash::read(uint32_t addr, uint8_t *data, uint16_t len)
{
  if (!initialized || data == nullptr || len == 0) {
    return false;
  }
  if (addr + len > kCapacity) {
    return false;  /* Address out of range */
  }

  uint8_t cmd_buf[3];
  cmd_buf[0] = Commands::kRead;
  cmd_buf[1] = (addr >> 8) & 0xFF;  /* A[14:8] */
  cmd_buf[2] =  addr       & 0xFF;  /* A[7:0]  */

  select();
  HAL_StatusTypeDef status = HAL_SPI_Transmit(hspi, cmd_buf, 3, kTimeoutMs);
  if (status == HAL_OK) {
    status = HAL_SPI_Receive(hspi, data, len, kTimeoutMs);
  }
  deselect();

  return (status == HAL_OK);
}

/* --------------------------------------------------------------------------
 * sleep / wake
 * sleep(): enter low-power mode (~10uA @ 3.3V)
 * wake():  exit via CS pulse + dummy read (tREC = 400us max)
 * -------------------------------------------------------------------------- */
bool SPIFlash::sleep(void)
{
  uint8_t cmd = Commands::kSleep;

  select();
  HAL_StatusTypeDef status = HAL_SPI_Transmit(hspi, &cmd, 1, kTimeoutMs);
  deselect();

  return (status == HAL_OK);
}

bool SPIFlash::wake(void)
{
  /* CS low pulse wakes the device; tREC ≤ 400us */
  uint8_t cmd = Commands::kWake;

  select();
  HAL_StatusTypeDef status = HAL_SPI_Transmit(hspi, &cmd, 1, kTimeoutMs);
  deselect();

  /* Wait tREC: 400us max (use HAL_Delay(1) = 1ms, conservative) */
  HAL_Delay(1);

  return (status == HAL_OK);
}
