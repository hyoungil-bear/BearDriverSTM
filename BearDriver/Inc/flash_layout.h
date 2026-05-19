/*
 * flash_layout.h
 *
 * Flash memory layout for persistent configuration storage.
 * Adapted from TI BearDriver for STM32G474.
 * Changes: Uses SPIFlash::FlashEntry<T> template, same structure as original
 */

#ifndef FLASH_LAYOUT_H_
#define FLASH_LAYOUT_H_

#include "spi_flash.h"
#include "pid.h"
#include "differential_drive_limiter_helper.h"

struct FlashLayout {
  SPIFlash::FlashEntry<PID_CONFIG> pid;
  SPIFlash::FlashEntry<uint16_t> disable_motors_on_boot;
  SPIFlash::FlashEntry<DifferentialDriveLimiter_Params_t> kinematic_limits;
};

#endif /* FLASH_LAYOUT_H_ */
