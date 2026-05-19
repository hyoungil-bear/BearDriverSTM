/**
  ******************************************************************************
  * @file    version.h
  * @author  Bear Robotics Motor Control Team
  * @brief   Firmware version definitions
  ******************************************************************************
  * @attention
  *
  * This file contains firmware version number definitions.
  * Update these values for each release.
  *
  ******************************************************************************
  */

/*
History:
6.0.0.0 2024-10-15 hyoungil - Major update for new project
7.0.0.0 2026-01-20 - STM32G474 port from TI C2000
*/

#ifndef VERSION_H
#define VERSION_H

#ifdef __cplusplus
extern "C" {
#endif

/** @addtogroup BearDriver
  * @{
  */

/** @addtogroup Version
  * @{
  */

/* Exported constants --------------------------------------------------------*/

/**
  * @brief  Firmware version numbers
  */
#define MAJOR_VERSION   7    /*!< Major version - significant changes */
#define MINOR_VERSION   0    /*!< Minor version - feature additions */
#define REVISION        0    /*!< Revision - bug fixes */
#define BUILD           0    /*!< Build number - incremental builds */

/**
  * @brief  Version string for display
  */
#define VERSION_STRING  "7.0.0.0"

/**
  * @brief  Platform identifier
  */
#define PLATFORM_NAME   "STM32G474"

/**
  * @brief  Project name
  */
#define PROJECT_NAME    "BearDriver_STM32_EV"

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* VERSION_H */

/******************* (C) COPYRIGHT Bear Robotics *****END OF FILE****/
