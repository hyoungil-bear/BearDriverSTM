/**
  ******************************************************************************
  * @file    version_info.h
  * @author  Bear Robotics Motor Control Team
  * @brief   Version information structure and function prototypes
  ******************************************************************************
  * @attention
  *
  * This file provides version information structure for firmware versioning.
  * Adapted from TI BearDriver for STM32G474.
  *
  ******************************************************************************
  */

#ifndef VERSION_INFO_H
#define VERSION_INFO_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdio.h>

/** @addtogroup BearDriver
  * @{
  */

/** @addtogroup Version
  * @{
  */

/* Exported types ------------------------------------------------------------*/

/**
  * @brief  Version information union
  * @note   Provides multiple ways to access version components
  */
typedef union
{
  struct
  {
    uint16_t major;      /*!< Major version number */
    uint16_t minor;      /*!< Minor version number */
    uint16_t revision;   /*!< Revision number */
    uint16_t build;      /*!< Build number */
  } parts;               /*!< Individual version parts */

  struct
  {
    uint32_t major_minor;     /*!< Combined major.minor */
    uint32_t revision_build;  /*!< Combined revision.build */
  } groups;                   /*!< Grouped version parts */

  uint64_t version;      /*!< Full version as 64-bit value */
} VERSION_INFO_t;

/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/

/**
  * @brief  Print version information
  * @param  stream: File stream pointer (for printf compatibility)
  * @note   On STM32, pass NULL and uses printf to UART
  * @retval None
  */
void Version_Print(void);

/**
  * @brief  Get version information
  * @retval Pointer to version info structure
  */
VERSION_INFO_t* Version_GetInfo(void);

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* VERSION_INFO_H */

/******************* (C) COPYRIGHT Bear Robotics *****END OF FILE****/
