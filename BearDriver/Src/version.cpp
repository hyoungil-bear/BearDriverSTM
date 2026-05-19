/**
  ******************************************************************************
  * @file    version.cpp
  * @author  Bear Robotics Motor Control Team
  * @brief   Firmware version implementation
  ******************************************************************************
  * @attention
  *
  * This file implements version information functions.
  * Adapted from TI BearDriver for STM32G474.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "version.h"
#include "version_info.h"
#include <stdio.h>

/** @addtogroup BearDriver
  * @{
  */

/** @addtogroup Version
  * @{
  */

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/**
  * @brief  Global version information structure
  */
VERSION_INFO_t version = {
  { MAJOR_VERSION, MINOR_VERSION, REVISION, BUILD }
};

/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  Print version information
  * @note   Uses printf which outputs to USART2 (debug console)
  * @retval None
  */
void Version_Print(void)
{
  printf("\n");
  printf("========================================\n");
  printf("  %s\n", PROJECT_NAME);
  printf("  Platform: %s\n", PLATFORM_NAME);
  printf("========================================\n");
  printf("  Firmware Version: %d.%d.%d.%d\n",
         version.parts.major,
         version.parts.minor,
         version.parts.revision,
         version.parts.build);
  printf("========================================\n");
  printf("\n");
}

/**
  * @brief  Get version information
  * @retval Pointer to version info structure
  */
VERSION_INFO_t* Version_GetInfo(void)
{
  return &version;
}

/**
  * @}
  */

/**
  * @}
  */

/******************* (C) COPYRIGHT Bear Robotics *****END OF FILE****/
