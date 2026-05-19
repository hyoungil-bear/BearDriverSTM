/**
  ******************************************************************************
  * @file    timers.h
  * @author  Bear Robotics Motor Control Team
  * @brief   Software timer management for periodic tasks
  ******************************************************************************
  * @attention
  *
  * This file provides software timer management for periodic tasks.
  * Adapted from TI BearDriver for STM32G474.
  *
  ******************************************************************************
  */

#ifndef TIMERS_H
#define TIMERS_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>

/** @addtogroup BearDriver
  * @{
  */

/** @addtogroup Timers
  * @{
  */

/* Exported types ------------------------------------------------------------*/

/**
  * @brief  Timer identifier enumeration
  */
typedef enum
{
  TIMER_INVALID = 0,           /*!< Invalid timer ID */
  TIMER_INIT,                  /*!< Initialization timer */
  TIMER_SPI_COMS,              /*!< SPI communication timeout timer */
  TIMER_CMD,                   /*!< Command timeout timer */
  TIMER_STALL_DETECTION,       /*!< Stall detection periodic timer */
  TIMER_SHORT_BRAKE,           /*!< Short brake timeout timer */
  TIMER_END                    /*!< End marker (total number of timers) */
} TIMER_ID;

/**
  * @brief  LED blink rate enumeration
  */
typedef enum
{
  LED_FAST = 0,                /*!< Fast blink rate (125ms period) */
  LED_SLOW = 1                 /*!< Slow blink rate (500ms period) */
} LED_BLINK_RATE_e;

/**
  * @brief  Timer definition structure
  */
typedef struct
{
  uint32_t period;             /*!< Timer period in ticks */
  uint32_t counter;            /*!< Current counter value */
  bool enabled;                /*!< Timer enabled flag */
  bool auto_reset;             /*!< Auto-reset when expired */
  bool flag;                   /*!< Timer expiration flag */
} TIMER_DEF;

/* Exported constants --------------------------------------------------------*/

/* Timer base period */
#define TIMER_BASE_MS           10U    /*!< Base timer tick period (10ms) */

/* Timer periods (in milliseconds) */
#define INIT_TIMER_PERIOD       1000U  /*!< 1 second initialization delay */
#define SPI_TIMER_PERIOD        200U   /*!< 200ms SPI communication timeout */
#define CMD_TIMER_PERIOD        100U   /*!< 100ms command timeout */
#define STALL_TIMER_PERIOD      100U   /*!< 100ms stall detection check */
#define SHORTBRAKE_TIMER_PERIOD 30000U /*!< 30 second short brake timeout */

/* LED blink rates */
#define LED_BLINK_PERIOD_MS     500U   /*!< LED blink base period (500ms) */
#define LED_FAST_DIVISOR        4U     /*!< Fast blink divisor (125ms) */

/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/

/**
  * @brief  Initialize timer system
  * @note   Should be called once during system initialization
  * @retval None
  */
void Timers_Init(void);

/**
  * @brief  Process timer updates (call from main loop)
  * @note   Checks ISR flag and updates all timers if needed
  * @retval None
  */
void Timers_Process(void);

/**
  * @brief  10ms timer ISR callback
  * @note   Called from TIM interrupt, updates all software timers
  * @retval None
  */
void Timers_10ms_Callback(void);

/**
  * @brief  Start or restart a timer
  * @param  timerId: Timer identifier
  * @retval None
  */
void Timers_Start(TIMER_ID timerId);

/**
  * @brief  Stop a running timer
  * @param  timerId: Timer identifier
  * @retval None
  */
void Timers_Stop(TIMER_ID timerId);

/**
  * @brief  Check if timer has expired
  * @note   Calling this function clears the timer flag
  * @param  timerId: Timer identifier
  * @retval true if timer expired, false otherwise
  */
bool Timers_Check(TIMER_ID timerId);

/**
  * @brief  Set LED blink rate
  * @param  rate: LED_FAST or LED_SLOW
  * @retval None
  */
void Timers_SetLedBlinkRate(LED_BLINK_RATE_e rate);

/**
  * @brief  Get timer remaining count
  * @param  timerId: Timer identifier
  * @retval Remaining count in ticks (0 if expired or invalid)
  */
uint32_t Timers_GetRemaining(TIMER_ID timerId);

/**
  * @brief  Check if timer is enabled
  * @param  timerId: Timer identifier
  * @retval true if enabled, false otherwise
  */
bool Timers_IsEnabled(TIMER_ID timerId);

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* TIMERS_H */

/******************* (C) COPYRIGHT Bear Robotics *****END OF FILE****/
