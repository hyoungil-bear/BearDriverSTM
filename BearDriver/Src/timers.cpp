/**
  ******************************************************************************
  * @file    timers.cpp
  * @author  Bear Robotics Motor Control Team
  * @brief   Software timer management implementation
  ******************************************************************************
  * @attention
  *
  * This file implements software timer management for periodic tasks.
  * Adapted from TI BearDriver for STM32G474.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "timers.h"
#include "main.h"
#include "gpio.h"
#include "sci_coms.h"
#include <string.h>

/** @addtogroup BearDriver
  * @{
  */

/** @addtogroup Timers
  * @{
  */

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/**
  * @brief  ISR flag for timer tick
  */
static volatile bool isr_flag = false;

/**
  * @brief  Timer definitions array
  */
static TIMER_DEF timers[TIMER_END] = {
  {0, 0, false, false, false},  // TIMER_INVALID - dummy timer
  {INIT_TIMER_PERIOD / TIMER_BASE_MS, 0, false, false, false},  // TIMER_INIT
  {SPI_TIMER_PERIOD / TIMER_BASE_MS, 0, false, false, false},   // TIMER_SPI_COMS
  {CMD_TIMER_PERIOD / TIMER_BASE_MS, 0, false, false, false},   // TIMER_CMD
  {STALL_TIMER_PERIOD / TIMER_BASE_MS, 0, false, false, false}, // TIMER_STALL_DETECTION
  {SHORTBRAKE_TIMER_PERIOD / TIMER_BASE_MS, 0, false, false, false} // TIMER_SHORT_BRAKE
};

/**
  * @brief  LED blink counters
  */
static volatile uint32_t LED_blink_count = 0;
static volatile uint32_t LED_flash_rate = 0;

/* Public variables ----------------------------------------------------------*/

//! \brief  10ms tick counter. Incremented every 10ms in Timers_10ms_Callback().
//!         Matches TI timerCounter_10ms pattern: Motor::run() checks this to
//!         trigger torque/temperature computation at 10ms rate.
uint32_t timerCounter_10ms = 0;

/* External variables --------------------------------------------------------*/
extern volatile uint32_t base_com_check_timer_10ms;

/* Private function prototypes -----------------------------------------------*/
static void Timers_Update10ms(void);
static void Timers_UpdateSingle(TIMER_ID timerId);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Update a single timer
  * @param  timerId: Timer identifier
  * @retval None
  */
static void Timers_UpdateSingle(TIMER_ID timerId)
{
  if (timers[timerId].enabled) {
    if (timers[timerId].counter > 0) {
      timers[timerId].counter--;
    }

    if (timers[timerId].counter == 0) {
      timers[timerId].flag = true;

      if (timers[timerId].auto_reset) {
        timers[timerId].counter = timers[timerId].period;
      } else {
        timers[timerId].enabled = false;
      }
    }
  }
}

/**
  * @brief  10ms periodic timer update
  * @note   Called from ISR callback
  * @retval None
  */
static void Timers_Update10ms(void)
{
  /* Update all timers */
  for (int t = 1; t < TIMER_END; t++) {
    Timers_UpdateSingle((TIMER_ID)t);
  }

  /* Update LED blink */
  if (LED_blink_count > 0) {
    LED_blink_count--;
  }

  if (LED_blink_count == 0) {
    LED_blink_count = LED_flash_rate;

    HAL_GPIO_TogglePin(do_LED_Run_GPIO_Port, do_LED_Run_Pin);
  }

  /* Update SCI communication check timer */
  if (base_com_check_timer_10ms > 0) {
    base_com_check_timer_10ms--;

    if (base_com_check_timer_10ms == 0) {
      /* Communication error - try to recover every 5 seconds */
      base_com_check_timer_10ms = BASE_COM_CHECK_TIMER_10MS_ERROR;
      SCI_Init();  /* Reinitialize SCI */
    }
  }
}

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  Initialize timer system
  * @retval None
  */
void Timers_Init(void)
{
  /* Clear ISR flag */
  isr_flag = false;

  /* Initialize all timers to default state */
  memset(timers, 0, sizeof(timers));

  /* Set timer periods */
  timers[TIMER_INIT].period = INIT_TIMER_PERIOD / TIMER_BASE_MS;
  timers[TIMER_SPI_COMS].period = SPI_TIMER_PERIOD / TIMER_BASE_MS;
  timers[TIMER_CMD].period = CMD_TIMER_PERIOD / TIMER_BASE_MS;
  timers[TIMER_STALL_DETECTION].period = STALL_TIMER_PERIOD / TIMER_BASE_MS;
  timers[TIMER_SHORT_BRAKE].period = SHORTBRAKE_TIMER_PERIOD / TIMER_BASE_MS;

  /* Set LED blink rate to fast initially (until motors initialized) */
  LED_flash_rate = (LED_BLINK_PERIOD_MS / TIMER_BASE_MS) / LED_FAST_DIVISOR;
  LED_blink_count = LED_flash_rate;
}

/**
  * @brief  Process timer updates (call from main loop)
  * @retval None
  */
void Timers_Process(void)
{
  if (isr_flag) {
    isr_flag = false;
    Timers_Update10ms();
  }
}

/**
  * @brief  10ms timer ISR callback
  * @note   Called from TIM interrupt handler
  * @retval None
  */
void Timers_10ms_Callback(void)
{
  timerCounter_10ms++;  // matches TI do10ms() pattern
  isr_flag = true;
}

/**
  * @brief  Start or restart a timer
  * @param  timerId: Timer identifier
  * @retval None
  */
void Timers_Start(TIMER_ID timerId)
{
  if (timerId > TIMER_INVALID && timerId < TIMER_END) {
    timers[timerId].counter = timers[timerId].period;
    timers[timerId].flag = false;
    timers[timerId].enabled = true;
  }
}

/**
  * @brief  Stop a running timer
  * @param  timerId: Timer identifier
  * @retval None
  */
void Timers_Stop(TIMER_ID timerId)
{
  if (timerId > TIMER_INVALID && timerId < TIMER_END) {
    timers[timerId].counter = 0;
    timers[timerId].flag = false;
    timers[timerId].enabled = false;
  }
}

/**
  * @brief  Check if timer has expired
  * @note   Calling this function clears the timer flag
  * @param  timerId: Timer identifier
  * @retval true if timer expired, false otherwise
  */
bool Timers_Check(TIMER_ID timerId)
{
  if (timerId > TIMER_INVALID && timerId < TIMER_END) {
    bool status = timers[timerId].flag;
    timers[timerId].flag = false;
    return status;
  }

  return false;
}

/**
  * @brief  Set LED blink rate
  * @param  rate: LED_FAST or LED_SLOW
  * @retval None
  */
void Timers_SetLedBlinkRate(LED_BLINK_RATE_e rate)
{
  if (rate == LED_FAST) {
    LED_flash_rate = (LED_BLINK_PERIOD_MS / TIMER_BASE_MS) / LED_FAST_DIVISOR;
  } else {
    LED_flash_rate = LED_BLINK_PERIOD_MS / TIMER_BASE_MS;
  }

  LED_blink_count = LED_flash_rate;
}

/**
  * @brief  Get timer remaining count
  * @param  timerId: Timer identifier
  * @retval Remaining count in ticks (0 if expired or invalid)
  */
uint32_t Timers_GetRemaining(TIMER_ID timerId)
{
  if (timerId > TIMER_INVALID && timerId < TIMER_END) {
    return timers[timerId].counter;
  }

  return 0;
}

/**
  * @brief  Check if timer is enabled
  * @param  timerId: Timer identifier
  * @retval true if enabled, false otherwise
  */
bool Timers_IsEnabled(TIMER_ID timerId)
{
  if (timerId > TIMER_INVALID && timerId < TIMER_END) {
    return timers[timerId].enabled;
  }

  return false;
}

/**
  * @}
  */

/**
  * @}
  */

/******************* (C) COPYRIGHT Bear Robotics *****END OF FILE****/
