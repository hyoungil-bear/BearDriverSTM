/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32g4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32g4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "gpio.h"
#include <stdint.h>
#include "stm32g4xx_hal.h"
#include "sci_coms.h"
#include "bear_driver.h"
#include "timers.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
volatile uint32_t ISR_SysTick_Count = 0;
volatile uint32_t ISR_ADC1_2_Count = 0;
volatile uint32_t ISR_ADC3_Count = 0;
volatile uint32_t ISR_TIM1_BRK_TIM15_Count = 0;
volatile uint32_t ISR_TIM2_Count = 0;
volatile uint32_t ISR_TIM5_Count = 0;
volatile uint32_t ISR_TIM20_BRK_Count = 0;
volatile uint32_t ISR_TIM3_Count = 0;
volatile uint32_t ISR_TIM4_Count = 0;
volatile uint32_t ISR_USART2_Count = 0;
volatile uint32_t ISR_USART3_Count = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// DWT-based microsecond delay function (uses hardware cycle counter)
static inline void delay_us(uint32_t us)
{
  uint32_t start = DWT->CYCCNT;
  uint32_t cycles = us * (SystemCoreClock / 1000000);
  while ((DWT->CYCCNT - start) < cycles);
}

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern ADC_HandleTypeDef hadc3;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim15;
extern TIM_HandleTypeDef htim20;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
/* USER CODE BEGIN EV */
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */
  ISR_SysTick_Count++;

  // Read previous Vbus regular ADC result and start next conversion (1kHz)
  BearDriver_SlowADC_Update();

  // 10ms timer callback (SysTick = 1kHz, every 10 ticks = 10ms)
  static uint32_t systick_10ms_cnt = 0;
  if (++systick_10ms_cnt >= 10) {
    systick_10ms_cnt = 0;
    Timers_10ms_Callback();
  }
  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32G4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32g4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles ADC1 and ADC2 global interrupt.
  */
void ADC1_2_IRQHandler(void)
{
  /* USER CODE BEGIN ADC1_2_IRQn 0 */
  ISR_ADC1_2_Count++;
  /* USER CODE END ADC1_2_IRQn 0 */
  HAL_ADC_IRQHandler(&hadc1);
  HAL_ADC_IRQHandler(&hadc2);
  /* USER CODE BEGIN ADC1_2_IRQn 1 */

  /* USER CODE END ADC1_2_IRQn 1 */
}

/**
  * @brief This function handles TIM1 break interrupt and TIM15 global interrupt.
  */
void TIM1_BRK_TIM15_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_BRK_TIM15_IRQn 0 */
  ISR_TIM1_BRK_TIM15_Count++;
  /* USER CODE END TIM1_BRK_TIM15_IRQn 0 */
  if (htim1.Instance != NULL)
  {
    HAL_TIM_IRQHandler(&htim1);
  }
  if (htim15.Instance != NULL)
  {
    HAL_TIM_IRQHandler(&htim15);
  }
  /* USER CODE BEGIN TIM1_BRK_TIM15_IRQn 1 */

  /* USER CODE END TIM1_BRK_TIM15_IRQn 1 */
}

/**
  * @brief This function handles TIM1 update interrupt and TIM16 global interrupt.
  */
void TIM1_UP_TIM16_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_UP_TIM16_IRQn 0 */

  /* USER CODE END TIM1_UP_TIM16_IRQn 0 */
  if (htim1.Instance != NULL)
  {
    HAL_TIM_IRQHandler(&htim1);
  }
  /* USER CODE BEGIN TIM1_UP_TIM16_IRQn 1 */

  /* USER CODE END TIM1_UP_TIM16_IRQn 1 */
}

/**
  * @brief This function handles TIM2 global interrupt.
  */
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */
  ISR_TIM2_Count++;
  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */

  /* USER CODE END TIM2_IRQn 1 */
}

/**
  * @brief This function handles USART2 global interrupt / USART2 wake-up interrupt through EXTI line 26.
  */
void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */
  ISR_USART2_Count++;

  /* Clear error flags (ORE, FE, NE, PE) to prevent re-entry loop */
  if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_ORE)) {
    __HAL_UART_CLEAR_FLAG(&huart2, UART_CLEAR_OREF);
  }
  if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_FE)) {
    __HAL_UART_CLEAR_FLAG(&huart2, UART_CLEAR_FEF);
  }
  if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_NE)) {
    __HAL_UART_CLEAR_FLAG(&huart2, UART_CLEAR_NEF);
  }
  if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_PE)) {
    __HAL_UART_CLEAR_FLAG(&huart2, UART_CLEAR_PEF);
  }

  /* RXNE: read received byte and pass to SCI buffer */
  if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_RXNE) &&
      __HAL_UART_GET_IT_SOURCE(&huart2, UART_IT_RXNE)) {
    uint8_t data = (uint8_t)(huart2.Instance->RDR & 0xFFU);
    SCI_RxCallback(SCI_A_FD, data);
  }

  /* TXE: transmit next byte from SCI buffer */
  if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_TXE) &&
      __HAL_UART_GET_IT_SOURCE(&huart2, UART_IT_TXE)) {
    SCI_TxCallback(SCI_A_FD);
  }

  return;  /* Skip HAL generic handler -- SCI manages its own buffers */
  /* USER CODE END USART2_IRQn 0 */
  HAL_UART_IRQHandler(&huart2);
  /* USER CODE BEGIN USART2_IRQn 1 */

  /* USER CODE END USART2_IRQn 1 */
}

/**
  * @brief This function handles USART3 global interrupt / USART3 wake-up interrupt through EXTI line 28.
  */
void USART3_IRQHandler(void)
{
  /* USER CODE BEGIN USART3_IRQn 0 */
  ISR_USART3_Count++;

  /* Clear error flags (ORE, FE, NE, PE) */
  if (__HAL_UART_GET_FLAG(&huart3, UART_FLAG_ORE)) {
    __HAL_UART_CLEAR_FLAG(&huart3, UART_CLEAR_OREF);
  }
  if (__HAL_UART_GET_FLAG(&huart3, UART_FLAG_FE)) {
    __HAL_UART_CLEAR_FLAG(&huart3, UART_CLEAR_FEF);
  }
  if (__HAL_UART_GET_FLAG(&huart3, UART_FLAG_NE)) {
    __HAL_UART_CLEAR_FLAG(&huart3, UART_CLEAR_NEF);
  }
  if (__HAL_UART_GET_FLAG(&huart3, UART_FLAG_PE)) {
    __HAL_UART_CLEAR_FLAG(&huart3, UART_CLEAR_PEF);
  }

  /* RXNE: read received byte and pass to SCI buffer */
  if (__HAL_UART_GET_FLAG(&huart3, UART_FLAG_RXNE) &&
      __HAL_UART_GET_IT_SOURCE(&huart3, UART_IT_RXNE)) {
    uint8_t data = (uint8_t)(huart3.Instance->RDR & 0xFFU);
    SCI_RxCallback(SCI_B_FD, data);
  }

  /* TXE: transmit next byte from SCI buffer */
  if (__HAL_UART_GET_FLAG(&huart3, UART_FLAG_TXE) &&
      __HAL_UART_GET_IT_SOURCE(&huart3, UART_IT_TXE)) {
    SCI_TxCallback(SCI_B_FD);
  }

  return;  /* Skip HAL generic handler -- SCI manages its own buffers */
  /* USER CODE END USART3_IRQn 0 */
  HAL_UART_IRQHandler(&huart3);
  /* USER CODE BEGIN USART3_IRQn 1 */

  /* USER CODE END USART3_IRQn 1 */
}

/**
  * @brief This function handles ADC3 global interrupt.
  */
void ADC3_IRQHandler(void)
{
  /* USER CODE BEGIN ADC3_IRQn 0 */

  /* USER CODE END ADC3_IRQn 0 */
  HAL_ADC_IRQHandler(&hadc3);
  /* USER CODE BEGIN ADC3_IRQn 1 */

  /* USER CODE END ADC3_IRQn 1 */
}

/**
  * @brief This function handles TIM5 global interrupt.
  */
void TIM5_IRQHandler(void)
{
  /* USER CODE BEGIN TIM5_IRQn 0 */
  ISR_TIM5_Count++;
  /* USER CODE END TIM5_IRQn 0 */
  HAL_TIM_IRQHandler(&htim5);
  /* USER CODE BEGIN TIM5_IRQn 1 */

  /* USER CODE END TIM5_IRQn 1 */
}

/**
  * @brief This function handles TIM20 break interrupt.
  */
void TIM20_BRK_IRQHandler(void)
{
  /* USER CODE BEGIN TIM20_BRK_IRQn 0 */
  ISR_TIM20_BRK_Count++;
  /* USER CODE END TIM20_BRK_IRQn 0 */
  HAL_TIM_IRQHandler(&htim20);
  /* USER CODE BEGIN TIM20_BRK_IRQn 1 */

  /* USER CODE END TIM20_BRK_IRQn 1 */
}

/**
  * @brief This function handles TIM20 update interrupt.
  */
void TIM20_UP_IRQHandler(void)
{
  /* USER CODE BEGIN TIM20_UP_IRQn 0 */

  /* USER CODE END TIM20_UP_IRQn 0 */
  HAL_TIM_IRQHandler(&htim20);
  /* USER CODE BEGIN TIM20_UP_IRQn 1 */

  /* USER CODE END TIM20_UP_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/**
  * @brief  TIM3 global interrupt (encoder capture overflow / input capture).
  */
void TIM3_IRQHandler(void)
{
  ISR_TIM3_Count++;
  HAL_TIM_IRQHandler(&htim3);
}

/**
  * @brief  TIM4 global interrupt (encoder capture overflow / input capture).
  */
void TIM4_IRQHandler(void)
{
  ISR_TIM4_Count++;
  HAL_TIM_IRQHandler(&htim4);
}

/* HAL_ADCEx_InjectedConvCpltCallback is defined in bear_driver.cpp (extern "C") */

/* USER CODE END 1 */
