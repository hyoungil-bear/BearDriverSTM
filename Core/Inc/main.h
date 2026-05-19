/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define po_INH1_2_Pin GPIO_PIN_2
#define po_INH1_2_GPIO_Port GPIOE
#define po_INH2_2_Pin GPIO_PIN_3
#define po_INH2_2_GPIO_Port GPIOE
#define po_INL1_2_Pin GPIO_PIN_4
#define po_INL1_2_GPIO_Port GPIOE
#define po_INL2_2_Pin GPIO_PIN_5
#define po_INL2_2_GPIO_Port GPIOE
#define po_INL3_2_Pin GPIO_PIN_6
#define po_INL3_2_GPIO_Port GPIOE
#define di_FAULT_2_Pin GPIO_PIN_9
#define di_FAULT_2_GPIO_Port GPIOF
#define ai_OA_OA_2_Pin GPIO_PIN_0
#define ai_OA_OA_2_GPIO_Port GPIOC
#define ai_OA_OB_2_Pin GPIO_PIN_1
#define ai_OA_OB_2_GPIO_Port GPIOC
#define ai_OA_OC_2_Pin GPIO_PIN_2
#define ai_OA_OC_2_GPIO_Port GPIOC
#define ai_Vref_Pin GPIO_PIN_3
#define ai_Vref_GPIO_Port GPIOC
#define po_INH3_2_Pin GPIO_PIN_2
#define po_INH3_2_GPIO_Port GPIOF
#define di_EQEPA_1_Pin GPIO_PIN_0
#define di_EQEPA_1_GPIO_Port GPIOA
#define di_EQEPB_1_Pin GPIO_PIN_1
#define di_EQEPB_1_GPIO_Port GPIOA
#define uart2_Debug_TX_Pin GPIO_PIN_2
#define uart2_Debug_TX_GPIO_Port GPIOA
#define uart2_Debug_RX_Pin GPIO_PIN_3
#define uart2_Debug_RX_GPIO_Port GPIOA
#define di_EQEPA_1A4_Pin GPIO_PIN_4
#define di_EQEPA_1A4_GPIO_Port GPIOA
#define dac1_Debug1_Pin GPIO_PIN_5
#define dac1_Debug1_GPIO_Port GPIOA
#define dac2_Debug2_Pin GPIO_PIN_6
#define dac2_Debug2_GPIO_Port GPIOA
#define do_LED_Err_Pin GPIO_PIN_7
#define do_LED_Err_GPIO_Port GPIOA
#define do_LED_Run_Pin GPIO_PIN_4
#define do_LED_Run_GPIO_Port GPIOC
#define do_nSTBY_1_Pin GPIO_PIN_5
#define do_nSTBY_1_GPIO_Port GPIOC
#define ai_OA_OC_1_Pin GPIO_PIN_0
#define ai_OA_OC_1_GPIO_Port GPIOB
#define ai_Thermistor_1_Pin GPIO_PIN_1
#define ai_Thermistor_1_GPIO_Port GPIOB
#define ai_Thermistor_2_Pin GPIO_PIN_2
#define ai_Thermistor_2_GPIO_Port GPIOB
#define ai_OA_OA_1_Pin GPIO_PIN_7
#define ai_OA_OA_1_GPIO_Port GPIOE
#define po_INL1_1_Pin GPIO_PIN_8
#define po_INL1_1_GPIO_Port GPIOE
#define po_INH1_1_Pin GPIO_PIN_9
#define po_INH1_1_GPIO_Port GPIOE
#define po_INL2_1_Pin GPIO_PIN_10
#define po_INL2_1_GPIO_Port GPIOE
#define po_INH2_1_Pin GPIO_PIN_11
#define po_INH2_1_GPIO_Port GPIOE
#define po_INL3_1_Pin GPIO_PIN_12
#define po_INL3_1_GPIO_Port GPIOE
#define po_INH3_1_Pin GPIO_PIN_13
#define po_INH3_1_GPIO_Port GPIOE
#define ai_OA_OB_1_Pin GPIO_PIN_14
#define ai_OA_OB_1_GPIO_Port GPIOE
#define di_FAULT_1_Pin GPIO_PIN_15
#define di_FAULT_1_GPIO_Port GPIOE
#define di_FLAG_1_Pin GPIO_PIN_10
#define di_FLAG_1_GPIO_Port GPIOB
#define di_HALLA_1_Pin GPIO_PIN_11
#define di_HALLA_1_GPIO_Port GPIOB
#define di_HALLB_1_Pin GPIO_PIN_12
#define di_HALLB_1_GPIO_Port GPIOB
#define di_HALLC_1_Pin GPIO_PIN_13
#define di_HALLC_1_GPIO_Port GPIOB
#define do_485_Dir_Pin GPIO_PIN_14
#define do_485_Dir_GPIO_Port GPIOB
#define po_Brake_2_Pin GPIO_PIN_15
#define po_Brake_2_GPIO_Port GPIOB
#define uart3_485_TX_Pin GPIO_PIN_8
#define uart3_485_TX_GPIO_Port GPIOD
#define uart3_485_RX_Pin GPIO_PIN_9
#define uart3_485_RX_GPIO_Port GPIOD
#define ai_Voltage_Pin GPIO_PIN_12
#define ai_Voltage_GPIO_Port GPIOD
#define ai_Current_Pin GPIO_PIN_13
#define ai_Current_GPIO_Port GPIOD
#define di_nESTOP_IN_Pin GPIO_PIN_14
#define di_nESTOP_IN_GPIO_Port GPIOD
#define uart1_Base_TX_Pin GPIO_PIN_9
#define uart1_Base_TX_GPIO_Port GPIOA
#define uart1_Base_RX_Pin GPIO_PIN_10
#define uart1_Base_RX_GPIO_Port GPIOA
#define CAN1_RX_Pin GPIO_PIN_11
#define CAN1_RX_GPIO_Port GPIOA
#define CAN1_TX_Pin GPIO_PIN_12
#define CAN1_TX_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define SPI3_FLASH_Pin GPIO_PIN_15
#define SPI3_FLASH_GPIO_Port GPIOA
#define spi3_SCK_Pin GPIO_PIN_10
#define spi3_SCK_GPIO_Port GPIOC
#define SPI3_MISO_Pin GPIO_PIN_11
#define SPI3_MISO_GPIO_Port GPIOC
#define SPI3_MOSI_Pin GPIO_PIN_12
#define SPI3_MOSI_GPIO_Port GPIOC
#define po_Brake_1_Pin GPIO_PIN_1
#define po_Brake_1_GPIO_Port GPIOD
#define di_EQEPA_2_Pin GPIO_PIN_3
#define di_EQEPA_2_GPIO_Port GPIOD
#define di_EQEPB_2_Pin GPIO_PIN_4
#define di_EQEPB_2_GPIO_Port GPIOD
#define di_HALLC_2_Pin GPIO_PIN_5
#define di_HALLC_2_GPIO_Port GPIOD
#define di_HALLB_2_Pin GPIO_PIN_6
#define di_HALLB_2_GPIO_Port GPIOD
#define di_HALLA_2_Pin GPIO_PIN_7
#define di_HALLA_2_GPIO_Port GPIOD
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define di_rev_B0_Pin GPIO_PIN_4
#define di_rev_B0_GPIO_Port GPIOB
#define di_rev_B1_Pin GPIO_PIN_5
#define di_rev_B1_GPIO_Port GPIOB
#define pi_EQEPA_2_Pin GPIO_PIN_6
#define pi_EQEPA_2_GPIO_Port GPIOB
#define di_rev_B2_Pin GPIO_PIN_7
#define di_rev_B2_GPIO_Port GPIOB
#define do_nSTBY_2_Pin GPIO_PIN_9
#define do_nSTBY_2_GPIO_Port GPIOB
#define di_FLAG_2_Pin GPIO_PIN_0
#define di_FLAG_2_GPIO_Port GPIOE
#define di_rev_B3_Pin GPIO_PIN_1
#define di_rev_B3_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
