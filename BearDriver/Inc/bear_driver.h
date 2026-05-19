/*
 * bear_driver.h
 *
 * Main application state machine and system-level declarations.
 * Adapted from TI BearDriver for STM32G474.
 * Changes: HAL_Handle → STM32, TI-specific PIE/CPU removed, C-compatible wrapper
 *
 * Note: This header is extern "C" since main.c (CubeMX) is a C file.
 */

#ifndef BEAR_DRIVER_H
#define BEAR_DRIVER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

// **************************************************************************
// the defines

#define ESTOP_DEBOUNCE_THRESHOLD  100  // ~22ms at 4.5kHz loop rate

// **************************************************************************
// the typedefs

//! \brief  Main application state machine (matches original)
typedef enum MAIN_STATE_t {
  STATE_INIT,
  STATE_CALC_OFFSETS,
  STATE_RUN,
  STATE_SS2_ESTOP,
  STATE_FAULT,
  STATE_FAULT_RESTART,
  STATE_ESTOP,
  STATE_ESTOP_RESTART,
  STATE_STALL_LOCK
} MAIN_STATE;

// **************************************************************************
// the functions (callable from C main.c)

//! \brief  Main application entry point (never returns)
void BearDriver_Main(void);

//! \brief  Get current state machine state
MAIN_STATE getMainState(void);

//! \brief  Set state machine state
void setMainState(MAIN_STATE state);

//! \brief  Disable both motors
void disableMotors(void);

//! \brief  Motor 1 ISR handler (call from ADC/TIM ISR)
void Motor1_ISR_Handler(void);

//! \brief  Motor 2 ISR handler (call from ADC/TIM ISR)
void Motor2_ISR_Handler(void);

//! \brief  Read ADC injected values and run Motor1 ISR
//!         ADC3(PhA) + ADC4(PhB), triggered by TIM20_TRGO
void Motor1_ADC_ReadAndISR(void);

//! \brief  Read ADC injected values and run Motor2 ISR
//!         ADC1(PhA,PhB), triggered by TIM1_TRGO
void Motor2_ADC_ReadAndISR(void);

//! \brief  Read bus voltage/current from ADC5 and thermistors.
//!         Call from SysTick at 1kHz. Pattern: read-then-start (no callback).
void BearDriver_SlowADC_Update(void);

#ifdef __cplusplus
}
#endif

#endif /* BEAR_DRIVER_H */
