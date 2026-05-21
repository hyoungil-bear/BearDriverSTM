/*
 * debug.h
 *
 * Debug output utility for oscilloscope / logic analyser use.
 *
 * ┌──────────────────────────────────────────────────────┐
 * │  DAC analogue output                                  │
 * │    dac1_Debug1 → DAC1_CH2 → PA5  (12-bit, 3.3 V)   │
 * │    dac2_Debug2 → DAC2_CH1 → PA6  (12-bit, 3.3 V)   │
 * ├──────────────────────────────────────────────────────┤
 * │  LED digital output                                   │
 * │    do_LED_Err  → PA7  (Error indicator)              │
 * │    do_LED_Run  → PC4  (Run / heartbeat)              │
 * └──────────────────────────────────────────────────────┘
 *
 * Initialisation:
 *   Debug_Init();   // call once after MX_DAC1_Init(), MX_DAC2_Init(), MX_GPIO_Init()
 */

#ifndef DEBUG_H
#define DEBUG_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/* ── DAC configuration ──────────────────────────────────────────────────── */
#define DBG_DAC_VREF_V      (3.3f)   /* VDDA reference voltage [V]          */
#define DBG_DAC_RESOLUTION  (4095U)  /* 12-bit DAC full-scale count          */

/* ── Initialisation ─────────────────────────────────────────────────────── */

/**
 * @brief  Start DAC channels and set LEDs to their default (off) state.
 *         Call once after MX_DAC1_Init(), MX_DAC2_Init(), and MX_GPIO_Init().
 */
void Debug_Init(void);

/* ── DAC output ─────────────────────────────────────────────────────────── */

/**
 * @brief  Output a voltage on the selected DAC channel.
 *
 * @param  channel  1 → dac1_Debug1  DAC1_CH2 (PA5)
 *                  2 → dac2_Debug2  DAC2_CH1 (PA6)
 * @param  value    Signal value [V].  Actual output = value + offset.
 * @param  offset   DC offset [V].
 *                  Use offset = 1.65f to centre a bipolar signal.
 * @note   Output is clamped to [0 V, 3.3 V].
 */
void Debug_DAC_SetVoltage(uint8_t channel, float value, float offset);

/* ── LED output ─────────────────────────────────────────────────────────── */

/**
 * @brief  Set Error LED state.
 * @param  on  true → LED on,  false → LED off
 */
void Debug_LED_Err_Set(bool on);

/**
 * @brief  Toggle Error LED.
 */
void Debug_LED_Err_Toggle(void);

/**
 * @brief  Set Run LED state.
 * @param  on  true → LED on,  false → LED off
 */
void Debug_LED_Run_Set(bool on);

/**
 * @brief  Toggle Run LED.
 */
void Debug_LED_Run_Toggle(void);

/* ── GPIO test-point output (no-op on this hardware) ────────────────────── */

/**
 * @brief  Set a test-point GPIO pin.  No-op — no TP pins on BearDriverSTM.
 */
void Debug_TP_Write(uint8_t channel, uint8_t data);

/**
 * @brief  Toggle a test-point GPIO pin.  No-op — no TP pins on BearDriverSTM.
 */
void Debug_TP_Toggle(uint8_t channel);

#ifdef __cplusplus
}
#endif

#endif /* DEBUG_H */
