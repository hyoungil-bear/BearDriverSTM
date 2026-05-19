/*
 * debug.h
 *
 * Debug output utility for oscilloscope / logic analyser use.
 *
 * ┌──────────────────────────────────────────────────────┐
 * │  DAC analogue output                                  │
 * │    DAC1 CH1 → PA4 (DBG_DAC_CH1)                      │
 * │    DAC1 CH2 → PA5 (DBG_DAC_CH2)                      │
 * │    12-bit, VREF = 3.3 V, range [0 V, 3.3 V]         │
 * ├──────────────────────────────────────────────────────┤
 * │  GPIO test-point output                               │
 * │    TP CH1  → PB5  (TP1,   OUTPUT PP, HIGH speed)     │
 * │    TP CH2  → PA12 (TP2,   OUTPUT PP, HIGH speed)     │
 * └──────────────────────────────────────────────────────┘
 *
 * Initialisation:
 *   Debug_Init();   // call once after MX_DAC1_Init() and MX_GPIO_Init()
 */

#ifndef DEBUG_H
#define DEBUG_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* ── DAC configuration ──────────────────────────────────────────────────── */
#define DBG_DAC_VREF_V      (3.3f)   /* VDDA reference voltage [V]          */
#define DBG_DAC_RESOLUTION  (4095U)  /* 12-bit DAC full-scale count          */

/* ── Initialisation ─────────────────────────────────────────────────────── */

/**
 * @brief  Start DAC1 CH1 and CH2.
 *         Call once after MX_DAC1_Init().
 */
void Debug_Init(void);

/* ── DAC output ─────────────────────────────────────────────────────────── */

/**
 * @brief  Output a voltage on the selected DAC channel.
 *
 * @param  channel  1 → DAC1_OUT1 (PA4)
 *                  2 → DAC1_OUT2 (PA5)
 * @param  value    Signal value [V].  Actual output = value + offset.
 * @param  offset   DC offset [V].
 *                  Use offset = 1.65f to centre a bipolar signal.
 * @note   Output is clamped to [0 V, 3.3 V].
 */
void Debug_DAC_SetVoltage(uint8_t channel, float value, float offset);

/* ── GPIO test-point output ─────────────────────────────────────────────── */

/**
 * @brief  Set a test-point GPIO pin.
 *
 * @param  channel  1 → TP1 (PB5)
 *                  2 → TP2 (PA12)
 * @param  data     0 → LOW,  non-zero → HIGH
 */
void Debug_TP_Write(uint8_t channel, uint8_t data);

/**
 * @brief  Toggle a test-point GPIO pin.
 *
 * @param  channel  1 → TP1 (PB5)
 *                  2 → TP2 (PA12)
 */
void Debug_TP_Toggle(uint8_t channel);

#ifdef __cplusplus
}
#endif

#endif /* DEBUG_H */
