/*
 * debug.c
 *
 * Debug output utility implementation.
 * See debug.h for API and pin mapping.
 *
 * DAC outputs:
 *   dac1_Debug1 → DAC1_CH2 → PA5  (channel 1, hdac1)
 *   dac2_Debug2 → DAC2_CH1 → PA6  (channel 2, hdac2)
 *
 * LED outputs:
 *   do_LED_Err  → PA7  (active-high)
 *   do_LED_Run  → PC4  (active-high)
 */

#include "debug.h"
#include "dac.h"    /* hdac1, hdac2                                      */
#include "main.h"   /* do_LED_Err_Pin/Port, do_LED_Run_Pin/Port defines  */

/* ── Initialisation ─────────────────────────────────────────────────────── */

void Debug_Init(void)
{
    HAL_DAC_Start(&hdac1, DAC_CHANNEL_2);  /* PA5 (dac1_Debug1) */
    HAL_DAC_Start(&hdac2, DAC_CHANNEL_1);  /* PA6 (dac2_Debug2) */

    /* LEDs off at startup */
    HAL_GPIO_WritePin(do_LED_Err_GPIO_Port, do_LED_Err_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(do_LED_Run_GPIO_Port, do_LED_Run_Pin, GPIO_PIN_RESET);
}

/* ── DAC output ─────────────────────────────────────────────────────────── */

void Debug_DAC_SetVoltage(uint8_t channel, float value, float offset)
{
    float voltage = value + offset;

    /* Clamp to [0 V, VREF] */
    if (voltage < 0.0f)             { voltage = 0.0f; }
    if (voltage > DBG_DAC_VREF_V)   { voltage = DBG_DAC_VREF_V; }

    uint32_t dac_count = (uint32_t)((voltage / DBG_DAC_VREF_V)
                                    * (float)DBG_DAC_RESOLUTION);

    switch (channel)
    {
        case 1:
            HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, dac_count);
            break;
        case 2:
            HAL_DAC_SetValue(&hdac2, DAC_CHANNEL_1, DAC_ALIGN_12B_R, dac_count);
            break;
        default:
            break;
    }
}

/* ── LED output ─────────────────────────────────────────────────────────── */

void Debug_LED_Err_Set(bool on)
{
    HAL_GPIO_WritePin(do_LED_Err_GPIO_Port, do_LED_Err_Pin,
                      on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void Debug_LED_Err_Toggle(void)
{
    HAL_GPIO_TogglePin(do_LED_Err_GPIO_Port, do_LED_Err_Pin);
}

void Debug_LED_Run_Set(bool on)
{
    HAL_GPIO_WritePin(do_LED_Run_GPIO_Port, do_LED_Run_Pin,
                      on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void Debug_LED_Run_Toggle(void)
{
    HAL_GPIO_TogglePin(do_LED_Run_GPIO_Port, do_LED_Run_Pin);
}

/* ── GPIO test-point output (no-op on this hardware) ────────────────────── */

void Debug_TP_Write(uint8_t channel, uint8_t data)
{
    (void)channel;
    (void)data;
}

void Debug_TP_Toggle(uint8_t channel)
{
    (void)channel;
}
