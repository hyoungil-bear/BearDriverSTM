/*
 * debug.c
 *
 * Debug output utility implementation.
 * See debug.h for API and pin mapping.
 *
 * BearDriverSTM: No dedicated test-point GPIOs (TP1/TP2) available.
 * DAC outputs: DAC1_CH2 (PA5), DAC2_CH1 (PA6).
 */

#include "debug.h"
#include "dac.h"    /* hdac1, hdac2                 */
#include "main.h"

/* ── Initialisation ─────────────────────────────────────────────────────── */

void Debug_Init(void)
{
    HAL_DAC_Start(&hdac1, DAC_CHANNEL_2);  /* PA5 (dac1_Debug1) */
    HAL_DAC_Start(&hdac2, DAC_CHANNEL_1);  /* PA6 (dac2_Debug2) */
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

/* ── GPIO test-point output ─────────────────────────────────────────────── */
/* BearDriverSTM has no dedicated TP pins — functions are no-ops. */

void Debug_TP_Write(uint8_t channel, uint8_t data)
{
    (void)channel;
    (void)data;
}

void Debug_TP_Toggle(uint8_t channel)
{
    (void)channel;
}
