/*
 * Ebrake.cpp
 *
 * Fail-safe electromagnetic brake driver for DRV8871DDAR H-bridge.
 * See Ebrake.h for hardware description and usage.
 *
 * Brake type: spring-engaged, electrically-disengaged.
 *   engage()    → cut PWM → coil off → spring ENGAGES brake (motor locked)
 *   disengage() → energise coil → spring overcome          (motor free)
 */

#include "Ebrake.h"

/* Minimum Vbus (V) below which duty calculation is skipped.
 * Prevents division-by-zero and unrealistic duty values during power-up. */
static constexpr float kMinVbusV = 5.0f;

/* -------------------------------------------------------------------------*/
/* Construction                                                              */
/* -------------------------------------------------------------------------*/

Ebrake::Ebrake(void)
    : htim(nullptr)
    , channel(0U)
    , rated_voltage_V(kDefaultRatedVoltageV)
    , hold_voltage_V(kDefaultHoldVoltageV)
    , rated_duration_ms(kDefaultRatedDurationMs)
    , state(kStateEngaged)
    , disengage_tick_ms(0U)
    , vbus_lpf_V(0.0f)
{
}

/* -------------------------------------------------------------------------*/
/* Setup                                                                     */
/* -------------------------------------------------------------------------*/

void Ebrake::setup(TIM_HandleTypeDef *htim, uint32_t channel)
{
  this->htim    = htim;
  this->channel = channel;

  if (htim == nullptr) {
    return;
  }

  /* Start PWM output — for advanced timers (TIM8, TIM15) HAL_TIM_PWM_Start()
   * also sets the MOE (Main Output Enable) bit required to activate outputs. */
  HAL_TIM_PWM_Start(htim, channel);

  /* Safe initial state: duty = 0 → IN1=0 → DRV8871 coast → coil off → brake ENGAGED. */
  setDuty(0.0f);
}

void Ebrake::setVoltages(float rated_V, float hold_V, uint32_t duration_ms)
{
  rated_voltage_V   = rated_V;
  hold_voltage_V    = hold_V;
  rated_duration_ms = duration_ms;
}

/* -------------------------------------------------------------------------*/
/* Disengage / Engage                                                        */
/* -------------------------------------------------------------------------*/

void Ebrake::disengage(void)
{
  /* Energise coil at rated voltage to overcome the spring.
   * vbus_lpf_V is kept warm by runVoltageControl() in all states. */
  setDuty(voltsToDuty(rated_voltage_V, vbus_lpf_V));
  disengage_tick_ms = HAL_GetTick();
  state             = kStateRated;
}

void Ebrake::engage(void)
{
  /* Cut PWM → coil de-energised → spring engages → motor locked. */
  setDuty(0.0f);
  state = kStateEngaged;
}

/* -------------------------------------------------------------------------*/
/* Periodic update                                                           */
/* -------------------------------------------------------------------------*/

void Ebrake::runVoltageControl(float vbus_V)
{
  /* Step 1: Update Vbus LPF (always, regardless of state).
   * α=0.95 @ 10ms → τ ≈ 195 ms. Filters ADC noise and Vbus fluctuations. */
  vbus_lpf_V = kVbusLpfAlpha * vbus_lpf_V
             + (1.0f - kVbusLpfAlpha) * vbus_V;

  /* Step 2: Voltage phase management using filtered Vbus. */
  switch (state) {

    case kStateRated:
      setDuty(voltsToDuty(rated_voltage_V, vbus_lpf_V));

      /* Step down to holding voltage after rated duration elapses. */
      if ((HAL_GetTick() - disengage_tick_ms) >= rated_duration_ms) {
        state = kStateHolding;
        setDuty(voltsToDuty(hold_voltage_V, vbus_lpf_V));
      }
      break;

    case kStateHolding:
      setDuty(voltsToDuty(hold_voltage_V, vbus_lpf_V));
      break;

    case kStateEngaged:
    default:
      break;
  }
}

/* -------------------------------------------------------------------------*/
/* Private helpers                                                           */
/* -------------------------------------------------------------------------*/

void Ebrake::setDuty(float duty)
{
  if (htim == nullptr) {
    return;
  }

  if (duty < 0.0f) { duty = 0.0f; }
  if (duty > 1.0f) { duty = 1.0f; }

  __HAL_TIM_SET_COMPARE(htim, channel,
      (uint32_t)(duty * (float)htim->Instance->ARR));
}

float Ebrake::voltsToDuty(float target_V, float vbus_V) const
{
  if (vbus_V < kMinVbusV) {
    return 0.0f;
  }

  float duty = target_V / vbus_V;

  if (duty > 1.0f) { duty = 1.0f; }
  if (duty < 0.0f) { duty = 0.0f; }

  return duty;
}

/* end of file */
