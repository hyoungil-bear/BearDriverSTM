/*
 * hall_sensor.cpp
 *
 * Hall sensor interface for direction detection and phase angle validation.
 * Adapted from TI BearDriver for STM32G474.
 * Changes: HAL_Handle → GPIO port/pin, _iq → float
 *
 * Note: TI original included low-speed 6-Step BLDC ↔ FOC auto-switching.
 *       This port uses encoder-only FOC; BLDC control code has been removed.
 */
#include <stdint.h>
#include "hall_sensor.h"

static const uint16_t hall_map[6][7] = {
    {0, 1, 5, 6, 3, 2, 4}, {0, 2, 6, 1, 4, 3, 5}, {0, 3, 1, 2, 5, 4, 6},
    {0, 4, 2, 3, 6, 5, 1}, {0, 5, 3, 4, 1, 6, 2}, {0, 6, 4, 5, 2, 1, 3}};

HALLSensor::HALLSensor(void)
    : hall_GpioData(0)
    , hall_State(0)
    , hall_PrevState(-1)
    , hall_dir(kDirectionUnchanged)
    , hall_Hall_Map(HALL_MAP_IDX)
    , hall_Flag_State_Change(false)
    , hall_gpio_port_a(nullptr)
    , hall_gpio_pin_a(0)
    , hall_gpio_port_b(nullptr)
    , hall_gpio_pin_b(0)
    , hall_gpio_port_c(nullptr)
    , hall_gpio_pin_c(0)
    , hall_angle_pu(0.0f)
{
  hall_PwmIndex = &hall_map[hall_Hall_Map][0];
}

void HALLSensor::setup(GPIO_TypeDef *portA, uint16_t pinA,
                       GPIO_TypeDef *portB, uint16_t pinB,
                       GPIO_TypeDef *portC, uint16_t pinC)
{
  this->hall_gpio_port_a = portA;
  this->hall_gpio_pin_a  = pinA;
  this->hall_gpio_port_b = portB;
  this->hall_gpio_pin_b  = pinB;
  this->hall_gpio_port_c = portC;
  this->hall_gpio_pin_c  = pinC;
}

HALLSensor::Direction HALLSensor::HALLBLDC_State_Check(void) {
  // Read Hall GPIOs (active-low, matching original ~HAL_readGpio)
  hall_GpioData =
      ((HAL_GPIO_ReadPin(hall_gpio_port_c, hall_gpio_pin_c) == GPIO_PIN_RESET) ? 1U : 0U) << 2;
  hall_GpioData +=
      ((HAL_GPIO_ReadPin(hall_gpio_port_b, hall_gpio_pin_b) == GPIO_PIN_RESET) ? 1U : 0U) << 1;
  hall_GpioData +=
      ((HAL_GPIO_ReadPin(hall_gpio_port_a, hall_gpio_pin_a) == GPIO_PIN_RESET) ? 1U : 0U);

  hall_State = hall_GpioData;

  // Bounds checking. It must be checked first.
  if (hall_State == 7 || hall_State == 0) {
    hall_PrevState = -1;  // Restart direction calculation
    hall_dir = kDirectionInvalid;
    return hall_dir;
  }

  // If this is the first measurement return without setting direction
  if (hall_PrevState < 0) {
    hall_PrevState = hall_State;
    return kDirectionUnchanged;
  }

  // A valid state is measured. Calculate direction.
  if (hall_State != hall_PrevState) {
    int16_t hall_State_delta =
        hall_PwmIndex[hall_State] - hall_PwmIndex[hall_PrevState];

    if ((hall_State_delta == -1) || (hall_State_delta == 5)) {
      hall_dir = kDirectionPositive;
    } else if ((hall_State_delta == 1) || (hall_State_delta == -5)) {
      hall_dir = kDirectionNegative;
    } else {
      hall_dir = kDirectionInvalid;  // More than 1 state transition
    }

    hall_PrevState = hall_State;
    hall_Flag_State_Change = true;
  } else {
    hall_dir = kDirectionUnchanged;
    hall_Flag_State_Change = false;
  }

  return hall_dir;
}

float HALLSensor::getHallAnglePU(void) {
  // reverse direction to match current/pwm sign
  float angle =
      1.0f - (float)hall_PwmIndex[hall_State] * (1.0f / 6.0f);

  hall_angle_pu = angle;

  return angle;
}
