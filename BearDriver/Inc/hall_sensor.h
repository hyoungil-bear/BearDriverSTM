/*
 * hall_sensor.h
 *
 * Hall sensor interface for direction detection and phase angle validation.
 * Adapted from TI BearDriver for STM32G474.
 * Changes: HAL_Handle → GPIO port/pin, _iq → float, class structure preserved
 *
 * Note: TI original included low-speed 6-Step BLDC ↔ FOC auto-switching.
 *       This port uses encoder-only FOC; BLDC control code has been removed.
 *       Only hall state detection and phase angle validation are retained.
 */

#ifndef INCLUDE_HALL_SENSOR_H_
#define INCLUDE_HALL_SENSOR_H_

#include <stdint.h>
#include "main.h"

/* Hall map index (motor wiring configuration) */
#ifndef HALL_MAP_IDX
#define HALL_MAP_IDX 0
#endif

#ifdef __cplusplus

class HALLSensor {
 public:
  //! \brief  Direction of rotation
  typedef enum Direction_ {
    kDirectionUnchanged = 0,  //!< No state change detected
    kDirectionPositive,       //!< Direction in increasing angle
    kDirectionNegative,       //!< Direction in decreasing angle
    kDirectionInvalid         //!< Direction is unknown/invalid
  } Direction;

 private:
  uint16_t hall_GpioData;
  int16_t hall_State;
  int16_t hall_PrevState;

  Direction hall_dir;

  const uint16_t *hall_PwmIndex;
  uint32_t hall_Hall_Map;

  bool hall_Flag_State_Change;

  /* STM32-specific: Hall sensor GPIO pins */
  GPIO_TypeDef *hall_gpio_port_a;
  uint16_t hall_gpio_pin_a;
  GPIO_TypeDef *hall_gpio_port_b;
  uint16_t hall_gpio_pin_b;
  GPIO_TypeDef *hall_gpio_port_c;
  uint16_t hall_gpio_pin_c;

 public:
  float hall_angle_pu;  //!< Hall-derived electrical angle (pu, 60° steps)

 public:
  HALLSensor(void);

  void setup(GPIO_TypeDef *portA, uint16_t pinA,
             GPIO_TypeDef *portB, uint16_t pinB,
             GPIO_TypeDef *portC, uint16_t pinC);

  //! \brief  Read hall GPIOs and determine rotation direction.
  //!         Returns kDirectionInvalid when hall_State is 0 or 7.
  Direction HALLBLDC_State_Check();

  //! \brief  Convert hall state to electrical angle (pu, 60° resolution).
  float getHallAnglePU(void);

  inline int16_t getState(void) { return hall_State; };
  inline Direction getDirection(void) { return hall_dir; };
  inline bool getStateChanged(void) { return hall_Flag_State_Change; };
};

#endif /* __cplusplus */

#endif /* INCLUDE_HALL_SENSOR_H_ */
