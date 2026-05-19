/**
  ******************************************************************************
  * @file    cpu_time.h
  * @brief   ISR execution time measurement using ARM DWT cycle counter
  ******************************************************************************
  * @attention
  *
  * Replaces TI MotorWare CPU_TIME module (cpu_time.h / cpu_time.c).
  * Uses ARM Cortex-M4 DWT->CYCCNT instead of TI CPU Timer 2.
  *
  * Usage:
  *   1. Call CPU_TIME_init() once at startup (after DWT->CTRL CYCCNTENA set).
  *   2. Call CPU_TIME_start() at ISR entry.
  *   3. Call CPU_TIME_end()   at ISR exit.
  *   4. Read cpu_time_m1 / cpu_time_m2 from debugger or API.
  *
  ******************************************************************************
  */

#ifndef CPU_TIME_H
#define CPU_TIME_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* DWT and CoreDebug are defined in CMSIS core_cm4.h, included via
   stm32g4xx_hal.h → stm32g4xx.h → stm32g474xx.h → core_cm4.h.
   This header must be included AFTER main.h or stm32g4xx_hal.h. */

/**
  * @brief  Per-motor ISR timing data
  */
typedef struct {
  uint32_t start_cnt;      /*!< DWT->CYCCNT at ISR entry                     */
  uint32_t isr_cycles;     /*!< Last ISR execution cycles                    */
  uint32_t isr_cycles_max; /*!< Peak ISR execution cycles (worst case)       */
  uint32_t pwm_period_cyc; /*!< PWM period in CPU cycles (= SYSCLK / PWM_Hz)*/
  float    cpu_usage;      /*!< CPU usage ratio (0.0 ~ 1.0), updated per ISR */
  float    cpu_usage_max;  /*!< Peak CPU usage ratio (0.0 ~ 1.0)             */
} CPU_TIME_Obj;

/**
  * @brief  Initialize CPU_TIME object
  * @param  obj           : pointer to CPU_TIME_Obj
  * @param  sysclk_hz     : system clock frequency (e.g. 170000000)
  * @param  pwm_freq_hz   : PWM / ISR frequency (e.g. 10000)
  */
static inline void CPU_TIME_init(CPU_TIME_Obj *obj,
                                 uint32_t sysclk_hz,
                                 uint32_t pwm_freq_hz) {
  obj->start_cnt      = 0;
  obj->isr_cycles     = 0;
  obj->isr_cycles_max = 0;
  obj->pwm_period_cyc = sysclk_hz / pwm_freq_hz;
  obj->cpu_usage      = 0.0f;
  obj->cpu_usage_max  = 0.0f;
}

/**
  * @brief  Record ISR entry timestamp
  * @param  obj : pointer to CPU_TIME_Obj
  */
static inline void CPU_TIME_start(CPU_TIME_Obj *obj) {
  obj->start_cnt = DWT->CYCCNT;
}

/**
  * @brief  Record ISR exit timestamp and compute elapsed cycles
  * @param  obj : pointer to CPU_TIME_Obj
  */
static inline void CPU_TIME_end(CPU_TIME_Obj *obj) {
  uint32_t elapsed = DWT->CYCCNT - obj->start_cnt;  /* unsigned wraparound safe */
  obj->isr_cycles = elapsed;
  if (elapsed > obj->isr_cycles_max) {
    obj->isr_cycles_max = elapsed;
  }
  obj->cpu_usage = (float)elapsed / (float)obj->pwm_period_cyc;
  if (obj->cpu_usage > obj->cpu_usage_max) {
    obj->cpu_usage_max = obj->cpu_usage;
  }
}

/**
  * @brief  Get CPU usage ratio (0.0 ~ 1.0)
  * @param  obj : pointer to CPU_TIME_Obj
  * @return ISR cycles / PWM period cycles
  */
static inline float CPU_TIME_getUsage(const CPU_TIME_Obj *obj) {
  return (float)obj->isr_cycles / (float)obj->pwm_period_cyc;
}

/**
  * @brief  Get peak CPU usage ratio (0.0 ~ 1.0)
  * @param  obj : pointer to CPU_TIME_Obj
  * @return Peak ISR cycles / PWM period cycles
  */
static inline float CPU_TIME_getUsageMax(const CPU_TIME_Obj *obj) {
  return (float)obj->isr_cycles_max / (float)obj->pwm_period_cyc;
}

/**
  * @brief  Reset peak value
  * @param  obj : pointer to CPU_TIME_Obj
  */
static inline void CPU_TIME_resetMax(CPU_TIME_Obj *obj) {
  obj->isr_cycles_max = 0;
  obj->cpu_usage_max  = 0.0f;
}

#ifdef __cplusplus
}
#endif

#endif /* CPU_TIME_H */
