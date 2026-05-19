/*
 * EStop_SCurve.h
 *
 * S-curve deceleration profile for emergency stop (E-Stop)
 * ISO 13849-1 Safe Stop 2 (SS2) functionality.
 *
 * Adapted from TI BearDriver for STM32G474.
 * Changes: _iq30 → float (Cortex-M4 FPU)
 */

#ifndef INCLUDE_ESTOP_SCURVE_H_
#define INCLUDE_ESTOP_SCURVE_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define FIRST_STEP_ZERK       0.0001f     // jerk (rpm/tick²)
#define FIRST_STEP_TARGET_ACC 0.08f
// range 0.02 to 0.1 (0.02 : long time to stop, 0.1 :
// short time to stop)
#define SECOND_STEP_TARGET_ACC 0.0f
#define FINAL_TARGET_SPEED     0.0f
#define DECEL_BOUNDARY_SPEED   0.2f  // m/s
#define DECEL_BOUNDARY(mPers) \
  ((mPers) / (0.18f * 3.14159f) * 60.0f)  // RPM

#define NOT_DONE 0
#define DONE     1

typedef struct {
  uint32_t S_Curve_step1_cnt;
  uint32_t S_Curve_step2_cnt;
  int16_t status_flag;
  int16_t init_flag;
  int16_t first_step_done_flag;
  int16_t second_step_done_flag;

  int16_t E_stop_command;

  float first_step_target_acc;
  float first_step_zerk;
  float first_step_acc;

  float second_step_acc;
  float second_step_zerk;

  float command_velocity;
  float command_acc;
  float decel_boundary_vel;
} ESTOP_SCURVE;

void EStop_SCurve_Init(ESTOP_SCURVE *estop_s);
void EStop_SCurve_Reset(ESTOP_SCURVE *estop_s);
void Calculate_EStop_Scurve(ESTOP_SCURVE *estop_s,
                            float current_vel, float current_acc);

#ifdef __cplusplus
}
#endif

#endif /* INCLUDE_ESTOP_SCURVE_H_ */
