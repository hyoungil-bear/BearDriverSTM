/*
 * EStop_SCurve.c
 *
 * Adapted from TI BearDriver for STM32G474.
 * Changes: _iq30 → float, _IQ30mpy → float multiply
 */

#include "EStop_SCurve.h"
#include <math.h>

void EStop_SCurve_Init(ESTOP_SCURVE *estop_s) {
  estop_s->S_Curve_step1_cnt = 0;
  estop_s->S_Curve_step2_cnt = 0;
  estop_s->status_flag = NOT_DONE;
  estop_s->init_flag = NOT_DONE;
  estop_s->first_step_done_flag = NOT_DONE;
  estop_s->second_step_done_flag = NOT_DONE;
  estop_s->E_stop_command = 0;
  estop_s->first_step_target_acc = 0.0f;
  estop_s->first_step_zerk = 0.0f;
  estop_s->first_step_acc = 0.0f;
  estop_s->second_step_acc = 0.0f;
  estop_s->second_step_zerk = 0.0f;
  estop_s->command_velocity = 0.0f;
  estop_s->command_acc = 0.0f;
  estop_s->decel_boundary_vel = 0.0f;
}

void EStop_SCurve_Reset(ESTOP_SCURVE *estop_s) { EStop_SCurve_Init(estop_s); }

// Generate trapezoidal and S-curve velocity profiles based on current velocity
// and acceleration.
void Calculate_EStop_Scurve(ESTOP_SCURVE *estop_s,
                            float current_vel, float current_acc) {
  float diff_vel = 0.0f;
  float diff_first_acc = 0.0f;
  float diff_secnd_acc = 0.0f;
  float decel_boudary = 0.0f;
  float f_second_step_zerk = 0.0f;

  if (current_vel >= 0.0f &&
      estop_s->init_flag == NOT_DONE) {  // if current velocity > 0 and didn't
                                         // initialize parameters
    estop_s->first_step_zerk = FIRST_STEP_ZERK;
    estop_s->first_step_target_acc = -FIRST_STEP_TARGET_ACC;
    estop_s->first_step_acc = current_acc;
    estop_s->command_velocity = current_vel;
    estop_s->decel_boundary_vel = current_vel;
    estop_s->second_step_acc = estop_s->first_step_target_acc;
    estop_s->init_flag = DONE;
  } else if (current_vel < 0.0f && estop_s->init_flag == NOT_DONE) {
    estop_s->first_step_zerk = FIRST_STEP_ZERK;
    estop_s->first_step_target_acc = FIRST_STEP_TARGET_ACC;
    estop_s->first_step_acc = current_acc;
    estop_s->command_velocity = current_vel;
    estop_s->decel_boundary_vel = current_vel;
    estop_s->second_step_acc = estop_s->first_step_target_acc;
    estop_s->init_flag = DONE;
  }

  decel_boudary = DECEL_BOUNDARY(DECEL_BOUNDARY_SPEED);

  diff_vel = FINAL_TARGET_SPEED - estop_s->command_velocity;

  if (diff_vel > decel_boudary ||
      diff_vel < -decel_boudary) {  // if the velocity before Scurve motion start
    diff_first_acc = estop_s->first_step_target_acc - estop_s->first_step_acc;

    if (diff_first_acc >= estop_s->first_step_zerk ||
        diff_first_acc <= -estop_s->first_step_zerk) {
      if (estop_s->first_step_target_acc >= estop_s->first_step_acc) {
        estop_s->first_step_acc += estop_s->first_step_zerk;
      } else {
        estop_s->first_step_acc -= estop_s->first_step_zerk;
      }
    } else {
      estop_s->first_step_done_flag = DONE;
      estop_s->first_step_acc = estop_s->first_step_target_acc;
    }

    estop_s->command_velocity += estop_s->first_step_acc;
    estop_s->command_acc = estop_s->first_step_acc;
    estop_s->decel_boundary_vel = estop_s->command_velocity;
    estop_s->S_Curve_step1_cnt++;
  } else if (diff_vel <= decel_boudary && diff_vel >= -decel_boudary &&
             diff_vel != 0.0f) {
    if (estop_s->decel_boundary_vel != 0.0f) {
      f_second_step_zerk =
          (estop_s->first_step_target_acc / estop_s->decel_boundary_vel) *
          estop_s->first_step_target_acc * 0.5f;
    }

    if (f_second_step_zerk >= 1.0f ||
        f_second_step_zerk <= -1.0f ||
        f_second_step_zerk == 0.0f) {
      estop_s->second_step_done_flag = DONE;
      estop_s->second_step_acc = SECOND_STEP_TARGET_ACC;
    }

    estop_s->second_step_zerk = fabsf(f_second_step_zerk);

    diff_secnd_acc = SECOND_STEP_TARGET_ACC - estop_s->second_step_acc;

    if (diff_secnd_acc >= estop_s->second_step_zerk ||
        diff_secnd_acc <= -estop_s->second_step_zerk) {
      if (SECOND_STEP_TARGET_ACC >= estop_s->second_step_acc) {
        estop_s->second_step_acc += estop_s->second_step_zerk;
      } else {
        estop_s->second_step_acc -= estop_s->second_step_zerk;
      }
    } else {
      estop_s->second_step_done_flag = DONE;
      estop_s->second_step_acc = SECOND_STEP_TARGET_ACC;
    }

    estop_s->command_velocity += estop_s->second_step_acc;
    estop_s->command_acc = estop_s->second_step_acc;
    estop_s->S_Curve_step2_cnt++;

  } else if (diff_vel == 0.0f) {
    estop_s->status_flag = DONE;
    estop_s->command_acc = 0.0f;
  }

  if (estop_s->second_step_done_flag == DONE) {
    estop_s->command_velocity = FINAL_TARGET_SPEED;
    estop_s->command_acc = 0.0f;
  }
}
