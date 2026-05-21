/*
 * bldc_motor.cpp
 *
 * Motor controller class implementation.
 * Adapted from TI BearDriver for STM32G474.
 * Changes: _iq → float, HAL → STM32, EST/CTRL → inline FOC,
 *          DRV_SPI_8323 → STDRIVE102BH (GPIO, no SPI)
 */
#include <stdint.h>
#include <math.h>
#include <string.h>

#include "main.h"
#include "bldc_motor.h"
#include "user_params.h"

// 10ms counter from timers.cpp (matches TI timerCounter_10ms pattern)
extern uint32_t timerCounter_10ms;
static uint32_t timerCounter_prev = 0;  // shared across both motors (file-scope)

// Mathematical constants
#define PI_F          3.14159265358979f
#define TWO_PI_F      (2.0f * PI_F)
#define SQRT3_F       1.73205080756888f
#define SQRT3_INV_F   0.57735026918963f
#define TWO_THIRDS_F  0.66666666666667f

// **************************************************************************
// Constructor

Motor::Motor(HAL_MtrSelect_e mtrNum)
    :
    // public members (declaration order in bldc_motor.h)
    Flag_Run_Identify(false)
    , Flag_Run_Identify_cmd(false)
    , Flag_enableOffsetcalc(false)
    , Flag_bypassFaultCheck(false)
    , Flag_needCurrentDecay(false)
    , hwVer(0)
    , IqRef_A(0.0f)
    , SpeedSet_rpm(0.0f)
    , SpeedRef_rpm(0.0f)
    , SpeedTraj_rpm(0.0f)
    , MaxAccel_rpmps(200.0f)
    , Speed_rpm(0.0f)
    , angle_pu(0.0f)
    , speed_pid_out(0.0f)
    , currentBwCoeff(0.25f)
    , Kp_spd(0.08f)
    , Ki_spd(2.0f)
    , Kout(1.0f)
    , Kd_spd(0.0008f)
    , Kff_spd(0.1f)
    , dN_spd(200.0f)
    , VdcBus_Volt(0.0f)
    , offsetCalcCount(0)
    , encoder()
    , mtrNum(mtrNum)
    , enablePIDLog(false)
    , stall_detection_count(0)
    , stall_threshold_zero_speed(0.0f)
    , stall_threshold_non_zero_speed(0.0f)
    , stall_threshold_high_current(0.0f)
    , motor_torque_Nm(0.0f)
    , motor_temperature_DegC(-273.15f)  // No NTC sensor on STM32 HW; matches TI invalid sentinel
    , kCurrentDecayFactor_(powf(0.5f, 4.3219f / USER_CTRL_FREQ_Hz))
    , current_decay_(1.0f)
    // private members (declaration order)
    , userParams_(NULL)
    , hallsensor_phasefault_count(0)
    , hallsensor_fault_count(0)
    , error_code(kErrorNone)
    , stCntSpeed(0)
    , stCntEncoder(0)
    , traj_target(0.0f)
    , traj_intValue(0.0f)
    , traj_minValue(0.0f)
    , traj_maxValue(0.0f)
    , traj_K(1.0f)
    , traj_maxDelta(0.0f)
    , traj_dValue(0.0f)
    , htim_pwm(nullptr)
    , hadc1(nullptr)
    , hadc2(nullptr)
{
  MATH_vec3 vec3_0 = {{0.0f, 0.0f, 0.0f}};
  MATH_vec2 vec2_0 = {{0.0f, 0.0f}};

  pwmData.Tabc = vec3_0;
  adcData.I = vec3_0;
  adcData.dcBus = 0.0f;

  offsets_I_A = vec3_0;
  Iabc_A = vec3_0;
  Iabc_fbk_A = vec3_0;

  idq_ref = vec2_0;
  vdq_out = vec2_0;
  idq = vec2_0;

  pidHandle[PID_Speed] = NULL;
  pidHandle[PID_Iq] = NULL;
  pidHandle[PID_Id] = NULL;

  windupCount[PID_Speed] = 0;
  windupCount[PID_Iq] = 0;
  windupCount[PID_Id] = 0;

  windupThreshold[PID_Speed] = MOTOR_WINDUP_SPEED_FAULT_THRESHOLD;
  windupThreshold[PID_Iq] = MOTOR_WINDUP_CURRENT_FAULT_THRESHOLD;
  windupThreshold[PID_Id] = MOTOR_WINDUP_CURRENT_FAULT_THRESHOLD;

  memset(offsetFilter, 0, sizeof(offsetFilter));

  setTrajK(1.0f);
}

// **************************************************************************
// PID Setup

void Motor::pidSetup(USER_Params *userParams) {
  if (!userParams) {
    return;
  }
  userParams_ = userParams;

  // Static voltage limit: 50 % of hardware-max Vbus, in Volts
  float maxVoltage_V = USER_MAX_VS_MAG_PU * VBUS_ADC_TO_VOLT;

  float IsrPeriod_sec =
      1.0e-6f * userParams->pwmPeriod_usec * userParams->numPwmTicksPerIsrTick;
  float Ls_d = userParams->motor_Ls_d;
  float Ls_q = userParams->motor_Ls_q;
  float Rs = userParams->motor_Rs;

  float ts =
      userParams->ctrlPeriod_sec * userParams->numCtrlTicksPerSpeedTick;

  float RoverLs_d = Rs / Ls_d;
  float RoverLs_q = Rs / Ls_q;

  // Current PID gains in physical units: Kp [V/A], Ki [dimensionless]
  //   Kp = α·Ls/Ts,  Ki = (Rs/Ls)·Ts  (pole cancellation)
  //   α = currentBwCoeff → bandwidth ≈ α/(2π·Ts) Hz
  float Kp_Id = currentBwCoeff * Ls_d / IsrPeriod_sec;   // [V/A]
  float Ki_Id = RoverLs_d * IsrPeriod_sec;

  float Kp_Iq = currentBwCoeff * Ls_q / IsrPeriod_sec;   // [V/A]
  float Ki_Iq = RoverLs_q * IsrPeriod_sec;

  // Speed Controller
  pidHandle[PID_Speed] = PID_init(&pid[PID_Speed], sizeof(pid[PID_Speed]));

  // Id current controller
  pidHandle[PID_Id] = PID_init(&pid[PID_Id], sizeof(pid[PID_Id]));

  // Iq current controller
  pidHandle[PID_Iq] = PID_init(&pid[PID_Iq], sizeof(pid[PID_Iq]));

  stCntSpeed = 0;
  stCntEncoder = 0;

  // Speed PID: input is RPM error, output is Iq_ref [A].
  // Gains are in SI units [A/RPM] — no PU scaling needed.
  PID_setTS(pidHandle[PID_Speed], ts);
  PID_setGains(pidHandle[PID_Speed],
               Kp_spd, Ki_spd, Kd_spd, Kff_spd,
               dN_spd, Kout);

  // Speed PID output limit [A]
  float spdMinMax = userParams->maxCurrent;

  PID_setMinMax(pidHandle[PID_Speed], -spdMinMax, spdMinMax);
  PID_setUiMinMax(pidHandle[PID_Speed], -spdMinMax, spdMinMax);
  PID_setUi(pidHandle[PID_Speed], 0.0f);

  // D-term limit: 10 % of maxCurrent to suppress derivative kick.
  // Chosen well above the normal D contribution at low speed (< 0.1 A)
  // but below P-term saturation (~8 A at 100 RPM error).
  PID_setUdMax(pidHandle[PID_Speed], spdMinMax * 0.1f);

  // Id PID
  PID_setTS(pidHandle[PID_Id], IsrPeriod_sec);
  PID_setGains(pidHandle[PID_Id], Kp_Id, Ki_Id, 0.0f, 0.0f, 0.0f, 1.0f);

  PID_setMinMax(pidHandle[PID_Id], -maxVoltage_V, maxVoltage_V);
  PID_setUiMinMax(pidHandle[PID_Id], -maxVoltage_V, maxVoltage_V);
  PID_setUi(pidHandle[PID_Id], 0.0f);

  // Iq PID
  PID_setTS(pidHandle[PID_Iq], IsrPeriod_sec);
  PID_setGains(pidHandle[PID_Iq], Kp_Iq, Ki_Iq, 0.0f, 0.0f, 0.0f, 1.0f);

  PID_setMinMax(pidHandle[PID_Iq], -maxVoltage_V, maxVoltage_V);
  PID_setUiMinMax(pidHandle[PID_Iq], -maxVoltage_V, maxVoltage_V);
  PID_setUi(pidHandle[PID_Iq], 0.0f);

}

// **************************************************************************
// FOC Transforms

void Motor::ClarkeTransform(const MATH_vec3 &abc, MATH_vec2 &ab) {
  // Clarke: alpha = (2*a - b - c)/3, beta = (b - c)/sqrt(3)
  ab.value[0] = TWO_THIRDS_F * abc.value[0]
              - (1.0f / 3.0f) * abc.value[1]
              - (1.0f / 3.0f) * abc.value[2];
  ab.value[1] = SQRT3_INV_F * (abc.value[1] - abc.value[2]);
}

void Motor::ParkTransform(const MATH_vec2 &ab, float cos_a, float sin_a,
                          MATH_vec2 &dq) {
  // Park: d = alpha*cos + beta*sin, q = -alpha*sin + beta*cos
  dq.value[0] =  ab.value[0] * cos_a + ab.value[1] * sin_a;
  dq.value[1] = -ab.value[0] * sin_a + ab.value[1] * cos_a;
}

void Motor::InverseParkTransform(const MATH_vec2 &dq, float cos_a, float sin_a,
                                 MATH_vec2 &ab) {
  // Inverse Park: alpha = d*cos - q*sin, beta = d*sin + q*cos
  ab.value[0] = dq.value[0] * cos_a - dq.value[1] * sin_a;
  ab.value[1] = dq.value[0] * sin_a + dq.value[1] * cos_a;
}

void Motor::SpaceVectorGen(const MATH_vec2 &ab, MATH_vec3 &Tabc) {
  // SVGEN: convert alpha-beta to 3-phase duty cycles
  // Ta = Valpha
  // Tb = -0.5*Valpha + sqrt(3)/2 * Vbeta
  // Tc = -0.5*Valpha - sqrt(3)/2 * Vbeta
  float Va = ab.value[0];
  float Vb = -0.5f * ab.value[0] + (SQRT3_F * 0.5f) * ab.value[1];
  float Vc = -0.5f * ab.value[0] - (SQRT3_F * 0.5f) * ab.value[1];

  // Shift to center (min-max centering)
  float vmax = (Va > Vb) ? Va : Vb;
  vmax = (vmax > Vc) ? vmax : Vc;
  float vmin = (Va < Vb) ? Va : Vb;
  vmin = (vmin < Vc) ? vmin : Vc;
  float vshift = -0.5f * (vmax + vmin);

  Tabc.value[0] = Va + vshift;
  Tabc.value[1] = Vb + vshift;
  Tabc.value[2] = Vc + vshift;
}

// **************************************************************************
// Trajectory (replaces TI TRAJ module)

void Motor::trajUpdate(void) {
  float error = traj_target - traj_intValue;
  float delta = traj_K * error;

  // Clamp delta to max acceleration
  if (delta > traj_maxDelta) delta = traj_maxDelta;
  if (delta < -traj_maxDelta) delta = -traj_maxDelta;

  traj_dValue = delta;  // acceleration feedforward
  traj_intValue += delta;

  // Clamp output
  if (traj_intValue > traj_maxValue) traj_intValue = traj_maxValue;
  if (traj_intValue < traj_minValue) traj_intValue = traj_minValue;
}

// **************************************************************************
// Current Loop

void Motor::RunCurrentLoop(float cos_angle, float sin_angle) {
  // Id current controller
  PID_run(pidHandle[PID_Id], idq_ref.value[0], idq.value[0],
          &(vdq_out.value[0]));
  if (pidHandle[PID_Id]->UiSatFlag) {
    windupCount[PID_Id]++;
  } else if (windupCount[PID_Id] > 0) {
    windupCount[PID_Id]--;
  }

  // Dynamic voltage limits for Iq: Vq_max = sqrt((0.5*Vbus)^2 - Vd^2)  [V]
  // VdcBus_Volt is in Volts; vdq_out.value[0] is Vd in Volts.
  float max_vs   = USER_MAX_VS_MAG_PU * VdcBus_Volt;  // [V]
  float max_vs2  = max_vs * max_vs;
  float vd2      = vdq_out.value[0] * vdq_out.value[0];
  float outMax_V = (vd2 < max_vs2) ? sqrtf(max_vs2 - vd2) : 0.0f;

  PID_setMinMax(pidHandle[PID_Iq], -outMax_V, outMax_V);
  PID_setUiMinMax(pidHandle[PID_Iq], -outMax_V, outMax_V);

  // Iq current controller
  PID_run(pidHandle[PID_Iq], idq_ref.value[1], idq.value[1],
          &(vdq_out.value[1]));
  if (pidHandle[PID_Iq]->UiSatFlag) {
    windupCount[PID_Iq]++;
  } else if (windupCount[PID_Iq] > 0) {
    windupCount[PID_Iq]--;
  }

  // Inverse Park transform
  MATH_vec2 Vab_V;
  InverseParkTransform(vdq_out, cos_angle, sin_angle, Vab_V);

  // DC bus compensation: normalise Vab [V] → Vab/Vbus [dimensionless] for SVGEN
  if (VdcBus_Volt > 1.0f) {
    float oneOverDcBus = 1.0f / VdcBus_Volt;
    Vab_V.value[0] *= oneOverDcBus;
    Vab_V.value[1] *= oneOverDcBus;
  }

  // Space vector generation
  SpaceVectorGen(Vab_V, pwmData.Tabc);
}

// **************************************************************************
// Speed Loop

bool Motor::RunSpeedLoop(const USER_Params &user_params) {
  if (++stCntSpeed < user_params.numCtrlTicksPerSpeedTick) {
    idq_ref.value[1] = speed_pid_out;  // already [A]
    return false;
  }

  stCntSpeed = 0;

  // Trajectory update at speed-loop rate (1 kHz) for correct ff units [RPM/tick]
  if (Flag_Run_Identify) {
    trajUpdate();
  }

  if (estop_s.E_stop_command == 0) {
    PID_run_spd(pidHandle[PID_Speed], traj_intValue,
                Speed_rpm, traj_dValue, &speed_pid_out);
  } else {
    Calculate_EStop_Scurve(&estop_s, Speed_rpm, traj_dValue);
    PID_run_spd(pidHandle[PID_Speed], estop_s.command_velocity,
                Speed_rpm, estop_s.command_acc, &speed_pid_out);
  }

  if (pidHandle[PID_Speed]->UiSatFlag) {
    windupCount[PID_Speed]++;
  } else if (windupCount[PID_Speed] > 0) {
    windupCount[PID_Speed]--;
  }

  idq_ref.value[1] = speed_pid_out;  // already [A]

  return true;
}

// **************************************************************************
// Offset Calibration

void Motor::runOffsetsCalculation(USER_Params *userParams) {
  for (int cnt = 0; cnt < 3; cnt++) {
    pwmData.Tabc.value[cnt] = 0.0f;
    offsets_I_A.value[cnt] = 0.0f;

    // Run current offset estimation filter (Iabc_A is already [A])
    float y_i = offsetFilter[cnt].b0 * Iabc_A.value[cnt]
              + offsetFilter[cnt].b1 * offsetFilter[cnt].x1
              - offsetFilter[cnt].a1 * offsetFilter[cnt].y1;
    offsetFilter[cnt].x1 = Iabc_A.value[cnt];
    offsetFilter[cnt].y1 = y_i;
  }

  if (offsetCalcCount++ >= (uint32_t)(userParams->ctrlFreq_Hz * 1.0f)) {
    Flag_enableOffsetcalc = false;
    offsetCalcCount = 0;

    bool offsetOutOfRange = false;

    for (int cnt = 0; cnt < 3; cnt++) {
      float measuredOffset = offsetFilter[cnt].y1;

      // Validate current offset — check if within acceptable range
      if ((measuredOffset < MOTOR_OFFSET_MIN_I_A) ||
          (measuredOffset > MOTOR_OFFSET_MAX_I_A)) {
        // Offset out of range — possible motor rotation or sensor fault
        offsets_I_A.value[cnt] = MOTOR_OFFSET_IDEAL_I_A;
        offsetOutOfRange = true;
      } else {
        // Offset within acceptable range — use measured value
        offsets_I_A.value[cnt] = measuredOffset;
      }

      offsetFilter[cnt].x1 = 0.0f;
      offsetFilter[cnt].y1 = 0.0f;
    }

    // Log warning if offset was out of range
    if (offsetOutOfRange) {
      SetErrorFlag(kOffsetCalibrationWarning);
    } else {
      ClearErrorFlag(kOffsetCalibrationWarning);
    }
  }
}

// **************************************************************************
// Setup Motor

void Motor::setupMotor(USER_Params *userParams,
                       TIM_HandleTypeDef *htim,
                       ADC_HandleTypeDef *hadc_1,
                       ADC_HandleTypeDef *hadc_2)
{
  this->htim_pwm = htim;
  this->hadc1 = hadc_1;
  this->hadc2 = hadc_2;

  hwVer = userParams->hwVersion;

  // Hall sensor setup is called from bear_driver.cpp after setupMotor()

  // Setup PID controllers
  pidSetup(userParams);

  // Setup offset filters
  {
    float b0 = userParams->offsetPole_rps / userParams->ctrlFreq_Hz;
    float a1 = (b0 - 1.0f);

    for (int cnt = 0; cnt < 3; cnt++) {
      offsetFilter[cnt].b0 = b0;
      offsetFilter[cnt].a1 = a1;
      offsetFilter[cnt].b1 = 0.0f;
      offsetFilter[cnt].x1 = 0.0f;
      offsetFilter[cnt].y1 = 0.0f;
    }

    Flag_enableOffsetcalc = false;
    offsetCalcCount = 0;
  }

  // Configure speed reference trajectory (rpm units)
  // K=0.010 matches TI TRAJ_setKValue(0.010): exponential convergence near target.
  // crossover: |error| < maxDelta/K = 100×maxDelta → exponential; above → rate-limited.
  traj_target = 0.0f;
  traj_intValue = 0.0f;
  traj_K = 0.010f;
  traj_maxDelta = MaxAccel_rpmps * (USER_CTRL_PERIOD_sec * (float)USER_NUM_CTRL_TICKS_PER_SPEED_TICK);
  traj_maxValue = USER_MOTOR_MAX_SPEED_RPM;
  traj_minValue = -USER_MOTOR_MAX_SPEED_RPM;

  // Encoder init: TIM handle is set from bear_driver.cpp after setupMotor()
  encoder.init(NULL,
               (uint16_t)USER_MOTOR_ENCODER_LINES,
               userParams->motor_numPolePairs);

  // Stall thresholds
  stall_threshold_zero_speed = USER_MOTOR_STALL_ZERO_SPEED_THRESHOLD;
  stall_threshold_non_zero_speed = USER_MOTOR_STALL_NON_ZERO_SPEED_THRESHOLD;
  stall_threshold_high_current =
      userParams->maxCurrent * USER_MOTOR_STALL_CURRENT_RATIO;

  EStop_SCurve_Init(&estop_s);
}

// **************************************************************************
// ISR - Main control interrupt

void Motor::ISR(USER_Params *userParams) {
  // --- Fault check at ISR rate (10 kHz) ---
  // Matches TI faultISR → disableMotors pattern:
  //   detect → SetErrorFlag → disablePWM + gateDriver.disable (immediate).
  // The ISR wrapper in bear_driver.cpp propagates to both motors via
  // disableMotors(), matching TI motor1_faultISR().
  if (!Flag_bypassFaultCheck) {
    if (checkFault()) {
      SetErrorFlag(kGateDriverError);
      /* Distinguish OCP from OT/UVLO using the FLAG pin (STDRIVE102BH):
       *   FLAG=H → overcurrent/short-circuit (OCP latch)
       *   FLAG=L → overtemperature or undervoltage lockout (OT/UVLO latch) */
      if (gateDriver.getFlag()) {
        SetErrorFlag(kGateDriverOCPError);
        ClearErrorFlag(kGateDriverOTUVLOError);
      } else {
        SetErrorFlag(kGateDriverOTUVLOError);
        ClearErrorFlag(kGateDriverOCPError);
      }
      disablePWM();
      gateDriver.disable();
      Flag_Run_Identify = false;
      return;
    } else {
      ClearErrorFlag(kGateDriverError);
      ClearErrorFlag(kGateDriverOCPError);
      ClearErrorFlag(kGateDriverOTUVLOError);
    }
  }

  // Read encoder position first (minimum latency)
  uint32_t position_counter = encoder.getPositionCounter();

  MATH_vec2 Iab;

  // Run Hall state check
  HALLSensor::Direction hall_direction = hallSensor.HALLBLDC_State_Check();
  if (hall_direction == HALLSensor::kDirectionInvalid) {
    hallsensor_fault_count++;
  } else {
    if (hallsensor_fault_count > 0) {
      hallsensor_fault_count--;
    }

    float encoder_offset_angle_pu = hallSensor.getHallAnglePU();

    switch (hall_direction) {
      case HALLSensor::kDirectionNegative:
        encoder_offset_angle_pu =
            (encoder_offset_angle_pu + (1.0f / 6.0f)) - 1.0f;
        // INTENTIONAL FALLTHROUGH
        /* fall through */
      case HALLSensor::kDirectionPositive:
        encoder.setHallOffset(encoder_offset_angle_pu);
        break;

      default:
        break;
    }
  }

  // --- Hall vs Encoder phase angle validation (TI pattern) ---
  // Compare only when hall offset is calibrated (hall_offset_cnt == 0).
  // Mismatch → resync FOC angle from Hall, increment phasefault counter.
  if (encoder.hall_offset_cnt == 0) {
    if (comparePhaseAngle(hallSensor.hall_angle_pu, encoder.electAngle,
                          MOTOR_HALL_SENSOR_PHASE_ANGLE_THRESHOLD)) {
      encoder.hall_offset_cnt = 3;  // resync FOC angle from Hall
      hallsensor_phasefault_count++;
      return;
    } else {
      if (hallsensor_phasefault_count > 0) {
        hallsensor_phasefault_count--;
      }
    }
  }

  // Check Hall sensor phase angle errors (low-pass filtered)
  if (hallsensor_phasefault_count > MOTOR_HALLSENSOR_FAULT_COUNT_THRESHOLD) {
    disablePWM();
    SetErrorFlag(kHallPhaseAngleError);
    return;
  }

  // Check Hall sensor errors (low-pass filtered)
  if (checkHallFault()) {
    disablePWM();
    SetErrorFlag(kHallError);
    return;
  }

  // Get angle from encoder (always update, even when motor disabled)
  encoder.getMechAnglePU(position_counter);  // updates internal mech/elect angle
  encoder.getElectAnglePU();

  // During initial Hall calibration (hall_offset_cnt > 1), encoder offset is
  // not yet reliable.  Use coarse Hall angle for FOC until calibrated (TI pattern).
  if (encoder.hall_offset_cnt > 1) {
    angle_pu = hallSensor.hall_angle_pu;
  } else {
    angle_pu = encoder.electAngle;
  }

  // Encoder phase angle validation (TI pattern):
  // After encoder angle update, check for stuck hall / excessive encoder drift.
  if (encoder.hall_offset_cnt == 0) {
    if (comparePhaseAngle(hallSensor.hall_angle_pu, encoder.electAngle,
                          MOTOR_ENCODER_PHASE_ANGLE_THRESHOLD)) {
      disablePWM();
      SetErrorFlag(kEncoderPhaseAngleError);
      return;
    }
  }

  // Track encoder period at ISR rate (10kHz) for low-speed estimation.
  // Replaces TI QEP capture hardware that measured time between encoder edges.
  encoder.trackPeriod(position_counter);

  // Update encoder velocity at 1kHz (ISR=10kHz / 10 = 1kHz)
  // Matches the encoder filter design frequency and speed controller rate.
  if (++stCntEncoder >= 10) {
    stCntEncoder = 0;
    encoder.updateValues();
  }
  Speed_rpm = encoder.getVelocity();

  // --- Current sensing + Clarke/Park (always, matches TI ISR structure) ---
  // Iabc_A [A] = ADC read (with bias), preserved for debug.
  // Iabc_fbk_A [A] = offset-removed, used by FOC.
  // Sign: low-side shunt with high-side PWM → ADC decreases when current
  // flows out of the inverter, so negate to get conventional positive = out.
  Iabc_fbk_A.value[0] = offsets_I_A.value[0] - Iabc_A.value[0];
  Iabc_fbk_A.value[1] = offsets_I_A.value[1] - Iabc_A.value[1];
  Iabc_fbk_A.value[2] = offsets_I_A.value[2] - Iabc_A.value[2];

  // Clarke transform
  ClarkeTransform(Iabc_fbk_A, Iab);

  // Compute sin/cos of electrical angle
  float angle_rad = angle_pu * TWO_PI_F;
  float cos_angle = cosf(angle_rad);
  float sin_angle = sinf(angle_rad);

  // Park transform
  ParkTransform(Iab, cos_angle, sin_angle, idq);

  // --- Controller selection (matches TI if/else structure) ---
  if (Flag_Run_Identify) {
    // Active: arm decay for next stop
    Flag_needCurrentDecay = true;
    current_decay_ = 1.0f;

#if (MOTOR_CTRL_MODE == MOTOR_CTRL_MODE_DALIGN)
    // ---------------- D-axis alignment ----------------
    idq_ref.value[0] = MOTOR_CTRL_DALIGN_ID_A;  // [A]
    idq_ref.value[1] = 0.0f;

    // Force electrical angle to 0 -> cos=1, sin=0
    cos_angle = 1.0f;
    sin_angle = 0.0f;
    angle_pu  = 0.0f;
    ParkTransform(Iab, cos_angle, sin_angle, idq);

    RunCurrentLoop(cos_angle, sin_angle);

#elif (MOTOR_CTRL_MODE == MOTOR_CTRL_MODE_TORQUE)
    // ---------------- Torque (Iq) control ----------------
    idq_ref.value[0] = 0.0f;
    idq_ref.value[1] = MOTOR_CTRL_TORQUE_IQ_A;  // [A]

    RunCurrentLoop(cos_angle, sin_angle);

#elif (MOTOR_CTRL_MODE == MOTOR_CTRL_MODE_PWM_FORCE)
    // ---------------- Forced PWM output (scope verification) ----------------
    pwmData.Tabc.value[0] = MOTOR_CTRL_PWM_FORCE_TA - 0.5f;
    pwmData.Tabc.value[1] = MOTOR_CTRL_PWM_FORCE_TB - 0.5f;
    pwmData.Tabc.value[2] = MOTOR_CTRL_PWM_FORCE_TC - 0.5f;

#else  /* MOTOR_CTRL_MODE_SPEED (default) */
    // ---------------- Speed control ----------------
    RunSpeedLoop(*userParams);

    idq_ref.value[0] = 0.0f;  // Id reference = 0 for surface mount PMSM
    RunCurrentLoop(cos_angle, sin_angle);
#endif

  } else if (Flag_enableOffsetcalc) {
    runOffsetsCalculation(userParams);
    // Keep MOE disabled during calibration: TIM1 still counts and triggers ADC,
    // but no PWM switching → no switch-induced noise on current sense shunts.
    return;

  } else if (Flag_needCurrentDecay) {
    // Current decay: exponentially ramp down Id/Iq refs to zero before
    // disabling PWM.  Matches TI: RunSpeedLoop keeps PID tracking,
    // but exit condition checks MEASURED current (idq), not reference.
#if (MOTOR_CTRL_MODE == MOTOR_CTRL_MODE_SPEED)
    RunSpeedLoop(*userParams);
#endif
    const float kMinCutoffCurrent = 0.1f;  // [A] (above ADC noise floor ~50mA)

    if (fabsf(idq.value[0]) > kMinCutoffCurrent ||
        fabsf(idq.value[1]) > kMinCutoffCurrent) {
      idq_ref.value[0] *= current_decay_;
      idq_ref.value[1] *= current_decay_;
      current_decay_ *= kCurrentDecayFactor_;
      RunCurrentLoop(cos_angle, sin_angle);
    } else {
      Flag_needCurrentDecay = false;
    }

  } else {
    // Decay complete (or never started): reset PID integrators and disable PWM
    traj_intValue = 0.0f;
    PID_setUi(pidHandle[PID_Speed], 0.0f);
    PID_setUi(pidHandle[PID_Id], 0.0f);
    PID_setUi(pidHandle[PID_Iq], 0.0f);
    pwmData.Tabc.value[0] = 0.0f;
    pwmData.Tabc.value[1] = 0.0f;
    pwmData.Tabc.value[2] = 0.0f;
    disablePWM();
    return;  // Skip writePwmData() so MOE stays disabled (no PWM switching)
  }

  writePwmData();
}

// **************************************************************************
// writePwmData - Write pwmData.Tabc to TIM CCR registers
// Mirrors TI HAL_writePwmData(). Tabc ∈ [-0.5, 0.5]; CCR = (0.5 + Tabc) × ARR.

void Motor::writePwmData(void) {
  if (htim_pwm == nullptr) return;
  __HAL_TIM_MOE_ENABLE(htim_pwm);
  float period = (float)__HAL_TIM_GET_AUTORELOAD(htim_pwm);
  __HAL_TIM_SET_COMPARE(htim_pwm, TIM_CHANNEL_1,
      (uint32_t)((0.5f + pwmData.Tabc.value[0]) * period));
  __HAL_TIM_SET_COMPARE(htim_pwm, TIM_CHANNEL_2,
      (uint32_t)((0.5f + pwmData.Tabc.value[1]) * period));
  __HAL_TIM_SET_COMPARE(htim_pwm, TIM_CHANNEL_3,
      (uint32_t)((0.5f + pwmData.Tabc.value[2]) * period));
}

// **************************************************************************
// Run - Main loop function

void Motor::run(USER_Params *userParams) {
  // Get torque and temperature every 10ms (matches TI Motor::run() pattern)
  // timerCounter_10ms is incremented by Timers_10ms_Callback() (SysTick, 10ms)
  if (timerCounter_10ms != timerCounter_prev) {
    // Torque: Iq [A] x torque constant [Nm/A]
    motor_torque_Nm = idq.value[1] * userParams->torqueConstant;
    // Temperature: NTC thermistor via ADC3_IN5 (PB13)
    computeTemperatureC();

    // Update prev only after MTR2 so both motors compute on the same 10ms tick
    if (mtrNum == HAL_MTR2) {
      timerCounter_prev = timerCounter_10ms;
    }
  }

  // Set the speed acceleration every cycle so API changes take effect immediately
  // (matches TI: TRAJ_setMaxDelta(trajHandle_spd, _IQmpy(MaxAccel_krpmps, SF)))
  traj_maxDelta = MaxAccel_rpmps * (USER_CTRL_PERIOD_sec * (float)USER_NUM_CTRL_TICKS_PER_SPEED_TICK);

  // If Flag_Run_Identify: set trajectory target; else clear it
  // (matches TI: TRAJ_setTargetValue(SpeedRef_pu) / TRAJ_setTargetValue(0))
  if (Flag_Run_Identify) {
    traj_target = SpeedRef_rpm;
  } else {
    traj_target = 0.0f;
  }

  // Expose current trajectory output for telemetry / API reads
  SpeedTraj_rpm = traj_intValue;
}

// **************************************************************************
// Temperature Calculation

void Motor::computeTemperatureC(void) {
  // NTC thermistor Steinhart-Hart equation (ported from TI computeTemperetureC)
  // Circuit: Vref → Rfixed(10kΩ) → ADC pin → NTC(50kΩ@25°C) → GND
  static const float B_const = 3950.0f;   // NTC B-constant
  static const float R0      = 50000.0f;  // NTC resistance at T0 (Ω)
  static const float T0      = 298.15f;   // Reference temp (K = 25°C)
  static const float Rfixed  = 10000.0f;  // Series resistor (Ω)
  static const float Vref    = 3.3f;      // ADC reference voltage (V)
  static const float ADC_MAX = 4095.0f;   // 12-bit full scale

  // Only HW_VERSION_3 has NTC sensor (matches TI hwVer check)
  if ((hwVer == MOTOR_HW_VERSION_15) || (hwVer < MOTOR_HW_VERSION_3)) {
    motor_temperature_DegC = -273.15f;
    return;
  }

  float Vntc = (float)adcData.temperature_adc * (Vref / ADC_MAX);
  if (Vntc >= Vref) Vntc = Vref - 0.0001f;  // avoid divide-by-zero

  float Rntc = (Rfixed * Vntc) / (Vref - Vntc);
  if (Rntc < 1.0f) Rntc = 1.0f;  // clamp to prevent logf(0)

  float Tntc = 1.0f / ((1.0f / T0) + (1.0f / B_const) * logf(Rntc / R0));
  motor_temperature_DegC = Tntc - 273.15f;
}

// **************************************************************************
// Stall Detection

bool Motor::detectStall(void) {
  float abs_speed = fabsf(Speed_rpm);
  float abs_ref = fabsf(SpeedRef_rpm);
  float abs_iq = fabsf(idq.value[1]);

  // Stall = all three conditions met simultaneously:
  // 1. Actual speed ≈ 0  (below zero-speed threshold)
  // 2. Target speed ≠ 0  (non-zero command present)
  // 3. Iq current is high (motor is trying hard but not moving)
  if (abs_speed <= stall_threshold_zero_speed &&
      abs_ref > stall_threshold_non_zero_speed &&
      abs_iq >= stall_threshold_high_current) {
    if (++stall_detection_count > MOTOR_STALL_FAULT_COUNT_THRESHOLD) {
      stall_detection_count = 0;
      return true;
    }
  } else if (stall_detection_count > 0) {
    stall_detection_count--;
  }

  return false;
}

// **************************************************************************
// Windup Detection

bool Motor::detectWindup(HAL_PID_Type_e pid_id) {
  if (pid_id >= 3) return false;

  if (windupCount[pid_id] > windupThreshold[pid_id]) {
    windupCount[pid_id] = 0;
    return true;
  }

  return false;
}

// **************************************************************************
// Phase Angle Comparison
//
// Returns true when the angular difference exceeds 'threshold' (mismatch /
// fault condition).  Matches TI comparePhaseAngle() convention.

bool Motor::comparePhaseAngle(float angle_hall, float angle_encoder,
                              float threshold) {
  float diff = angle_hall - angle_encoder;

  // Normalize to (-0.5, +0.5)  i.e. (-180°, +180°)
  if (diff > 0.5f) {
    diff -= 1.0f;
  } else if (diff < -0.5f) {
    diff += 1.0f;
  }

  return (diff < -threshold) || (diff > threshold);
}

// **************************************************************************
// Fault Check

bool Motor::checkFault(void) {
  // Read the STDRIVE102BH nFAULT pin only (matches TI checkFault pattern).
  // Active-low open-drain: LOW = UVLO / thermal shutdown latched.
  return gateDriver.isFault();
}

bool Motor::checkHallFault(void) {
  return (hallsensor_fault_count > MOTOR_HALLSENSOR_FAULT_COUNT_THRESHOLD);
}

// **************************************************************************
// Disable PWM

void Motor::disablePWM(void) {
  if (htim_pwm != nullptr) {
    // __HAL_TIM_MOE_DISABLE is conditional on all CCER bits being cleared,
    // which is never true while PWM channels are running.
    // Use the unconditional variant to always clear MOE.
    // With OSSI=1, OISx=OISxN=RESET(LOW): all HIN/LIN go LOW → all FETs off.
    __HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(htim_pwm);
  }
}

// **************************************************************************
// Short Brake PWM
// Matches TI HAL_setupStallFaults(): TZA=Low (high-side OFF), TZB=High (low-side ON).
// In PWM1 mode with OCxPolarity=HIGH, OCNPolarity=HIGH:
//   CCR=0 → OCxREF always LOW → HIN=LOW (high-side OFF), LIN=HIGH (low-side ON)
// All three motor phases are shorted through the low-side FETs → regenerative braking.

void Motor::shortBrakePWM(void) {
  if (htim_pwm == nullptr) return;

  __HAL_TIM_SET_COMPARE(htim_pwm, TIM_CHANNEL_1, 0);
  __HAL_TIM_SET_COMPARE(htim_pwm, TIM_CHANNEL_2, 0);
  __HAL_TIM_SET_COMPARE(htim_pwm, TIM_CHANNEL_3, 0);
  __HAL_TIM_MOE_ENABLE(htim_pwm);
}

// **************************************************************************
// Encoder Cable Detection

bool Motor::IsEncoderCableAttached() {
  uint32_t exp = encoder.getEncoderExpiration();
  return (exp < 0xFFFF);
}

// **************************************************************************
// Speed PID Gain Accessors

void Motor::setSpeedPIDGains(float Kp, float Ki, float Kd, float Kff,
                             float DN, float gain) {
  Kp_spd = Kp;
  Ki_spd = Ki;
  Kd_spd = Kd;
  Kff_spd = Kff;
  dN_spd = DN;
  Kout = gain;

  if (pidHandle[PID_Speed]) {
    PID_setGains(pidHandle[PID_Speed],
                 Kp, Ki, Kd, Kff, DN, gain);
  }
}

void Motor::getSpeedPIDGains(float &Kp, float &Ki, float &Kd, float &Kff,
                             float &DN, float &gain) {
  Kp = Kp_spd;
  Ki = Ki_spd;
  Kd = Kd_spd;
  Kff = Kff_spd;
  DN = dN_spd;
  gain = Kout;
}

void Motor::setSpeedPGain(float Kp) {
  Kp_spd = Kp;
  if (pidHandle[PID_Speed]) PID_setKp(pidHandle[PID_Speed], Kp);
}
float Motor::getSpeedPGain(void) { return Kp_spd; }

void Motor::setSpeedIGain(float Ki) {
  Ki_spd = Ki;
  if (pidHandle[PID_Speed]) PID_setKi(pidHandle[PID_Speed], Ki);
}
float Motor::getSpeedIGain(void) { return Ki_spd; }

void Motor::setSpeedDGain(float Kd) {
  Kd_spd = Kd;
  if (pidHandle[PID_Speed]) PID_setKd(pidHandle[PID_Speed], Kd);
}
float Motor::getSpeedDGain(void) { return Kd_spd; }

void Motor::setSpeedFFGain(float Kff) {
  Kff_spd = Kff;
  if (pidHandle[PID_Speed]) PID_setKff(pidHandle[PID_Speed], Kff);
}
float Motor::getSpeedFFGain(void) { return Kff_spd; }

void Motor::setSpeedDN(float dN) {
  dN_spd = dN;
  if (pidHandle[PID_Speed]) PID_setDN(pidHandle[PID_Speed], dN);
}
float Motor::getSpeedDN(void) { return dN_spd; }

void Motor::setSpeedKout(float gain) {
  Kout = gain;
  if (pidHandle[PID_Speed]) PID_setKout(pidHandle[PID_Speed], gain);
}
float Motor::getSpeedKout(void) { return Kout; }

void Motor::enablePIDLogging(bool enable) { enablePIDLog = enable; }

// **************************************************************************
// Trajectory Accessors

void Motor::setTrajK(float K) { traj_K = K; }
float Motor::getTrajK(void) { return traj_K; }

void Motor::setTrajMaxDelta(float D) { MaxAccel_rpmps = D; traj_maxDelta = D; }
float Motor::getTrajMaxDelta(void) { return traj_maxDelta; }

float Motor::getSpeedMax(void) {
  return pidHandle[PID_Speed] ? PID_getOutMax(pidHandle[PID_Speed]) : 0.0f;
}

float Motor::getSpeedMin(void) {
  return pidHandle[PID_Speed] ? PID_getOutMin(pidHandle[PID_Speed]) : 0.0f;
}

bool Motor::setSpeedMinMax(float minVal, float maxVal) {
  if (pidHandle[PID_Speed]) {
    PID_setMinMax(pidHandle[PID_Speed], minVal, maxVal);
    PID_setUiMinMax(pidHandle[PID_Speed], minVal, maxVal);
    return true;
  }
  return false;
}
