/*
 * bldc_motor.h
 *
 * Motor controller class definition.
 * Adapted from TI BearDriver for STM32G474.
 * Changes: _iq → float, HAL_Handle → STM32 handles, TI modules → inline FOC,
 *          DRV_SPI_8323 → STDRIVE102BH (GPIO, no SPI)
 */

#ifndef INCLUDE_BLDC_MOTOR_H_
#define INCLUDE_BLDC_MOTOR_H_

#include "main.h"
#include "user_params.h"
#include "hall_sensor.h"
#include "encoder.h"
#include "EStop_SCurve.h"
#include "gatedriver.h"
#include "pid.h"

// **************************************************************************
// the defines

// -------------------------------------------------------------------------
// Control mode selector (for controller bring-up / verification)
//
//   MOTOR_CTRL_MODE_SPEED     : normal closed-loop speed control
//   MOTOR_CTRL_MODE_DALIGN    : D-axis alignment — Id = setpoint, Iq = 0,
//                               theta forced to 0 (rotor pulled to d-axis)
//   MOTOR_CTRL_MODE_TORQUE    : open-loop torque — Id = 0,
//                               Iq = setpoint, theta from encoder
//   MOTOR_CTRL_MODE_PWM_FORCE : forced open-loop PWM output (default)
//                               Tabc set directly to fixed duty cycles.
//                               No FOC, no current loop, no motor required.
//                               Use for scope verification of PWM signals.
// -------------------------------------------------------------------------
#define MOTOR_CTRL_MODE_SPEED     (0)
#define MOTOR_CTRL_MODE_DALIGN    (1)
#define MOTOR_CTRL_MODE_TORQUE    (2)
#define MOTOR_CTRL_MODE_PWM_FORCE (3)

#ifndef MOTOR_CTRL_MODE
#define MOTOR_CTRL_MODE         MOTOR_CTRL_MODE_SPEED
#endif

//! \brief  Id setpoint used in D-axis alignment mode (Amps)
#define MOTOR_CTRL_DALIGN_ID_A  (0.5f)

//! \brief  Iq setpoint used in torque-control mode (Amps)
#define MOTOR_CTRL_TORQUE_IQ_A  (0.5f)

//! \brief  Forced PWM duty cycles for scope verification (0.0 ~ 1.0, center = 0.5)
//!         Tabc maps directly to CCR: 0.5 = 50% duty (mid-rail, safe for no-load)
#define MOTOR_CTRL_PWM_FORCE_TA (0.5f)
#define MOTOR_CTRL_PWM_FORCE_TB (0.5f)
#define MOTOR_CTRL_PWM_FORCE_TC (0.5f)

#define MOTOR_HALLSENSOR_FAULT_COUNT_THRESHOLD (20)
#define MOTOR_STALL_FAULT_COUNT_THRESHOLD      (40)  //!< 100ms×40 = 4sec (matches TI battery OCP time)
#define MOTOR_WINDUP_SPEED_TIMEOUT \
  (2.5f)  //!< Speed controller windup timeout in seconds
#define MOTOR_WINDUP_CURRENT_TIMEOUT \
  (2.5f)  //!< Current controller windup timeout in seconds
#define MOTOR_WINDUP_SPEED_FAULT_THRESHOLD \
  ((uint32_t)(USER_CTRL_FREQ_Hz / \
   (USER_NUM_CTRL_TICKS_PER_SPEED_TICK / MOTOR_WINDUP_SPEED_TIMEOUT)))
#define MOTOR_WINDUP_CURRENT_FAULT_THRESHOLD \
  ((uint32_t)(USER_CTRL_FREQ_Hz / \
   (USER_NUM_ISR_TICKS_PER_CTRL_TICK / MOTOR_WINDUP_CURRENT_TIMEOUT)))

#define MOTOR_STATUS_ERROR_CODE_SHIFT (22)  //!< Motor error code in status
#define MOTOR_STATUS_ERROR_CODE_MASK  (0xFFC00000)  //!< Motor error code mask
//! \brief  Lower 22 bits of getMotorStatus() carry the STDRIVE102BH gate
//!         driver status word (raw pin levels + decoded state). See
//!         GateDriver::getStatusWord() for the bit layout.
#define MOTOR_STATUS_GATEDRV_MASK     (0x003FFFFF)

//! \brief  Hall sensor phase angle validation threshold
//!         75deg = 60deg (Hall sensor step) * 1.25 :  75 / 360 = 0.208
#define MOTOR_HALL_SENSOR_PHASE_ANGLE_THRESHOLD (0.208f)

//! \brief  Encoder phase angle validation threshold
//!         120deg = 60deg (Hall sensor step) * 2 : 120 / 360 = 0.333
#define MOTOR_ENCODER_PHASE_ANGLE_THRESHOLD (0.333f)

//! \brief  Current offset calibration validation thresholds [A]
//!         Matches TI: ±7.05A acceptable range around ideal zero-current offset
#define MOTOR_OFFSET_IDEAL_I_A   (0.0f)    //!< Ideal current offset at zero current [A]
#define MOTOR_OFFSET_MIN_I_A     (-7.05f)  //!< Minimum acceptable offset [A]
#define MOTOR_OFFSET_MAX_I_A     (7.05f)   //!< Maximum acceptable offset [A]

// **************************************************************************
// the typedefs

//! \brief  Motor select enumeration (matches original HAL_MtrSelect_e)
typedef enum {
  HAL_MTR1 = 0,
  HAL_MTR2 = 1
} HAL_MtrSelect_e;

//! \brief  The ID of each PID
typedef enum {
  PID_Speed    = 0,  //!< PID Speed controller
  PID_Id       = 1,  //!< PID for Id control
  PID_Iq       = 2   //!< PID for Iq control
} HAL_PID_Type_e;

//! \brief  2-element vector (d-q, alpha-beta)
typedef struct {
  float value[2];
} MATH_vec2;

//! \brief  3-element vector (ABC phases)
typedef struct {
  float value[3];
} MATH_vec3;

//! \brief  PWM data structure (matches original HAL_PwmData_t)
typedef struct {
  MATH_vec3 Tabc;  //!< PWM duty cycles for three phases. -1.0 is 0%, 1.0 is 100%
} HAL_PwmData_t;

//! \brief  ADC data structure
typedef struct {
  MATH_vec3 I;               //!< Three phase currents (ADC fraction 0~1.0)
  float dcBus;               //!< DC bus voltage (ADC fraction 0~1.0)
  uint32_t temperature_adc;  //!< NTC thermistor raw ADC value (ADC3_IN5, 12-bit)
  float IBus_A;              //!< DC bus current [A] from ACS71240LLCBTR-050B3 (ai_Current, ADC5_CH10)
} HAL_AdcData_t;

// **************************************************************************
// the Motor class

class Motor {
 public:
  //! Error code bitfield flags
  enum ErrorFlag {
    kErrorNone = 0,
    kStallError = (1 << 0),            //!< Stall condition occurred
    kHallError = (1 << 1),             //!< Hall sensor invalid reading value
    kGateDriverError = (1 << 2),       //!< Gate driver error - nFault pin
    kWindupSpeedError = (1 << 3),      //!< Speed PID controller windup error
    kWindupCurrentIdError = (1 << 4),  //!< Current (Id) PID controller windup error
    kWindupCurrentIqError = (1 << 5),  //!< Current (Iq) PID controller windup error
    kHallPhaseAngleError = (1 << 6),   //!< Hall sensor phase angle error
    kEncoderPhaseAngleError = (1 << 7),//!< Encoder phase angle error
    kCommunicationsError = (1 << 8),   //!< Communications timeout
    kEStopError = (1 << 9),            //!< E-Stop active
    kOffsetCalibrationWarning = (1 << 10),  //!< Offset calibration out of range, using ideal value
    kGateDriverOCPError    = (1 << 11),  //!< Gate driver OCP:   nFAULT=L + FLAG=H (overcurrent/short-circuit)
    kGateDriverOTUVLOError = (1 << 12),  //!< Gate driver OT/UVLO: nFAULT=L + FLAG=L (overtemperature or undervoltage)
  };

  //! \brief  Error codes that indicate warnings (motors not disabled)
  static const uint32_t kMotorErrorCodeWarnMask =
      (kWindupSpeedError | kWindupCurrentIdError | kWindupCurrentIqError |
       kCommunicationsError | kOffsetCalibrationWarning);

  // Motor feature enable flags
  bool Flag_Run_Identify;
  bool Flag_Run_Identify_cmd;
  bool Flag_enableOffsetcalc;
  bool Flag_bypassFaultCheck;   //!< Skip ISR fault check during restart/calibration
  bool Flag_needCurrentDecay;

  //! \brief  motor board hw version
  int16_t hwVer;

  // Motor Parameters
  float IqRef_A;
  float SpeedSet_rpm;
  float SpeedRef_rpm;     // motor target speed
  float SpeedTraj_rpm;
  float MaxAccel_rpmps;
  float Speed_rpm;        // speed reported from encoders

  float angle_pu;

  float speed_pid_out;

  float currentBwCoeff; //!< Current-loop bandwidth coefficient α: Kp = α·Ls/Ts
  float Kp_spd;
  float Ki_spd;
  float Kout;          //!< final output loop gain
  float Kd_spd;
  float Kff_spd;
  float dN_spd;        // speed pid derivative filter pole

  float VdcBus_Volt;

  HAL_PwmData_t pwmData;   //!< PWM duty cycles for each phase
  HAL_AdcData_t adcData;   //!< ADC measurements (I: [A], dcBus: raw [0..1])

  MATH_vec3 offsets_I_A;    //!< current feedback offsets [A]
  MATH_vec3 Iabc_A;         //!< phase currents [A] (ADC read, with bias)
  MATH_vec3 Iabc_fbk_A;     //!< phase currents [A] (offset-removed, for FOC)

  MATH_vec2 idq_ref;      //!< Id and Iq references [A]
  MATH_vec2 vdq_out;      //!< Vd and Vq from current controllers [V]
  MATH_vec2 idq;          //!< Id and Iq measured values [A]

  uint32_t offsetCalcCount;

  Encoder encoder;           //!< encoder for this motor
  HALLSensor hallSensor;     //!< HALL Sensor handler for the motor
  GateDriver gateDriver;     //!< STDRIVE102BH gate driver for this motor

  PID_Obj pid[3];            //!< PID controllers: 0-Speed, 1-Id, 2-Iq
  PID_Handle pidHandle[3];   //!< PID handles

  uint32_t windupCount[3];       //!< Windup detection counter
  uint32_t windupThreshold[3];   //!< Windup detection threshold

  HAL_MtrSelect_e mtrNum;   //!< ID for this motor instance

  bool enablePIDLog;

  // stall values
  int32_t stall_detection_count;
  float stall_threshold_zero_speed;
  float stall_threshold_non_zero_speed;
  float stall_threshold_high_current;

  float motor_torque_Nm;        //!< Motor torque [Nm] (placeholder, 0.0 until sensor added)
  float motor_temperature_DegC; //!< Motor temperature [degC] (placeholder, 0.0 until sensor added)

  float kCurrentDecayFactor_;
  float current_decay_;

  ESTOP_SCURVE estop_s;

 private:
  const USER_Params *userParams_;

  void pidSetup(USER_Params *userParams);

  // Set idq_ref.value[{0,1}] first, this generates PWM command
  void RunCurrentLoop(float cos_angle, float sin_angle);

  // Write pwmData.Tabc to TIM CCR registers (CCR = (0.5 + Tabc) × ARR)
  // Mirrors TI HAL_writePwmData(). Uses this->htim_pwm — works for motor1/motor2.
  void writePwmData(void);

  // Runs the speed loop with decimation
  bool RunSpeedLoop(const USER_Params &userParams);

  void runOffsetsCalculation(USER_Params *userParams);

  bool comparePhaseAngle(float angle_hall, float angle_encoder, float threshold);

  //! Compute motor_temperature_DegC from NTC thermistor ADC reading (TI port)
  void computeTemperatureC(void);

  int32_t hallsensor_phasefault_count;
  int32_t hallsensor_fault_count;
  uint32_t error_code;

  uint16_t stCntSpeed;   //!< speed loop decimation counter
  uint16_t stCntEncoder; //!< encoder velocity update decimation counter

  // Speed reference trajectory (replaces TI TRAJ module)
  float traj_target;
  float traj_intValue;
  float traj_minValue;
  float traj_maxValue;
  float traj_K;
  float traj_maxDelta;
  float traj_dValue;     //!< derivative (acceleration feedforward)

  // Offset filter coefficients (replaces TI FILTER_FO module)
  PID_Filter_t offsetFilter[3];   //!< 3 current offset filters

  /* STM32-specific hardware handles */
  TIM_HandleTypeDef *htim_pwm;
  ADC_HandleTypeDef *hadc1;
  ADC_HandleTypeDef *hadc2;

  // FOC helper methods (replace TI Clarke/Park/SVGEN modules)
  static void ClarkeTransform(const MATH_vec3 &abc, MATH_vec2 &ab);
  static void ParkTransform(const MATH_vec2 &ab, float cos_a, float sin_a, MATH_vec2 &dq);
  static void InverseParkTransform(const MATH_vec2 &dq, float cos_a, float sin_a, MATH_vec2 &ab);
  static void SpaceVectorGen(const MATH_vec2 &ab, MATH_vec3 &Tabc);

  // Trajectory update (replaces TI TRAJ module)
  void trajUpdate(void);

 public:
  // Constructor
  Motor(HAL_MtrSelect_e mtrNum);

  void ISR(USER_Params *userParams);

  void setupMotor(USER_Params *userParams,
                  TIM_HandleTypeDef *htim_pwm,
                  ADC_HandleTypeDef *hadc1,
                  ADC_HandleTypeDef *hadc2);

  void run(USER_Params *userParams);

  void setSpeedPIDGains(float Kp, float Ki, float Kd, float Kff, float DN,
                        float Kout);
  void getSpeedPIDGains(float &Kp, float &Ki, float &Kd, float &Kff, float &DN,
                        float &Kout);

  void setSpeedPGain(float Kp);
  float getSpeedPGain(void);

  void setSpeedIGain(float Ki);
  float getSpeedIGain(void);

  void setSpeedDGain(float Kd);
  float getSpeedDGain(void);

  void setSpeedFFGain(float Kff);
  float getSpeedFFGain(void);

  void setSpeedDN(float dN);
  float getSpeedDN(void);

  void setSpeedKout(float gain);
  float getSpeedKout(void);

  void enablePIDLogging(bool enable);

  void setTrajK(float K);
  float getTrajK(void);

  void setTrajMaxDelta(float D);
  float getTrajMaxDelta(void);

  float getSpeedMax(void);
  float getSpeedMin(void);

  bool setSpeedMinMax(float minVal, float maxVal);

  bool IsEncoderCableAttached();

  //! \brief  Gets the motor status as a formatted 32-bit value.
  //!         Layout:
  //!           bits 31..22 : error_code        (MOTOR_STATUS_ERROR_CODE_MASK)
  //!           bits 21..0  : gate driver word  (MOTOR_STATUS_GATEDRV_MASK)
  //!                         -> see GateDriver::getStatusWord()
  inline uint32_t getMotorStatus(void) {
    uint32_t motor_status =
        (error_code << MOTOR_STATUS_ERROR_CODE_SHIFT) &
        MOTOR_STATUS_ERROR_CODE_MASK;
    motor_status |=
        (gateDriver.getStatusWord() & MOTOR_STATUS_GATEDRV_MASK);
    return motor_status;
  }

  bool detectStall(void);

  //! \brief  Read the STDRIVE102BH nFAULT pin (active-LOW).
  //!         Returns true when a gate driver fault is present.
  //!         Matches TI checkFault() pattern: pure GPIO read, no ErrorFlag side-effects.
  bool checkFault(void);

  //! \brief  Check hall sensor fault counter against threshold.
  //!         Returns true when hallsensor_fault_count exceeds the threshold.
  bool checkHallFault(void);

  //! \brief  Disables all PWM outputs (Hi-Z: all FETs off via MOE clear)
  void disablePWM(void);

  //! \brief  Short brake: high-side OFF, low-side ON (CCR=0, MOE enabled)
  //!         Matches TI HAL_setupStallFaults(): TZA=Low, TZB=High.
  void shortBrakePWM(void);

  bool detectWindup(HAL_PID_Type_e pid_id);

  uint32_t GetErrorCode() { return error_code; };
  void SetErrorCode(uint32_t code) { error_code = code; };
  void SetErrorFlag(ErrorFlag flag) { error_code |= flag; };
  void ClearErrorFlag(ErrorFlag flag) { error_code &= (flag ^ 0xffffffff); };

  bool IsErrorFlagSet(ErrorFlag flag) { return (error_code & flag); };

  bool HasErrors(uint32_t warn_mask = 0UL) {
    return (error_code & ~(warn_mask));
  };
};

#endif /* INCLUDE_BLDC_MOTOR_H_ */
