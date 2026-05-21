/*
 * bear_driver.cpp
 *
 * Main application state machine and system-level control.
 * Adapted from TI BearDriver main_ram() and main_loop() for STM32G474.
 * Changes: HAL_Handle → STM32 HAL, TI PIE/CPU → NVIC, _iq → float
 *
 * Structure matches original TI bear_driver.cpp:
 *   - Global Motor objects (motor1, motor2, gMotors[])
 *   - Global USER_Params (gUserParams)
 *   - MAIN_STATE state machine
 *   - disableMotors(), checkESTOP(), main_loop()
 *   - ISR wrappers as extern "C"
 */

// **************************************************************************
// the includes

#include <stdio.h>
#include <math.h>
#include <string.h>

#include "bear_driver.h"
#include "main.h"
#include "tim.h"
#include "adc.h"
#include "gpio.h"

#include "sci_coms.h"
#include "timers.h"
#include "iwdg.h"

#include "hall_sensor.h"
#include "bldc_motor.h"
#include "encoder.h"
#include "Ebrake.h"
#include "flash_layout.h"
#include "spi_flash.h"
#include "version.h"
#include "user_params.h"
#include "api.h"
#include "differential_drive_limiter_helper.h"
#include "cpu_time.h"
#include "debug.h"

#ifdef DEBUG
#define DPRINTF(x, ...) SCI_Printf(SCI_USART2, x, ##__VA_ARGS__)
#define DFLUSH()
#else
#define DPRINTF(x, ...)
#define DFLUSH()
#endif

// forward declarations
static void main_loop(void);
static bool checkESTOP(void);

/* External HAL handles (defined in Core/Src/ by CubeMX) */
extern ADC_HandleTypeDef hadc1;   /* Motor 2 phase currents (injected, TIM1_TRGO): CH6=PhA, CH7=PhB, CH8=PhC */
extern ADC_HandleTypeDef hadc2;   /* Motor 2 thermistor (regular, SW): CH12=PB2 */
extern ADC_HandleTypeDef hadc3;   /* Motor 1 phase currents (injected, TIM20_TRGO): CH4=PhA, CH1=PhB, CH12=PhC */
extern ADC_HandleTypeDef hadc4;   /* Motor 1 thermistor (regular,  SW): CH1=PE14 */
extern ADC_HandleTypeDef hadc5;   /* Bus voltage + current (regular, SW): CH9=Volt, CH10=Curr */
extern TIM_HandleTypeDef htim1;   /* Motor 1 PWM (advanced timer, TRGO -> ADC1/ADC2) */
extern TIM_HandleTypeDef htim2;   /* Motor 2 Encoder (quadrature, PD3/PD4) */
extern TIM_HandleTypeDef htim3;   /* Motor 1 Encoder capture (TIM3_CH2, PA4) */
extern TIM_HandleTypeDef htim4;   /* Motor 2 Encoder capture (TIM4_CH1, PB6) */
extern TIM_HandleTypeDef htim5;   /* Motor 1 Encoder (quadrature, PA0/PA1) */
extern TIM_HandleTypeDef htim8;   /* Motor 1 Brake PWM (TIM8_CH4, PD1) */
extern TIM_HandleTypeDef htim15;  /* Motor 2 Brake PWM (TIM15_CH2, PB15) */
extern TIM_HandleTypeDef htim20;  /* Motor 2 PWM (advanced timer, TRGO -> ADC3/ADC4) */
extern SPI_HandleTypeDef hspi3;   /* SPI FRAM (hardware NSS, PA15) */
extern volatile uint32_t IdleLoopCount;

// **************************************************************************
// the globals

USER_Params gUserParams;  // shared by both motors

// global system enable flag
bool gFlag_enableSystem = false;

// the two motor instances
Motor motor1(HAL_MTR1);
Motor motor2(HAL_MTR2);

// electromagnetic brake instances (DRV8871DDAR, one per motor)
Ebrake ebrake1;  // Motor1 Right — TIM8_CH4  (PD1)
Ebrake ebrake2;  // Motor2 Left  — TIM15_CH2 (PB15)
Motor *gMotors[2] = {&motor1, &motor2};

bool disable_motors_on_boot = true;
MAIN_STATE main_state = STATE_INIT;
bool ran_init_sequence = false;

bool stall_lock_enable = false;

// Sticky fault latch: captures error_code at the moment of STATE_FAULT entry
// (error_code may be cleared by ISR before debugger reads it)
volatile uint32_t fault_latch_m1 = 0;
volatile uint32_t fault_latch_m2 = 0;

// ISR execution time measurement (replaces TI CPU_TIME module)
CPU_TIME_Obj cpu_time_m1;
CPU_TIME_Obj cpu_time_m2;

// motor version check
int16_t motor_hd_version;

// **************************************************************************
// the functions

// disable both motors
void disableMotors(void) {
  motor1.disablePWM();
  motor2.disablePWM();
  // Assert SD on both STDRIVE102BH gate drivers so all MOSFETs are
  // hard-latched to the OFF state independent of the PWM timer.
  motor1.gateDriver.disable();
  motor2.gateDriver.disable();
  motor1.Flag_Run_Identify = false;
  motor2.Flag_Run_Identify = false;
  motor1.Flag_Run_Identify_cmd = false;
  motor2.Flag_Run_Identify_cmd = false;
  motor1.SpeedRef_rpm = 0.0f;
  motor2.SpeedRef_rpm = 0.0f;
}

static bool checkESTOP(void) {
  static uint16_t debounceCounter = 0;
  static bool stableEstopState = false;

  // E-Stop I/O pin is active low (PD14)
  bool estop = (HAL_GPIO_ReadPin(di_nESTOP_IN_GPIO_Port, di_nESTOP_IN_Pin) == GPIO_PIN_RESET);

  // Detect a potential state change and start debouncing
  if (estop != stableEstopState) {
    if (debounceCounter < ESTOP_DEBOUNCE_THRESHOLD) {
      debounceCounter++;
    }

    // Update stable state only if the debounce counter reaches the threshold
    if (debounceCounter >= ESTOP_DEBOUNCE_THRESHOLD) {
      stableEstopState = estop;
      debounceCounter = 0;
    }
  } else {
    debounceCounter = 0;
  }

  return stableEstopState;
}

// Parts of main that can be run from ram (matches original main_ram)
void BearDriver_Main(void) {
  // Enable DWT cycle counter for precise timing
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  // Initialize ISR execution time measurement (replaces TI CPU_TIME module)
  CPU_TIME_init(&cpu_time_m1, SystemCoreClock, PWM_FREQUENCY);
  CPU_TIME_init(&cpu_time_m2, SystemCoreClock, PWM_FREQUENCY);

  // ADC Calibration
  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
  HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);
  HAL_ADCEx_Calibration_Start(&hadc3, ADC_SINGLE_ENDED);
  HAL_ADCEx_Calibration_Start(&hadc4, ADC_SINGLE_ENDED);
  HAL_ADCEx_Calibration_Start(&hadc5, ADC_SINGLE_ENDED);

  HAL_Delay(1000);

  // Read hardware version from GPIO pins (di_rev_B0..B3)
  motor_hd_version  = (int16_t)HAL_GPIO_ReadPin(di_rev_B0_GPIO_Port, di_rev_B0_Pin);
  motor_hd_version += (int16_t)HAL_GPIO_ReadPin(di_rev_B1_GPIO_Port, di_rev_B1_Pin) << 1;
  motor_hd_version += (int16_t)HAL_GPIO_ReadPin(di_rev_B2_GPIO_Port, di_rev_B2_Pin) << 2;
  motor_hd_version += (int16_t)HAL_GPIO_ReadPin(di_rev_B3_GPIO_Port, di_rev_B3_Pin) << 3;

  // initialize the user parameters
  USER_setParamsMtr(&gUserParams, motor_hd_version);

  // check for errors in user parameters — halt if invalid (matches TI behavior)
  USER_ErrorCode_e userErr = USER_checkForErrors(&gUserParams);
  if (userErr != USER_ErrorCode_NoError) {
    for (;;) {
      // Parameter error detected — do not allow motor operation.
      // Connect debugger and inspect userErr to diagnose.
      __NOP();
    }
  }

  // Init Coms
  SCI_Init();

  DPRINTF("\r\n\r\n\r\nBLDC Controller Initializing\n");
  DPRINTF("Version: %d.%d.%d.%d  HW: %d\n",
          MAJOR_VERSION, MINOR_VERSION, REVISION, BUILD, motor_hd_version);

  // Initialize SPI FRAM (hardware NSS on PA15, no GPIO CS needed)
  spi_flash.init(&hspi3, nullptr, 0);

  // Read configuration from flash
  FlashLayout flash_data;
  spi_flash.read(0, flash_data);

  if (flash_data.disable_motors_on_boot.magic == SPIFlash::MagicBits::kValid) {
    disable_motors_on_boot = flash_data.disable_motors_on_boot.data;
  }

  if (flash_data.pid.magic == SPIFlash::MagicBits::kValid) {
    motor1.setSpeedPIDGains(flash_data.pid.data.Kp, flash_data.pid.data.Ki,
                            flash_data.pid.data.Kd, flash_data.pid.data.Kff,
                            flash_data.pid.data.dN, flash_data.pid.data.Kout);
    motor2.setSpeedPIDGains(flash_data.pid.data.Kp, flash_data.pid.data.Ki,
                            flash_data.pid.data.Kd, flash_data.pid.data.Kff,
                            flash_data.pid.data.dN, flash_data.pid.data.Kout);
    motor1.setTrajK(flash_data.pid.data.K);
    motor2.setTrajK(flash_data.pid.data.K);
  } else {
    // default controller gains
    motor1.setSpeedPIDGains(gUserParams.spdParams.Kp, gUserParams.spdParams.Ki,
                            gUserParams.spdParams.Kd, gUserParams.spdParams.Kff,
                            gUserParams.spdParams.dN, gUserParams.spdParams.Kout);
    motor2.setSpeedPIDGains(gUserParams.spdParams.Kp, gUserParams.spdParams.Ki,
                            gUserParams.spdParams.Kd, gUserParams.spdParams.Kff,
                            gUserParams.spdParams.dN, gUserParams.spdParams.Kout);
  }

  if (flash_data.kinematic_limits.magic == SPIFlash::MagicBits::kValid) {
    set_cmd_limiter(&flash_data.kinematic_limits.data);
  } else {
    DifferentialDriveLimiter_Params_t default_kinematic_limits;
    default_kinematic_limits.wheel_radius_m = DEFAULT_WHEEL_RADIUS_M;
    default_kinematic_limits.wheel_base_radius_m = DEFAULT_WHEEL_BASE_RADIUS_M;
    default_kinematic_limits.linear_limit_m_s = DEFAULT_LINEAR_LIMIT_M_S;
    default_kinematic_limits.angular_limit_rad_s = DEFAULT_ANGULAR_LIMIT_RAD_S;
    set_cmd_limiter(&default_kinematic_limits);
  }

  // setup the motor objects
  motor1.setupMotor(&gUserParams, &htim1, &hadc3, &hadc4);
  motor2.setupMotor(&gUserParams, &htim20, &hadc1, &hadc2);

  // Setup Hall sensors with GPIO pins from CubeMX
  motor1.hallSensor.setup(
      di_HALLA_1_GPIO_Port, di_HALLA_1_Pin,   // PB11
      di_HALLB_1_GPIO_Port, di_HALLB_1_Pin,   // PB12
      di_HALLC_1_GPIO_Port, di_HALLC_1_Pin);  // PB13
  motor2.hallSensor.setup(
      di_HALLA_2_GPIO_Port, di_HALLA_2_Pin,   // PD7
      di_HALLB_2_GPIO_Port, di_HALLB_2_Pin,   // PD6
      di_HALLC_2_GPIO_Port, di_HALLC_2_Pin);  // PD5

  // Setup encoder TIM handles (setupMotor passes NULL, we set actual handles here)
  motor1.encoder.init(&htim5,
                      (uint16_t)USER_MOTOR_ENCODER_LINES,
                      gUserParams.motor_numPolePairs);
  // TIM3_CH2 (PA4) input-capture for Motor 1 low-speed period measurement.
  motor1.encoder.initCapture(&htim3, TIM_CHANNEL_2);
  motor2.encoder.init(&htim2,
                      (uint16_t)USER_MOTOR_ENCODER_LINES,
                      gUserParams.motor_numPolePairs);
  // TIM4_CH1 (PB6) input-capture for Motor 2 low-speed period measurement.
  motor2.encoder.initCapture(&htim4, TIM_CHANNEL_1);

  // Setup STDRIVE102BH gate drivers
  //   Motor 1: nSTBY=PC5, FAULT=PE15(TIM1_BKIN), FLAG=PB10
  motor1.gateDriver.setup(do_nSTBY_1_GPIO_Port,  do_nSTBY_1_Pin,
                          di_FAULT_1_GPIO_Port,   di_FAULT_1_Pin,
                          di_FLAG_1_GPIO_Port,    di_FLAG_1_Pin);
  //   Motor 2: nSTBY=PB9, FAULT=PF9(TIM20_BKIN), FLAG=PE0
  motor2.gateDriver.setup(do_nSTBY_2_GPIO_Port,  do_nSTBY_2_Pin,
                          di_FAULT_2_GPIO_Port,   di_FAULT_2_Pin,
                          di_FLAG_2_GPIO_Port,    di_FLAG_2_Pin);

  // Setup electromagnetic brakes (DRV8871DDAR, IN2=GND on PCB)
  ebrake1.setup(&htim8,  TIM_CHANNEL_4);   // Motor1 Right — PD1  TIM8_CH4  AF4
  ebrake2.setup(&htim15, TIM_CHANNEL_2);   // Motor2 Left  — PB15 TIM15_CH2 AF1

  // Initialize timers
  Timers_Init();

  // enable the ADC interrupts (injected conversions for motor current sensing)
  HAL_ADCEx_InjectedStart_IT(&hadc1);   // Motor 2 currents A/B/C (TIM1_TRGO, interrupt)
  HAL_ADCEx_InjectedStart_IT(&hadc3);   // Motor 1 currents A/B/C (TIM20_TRGO, interrupt)
  HAL_ADC_Start(&hadc2);                  // prime Motor2 thermistor regular conversion
  HAL_ADC_Start(&hadc4);                  // prime Motor1 thermistor regular conversion (SW)
  HAL_ADC_Start(&hadc5);                // prime first Vbus regular conversion

  // Start encoder timers
  HAL_TIM_Encoder_Start_IT(&htim5, TIM_CHANNEL_ALL);  // Motor 1 encoder (PA0/PA1)
  HAL_TIM_Encoder_Start_IT(&htim2, TIM_CHANNEL_ALL);  // Motor 2 encoder (PD3/PD4)

  // Start Motor 1 PWM (TIM1)
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);

  // Start Motor 2 PWM (TIM20) with half-period offset for ISR interleaving.
  // In center-aligned mode, setting CNT = ARR before CEN places TIM20 at its
  // peak while TIM1 is near its valley, so the two ADC/TRGO triggers are
  // separated by half a PWM period (~50 us at 10 kHz) instead of firing
  // nearly simultaneously.
  __HAL_TIM_SET_COUNTER(&htim20, __HAL_TIM_GET_AUTORELOAD(&htim20));
  HAL_TIM_PWM_Start(&htim20, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim20, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim20, TIM_CHANNEL_3);
  HAL_TIMEx_PWMN_Start(&htim20, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim20, TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Start(&htim20, TIM_CHANNEL_3);

  // Disable the PWM during startup
  motor1.disablePWM();
  motor2.disablePWM();

  // Power up both gate drivers: EN is tied HIGH on the PCB, so powerUp()
  // just waits for bootstrap settle then releases SD. After this the
  // drivers are ready to switch as soon as MOE is re-enabled in
  // STATE_CALC_OFFSETS. PWM timer outputs are still gated off by
  // disablePWM() above, so the MOSFETs remain in a safe state.
  motor1.gateDriver.powerUp();
  motor2.gateDriver.powerUp();

  // start with system enabled
  gFlag_enableSystem = true;

  motor1.Flag_Run_Identify = false;
  motor2.Flag_Run_Identify = false;
  motor1.Flag_Run_Identify_cmd = false;
  motor2.Flag_Run_Identify_cmd = false;

  // Run both motor update routines (run once before first interrupt)
  motor1.run(&gUserParams);
  motor2.run(&gUserParams);

  main_state = STATE_INIT;
  DPRINTF("\nSTATE->INIT\n");

  // start initialization timer
  Timers_Start(TIMER_INIT);

  DPRINTF("BLDC Controller Ready\n");
  DFLUSH();

  // Start IWDG now — matches TI HAL_setupWatchdog() pattern:
  // all initialization is complete; IWDG enabled just before the main loop.
  // Ti = 512 ms (PRESCALER_4, Reload=4095, LSI=32kHz)
  MX_IWDG_Init();

  // Begin the background loop
  for (;;) {
    HAL_IWDG_Refresh(&hiwdg);

    // loop while the enable system flag is true
    while (gFlag_enableSystem) {
      HAL_IWDG_Refresh(&hiwdg);
      IdleLoopCount++;
      Timers_Process();
      main_loop();
    }

    // disable the PWM
    motor1.disablePWM();
    motor2.disablePWM();

    motor1.Flag_Run_Identify = false;
    motor2.Flag_Run_Identify = false;
    motor1.Flag_Run_Identify_cmd = false;
    motor2.Flag_Run_Identify_cmd = false;

    main_state = STATE_INIT;

    Timers_Process();
  }
}

// main_loop() runs around 4.5kHz
static void main_loop(void) {
#ifdef DEBUG
  API_ProcessDebugComs();
#endif

  bool estop_active = checkESTOP();

  if (estop_active) {
    if (main_state > STATE_CALC_OFFSETS) {
      if (motor_hd_version != MOTOR_HW_VERSION_15) {
        if (main_state != STATE_SS2_ESTOP) {
          DPRINTF("\nSS2_ESTOP ON\n");
          Debug_LED_Err_Set(true);
        }

        motor1.Flag_Run_Identify = true;
        motor2.Flag_Run_Identify = true;
        main_state = STATE_SS2_ESTOP;

      } else {
        if (main_state != STATE_ESTOP) {
          DPRINTF("\nSTO_ESTOP ON\n");
          Debug_LED_Err_Set(true);
          Timers_SetLedBlinkRate(LED_FAST);
        }

        disableMotors();  // also asserts SD on both STDRIVE102BH
        motor1.SetErrorFlag(Motor::kEStopError);
        motor2.SetErrorFlag(Motor::kEStopError);
        main_state = STATE_ESTOP;
      }
    }
  }

  switch (main_state) {
    case STATE_INIT:
      if (Timers_Check(TIMER_INIT)) {
        motor1.Flag_Run_Identify = false;
        motor1.Flag_enableOffsetcalc = true;

        motor2.Flag_Run_Identify = false;
        motor2.Flag_enableOffsetcalc = true;
        main_state = STATE_CALC_OFFSETS;
        DPRINTF("\nSTATE->CALC OFFSETS\n");
      }
      break;

    case STATE_CALC_OFFSETS:
      if (motor1.Flag_enableOffsetcalc == false &&
          motor2.Flag_enableOffsetcalc == false) {
        if (ran_init_sequence) {
          // Returning from ESTOP/fault restart - restore last API command
          motor1.Flag_Run_Identify = motor1.Flag_Run_Identify_cmd;
          motor2.Flag_Run_Identify = motor2.Flag_Run_Identify_cmd;
        } else {
          // First boot - use disable_motors_on_boot setting
          motor1.Flag_Run_Identify = !disable_motors_on_boot;
          motor2.Flag_Run_Identify = !disable_motors_on_boot;

          // Initialize cmd flags to match initial state
          motor1.Flag_Run_Identify_cmd = motor1.Flag_Run_Identify;
          motor2.Flag_Run_Identify_cmd = motor2.Flag_Run_Identify;
        }

        // Re-enable ISR fault checking now that gate drivers are stable
        // and offset calibration is complete.
        motor1.Flag_bypassFaultCheck = false;
        motor2.Flag_bypassFaultCheck = false;

        main_state = STATE_RUN;
        ran_init_sequence = true;

        Timers_SetLedBlinkRate(LED_SLOW);

        Timers_Start(TIMER_STALL_DETECTION);

        DPRINTF("\nRUNNING\n");
      }
      break;

    case STATE_RUN:
      // Gate driver nFAULT and hall sensor faults are now checked inside
      // Motor::ISR() at 10 kHz (matches TI faultISR pattern).

      if (Timers_Check(TIMER_STALL_DETECTION)) {
        Timers_Start(TIMER_STALL_DETECTION);

        // Perform stall detection
        if (motor1.detectStall()) {
          motor1.SetErrorFlag(Motor::kStallError);
        }

        if (motor2.detectStall()) {
          motor2.SetErrorFlag(Motor::kStallError);
        }

        if (motor1.IsErrorFlagSet(Motor::kStallError) ||
            motor2.IsErrorFlagSet(Motor::kStallError)) {
          // Short brake: low-side ON to brake motor (matches TI HAL_setupStallFaults)
          motor1.shortBrakePWM();
          motor2.shortBrakePWM();
          Timers_Start(TIMER_SHORT_BRAKE);

          Timers_Stop(TIMER_STALL_DETECTION);
          DPRINTF("\nSTALL Detection!! Torque off!\n");
        }

        // Perform windup detection on speed controllers
        if (motor1.detectWindup(PID_Speed)) {
          motor1.SetErrorFlag(Motor::kWindupSpeedError);
        } else {
          motor1.ClearErrorFlag(Motor::kWindupSpeedError);
        }

        if (motor2.detectWindup(PID_Speed)) {
          motor2.SetErrorFlag(Motor::kWindupSpeedError);
        } else {
          motor2.ClearErrorFlag(Motor::kWindupSpeedError);
        }

        // Perform windup detection on current controllers
        if (motor1.detectWindup(PID_Id)) {
          motor1.SetErrorFlag(Motor::kWindupCurrentIdError);
        } else {
          motor1.ClearErrorFlag(Motor::kWindupCurrentIdError);
        }

        if (motor2.detectWindup(PID_Id)) {
          motor2.SetErrorFlag(Motor::kWindupCurrentIdError);
        } else {
          motor2.ClearErrorFlag(Motor::kWindupCurrentIdError);
        }

        if (motor1.detectWindup(PID_Iq)) {
          motor1.SetErrorFlag(Motor::kWindupCurrentIqError);
        } else {
          motor1.ClearErrorFlag(Motor::kWindupCurrentIqError);
        }

        if (motor2.detectWindup(PID_Iq)) {
          motor2.SetErrorFlag(Motor::kWindupCurrentIqError);
        } else {
          motor2.ClearErrorFlag(Motor::kWindupCurrentIqError);
        }
      }

      if (motor1.HasErrors(Motor::kMotorErrorCodeWarnMask) ||
          motor2.HasErrors(Motor::kMotorErrorCodeWarnMask)) {
        // add experimental flag for stall-error behavior
        if ((motor1.IsErrorFlagSet(Motor::kStallError) ||
             motor2.IsErrorFlagSet(Motor::kStallError)) &&
            stall_lock_enable) {
          DPRINTF("\nSTALL LOCK\n");
          Debug_LED_Err_Set(true);
          Timers_SetLedBlinkRate(LED_FAST);
          main_state = STATE_STALL_LOCK;
        } else {
          disableMotors();
          DPRINTF("\nFAULT\n");
          Debug_LED_Err_Set(true);
          Timers_SetLedBlinkRate(LED_FAST);
          main_state = STATE_FAULT;
        }
      }

      break;

    case STATE_SS2_ESTOP:
      if (estop_active) {
        motor1.estop_s.E_stop_command = 1;
        motor2.estop_s.E_stop_command = 1;

        Timers_Stop(TIMER_CMD);
      } else {
        if (motor1.estop_s.status_flag == DONE &&
            motor2.estop_s.status_flag == DONE) {
          // SCurve flags and speed references reset
          EStop_SCurve_Reset(&motor1.estop_s);
          EStop_SCurve_Reset(&motor2.estop_s);
          motor1.SpeedRef_rpm = 0.0f;
          motor2.SpeedRef_rpm = 0.0f;

          // Restore motor enable state from last setEnableMotor API call
          motor1.Flag_Run_Identify = motor1.Flag_Run_Identify_cmd;
          motor2.Flag_Run_Identify = motor2.Flag_Run_Identify_cmd;

          main_state = STATE_RUN;
          Debug_LED_Err_Set(false);

          DPRINTF("\nSTATE->RUN (after SS2_ESTOP)\n");
        }
      }
      break;

    case STATE_FAULT:
      disableMotors();

      // short brake timeout check
      if (motor1.IsErrorFlagSet(Motor::kStallError) ||
          motor2.IsErrorFlagSet(Motor::kStallError)) {
        if (Timers_Check(TIMER_SHORT_BRAKE)) {
          // after timeout, all power switches into high impedance state
          motor1.disablePWM();
          motor2.disablePWM();

          Timers_Stop(TIMER_SHORT_BRAKE);
        }
      }
      break;

    case STATE_ESTOP:
      if (!checkESTOP() && !motor1.checkFault() && !motor2.checkFault()) {
        main_state = STATE_ESTOP_RESTART;

        DPRINTF("\nESTOP OFF\n");
      } else {
        disableMotors();  // also asserts SD on both STDRIVE102BH

        motor1.SetErrorCode(Motor::kErrorNone);
        motor2.SetErrorCode(Motor::kErrorNone);
      }
      break;

    case STATE_FAULT_RESTART:
      fault_latch_m1 = 0;
      fault_latch_m2 = 0;
      motor1.SetErrorCode(Motor::kErrorNone);
      motor2.SetErrorCode(Motor::kErrorNone);
      // Bypass ISR fault check during gate driver reset and offset calibration.
      // Without this, the ISR can re-disable the gate driver while reset() is
      // still in its blocking HAL_Delay, causing a fault loop and corrupted
      // current offsets (-275 A instead of ~0 A).
      motor1.Flag_bypassFaultCheck = true;
      motor2.Flag_bypassFaultCheck = true;
      // Clear any latched UVLO / OT fault inside the STDRIVE102BH and
      // release SD so the driver is ready to switch again.
      motor1.gateDriver.reset();
      motor2.gateDriver.reset();
      //! INTENTIONAL FALLTHROUGH
      /* fall through */
    case STATE_ESTOP_RESTART: {
      // Bypass ISR fault check for ESTOP path as well (FAULT_RESTART path
      // sets this before fallthrough, but direct ESTOP entry needs it too).
      motor1.Flag_bypassFaultCheck = true;
      motor2.Flag_bypassFaultCheck = true;
      // Preserve motor enable command state before reinitializing
      bool motor1_cmd_backup = motor1.Flag_Run_Identify_cmd;
      bool motor2_cmd_backup = motor2.Flag_Run_Identify_cmd;

      // reinitialize motors
      motor1.setupMotor(&gUserParams, &htim1, &hadc3, &hadc4);
      motor2.setupMotor(&gUserParams, &htim20, &hadc1, &hadc2);

      // Re-setup Hall sensors after motor reinitialization
      motor1.hallSensor.setup(
          di_HALLA_1_GPIO_Port, di_HALLA_1_Pin,
          di_HALLB_1_GPIO_Port, di_HALLB_1_Pin,
          di_HALLC_1_GPIO_Port, di_HALLC_1_Pin);
      motor2.hallSensor.setup(
          di_HALLA_2_GPIO_Port, di_HALLA_2_Pin,
          di_HALLB_2_GPIO_Port, di_HALLB_2_Pin,
          di_HALLC_2_GPIO_Port, di_HALLC_2_Pin);

      // Re-setup encoder TIM handles
      motor1.encoder.init(&htim5,
                          (uint16_t)USER_MOTOR_ENCODER_LINES,
                          gUserParams.motor_numPolePairs);
      motor1.encoder.initCapture(&htim3, TIM_CHANNEL_2);
      motor2.encoder.init(&htim2,
                          (uint16_t)USER_MOTOR_ENCODER_LINES,
                          gUserParams.motor_numPolePairs);
      motor2.encoder.initCapture(&htim4, TIM_CHANNEL_1);

      // Restore motor enable command state
      motor1.Flag_Run_Identify_cmd = motor1_cmd_backup;
      motor2.Flag_Run_Identify_cmd = motor2_cmd_backup;

      // Run both motor update routines to get them started
      motor1.run(&gUserParams);
      motor2.run(&gUserParams);

      // clear startup transient errors
      motor1.SetErrorCode(Motor::kErrorNone);
      motor2.SetErrorCode(Motor::kErrorNone);

      // start re-initialization timer
      Timers_Start(TIMER_INIT);
      main_state = STATE_INIT;
      Debug_LED_Err_Set(false);
      Timers_SetLedBlinkRate(LED_FAST);
      DPRINTF("\nSTATE->INIT\n");

      break;
    }

    case STATE_STALL_LOCK:
      // Prevention of unintended lock release caused by other factors
      motor1.Flag_Run_Identify = true;
      motor2.Flag_Run_Identify = true;
      break;

    default:
      main_state = STATE_INIT;
      DPRINTF("\nBad State\r\nSTATE->INIT\n");
      break;
  }

  // Run both motor update routines
  API_ProcessHostComs();
  motor1.run(&gUserParams);
  motor2.run(&gUserParams);
}

// return the state that the state machine is in
MAIN_STATE getMainState(void) { return main_state; }

// Set the main state
void setMainState(MAIN_STATE state) { main_state = state; }

// **************************************************************************
// the ISR handlers (callable from C stm32g4xx_it.c)

extern "C" void Motor1_ISR_Handler(void) {
  motor1.ISR(&gUserParams);

  // Match TI motor1_faultISR → disableMotors() pattern:
  // If this motor detected a critical fault, disable both motors immediately.
  if (motor1.HasErrors(Motor::kMotorErrorCodeWarnMask)) {
    disableMotors();
  }
}

extern "C" void Motor2_ISR_Handler(void) {
  motor2.ISR(&gUserParams);

  if (motor2.HasErrors(Motor::kMotorErrorCodeWarnMask)) {
    disableMotors();
  }
}

// ADC normalization: right-aligned 12-bit to 0.0~1.0
static float adcToFloat(uint32_t raw) {
  return (float)raw / 4096.0f;  // 12-bit ADC, right-aligned → 0–4095
}

// **************************************************************************
// Encoder capture overflow callback (UIE).
// CC interrupt is disabled; CCR1 latch is read by updateValues() each cycle.
// Motor 1: TIM3 CH2 (PA4), Motor 2: TIM4 CH1 (PB6)

extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if (htim == motor1.encoder.getCaptureTimHandle()) {
    motor1.encoder.handleCaptureOverflow();
  } else if (htim == motor2.encoder.getCaptureTimHandle()) {
    motor2.encoder.handleCaptureOverflow();
  }
}

extern "C" void Motor1_ADC_ReadAndISR(void) {
  CPU_TIME_start(&cpu_time_m1);

  // Motor 1: 3-shunt configuration
  //   ADC3 Injected Rank1 = CH4  (PE7, ai_OA_OA_1) = Phase A
  //   ADC3 Injected Rank2 = CH1  (PB1, ai_OA_OB_1) = Phase B
  //   ADC3 Injected Rank3 = CH12 (PB0, ai_OA_OC_1) = Phase C
  //   ADC4 Injected Rank1 = CH1  (PE14, ai_Thermistor_1) = Thermistor
  // Convert ADC raw → Amperes with Vref/2 offset removal:
  //   I [A] = ((raw - 2048) / 4096) × ADC_TO_AMPS
  //   12-bit ADC, right-aligned → raw 0–4095, mid-point 2048 = 1.65V bias.
  //   ADC_TO_AMPS = Vref / (Rshunt × Gain) = 3.3 / (0.004 × 10) = 82.5  →  range ±41.25 A
  constexpr float kAdcRawToAmps = ADC_TO_AMPS / 4096.0f;
  constexpr float kAdcMidScale  = 2048.0f;  // Vref/2 hardware bias (R19/R20 divider)
  motor1.Iabc_A.value[0] =
      ((float)HAL_ADCEx_InjectedGetValue(&hadc3, ADC_INJECTED_RANK_1) - kAdcMidScale) * kAdcRawToAmps;
  motor1.Iabc_A.value[1] =
      ((float)HAL_ADCEx_InjectedGetValue(&hadc3, ADC_INJECTED_RANK_2) - kAdcMidScale) * kAdcRawToAmps;
  motor1.Iabc_A.value[2] =
      ((float)HAL_ADCEx_InjectedGetValue(&hadc3, ADC_INJECTED_RANK_3) - kAdcMidScale) * kAdcRawToAmps;

  Motor1_ISR_Handler();

  /* DAC debug output — CH1: SpeedTraj_rpm, CH2: Speed_rpm
   * Scale: -100 RPM → 0 V,  0 RPM → 1.65 V,  +100 RPM → 3.3 V       */
  constexpr float kDacSpdScale  = (DBG_DAC_VREF_V * 0.5f) / 100.0f;
  constexpr float kDacSpdOffset = DBG_DAC_VREF_V * 0.5f;
  Debug_DAC_SetVoltage(1, motor1.SpeedTraj_rpm * kDacSpdScale, kDacSpdOffset);
  Debug_DAC_SetVoltage(2, motor1.Speed_rpm     * kDacSpdScale, kDacSpdOffset);

  CPU_TIME_end(&cpu_time_m1);
}

extern "C" void Motor2_ADC_ReadAndISR(void) {
  CPU_TIME_start(&cpu_time_m2);

  // Motor 2: 3-shunt configuration
  //   ADC1 Injected Rank1 = CH6 (PC0, ai_OA_OA_2) = Phase A
  //   ADC1 Injected Rank2 = CH7 (PC1, ai_OA_OB_2) = Phase B
  //   ADC1 Injected Rank3 = CH8 (PC2, ai_OA_OC_2) = Phase C
  // Convert ADC raw → Amperes with Vref/2 offset removal:
  //   I [A] = ((raw - 2048) / 4096) × ADC_TO_AMPS
  //   12-bit ADC, right-aligned → raw 0–4095, mid-point 2048 = 1.65V bias.
  //   ADC_TO_AMPS = Vref / (Rshunt × Gain) = 3.3 / (0.004 × 10) = 82.5  →  range ±41.25 A
  constexpr float kAdcRawToAmps = ADC_TO_AMPS / 4096.0f;
  constexpr float kAdcMidScale  = 2048.0f;  // Vref/2 hardware bias (R19/R20 divider)
  motor2.Iabc_A.value[0] =
      ((float)HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1) - kAdcMidScale) * kAdcRawToAmps;
  motor2.Iabc_A.value[1] =
      ((float)HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_2) - kAdcMidScale) * kAdcRawToAmps;
  motor2.Iabc_A.value[2] =
      ((float)HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_3) - kAdcMidScale) * kAdcRawToAmps;

  Motor2_ISR_Handler();

  CPU_TIME_end(&cpu_time_m2);
}

// ADC Injected conversion complete callback (dispatches to Motor1 or Motor2)
extern "C" void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc) {
  if (hadc->Instance == ADC3) {
    // ADC3 + ADC4 triggered by TIM20_TRGO -> Motor 1
    Motor1_ADC_ReadAndISR();
  } else if (hadc->Instance == ADC1) {
    // ADC1 + ADC2 triggered by TIM1_TRGO -> Motor 2
    Motor2_ADC_ReadAndISR();
  }
}

extern "C" void BearDriver_SlowADC_Update(void) {
  // Sequential poll-and-read for ADC5 scan (2 ranks).
  // Reading DR after each rank clears EOC, preventing OVR on Rank2.
  // Blocks ~2–4 µs total (acceptable in 1 ms SysTick context).

  // Bus voltage + bus current: ADC5 Rank1=CH9(PD12, ai_Voltage), Rank2=CH10(PD13, ai_Current)
  HAL_ADC_Start(&hadc5);

  // Rank1: bus voltage
  HAL_ADC_PollForConversion(&hadc5, 2);
  float dcBus_raw = adcToFloat(HAL_ADC_GetValue(&hadc5));  // clears EOC → allows Rank2 to write DR
  float dcBus_V   = dcBus_raw * VBUS_ADC_TO_VOLT;
  motor1.adcData.dcBus = dcBus_raw;
  motor1.VdcBus_Volt   = dcBus_V;
  motor2.adcData.dcBus = dcBus_raw;
  motor2.VdcBus_Volt   = dcBus_V;

  // Rank2: bus current (ACS71240LLCBTR-050B3, ±50A, VIOUT=VCC/2 at 0A, 26.4 mV/A)
  HAL_ADC_PollForConversion(&hadc5, 2);
  float viout  = adcToFloat(HAL_ADC_GetValue(&hadc5)) * ADC_REFERENCE_VOLTAGE;
  float ibus_A = (viout - ACS71240_ZERO_CURRENT_V) / ACS71240_SENSITIVITY_V_PER_A;
  motor1.adcData.IBus_A = ibus_A;
  motor2.adcData.IBus_A = ibus_A;

  // Thermistor_1: ADC4 regular SW CH1 (PE14, ai_Thermistor_1)
  motor1.adcData.temperature_adc = HAL_ADC_GetValue(&hadc4);
  HAL_ADC_Start(&hadc4);

  // Thermistor_2: ADC2 regular CH12 (PB2, ai_Thermistor_2)
  motor2.adcData.temperature_adc = HAL_ADC_GetValue(&hadc2);
  HAL_ADC_Start(&hadc2);

  // Electromagnetic brake voltage control at 10 ms rate (Vbus LPF τ≈1s, phase transition)
  static uint8_t brake_ctrl_cnt = 0;
  if (++brake_ctrl_cnt >= 10U) {
    brake_ctrl_cnt = 0;
    ebrake1.runVoltageControl(motor1.VdcBus_Volt);
    ebrake2.runVoltageControl(motor2.VdcBus_Volt);
  }
}
