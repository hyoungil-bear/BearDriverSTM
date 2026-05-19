/*
 * api.cpp
 *
 * Motor control API implementation.
 * Adapted from TI BearDriver api.cpp for STM32G474.
 * Changes: bit_cast → API_FloatToUint32/API_Uint32ToFloat, _iq → float,
 *          extern Motor objects instead of HAL handles
 *
 * Structure matches original TI api.cpp:
 *   - APIMap table with set/get function pointers
 *   - processCommand() dispatch
 *   - processHostComs() with timeout
 *   - processDebugComs() + doCmd() debug console
 */

#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "api.h"
#include "timers.h"
#include "bldc_motor.h"
#include "encoder.h"
#include "flash_layout.h"
#include "pid.h"
#include "spi_flash.h"
#include "version.h"
#include "version_info.h"
#include "user_params.h"
#include "bear_driver.h"
#include "differential_drive_limiter_helper.h"
#include "sci_coms.h"

// **************************************************************************
// externs from bear_driver.cpp

extern Motor motor1;
extern Motor motor2;
extern Motor *gMotors[2];
extern USER_Params gUserParams;
extern bool stall_lock_enable;
extern bool disable_motors_on_boot;
extern int16_t motor_hd_version;
extern MAIN_STATE main_state;

// **************************************************************************
// forward declarations

static void processDebugCmd(void);
static void doCmd(uint16_t argc, char **argv);

static void setSpeedPGain(const BasePacket_t *pIn);
static void setSpeedIGain(const BasePacket_t *pIn);
static void setSpeedDGain(const BasePacket_t *pIn);
static void setSpeedFFGain(const BasePacket_t *pIn);
static void setSpeedDN(const BasePacket_t *pIn);
static void setSpeedOutGain(const BasePacket_t *pIn);
static void setTrajK(const BasePacket_t *pIn);
static void setTrajMaxDelta(const BasePacket_t *pIn);
static void setTargetSpdKRPM(const BasePacket_t *pIn);
static void setEnableMotor(const BasePacket_t *pIn);
static void setEnableOffsetCalcs(const BasePacket_t *pIn);
static void setResetFault(const BasePacket_t *pIn);
static void setDisableMotorsOnBoot(const BasePacket_t *pIn);
static void setStallLockEnable(const BasePacket_t *pIn);

static void getVersion(MotorPacket_t *pOut);
static void getTrajK(MotorPacket_t *pOut);
static void getTrajMaxDelta(MotorPacket_t *pOut);
static void getSpeedPGain(MotorPacket_t *pOut);
static void getSpeedIGain(MotorPacket_t *pOut);
static void getSpeedDGain(MotorPacket_t *pOut);
static void getSpeedFFGain(MotorPacket_t *pOut);
static void getSpeedDN(MotorPacket_t *pOut);
static void getSpeedOutGain(MotorPacket_t *pOut);
static void getTargetSpdKRPM(MotorPacket_t *pOut);
static void getSpdKRPM(MotorPacket_t *pOut);
static void getEnableMotor(MotorPacket_t *pOut);
static void getEncoder(MotorPacket_t *pOut);
static void getEncoderVel(MotorPacket_t *pOut);
static void getStatus(MotorPacket_t *pOut);
static void getHardwareRev(MotorPacket_t *pOut);
static void getDisableMotorsOnBoot(MotorPacket_t *pOut);
static void getStallLockEnable(MotorPacket_t *pOut);

// **************************************************************************
// API map table

typedef struct {
  API_REG_e reg;
  void (*write)(const BasePacket_t *pIn);
  void (*read)(MotorPacket_t *pOut);
} API_Map_t;

//! \brief  Sorted table of API interface functions
//!         \note This table is sorted for indexing and should match the order
//!               in API_REG_e enum
static API_Map_t api_table[API_REG_NUM] = {
  {API_REG_VERSION,                              NULL,                 getVersion},
  {API_REG_STATUS,                               NULL,                 getStatus},
  {API_REG_ST_BW,                                NULL,                 NULL},
  {API_REG_PID_Idq_P,                            NULL,                 NULL},
  {API_REG_PID_Idq_I,                            NULL,                 NULL},
  {API_REG_TARGET_SPD_KRPM,                      setTargetSpdKRPM,     getTargetSpdKRPM},
  {API_REG_SPD_KRPM,                             NULL,                 getSpdKRPM},
  {API_REG_ENABLE_MOTOR,                         setEnableMotor,       getEnableMotor},
  {API_REG_ENABLE_OFFSET_CALCS,                  setEnableOffsetCalcs, NULL},
  {API_REG_ENCODER,                              NULL,                 getEncoder},
  {API_REG_ENCODER_VEL,                          NULL,                 getEncoderVel},
  {API_REG_PID_SPEED_P,                          setSpeedPGain,        getSpeedPGain},
  {API_REG_PID_SPEED_I,                          setSpeedIGain,        getSpeedIGain},
  {API_REG_PID_SPEED_D,                          setSpeedDGain,        getSpeedDGain},
  {API_REG_PID_SPEED_FF,                         setSpeedFFGain,       getSpeedFFGain},
  {API_REG_PID_SPEED_Dn,                         setSpeedDN,           getSpeedDN},
  {API_REG_RESET_FAULT,                          setResetFault,        NULL},
  {API_REG_TRAJ_K,                               setTrajK,             getTrajK},
  {API_REG_TRAJ_MAX_DELTA,                       setTrajMaxDelta,      getTrajMaxDelta},
  {API_REG_PID_SPEED_OUT,                        setSpeedOutGain,      getSpeedOutGain},
  {API_REG_HARDWARE_REV,                         NULL,                 getHardwareRev},
  {API_REG_SET_TARGET_SPD_KRPM_GET_ENCODER_VEL,  setTargetSpdKRPM,     getEncoderVel},
  {API_REG_DISABLE_MOTORS_ON_BOOT,               setDisableMotorsOnBoot, getDisableMotorsOnBoot},
  {API_REG_STALL_LOCK_ENABLE,                    setStallLockEnable,   getStallLockEnable},
};

// **************************************************************************
// the set functions

static void setSpeedOutGain(const BasePacket_t *pIn) {
  float arg1 = API_Uint32ToFloat(pIn->arg1);
  float arg2 = API_Uint32ToFloat(pIn->arg2);
  motor1.setSpeedKout(arg1);
  motor2.setSpeedKout(arg2);
}

static void setSpeedPGain(const BasePacket_t *pIn) {
  float arg1 = API_Uint32ToFloat(pIn->arg1) * SPD_KP_PU_TO_SI;
  float arg2 = API_Uint32ToFloat(pIn->arg2) * SPD_KP_PU_TO_SI;
  motor1.setSpeedPGain(arg1);
  motor2.setSpeedPGain(arg2);
}

static void setSpeedIGain(const BasePacket_t *pIn) {
  float arg1 = API_Uint32ToFloat(pIn->arg1) * SPD_KI_PU_TO_SI;
  float arg2 = API_Uint32ToFloat(pIn->arg2) * SPD_KI_PU_TO_SI;
  motor1.setSpeedIGain(arg1);
  motor2.setSpeedIGain(arg2);
}

static void setSpeedDGain(const BasePacket_t *pIn) {
  float arg1 = API_Uint32ToFloat(pIn->arg1) * SPD_KD_PU_TO_SI;
  float arg2 = API_Uint32ToFloat(pIn->arg2) * SPD_KD_PU_TO_SI;
  motor1.setSpeedDGain(arg1);
  motor2.setSpeedDGain(arg2);
}

static void setSpeedFFGain(const BasePacket_t *pIn) {
  float arg1 = API_Uint32ToFloat(pIn->arg1) * SPD_KFF_PU_TO_SI;
  float arg2 = API_Uint32ToFloat(pIn->arg2) * SPD_KFF_PU_TO_SI;
  motor1.setSpeedFFGain(arg1);
  motor2.setSpeedFFGain(arg2);
}

static void setSpeedDN(const BasePacket_t *pIn) {
  float arg1 = API_Uint32ToFloat(pIn->arg1);
  float arg2 = API_Uint32ToFloat(pIn->arg2);
  motor1.setSpeedDN(arg1);
  motor2.setSpeedDN(arg2);
}

static void setTrajK(const BasePacket_t *pIn) {
  float arg1 = API_Uint32ToFloat(pIn->arg1);
  float arg2 = API_Uint32ToFloat(pIn->arg2);
  motor1.setTrajK(arg1);
  motor2.setTrajK(arg2);
}

static void setTrajMaxDelta(const BasePacket_t *pIn) {
  float arg1 = API_Uint32ToFloat(pIn->arg1);
  float arg2 = API_Uint32ToFloat(pIn->arg2);
  motor1.setTrajMaxDelta(arg1);
  motor2.setTrajMaxDelta(arg2);
}

static void setTargetSpdKRPM(const BasePacket_t *pIn) {
  float arg1 = API_Uint32ToFloat(pIn->arg1);
  float arg2 = API_Uint32ToFloat(pIn->arg2);

  // Only set velocity if in run state
  if (getMainState() == STATE_RUN) {
    // MTR_2 is left motor, MTR_1 is right motor
    // Due to the layout of the hardware the right motor's input is reversed
    DifferentialDriveLimiter_Command_t requested = {arg2, -arg1};
    DifferentialDriveLimiter_Command_t limited;
    DifferentialDriveLimiter_Limit(&gCmdLimiter, &requested, &limited);

    motor1.SpeedRef_rpm = -limited.right * 1000.0f;  // KRPM → RPM
    motor2.SpeedRef_rpm = limited.left * 1000.0f;    // KRPM → RPM
    Timers_Start(TIMER_CMD);  // Got valid command, restart timeout
  } else {
    motor1.SpeedRef_rpm = 0.0f;
    motor2.SpeedRef_rpm = 0.0f;
  }
}

static void setEnableMotor(const BasePacket_t *pIn) {
  // Always update cmd flags to preserve setting for ESTOP/fault recovery
  motor1.Flag_Run_Identify_cmd = pIn->arg1 != 0;
  motor2.Flag_Run_Identify_cmd = pIn->arg2 != 0;

  if (getMainState() == STATE_SS2_ESTOP || getMainState() == STATE_ESTOP ||
      getMainState() == STATE_STALL_LOCK) {
    return;
  }

  // In normal state, also update the active flags
  motor1.Flag_Run_Identify = motor1.Flag_Run_Identify_cmd;
  motor2.Flag_Run_Identify = motor2.Flag_Run_Identify_cmd;
}

static void setEnableOffsetCalcs(const BasePacket_t *pIn) {
  if (getMainState() == STATE_SS2_ESTOP || getMainState() == STATE_ESTOP) {
    return;
  }
  motor1.Flag_Run_Identify = pIn->arg1 == 0;
  motor1.Flag_enableOffsetcalc = pIn->arg1 == 1;

  motor2.Flag_Run_Identify = pIn->arg2 == 0;
  motor2.Flag_enableOffsetcalc = pIn->arg2 == 1;
}

static void setResetFault(const BasePacket_t *pIn) {
  (void)pIn;
  // Only allow resetting from fault state
  if (getMainState() == STATE_FAULT) {
    setMainState(STATE_FAULT_RESTART);
  }
}

static void setDisableMotorsOnBoot(const BasePacket_t *pIn) {
  FlashLayout flash_data;
  if (!spi_flash.read(0, flash_data)) {
    return;
  }

  bool arg = !!(pIn->arg1);

  if (flash_data.disable_motors_on_boot.magic != SPIFlash::MagicBits::kValid ||
      pIn->arg1 != flash_data.disable_motors_on_boot.data) {
    flash_data.disable_motors_on_boot.data = arg;
    flash_data.disable_motors_on_boot.magic = SPIFlash::MagicBits::kValid;
    spi_flash.write(0, flash_data);  /* FRAM: no erase needed, direct overwrite */
  }
}

static void setStallLockEnable(const BasePacket_t *pIn) {
  stall_lock_enable = pIn->arg1 != 0;
}

// **************************************************************************
// the get functions

static void getVersion(MotorPacket_t *pOut) {
  VERSION_INFO_t *v = Version_GetInfo();
  pOut->arg1 = v->groups.major_minor;
  pOut->arg2 = v->groups.revision_build;
}

static void getTrajK(MotorPacket_t *pOut) {
  pOut->arg1 = API_FloatToUint32(motor1.getTrajK());
  pOut->arg2 = API_FloatToUint32(motor2.getTrajK());
}

static void getTrajMaxDelta(MotorPacket_t *pOut) {
  pOut->arg1 = API_FloatToUint32(motor1.getTrajMaxDelta());
  pOut->arg2 = API_FloatToUint32(motor2.getTrajMaxDelta());
}

static void getSpeedPGain(MotorPacket_t *pOut) {
  pOut->arg1 = API_FloatToUint32(motor1.Kp_spd / SPD_KP_PU_TO_SI);
  pOut->arg2 = API_FloatToUint32(motor2.Kp_spd / SPD_KP_PU_TO_SI);
}

static void getSpeedIGain(MotorPacket_t *pOut) {
  pOut->arg1 = API_FloatToUint32(motor1.Ki_spd / SPD_KI_PU_TO_SI);
  pOut->arg2 = API_FloatToUint32(motor2.Ki_spd / SPD_KI_PU_TO_SI);
}

static void getSpeedDGain(MotorPacket_t *pOut) {
  pOut->arg1 = API_FloatToUint32(motor1.Kd_spd / SPD_KD_PU_TO_SI);
  pOut->arg2 = API_FloatToUint32(motor2.Kd_spd / SPD_KD_PU_TO_SI);
}

static void getSpeedFFGain(MotorPacket_t *pOut) {
  pOut->arg1 = API_FloatToUint32(motor1.Kff_spd / SPD_KFF_PU_TO_SI);
  pOut->arg2 = API_FloatToUint32(motor2.Kff_spd / SPD_KFF_PU_TO_SI);
}

static void getSpeedDN(MotorPacket_t *pOut) {
  pOut->arg1 = API_FloatToUint32(motor1.dN_spd);
  pOut->arg2 = API_FloatToUint32(motor2.dN_spd);
}

static void getSpeedOutGain(MotorPacket_t *pOut) {
  pOut->arg1 = API_FloatToUint32(motor1.Kout);
  pOut->arg2 = API_FloatToUint32(motor2.Kout);
}

static void getTargetSpdKRPM(MotorPacket_t *pOut) {
  pOut->arg1 = API_FloatToUint32(motor1.SpeedRef_rpm / 1000.0f);  // RPM → KRPM
  pOut->arg2 = API_FloatToUint32(motor2.SpeedRef_rpm / 1000.0f);  // RPM → KRPM
}

static void getSpdKRPM(MotorPacket_t *pOut) {
  pOut->arg1 = API_FloatToUint32(motor1.Speed_rpm / 1000.0f);  // RPM → KRPM
  pOut->arg2 = API_FloatToUint32(motor2.Speed_rpm / 1000.0f);  // RPM → KRPM
}

static void getEnableMotor(MotorPacket_t *pOut) {
  if (getMainState() == STATE_SS2_ESTOP || getMainState() == STATE_ESTOP ||
      getMainState() == STATE_STALL_LOCK) {
    pOut->arg1 = motor1.Flag_Run_Identify_cmd;
    pOut->arg2 = motor2.Flag_Run_Identify_cmd;
  } else {
    pOut->arg1 = motor1.Flag_Run_Identify;
    pOut->arg2 = motor2.Flag_Run_Identify;
  }
}

static void getEncoder(MotorPacket_t *pOut) {
  // Q16 fixed-point: matches TI _IQ16() = (int32_t)(radians * 65536)
  pOut->arg1 = (uint32_t)(int32_t)(-motor1.encoder.getPosition() * 65536.0f);
  pOut->arg2 = (uint32_t)(int32_t)(motor2.encoder.getPosition() * 65536.0f);
}

static void getEncoderVel(MotorPacket_t *pOut) {
  // IQ24 KRPM: matches TI (uint32_t)velocity_iq where velocity is _iq (Q24) KRPM
  // encoder.getVelocity() returns float RPM; convert: RPM / 1000 * 2^24
  pOut->arg1 = (uint32_t)(int32_t)(-motor1.encoder.getVelocity() / 1000.0f * 16777216.0f);
  pOut->arg2 = (uint32_t)(int32_t)(motor2.encoder.getVelocity() / 1000.0f * 16777216.0f);
}

static void getStatus(MotorPacket_t *pOut) {
  pOut->arg1 = motor1.getMotorStatus();
  pOut->arg2 = motor2.getMotorStatus();
}

static void getHardwareRev(MotorPacket_t *pOut) {
  pOut->arg1 = (uint32_t)motor_hd_version;
  pOut->arg2 = 0;
}

static void getDisableMotorsOnBoot(MotorPacket_t *pOut) {
  pOut->arg1 = 0xFFFFFFFF;
  pOut->arg2 = 0xFFFFFFFF;

  FlashLayout flash_data;
  if (!spi_flash.read(0, flash_data)) {
    return;
  }

  if (flash_data.disable_motors_on_boot.magic == SPIFlash::MagicBits::kValid) {
    pOut->arg1 = flash_data.disable_motors_on_boot.data;
    pOut->arg2 = flash_data.disable_motors_on_boot.magic;
  }
}

static void getStallLockEnable(MotorPacket_t *pOut) {
  pOut->arg1 = stall_lock_enable;
  pOut->arg2 = 0;
}

// **************************************************************************
// command processing

static inline API_Map_t *apiSearch(API_REG_e reg) {
  if (reg < API_REG_NUM) {
    return &api_table[reg];
  }
  return NULL;
}

bool API_ProcessCommand(const BasePacket_t *pIn, MotorPacket_t *pOut) {
  if (pIn == NULL || pOut == NULL) return false;

  API_CMD_TYPE_e cmd = pIn->rw;
  API_REG_e reg = pIn->reg;
  API_Map_t *map;

  if ((map = apiSearch(reg)) != NULL) {
    switch (cmd) {
      case API_CMD_WR:
        if (map->write != NULL) {
          map->write(pIn);
          return false;
        }
        break;
      case API_CMD_RD:
        if (map->read != NULL) {
          pOut->rw = API_CMD_RD_RESP;
          pOut->reg = reg;
          pOut->arg3 = motor1.getMotorStatus();
          pOut->arg4 = motor2.getMotorStatus();
          // arg5: torque (motor1=right upper16, motor2=left lower16), units 0.1 Nm
          // arg6: temperature (motor1=right upper16, motor2=left lower16), units 0.1 degC
          // Matches TI: (motor2_TI << 16) | motor1_TI; TI motor2=right = STM32 motor1
          pOut->arg5 = ((uint32_t)(int32_t)(motor1.motor_torque_Nm * 10.0f) << 16) |
                       ((uint32_t)(int32_t)(motor2.motor_torque_Nm * 10.0f) & 0xFFFF);
          pOut->arg6 = ((uint32_t)(int32_t)(motor1.motor_temperature_DegC * 10.0f) << 16) |
                       ((uint32_t)(int32_t)(motor2.motor_temperature_DegC * 10.0f) & 0xFFFF);
          map->read(pOut);
          return true;
        }
        break;
      case API_CMD_WR_RD:
        if (!map->write || !map->read) {
          return false;
        }
        map->write(pIn);
        pOut->rw = API_CMD_RD_RESP;
        pOut->reg = reg;
        pOut->arg3 = motor1.getMotorStatus();
        pOut->arg4 = motor2.getMotorStatus();
        pOut->arg5 = ((uint32_t)(int32_t)(motor1.motor_torque_Nm * 10.0f) << 16) |
                     ((uint32_t)(int32_t)(motor2.motor_torque_Nm * 10.0f) & 0xFFFF);
        pOut->arg6 = ((uint32_t)(int32_t)(motor1.motor_temperature_DegC * 10.0f) << 16) |
                     ((uint32_t)(int32_t)(motor2.motor_temperature_DegC * 10.0f) & 0xFFFF);
        map->read(pOut);
        return true;
      default:
        return false;
    }
  }

  return false;
}

void API_ProcessHostComs(void) {
  SCI_ProcessHostComs();

  if (Timers_Check(TIMER_CMD)) {
    // no valid command before timeout, force 0 velocity
    motor1.SpeedRef_rpm = 0.0f;
    motor2.SpeedRef_rpm = 0.0f;

    motor1.SetErrorFlag(Motor::kCommunicationsError);
    motor2.SetErrorFlag(Motor::kCommunicationsError);
  }
}

void API_Init(void) {
  // Nothing to initialize - Motor objects own all state
}

// **************************************************************************
// debug console

#define CMD_BUFFER_LEN 128
#define MAX_ARGS 10

static char cmd_buffer[CMD_BUFFER_LEN];
static uint16_t cmd_idx = 0;

void API_ProcessDebugComs(void) {
  // Read characters from debug UART (SCI_A)
  while (SCI_RxAvailable(SCI_A_FD)) {
    int16_t c = SCI_GetChar(SCI_A_FD);
    if (c < 0) break;

    switch (c) {
      case 127:  // del
      case 8:    // backspace
        if (cmd_idx > 0) {
          cmd_idx--;
          SCI_PutChar(SCI_A_FD, 8);
          SCI_PutChar(SCI_A_FD, ' ');
          SCI_PutChar(SCI_A_FD, 8);
        }
        break;

      case '\033':  // ESC
        motor1.enablePIDLogging(false);
        motor2.enablePIDLogging(false);
        cmd_idx = 0;
        printf("\n\nLogging off\n\n>");
        break;

      case '\n':
        break;

      case '\r':
        cmd_buffer[cmd_idx] = '\0';
        printf("\n");
        processDebugCmd();
        cmd_idx = 0;
        break;

      default:
        if (cmd_idx < CMD_BUFFER_LEN - 1) {
          SCI_PutChar(SCI_A_FD, (uint8_t)c);
          cmd_buffer[cmd_idx++] = (char)c;
        }
        break;
    }
  }
}

static void processDebugCmd(void) {
  char *argv[MAX_ARGS];
  uint16_t argc = 0;

  char *ptr = strtok(cmd_buffer, " ");

  while (ptr != NULL) {
    if (argc < MAX_ARGS) {
      argv[argc++] = ptr;
      ptr = strtok(NULL, " ");
    } else {
      printf("Too many arguments.\n");
      break;
    }
  }

  if (argc > 0) {
    doCmd(argc, argv);
  }

  printf(">");
}

static void doCmd(uint16_t argc, char **argv) {
  FlashLayout flash_data;
  PID_CONFIG pid_config;
  bool show_help = false;

  if (strncmp(argv[0], "pid", sizeof("pid")) == 0) {
    if (argc <= 2) {
      motor1.getSpeedPIDGains(pid_config.Kp, pid_config.Ki, pid_config.Kd,
                              pid_config.Kff, pid_config.dN, pid_config.Kout);
      pid_config.K = motor1.getTrajK();
      printf("pid1: Kp=%2.2f Ki=%2.2f Kd=%2.2f Kff=%2.2f dN=%2.2f K=%2.3f "
             "Kout=%2.3f\n",
             pid_config.Kp, pid_config.Ki, pid_config.Kd,
             pid_config.Kff, pid_config.dN, pid_config.K, pid_config.Kout);
      motor2.getSpeedPIDGains(pid_config.Kp, pid_config.Ki, pid_config.Kd,
                              pid_config.Kff, pid_config.dN, pid_config.Kout);
      pid_config.K = motor2.getTrajK();
      printf("pid2: Kp=%2.2f Ki=%2.2f Kd=%2.2f Kff=%2.2f dN=%2.2f K=%2.3f "
             "Kout=%2.3f\n",
             pid_config.Kp, pid_config.Ki, pid_config.Kd,
             pid_config.Kff, pid_config.dN, pid_config.K, pid_config.Kout);

    } else if (argc == 9) {
      pid_config.Kp = (float)atof(argv[2]);
      pid_config.Ki = (float)atof(argv[3]);
      pid_config.Kd = (float)atof(argv[4]);
      pid_config.Kff = (float)atof(argv[5]);
      pid_config.dN = (float)atof(argv[6]);
      pid_config.K = (float)atof(argv[7]);
      pid_config.Kout = (float)atof(argv[8]);

      motor1.setSpeedPIDGains(pid_config.Kp, pid_config.Ki, pid_config.Kd,
                              pid_config.Kff, pid_config.dN, pid_config.Kout);
      motor1.setTrajK(pid_config.K);
      motor2.setSpeedPIDGains(pid_config.Kp, pid_config.Ki, pid_config.Kd,
                              pid_config.Kff, pid_config.dN, pid_config.Kout);
      motor2.setTrajK(pid_config.K);

      if (!spi_flash.read(0, flash_data)) {
        printf("Failed to read flash data\n");
      }

      flash_data.pid.magic = SPIFlash::MagicBits::kValid;
      flash_data.pid.data = pid_config;
      bool sts = spi_flash.write(0, flash_data);  /* FRAM: no erase needed */

      flash_data.pid.magic = 0;
      sts |= spi_flash.read(0, flash_data);
      if (flash_data.pid.magic != SPIFlash::MagicBits::kValid || !sts) {
        printf("\n\n*** ERROR: could not save PID values to FLASH ***\n\n");
      } else {
        printf("pid set to: Kp=%2.3f Ki=%2.3f Kd=%2.3f Kff=%2.3f dN=%2.3f "
               "K=%2.3f Kout=%2.3f\n",
               pid_config.Kp, pid_config.Ki, pid_config.Kd,
               pid_config.Kff, pid_config.dN, pid_config.K, pid_config.Kout);
      }
    } else {
      printf("invalid number of arguments for pid command.\n");
      show_help = true;
    }

  } else if (strncmp(argv[0], "reset", sizeof("reset")) == 0) {
    memset(&flash_data, 0x0, sizeof(flash_data));
    flash_data.pid.magic = SPIFlash::MagicBits::kClear;
    flash_data.disable_motors_on_boot.magic = SPIFlash::MagicBits::kClear;
    bool sts = spi_flash.write(0, flash_data);  /* FRAM: no erase needed */

    flash_data.pid.magic = 0;
    sts |= spi_flash.read(0, flash_data);
    if (flash_data.pid.magic != SPIFlash::MagicBits::kClear || !sts) {
      printf("\n\n*** ERROR: clearing FLASH failed ***\n\n");
    } else {
      printf("\n\nReset FLASH Success!\n\n");
    }

    // restore default values
    motor1.setSpeedPIDGains(gUserParams.spdParams.Kp, gUserParams.spdParams.Ki,
                            gUserParams.spdParams.Kd, gUserParams.spdParams.Kff,
                            gUserParams.spdParams.dN, gUserParams.spdParams.Kout);
    motor2.setSpeedPIDGains(gUserParams.spdParams.Kp, gUserParams.spdParams.Ki,
                            gUserParams.spdParams.Kd, gUserParams.spdParams.Kff,
                            gUserParams.spdParams.dN, gUserParams.spdParams.Kout);

  } else if (strncmp(argv[0], "trace", sizeof("trace")) == 0) {
    show_help = true;
    uint16_t mtr_idx = 0;
    bool enableLog = false;

    if (argc == 3) {
      mtr_idx = (uint16_t)atoi(argv[1]);
      enableLog = strcmp(argv[2], "e") == 0;

      if (mtr_idx == 1) {
        motor1.enablePIDLogging(enableLog);
        show_help = false;
      } else if (mtr_idx == 2) {
        motor2.enablePIDLogging(enableLog);
        show_help = false;
      } else {
        printf("invalid motor number\n");
      }
    }

    if (!show_help) {
      printf("PID Logging for motor %d %s\n", mtr_idx,
             enableLog ? "enabled" : "disabled");
    }

  } else if (strncmp(argv[0], "traj", sizeof("traj")) == 0) {
    if (argc > 1) {
      if (strncmp(argv[1], "K", sizeof("K")) == 0) {
        if (argc > 2) {
          float traj_k = (float)atof(argv[2]);
          motor1.setTrajK(traj_k);
          motor2.setTrajK(traj_k);
        }
        printf("Traj K(1)=%2.3f, K(2)=%2.3f\n",
               motor1.getTrajK(), motor2.getTrajK());
      } else if (strncmp(argv[1], "maxDelta", sizeof("maxDelta")) == 0) {
        if (argc > 2) {
          float maxD = (float)atof(argv[2]);
          motor1.setTrajMaxDelta(maxD);
          motor2.setTrajMaxDelta(maxD);
        }
        printf("Traj maxDelta(1)=%2.3f, maxDelta(2)=%2.3f\n",
               motor1.getTrajMaxDelta(), motor2.getTrajMaxDelta());
      } else {
        printf("Traj: unrecognized command\n");
        show_help = true;
      }
    } else {
      printf("traj: invalid number of arguments\n");
      show_help = true;
    }

  } else if (strncmp(argv[0], "MinMax", sizeof("MinMax")) == 0) {
    if (argc > 1) {
      float minMax = (float)atof(argv[1]);
      motor1.setSpeedMinMax(-minMax, minMax);
      motor2.setSpeedMinMax(-minMax, minMax);
    }
    printf("motor1 scaled limits: %2.3f %2.3f\n",
           motor1.getSpeedMin(), motor1.getSpeedMax());
    printf("motor2 scaled limits: %2.3f %2.3f\n",
           motor2.getSpeedMin(), motor2.getSpeedMax());

  } else if (strncmp(argv[0], "encoder_exp", sizeof("encoder_exp")) == 0) {
    if (argc > 1) {
      uint32_t exp = (uint32_t)atoi(argv[1]);
      motor1.encoder.setEncoderExpiration(exp);
      motor2.encoder.setEncoderExpiration(exp);
    }
    printf("motor 1 encoder expiration tics: %lu\n",
           (unsigned long)motor1.encoder.getEncoderExpiration());
    printf("motor 2 encoder expiration tics: %lu\n",
           (unsigned long)motor2.encoder.getEncoderExpiration());

  } else if (strncmp(argv[0], "detectCable", sizeof("detectCable")) == 0) {
    printf("Motor 1 encoder cable %s\n",
           motor1.IsEncoderCableAttached() ? "connected" : "disconnected");
    printf("Motor 2 encoder cable %s\n",
           motor2.IsEncoderCableAttached() ? "connected" : "disconnected");

  } else {
    show_help = true;
  }

  if (show_help) {
    printf("Firmware Ver: %d.%d.%d.%d\n", MAJOR_VERSION,
           MINOR_VERSION, REVISION, BUILD);
    printf("Hardware Ver: %d\n\n", motor1.hwVer);
    printf("\nUsage:\n");
    printf("pid ? : show pid settings\n");
    printf("pid set <Kp> <Ki> <Kd> <Kff> <dN> <K> <Kout>: set pid settings\n");
    printf("trace <1|2> <e|d> : enable/disable speed PID log\n");
    printf("<ESC> : disables all logging\n");
    printf("traj K [value] : display/set input tracking gain\n");
    printf("traj maxDelta [value] : display/set max delta (KRPM/s)\n");
    printf("MinMax [value] : display/set speed control current limit\n");
    printf("encoder_exp [value] : display/set encoder expiration (tics)\n");
    printf("reset : clears flash and restores default parameters\n");
    printf("detectCable : check for encoder cable presence\n");
  }
}
