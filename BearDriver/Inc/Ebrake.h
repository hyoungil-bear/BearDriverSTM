/*
 * Ebrake.h
 *
 * Electromagnetic brake driver — DRV8871DDAR H-bridge, one instance per motor.
 *
 * ---------------------------------------------------------------------------
 * Hardware configuration (fixed on PCB):
 *   Motor1 (Right): TIM8_CH4 → po_Brake_1 (PD1),  IC8 DRV8871DDAR
 *   Motor2 (Left) : TIM15_CH2 → po_Brake_2 (PB15), IC9 DRV8871DDAR
 *
 *   IN1 ← MCU PWM output (po_Brake_x)
 *   IN2 ← GND (tied on PCB, not software-controlled)
 *
 * ---------------------------------------------------------------------------
 * Brake coil electrical characteristics:
 *   Coil resistance        : 43 ~ 50 Ω  (nominal ~46 Ω at 20 °C)
 *   Rated voltage / current: 24 Vdc / 0.52 A  → rated power ≈ 12.5 W
 *   Coil inductance        : 1200 ~ 2100 mH (nominal ~1650 mH)
 *   Recommended drive      : DC voltage, supply ripple < 10 %
 *
 * Performance data:
 *   Disengage time         : 80 ms  (coil energised → spring fully overcome)
 *   Engage time            : 30 ms without diode / 100 ms with flyback diode
 *   Minimum disengage voltage: ≥ 18 Vdc @ 20 °C  (kDefaultRatedVoltageV = 24 V ≥ 18 V ✓)
 *   Rated holding torque   : ≥ 6 Nm
 *
 * Thermal characteristics:
 *   Coil temperature rise  : ≤ 110 °C @ 24 Vdc, ambient 20 °C
 *   Maximum ambient        : ≤ 40 °C  (max coil temp = 40 + 110 = 150 °C)
 *   Drive note             : Voltage-controlled only (24 Vdc rated); the
 *                            manufacturer does not specify a separate hold-in
 *                            voltage. kDefaultHoldVoltageV = 20 V (83 % rated,
 *                            ~8.7 W) provides 2 V margin above the 18 V
 *                            minimum disengage voltage.
 *
 * PWM vs. DC drive note:
 *   Datasheet recommends DC with < 10 % ripple. At 20 kHz with L ≈ 1650 mH
 *   and R ≈ 46 Ω, the electrical time constant τ = L/R ≈ 35.9 ms >> 50 µs
 *   (one PWM period). The coil current ripple is therefore < 0.1 % — the
 *   coil sees the PWM duty cycle as a clean DC average voltage. ✓
 *
 * ---------------------------------------------------------------------------
 * Brake type: FAIL-SAFE (spring-engaged, electrically-disengaged)
 *   IN1 = 0    → coast (Hi-Z)  : coil de-energised → spring ENGAGES brake (motor locked)
 *   IN1 = PWM  → forward drive : coil energised    → spring overcome      (motor free)
 *
 * Voltage regulation (Vbus feed-forward):
 *   duty = target_V / Vbus     →   V_coil(avg) ≈ target_V
 *
 * Two-phase thermal management — applies while DISENGAGED (coil energised):
 *   Phase 1 – Rated  : 24 V for kDefaultRatedDurationMs (300 ms > 80 ms disengage time).
 *                      Ensures the electromagnet fully overcomes the spring.
 *   Phase 2 – Holding: reduce to hold_voltage_V (20 V) to limit resistive
 *                      heating while maintaining the disengaged state.
 *
 * ---------------------------------------------------------------------------
 * Typical usage in bear_driver.cpp:
 *   // Init (once, after MX_TIM8_Init / MX_TIM15_Init — ARR set to 8499 in USER CODE)
 *   ebrake1.setup(&htim8,  TIM_CHANNEL_4);        // Motor1 PD1  — TIM8_CH4  AF4
 *   ebrake2.setup(&htim15, TIM_CHANNEL_2);        // Motor2 PB15 — TIM15_CH2 AF1
 *
 *   // Voltage control loop (10 ms period, BearDriver_SlowADC_Update)
 *   ebrake1.runVoltageControl(motor1.VdcBus_Volt);
 *   ebrake2.runVoltageControl(motor2.VdcBus_Volt);
 *
 *   // On-demand
 *   ebrake1.disengage();   // energise coil → allow motor rotation
 *   ebrake1.engage();      // cut power → spring locks motor
 *
 * ---------------------------------------------------------------------------
 * Vbus LPF design:
 *   IIR 1st-order:  y[n] = α·y[n-1] + (1-α)·x[n]
 *   α = 0.95, T = 10 ms  →  τ = −T/ln(α) ≈ 195 ms,  fc = 1/(2πτ) ≈ 0.82 Hz
 *   3τ ≈ 585 ms (95 % settled),  5τ ≈ 975 ms (99 % settled)
 *   runVoltageControl() updates the LPF in ALL states (including kStateEngaged)
 *   so vbus_lpf_V is always warm — disengage() needs no external Vbus argument.
 *
 * ---------------------------------------------------------------------------
 * Software architecture:
 *   Timer HW config (ARR=8499 for 20 kHz) → tim.c USER CODE (HAL layer)
 *   Ebrake class → setup() binds htim/channel, reads ARR from HW at runtime
 *   Duty calc uses htim->Instance->ARR → hardware-independent
 *
 * ---------------------------------------------------------------------------
 * Appendix: BearDriverSTM (Ebrake) vs BearDriverSTM_EV (TI port) brake 비교
 *
 *   항목              | BearDriverSTM (본 프로젝트)        | BearDriverSTM_EV (TI port)
 *   ------------------|------------------------------------|------------------------------------
 *   브레이크 타입     | Electromagnetic (fail-safe)        | Short brake (regenerative)
 *   동작 원리         | 스프링 체결, 코일 통전 시 해제     | 3상 Low-side ON → 상간 단락
 *   하드웨어          | DRV8871DDAR H-bridge (별도 IC)     | FOC 인버터 FET 재활용 (TIM1)
 *   PWM 타이머        | TIM8_CH4 / TIM15_CH2 (전용)       | TIM1_CH1-3 (모터 PWM 공용)
 *   PWM 모드          | Edge-aligned, UP-count, 20 kHz    | Center-aligned, UP-DOWN, 10 kHz
 *   제어 방식         | Vbus feed-forward 전압 제어        | CCR=0 (duty 0%) 고정
 *   체결(engage)      | duty=0 → 코일 OFF → 스프링 잠금   | disablePWM() → MOE OFF → Hi-Z
 *   해제(disengage)   | 코일 24V→20V 2상 열관리            | shortBrakePWM() → Low-side ON
 *   안전 상태(무전원) | 체결 (모터 잠금) ✓                 | 해제 (모터 자유) — fail-safe 아님
 *   Vbus 필터         | IIR LPF α=0.95, τ≈195ms           | 없음 (duty 고정이므로 불필요)
 *   타임아웃          | rated→hold 전환 (300ms)            | Short brake 30초 후 Hi-Z 전환
 *   TI 원본 매핑      | 해당 없음 (신규)                   | HAL_setupStallFaults() → TZA/TZB
 *   수학 라이브러리   | IEEE float                         | TI IQmath → float (포팅 시)
 *   상태 머신         | kStateEngaged/Rated/Holding        | STATE_RUN/FAULT/ESTOP
 *
 *   Short brake (EV) 동작:
 *     TI 원본: EPWM TZA=Low(H-side OFF), TZB=High(L-side ON) → 상간 단락
 *     STM32:   CCR=0 + MOE enable → OCxREF=LOW(HIN OFF), LIN=HIGH → 동일 효과
 *     30초 후: MOE disable → 모든 FET OFF (Hi-Z)
 *
 *   Electromagnetic brake (본 프로젝트) 동작:
 *     disengage() → kStateRated(24V, 300ms) → kStateHolding(20V, 지속)
 *     engage()    → duty=0 → 코일 OFF → 스프링이 모터 잠금
 *     무전원 시 자동 체결 (fail-safe) ✓
 */

#ifndef INCLUDE_EBRAKE_H_
#define INCLUDE_EBRAKE_H_

#include <stdint.h>
#include <stdbool.h>
#include "main.h"
#include "user_params.h"

#ifdef __cplusplus

class Ebrake {
 public:
  /* ---- Default voltage parameters ---- */

  //! Rated coil voltage (V) — initial disengage phase. Coil nameplate: 24 Vdc.
  static constexpr float    kDefaultRatedVoltageV   = 24.0f;

  //! Holding coil voltage (V) — thermal steady-state while disengaged.
  //! Must remain above minimum disengage voltage (≥ 18 Vdc @ 20 °C).
  //! 20 V → I ≈ 0.43 A, P ≈ 8.7 W  (vs. 12.5 W at 24 V rated).
  static constexpr float    kDefaultHoldVoltageV    = 20.0f;

  //! Duration at rated voltage before stepping down to holding voltage (ms).
  //! Must exceed disengage time (80 ms). Default 300 ms provides ample margin.
  static constexpr uint32_t kDefaultRatedDurationMs = 300U;

  /* ---- Vbus LPF ---- */

  //! IIR LPF coefficient for Vbus filtering inside runVoltageControl().
  //! runVoltageControl() is called at 10 ms rate.
  //! α = 0.95  →  τ = −T / ln(α) = −0.01 / ln(0.95) ≈ 195 ms.
  //! Filters ADC noise and low-frequency Vbus fluctuations while tracking DC changes.
  static constexpr float kVbusLpfAlpha = 0.95f;

  //! Operating state of the brake.
  enum State {
    kStateEngaged  = 0,  //!< Brake engaged   — coil de-energised, spring locks motor
    kStateRated    = 1,  //!< Disengaging     — coil at rated voltage (initial phase)
    kStateHolding  = 2,  //!< Disengaged      — coil at reduced voltage (thermal hold)
  };

  Ebrake(void);

  //! \brief  Bind to a timer channel and start PWM output.
  //!         Must be called after MX_TIMx_Init() (CubeMX sets ARR for 20 kHz).
  //!         Starts PWM at zero duty: coil de-energised, brake engaged (safe state).
  //! \param[in] htim     Pointer to timer handle (htim8 or htim15).
  //! \param[in] channel  TIM_CHANNEL_4 (Motor1) or TIM_CHANNEL_2 (Motor2).
  void setup(TIM_HandleTypeDef *htim, uint32_t channel);

  //! \brief  Override voltage parameters (optional — defaults from class constants).
  //! \param[in] rated_V          Peak disengage voltage (V).
  //! \param[in] hold_V           Thermal holding voltage while disengaged (V).
  //! \param[in] rated_duration_ms Duration at rated voltage (ms).
  void setVoltages(float rated_V, float hold_V, uint32_t rated_duration_ms);

  //! \brief  Disengage the brake: energise coil at rated voltage.
  //!         Transitions kStateRated → kStateHolding automatically via runVoltageControl().
  //!         vbus_lpf_V is kept warm by runVoltageControl() running in all states.
  void disengage(void);

  //! \brief  Engage the brake: cut PWM → coil de-energised → spring locks motor.
  void engage(void);

  //! \brief  Voltage control loop — call every 10 ms from BearDriver_SlowADC_Update().
  //!         1) Applies IIR LPF (α=0.95, τ≈195 ms) to the raw Vbus reading.
  //!         2) Manages the kStateRated→kStateHolding phase transition.
  //!         3) Updates PWM duty using the filtered Vbus.
  //! \param[in] vbus_V  Raw DC-bus voltage (V) from motor.VdcBus_Volt.
  void runVoltageControl(float vbus_V);

  //! Current operating state.
  inline State getState(void) const { return state; }

  //! True when brake is engaged (coil de-energised, spring locks motor).
  inline bool isEngaged(void)    const { return state == kStateEngaged; }

  //! True when brake is disengaged (coil energised, motor free to rotate).
  inline bool isDisengaged(void) const { return state != kStateEngaged; }

 private:
  //! Write a duty cycle [0.0 .. 1.0] to the timer compare register.
  void  setDuty(float duty);

  //! Compute duty = target_V / vbus_V, clamped to [0, 1].
  //! Returns 0 if vbus_V is below the minimum meaningful level.
  float voltsToDuty(float target_V, float vbus_V) const;

  TIM_HandleTypeDef *htim;
  uint32_t           channel;

  float    rated_voltage_V;    //!< Rated disengage voltage (V)
  float    hold_voltage_V;     //!< Thermal holding voltage while disengaged (V)
  uint32_t rated_duration_ms;  //!< Duration in rated phase (ms)

  State    state;
  uint32_t disengage_tick_ms;  //!< HAL_GetTick() snapshot taken at disengage()
  float    vbus_lpf_V;         //!< LPF-filtered Vbus (V), used for duty calculation
};

#endif /* __cplusplus */

#endif /* INCLUDE_EBRAKE_H_ */
