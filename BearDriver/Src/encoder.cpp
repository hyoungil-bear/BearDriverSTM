/*
 * encoder.cpp
 *
 * Adapted from TI BearDriver for STM32G474.
 * Changes: QEP → STM32 TIM encoder mode, _iq → float, FILTER_FO → PID_Filter_t
 */

#include <cmath>
#include <cstring>
#include <limits>
#include "main.h"
#include "encoder.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#define MATH_TWO_PI (2.0f * (float)M_PI)

/* First-order filter helpers (replaces TI FILTER_FO) */
static void Filter_Init(PID_Filter_t *f, float fc_hz, float fs_hz)
{
  float b0 = fc_hz * MATH_TWO_PI / fs_hz;
  float a1 = b0 - 1.0f;
  f->b0 = b0;
  f->a1 = a1;
  f->b1 = 0.0f;
  f->x1 = 0.0f;
  f->y1 = 0.0f;
}

static float Filter_Run(PID_Filter_t *f, float x)
{
  float y = f->b0 * x + f->b1 * f->x1 - f->a1 * f->y1;
  f->x1 = x;
  f->y1 = y;
  return y;
}

Encoder::Encoder(void) {
  htim = NULL;
  encoder_resolution = 0;
  hallOffsetAngle_pu = -0.0f;
  position_zero_offset = 0;
  hall_offset_cnt = 3;  // need to get 3 hall angle updates before setting offset

  qep_pstn = 0.0f;
  qep_pstn_prev = 0.0f;

  speed_rpm = 0.0f;

  high_speed_lpf_rpm = 0.0f;
  low_speed_lpf_rpm = 0.0f;

  // Time since last encoder tick is infinite
  period_sum = std::numeric_limits<uint32_t>::max();
  speriod = std::numeric_limits<uint32_t>::max();
  period_cntr = 0;
  prev_track_pos = 0;

  vel_rpm = 0.0f;

  high_speed_mode = false;
  direction = true;
  update_cntr = 0;
  overflow_cntr = 0;

  electAngle = 0.0f;
  mechAngle = 0.0f;

  num_pole_pairs = 0;
  mechAngle_pu = 0.0f;
  electAngle_pu = 0.0f;

  high_speed_rpm = 0.0f;
  low_speed_rpm = 0.0f;
  low_speed_expiration = 0;

  capture_htim = NULL;
  capture_channel = 0;
  prev_capture = 0;
  capture_overflow = 0;
  capture_tick_hz = 0;
  capture_seen = false;

  memset(&vel_filter1, 0, sizeof(PID_Filter_t));
  memset(&vel_filter2, 0, sizeof(PID_Filter_t));
}

void Encoder::init(TIM_HandleTypeDef *htim,
                   uint16_t encoder_resolution, uint16_t num_pole_pairs) {
  this->htim = htim;
  this->num_pole_pairs = num_pole_pairs;
  this->encoder_resolution = encoder_resolution;

  // Designed for 1kHz Sample rate (matches speed controller update rate)
  // First order butterworth lowpass
  const float fs = 1000.0f;  // Hz
  // High-speed path: wider BW (20 Hz) for faster dynamic response when
  // T-method is used; legacy M-method keeps 10 Hz to suppress quantisation noise.
#ifdef ENCODER_HIGH_SPEED_TMETHOD
  const float fc1 = 20.0f;  // Hz - T-method high-speed path
#else
  const float fc1 = 10.0f;  // Hz - M-method legacy
#endif
  const float fc2 = 10.0f;  // Hz - low-speed path (always)

  Filter_Init(&vel_filter1, fc1, fs);
  Filter_Init(&vel_filter2, fc2, fs);

  // SW-mode zero-speed timeout = 2 × edge period at minimum operating speed.
  // trackPeriod() runs at 10 kHz; check is (update_cntr > low_speed_expiration × 20).
  // low_speed_expiration = 2 × 60 × (10000/20) / (ENCODER_MIN_SPEED_RPM × resolution)
  //                      = 2 × 60 × 500 / (ENCODER_MIN_SPEED_RPM × resolution)
  // 1024 CPR → 1172,  4096 CPR → 293.  Consistent with HW capture timeout.
  low_speed_expiration = (uint32_t)(2.0f * 60.0f * 500.0f /
                                    (ENCODER_MIN_SPEED_RPM * (float)encoder_resolution));

  // SW-mode: first edge after init is trusted (no prior direction change).
  // capture_seen is shared with HW capture mode; initCapture() resets it to false.
  capture_seen = true;

  // Start encoder timer
  if (htim != NULL) {
    HAL_TIM_Encoder_Start(htim, TIM_CHANNEL_ALL);
  }
}

// Software replacement for TI QEP capture unit.
// Called at ISR rate (10kHz) to track encoder position changes
// and measure the period between edges for low-speed estimation.
//
// When hardware input capture is enabled (capture_htim != NULL), only
// direction tracking from the TIM2 CR1.DIR bit is done here; period
// measurement is performed in handleCapture().
void Encoder::trackPeriod(uint32_t raw_counter) {
  int32_t pos = (int32_t)raw_counter;
  int32_t delta = pos - prev_track_pos;

  // Handle counter wrap (TIM2 ARR = encoder_resolution * 4 - 1)
  int32_t half = (int32_t)encoder_resolution * 2;
  if (delta > half)       delta -= (int32_t)encoder_resolution * 4;
  else if (delta < -half) delta += (int32_t)encoder_resolution * 4;

  // Read hardware direction from TIM2 CR1.DIR bit
  // DIR=0: upcounting (positive), DIR=1: downcounting (negative)
  bool hw_dir = !(htim->Instance->CR1 & TIM_CR1_DIR);

  if (capture_htim != NULL) {
    // Hardware capture mode: only track direction for sign.
    // Direction change forces speriod = MAX to discard stale period,
    // and clears the capture anchor so the next edge re-anchors fresh.
    if ((delta != 0) && (direction != hw_dir)) {
      speriod = std::numeric_limits<uint32_t>::max();
      capture_seen = false;
    }
    direction = hw_dir;
    prev_track_pos = pos;
    return;
  }

  period_cntr++;

  if (delta != 0) {
    if (direction != hw_dir) {
      // Direction changed — force zero velocity.
      // capture_seen = false: the next same-direction edge's period_cntr spans
      // the reversal interval and is too short to be a valid period.
      // Discard it (keep speriod = MAX) to prevent a false high-speed spike.
      speriod = std::numeric_limits<uint32_t>::max();
      capture_seen = false;
    } else {
      // Same direction as before
      if (!capture_seen) {
        // First same-direction edge after a reversal: anchor only.
        // period_cntr was reset at the direction-change edge, so its value
        // is the short cross-reversal interval — not a valid speed period.
        // Keep speriod = MAX (vel stays 0) and mark anchor as seen.
        capture_seen = true;
      } else {
        // Normal case: valid same-direction period.
        speriod = period_cntr;
      }
    }
    direction = hw_dir;
    period_sum = 0;
    period_cntr = 0;
    update_cntr = 0;
  } else {
    // No change — accumulate time since last change
    period_sum = period_cntr;
    update_cntr++;
  }

  prev_track_pos = pos;
}

void Encoder::updateValues(void) {
  if (htim == NULL) return;

#ifndef ENCODER_HIGH_SPEED_TMETHOD
  // M-method (legacy): velocity from angle change over fixed 1 ms sample interval.
  // Resolution: 1 count / (4 × encoder_resolution × 0.001 s)
  //   1024 CPR → ±14.6 RPM/LSB,  4096 CPR → ±3.7 RPM/LSB
  {
    int32_t position = (int32_t)__HAL_TIM_GET_COUNTER(htim);

    qep_pstn_prev = qep_pstn;
    qep_pstn = (MATH_TWO_PI / (4.0f * (float)encoder_resolution)) * (float)position;

    float angle_delta_rad = qep_pstn - qep_pstn_prev;
    if (angle_delta_rad < -M_PI) angle_delta_rad += MATH_TWO_PI;
    else if (angle_delta_rad >  M_PI) angle_delta_rad -= MATH_TWO_PI;

    // dTheta / 2pi / 1ms * 60 → RPM
    speed_rpm = angle_delta_rad * (1000.0f * 60.0f / MATH_TWO_PI);
    high_speed_rpm = speed_rpm;
  }
#endif

  // Low speed estimation: period measured by trackPeriod() at 10kHz (ISR rate)
  // or by hardware input capture when capture_htim != NULL.
  float vel;

  if (capture_htim != NULL) {
    // Slave-reset mode: counter resets to 0 on each rising edge.
    // CC1IF is set by hardware on every capture; reading CCR1 clears it.
    // Check CC1IF BEFORE reading CCR1 so consecutive same-period edges
    // (identical CCR1 values) are not silently dropped.
    bool new_edge = (capture_htim->Instance->SR & TIM_SR_CC1IF) != 0u;

    uint32_t ov;
    uint16_t ccr, cnt;
    do {
      ov  = capture_overflow;
      ccr = (uint16_t)capture_htim->Instance->CCR1;  // reading clears CC1IF
      cnt = (uint16_t)__HAL_TIM_GET_COUNTER(capture_htim);
    } while (ov != capture_overflow);

    if (new_edge) {
      // Full period = overflows-since-last-edge × 65536 + CCR1.
      // Required below ~9 RPM where the edge interval exceeds one overflow (6.5 ms).
      uint32_t full_period = ((uint32_t)ov << 16) | ccr;
      prev_capture = ccr;
      capture_overflow = 0u;  // restart idle counter (safe: counter just reset)
      period_sum = 0u;

      if (capture_seen) {
        // Valid same-direction edge: full_period is the true edge-to-edge time.
        speriod = (full_period > 0u) ? full_period : 1u;
      } else {
        // First edge after direction change or timeout.
        // full_period spans the cross-direction interval → speed is invalid.
        // Mark anchor only; keep speriod=MAX until the next same-dir edge.
        capture_seen = true;
      }
    } else {
      // No new edge: accumulate idle time since last detected edge.
      period_sum = ((uint32_t)ov << 16) | cnt;
    }

    // RPM = 60 * tick_hz / (edges_per_rev * period_ticks)
    // One rising edge per encoder line → edges_per_rev = encoder_resolution.
    uint32_t cur_period = (period_sum > speriod) ? period_sum : speriod;

    // Zero-speed timeout = 2 × edge period at minimum operating speed.
    uint32_t timeout_ticks = (uint32_t)(2.0f * 60.0f * (float)capture_tick_hz /
                                        (ENCODER_MIN_SPEED_RPM * (float)encoder_resolution));

    if (!capture_seen || cur_period == 0u || period_sum > timeout_ticks) {
      vel = 0.0f;
      if (period_sum > timeout_ticks) {
        capture_seen = false;
        speriod = std::numeric_limits<uint32_t>::max();
      }
    } else {
      vel = (60.0f * (float)capture_tick_hz) /
            ((float)encoder_resolution * (float)cur_period);
    }
  } else {
    // Software mode: period_sum/speriod in ISR ticks (0.1ms each).
    // One encoder count = 1/(4 * encoder_resolution) of a revolution.
    // RPM = ISR_FREQ * 60 / (4 * encoder_resolution * period_ticks)
    uint32_t cur_period = period_sum > speriod ? period_sum : speriod;

    if (update_cntr > low_speed_expiration * 20) {
      vel = 0.0f;
    } else {
      vel = (10000.0f * 60.0f / (4.0f * (float)encoder_resolution)) /
            (float)cur_period;
    }
  }

  if (direction) {
    low_speed_rpm = vel;
  } else {
    low_speed_rpm = -vel;
  }

#ifdef ENCODER_HIGH_SPEED_TMETHOD
  // T-method high-speed path: reuse capture-period velocity.
  // Resolution at 80 RPM, 10 MHz clock:
  //   T-method → ±0.011 RPM/LSB  vs  M-method → ±14.6 RPM/LSB
  // vel_filter1 uses fc = 20 Hz (wider BW) for faster dynamic response;
  // vel_filter2 keeps fc = 10 Hz for smooth low-speed output.
  // Hysteresis boundary (40 / 30 RPM) and filter states remain independent.
  high_speed_rpm = low_speed_rpm;
#endif

  // Apply LPF to each estimate
  high_speed_lpf_rpm = Filter_Run(&vel_filter1, high_speed_rpm);
  low_speed_lpf_rpm  = Filter_Run(&vel_filter2, low_speed_rpm);

  // decide whether to use high-speed or low-speed estimate using hysteresis
  float vel_abs = fabsf(low_speed_lpf_rpm);

  if ((vel_abs > 40.0f) && (high_speed_mode == false)) {
    high_speed_mode = true;
  } else if ((vel_abs < 30.0f) && (high_speed_mode == true)) {
    high_speed_mode = false;
  }

  if (high_speed_mode == true) {
    vel_rpm = high_speed_lpf_rpm;
  } else {
    vel_rpm = low_speed_lpf_rpm;
  }

  return;
}

void Encoder::setHallOffset(float hallOffsetAngle_pu) {
  if (hall_offset_cnt > 0) {
    hall_offset_cnt--;
    this->hallOffsetAngle_pu = hallOffsetAngle_pu;
    position_zero_offset =
        (int32_t)__HAL_TIM_GET_COUNTER(htim) -
        (int32_t)((hallOffsetAngle_pu - (float)(1.0f / 12.0f)) *
         (float)((float)encoder_resolution * 4.0f /
                 (float)num_pole_pairs));
  }
}

float Encoder::getMechAnglePU(uint32_t posn_count) {
  int32_t position = ((int32_t)posn_count - position_zero_offset);

  float angle_pu =
      (float)position *
      (float)(1.0f / ((float)encoder_resolution * 4.0f));

  // Wrap to [0, 1) — while loops handle edge cases where
  // position_zero_offset exceeds counter range
  while (angle_pu < 0.0f)  angle_pu += 1.0f;
  while (angle_pu >= 1.0f) angle_pu -= 1.0f;

  mechAngle_pu = angle_pu;
  mechAngle = mechAngle_pu;  // for data logger
  return mechAngle_pu;
}

float Encoder::getElectAnglePU(void) {
  float angle_float = mechAngle_pu * (float)num_pole_pairs;
  int32_t angle_int = (int32_t)angle_float;
  float angle_pu = angle_float - (float)angle_int;

  electAngle_pu = angle_pu;
  electAngle = electAngle_pu;  // for data logger
  return electAngle_pu;
}

int32_t Encoder::getPositionCounterWOffset(void) {
  return (((int32_t)__HAL_TIM_GET_COUNTER(htim)) - position_zero_offset);
}

uint32_t Encoder::getPositionCounter(void) {
  return __HAL_TIM_GET_COUNTER(htim);
}

// **************************************************************************
// Hardware input-capture interface

// Enable hardware input-capture mode for low-speed period measurement.
// Caller must have configured the given timer/channel for input capture
// (rising edge) on the encoder signal before calling this.
void Encoder::initCapture(TIM_HandleTypeDef *cap_htim, uint32_t channel) {
  if (cap_htim == NULL) return;

  capture_htim = cap_htim;
  capture_channel = channel;
  prev_capture = 0;
  capture_overflow = 0;
  capture_seen = false;
  speriod = std::numeric_limits<uint32_t>::max();
  period_sum = std::numeric_limits<uint32_t>::max();

  // Compute tick frequency from timer PSC and APB1 clock.
  // Timer kernel clock = PCLK1 when APB1 prescaler = 1, else 2 * PCLK1.
  uint32_t pclk1 = HAL_RCC_GetPCLK1Freq();
  RCC_ClkInitTypeDef clk_cfg;
  uint32_t flash_latency;
  HAL_RCC_GetClockConfig(&clk_cfg, &flash_latency);
  uint32_t timer_clk = (clk_cfg.APB1CLKDivider == RCC_HCLK_DIV1)
                           ? pclk1
                           : (pclk1 * 2u);
  capture_tick_hz = timer_clk / (cap_htim->Instance->PSC + 1u);

  // Slave reset mode: counter resets to 0 on each rising edge of TI1.
  // CCR1 captures the count before reset → direct edge-to-edge period.
  // UIE now fires only when no edge arrives within 6.5 ms (< ~9 RPM).
  // CC interrupt disabled (HAL_TIM_IC_Start does not clear DIER.CC1IE).
  __HAL_TIM_SET_COUNTER(cap_htim, 0);
  __HAL_TIM_DISABLE_IT(cap_htim, TIM_IT_CC1);
  HAL_TIM_IC_Start(cap_htim, channel);

  TIM_SlaveConfigTypeDef sSlave = {};
  sSlave.SlaveMode        = TIM_SLAVEMODE_RESET;
  sSlave.InputTrigger     = TIM_TS_TI1FP1;
  sSlave.TriggerPolarity  = TIM_TRIGGERPOLARITY_RISING;
  sSlave.TriggerPrescaler = TIM_TRIGGERPRESCALER_DIV1;
  sSlave.TriggerFilter    = 0;
  HAL_TIM_SlaveConfigSynchro(cap_htim, &sSlave);

  // URS=1: Update interrupt fires only on counter overflow/underflow.
  // Without this, the slave-reset event (each encoder edge) also generates
  // a UEV and triggers UIE → ISR fires at encoder frequency, not ~152 Hz.
  cap_htim->Instance->CR1 |= TIM_CR1_URS;

  __HAL_TIM_CLEAR_FLAG(cap_htim, TIM_FLAG_UPDATE);
  __HAL_TIM_ENABLE_IT(cap_htim, TIM_IT_UPDATE);

  // Enable NVIC for this timer so overflow (and capture in legacy mode)
  // interrupts actually reach HAL_TIM_IRQHandler → callbacks.
  IRQn_Type irqn = TIM3_IRQn;  // default
  if (cap_htim->Instance == TIM3) irqn = TIM3_IRQn;
  else if (cap_htim->Instance == TIM4) irqn = TIM4_IRQn;
  HAL_NVIC_SetPriority(irqn, 3, 0);
  HAL_NVIC_EnableIRQ(irqn);
}

// Called from HAL_TIM_PeriodElapsedCallback on 16-bit counter wrap (~152 Hz).
// Used to extend effective period measurement beyond the 65.535 ms
// timer range (at 1 MHz tick) for very low speeds.
void Encoder::handleCaptureOverflow(void) {
  if (capture_htim == NULL) return;
  // Free-running 32-bit wrap counter. Subtractions against it use modular
  // arithmetic and remain correct for any delta below 2^31 ticks
  // (~214 s at 10 MHz, far beyond the 600 ms timeout).
  capture_overflow++;
}
