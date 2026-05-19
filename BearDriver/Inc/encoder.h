/*
 * encoder.h
 *
 * Quadrature encoder interface for position and velocity estimation.
 * Adapted from TI BearDriver for STM32G474.
 * Changes: QEP_Handle → TIM_HandleTypeDef*, _iq → float, HAL_Handle removed
 */

#ifndef INCLUDE_ENCODER_H_
#define INCLUDE_ENCODER_H_

#include <stdint.h>
#include <stdbool.h>
#include "main.h"
#include "pid.h"
#include "user_params.h"

/* Minimum operating speed used to compute the zero-speed detection timeout.
 * Zero-speed is declared after the motor has produced no encoder edge for
 * 2 × edge_period_at_min_speed:
 *
 *   edge_period = 60 s / (ENCODER_MIN_SPEED_RPM × encoder_resolution)
 *   timeout     = 2 × edge_period
 *
 * HW capture (slave-reset, 10 MHz tick):
 *   timeout_ticks = 2 × 60 × capture_tick_hz / (ENCODER_MIN_SPEED_RPM × resolution)
 *   1024 CPR → 23,437,500 ticks = 2.34 s
 *   4096 CPR →  5,859,375 ticks = 0.59 s
 *
 * SW mode (trackPeriod at 10 kHz, check: update_cntr > low_speed_expiration × 20):
 *   low_speed_expiration = 2 × 60 × 500 / (ENCODER_MIN_SPEED_RPM × resolution)
 *   1024 CPR → 1172,  4096 CPR → 293
 */
#define ENCODER_MIN_SPEED_RPM (0.05f)

/* When defined, the high-speed estimation path also uses the T-method
 * (hardware capture period) instead of the legacy M-method (angle delta).
 *
 * Resolution comparison at 80 RPM, 1024 CPR, 10 MHz capture clock:
 *   M-method (1 kHz update):  1 count / (4 × 1024 × 0.001 s) = ±14.6 RPM/LSB
 *   T-method (10 MHz capture): 80 RPM / 7300 ticks            = ±0.011 RPM/LSB
 *
 * Both paths retain independent LPF state (vel_filter1 / vel_filter2) so
 * the high-/low-speed hysteresis switch remains unchanged.
 * With T-method, vel_filter1 (high-speed path) uses fc = 20 Hz for faster
 * dynamic response; vel_filter2 (low-speed path) keeps fc = 10 Hz.
 *
 * Comment out to revert to the original M-method + 10 Hz filter for high speed.
 */
#define ENCODER_HIGH_SPEED_TMETHOD

#ifdef __cplusplus

class Encoder {
 private:
  TIM_HandleTypeDef *htim;

  PID_Filter_t vel_filter1;
  PID_Filter_t vel_filter2;

  uint16_t encoder_resolution;

  uint32_t update_cntr;    // number of iterations since last update
  uint32_t overflow_cntr;  // number of overflow events since last update

  float qep_pstn_prev;    // previous encoder angle (radians)
  float qep_pstn;         // current encoder angle (radians)

  uint32_t period_sum;    // time since last position change (ticks)
  uint32_t speriod;       // last measured period between position changes (ticks)
  uint32_t period_cntr;   // SW-mode: ISR tick counter for period measurement
  int32_t  prev_track_pos; // SW-mode: previous raw counter for period tracking
  bool direction;         // direction (true = positive, false = negative)

  // Hardware input-capture (TIM3/TIM4), slave-reset mode.
  // Counter resets to 0 on each rising edge; CCR1 = direct edge-to-edge
  // period. UIE fires only when motor stopped (counter wraps at ~152 Hz).
  TIM_HandleTypeDef *capture_htim;
  uint32_t capture_channel;    // TIM_CHANNEL_1..4
  uint16_t prev_capture;       // last polled CCR1 value (edge detection)
  uint32_t capture_overflow;   // idle wrap counter; reset on each new edge
  uint32_t capture_tick_hz;    // capture timer tick frequency (Hz, 10 MHz)
  bool     capture_seen;       // first edge received flag

  float speed_rpm;       // current estimated speed (RPM)

  float high_speed_lpf_rpm;  // filtered high speed estimate (RPM)
  float low_speed_lpf_rpm;   // filtered low speed estimate (RPM)

  float high_speed_rpm;      // unfiltered high speed estimate (RPM)
  float low_speed_rpm;       // unfiltered low speed estimate (RPM)

  bool high_speed_mode;

  float vel_rpm;

  float hallOffsetAngle_pu;
  int32_t position_zero_offset;   // offset to 'zero' position to match hall sensor angles
  uint32_t low_speed_expiration;  // number of encoder cycles without updates before speed = 0

  uint16_t num_pole_pairs;
  float mechAngle_pu;
  float electAngle_pu;

 public:  // make public for data log
  float electAngle;
  float mechAngle;

  uint16_t hall_offset_cnt;

 public:
  Encoder(void);

  void init(TIM_HandleTypeDef *htim,
            uint16_t encoder_resolution, uint16_t num_pole_pairs);

  float getPosition(void) { return qep_pstn; };
  int32_t getPeriod(void) { return speriod; };
  int32_t getPeriodSum(void) { return period_sum; };

  float getVelocity(void) { return vel_rpm; };
  float getVelocityHighLPF(void) { return high_speed_lpf_rpm; };
  float getVelocityLowLPF(void) { return low_speed_lpf_rpm; };
  float getVelocityHigh(void) { return high_speed_rpm; };
  float getVelocityLow(void) { return low_speed_rpm; };

  void setHallOffset(float hallOffsetAngle_pu);
  void trackPeriod(uint32_t raw_counter);  // call at ISR rate (10kHz)
  void updateValues(void);                 // call at 500Hz (decimated from ISR)

  // Hardware input-capture mode for low-speed period measurement.
  // When enabled via initCapture(), each rising edge on the capture input
  // records the time since the previous edge with sub-microsecond
  // resolution, replacing the software trackPeriod() fallback.
  void initCapture(TIM_HandleTypeDef *cap_htim, uint32_t channel);
  void handleCaptureOverflow(void);  // from HAL_TIM_PeriodElapsedCallback (UIE)

  inline TIM_HandleTypeDef* getTimHandle(void) { return htim; };
  inline TIM_HandleTypeDef* getCaptureTimHandle(void) { return capture_htim; };

  float getMechAnglePU(uint32_t posn_count);
  float getElectAnglePU(void);
  int32_t getPositionCounterWOffset(void);
  uint32_t getPositionCounter(void);
  uint32_t getEncoderExpiration(void) { return low_speed_expiration; };
  void setEncoderExpiration(uint32_t tics) { low_speed_expiration = tics; };
};

#endif /* __cplusplus */

#endif /* INCLUDE_ENCODER_H_ */
