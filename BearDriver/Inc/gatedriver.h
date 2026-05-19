/*
 * gatedriver.h
 *
 * Driver for the ST STDRIVE102BH triple half-bridge gate driver IC.
 *
 * The STDRIVE102BH is a high-voltage (up to 75 V) triple half-bridge gate
 * driver for N-channel MOSFETs. It is a pin-controlled device (NO SPI):
 *
 *   - 6 logic inputs (HIN/LIN per phase) driven directly by the MCU PWM
 *     peripheral. These are NOT managed by this driver - the PWM timer
 *     drives them.
 *   - EN  (enable) : tied HIGH on the PCB — always active, not software
 *                     controlled.
 *   - SD  (shutdown, active LOW) : forces all outputs low when asserted.
 *   - OUT / nFAULT (active LOW, open-drain) : asserted on UVLO (VCC/VBOOT),
 *                                             thermal shutdown, etc.
 *   - DT  (dead-time) : hardware-configured via external resistor; the MCU
 *                       has no control over this.
 *
 * Runtime controls exposed by this class:
 *   - powerUp()  : wait for bootstrap/charge-pump settle, then release SD.
 *                  Use once at boot.
 *   - powerDown(): assert SD LOW. Outputs inhibited until powerUp() again.
 *   - enable()   : fast runtime activation - release SD.
 *   - disable()  : fast runtime shutdown - assert SD.
 *   - isFault()  : read the nFAULT input (active LOW).
 *   - getFlag()  : read the FLAG status input (raw level).
 *   - reset()    : pulse SD low to clear a latched fault.
 *
 * One instance should be created per motor (each motor has its own
 * STDRIVE102BH). Pin mapping is supplied at setup() time so the same class
 * can serve motor 1 and motor 2 independently. Any pin that is not wired
 * on a particular motor may be passed as (nullptr, 0); the corresponding
 * operations become no-ops / return false.
 */

#ifndef INCLUDE_GATEDRIVER_H_
#define INCLUDE_GATEDRIVER_H_

#include <stdint.h>
#include <stdbool.h>
#include "main.h"

#ifdef __cplusplus

//! \brief  STDRIVE102BH gate driver interface (EN tied HIGH on PCB)
class GateDriver {
 public:
  //! \brief  Minimum SD pulse width required to reset a latched fault (ms)
  //!         Datasheet specifies ~1 us; we use 1 ms for HAL_Delay resolution.
  static constexpr uint32_t kResetPulseMs = 1U;

  //! \brief  Delay after releasing SD before driver is ready to switch (ms)
  //!         Allows the internal bootstrap/charge-pump to stabilise.
  static constexpr uint32_t kEnableStartupMs = 2U;

  //! \brief  Decoded gate driver state derived from the SD / nFAULT
  //!         combination.  EN is tied HIGH on the PCB and not read.
  //!         FLAG is carried raw in the status word but does not influence
  //!         the state decision (interpretation is datasheet-specific and
  //!         left to the caller).
  enum State {
    kStateActive  = 0,  //!< SD=H, nFAULT=H : normal operation (healthy)
    kStateStandby = 1,  //!< SD=L, nFAULT=H : outputs inhibited
    kStateFault   = 2,  //!< nFAULT=L        : latched UVLO / OT fault
  };

  //! \brief  Bit layout of getStatusWord() return value.
  enum StatusBits {
    kStatusBit_nSTBY   = 0,   //!< nSTBY (SD) level  (1=HIGH)
    kStatusBit_FLAG    = 1,   //!< FLAG pin level    (raw)
    kStatusBit_nFAULT  = 2,   //!< nFAULT pin level  (1=HIGH=OK)
    kStatusShift_State = 3,   //!< State enum occupies bits 3..4
    kStatusMask_State  = 0x3U << 3,  //!< 2-bit mask for state enum
  };

  GateDriver(void);

  //! \brief  Configure the GPIO pins connected to the STDRIVE102BH.
  //!         EN is tied HIGH on the PCB — not software-controlled.
  //! \param[in] nstby_port,nstby_pin           SD pin (active LOW output from MCU)
  //!                                     Fast runtime shutdown.
  //! \param[in] nfault_port,nfault_pin   nFAULT pin (active LOW input to MCU,
  //!                                     open-drain). UVLO / OT error.
  //! \param[in] flag_port,flag_pin       FLAG pin (input to MCU). IC status
  //!                                     output; raw level returned by
  //!                                     getFlag(). Interpretation per
  //!                                     datasheet.
  //!
  //!         Any pin pair may be passed as (nullptr, 0) if that signal is
  //!         not wired on this motor. Operations on unwired pins become
  //!         no-ops; read operations return false.
  void setup(GPIO_TypeDef *nstby_port,     uint16_t nstby_pin,
             GPIO_TypeDef *nfault_port, uint16_t nfault_pin,
             GPIO_TypeDef *flag_port,   uint16_t flag_pin);

  //! \brief  Power-up sequence: wait for the bootstrap / charge-pump to
  //!         settle (EN is already HIGH on PCB), then release SD.
  //!         Use once at boot.
  void powerUp(void);

  //! \brief  Power-down: assert SD LOW. Outputs are inhibited until
  //!         powerUp() is called again.
  void powerDown(void);

  //! \brief  Fast runtime activation: release SD.
  void enable(void);

  //! \brief  Fast runtime shutdown: assert SD.
  void disable(void);

  //! \brief  True if the driver has been enabled and not disabled since.
  inline bool isEnabled(void) const { return enabled; }

  //! \brief  Read the nFAULT pin. Returns true when a fault is present.
  //!         Returns false if no nFAULT pin was configured.
  bool isFault(void);

  //! \brief  Read the FLAG pin as a raw level. Returns true if the pin is
  //!         HIGH, false if LOW or not configured. Caller is responsible
  //!         for interpreting the signal per the STDRIVE102BH datasheet.
  bool getFlag(void);

  //! \brief  Decode the current SD / nFAULT combination into a single
  //!         State value. See the State enum for the mapping.
  State getState(void);

  //! \brief  Pack the raw pin levels and decoded state into a single
  //!         word suitable for embedding in Motor::getMotorStatus().
  //!         See StatusBits for the layout.
  uint32_t getStatusWord(void);

  //! \brief  Pulse SD low to clear a latched UVLO / thermal fault.
  //!         After this call the driver is re-enabled and ready to switch.
  void reset(void);

 private:
  GPIO_TypeDef *nstby_port;
  uint16_t      nstby_pin;

  GPIO_TypeDef *nfault_port;
  uint16_t      nfault_pin;

  GPIO_TypeDef *flag_port;
  uint16_t      flag_pin;

  bool          enabled;
};

#endif /* __cplusplus */

#endif /* INCLUDE_GATEDRIVER_H_ */
