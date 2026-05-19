/*
 * gatedriver.cpp
 *
 * Driver for the ST STDRIVE102BH triple half-bridge gate driver IC.
 * See gatedriver.h for a description of the device and the API.
 *
 * EN is tied HIGH on the PCB — not software-controlled.  All enable /
 * disable sequencing is done through the SD (shutdown) pin only.
 */

#include "gatedriver.h"

/* ------------------------------------------------------------------------- */
/* Construction                                                              */
/* ------------------------------------------------------------------------- */

GateDriver::GateDriver(void)
    : nstby_port(nullptr)
    , nstby_pin(0)
    , nfault_port(nullptr)
    , nfault_pin(0)
    , flag_port(nullptr)
    , flag_pin(0)
    , enabled(false)
{
}

/* ------------------------------------------------------------------------- */
/* Setup                                                                     */
/* ------------------------------------------------------------------------- */

void GateDriver::setup(GPIO_TypeDef *nstby_port,     uint16_t nstby_pin,
                       GPIO_TypeDef *nfault_port, uint16_t nfault_pin,
                       GPIO_TypeDef *flag_port,   uint16_t flag_pin)
{
  this->nstby_port  = nstby_port;
  this->nstby_pin   = nstby_pin;
  this->nfault_port = nfault_port;
  this->nfault_pin  = nfault_pin;
  this->flag_port   = flag_port;
  this->flag_pin    = flag_pin;

  /* Start in the safe state: SD held LOW so the driver outputs are
   * inhibited until powerUp() is called. */
  if (nstby_port != nullptr) {
    HAL_GPIO_WritePin(nstby_port, nstby_pin, GPIO_PIN_RESET);
  }
  enabled = false;
}

/* ------------------------------------------------------------------------- */
/* Power sequencing (SD only — EN is tied HIGH on the PCB)                   */
/* ------------------------------------------------------------------------- */

void GateDriver::powerUp(void)
{
  /* EN is permanently HIGH on the PCB, so the charge pump / bootstrap
   * supply is always active. A short settling delay before releasing SD
   * ensures the internal rails are stable after any previous shutdown. */
  HAL_Delay(kEnableStartupMs);

  /* Release SD: driver outputs now follow HIN/LIN. */
  if (nstby_port != nullptr) {
    HAL_GPIO_WritePin(nstby_port, nstby_pin, GPIO_PIN_SET);
  }

  enabled = true;
}

void GateDriver::powerDown(void)
{
  /* Assert SD to latch all MOSFETs off. */
  if (nstby_port != nullptr) {
    HAL_GPIO_WritePin(nstby_port, nstby_pin, GPIO_PIN_RESET);
  }

  enabled = false;
}

/* ------------------------------------------------------------------------- */
/* Fast runtime enable / disable (SD only)                                   */
/* ------------------------------------------------------------------------- */

void GateDriver::enable(void)
{
  if (nstby_port == nullptr) {
    return;
  }

  /* Release SD so the driver outputs follow HIN/LIN again. */
  HAL_GPIO_WritePin(nstby_port, nstby_pin, GPIO_PIN_SET);

  enabled = true;
}

void GateDriver::disable(void)
{
  if (nstby_port == nullptr) {
    return;
  }

  /* Assert SD LOW: all six MOSFETs are forced off by the gate driver. */
  HAL_GPIO_WritePin(nstby_port, nstby_pin, GPIO_PIN_RESET);

  enabled = false;
}

/* ------------------------------------------------------------------------- */
/* Fault handling                                                            */
/* ------------------------------------------------------------------------- */

bool GateDriver::isFault(void)
{
  if (nfault_port == nullptr) {
    return false;
  }

  /* nFAULT is active LOW (open-drain with external pull-up). */
  return (HAL_GPIO_ReadPin(nfault_port, nfault_pin) == GPIO_PIN_RESET);
}

bool GateDriver::getFlag(void)
{
  if (flag_port == nullptr) {
    return false;
  }

  /* Return the raw pin level. Datasheet interpretation is caller's job. */
  return (HAL_GPIO_ReadPin(flag_port, flag_pin) == GPIO_PIN_SET);
}

/* ------------------------------------------------------------------------- */
/* State decoding                                                            */
/* ------------------------------------------------------------------------- */

GateDriver::State GateDriver::getState(void)
{
  /* EN is tied HIGH on the PCB — only SD and nFAULT matter.
   * Unwired nFAULT is treated as HIGH (no fault). */
  const bool nstby_hi = (nstby_port != nullptr) &&
      (HAL_GPIO_ReadPin(nstby_port, nstby_pin) == GPIO_PIN_SET);
  const bool nfault_hi = (nfault_port == nullptr) ||
      (HAL_GPIO_ReadPin(nfault_port, nfault_pin) == GPIO_PIN_SET);

  if (!nfault_hi) {
    return kStateFault;
  }
  if (nstby_hi) {
    return kStateActive;
  }
  return kStateStandby;
}

uint32_t GateDriver::getStatusWord(void)
{
  const uint32_t nstby_hi = (nstby_port != nullptr &&
      HAL_GPIO_ReadPin(nstby_port, nstby_pin) == GPIO_PIN_SET) ? 1U : 0U;

  const uint32_t flag_hi = getFlag() ? 1U : 0U;

  /* nFAULT: unwired -> treat as HIGH (OK) so getStatusWord stays consistent
   * with getState() / isFault(). */
  const uint32_t nfault_hi = (nfault_port == nullptr ||
      HAL_GPIO_ReadPin(nfault_port, nfault_pin) == GPIO_PIN_SET) ? 1U : 0U;

  const uint32_t st = (uint32_t)getState() & 0x3U;

  return (nstby_hi     << kStatusBit_nSTBY)   |
         (flag_hi   << kStatusBit_FLAG)    |
         (nfault_hi << kStatusBit_nFAULT)  |
         (st        << kStatusShift_State);
}

void GateDriver::reset(void)
{
  if (nstby_port == nullptr) {
    return;
  }

  /* Pulse SD low to clear any latched UVLO / OT fault inside the driver,
   * then release it so the stage is ready to switch again. */
  HAL_GPIO_WritePin(nstby_port, nstby_pin, GPIO_PIN_RESET);
  HAL_Delay(kResetPulseMs);
  HAL_GPIO_WritePin(nstby_port, nstby_pin, GPIO_PIN_SET);
  HAL_Delay(kEnableStartupMs);

  enabled = true;
}

/* end of file */
