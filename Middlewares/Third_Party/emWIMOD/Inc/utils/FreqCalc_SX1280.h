//------------------------------------------------------------------------------
//! @file FreqCalc_SX1280.h
//! @ingroup Utils
//! <!------------------------------------------------------------------------->
//! @brief Helper Utility to calc frequency to transceiver register and vice versa for the SX128x transceiver series
//! @version 0.1
//! <!------------------------------------------------------------------------->
//!
//!
//!
//! <!--------------------------------------------------------------------------
//! Copyright (c) 2018
//! IMST GmbH
//! Carl-Friedrich Gauss Str. 2
//! 47475 Kamp-Lintfort
//! --------------------------------------------------------------------------->
//! @author (FB), IMST
//! <!--------------------------------------------------------------------------
//! Target OS:    none
//! Target CPU:   tbd
//! Compiler:     tbd
//! --------------------------------------------------------------------------->
//! @internal
//! @par Revision History:
//! <PRE>
//!-----------------------------------------------------------------------------
//! Version | Date       | Author | Comment
//!-----------------------------------------------------------------------------
//!
//! </PRE>
//------------------------------------------------------------------------------


#ifndef ARDUINO_UTILS_FREQCALC_SX1280_H_
#define ARDUINO_UTILS_FREQCALC_SX1280_H_

//------------------------------------------------------------------------------
//
// Section Includes Files
//
//------------------------------------------------------------------------------


#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void     FreqCalcSX1280_calcFreqToRegister(uint32_t freq, uint8_t* msb, uint8_t* mid, uint8_t* lsb);
uint32_t FreqCalcSX1280_calcRegisterToFreq(uint8_t msb, uint8_t mid, uint8_t lsb);

#ifdef __cplusplus
}
#endif




#endif /* ARDUINO_UTILS_FREQCALC_SX1280_H_ */
