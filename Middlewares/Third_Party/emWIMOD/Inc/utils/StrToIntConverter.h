/*
 * StrToIntConverter.h
 *
 *  Created on: Aug 8, 2019
 *      Author: tux
 */

//------------------------------------------------------------------------------
//! @file StrToIntConverter.h
//! @ingroup
//! <!------------------------------------------------------------------------->
//! @brief
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


#ifndef ARDUINO_UTILS_STRTOINTCONVERTER_H_
#define ARDUINO_UTILS_STRTOINTCONVERTER_H_

//------------------------------------------------------------------------------
//
// Section Includes Files
//
//------------------------------------------------------------------------------

#include <stdint.h>




void StrToIntConverter_convertHexStrToArray(char*       inputString,
                                            uint8_t*    targetArray,
                                            uint8_t     numberOfEntries );



#endif /* ARDUINO_UTILS_STRTOINTCONVERTER_H_ */