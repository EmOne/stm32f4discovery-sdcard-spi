/*
 * StrToIntConverter.cpp
 *
 *  Created on: Aug 8, 2019
 *      Author: tux
 */


//------------------------------------------------------------------------------
//! @file StrToIntConverter.cpp
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

//------------------------------------------------------------------------------
//
// Section Includes Files
//
//------------------------------------------------------------------------------

#include "StrToIntConverter.h"

#include <stdlib.h>
#include <string.h>


//------------------------------------------------------------------------------
//
// Section public functions
//
//------------------------------------------------------------------------------

/**
 * @brief Convert a String containing hex numbers into an C array
 *
 * @param inputString   The C String that contains the numbers e.g. "12 0x34 AB"
 *
 * @param targetArray   The array where to store the converted numbers
 *
 * @param numberOfEntries   The number of numbers contained in the string (e.g. 3)
 */
void StrToIntConverter_convertHexStrToArray(char* inputString, uint8_t* targetArray, uint8_t numberOfEntries )
{
    char    *eptr;
    char    *token;
    long    tmp;
    uint8_t i;

    if (inputString && targetArray && numberOfEntries > 0) {

        token = inputString;
        for(i = 0; i < numberOfEntries; i++) {
            tmp = strtol(token, &eptr, 16);
            targetArray[i] = (uint8_t) tmp & 0xFF;
            token = eptr;
        }
    }
    return;
}


//------------------------------------------------------------------------------
//
// Section protected functions
//
//------------------------------------------------------------------------------


//------------------------------------------------------------------------------
//
// Section private functions
//
//------------------------------------------------------------------------------


