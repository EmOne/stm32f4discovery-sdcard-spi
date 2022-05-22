//------------------------------------------------------------------------------
//! @file WiMOD_SAP_Generic_IDs.h
//! @ingroup WIMOD_SAP_Generic
//! <!------------------------------------------------------------------------->
//! @brief Generic HCI commmand interface
//! @version 0.1
//! <!------------------------------------------------------------------------->
//!
//!
//!
//! <!--------------------------------------------------------------------------
//! Copyright (c) 2017
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


#ifndef SAP_WIMOD_SAP_GENERIC_IDS_H_
#define SAP_WIMOD_SAP_GENERIC_IDS_H_

//------------------------------------------------------------------------------
//
// Section Includes Files
//
//------------------------------------------------------------------------------

#include "../utils/WMDefs.h"

//------------------------------------------------------------------------------
//
// Section defines
//
//------------------------------------------------------------------------------


/** Buffer size for Generic related messages */
#define WiMOD_GENERIC_MSG_SIZE               (100)


//------------------------------------------------------------------------------
//
// structures
//
//------------------------------------------------------------------------------

//! @cond Doxygen_Suppress

/**
 * @brief Structure containing basic information about the WiMOD device
 */
typedef struct TWiMODLR_Generic_CmdInfo
{
    UINT8       Status;                                                         /*!< status flag; indicates if other values are vaild */
    UINT8       SapID;	                                                        /*!< SAP ID of command to be executed */
    UINT8       MsgReqID;                                                       /*!< message ID of request */
    UINT8       MsgRspID;                                                       /*!< message ID of response */
    UINT16      CmdPayloadLength;                                               /*!< length of payload bytes */
    UINT8       CmdPayload[WiMOD_GENERIC_MSG_SIZE];                             /*!< the ID of the WiMOD */
} TWiMODLR_Generic_CmdInfo;

//! @endcond


#endif /* SAP_WIMOD_SAP_GENERIC_IDS_H_ */
