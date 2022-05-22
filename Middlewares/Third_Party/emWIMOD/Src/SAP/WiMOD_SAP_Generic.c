//------------------------------------------------------------------------------
//! @file WiMOD_SAP_Generic.cpp
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


//------------------------------------------------------------------------------
//
// Section Includes Files
//
//------------------------------------------------------------------------------

#include <string.h>
#include "../SAP/WiMOD_SAP_Generic.h"

//------------------------------------------------------------------------------
//
// Section public functions
//
//------------------------------------------------------------------------------
static TWiMODLRResultCodes executeGenericCmd(TWiMODLR_Generic_CmdInfo* info, UINT8* statusRsp);
//-----------------------------------------------------------------------------
/**
 * @brief Constructor
 *
 * @param hci       Pointer to HCI processor object
 *
 * @param buffer    pointer to storage area for building tx frames; MUST BE VALID
 *
 * @param bufferSize    size of the buffer
 *
 */
WiMOD_SAP_Generic_t WiMOD_SAP_Generic = {
		executeGenericCmd,
		NULL,
		0x00,
};

////-----------------------------------------------------------------------------
///**
// * @brief Destructor
// */
//WiMOD_SAP_Generic::~WiMOD_SAP_Generic(void)
//{
//
//}


//-----------------------------------------------------------------------------
/**
 * @brief Reset Cmd - Do a reset / reboot of the WiMOD
 *
 *
 * @param   info        pointer to store the received information
 * @param   statusRsp   pointer to store status byte of response mesg from WiMOD
 *
 * @retval WiMODLR_RESULT_OK     if command transmit to WiMOD was ok
 */
TWiMODLRResultCodes executeGenericCmd(TWiMODLR_Generic_CmdInfo* info, UINT8* statusRsp)
{
    TWiMODLRResultCodes result = WiMODLR_RESULT_TRANMIT_ERROR;
    UINT8              offset = WiMODLR_HCI_RSP_CMD_PAYLOAD_POS;
    UINT16			   payLen;

    if (info && statusRsp) {

    	payLen = MIN(WiMOD_SAP_Generic.txyPayloadSize, info->CmdPayloadLength);

    	memcpy(WiMOD_SAP_Generic.txPayload, info->CmdPayload, payLen);

        result = WiMOD_SAP_Generic.HciParser->SendHCIMessage(info->SapID,
                                           info->MsgReqID,
                                           info->MsgRspID,
										   WiMOD_SAP_Generic.txPayload, payLen);

        if (result == WiMODLR_RESULT_OK) {
            const TWiMODLR_HCIMessage* rx = WiMOD_SAP_Generic.HciParser->GetRxMessage();

            // extract data from response
            info->Status     = rx->Payload[WiMODLR_HCI_RSP_STATUS_POS];

            info->CmdPayloadLength = 0;
            if (rx->Length > WiMODLR_HCI_RSP_CMD_PAYLOAD_POS) {
				info->CmdPayloadLength = MIN(rx->Length-WiMODLR_HCI_RSP_CMD_PAYLOAD_POS, WiMOD_GENERIC_MSG_SIZE);
				memcpy(info->CmdPayload, &rx->Payload[offset], info->CmdPayloadLength);

            }

            *statusRsp = rx->Payload[WiMODLR_HCI_RSP_STATUS_POS];
        }
    }
    return result;
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


