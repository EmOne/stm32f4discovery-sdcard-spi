//------------------------------------------------------------------------------
//! @file WIMD_SAP_GlobalLink24.h
//! @ingroup
//! <!------------------------------------------------------------------------->
//! @brief
//! @version 0.1
//! <!------------------------------------------------------------------------->
//!
//!
//!
//! <!--------------------------------------------------------------------------
//! Copyright (c) 2020
//! IMST GmbH
//! Carl-Friedrich Gauss Str. 2-4
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


#ifndef ARDUINO_SAP_WIMOD_SAP_GLOBALLINK24_H_
#define ARDUINO_SAP_WIMOD_SAP_GLOBALLINK24_H_

//------------------------------------------------------------------------------
//
// Section Includes Files
//
//------------------------------------------------------------------------------

#include "WiMOD_SAP_LORAWAN.h"


//------------------------------------------------------------------------------
// since the GlobalLink24 Stack emulates LoRaWAN most things are identical
//------------------------------------------------------------------------------
//! @cond Doxygen_Suppress

//#define TGlobalLink24Status                  TLoRaWANStatus
//#define TGlobalLink24JoinNwkIndStatusFormat  TLoRaWanJoinNwkIndStatusFormat
//#define TGlobalLink24TxDataIndStatusFormat   TLoRaWanTxDataIndStatusFormat
////#define TGlobalLink24_FreqBand               TLoRaWAN_FreqBand
//#define TGlobalLink24_PowerSaving            TLoRaWAN_PowerSaving
//#define TGlobalLink24_NwkStatus              TLoRaWAN_NwkStatus
//
//#define GlobalLink24_MAC_DATA_SERVICE_TYPE_U_DATA    LORAWAN_MAC_DATA_SERVICE_TYPE_U_DATA
//#define GlobalLink24_MAC_DATA_SERVICE_TYPE_C_DATA    LORAWAN_MAC_DATA_SERVICE_TYPE_C_DATA
//
//#define TWiMODGlobalLink24_ActivateDeviceData     TWiMODLORAWAN_ActivateDeviceData
//#define TWiMODGlobalLink24_JoinParams             TWiMODLORAWAN_JoinParams
//#define TWiMODGlobalLink24_OptIndInfos            TWiMODLORAWAN_OptIndInfos
//#define TWiMODGlobalLink24_TxIndData              TWiMODLORAWAN_TxIndData
//#define TWiMODGlobalLink24_TX_Data                TWiMODLORAWAN_TX_Data
//#define TWiMODGlobalLink24_RX_Data                TWiMODLORAWAN_RX_Data
//#define TWiMODGlobalLink24_RX_MacCmdData          TWiMODLORAWAN_RX_MacCmdData
//#define TWiMODGlobalLink24_RX_JoinedNwkData       TWiMODLORAWAN_RX_JoinedNwkData
////#define TWiMODGlobalLink24_RadioStackConfig       TWiMODLORAWAN_RadioStackConfig
//#define TWiMODGlobalLink24_MacCmd                 TWiMODLORAWAN_MacCmd
//#define TWiMODGlobalLink24_RX_ACK_Data            TWiMODLORAWAN_RX_ACK_Data
//#define TWiMODGlobalLink24_NwkStatus_Data         TWiMODLORAWAN_NwkStatus_Data
//#define TWiMODGlobalLink24_NoData_Data            TWiMODLORAWAN_NoData_Data
//#define TWiMODGlobalLink24_SupportedBands         TWiMODLORAWAN_SupportedBands
//#define TWiMODGlobalLink24_TxPwrLimitConfig       TWiMODLORAWAN_TxPwrLimitConfig
//#define TWiMODGlobalLink24_LinkAdrReqConfig       TWiMODLORAWAN_LinkAdrReqConfig
//! @endcond


#define TGlobalLink24Status                 TLoRaWANStatus
#define TGlobalLink24JoinNwkIndStatusFormat TLoRaWanJoinNwkIndStatusFormat
#define TGlobalLink24TxDataIndStatusFormat  TLoRaWanTxDataIndStatusFormat
#define TGlobalLink24_FreqBand              TLoRaWAN_FreqBand
#define TGlobalLink24_PowerSaving           TLoRaWAN_PowerSaving
#define TGlobalLink24_NwkStatus             TLoRaWAN_NwkStatus

#define GlobalLink24_MAC_DATA_SERVICE_TYPE_U_DATA    LORAWAN_MAC_DATA_SERVICE_TYPE_U_DATA
#define GlobalLink24_MAC_DATA_SERVICE_TYPE_C_DATA    LORAWAN_MAC_DATA_SERVICE_TYPE_C_DATA

#define TWiMODGlobalLink24_OptIndInfos          TWiMODLORAWAN_OptIndInfos
#define TWiMODGlobalLink24_LinkAdrReqConfig     TWiMODLORAWAN_LinkAdrReqConfig

struct TWiMODGlobalLink24_ActivateDeviceData    : TWiMODLORAWAN_ActivateDeviceData   { };
struct TWiMODGlobalLink24_JoinParams            : TWiMODLORAWAN_JoinParams           { };
struct TWiMODGlobalLink24_TxIndData             : TWiMODLORAWAN_TxIndData            { };
struct TWiMODGlobalLink24_TX_Data               : TWiMODLORAWAN_TX_Data              { };
struct TWiMODGlobalLink24_RX_Data               : TWiMODLORAWAN_RX_Data              { };
struct TWiMODGlobalLink24_RX_MacCmdData         : TWiMODLORAWAN_RX_MacCmdData        { };
struct TWiMODGlobalLink24_RX_JoinedNwkData      : TWiMODLORAWAN_RX_JoinedNwkData     { };
struct TWiMODGlobalLink24_RadioStackConfig      : TWiMODLORAWAN_RadioStackConfig     { };
struct TWiMODGlobalLink24_MacCmd                : TWiMODLORAWAN_MacCmd               { };
struct TWiMODGlobalLink24_RX_ACK_Data           : TWiMODLORAWAN_RX_ACK_Data          { };
struct TWiMODGlobalLink24_NwkStatus_Data        : TWiMODLORAWAN_NwkStatus_Data       { };
struct TWiMODGlobalLink24_NoData_Data           : TWiMODLORAWAN_NoData_Data          { };
struct TWiMODGlobalLink24_SupportedBands        : TWiMODLORAWAN_SupportedBands       { };


//! @cond Doxygen_Suppress

// Status Codes

#define GLOBALLINK24_STATUS_OK                       LORAWAN_STATUS_OK                        //Operation successful
#define GLOBALLINK24_STATUS_ERROR                    LORAWAN_STATUS_ERROR                     //Operation failed
#define GLOBALLINK24_STATUS_CMD_NOT_SUPPORTED        LORAWAN_STATUS_CMD_NOT_SUPPORTED         //Command is not supported
#define GLOBALLINK24_STATUS_WRONG_PARAMETER          LORAWAN_STATUS_WRONG_PARAMETER           //HCI message contains wrong parameter
#define GLOBALLINK24_STATUS_WRONG_DEVICE_MODE        LORAWAN_STATUS_WRONG_DEVICE_MODE         //Stack is running in a wrong mode
#define GLOBALLINK24_STATUS_DEVICE_NOT_ACTIVATED     LORAWAN_STATUS_DEVICE_NOT_ACTIVATED      //Device is not activated
#define GLOBALLINK24_STATUS_DEVICE_BUSY              LORAWAN_STATUS_DEVICE_BUSY               //Device is busy, command rejected
#define GLOBALLINK24_STATUS_QUEUE_FULL               LORAWAN_STATUS_QUEUE_FULL                //Message queue is full, command rejected
#define GLOBALLINK24_STATUS_LENGTH_ERROR             LORAWAN_STATUS_LENGTH_ERROR              //HCI message length is invalid or radio payload size is too large
#define GLOBALLINK24_STATUS_NO_FACTORY_SETTINGS      LORAWAN_STATUS_NO_FACTORY_SETTINGS       //Factory Settings EEPROM block missing or incompatible with current firmware version
#define GLOBALLINK24_STATUS_CHANNEL_BLOCKED          LORAWAN_STATUS_CHANNEL_BLOCKED           //Channel blocked by Duty Cycle
#define GLOBALLINK24_STATUS_CHANNEL_NOT_AVAILABLE    LORAWAN_STATUS_CHANNEL_NOT_AVAILABLE     //No channel available (e.g. no channel defined for the configured spreading factor)



//TxPowerLimitConfig ist nur in EU868 verfügbar
//SupportedBands ist auch in GlobalLink24 verfügbar
    //MAC Commands von EU868 sind auch in GlobalLink24 gültig
    //TxParamSetupReq MAC Command ist in GlobalLink24 gültig (in EU868 aber nicht)





#endif /* ARDUINO_SAP_WIMOD_SAP_GLOBALLINK24_H_ */
