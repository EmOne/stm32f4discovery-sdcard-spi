//------------------------------------------------------------------------------
//! @file WiMODLoRaWAN.h
//! @ingroup WiMODLoRaWAN
//! <!------------------------------------------------------------------------->
//! @brief Declarations for the High-level Interface for WiMOD LoRaWAN EndNode Modem firmware
//! @version 0.1
//! <!------------------------------------------------------------------------->
//!
//!
//!
//! <!--------------------------------------------------------------------------
//! Copyright (c) 2016
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


#ifndef ARDUINO_WIMODLORAWAN_H_
#define ARDUINO_WIMODLORAWAN_H_



/**
 * THIS IS AN EXAMPLE IMPLEMENTATION ACCORDING THE THE HCI SPEC: V1.26
 * FOR FIRMWARE: LoRaWAN
 */


//------------------------------------------------------------------------------
//
// Section Includes Files
//
//------------------------------------------------------------------------------

//#include <cstddef>
#include <string.h>

#include "WiMOD_SAP_LORAWAN.h"
#include "WiMOD_SAP_DEVMGMT.h"
#include "WiMOD_SAP_Generic.h"
#include "ComSLIP.h"
#include "WiMODLRHCI.h"

//-----------------------------------------------------------------------------
// common defines
//-----------------------------------------------------------------------------
//! @cond Doxygen_Suppress
#define WIMOD_LORAWAN_SERIAL_BAUDRATE               115200

#define WiMOD_LORAWAN_TX_BUFFER_SIZE                256
//! @endcond
//-----------------------------------------------------------------------------
// types for callback functions
//-----------------------------------------------------------------------------



//-----------------------------------------------------------------------------
// API class declaration for the WiMOD LR BASE Stack
//
// note: this is NOT compliant with the WiMOD LoRaWAN Stack
//-----------------------------------------------------------------------------
/**
 * @brief Main class representing the interface to the WiMOD running the firmware LoRaWAN EndNode Modem
 *
 * This class is the only API class a user should use for interacting with
 * a WiMOD module that runs the IMST LoRaWAN EndNode Modem firmware.
 *
 */
typedef struct {
//public:
//    /*explicit*/ WiMODLoRaWAN(Stream& s);
//    ~WiMODLoRaWAN(void);

    void (*begin)(TLoRaWANregion region /*= LoRaWAN_Region_EU868(*/);
    void (*end)(void);

    //! @cond Doxygen_Suppress
    void (*beginAndAutoSetup)(void);
    void (*autoSetupSupportedRegion)(void);
    //! @endcond

    /*
     * DevMgmt SAP
     */
    bool (*Ping)(TWiMODLRResultCodes* hciResult, UINT8* rspStatus);
    bool (*Reset)(TWiMODLRResultCodes* hciResult, UINT8* rspStatus);
    bool (*GetDeviceInfo)(TWiMODLR_DevMgmt_DevInfo* info, TWiMODLRResultCodes* hciResult, UINT8* rspStatus);
    bool (*GetFirmwareInfo)(TWiMODLR_DevMgmt_FwInfo* info, TWiMODLRResultCodes* hciResult, UINT8* rspStatus);
    bool (*GetSystemStatus)(TWiMODLR_DevMgmt_SystemStatus* info, TWiMODLRResultCodes* hciResult, UINT8* rspStatus);
    bool (*GetRtc)(UINT32* rtcTime,TWiMODLRResultCodes* hciResult, UINT8* rspStatus);
    bool (*SetRtc)(const UINT32 rtcTime, TWiMODLRResultCodes* hciResult, UINT8* rspStatus);

    bool (*GetOperationMode)(TWiMOD_OperationMode* opMode, TWiMODLRResultCodes* hciResult, UINT8* rspStatus);
    bool (*SetOperationMode)(const TWiMOD_OperationMode opMode, TWiMODLRResultCodes* hciResult, UINT8* rspStatus);

    bool (*SetRtcAlarm)(const TWiMODLR_DevMgmt_RtcAlarm* rtcAlarm, TWiMODLRResultCodes* hciResult, UINT8* rspStatus);
    bool (*GetRtcAlarm)(TWiMODLR_DevMgmt_RtcAlarm* rtcAlarm, TWiMODLRResultCodes* hciResult, UINT8* rspStatus);
    bool (*ClearRtcAlarm)(TWiMODLRResultCodes* hciResult, UINT8* rspStatus);

    void (*RegisterPowerUpIndicationClient)(TDevMgmtPowerUpCallback cb);
    void (*RegisterRtcAlarmIndicationClient)(TDevMgmtRtcAlarmCallback cb);

    bool (*GetHciConfig)(TWiMODLR_DevMgmt_HciConfig* hciConfig, TWiMODLRResultCodes* hciResult, UINT8* rspStatus);
    bool (*SetHciConfig)(const TWiMODLR_DevMgmt_HciConfig* hciConfig, TWiMODLRResultCodes* hciResult, UINT8* rspStatus);


    /*
     * LoRaWAN SAP
     */
    bool (*ActivateDevice)(TWiMODLORAWAN_ActivateDeviceData* activationData,TWiMODLRResultCodes* hciResult, UINT8* rspStatus);
    bool (*ReactivateDevice)(UINT32* devAdr, TWiMODLRResultCodes* hciResult, UINT8* rspStatus);
    bool (*SetJoinParameter)(TWiMODLORAWAN_JoinParams* joinParams, TWiMODLRResultCodes* hciResult, UINT8* rspStatus);
    bool (*JoinNetwork)(TWiMODLRResultCodes* hciResult, UINT8* rspStatus);

    void (*RegisterJoinTxIndicationClient)(TJoinTxIndicationCallback cb);

    bool (*convert1)(TWiMODLR_HCIMessage* RxMsg, TWiMODLORAWAN_RX_Data* loraWanRxData);
    bool (*convert2)(TWiMODLR_HCIMessage* rxMsg, TWiMODLORAWAN_TxIndData* sendIndData);
    bool (*convert3)(TWiMODLR_HCIMessage* RxMsg, TWiMODLORAWAN_RX_MacCmdData* loraWanMacCmdData);
    bool (*convert4)(TWiMODLR_HCIMessage* RxMsg,TWiMODLORAWAN_RX_JoinedNwkData* joinedNwkData);
    bool (*convert5)(TWiMODLR_HCIMessage* RxMsg, TWiMODLORAWAN_RX_ACK_Data* ackData);
    bool (*convert6)(TWiMODLR_HCIMessage* RxMsg, TWiMODLORAWAN_NoData_Data* info);

    void (*RegisterNoDataIndicationClient)(TNoDataIndicationCallback cb);
    void (*RegisterTxCDataIndicationClient)(TTxCDataIndicationCallback cb);
    void (*RegisterTxUDataIndicationClient)(TTxUDataIndicationCallback cb);
    void (*RegisterRxUDataIndicationClient)(TRxUDataIndicationCallback cb);
    void (*RegisterRxCDataIndicationClient)(TRxCDataIndicationCallback cb);
    void (*RegisterRxMacCmdIndicationClient)(TRxMacCmdIndicationCallback cb);
    void (*RegisterJoinedNwkIndicationClient)(TJoinedNwkIndicationCallback cb);
    void (*RegisterRxAckIndicationClient)(TRxAckIndicationCallback cb);

    bool (*SendUData)(const TWiMODLORAWAN_TX_Data* data, TWiMODLRResultCodes* hciResult, UINT8* rspStatus);
    bool (*SendCData)(const TWiMODLORAWAN_TX_Data* data,TWiMODLRResultCodes* hciResult, UINT8* rspStatus);
    bool (*SetRadioStackConfig)(TWiMODLORAWAN_RadioStackConfig* data,TWiMODLRResultCodes* hciResult, UINT8* rspStatus);
    bool (*GetRadioStackConfig)(TWiMODLORAWAN_RadioStackConfig* data, TWiMODLRResultCodes* hciResult, UINT8* rspStatus);
    bool (*DeactivateDevice)(TWiMODLRResultCodes* hciResult, UINT8* rspStatus);
    bool (*FactoryReset)(TWiMODLRResultCodes* hciResult, UINT8* rspStatus);
    bool (*SetDeviceEUI)(const UINT8* deviceEUI, TWiMODLRResultCodes* hciResult, UINT8* rspStatus);
    bool (*GetDeviceEUI)(UINT8* deviceEUI, TWiMODLRResultCodes* hciResult, UINT8* rspStatus);
//    bool GetNwkStatus(UINT8* nwkStatus, TWiMODLRResultCodes* hciResult = NULL, UINT8* rspStatus = NULL); // implementation for spec up to  V1.13
    bool (*GetNwkStatus)(TWiMODLORAWAN_NwkStatus_Data*	nwkStatus, TWiMODLRResultCodes* hciResult, UINT8* rspStatus); // new implementation for spec. V1.14
    bool (*SendMacCmd)(const TWiMODLORAWAN_MacCmd* cmd, TWiMODLRResultCodes* hciResult, UINT8* rspStatus);
    bool (*SetCustomConfig)(const INT8 rfGain, TWiMODLRResultCodes* hciResult, UINT8* rspStatus);
    bool (*GetCustomConfig)(INT8* rfGain, TWiMODLRResultCodes* hciResult, UINT8* rspStatus);
    bool (*GetSupportedBands)(TWiMODLORAWAN_SupportedBands* supportedBands, TWiMODLRResultCodes*  hciResult, UINT8* rspStatus);
    bool (*GetTxPowerLimitConfig)(TWiMODLORAWAN_TxPwrLimitConfig* txPwrLimitCfg, TWiMODLRResultCodes*  hciResult, UINT8* rspStatus);
    bool (*SetTxPowerLimitConfig)(TWiMODLORAWAN_TxPwrLimitConfig* txPwrLimitCfg, TWiMODLRResultCodes*  hciResult, UINT8* rspStatus);
    bool (*GetLinkAdrReqConfig)(TWiMODLORAWAN_LinkAdrReqConfig* linkAdrReqCfg, TWiMODLRResultCodes*  hciResult, UINT8* rspStatus);
    bool (*SetLinkAdrReqConfig)(TWiMODLORAWAN_LinkAdrReqConfig* linkAdrReqCfg, TWiMODLRResultCodes*  hciResult, UINT8* rspStatus);
    bool (*SetBatteryLevelStatus)(UINT8 battStatus, TWiMODLRResultCodes*  hciResult, UINT8* rspStatus);

    /*
     * Generic Cmd
     */
    bool (*ExecuteGenericCmd)(TWiMODLR_Generic_CmdInfo* info, TWiMODLRResultCodes* hciResult, UINT8* rspStatus);

    /*
     * Info & QuickStart Cmds
     */
    void (*PrintBasicDeviceInfo)(Stream* s);
    void (*ConnectViaOTAA)(const uint8_t* appEUI, const uint8_t* appKey);
    void (*ConvertAppEuiStrToArray)(char* appEuiStr, uint8_t* appEuiArray);
    void (*ConvertAppKeyStrToArray)(char* appKeyStr, uint8_t* appKeyArray);

    void (*ConvertNwkSKeyStrToArray)(char* nwkSKeyStr, uint8_t* nwkSKeyArray);
    void (*ConvertAppSKeyStrToArray)(char* appSKeyStr, uint8_t* appSKeyArray);

    TWiMODLRResultCodes  (*GetLastHciResult)(void);
    UINT8 (*GetLastResponseStatus)(void);
    void (*Process)(TWiMODLR_HCIMessage* rxMsg);
//protected:
    WiMOD_SAP_DevMgmt_t*   SapDevMgmt;                                           /*!< Service Access Point for 'DeviceManagement' */
    WiMOD_SAP_LoRaWAN_t*   SapLoRaWan;                                           /*!< Service Access Point for 'LoRaWAN' */
    WiMOD_SAP_Generic_t*   SapGeneric;											/*!< dummy SAP for generic HCI command */


    void (*ProcessUnexpectedRxMessage)(TWiMODLR_HCIMessage* rxMsg);


    bool (*copyLoRaWanResultInfos)(TWiMODLRResultCodes* hciResult, UINT8* rspStatus);
    bool (*copyDevMgmtResultInfos)(TWiMODLRResultCodes* hciResult, UINT8* rspStatus);
//private:
    //! @cond Doxygen_Suppress
    UINT8               txBuffer[WiMOD_LORAWAN_TX_BUFFER_SIZE];

    UINT8               localStatusRsp;
    bool                cmdResult;
    TWiMODLRResultCodes  localHciRes;

    TWiMODLRResultCodes  lastHciRes;
    UINT8               lastStatusRsp;

    //! @endcond
}WiMODLoRaWAN_t;

extern WiMODLoRaWAN_t WiMODLoRaWAN;

#endif /* ARDUINO_WIMODLORAWAN_H_ */
