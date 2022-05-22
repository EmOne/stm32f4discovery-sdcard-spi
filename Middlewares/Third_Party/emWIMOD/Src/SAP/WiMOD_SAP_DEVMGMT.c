//------------------------------------------------------------------------------
//! @file WiMOD_SAP_DEVMGMT.cpp
//! @ingroup WiMOD_SAP_DEVMGMT
//! <!------------------------------------------------------------------------->
//! @brief Implementation of the commands of the DeviceManagement SericeAccessPoint
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

/*
 * THIS IS AN EXAMPLE IMPLEMENTATION ACCORDING THE THE HCI SPEC: V1.8
 * FOR FIRMWARE: LR-BASE
 */

//------------------------------------------------------------------------------
//
// Section Includes Files
//
//------------------------------------------------------------------------------

#include "WiMOD_SAP_DEVMGMT.h"
#include <string.h>

//------------------------------------------------------------------------------
//
// Section public functions
//
//------------------------------------------------------------------------------
static TWiMODLRResultCodes ping(UINT8* statusRsp);
static TWiMODLRResultCodes reset(UINT8* statusRsp);
static TWiMODLRResultCodes getDeviceInfo(TWiMODLR_DevMgmt_DevInfo* info, UINT8* statusRsp);
static TWiMODLRResultCodes getFirmwareInfo(TWiMODLR_DevMgmt_FwInfo* info, UINT8* statusRsp);
static TWiMODLRResultCodes getSystemStatus(TWiMODLR_DevMgmt_SystemStatus* info, UINT8* statusRsp);
static TWiMODLRResultCodes getRtc(UINT32* rtcTime, UINT8* statusRsp);
static TWiMODLRResultCodes setRtc(const UINT32 rtcTime, UINT8* statusRsp);
static TWiMODLRResultCodes getSystemStatus(TWiMODLR_DevMgmt_SystemStatus* info, UINT8* statusRsp);
static TWiMODLRResultCodes getRtc(UINT32* rtcTime, UINT8* statusRsp);
static TWiMODLRResultCodes setRtc(const UINT32 rtcTime, UINT8* statusRsp);
static TWiMODLRResultCodes getRadioConfig(TWiMODLR_DevMgmt_RadioConfig* radioCfg, UINT8* statusRsp);
static TWiMODLRResultCodes setRadioConfig(const TWiMODLR_DevMgmt_RadioConfig* radioCfg, UINT8* statusRsp);
static TWiMODLRResultCodes resetRadioConfig(UINT8* statusRsp);
static TWiMODLRResultCodes getOperationMode(TWiMOD_OperationMode* opMode, UINT8* statusRsp);
static TWiMODLRResultCodes setOperationMode(const TWiMOD_OperationMode opMode, UINT8* statusRsp);
static TWiMODLRResultCodes setRadioMode(const TRadioCfg_RadioMode radioMode, UINT8* statusRsp);
static TWiMODLRResultCodes setAesKey(const UINT8* key, UINT8* statusRsp);
static TWiMODLRResultCodes getAesKey(UINT8* key, UINT8* statusRsp);
static TWiMODLRResultCodes setRtcAlarm(const TWiMODLR_DevMgmt_RtcAlarm* rtcAlarm, UINT8* statusRsp);
static TWiMODLRResultCodes getRtcAlarm(TWiMODLR_DevMgmt_RtcAlarm* rtcAlarm, UINT8* statusRsp);
static TWiMODLRResultCodes clearRtcAlarm(UINT8* statusRsp);
static TWiMODLRResultCodes setHCIConfig(const TWiMODLR_DevMgmt_HciConfig* hciConfig, UINT8* statusRsp);
static TWiMODLRResultCodes getHCIConfig(TWiMODLR_DevMgmt_HciConfig* hciConfig, UINT8* statusRsp);
static void registerPowerUpIndicationClient(TDevMgmtPowerUpCallback cb);
static void WregisterRtcAlarmIndicationClient(TDevMgmtRtcAlarmCallback cb);
static void dispatchDeviceMgmtMessage(TWiMODLR_HCIMessage* rxMsg);
static void process (UINT8* statusRsp, TWiMODLR_HCIMessage* rxMsg);
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

TWiMODLR_DevMgmt_DevInfo DeviceInfo = {
		WiMODLR_RESULT_OK,
		WIMOD_MODULE_TYPE_IM880B,
		WIMOD_DEV_ADDR,
		0x00,
		0x01,
};

TWiMODLR_DevMgmt_FwInfo firmwareInfo = {
		WiMODLR_RESULT_OK,
		0x00,
		0x01,
		0x0000,
		"14 Jan 22",
		"WIMOD Module by EmOne",
};

TWiMODLR_DevMgmt_SystemStatus SystemInfo = {
		WiMODLR_RESULT_OK,
		HAL_TICK_FREQ_DEFAULT,
		HAL_GetTick,
		0x00000000,
	    0x0000,
	    0x0000,
		0x0000,
		0x00000000,
		0x00000000,
		0x00000000,
		0x00000000,
		0x00000000,
		0x00000000,
};

TWiMOD_OperationMode OperationMode = OperationMode_Application;
uint32_t RTCTime;

TWiMODLR_DevMgmt_RtcAlarm RTCAlarm = {
		RTC_Alarm_No_Alarm_Set,
		0x08,
		0x05,
		0x00,
};

WiMOD_SAP_DevMgmt_t WiMOD_SAP_DevMgmt = {
	ping,
	reset,
	getDeviceInfo,
	getFirmwareInfo,
	getSystemStatus,
	getRtc,
	setRtc,
	getRadioConfig,
	setRadioConfig,
	resetRadioConfig,
	getOperationMode,
	setOperationMode,
	setRadioMode,
	setAesKey,
	getAesKey,
	setRtcAlarm,
	getRtcAlarm,
	clearRtcAlarm,
	setHCIConfig,
	getHCIConfig,
	registerPowerUpIndicationClient,
	WregisterRtcAlarmIndicationClient,
	dispatchDeviceMgmtMessage,
	process,
};

//-----------------------------------------------------------------------------
/**
 * @brief Destructor
 */
//WiMOD_SAP_DevMgmt::~WiMOD_SAP_DevMgmt(void)
//{
//
//}

//-----------------------------------------------------------------------------
/**
 * @brief Ping Cmd - Checks if the serial connection of to the WiMOD module is OK
 *
 *
 * @param   statusRsp   pointer to store status byte of response mesg from WiMOD
 *
 * @retval WiMODLR_RESULT_OK     if command transmit to WiMOD was ok
 */
TWiMODLRResultCodes ping(UINT8* statusRsp)
{
    TWiMODLRResultCodes result = WiMODLR_RESULT_TRANMIT_ERROR;

    if (statusRsp) {

            TWiMODLR_HCIMessage* tx = &WiMOD_SAP_DevMgmt.HciParser->TxMessage;

            // put data to tx
            tx->Payload[0] = 0x55;

            *statusRsp = WiMOD_SAP_DevMgmt.HciParser->PostMessage(DEVMGMT_SAP_ID,DEVMGMT_MSG_PING_RSP, &tx->Payload[WiMODLR_HCI_RSP_STATUS_POS], 1);
            result = WiMODLR_RESULT_OK;
    }
    return result;
}



//-----------------------------------------------------------------------------
/**
 * @brief Reset Cmd - Do a reset / reboot of the WiMOD
 *
 *
 * @param   statusRsp   pointer to store status byte of response mesg from WiMOD
 *
 * @retval WiMODLR_RESULT_OK     if command transmit to WiMOD was ok
 */
TWiMODLRResultCodes reset(UINT8* statusRsp)
{
    TWiMODLRResultCodes result = WiMODLR_RESULT_TRANMIT_ERROR;

	if (statusRsp) {

			TWiMODLR_HCIMessage* tx = &WiMOD_SAP_DevMgmt.HciParser->TxMessage;

			// put data to tx
			tx->Payload[0] = 0x55;

			*statusRsp = WiMOD_SAP_DevMgmt.HciParser->PostMessage(DEVMGMT_SAP_ID, DEVMGMT_MSG_RESET_RSP, &tx->Payload[WiMODLR_HCI_RSP_STATUS_POS], 1);
			result = WiMODLR_RESULT_OK;
	}
    return result;
}

//-----------------------------------------------------------------------------
/**
 * @brief Get Device Info - Basic Information about the WiMOD module
 *
 *
 * @param   info        pointer to store the received information
 * @param   statusRsp   pointer to store status byte of response mesg from WiMOD
 *
 * @retval WiMODLR_RESULT_OK     if command transmit to WiMOD was ok
 */
TWiMODLRResultCodes getDeviceInfo (TWiMODLR_DevMgmt_DevInfo* info, UINT8* statusRsp)
{
    TWiMODLRResultCodes result = WiMODLR_RESULT_TRANMIT_ERROR;
    UINT8              offset = WiMODLR_HCI_RSP_CMD_PAYLOAD_POS;

    if (statusRsp) {

            TWiMODLR_HCIMessage* tx = &WiMOD_SAP_DevMgmt.HciParser->TxMessage;

            // put data to tx
            tx->Payload[WiMODLR_HCI_RSP_STATUS_POS] = info->Status;
            tx->Payload[offset++]					= info->ModuleType;
            tx->Payload[offset]					    = info->DevAdr;
            offset += 0x02;
            tx->Payload[offset++]					= info->GroupAdr;
            // reserved field
            offset++;
            tx->Payload[offset]					    = info->DevID;
            offset += 0x04;

            *statusRsp = WiMOD_SAP_DevMgmt.HciParser->PostMessage(DEVMGMT_SAP_ID,DEVMGMT_MSG_GET_DEVICEINFO_RSP, &tx->Payload[WiMODLR_HCI_RSP_STATUS_POS], sizeof(TWiMODLR_DevMgmt_DevInfo));
            result = WiMODLR_RESULT_OK;

    }
    return result;
}

//-----------------------------------------------------------------------------
/**
 * @brief GetFirmwareInfo Cmd - Get basic info about the running firmware
 *
 *
 * @param   info        pointer to store the received information
 * @param   statusRsp   pointer to store status byte of response mesg from WiMOD
 *
 * @retval WiMODLR_RESULT_OK     if command transmit to WiMOD was ok
 */
TWiMODLRResultCodes getFirmwareInfo(TWiMODLR_DevMgmt_FwInfo* info, UINT8* statusRsp)
{
    TWiMODLRResultCodes result = WiMODLR_RESULT_TRANMIT_ERROR;
    UINT8              offset = WiMODLR_HCI_RSP_STATUS_POS;

	if (statusRsp) {

			TWiMODLR_HCIMessage* tx = &WiMOD_SAP_DevMgmt.HciParser->TxMessage;

			// put data to tx
            tx->Payload[offset++] 	= info->Status;
            tx->Payload[offset++]	= info->FirmwareMinorVersion;
            tx->Payload[offset++]	= info->FirmwareMayorVersion;
            tx->Payload[offset]		= info->BuildCount;
            offset+= 0x02;

            strncpy((char*) &tx->Payload[offset], (char*)info->BuildDateStr, sizeof(info->BuildDateStr));
            offset += WIMOD_DEVMGMT_BUILDDATE_LEN;

            strncpy((const char*) &tx->Payload[offset], (char*)info->FirmwareName, sizeof(info->FirmwareName));


			*statusRsp = WiMOD_SAP_DevMgmt.HciParser->PostMessage(DEVMGMT_SAP_ID, DEVMGMT_MSG_GET_FW_VERSION_RSP, &tx->Payload[WiMODLR_HCI_RSP_STATUS_POS], sizeof(TWiMODLR_DevMgmt_FwInfo));
			result = WiMODLR_RESULT_OK;
	}
    return result;
}

//-----------------------------------------------------------------------------
/**
 * @brief GetSystemStatus Cmd - Get basic info about the system status of WiMOD
 *
 *
 * @param   info        pointer to store the received information
 * @param   statusRsp   pointer to store status byte of response mesg from WiMOD
 *
 * @retval WiMODLR_RESULT_OK     if command transmit to WiMOD was ok
 */
TWiMODLRResultCodes getSystemStatus(TWiMODLR_DevMgmt_SystemStatus* info, UINT8* statusRsp)
{
    TWiMODLRResultCodes result = WiMODLR_RESULT_TRANMIT_ERROR;
    UINT8              offset = WiMODLR_HCI_RSP_STATUS_POS;

    if (statusRsp) {

        	TWiMODLR_HCIMessage* tx = &WiMOD_SAP_DevMgmt.HciParser->TxMessage;

            tx->Payload[offset++]	= info->Status;
            tx->Payload[offset++]	= info->SysTickResolution;
            tx->Payload[offset]		= info->SysTickCounter;
            offset += 0x04;
            tx->Payload[offset]		= info->RtcTime;
            offset += 0x04;
            tx->Payload[offset]		= info->NvmStatus;
            offset += 0x02;
            tx->Payload[offset]		= info->BatteryStatus;
            offset += 0x02;
            tx->Payload[offset]		= info->ExtraStatus;
            offset += 0x02;
            tx->Payload[offset]		= info->RxPackets;
            offset += 0x04;
            tx->Payload[offset]		= info->RxAddressMatch;
            offset += 0x04;
            tx->Payload[offset]		= info->RxCRCError;
            offset += 0x04;
            tx->Payload[offset]		= info->TxPackets;
            offset += 0x04;
            tx->Payload[offset]		= info->TxError;
            offset += 0x04;
            tx->Payload[offset]		= info->TxMediaBusyEvents;
            offset += 0x04;

			*statusRsp = WiMOD_SAP_DevMgmt.HciParser->PostMessage(DEVMGMT_SAP_ID, DEVMGMT_MSG_GET_SYSTEM_STATUS_RSP, &tx->Payload[WiMODLR_HCI_RSP_STATUS_POS], sizeof(TWiMODLR_DevMgmt_SystemStatus));
			result = WiMODLR_RESULT_OK;
    }
    return result;

}

//-----------------------------------------------------------------------------
/**
 * @brief GetRtc Cmd - Get the current RTC time/date from WiMOD
 *
 *
 * @param   rtcTime     pointer to store the received information
 * @param   statusRsp   pointer to store status byte of response mesg from WiMOD
 *
 * @retval WiMODLR_RESULT_OK     if command transmit to WiMOD was ok
 */
TWiMODLRResultCodes getRtc(UINT32* rtcTime, UINT8* statusRsp)
{
    TWiMODLRResultCodes result = WiMODLR_RESULT_TRANMIT_ERROR;
    UINT8              offset = WiMODLR_HCI_RSP_STATUS_POS;

    if (statusRsp) {

    	TWiMODLR_HCIMessage* tx = &WiMOD_SAP_DevMgmt.HciParser->TxMessage;
    	tx->Payload[offset++] = RTCAlarm.AlarmStatus;
    	*((UINT32 *) &tx->Payload[offset])	= *rtcTime;
        offset += 0x04;
		*statusRsp = WiMOD_SAP_DevMgmt.HciParser->PostMessage(DEVMGMT_SAP_ID, DEVMGMT_MSG_GET_RTC_RSP, &tx->Payload[WiMODLR_HCI_RSP_STATUS_POS], 5);
		result = WiMODLR_RESULT_OK;

    }
    return result;
}

//-----------------------------------------------------------------------------
/**
 * @brief SetRtc Cmd - Set the current RTC time/date of WiMOD
 *
 *
 * @param   rtcTime     32bit data containing the new RTC timestamp
 * @param   statusRsp   pointer to store status byte of response mesg from WiMOD
 *
 * @retval WiMODLR_RESULT_OK     if command transmit to WiMOD was ok
 */
TWiMODLRResultCodes setRtc(const UINT32 rtcTime, UINT8* statusRsp)
{
    TWiMODLRResultCodes result = WiMODLR_RESULT_TRANMIT_ERROR;


    if (statusRsp && (WiMOD_SAP_DevMgmt.HciParser->Rx.Message.Length >= 4)) {

    	TWiMODLR_HCIMessage* tx = &WiMOD_SAP_DevMgmt.HciParser->TxMessage;
        RTCTime = rtcTime;
        *statusRsp = WiMOD_SAP_DevMgmt.HciParser->PostMessage(DEVMGMT_SAP_ID, DEVMGMT_MSG_SET_RTC_RSP, &tx->Payload[WiMODLR_HCI_RSP_STATUS_POS], 4);
        result = WiMODLR_RESULT_OK;
    }
    return result;
}

/**
 * @brief GetRadioConfig Cmd - Get the radio settings of the WiMOD
 *
 *
 * @param   radioCfg    pointer to store the received information
 * @param   statusRsp   pointer to store status byte of response mesg from WiMOD
 *
 * @retval WiMODLR_RESULT_OK     if command transmit to WiMOD was ok
 */
TWiMODLRResultCodes getRadioConfig(TWiMODLR_DevMgmt_RadioConfig* radioCfg, UINT8* statusRsp)
{
    TWiMODLRResultCodes result = WiMODLR_RESULT_TRANMIT_ERROR;
    UINT8              offset = WiMODLR_HCI_RSP_STATUS_POS;

    if (radioCfg && statusRsp) {

        result = WiMOD_SAP_DevMgmt.HciParser->SendHCIMessage(DEVMGMT_SAP_ID,
                                           DEVMGMT_MSG_GET_RADIO_CONFIG_REQ,
                                           DEVMGMT_MSG_GET_RADIO_CONFIG_RSP,
                                           NULL, 0);

        if (result == WiMODLR_RESULT_OK) {
            const TWiMODLR_HCIMessage* rx = WiMOD_SAP_DevMgmt.HciParser->GetRxMessage();
            *statusRsp = rx->Payload[offset++];

            // status check
            if (*statusRsp == DEVMGMT_STATUS_OK) {
                    radioCfg->Status = *statusRsp;
                    radioCfg->RadioMode = (TRadioCfg_RadioMode) rx->Payload[offset++];
                    radioCfg->GroupAddress = rx->Payload[offset++];
                    radioCfg->TxGroupAddress = rx->Payload[offset++];
                    radioCfg->DeviceAddress = NTOH16(&rx->Payload[offset]);
                    offset += 0x02;
                    radioCfg->TxDeviceAddress = NTOH16(&rx->Payload[offset]);
                    offset += 0x02;
                    radioCfg->Modulation = (TRadioCfg_Modulation) rx->Payload[offset++];
                    radioCfg->RfFreq_LSB = rx->Payload[offset++];
                    radioCfg->RfFreq_MID = rx->Payload[offset++];
                    radioCfg->RfFreq_MSB = rx->Payload[offset++];
                    radioCfg->LoRaBandWidth = (TRadioCfg_LoRaBandwidth) rx->Payload[offset++];
                    radioCfg->LoRaSpreadingFactor = (TRadioCfg_LoRaSpreadingFactor) rx->Payload[offset++];;
                    radioCfg->ErrorCoding = (TRadioCfg_ErrorCoding) rx->Payload[offset++];;
                    radioCfg->PowerLevel =  (TRadioCfg_PowerLevel)  rx->Payload[offset++];
                    radioCfg->TxControl = rx->Payload[offset++];
                    radioCfg->RxControl = (TRadioCfg_RxControl) rx->Payload[offset++];
                    radioCfg->RxWindowTime = NTOH16(&rx->Payload[offset]);
                    offset += 0x02;
                    radioCfg->LedControl = rx->Payload[offset++];
                    radioCfg->MiscOptions = rx->Payload[offset++];
                    radioCfg->FskDatarate = (TRadioCfg_FskDatarate) rx->Payload[offset++];
                    radioCfg->PowerSavingMode = (TRadioCfg_PowerSavingMode) rx->Payload[offset++];
                    radioCfg->LbtThreshold = (INT16) NTOH16(&rx->Payload[offset]);
                    offset += 0x02;
            }
        }
    }
    return result;
}

//-----------------------------------------------------------------------------
/**
 * @brief SetRadioConfig Cmd - Set the radio settings of the WiMOD
 *
 *
 * @param   radioCfg    pointer to the new radio configuration
 * @param   statusRsp   pointer to store status byte of response mesg from WiMOD
 *
 * @retval WiMODLR_RESULT_OK     if command transmit to WiMOD was ok
 */
TWiMODLRResultCodes setRadioConfig(const TWiMODLR_DevMgmt_RadioConfig* radioCfg, UINT8* statusRsp) {
    TWiMODLRResultCodes result = WiMODLR_RESULT_TRANMIT_ERROR;
    UINT8              offset = 0;

    if (radioCfg && statusRsp && (WiMOD_SAP_DevMgmt.txyPayloadSize >= 0x1A)) {
    	WiMOD_SAP_DevMgmt.txPayload[offset++] = (UINT8) radioCfg->StoreNwmFlag;
    	WiMOD_SAP_DevMgmt.txPayload[offset++] = (UINT8) radioCfg->RadioMode;
    	WiMOD_SAP_DevMgmt.txPayload[offset++] = (UINT8) radioCfg->GroupAddress;
    	WiMOD_SAP_DevMgmt.txPayload[offset++] = (UINT8) radioCfg->TxGroupAddress;
        HTON16(&WiMOD_SAP_DevMgmt.txPayload[offset], radioCfg->DeviceAddress);
        offset += 0x02;
        HTON16(&WiMOD_SAP_DevMgmt.txPayload[offset], radioCfg->TxDeviceAddress);
        offset += 0x02;
        WiMOD_SAP_DevMgmt.txPayload[offset++] = (UINT8) radioCfg->Modulation;
        WiMOD_SAP_DevMgmt.txPayload[offset++] = (UINT8) radioCfg->RfFreq_LSB;
        WiMOD_SAP_DevMgmt.txPayload[offset++] = (UINT8) radioCfg->RfFreq_MID;
        WiMOD_SAP_DevMgmt.txPayload[offset++] = (UINT8) radioCfg->RfFreq_MSB;
        WiMOD_SAP_DevMgmt.txPayload[offset++] = (UINT8) radioCfg->LoRaBandWidth;
        WiMOD_SAP_DevMgmt.txPayload[offset++] = (UINT8) radioCfg->LoRaSpreadingFactor;
        WiMOD_SAP_DevMgmt.txPayload[offset++] = (UINT8) radioCfg->ErrorCoding;
        WiMOD_SAP_DevMgmt.txPayload[offset++] = (UINT8) radioCfg->PowerLevel;
        WiMOD_SAP_DevMgmt.txPayload[offset++] = (UINT8) radioCfg->TxControl;
        WiMOD_SAP_DevMgmt.txPayload[offset++] = (UINT8) radioCfg->RxControl;
        HTON16(&WiMOD_SAP_DevMgmt.txPayload[offset], radioCfg->RxWindowTime);
        offset += 0x02;
        WiMOD_SAP_DevMgmt.txPayload[offset++] = (UINT8) radioCfg->LedControl;
        WiMOD_SAP_DevMgmt.txPayload[offset++] = (UINT8) radioCfg->MiscOptions;
        WiMOD_SAP_DevMgmt.txPayload[offset++] = (UINT8) radioCfg->FskDatarate;
        WiMOD_SAP_DevMgmt.txPayload[offset++] = (UINT8) radioCfg->PowerSavingMode;
        HTON16(&WiMOD_SAP_DevMgmt.txPayload[offset], (UINT16) radioCfg->LbtThreshold);
        offset += 0x02;

        result = WiMOD_SAP_DevMgmt.HciParser->SendHCIMessage(DEVMGMT_SAP_ID,
                DEVMGMT_MSG_SET_RADIO_CONFIG_REQ,
                DEVMGMT_MSG_SET_RADIO_CONFIG_RSP,
				WiMOD_SAP_DevMgmt.txPayload, offset);

        if (result == WiMODLR_RESULT_OK) {
            const TWiMODLR_HCIMessage* rx = WiMOD_SAP_DevMgmt.HciParser->GetRxMessage();
            offset = WiMODLR_HCI_RSP_STATUS_POS;
            *statusRsp = rx->Payload[offset++];

            // status check
            if (*statusRsp == DEVMGMT_STATUS_OK) {


            } else {
            }
        }
    }
    return result;
}

//-----------------------------------------------------------------------------
/**
 * @brief ResetRadioConfig Cmd - Reset the radio config to factory defaults.
 *
 *
 * @param   statusRsp   pointer to store status byte of response mesg from WiMOD
 *
 * @retval WiMODLR_RESULT_OK     if command transmit to WiMOD was ok
 */
TWiMODLRResultCodes resetRadioConfig(UINT8* statusRsp) {
    TWiMODLRResultCodes result = WiMODLR_RESULT_TRANMIT_ERROR;
    if (statusRsp) {
        result = WiMOD_SAP_DevMgmt.HciParser->SendHCIMessage(DEVMGMT_SAP_ID,
                DEVMGMT_MSG_RESET_RADIO_CONFIG_REQ,
                DEVMGMT_MSG_RESET_RADIO_CONFIG_RSP,
                NULL, 0);
        // copy response status
        if (WiMODLR_RESULT_OK == result) {
            *statusRsp = WiMOD_SAP_DevMgmt.HciParser->GetRxMessage()->Payload[WiMODLR_HCI_RSP_STATUS_POS];
        }
    }
    return result;
}


//-----------------------------------------------------------------------------
/**
 * @brief GetOperationMode Cmd - Get the current operation mode of the WiMOD
 *
 *
 * @param   opMode      pointer to store the received information
 * @param   statusRsp   pointer to store status byte of response mesg from WiMOD
 *
 * @retval WiMODLR_RESULT_OK     if command transmit to WiMOD was ok
 */
TWiMODLRResultCodes getOperationMode(TWiMOD_OperationMode* opMode, UINT8* statusRsp) {
    TWiMODLRResultCodes result = WiMODLR_RESULT_TRANMIT_ERROR;
//    if (statusRsp && opMode) {
//        result = WiMOD_SAP_DevMgmt.HciParser->SendHCIMessage(DEVMGMT_SAP_ID,
//                DEVMGMT_MSG_GET_OPMODE_REQ,
//                DEVMGMT_MSG_GET_OPMODE_RSP,
//                NULL, 0);
//        // copy response status
//        if (WiMODLR_RESULT_OK == result) {
//            *statusRsp = WiMOD_SAP_DevMgmt.HciParser->GetRxMessage()->Payload[WiMODLR_HCI_RSP_STATUS_POS];
//            *opMode = (TWiMOD_OperationMode) WiMOD_SAP_DevMgmt.HciParser->GetRxMessage()->Payload[WiMODLR_HCI_RSP_CMD_PAYLOAD_POS];
//        }
//    }
//    return result;

	if (statusRsp) {

			TWiMODLR_HCIMessage* tx = &WiMOD_SAP_DevMgmt.HciParser->TxMessage;

			// put data to tx
			tx->Payload[0] = *opMode;

			*statusRsp = WiMOD_SAP_DevMgmt.HciParser->PostMessage(DEVMGMT_SAP_ID, DEVMGMT_MSG_GET_OPMODE_RSP, &tx->Payload[WiMODLR_HCI_RSP_STATUS_POS], 1);
			result = WiMODLR_RESULT_OK;
	}
    return result;
}

//-----------------------------------------------------------------------------
/**
 * @brief SetOperationMode Cmd - Set the current operation mode of the WiMOD
 *
 *
 * @param   opMode      the new operation mode to set
 * @param   statusRsp   pointer to store status byte of response mesg from WiMOD
 *
 * @retval WiMODLR_RESULT_OK     if command transmit to WiMOD was ok
 */
TWiMODLRResultCodes setOperationMode(const TWiMOD_OperationMode opMode, UINT8* statusRsp) {
    TWiMODLRResultCodes result = WiMODLR_RESULT_TRANMIT_ERROR;
//    if (statusRsp && (WiMOD_SAP_DevMgmt.txyPayloadSize >= 1)) {
//    	WiMOD_SAP_DevMgmt.txPayload[0] = (UINT8) opMode;
//        result = WiMOD_SAP_DevMgmt.HciParser->SendHCIMessage(DEVMGMT_SAP_ID,
//                DEVMGMT_MSG_SET_OPMODE_REQ,
//                DEVMGMT_MSG_SET_OPMODE_RSP,
//				WiMOD_SAP_DevMgmt.txPayload, 1);
//        // copy response status
//        if (WiMODLR_RESULT_OK == result) {
//            *statusRsp = WiMOD_SAP_DevMgmt.HciParser->GetRxMessage()->Payload[WiMODLR_HCI_RSP_STATUS_POS];
//        }
//    }
//    return result;
	if (statusRsp) {

			TWiMODLR_HCIMessage* tx = &WiMOD_SAP_DevMgmt.HciParser->TxMessage;

			// put data to tx
			OperationMode = opMode;

			*statusRsp = WiMOD_SAP_DevMgmt.HciParser->PostMessage(DEVMGMT_SAP_ID, DEVMGMT_MSG_SET_OPMODE_RSP, &tx->Payload[WiMODLR_HCI_RSP_STATUS_POS], 1);
			result = WiMODLR_RESULT_OK;
	}
    return result;
}

//-----------------------------------------------------------------------------
/**
 * @brief SetRadioMode - Set the current radio mode of the WiMOD
 *
 *
 * @param   radioMode    the new radio mode to set
 * @param   statusRsp   pointer to store status byte of response mesg from WiMOD
 *
 * @warning: This use this command with care!
 *
 * @retval WiMODLR_RESULT_OK     if command transmit to WiMOD was ok
 */
TWiMODLRResultCodes setRadioMode(const TRadioCfg_RadioMode radioMode, UINT8* statusRsp) {
     TWiMODLRResultCodes result = WiMODLR_RESULT_TRANMIT_ERROR;
     if (statusRsp && (WiMOD_SAP_DevMgmt.txyPayloadSize >= 1)) {
    	 WiMOD_SAP_DevMgmt.txPayload[0] = (UINT8) radioMode;
         result = WiMOD_SAP_DevMgmt.HciParser->SendHCIMessage(DEVMGMT_SAP_ID,
                 DEVMGMT_MSG_SET_RADIO_MODE_REQ,
                 DEVMGMT_MSG_SET_RADIO_MODE_RSP,
				 WiMOD_SAP_DevMgmt.txPayload, 1);
         // copy response status
         if (WiMODLR_RESULT_OK == result) {
             *statusRsp = WiMOD_SAP_DevMgmt.HciParser->GetRxMessage()->Payload[WiMODLR_HCI_RSP_STATUS_POS];
         }
     }
     return result;
 }


//-----------------------------------------------------------------------------
/**
 * @brief SetAesKey Cmd - Set the 128bit AES that is to be used for encryption
 *
 *
 * @param   key         pointer to the 128bit AES key (pointer to array of 16 UINT8 entries)
 * @param   statusRsp   pointer to store status byte of response mesg from WiMOD
 *
 * @retval WiMODLR_RESULT_OK     if command transmit to WiMOD was ok
 */
TWiMODLRResultCodes setAesKey(const UINT8* key, UINT8* statusRsp) {
    TWiMODLRResultCodes result = WiMODLR_RESULT_TRANMIT_ERROR;
    if (statusRsp && key && (WiMOD_SAP_DevMgmt.txyPayloadSize >= DEVMGMT_AES_KEY_LEN)) {

        memcpy(WiMOD_SAP_DevMgmt.txPayload, key, DEVMGMT_AES_KEY_LEN);
        result = WiMOD_SAP_DevMgmt.HciParser->SendHCIMessage(DEVMGMT_SAP_ID,
                DEVMGMT_MSG_SET_AES_KEY_REQ,
                DEVMGMT_MSG_SET_AES_KEY_RSP,
				WiMOD_SAP_DevMgmt.txPayload, DEVMGMT_AES_KEY_LEN);
        // copy response status
        if (WiMODLR_RESULT_OK == result) {
            *statusRsp = WiMOD_SAP_DevMgmt.HciParser->GetRxMessage()->Payload[WiMODLR_HCI_RSP_STATUS_POS];
        }
    } else {
        result = WiMODLR_RESULT_PAYLOAD_PTR_ERROR;
    }
    return result;
}

//-----------------------------------------------------------------------------
/**
 * @brief GetAesKey Cmd - Get the 128bit AES that that is used for encryption
 *
 *
 * @param   key         pointer where to store the 128bit AES key (pointer to array of 16 UINT8 entries)
 * @param   statusRsp   pointer to store status byte of response mesg from WiMOD
 *
 * @retval WiMODLR_RESULT_OK     if command transmit to WiMOD was ok
 */
TWiMODLRResultCodes getAesKey(UINT8* key, UINT8* statusRsp)
{
    TWiMODLRResultCodes result = WiMODLR_RESULT_TRANMIT_ERROR;
    if (statusRsp && key) {

        result = WiMOD_SAP_DevMgmt.HciParser->SendHCIMessage(DEVMGMT_SAP_ID,
                DEVMGMT_MSG_GET_AES_KEY_REQ,
                DEVMGMT_MSG_GET_AES_KEY_RSP,
                NULL, 0);

        // copy response status
        if (WiMODLR_RESULT_OK == result) {
            *statusRsp = WiMOD_SAP_DevMgmt.HciParser->GetRxMessage()->Payload[WiMODLR_HCI_RSP_STATUS_POS];
            memcpy(key, &WiMOD_SAP_DevMgmt.HciParser->GetRxMessage()->Payload[WiMODLR_HCI_RSP_CMD_PAYLOAD_POS],  DEVMGMT_AES_KEY_LEN);
        }
    } else {
        result = WiMODLR_RESULT_PAYLOAD_PTR_ERROR;
    }
    return result;
}

//-----------------------------------------------------------------------------
/**
 * @brief SetRtcAlarm Cmd - Set a new RTC alarm config
 *
 *
 * @param   rtcAlarm   pointer to write the RTC alarm relevant parameter set to
 * @param   statusRsp   pointer to store status byte of response mesg from WiMOD
 *
 * @retval WiMODLR_RESULT_OK     if command transmit to WiMOD was ok
 */
TWiMODLRResultCodes setRtcAlarm(const TWiMODLR_DevMgmt_RtcAlarm* rtcAlarm, UINT8* statusRsp)
{
    TWiMODLRResultCodes result = WiMODLR_RESULT_TRANMIT_ERROR;
    UINT8 			   offset = 0;

    if (statusRsp && rtcAlarm && (WiMOD_SAP_DevMgmt.HciParser->Rx.ExpectedRsponseMsg.Length >= 0x04)) {

    	TWiMODLR_HCIMessage* tx = &WiMOD_SAP_DevMgmt.HciParser->TxMessage;

    	tx->Payload[offset++] = (UINT8) rtcAlarm->Options;
    	tx->Payload[offset++] = (UINT8) rtcAlarm->Hour;
    	tx->Payload[offset++] = (UINT8) rtcAlarm->Minutes;
    	tx->Payload[offset++] = (UINT8) rtcAlarm->Seconds;

		*statusRsp = WiMOD_SAP_DevMgmt.HciParser->PostMessage(DEVMGMT_SAP_ID, DEVMGMT_MSG_SET_RTC_ALARM_RSP, &tx->Payload[WiMODLR_HCI_RSP_STATUS_POS], sizeof(TWiMODLR_DevMgmt_RtcAlarm));
		result = WiMODLR_RESULT_OK;
    }
    else {
        result = WiMODLR_RESULT_PAYLOAD_PTR_ERROR;
    }
    return result;
}

//-----------------------------------------------------------------------------
/**
 * @brief GetRtcAlarm Cmd - Get information about RTC alarm feature
 *
 *
 * @param   rtcAlarm    pointer where to store the RTC alarm parameter set
 * @param   statusRsp   pointer to store status byte of response mesg from WiMOD
 *
 * @retval WiMODLR_RESULT_OK     if command transmit to WiMOD was ok
 */
TWiMODLRResultCodes getRtcAlarm(TWiMODLR_DevMgmt_RtcAlarm* rtcAlarm, UINT8* statusRsp)
{
    TWiMODLRResultCodes result = WiMODLR_RESULT_TRANMIT_ERROR;
    UINT8			   offset = WiMODLR_HCI_RSP_STATUS_POS;

    // copy response status
    if (statusRsp) {
    	TWiMODLR_HCIMessage* tx = &WiMOD_SAP_DevMgmt.HciParser->TxMessage;
    	tx->Payload[offset++] = *statusRsp;
		tx->Payload[offset++] = rtcAlarm->AlarmStatus;
		tx->Payload[offset++] = rtcAlarm->Options;
		tx->Payload[offset++] = rtcAlarm->Hour;
		tx->Payload[offset++] = rtcAlarm->Minutes;
		tx->Payload[offset++] = rtcAlarm->Seconds;
		*statusRsp = WiMOD_SAP_DevMgmt.HciParser->PostMessage(DEVMGMT_SAP_ID, DEVMGMT_MSG_GET_RTC_ALARM_RSP, &tx->Payload[WiMODLR_HCI_RSP_STATUS_POS], sizeof(TWiMODLR_DevMgmt_RtcAlarm) + 1);
		result = WiMODLR_RESULT_OK;
    }
    else {
        result = WiMODLR_RESULT_PAYLOAD_PTR_ERROR;
    }
    return result;
}

//-----------------------------------------------------------------------------
/**
 * @brief ClearRtcAlarm Cmd - Clear a pending RTC alarm
 *
 *
 * @param   statusRsp   pointer to store status byte of response mesg from WiMOD
 *
 * @retval WiMODLR_RESULT_OK     if command transmit to WiMOD was ok
 */

TWiMODLRResultCodes clearRtcAlarm(UINT8* statusRsp)
{
    TWiMODLRResultCodes result = WiMODLR_RESULT_TRANMIT_ERROR;

    if (statusRsp) {
    	TWiMODLR_HCIMessage* tx = &WiMOD_SAP_DevMgmt.HciParser->TxMessage;
    	tx->Payload[0] = 0x55;
		*statusRsp = WiMOD_SAP_DevMgmt.HciParser->PostMessage(DEVMGMT_SAP_ID, DEVMGMT_MSG_CLEAR_RTC_ALARM_RSP, &tx->Payload[WiMODLR_HCI_RSP_STATUS_POS], 1);
		result = WiMODLR_RESULT_OK;
	}
    return result;
}


//-----------------------------------------------------------------------------
/**
 * @brief SetHCIConfig Cmd - Sets the HCI config options for the LR-BASE_PLUS firmware
 *
 *
 * @param   hciConfig   pointer to the new configuration
 * @param   statusRsp   pointer to store status byte of response mesg from WiMOD
 *
 * @retval WiMODLR_RESULT_OK     if command transmit to WiMOD was ok
 */

TWiMODLRResultCodes setHCIConfig(const TWiMODLR_DevMgmt_HciConfig* hciConfig, UINT8* statusRsp)
{
    TWiMODLRResultCodes result = WiMODLR_RESULT_TRANMIT_ERROR;
    UINT8              offset = 0;

    if (hciConfig && statusRsp && (WiMOD_SAP_DevMgmt.txyPayloadSize >= 0x06)) {
    	WiMOD_SAP_DevMgmt.txPayload[offset++] = (UINT8) hciConfig->StoreNwmFlag;
    	WiMOD_SAP_DevMgmt.txPayload[offset++] = (UINT8) hciConfig->BaudrateID;
        HTON16(&WiMOD_SAP_DevMgmt.txPayload[offset], hciConfig->NumWakeUpChars);
        offset += 0x02;
        WiMOD_SAP_DevMgmt.txPayload[offset++] = (UINT8) hciConfig->TxHoldTime;
        WiMOD_SAP_DevMgmt.txPayload[offset++] = (UINT8) hciConfig->RxHoldTime;

        result = WiMOD_SAP_DevMgmt.HciParser->SendHCIMessage(DEVMGMT_SAP_ID,
                                        DEVMGMT_MSG_SET_HCI_CFG_REQ,
                                        DEVMGMT_MSG_SET_HCI_CFG_RSP,
										WiMOD_SAP_DevMgmt.txPayload, offset);

        if (result == WiMODLR_RESULT_OK) {
            const TWiMODLR_HCIMessage* rx = WiMOD_SAP_DevMgmt.HciParser->GetRxMessage();
            offset = WiMODLR_HCI_RSP_STATUS_POS;
            *statusRsp = rx->Payload[offset++];

            // status check
            if (*statusRsp == DEVMGMT_STATUS_OK) {


            } else {
            }
        }
    }
    return result;

}

//-----------------------------------------------------------------------------
/**
 * @brief SetHCIConfig Cmd - Gets the HCI config options for the LR-BASE_PLUS firmware
 *
 *
 * @param   hciConfig   pointer to store the received information
 * @param   statusRsp   pointer to store status byte of response mesg from WiMOD
 *
 * @retval WiMODLR_RESULT_OK     if command transmit to WiMOD was ok
 */
TWiMODLRResultCodes getHCIConfig(TWiMODLR_DevMgmt_HciConfig* hciConfig, UINT8* statusRsp)
{
    TWiMODLRResultCodes result = WiMODLR_RESULT_TRANMIT_ERROR;
    UINT8              offset = WiMODLR_HCI_RSP_STATUS_POS;

    if (hciConfig && statusRsp) {
        result = WiMOD_SAP_DevMgmt.HciParser->SendHCIMessage(DEVMGMT_SAP_ID,
                                           DEVMGMT_MSG_GET_HCI_CFG_REQ,
                                           DEVMGMT_MSG_GET_HCI_CFG_RSP,
                                           NULL, 0);

        if (result == WiMODLR_RESULT_OK) {
            const TWiMODLR_HCIMessage* rx = WiMOD_SAP_DevMgmt.HciParser->GetRxMessage();

            hciConfig->Status         = rx->Payload[offset++];
            hciConfig->BaudrateID     = (TWiMOD_HCI_Baudrate) rx->Payload[offset++];
            hciConfig->NumWakeUpChars = NTOH16(&rx->Payload[offset]);
            offset += 0x02;
            hciConfig->TxHoldTime     = (UINT8)rx->Payload[offset++];
            hciConfig->RxHoldTime     = (UINT8)rx->Payload[offset++];

            *statusRsp = rx->Payload[WiMODLR_HCI_RSP_STATUS_POS];
        }
    }
    return result;
}


//-----------------------------------------------------------------------------
/**
 * @brief Register a callback function for processing a PowerUp Indication message
 *
 *
 * @param   cb          pointer to callback function
 */

void registerPowerUpIndicationClient(TDevMgmtPowerUpCallback cb)
{
	WiMOD_SAP_DevMgmt.PowerUpCallack = cb;
}

//-----------------------------------------------------------------------------
/**
 * @brief Register a callback function for processing a RTC Alarm Indication message
 *
 *
 * @param   cb          pointer to callback function
 */
void WregisterRtcAlarmIndicationClient(TDevMgmtRtcAlarmCallback cb)
{
	WiMOD_SAP_DevMgmt.RtcAlarmCallback = cb;
}

void process (UINT8* statusRsp, TWiMODLR_HCIMessage* rxMsg)
{
    switch (rxMsg->MsgID)
    {
        case DEVMGMT_MSG_PING_REQ:
        	WiMOD_SAP_DevMgmt.Ping(statusRsp);

            break;
        case DEVMGMT_MSG_GET_DEVICEINFO_REQ:
        	WiMOD_SAP_DevMgmt.GetDeviceInfo(&DeviceInfo, statusRsp);

        	break;
        case DEVMGMT_MSG_GET_FW_VERSION_REQ:
        	WiMOD_SAP_DevMgmt.GetFirmwareInfo(&firmwareInfo, statusRsp);
        	break;
        case DEVMGMT_MSG_RESET_REQ:
        	WiMOD_SAP_DevMgmt.Reset(statusRsp);
        	break;
        case DEVMGMT_MSG_SET_OPMODE_REQ:
        	WiMOD_SAP_DevMgmt.SetOperationMode(rxMsg->Payload[0], statusRsp);
        	break;
        case DEVMGMT_MSG_GET_OPMODE_REQ:
        	WiMOD_SAP_DevMgmt.GetOperationMode(&OperationMode, statusRsp);
        	break;
        case DEVMGMT_MSG_SET_RTC_REQ:
        	WiMOD_SAP_DevMgmt.SetRtc(NTOH32(rxMsg->Payload[0]), statusRsp);
        	break;
        case DEVMGMT_MSG_GET_RTC_REQ:
        	WiMOD_SAP_DevMgmt.GetRtc(&RTCTime, statusRsp);
        	break;
        case DEVMGMT_MSG_GET_SYSTEM_STATUS_REQ:
        	WiMOD_SAP_DevMgmt.GetSystemStatus(&SystemInfo, statusRsp);
        	break;
        case DEVMGMT_MSG_SET_RTC_ALARM_REQ:
        	WiMOD_SAP_DevMgmt.SetRtcAlarm(rxMsg->Payload[0], statusRsp);
        	break;
        case DEVMGMT_MSG_CLEAR_RTC_ALARM_REQ:
        	WiMOD_SAP_DevMgmt.ClearRtcAlarm(statusRsp);
        	break;
        case DEVMGMT_MSG_GET_RTC_ALARM_REQ:
        	WiMOD_SAP_DevMgmt.GetRtcAlarm(rxMsg->Payload[0], statusRsp);
        	break;
        default:
            break;
    }
    return;
}
//------------------------------------------------------------------------------
//
// Section protected functions
//
//------------------------------------------------------------------------------

/**
 * @internal
 *
 * @brief Dispatch messages from the WiMOD (aka indications)
 *
 * @param rxMsg reference to the complete received HCI message; DO NOT MODIFIY it!
 *
 * @endinternal
 */
void dispatchDeviceMgmtMessage(TWiMODLR_HCIMessage* rxMsg)
{
    switch (rxMsg->MsgID)
    {
        case DEVMGMT_MSG_POWER_UP_IND:
            if (WiMOD_SAP_DevMgmt.PowerUpCallack) {
            	WiMOD_SAP_DevMgmt.PowerUpCallack();
            }
            break;
        case DEVMGMT_MSG_RTC_ALARM_IND:
        	if (WiMOD_SAP_DevMgmt.RtcAlarmCallback) {
        		WiMOD_SAP_DevMgmt.RtcAlarmCallback();
        	}
        	break;
        default:
            break;
    }
    return;
}


//------------------------------------------------------------------------------
//
// Section private functions
//
//------------------------------------------------------------------------------


