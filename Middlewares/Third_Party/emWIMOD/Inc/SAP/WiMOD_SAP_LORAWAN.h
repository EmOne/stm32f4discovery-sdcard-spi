//------------------------------------------------------------------------------
//! @file WiMOD_SAP_LORAWAN.h
//! @ingroup WiMOD_SAP_LORAWAN
//! <!------------------------------------------------------------------------->
//! @brief Declarations for the LoRaWAN SericeAccessPoint
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


#ifndef ARDUINO_WIMOD_SAP_LORAWAN_H_
#define ARDUINO_WIMOD_SAP_LORAWAN_H_

/*
 * THIS IS AN EXAMPLE IMPLEMENTATION ACCORDING THE THE HCI SPEC: V1.11
 * FOR FIRMWARE: LoRaWAN_EndNode_Modem
 *
 * SEE FILE: WiMOD_LoRaWAN_EndNode_Modem_HCI_Spec_V1_11.pdf
 * for detailed information
 *
 */


//------------------------------------------------------------------------------
//
// Section Includes Files
//
//------------------------------------------------------------------------------

#include "WiMOD_SAP_LORAWAN_IDs.h"
#include "../HCI/WiMODLRHCI.h"

/*
 * C++11 supports a better way for function pointers / function objects
 * But C++11 mode is not supported by all platforms.
 */
#ifdef WIMOD_USE_CPP11
#include <functional>
#endif
//------------------------------------------------------------------------------
//
// Section defines
//
//------------------------------------------------------------------------------

//! @cond Doxygen_Suppress
#define WiMOD_LORAWAN_TX_PAYLOAD_SIZE               (WiMODLORAWAN_APP_PAYLOAD_LEN + 5)
//! @endcond

//! @cond Doxygen_Suppress
//! @endcond

/**
 * @brief LoRaWAN Region support
 */
typedef enum {
	LoRaWAN_Region_EU868,                                                       /*!< regional settings for EU 868 MHz  */
	LoRaWAN_Region_US915,                                                       /*!< regional settings for US 915 MHz  */
	LoRaWAN_Region_IN865,                                                       /*!< regional settings for India 865 MHz  */
	LoRaWAN_Region_AS923,                                                       /*!< regional settings for Asia 923 MHz  */
	LoRaWAN_Region_IL915,                                                       /*!< regional settings for Israel 915 MHz  */
	LoRaWAN_Region_RU868,                                                       /*!< regional settings for Russia 868 MHz  */
	LoRaWAN_Region_proprietary_WW2G4,

} TLoRaWANregion;



//-----------------------------------------------------------------------------
//
// types for callback functions
//
//-----------------------------------------------------------------------------

// C++11 check
#ifdef WIMOD_USE_CPP11
/* C++11 function callback definitions */

	/** Type definition for a 'join tx' indication callback  */
	typedef std::function<void (TWiMODLR_HCIMessage& rxMsg)> TJoinTxIndicationCallback;

	/** Type definition for a 'joined (lora) nwk' indication callback  */
	typedef std::function<void (TWiMODLR_HCIMessage& rxMsg)> TJoinedNwkIndicationCallback;

	/** Type definition for a 'no data / timemout' indication callback  */
	typedef std::function<void (void)> TNoDataIndicationCallback;

	/** Type definition for a 'Tx C-Data' indication callback  */
	typedef std::function<void (TWiMODLR_HCIMessage& rxMsg)> TTxCDataIndicationCallback;

	/** Type definition for a 'Tx U-Data' indication callback  */
	typedef std::function<void (TWiMODLR_HCIMessage& rxMsg)> TTxUDataIndicationCallback;

	/** Type definition for a 'Rx U-Data' indication callback  */
	typedef std::function<void (TWiMODLR_HCIMessage& rxMsg)> TRxUDataIndicationCallback;

	/** Type definition for a 'Rx C-Datat' indication callback  */
	typedef std::function<void (TWiMODLR_HCIMessage& rxMsg)> TRxCDataIndicationCallback;

	/** Type definition for a 'Rx MAC-Cmd' indication callback  */
	typedef std::function<void (TWiMODLR_HCIMessage& rxMsg)> TRxMacCmdIndicationCallback;

	/** Type definition for a 'Rx Ack' indication callback  */
	typedef std::function<void (TWiMODLR_HCIMessage& rxMsg)> TRxAckIndicationCallback;

#else
	/* pre C++11 function callback definitions */

	/** Type definition for a 'join tx' indication callback function */
	typedef void (*TJoinTxIndicationCallback)(TWiMODLR_HCIMessage* rxMsg);

	/** Type definition for a 'joined (lora) nwk' indication callback function */
	typedef void (*TJoinedNwkIndicationCallback)(TWiMODLR_HCIMessage* rxMsg);

	/** Type definition for a 'no data / timemout' indication callback function */
	typedef void (*TNoDataIndicationCallback)(void);

	/** Type definition for a 'Tx C-Data' indication callback function */
	typedef void (*TTxCDataIndicationCallback)(TWiMODLR_HCIMessage* rxMsg);

	/** Type definition for a 'Tx U-Data' indication callback function */
	typedef void (*TTxUDataIndicationCallback)(TWiMODLR_HCIMessage* rxMsg);

	/** Type definition for a 'Rx U-Data' indication callback function */
	typedef void (*TRxUDataIndicationCallback)(TWiMODLR_HCIMessage* rxMsg);

	/** Type definition for a 'Rx C-Datat' indication callback function */
	typedef void (*TRxCDataIndicationCallback)(TWiMODLR_HCIMessage* rxMsg);

	/** Type definition for a 'Rx MAC-Cmd' indication callback function */
	typedef void (*TRxMacCmdIndicationCallback)(TWiMODLR_HCIMessage* rxMsg);

	/** Type definition for a 'Rx Ack' indication callback function */
	typedef void (*TRxAckIndicationCallback)(TWiMODLR_HCIMessage* rxMsg);

#endif


//------------------------------------------------------------------------------
//
// Section class
//
//------------------------------------------------------------------------------

/**
 * @brief Implementation for the ServiceAccessPoint LoRaWAN (End Node Modem firmware only)
 */
typedef struct {
//public:
//
//    WiMOD_SAP_LoRaWAN(TWiMODLRHCI* hci, UINT8* buffer, UINT16 bufferSize);
//    virtual ~WiMOD_SAP_LoRaWAN(void);


    void (*setRegion)(TLoRaWANregion regionalSetting);

    TWiMODLRResultCodes (*ActivateDevice)(TWiMODLORAWAN_ActivateDeviceData* activationData, UINT8* statusRsp);
    TWiMODLRResultCodes (*ReactivateDevice)(UINT32* devAdr, UINT8* statusRsp);
    TWiMODLRResultCodes (*SetJoinParameter)(TWiMODLORAWAN_JoinParams* joinParams, UINT8* statusRsp);
    TWiMODLRResultCodes (*JoinNetwork)(UINT8* statusRsp);

    void (*RegisterJoinTxIndicationClient)(TJoinTxIndicationCallback cb);

//    bool (*convert1)(TWiMODLR_HCIMessage* RxMsg, TWiMODLORAWAN_JoinNwkTxIndData* indicationData);
    bool (*convert2)(TWiMODLR_HCIMessage* RxMsg, TWiMODLORAWAN_RX_Data* loraWanRxData);
//    bool (*convert3)(TWiMODLR_HCIMessage* rxMsg, TWiMODLORAWAN_SendDataTxInd_Data* sendIndData);
    bool (*convert4)(TWiMODLR_HCIMessage* rxMsg, TWiMODLORAWAN_TxIndData* sendIndData);
    bool (*convert5)(TWiMODLR_HCIMessage* RxMsg, TWiMODLORAWAN_RX_MacCmdData* loraWanMacCmdData);
    bool (*convert6)(TWiMODLR_HCIMessage* RxMsg, TWiMODLORAWAN_RX_JoinedNwkData* joinedNwkData);
    bool (*convert7)(TWiMODLR_HCIMessage* RxMsg, TWiMODLORAWAN_RX_ACK_Data* ackData);
    bool (*convert8)(TWiMODLR_HCIMessage* RxMsg, TWiMODLORAWAN_NoData_Data* info);


    void (*RegisterNoDataIndicationClient)(TNoDataIndicationCallback cb);
    void (*RegisterTxCDataIndicationClient)(TTxCDataIndicationCallback cb);
    void (*RegisterTxUDataIndicationClient)(TTxUDataIndicationCallback cb);
    void (*RegisterRxUDataIndicationClient)(TRxUDataIndicationCallback cb);
    void (*RegisterRxCDataIndicationClient)(TRxCDataIndicationCallback cb);
    void (*RegisterRxMacCmdIndicationClient)(TRxMacCmdIndicationCallback cb);
    void (*RegisterJoinedNwkIndicationClient)(TJoinedNwkIndicationCallback cb);
    void (*RegisterRxAckIndicationClient)(TRxAckIndicationCallback cb);


    TWiMODLRResultCodes (*SendUData)(const TWiMODLORAWAN_TX_Data* data, UINT8* statusRsp);
    TWiMODLRResultCodes (*SendCData)(const TWiMODLORAWAN_TX_Data* data, UINT8* statusRsp);
    TWiMODLRResultCodes (*SetRadioStackConfig)(TWiMODLORAWAN_RadioStackConfig* data, UINT8* statusRsp);
    TWiMODLRResultCodes (*GetRadioStackConfig)(TWiMODLORAWAN_RadioStackConfig* data, UINT8* statusRsp);
    TWiMODLRResultCodes (*DeactivateDevice)(UINT8* statusRsp);
    TWiMODLRResultCodes (*FactoryReset)(UINT8* statusRsp);
    TWiMODLRResultCodes (*SetDeviceEUI)(const UINT8* deviceEUI, UINT8* statusRsp);
    TWiMODLRResultCodes (*GetDeviceEUI)(UINT8* deviceEUI, UINT8* statusRsp);
//    TWiMODLRResultCodes GetNwkStatus(UINT8* nwkStatus, UINT8* statusRsp); // implementation up to spec V1.13
    TWiMODLRResultCodes (*GetNwkStatus)(TWiMODLORAWAN_NwkStatus_Data* nwkStatus, UINT8* statusRsp); // new implemation for spec. V1.14

    TWiMODLRResultCodes (*SendMacCmd)(const TWiMODLORAWAN_MacCmd* cmd, UINT8* statusRsp);
    TWiMODLRResultCodes (*SetCustomConfig)(const INT8 rfGain, UINT8* statusRsp);
    TWiMODLRResultCodes (*GetCustomConfig)(INT8* rfGain, UINT8* statusRsp);
    TWiMODLRResultCodes (*GetSupportedBands)(TWiMODLORAWAN_SupportedBands* supportedBands, UINT8* statusRsp);
    TWiMODLRResultCodes (*GetTxPowerLimitConfig)(TWiMODLORAWAN_TxPwrLimitConfig* txPwrLimitCfg, UINT8* statusRsp);
    TWiMODLRResultCodes (*SetTxPowerLimitConfig)(TWiMODLORAWAN_TxPwrLimitConfig* txPwrLimitCfg, UINT8* statusRsp);

    TWiMODLRResultCodes	(*GetLinkAdrReqConfig)(TWiMODLORAWAN_LinkAdrReqConfig* linkAdrReqCfg, UINT8* statusRsp);
    TWiMODLRResultCodes (*SetLinkAdrReqConfig)(TWiMODLORAWAN_LinkAdrReqConfig* linkAdrReqCfg, UINT8* statusRsp);

    TWiMODLRResultCodes (*SetBatteryLevelStatus)(UINT8 battStatus, UINT8* statusRsp);

    void (*DispatchLoRaWANMessage)(TWiMODLR_HCIMessage* rxMsg);
    void (*Process)(UINT8* statusRsp, TWiMODLR_HCIMessage* rxMsg);
//protected:
    //! @cond Doxygen_Suppress
    TJoinTxIndicationCallback    JoinTxIndCallback;
    TNoDataIndicationCallback    NoDataIndCallback;
    TTxCDataIndicationCallback   TxCDataIndCallback;
    TTxUDataIndicationCallback   TxUDataIndCallback;
    TRxUDataIndicationCallback   RxUDataIndCallback;
    TRxCDataIndicationCallback   RxCDataIndCallback;
    TRxMacCmdIndicationCallback  RxMacCmdIndCallback;
    TJoinedNwkIndicationCallback JoinedNwkIndCallback;
    TRxAckIndicationCallback     RxAckIndCallback;

    UINT8*              	     txPayload;
    UINT16              		 txPayloadSize;
    TWiMODLRHCI_t*        		 HciParser;
    TLoRaWANregion				 region;
    //! @endcond
//private:
    //! @cond Doxygen_Suppress

    //! @endcond

}WiMOD_SAP_LoRaWAN_t;

extern WiMOD_SAP_LoRaWAN_t WiMOD_SAP_LoRaWAN;

#endif /* ARDUINO_WIMOD_SAP_LORAWAN_H_ */
