//------------------------------------------------------------------------------
//! @file WiMOD_SAP_LORAWAN_IDs.h
//! @ingroup WiMOD_SAP_LORAWAN
//! <!------------------------------------------------------------------------->
//! @brief Supporting IDs and definitions for the LoRaWAN ServiceAccessPoint
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
 * THIS IS AN EXAMPLE IMPLEMENTATION ACCORDING THE THE HCI SPEC: V1.26
 * FOR FIRMWARE: LoRaWAN_EndNode_Modem
 *
 * SEE FILE: WiMOD_LoRaWAN_EndNode_Modem_HCI_Spec_V1_26.pdf
 * for detailed information
 *
 */


#ifndef ARDUINO_WIMOD_SAP_LORAWAN_IDS_H_
#define ARDUINO_WIMOD_SAP_LORAWAN_IDS_H_

//------------------------------------------------------------------------------
//
// Section Includes Files
//
//------------------------------------------------------------------------------

#include "../utils/WMDefs.h"

#include "WiMOD_SAP_LORAWAN_IDs_EU868.h"  /* Europe       */
#include "WiMOD_SAP_LORAWAN_IDs_RU868.h"  /* Russia       */
#include "WiMOD_SAP_LORAWAN_IDs_IN865.h"  /* India        */
#include "WiMOD_SAP_LORAWAN_IDs_AS923.h"  /* Asia/Pacific */
#include "WiMOD_SAP_LORAWAN_IDs_IL915.h"  /* Israel       */
#include "WiMOD_SAP_LORAWAN_IDs_US915.h"  /* US           */

#include "WiMOD_SAP_GlobalLink24_IDs_WW2G4.h" /* prorieaty 2.4 GHz WorldWide */

//------------------------------------------------------------------------------
//
// Service Access Point Identifier
//
//------------------------------------------------------------------------------

/** SAP ID for the LoRaWAN service */

//! @cond Doxygen_Suppress
#define LORAWAN_SAP_ID                      0x10
//! @endcond


//------------------------------------------------------------------------------
//
//  LoRaWAN Message Identifier
//
//------------------------------------------------------------------------------
//! @cond Doxygen_Suppress

// Status Codes

#define LORAWAN_STATUS_OK                       0x00 //Operation successful
#define LORAWAN_STATUS_ERROR                    0x01 //Operation failed
#define LORAWAN_STATUS_CMD_NOT_SUPPORTED        0x02 //Command is not supported
#define LORAWAN_STATUS_WRONG_PARAMETER          0x03 //HCI message contains wrong parameter
#define LORAWAN_STATUS_WRONG_DEVICE_MODE        0x04 //Stack is running in a wrong mode
#define LORAWAN_STATUS_DEVICE_NOT_ACTIVATED     0x05 //Device is not activated
#define LORAWAN_STATUS_DEVICE_BUSY              0x06 //Device is busy, command rejected
#define LORAWAN_STATUS_QUEUE_FULL               0x07 //Message queue is full, command rejected
#define LORAWAN_STATUS_LENGTH_ERROR             0x08 //HCI message length is invalid or radio payload size is too large
#define LORAWAN_STATUS_NO_FACTORY_SETTINGS      0x09 //Factory Settings EEPROM block missing or incompatible with current firmware version
#define LORAWAN_STATUS_CHANNEL_BLOCKED          0x0A //Channel blocked by Duty Cycle
#define LORAWAN_STATUS_CHANNEL_NOT_AVAILABLE    0x0B //No channel available (e.g. no channel defined for the configured spreading factor)

//! @endcond

/**
 * @brief Status code enumeration for LoRaWAN SAP of the WiMOD EndNode Modem firmware
 */
typedef enum TLoRaWANStatus
{
    LoRaWAN_Status_Ok = 0,                                                      /*!< OK; No error */
    LoRaWAN_Status_Error,                                                       /*!< general (un-specific) error */
    LoRaWAN_Status_Cmd_Not_Supported,                                           /*!< Command is not supported (or unknown) */
    LoRaWAN_Status_Wrong_parameter,                                             /*!< Wrong parameter applied */
    LoRaWAN_Status_Wrong_DeviceMode,                                            /*!< Device is in wong device mode for specific command */
    LoRaWAN_Status_Device_Not_Activated,                                        /*!< Device has NOT been activated; command has not been executed */
    LoRaWAN_Status_Device_Busy,                                                 /*!< There is another operation pending; and or duty cycle restriction is active  */
    LoRaWAN_Status_Queue_Full,                                                  /*!< internal command queue is full */
    LoRaWAN_Status_Length_Error,                                                /*!< some error related to the given length information */
    LoRaWAN_Status_No_Factory_Settings,                                         /*!< Factory settings are not available */
    LoRaWAN_Status_Channel_Blocked,                                             /*!< Selected channel is blocked (duty cycle?) */
    LoRaWAN_Status_Channel_Not_Available,                                       /*!< Selected channel is not available */
} TLoRaWANStatus;

//! @cond Doxygen_Suppress

// These codes are used for a Join Network Indication

#define LORAWAN_JOIN_NWK_IND_FORMAT_STATUS_JOIN_OK                0x00          /*!< device activated */
#define LORAWAN_JOIN_NWK_IND_FORMAT_STATUS_JOIN_OK_CH_INFO        0x01          /*!< device additional channel info attached */

//! @endcond

/**
 * @brief Status code enumeration for LoRaWAN Join Network Indications
 */
typedef enum TLoraWANJoinNwkIndStatusFormat
{
	LoRaWAN_JoinNkwInd_Satus_Format_Join_Ok                     = 0x00,         /*!< device activated */
	LoRaWAN_JoinNkwInd_Satus_Format_Join_Ok_Ch_Info             = 0x01,         /*!< device additional channel info attached */
} TLoRaWanJoinNwkIndStatusFormat;


//! @cond Doxygen_Suppress

// These codes are used for a UDATA / CDATA TX Indications

#define LORAWAN_DATA_TX_IND_FORMAT_STATUS_OK                 0x00               /*!< radio packet sent */
#define LORAWAN_DATA_TX_IND_FORMAT_STATUS_OK_CH_INFO         0x01               /*!< radio packet sent + channel info attached */
#define LORAWAN_DATA_TX_IND_FORMAT_STATUS_ERR_MAX_RETRANS    0x02               /*!< error: max number retransmissions reached */
#define LORAWAN_DATA_TX_IND_FORMAT_STATUS_ERR_PAYLOAD        0x04               /*!< error: payload size exceeted for current data rate */

//! @endcond

/**
 * @brief Status code enumeration for LoRaWAN Join Network Indications
 */
typedef enum TLoraWANTxDataIndStatusFormat
{
	/* for udata + cdata indications */
	LoRaWAN_TxDataInd_Satus_TX_Ok                   = 0x00,                     /*!< radio packet sent */
	LoRaWAN_TxDataInd_Satus_TX_Ok_Ch_Info           = 0x01,                     /*!< radio packet sent + channel info attached */
	/*only for cdata indications */
	LoRaWAN_TxDataInd_Satus_TX_Err_Max_Retrans      = 0x02,                     /*!< error: max number retransmissions reached */
	LoRaWAN_TxDataInd_Satus_TX_Err_Payload          = 0x04,                     /*!< error: payload size exceeted for current data rate */
} TLoRaWanTxDataIndStatusFormat;





//! @cond Doxygen_Suppress


// Status Format Codes
//  This is a bit mask field!
#define LORAWAN_FORMAT_OK                       0x00                            /*!< OK, no error */
#define LORAWAN_FORMAT_EXT_HCI_OUT_ACTIVE       0x01                            /*!< extended HCI output format */
#define LORAWAN_FORMAT_ACK_RECEIVED             0x02                            /*!< message contains an ACK */
#define LORAWAN_FORMAT_FP_RECEIVED              0x04                            /*!< a FramePending bit as been received */

// Message IDs
//! @cond Doxygen_Suppress

#define LORAWAN_MSG_ACTIVATE_DEVICE_REQ          0x01
#define LORAWAN_MSG_ACTIVATE_DEVICE_RSP          0x02
#define LORAWAN_MSG_SET_JOIN_PARAM_REQ           0x05
#define LORAWAN_MSG_SET_JOIN_PARAM_RSP           0x06
#define LORAWAN_MSG_JOIN_NETWORK_REQ             0x09
#define LORAWAN_MSG_JOIN_NETWORK_RSP             0x0A
#define LORAWAN_MSG_JOIN_NETWORK_TX_IND          0x0B
#define LORAWAN_MSG_JOIN_NETWORK_IND             0x0C
#define LORAWAN_MSG_SEND_UDATA_REQ               0x0D
#define LORAWAN_MSG_SEND_UDATA_RSP               0x0E
#define LORAWAN_MSG_SEND_UDATA_TX_IND            0x0F
#define LORAWAN_MSG_RECV_UDATA_IND               0x10
#define LORAWAN_MSG_SEND_CDATA_REQ               0x11
#define LORAWAN_MSG_SEND_CDATA_RSP               0x12
#define LORAWAN_MSG_SEND_CDATA_TX_IND            0x13
#define LORAWAN_MSG_RECV_CDATA_IND               0x14
#define LORAWAN_MSG_RECV_ACK_IND                 0x15
#define LORAWAN_MSG_RECV_NO_DATA_IND             0x16
#define LORAWAN_MSG_SET_RSTACK_CONFIG_REQ        0x19
#define LORAWAN_MSG_SET_RSTACK_CONFIG_RSP        0x1A
#define LORAWAN_MSG_GET_RSTACK_CONFIG_REQ        0x1B
#define LORAWAN_MSG_GET_RSTACK_CONFIG_RSP        0x1C
#define LORAWAN_MSG_REACTIVATE_DEVICE_REQ        0x1D
#define LORAWAN_MSG_REACTIVATE_DEVICE_RSP        0x1E
#define LORAWAN_MSG_DEACTIVATE_DEVICE_REQ        0x21
#define LORAWAN_MSG_DEACTIVATE_DEVICE_RSP        0x22
#define LORAWAN_MSG_FACTORY_RESET_REQ            0x23
#define LORAWAN_MSG_FACTORY_RESET_RSP            0x24
#define LORAWAN_MSG_SET_DEVICE_EUI_REQ           0x25
#define LORAWAN_MSG_SET_DEVICE_EUI_RSP           0x26
#define LORAWAN_MSG_GET_DEVICE_EUI_REQ           0x27
#define LORAWAN_MSG_GET_DEVICE_EUI_RSP           0x28
#define LORAWAN_MSG_GET_NWK_STATUS_REQ           0x29
#define LORAWAN_MSG_GET_NWK_STATUS_RSP           0x2A
#define LORAWAN_MSG_SEND_MAC_CMD_REQ             0x2B
#define LORAWAN_MSG_SEND_MAC_CMD_RSP             0x2C
#define LORAWAN_MSG_RECV_MAC_CMD_IND             0x2D
#define LORAWAN_MSG_SET_BATTERY_LEVEL_REQ        0x2E
#define LORAWAN_MSG_SET_BATTERY_LEVEL_RSP        0x2F
#define LORAWAN_MSG_SET_CUSTOM_CFG_REQ           0x31
#define LORAWAN_MSG_SET_CUSTOM_CFG_RSP           0x32
#define LORAWAN_MSG_GET_CUSTOM_CFG_REQ           0x33
#define LORAWAN_MSG_GET_CUSTOM_CFG_RSP           0x34
#define LORAWAN_MSG_GET_SUPPORTED_BANDS_REQ      0x35
#define LORAWAN_MSG_GET_SUPPORTED_BANDS_RSP      0x36
#define LORAWAN_MSG_SET_TXPOWER_LIMIT_CONFIG_REQ 0x37
#define LORAWAN_MSG_SET_TXPOWER_LIMIT_CONFIG_RSP 0x38
#define LORAWAN_MSG_GET_TXPOWER_LIMIT_CONFIG_REQ 0x39
#define LORAWAN_MSG_GET_TXPOWER_LIMIT_CONFIG_RSP 0x3A
#define LORAWAN_MSG_SET_LINKADRREQ_CONFIG_REQ    0x3B
#define LORAWAN_MSG_SET_LINKADRREQ_CONFIG_RSP    0x3C
#define	LORAWAN_MSG_GET_LINKADRREQ_CONFIG_REQ    0x3D
#define LORAWAN_MSG_GET_LINKADRREQ_CONFIG_RSP    0x3E
//! @endcond

// Radio Stack Config Options; bit field

#define LORAWAN_STK_OPTION_ADR                  (0x01 << 0)                     /*!< Stack option ADR */
#define LORAWAN_STK_OPTION_DUTY_CYCLE_CTRL      (0x01 << 1)                     /*!< Stack option Duty Cycle Control */
#define LORAWAN_STK_OPTION_DEV_CLASS_C          (0x01 << 2)                     /*!< Stack option ClassC device */
#define LORAWAN_STK_OPTION_POWER_UP_IND         (0x01 << 4)                     /*!< Stack option for sending a PowerUp Indication */
#define LORAWAN_STK_OPTION_PRIVATE_NETOWRK      (0x01 << 5)                     /*!< Stack option for using a Private LoRaWAN Network; 0 = public LoRaWAN network */
#define LORAWAN_STK_OPTION_EXT_PKT_FORMAT       (0x01 << 6)                     /*!< Stack option extended format */
#define LORAWAN_STK_OPTION_MAC_CMD              (0x01 << 7)                     /*!< Stack option forwad MAC commands */











// Radio Stack Error Codes; bit field

#define LORAWAN_STK_PARAM_ERR_WRONG_DATARATE    (0x01 << 0)                     /*!< Stack Error. wrong parameter */
#define LORAWAN_STK_PARAM_ERR_WRONG_TX_PWRLEVEL (0x01 << 1)                     /*!< Stack Error. wrong TX power level */
#define LORAWAN_STK_PARAM_ERR_WRONG_TX_BANDIDX  (0x01 << 5)                     /*!< Stack Error. wrong band index */

// Radio Tx Power Limit Error Codes; bit field
#define LORAWAN_TX_PWR_LIMIT_ERR_WRONG_SUBBAND      (0x01 << 0)                 /*!< Power Limit Error: Wrong sub-band */
#define LORAWAN_TX_PWR_LIMIT_ERR_WRONG_TX_PWR_VAL   (0x01 << 2)                 /*!< Power Limit Error: Wrong TX power value */






//! @cond Doxygen_Suppress

//! @endcond



/**
 * @brief Channel Band Selection
 */

typedef enum TLoRaWAN_FreqBand
{
    LoRaWAN_FreqBand_EU_868            = LORAWAN_BAND_EU_868,                   /*!< EU 868 MHz band */

	LoRaWAN_FreqBand_US_915            = LORAWAN_BAND_US_915,                   /*!< US 915 MHz band */

	LoRaWAN_FreqBand_India_865         = LORAWAN_BAND_IN_865,                   /*!< India */

	LoRaWAN_FreqBand_AS_923_Brunei     = LORAWAN_BAND_AS_923_BN_923,            /*!< Brunei */
	LoRaWAN_FreqBand_AS_923_Cambodia   = LORAWAN_BAND_AS_923_KH_923,            /*!< Cambodia */
	LoRaWAN_FreqBand_AS_923_Indonesia  = LORAWAN_BAND_AS_923_ID_923,            /*!< Indonesia */
	LoRaWAN_FreqBand_AS_923_Japan      = LORAWAN_BAND_AS_923_JP_920,            /*!< Japan */
	LoRaWAN_FreqBand_AS_923_Laos       = LORAWAN_BAND_AS_923_LA_920,            /*!< Laos */
	LoRaWAN_FreqBand_AS_923_NewZealand = LORAWAN_BAND_AS_923_LA_915,            /*!< New Zealand */
	LoRaWAN_FreqBand_AS_923_Singapore  = LORAWAN_BAND_AS_923_SG_920,            /*!< Singapore */
	LoRaWAN_FreqBand_AS_923_Taiwan     = LORAWAN_BAND_AS_923_TW_922,            /*!< Taiwan */
	LoRaWAN_FreqBand_AS_923_Thailand   = LORAWAN_BAND_AS_923_TH_920,            /*!< Thailand */
	LoRaWAN_FreqBand_AS_923_Vietnam    = LORAWAN_BAND_AS_923_VN_920,            /*!< Vietnam */
	LoRaWAN_FreqBand_RU_868_V1         = LORAWAN_BAND_RU1_868,                  /*!< Russia */
	LoRaWAN_FreqBand_RU_868_V2         = LORAWAN_BAND_RU2_868,                  /*!< Russia */
	LoRaWAN_FreqBand_RU_868_V3         = LORAWAN_BAND_RU3_868,                  /*!< Russia */
	LoRaWAN_FreqBand_RU_868_V4         = LORAWAN_BAND_RU4_868,                  /*!< Russia */
	LoRaWAN_FreqBand_RU_868_V5         = LORAWAN_BAND_RU5_868,                  /*!< Russia */
	LoRaWAN_FreqBand_RU_868_V6         = LORAWAN_BAND_RU6_868,                  /*!< Russia */
	LoRaWAN_FreqBand_RU_868_V7         = LORAWAN_BAND_RU7_868,                  /*!< Russia */

    LoRaWAN_FreqBand_EU_868_RX2_SF9    = LORAWAN_BAND_EU_868_RX2_SF9,           /*!< alternative EU band, using SF9 for 2nd RX window */
    LoRaWAN_FreqBand_IN_865_RX2_SF8    = LORAWAN_BAND_IN_865_RX2_SF8,           /*!< alternative IN band, using SF8 for 2nd RX window */

    LoRaWAN_FreqBand_WW2G4             = GLOBALLINK24_BAND_WW_2G4,              /*!< proprietary extension for emulating LoRaWAN on 2.4 GHz */
    LoRaWAN_FreqBand_WW2G4_RX2         = GLOBALLINK24_BAND_WW_2G4_RX2,          /*!< proprietary extension for emulating LoRaWAN on 2.4 GHz with RX2 window using 2.422 GHz */
} TLoRaWAN_FreqBand;


//! @cond Doxygen_Suppress
// LoRa Radio Power Level

#define LORAWAN_TX_POWER_LEVEL_MIN                  0
#define LORAWAN_TX_POWER_LEVEL_MAX                  20

#define IS_LORAWAN_TX_POWER_LEVEL(x)            ( ((x) >= LORAWAN_TX_POWER_LEVEL_MIN) && \
                                                  ((X) <= LORAWAN_TX_POWER_LEVEL_MAX) )


// LoRa Radio Power Saving Mode

#define LORAWAN_POWER_SAVING_MODE_OFF               0
#define LORAWAN_POWER_SAVING_MODE_AUTO              1

//! @endcond

/**
 * @brief Power Saving modes
 */
typedef enum TLoRaWAN_PowerSaving
{
    LoRaWAN_PowerSaving_Off = 0,                                                /*!<Power Saving is NOT used (=off) */
    LoRaWAN_PowerSaving_On  = 1,                                                /*!< automatic Power Saving is enabled (=on) */
} TLoRaWAN_PowerSaving;


//! @cond Doxygen_Suppress
// LoRaWAN Network Status

#define LORAWAN_NWK_STATUS_INACTIVE                 0
#define LORAWAN_NWK_STATUS_ACTIVE_ABP               1
#define LORAWAN_NWK_STATUS_ACTIVE_OTAA              2
#define LORAWAN_NWK_STATUS_JOINING_OTAA             3

//! @endcond

/**
 * @brief Network Status Status defines
 */
typedef enum TLoRaWAN_NwkStatus
{
    LoRaWAN_NwkStatus_Inactive     = 0,                                         /*!< Device is not active ( = no Data transmission is allowed) */
    LoRaWAN_NwkStatus_Active_ABP   = 1,                                         /*!< Device has been activated / registered via ABP procedure */
    LoRaWAN_NwkStatus_Active_OTAA  = 2,                                         /*!< Device has been activated / registered via OTAA procedure */
    LoRaWAN_NwkStatus_Joining_OTAA = 3,                                         /*!< OTAA procedure is currently in progress */
} TLoRaWAN_NwkStatus;

//! @cond Doxygen_Suppress

// LoRaWAN MAC Commands

#define LORAWAN_MAC_DATA_SERVICE_TYPE_U_DATA        0
#define LORAWAN_MAC_DATA_SERVICE_TYPE_C_DATA        1

//! @endcond


//! @cond Doxygen_Suppress
/*
 * Header MAC command capatity
 */
#define LORAWAN_HEADER_MAC_CMD_CAP_MIN              0
#define LORAWAN_HEADER_MAC_CMD_CAP_MAX              15

//! @endcond

//------------------------------------------------------------------------------
//
// misc. defines
//
//------------------------------------------------------------------------------
//! @cond Doxygen_Suppress

#define WiMODLORAWAN_NWK_SESSION_KEY_LEN            16      // 128 bit
#define WiMODLORAWAN_APP_SESSION_KEY_LEN            16      // 128 bit

#define WiMODLORAWAN_DEV_EUI_LEN                    8       // 64 bit
#define WiMODLORAWAN_APP_EUI_LEN                    8       // 64 bit
#define WiMODLORAWAN_APP_KEY_LEN                    16      // 128 bit

#define WiMODLORAWAN_MAC_CMD_PAYLOAD_LENGTH         30

//! @endcond

//------------------------------------------------------------------------------
//
// structures
//
//------------------------------------------------------------------------------

/**
 * @brief Activation data structure for LoRa network activation
 */
typedef struct TWiMODLORAWAN_ActivateDeviceData
{
    UINT32      DeviceAddress;                                                  /*!< address of the WiMOD within the LoRa network (pre-set for ABP; set by nwk if OTAA  was used*/
    UINT8       NwkSKey[WiMODLORAWAN_NWK_SESSION_KEY_LEN];                      /*!< secret network session key (128bit) */
    UINT8       AppSKey[WiMODLORAWAN_NWK_SESSION_KEY_LEN];                      /*!< secret application session key (128bit) */
} TWiMODLORAWAN_ActivateDeviceData;


/**
 * @brief JOIN data structure for LoRa network activation
 */
typedef struct TWiMODLORAWAN_JoinParams
{
    UINT8       AppEUI[WiMODLORAWAN_APP_EUI_LEN];                               /*!< application EUI (64bit) */
    UINT8       AppKey[WiMODLORAWAN_APP_KEY_LEN];                               /*!< application key (128it) */
} TWiMODLORAWAN_JoinParams;


/**
 * @brief Optional TX information flag
 */
typedef enum TWiMODLORAWAN_OptIndInfos
{
    LORAWAN_OPT_TX_IND_INFOS_NOT_AVAILABLE = 0,                                 /*!< no TX informations attached / available */
    LORAWAN_OPT_TX_IND_INFOS_EXCL_PKT_CNT  = 1,                                 /*!< ext. infos available, but without packet counter */
    LORAWAN_OPT_TX_IND_INFOS_INCL_PKT_CNT  = 2,                                 /*!< ext. infos available inclusive packet counter */
} TWiMODLORAWAN_OptIndInfos;



/**
 * @brief Activation data structure for LoRa network activation
 */
typedef struct TWiMODLORAWAN_TxIndData
{
    TWiMODLORAWAN_OptIndInfos      FieldAvailability;                           /*!< indicator flag for optional infos */

    UINT8                   StatusFormat;                                       /*!< Status/Format field of reponse packet from WiMOD @see TLoRaWanTxDataIndStatusFormat */
    // optional - see StatusFormat code
    UINT8                   ChannelIndex;                                       /*!< used channel index (@see TLoRaWAN_Channel_* definitions)  */
    // optional - see StatusFormat code
    UINT8                   DataRateIndex;                                      /*!< used data rate index (@see TLoRaWANDataRate*)  */
    // optional - see StatusFormat code
    UINT8                   NumTxPackets;                                       /*!< number of TX packet (for current operation; can be used as retransmission counter) */
    // new in V1.14 optional
    UINT8                   PowerLevel;                                         /*!< transmit power in dBm */
    // new in V1.14 optional
    UINT32                  RfMsgAirtime;                                       /*!< airtime in ms of packet transmission */
} TWiMODLORAWAN_TxIndData;

/*
 * Number of payload bytes for a user defined message
 *
 * Note: the actual number of payload bytes can be less, if the air frame
 * contains a list of MAC commands.
 */
//! @cond Doxygen_Suppress
#define WiMODLORAWAN_APP_PAYLOAD_LEN                128
//! @endcond

/**
 * @brief Structure containing all data necessary to transmitting a data message
 */
typedef struct TWiMODLORAWAN_TX_Data
{
    UINT8       Port;                                                           /*!< target LoRaWAN port number */
    UINT8       Length;                                                         /*!< Length of the payload data */
    UINT8       Payload[WiMODLORAWAN_APP_PAYLOAD_LEN];                          /*!< (user) payload */
} TWiMODLORAWAN_TX_Data;


/**
 * @brief Structure containing RX (user) payload data
 */
typedef struct TWiMODLORAWAN_RX_Data
{
    UINT8       StatusFormat;                                                   /*!< Status/Format Field for this message */
    UINT8       Port;                                                           /*!< (Target) LoRaWAN port */
    UINT8       Length;                                                         /*!< Length of the used (user) payload field */
    UINT8       Payload[WiMODLORAWAN_APP_PAYLOAD_LEN];                          /*!< payload data */

    // optional RX info
    UINT8       ChannelIndex;                                                   /*!< used channel index (@see TLoRaWAN_Channel_* definitions)*/
    UINT8       DataRateIndex;                                                  /*!< used data rate index (@see TLoRaWANDataRate*)  */
    INT8        RSSI;                                                           /*!< RSSI value for the received packet */
    INT8        SNR;                                                            /*!< SNR value for the received packet  */
    UINT8       RxSlot;                                                         /*!< number of the rx slot that contained the messsage */
    bool        OptionalInfoAvaiable;                                           /*!< are the optional value fields available */
} TWiMODLORAWAN_RX_Data;


/**
 * @brief Structure containing a received MAC command
 */
typedef struct TWiMODLORAWAN_RX_MacCmdData
{
    UINT8       Length;                                                         /*!< length of the MAC command / data */
    UINT8       MacCmdData[WiMODLORAWAN_APP_PAYLOAD_LEN];                       /*!< MAC command / data */

    // optional RX info
    UINT8       ChannelIndex;                                                   /*!< used channel index (@see TLoRaWAN_Channel_* definitions)*/
    UINT8       DataRateIndex;                                                  /*!< used data rate index (@see TLoRaWANDataRate*)  */
    INT8        RSSI;                                                           /*!< RSSI value for the received packet */
    INT8        SNR;                                                            /*!< SNR value for the received packet  */
    UINT8       RxSlot;                                                         /*!< number of the rx slot that contained the messsage */
    bool        OptionalInfoAvaiable;                                           /*!< are the optional value fields available */
} TWiMODLORAWAN_RX_MacCmdData;

/**
 * @brief Structure containing data of the joined network indication
 */
typedef struct TWiMODLORAWAN_RX_JoinedNwkData
{
    UINT8       StatusFormat;                                                   /*!< Status/Format Field for this message */
    UINT32      DeviceAddress;                                                  /*!< (new) address for this device within network */

    // optional RX info
    UINT8       ChannelIndex;                                                   /*!< used channel index (@see TLoRaWAN_Channel_* definitions)*/
    UINT8       DataRateIndex;                                                  /*!< used data rate index (@see)  */
    INT8        RSSI;                                                           /*!< RSSI value for the received packet */
    INT8        SNR;                                                            /*!< SNR value for the received packet  */
    UINT8       RxSlot;                                                         /*!< number of the rx slot that contained the messsage */
    bool        OptionalInfoAvaiable;                                           /*!< are the optional value fields available */
} TWiMODLORAWAN_RX_JoinedNwkData;

/**
 * @brief Radio stack configuration related to to LoRaWAN specific options.
 */
typedef struct TWiMODLORAWAN_RadioStackConfig
{
    UINT8       DataRateIndex;                                                  /*!< data rate to use (@see TLoRaWANDataRate*) */
    UINT8       TXPowerLevel;                                                   /*!< default / initial power level settings to be used */
    UINT8       Options;                                                        /*!< misc Options (see LORAWAN_STK_OPTION defines) */
    UINT8       PowerSavingMode;                                                /*!< set automatic power saving mode */
    UINT8       Retransmissions;                                                /*!< max number of retransmissions (for C-Data) to use */
    UINT8       BandIndex;                                                      /*!< Frequency band to use (@see TLoRaWAN_FreqBand) */
    UINT8       HeaderMacCmdCapacity;                                           /*!< Header MAC Cmd Capatity (size in Byte) */

    // US915 Firmware only
    UINT8       SubBandMask1;                                                   /*!< Sub-band selection for 125 kHz bandwith channels */
    UINT8       SubBandMask2;                                                   /*!< Sub-band selection for 500 kHz bandwith channels */

    // optional for SetRadioStack cmd
    UINT8       WrongParamErrCode;                                              /*!< error code in case of a wrong parameter (@see LORAWAN_STK_PARAM_ERR defines)  */

} TWiMODLORAWAN_RadioStackConfig;

/**
 * @brief LoRaWAN MAC command structure
 */
typedef struct TWiMODLORAWAN_MacCmd
{
    UINT8       DataServiceType;                                                /*!< Data service type */ // see LORAWAN_MAC_DATA_SERVICE_TYPE
    UINT8       MacCmdID;                                                       /*!< MAC command ID    */ // see LoRaWAN Spec for command IDs
    UINT8       Payload[WiMODLORAWAN_MAC_CMD_PAYLOAD_LENGTH];                   /*!< payload for MAC command */
    UINT8       Length;                                                         /*!< length of payload */
} TWiMODLORAWAN_MacCmd;


/**
 * @brief Data structure for received Acknowledgment data
 */
typedef struct TWiMODLORAWAN_RX_ACK_Data
{
    UINT8       StatusFormat;                                                   /*!< Status/Format Field for this message */

    // optional RX info
    UINT8       ChannelIndex;                                                   /*!< used channel index (@see TLoRaWAN_Channel_* definitions)*/
    UINT8       DataRateIndex;                                                  /*!< used data rate index (@see TLoRaWANDataRate*)  */
    INT8        RSSI;                                                           /*!< RSSI value for the received packet */
    INT8        SNR;                                                            /*!< SNR value for the received packet  */
    UINT8       RxSlot;                                                         /*!< number of the rx slot that contained the messsage */
    bool        OptionalInfoAvaiable;                                           /*!< are the optional value fields available */
} TWiMODLORAWAN_RX_ACK_Data;


/**
 * @brief Structure containing all data about LoRaWAN network status
 */
typedef struct TWiMODLORAWAN_NwkStatus_Data
{
    UINT8       NetworkStatus;                                                  /*!< Current NetworkStatus (@see TLoRaWAN_NwkStatus) */
    UINT32      DeviceAddress;                                                  /*!< 32bit DeviceAddress of module in LoRa network */
    UINT8       DataRateIndex;                                                  /*!< used data rate index (@see TLoRaWANDataRate*)  */
    UINT8       PowerLevel;                                                     /*!< transmit power in dBm */
    UINT8       MaxPayloadSize;                                                 /*!< maximum number of bytes allowed in payload field */
} TWiMODLORAWAN_NwkStatus_Data;


//! @cond Doxygen_Suppress

/*
 * Error Code used by NoData Indication
 */
#define LORAWAN_NODATA_ERR_WRONG_MTYPE                  (0x01 << 0)
#define LORAWAN_NODATA_ERR_WRONG_DEVICE_ADDRESS         (0x01 << 1)
#define LORAWAN_NODATA_ERR_WRONG_MIC                    (0x01 << 2)
#define LORAWAN_NODATA_ERR_UNEXPECTED_FCNT              (0x01 << 3)
#define LORAWAN_NODATA_ERR_WRONG_MAC_CMD                (0x01 << 4)
#define LORAWAN_NODATA_ERR_WRONG_DOWNLINK               (0x01 << 5)
#define LORAWAN_NODATA_ERR_EXPECTED_ACK_MISSING         (0x01 << 6)

//! @endcond

/**
 * @brief Structure containing infos of a NoData Indication message
 */
typedef struct TWiMODLORAWAN_NoData_Data
{
	UINT8       StatusFormat;                                                   /*!< Status/Format Field for this message */

	// optional info
	UINT8       ErrorCode;                                                      /*!< ErrorCode, see LORAWAN_NODATA_ERR for meaning  */

    bool        OptionalInfoAvaiable;                                           /*!< are the optional value fields available */

} TWiMODLORAWAN_NoData_Data;


/**
 * @brief data structure for supported bands
 */
typedef struct TWiMODLORAWAN_SupportedBands
{
	UINT8       NumOfEntries;                                                   /*!< number of entries in this data structure */
    UINT8       BandIndex[(WiMODLORAWAN_APP_PAYLOAD_LEN / 2)] ;                 /*!< BandIndex */
    UINT8       MaxEIRP[(WiMODLORAWAN_APP_PAYLOAD_LEN / 2)];                    /*!< Max EIRP for band */
} TWiMODLORAWAN_SupportedBands;


/**
 * @brief data structure for TX Power Limits
 */
typedef struct TWiMODLORAWAN_TxPwrLimitConfig
{
	UINT8       NumOfEntries;                                                   /*!< number of entries in this data structure */
    UINT8       SubBandIndex[(WiMODLORAWAN_APP_PAYLOAD_LEN / 3)];               /*!< SubBandIndex */
    UINT8       TxPwrLimitFlag[(WiMODLORAWAN_APP_PAYLOAD_LEN / 3)];             /*!< TX Power Limit Flag*/
    UINT8       TxPwrLimitValue[(WiMODLORAWAN_APP_PAYLOAD_LEN / 3)];            /*!< TX Power Limit Value*/

    // only uses as response code for set operation
    UINT8       WrongParamErrCode;                                              /*!< Optional error code for set operation (@see LORAWAN_TX_PWR_LIMIT_ERR defines) */
} TWiMODLORAWAN_TxPwrLimitConfig;


//! @cond Doxygen_Suppress

/*
 * LinkAdrReq Options
 */
#define LORAWAN_LINKADR_CFG_OPTION_LORAWAN_V102     0
#define LORAWAN_LINKADR_CFG_OPTION_SEMTECH          1
#define LORAWAN_LINKADR_CFG_OPTION_KPN_ACTILITY     2

//! @endcond

//! @cond Doxygen_Suppress

/**
 * @brief LinkAdr behavior options
 */
typedef enum TWiMODLORAWAN_LinkAdrReqConfig
{
	LinkAdrCfg_Option_LoRaWAN_V1_0_2            = 0,                            /*!< vanilla LoRaWAN V1.0.2 */
	LinkAdrCfg_Option_Semtech_Proposal          = 1,                            /*!< Semtech proposal       */
	LinkAdrCfg_Option_KPN_Actility_Proposal     = 2,                            /*!< KPN / Actility proposal*/
} TWiMODLORAWAN_LinkAdrReqConfig;

//! @endcond

#endif /* ARDUINO_WIMOD_SAP_LORAWAN_IDS_H_ */
