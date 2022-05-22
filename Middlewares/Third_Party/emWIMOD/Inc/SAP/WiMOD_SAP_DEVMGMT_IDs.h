//------------------------------------------------------------------------------
//! @file WiMOD_SAP_DEVMGMT_IDs.h
//! @ingroup WiMOD_SAP_DEVMGMT
//! <!------------------------------------------------------------------------->
//! @brief Supporting IDs and definitions for the DeviceManagement ServiceAccessPoint
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
 *
 * SEE FILE: WiMOD_LR_Base_HCI_Spec_V1_8.pdf for detailed information
 */


#ifndef ARDUINO_WIMOD_SAP_DEVMGMT_IDS_H_
#define ARDUINO_WIMOD_SAP_DEVMGMT_IDS_H_

//------------------------------------------------------------------------------
//
// Section Includes Files
//
//------------------------------------------------------------------------------

#include "../utils/WMDefs.h"


//------------------------------------------------------------------------------
//
// Service Access Point Identifier
//
//------------------------------------------------------------------------------

/** SAP ID for the DevMmgt service */
#define DEVMGMT_SAP_ID                      0x01



//------------------------------------------------------------------------------
//
//  Device Management Message Identifier
//
//------------------------------------------------------------------------------
//! @cond Doxygen_Suppress

// Status Codes
#define DEVMGMT_STATUS_OK                   0x00
#define DEVMGMT_STATUS_ERROR                0x01
#define DEVMGMT_STATUS_CMD_NOT_SUPPORTED    0x02
#define DEVMGMT_STATUS_WRONG_PARAMETER      0x03
#define DEVMGMT_STATUS_WRONG_DEVICE_MODE    0x04
#define DEVMGMT_STATUS_DEVICE_BUSY          0x06

// Message IDs
#define DEVMGMT_MSG_PING_REQ                0x01
#define DEVMGMT_MSG_PING_RSP                0x02

#define DEVMGMT_MSG_GET_DEVICEINFO_REQ      0x03
#define DEVMGMT_MSG_GET_DEVICEINFO_RSP      0x04

#define DEVMGMT_MSG_GET_FW_VERSION_REQ      0x05
#define DEVMGMT_MSG_GET_FW_VERSION_RSP      0x06

#define DEVMGMT_MSG_RESET_REQ               0x07
#define DEVMGMT_MSG_RESET_RSP               0x08

#define DEVMGMT_MSG_SET_OPMODE_REQ          0x09
#define DEVMGMT_MSG_SET_OPMODE_RSP          0x0A

#define DEVMGMT_MSG_GET_OPMODE_REQ          0x0B
#define DEVMGMT_MSG_GET_OPMODE_RSP          0x0C

#define DEVMGMT_MSG_SET_RTC_REQ             0x0D
#define DEVMGMT_MSG_SET_RTC_RSP             0x0E
#define DEVMGMT_MSG_GET_RTC_REQ             0x0F
#define DEVMGMT_MSG_GET_RTC_RSP             0x10

#define DEVMGMT_MSG_SET_RADIO_CONFIG_REQ    0x11
#define DEVMGMT_MSG_SET_RADIO_CONFIG_RSP    0x12
#define DEVMGMT_MSG_GET_RADIO_CONFIG_REQ    0x13
#define DEVMGMT_MSG_GET_RADIO_CONFIG_RSP    0x14

#define DEVMGMT_MSG_RESET_RADIO_CONFIG_REQ  0x15
#define DEVMGMT_MSG_RESET_RADIO_CONFIG_RSP  0x16

#define DEVMGMT_MSG_GET_SYSTEM_STATUS_REQ   0x17
#define DEVMGMT_MSG_GET_SYSTEM_STATUS_RSP   0x18

#define DEVMGMT_MSG_SET_RADIO_MODE_REQ      0x19
#define DEVMGMT_MSG_SET_RADIO_MODE_RSP      0x1A

#define DEVMGMT_MSG_SET_PSV_MODE_REQ        0x1B
#define DEVMGMT_MSG_SET_PSV_MODE_RSP        0x1C

#define DEVMGMT_MSG_POWER_UP_IND            0x20

#define DEVMGMT_MSG_SET_AES_KEY_REQ         0x21
#define DEVMGMT_MSG_SET_AES_KEY_RSP         0x22

#define DEVMGMT_MSG_GET_AES_KEY_REQ         0x23
#define DEVMGMT_MSG_GET_AES_KEY_RSP         0x24

#define DEVMGMT_MSG_SET_RTC_ALARM_REQ 		0x31
#define DEVMGMT_MSG_SET_RTC_ALARM_RSP 		0x32

#define DEVMGMT_MSG_CLEAR_RTC_ALARM_REQ 	0x33
#define DEVMGMT_MSG_CLEAR_RTC_ALARM_RSP 	0x34

#define DEVMGMT_MSG_GET_RTC_ALARM_REQ 		0x35
#define DEVMGMT_MSG_GET_RTC_ALARM_RSP 		0x36

#define DEVMGMT_MSG_RTC_ALARM_IND 			0x38


// new Message IDs (only part of LR-BASE PLUS & LoRaWAN EndNode FW)

//! @cond Doxygen_Suppress
#define DEVMGMT_MSG_SET_HCI_CFG_REQ         0x41
#define DEVMGMT_MSG_SET_HCI_CFG_RSP         0x42
#define DEVMGMT_MSG_GET_HCI_CFG_REQ         0x43
#define DEVMGMT_MSG_GET_HCI_CFG_RSP         0x44
//! @endcond



//! @endcond
//------------------------------------------------------------------------------
//
// misc. defines
//
//------------------------------------------------------------------------------
//! @cond Doxygen_Suppress

#define DEVMGMT_AES_KEY_LEN                         16      // 128 bit

//! @endcond

//------------------------------------------------------------------------------
//
// structures
//
//------------------------------------------------------------------------------
/**
 * @brief Structure containing basic information about the WiMOD device
 */
typedef struct TWiMODLR_DevMgmt_DevInfo
{
    UINT8       Status;                                                         /*!< status flag; indicates if other values are vaild */
    UINT8       ModuleType;                                                     /*!< type of WiMOD module */
    UINT16      DevAdr;                                                         /*!< device address; not to be mixed up with the LoRaWAN device address */
    UINT8       GroupAdr;                                                       /*!< group address for radio communication */
    UINT32      DevID;                                                          /*!< the ID of the WiMOD */
} TWiMODLR_DevMgmt_DevInfo;

//! @cond Doxygen_Suppress

#define WIMOD_DEVMGMT_BUILDDATE_LEN                     (10)
#define WIMOD_DEVMGMT_MAX_BUILDDATE_LEN                 (WIMOD_DEVMGMT_BUILDDATE_LEN+1)
#define WIMOD_DEVMGMT_FIRMWARE_NAME_LEN                 (40)
#define WIMOD_DEVMGMT_MAX_FIRMWARE_NAME_LEN             (WIMOD_DEVMGMT_FIRMWARE_NAME_LEN+1)

// @endcond


// WiMOD Module Types
//! @cond Doxygen_Suppress
#define WIMOD_MODULE_TYPE_IM880B						(0x98)
//! @endcond

// WiMOD DEV Address
//! @cond Doxygen_Suppress
#define WIMOD_DEV_ADDR									0x22
//! @endcond

/**
 * @brief This enum describes the WiMOD module types
 */
typedef enum TWiMOD_ModuleType
{
	ModuleType_iM880A    = 0x90,
	ModuleType_iM880A_L  = 0x92,
	ModuleType_iU880A	 = 0x93,
	ModuleType_iM880B    = 0x98,    											/*!< iM880B */
	ModuleType_iU880B    = 0x99,
	ModuleType_iM881A    = 0xA0,
	ModuleType_iU881A    = 0xA1,
	ModuleType_iM282A    = 0xB0,
	ModuleType_iMS62F    = 0xC0,
} TWiMOD_ModuleType;



/**
 * @brief Basic information about the current firmware of the WiMOD
 */
typedef struct TWiMODLR_DevMgmt_FwInfo
{
    UINT8       Status;                                                         /*!< status flag; indicates if other values are vaild */
    UINT8       FirmwareMinorVersion;                                           /*!< minor version number of the firmware */
    UINT8       FirmwareMayorVersion;                                           /*!< mayor version number of the firmware */
    UINT16      BuildCount;                                                     /*!< build count  number of the firmware */
    UINT8       BuildDateStr[WIMOD_DEVMGMT_MAX_BUILDDATE_LEN];                  /*!< human readable string containing the build date */
    UINT8       FirmwareName[WIMOD_DEVMGMT_MAX_FIRMWARE_NAME_LEN];              /*!<  human readable string containing name of the firmware */
} TWiMODLR_DevMgmt_FwInfo;


/**
 * @brief Basic information about system status of the WiMOD
 */
typedef struct TWiMODLR_DevMgmt_SystemStatus
{
    UINT8       Status;                                                         /*!< status flag; indicates if other values are vaild */
    UINT8       SysTickResolution;                                              /*!< systick interval time in ms */
    UINT32      SysTickCounter;                                                 /*!< virtual systick counter value */
    UINT32      RtcTime;                                                        /*!< 32bit encoded current RTC timestamp (@see WIMOD_RTC_* macros)*/
    UINT16      NvmStatus;                                                      /*!< Status of the internal NVM slots */
    UINT16      BatteryStatus;                                                  /*!< current voltage level of power supply in mV */
    UINT16      ExtraStatus;                                                    /*!< extra status bits */
    UINT32 		RxPackets;														/*!< number of received radio packets with CRC OK */
	UINT32 		RxAddressMatch;													/*!< number of received radio packets with CRC and Address OK */
	UINT32 		RxCRCError;														/*!< number of received radio packets with CRC Error */
	UINT32 		TxPackets;														/*!< number of transmitted radio packets */
	UINT32 		TxError;														/*!< number of not transmitted radio packets */
	UINT32 		TxMediaBusyEvents;												/*!< number of not transmitted packets due to LBT result “media busy” */
} TWiMODLR_DevMgmt_SystemStatus;


/**
 * @brief general operation mode of the radio part (only for LR-BASE)
 */
typedef enum TRadioCfg_RadioMode
{
    RadioMode_Standard = 0,                                                     /*!< normal operation mode */
    RadioMode_Reserved,                                                         /*!< DO NOT USE */
    RadioMode_Sniffer,                                                          /*!< DO NOT USE */
} TRadioCfg_RadioMode;


/**
 * @brief Modulation technique supported by the WiMOD (only for LR-BASE)
 */
typedef enum TRadioCfg_Modulation
{
    Modulation_LoRa = 0,                                                        /*!< LoRa modulation */
    Modulation_FSK,                                                             /*!< FSK modulation  */
} TRadioCfg_Modulation;

/**
 * @brief Bandwidth settings for the LoRa modulation (only for LR-BASE)
 */
typedef enum TRadioCfg_LoRaBandwidth
{
    LoRaBandwith_125kHz = 0,                                                    /*!< 125 kHz */
    LoRaBandwith_250kHz,                                                        /*!< 250 kHz */
    LoRaBandwith_500kHz                                                         /*!< 500 kHz */
} TRadioCfg_LoRaBandwidth;

/**
 * @brief SpreadingFactors for LoRa modulation (only for LR-BASE)
 */
typedef enum TRadioCfg_LoRaSpreadingFactor
{
    LoRa0_SF7 = 0,                                                              /*!< Spreading Factor 7 */
    LoRa1_SF7,                                                                  /*!< Spreading Factor 7 */
    LoRa2_SF7,                                                                  /*!< Spreading Factor 7 */
    LoRa3_SF7,                                                                  /*!< Spreading Factor 7 */
    LoRa4_SF7,                                                                  /*!< Spreading Factor 7 */
    LoRa5_SF7,                                                                  /*!< Spreading Factor 7 */
    LoRa6_SF7,                                                                  /*!< Spreading Factor 7 */
    LoRa7_SF7,                                                                  /*!< Spreading Factor 7 */
    LoRa8_SF8,                                                                  /*!< Spreading Factor 8 */
    LoRa9_SF9,                                                                  /*!< Spreading Factor 9 */
    LoRa10_SF10,                                                                /*!< Spreading Factor 10 */
    LoRa11_SF11,                                                                /*!< Spreading Factor 11 */
    LoRa12_SF12,                                                                /*!< Spreading Factor 12 */
} TRadioCfg_LoRaSpreadingFactor;

/**
 * @brief ErrorCoding settings (for LoRa modulation) (only for LR-BASE)
 */
typedef enum TRadioCfg_ErrorCoding
{
    ErrorCoding0_4_5 = 0,                                                       /*!< ErrorCoding 4/5 */
    ErrorCoding1_4_5,                                                           /*!< ErrorCoding 4/5 */
    ErrorCoding2_4_6,                                                           /*!< ErrorCoding 4/6 */
    ErrorCoding3_4_7,                                                           /*!< ErrorCoding 4/7 */
    ErrorCoding4_4_8,                                                           /*!< ErrorCoding 4/8 */
} TRadioCfg_ErrorCoding;

/**
 * @brief TxPowerLevel settings (only for LR-BASE)
 */
typedef enum TRadioCdf_TxPowerLevel
{
    TxPowerLevel_5_dBm = 5,                                                     /*!< 5 dBm */
    TxPowerLevel_6_dBm,                                                         /*!< 6 dBm */
    TxPowerLevel_7_dBm,                                                         /*!< 7 dBm */
    TxPowerLevel_8_dBm,                                                         /*!< 8 dBm */
    TxPowerLevel_9_dBm,                                                         /*!< 9 dBm */
    TxPowerLevel_10_dBm,                                                        /*!< 10 dBm */
    TxPowerLevel_11_dBm,                                                        /*!< 11 dBm */
    TxPowerLevel_12_dBm,                                                        /*!< 12 dBm */
    TxPowerLevel_13_dBm,                                                        /*!< 13 dBm */
    TxPowerLevel_14_dBm,                                                        /*!< 14 dBm */
    TxPowerLevel_15_dBm,                                                        /*!< 15 dBm */
    TxPowerLevel_16_dBm,                                                        /*!< 16 dBm */
    TxPowerLevel_17_dBm,                                                        /*!< 17 dBm */
    TxPowerLevel_18_dBm,                                                        /*!< 18 dBm */
    TxPowerLevel_19_dBm,                                                        /*!< 19 dBm */
    TxPowerLevel_20_dBm,                                                        /*!< 20 dBm */
} TRadioCfg_PowerLevel;


// bit field for TxControl field of RadioConfig structure
//! @cond Doxygen_Suppress
#define DEVMGMT_RADIO_CFG_TX_CTRL_TX_NARROW_FILTER_ON       (1 << 0)
#define DEVMGMT_RADIO_CFG_TX_CTRL_LBT_ON                    (1 << 1)
//! @endcond

/**
 * @brief RX Operation modes (only for LR-BASE)
 */
typedef enum TRadioCfg_RxControl
{
    RxCtrl_Receiver_Off = 0,                                                    /*!< Disable the RX operation completely */
    RxCtrl_Receiver_AlwaysOn,                                                   /*!< RX operation is always ON*/
    RxCtrl_Receiver_RxWindowed,                                                 /*!< RX operation is only used in defined time windows */
} TRadioCfg_RxControl;

//! @cond Doxygen_Suppress

// bit field for LedControl in RadioConfig structure
#define DEVMGMT_RADIO_CFG_LED_CTRL_LED3_RX_IND_ON           (1 << 0)
#define DEVMGMT_RADIO_CFG_LED_CTRL_LED2_TX_IND_ON           (1 << 1)
#define DEVMGMT_RADIO_CFG_LED_CTRL_LED4_ALIVE_IND_ON        (1 << 2)
#define DEVMGMT_RADIO_CFG_LED_CTRL_LED1_BUTTON_IND_ON       (1 << 3)

// bit field for Misc.Options in RadioConfig structure
#define DEVMGMT_RADIO_CFG_MISC_EXTENDED_HCI_OUTPUT_FORMAT   (1 << 0)
#define DEVMGMT_RADIO_CFG_MISC_RTC_ENABLED                  (1 << 1)
#define DEVMGMT_RADIO_CFG_MISC_HCI_TX_IND_ENABLED           (1 << 2)
#define DEVMGMT_RADIO_CFG_MISC_HCI_PWR_UP_IND_ENABLED       (1 << 3)
#define DEVMGMT_RADIO_CFG_MISC_HCI_BTN_PRESS_IND_ENABLED    (1 << 4)
#define DEVMGMT_RADIO_CFG_MISC_HCI_ENCTRYPTION_ENABLED      (1 << 5)

//! @endcond

/**
 * @brief Settings for the FSK data rate (only for LR-BASE)
 */
typedef enum TRadioCfg_FskDatarate
{
    FskDatarate_50kbps = 0,                                                     /*!< FSK modulation using 50  kbps */
    FskDatarate_100kbps,                                                        /*!< FSK modulation using 100 kbps */
    FskDatarate_250kbps,                                                        /*!< FSK modulation using 250 kbps */
} TRadioCfg_FskDatarate;


// power saving mode field of the RadioConfig structure
//! @cond Doxygen_Suppress
#define DEVMGMT_RADIO_CFG_POWER_SAVING_MODE_OFF              0
#define DEVMGMT_RADIO_CFG_POWER_SAVING_MODE_ON               1
//! @endcond


/**
 * @brief Settings general PowerSaving mode (aka. Sleep mode)
 */
typedef enum TRadioCfg_PowerSavingMode
{
    PowerSaving_Off = 0,                                                        /*!< Entering sleep mode is disabled */
    PowerSaving_On  = 1,                                                        /*!< Entering sleep mode is allowed; (may influence LED and radio RX settings */
} TRadioCfg_PowerSavingMode;

/**
 * @brief Structure containing the radio (stack) config parameters of the WiMOD (only for LR-BASE)
 */
typedef struct TWiMODLR_DevMgmt_RadioConfig
{
    // only for reading
    UINT8                           Status;                                     /*!< Status Flag of HCI get operation; only vaild if config has been requested*/

    // only for writing
    UINT8                           StoreNwmFlag;                               /*!< Store the config in NVM or RAM; 1 = store in NVM*/

    TRadioCfg_RadioMode             RadioMode;                                  /*!< Radio Mode setting */
    UINT8                           GroupAddress;                               /*!< Group address of WiMOD (for RX)*/
    UINT8                           TxGroupAddress;                             /*!< Group address to be used for TX*/
    UINT16                          DeviceAddress;                              /*!< RF device address used for RX*/
    UINT16                          TxDeviceAddress;                            /*!< RF device address used for TX*/
    TRadioCfg_Modulation            Modulation;                                 /*!< RF Modulation to be used for TX*/
    UINT8                           RfFreq_LSB;                                 /*!< lower part of the 24bit frequency register of the transceiver*/
    UINT8                           RfFreq_MID;                                 /*!< mid part of the 24bit frequency register of the transceiver*/
    UINT8                           RfFreq_MSB;                                 /*!< high part of the 24bit frequency register of the transceiver*/
    TRadioCfg_LoRaBandwidth         LoRaBandWidth;                              /*!< Bandwidth setting for the LoRa modulation */
    TRadioCfg_LoRaSpreadingFactor   LoRaSpreadingFactor;                        /*!< LoRa Spreading factor*/
    TRadioCfg_ErrorCoding           ErrorCoding;                                /*!< Error Coding */
    TRadioCfg_PowerLevel            PowerLevel;                                 /*!< Power Level setting*/
    UINT8                           TxControl;                                  /*!< Options related to TX operations */
    TRadioCfg_RxControl             RxControl;                                  /*!< Options related to RX operations*/
    UINT16                          RxWindowTime;                               /*!< Time settings related to the RX Window */
    UINT8                           LedControl;                                 /*!< control byte for the LED (of the demoboard)*/
    UINT8                           MiscOptions;                                /*!< misc options field*/
    TRadioCfg_FskDatarate           FskDatarate;                                /*!< Datarate setting for the FSK modulation*/
    TRadioCfg_PowerSavingMode       PowerSavingMode;                            /*!< PowerSaving; 1 = enable/ on; 0 = disable / off*/
    INT16                           LbtThreshold;                               /*!< ListenBeforeTalk threshold value*/
} TWiMODLR_DevMgmt_RadioConfig;

/**
 * @brief This enum describes the possible operation modes of the WiMOD (only for LR-BASE).
 */
typedef enum TWiMOD_OperationMode
{
    OperationMode_Application = 0,                                              /*!< normal application mode; full function set available */
    OperationMode_Test,                                                         /*!< test mode; only the test function set is  avaiable */
    OperationMode_Reserved2 = 2,                                                /*!< this is reserved */
    OperationMode_Customer  = 3,                                                /*!< customer mode; special config mode */
} TWiMOD_OperationMode;



// RTC alarm status
//! @cond Doxygen_Suppress
#define DEVMGMT_RTC_ALARM_STATUS_NO_ALARM_SET              0
#define DEVMGMT_RTC_ALARM_STATUS_ALARM_SET                 1
//! @endcond

/**
 * @brief This enum describes the possible RTC alarm status types
 */
typedef enum TWiMOD_RtcAlarmStatus
{
	RTC_Alarm_No_Alarm_Set = 0,													/*!< RTC Alarm feature is not active */
	RTC_Alarm_Alarm_Set,	     												/*!< RTC Alarm feature is active     */
} TWiMOD_RtcAlarmStatus;


// RTC alarm options / alarm type
//! @cond Doxygen_Suppress
#define DEVMGMT_RTC_ALARM_TYPE_SIGNLE              0
#define DEVMGMT_RTC_ALARM_TYPE_DAILY_REPEATED      1
//! @endcond

/**
 * @brief This enum describes the possible RTC alarm types
 */
typedef enum TWiMOD_RtcAlarmType
{
	RTC_Alarm_Single = 0,														/*!< Single alarm that is triggered only once */
	RTC_Alarm_DailyRepeated,													/*!< Alarm that is repeated on daily bases */
} TWiMOD_RtcAlarmType;

/**
 * @brief Structure containing the RTC Alarm config parameters of the WiMOD
 */
typedef struct TWiMODLR_DevMgmt_RtcAlarm
{
    // RX only
	TWiMOD_RtcAlarmStatus   AlarmStatus;        					            /*!< RTC alarm trigger status */

	TWiMOD_RtcAlarmType		Options;											/*!< RTC alarm type; single or repeated alarm */
	UINT8					Hour;												/*!< hour part of   alarm */
	UINT8					Minutes;											/*!< minute part of alarm */
	UINT8					Seconds;											/*!< second part of alarm */
} TWiMODLR_DevMgmt_RtcAlarm;




/**
 * @brief This enum describes the Baudrate Settings for the HCI configuration (LR-BASE PLUS & LoRaWAN)
 */
typedef enum TWiMOD_HCI_Baudrate
{
    WimodHciBaudrate_9600    = 0x00,                                            /*!<   9600 baud */
    WimodHciBaudrate_19200   = 0x01,                                            /*!<  19200 baud */
    WimodHciBaudrate_38400   = 0x02,                                            /*!<  38400 baud */
    WimodHciBaudrate_57600   = 0x03,                                            /*!<  57600 baud */
    WimodHciBaudrate_115200  = 0x04,                                            /*!< 115200 baud */
    WimodHciBaudrate_230400  = 0x05,
    WimodHciBaudrate_460800  = 0x06,
} TWiMOD_HCI_Baudrate;




/**
 * @brief HCI Configuration Parameters
 */

typedef struct TWiMODLR_DevMgmt_HciConfig
{
    // only for writing
    UINT8                           StoreNwmFlag;                               /*!< Store the config in NVM or RAM; 1 = store in NVM*/
    // only for reading
    UINT8                           Status;                                     /*!< status flag; indicates if other values are vaild */
    TWiMOD_HCI_Baudrate             BaudrateID;                                 /*!< Baudrate setting; see documentation */
    UINT16                          NumWakeUpChars;                             /*!< Number of wakeUp chars; see documentation */
    UINT8                           TxHoldTime;                                 /*!< Tx hold time in ms; see documentation */
    UINT8                           RxHoldTime;                                 /*!< Rx hold time in ms; see documentation */

} TWiMODLR_DevMgmt_HciConfig;


//------------------------------------------------------------------------------
//
// Section Macros
//
//------------------------------------------------------------------------------

/** year offset for the RTC macros */
#define WiMOD_RTC_YEAR_OFFSET              2000

/** RTC macro for extracting the "second" part out of the 32bit timestamp*/
#define WIMOD_RTC_GET_SECONDS(t)  (t & 0x3F)
/** RTC macro for extracting the "minute" part out of the 32bit timestamp*/
#define WIMOD_RTC_GET_MINUTES(t)  ((t >> 6) & 0x3F)
/** RTC macro for extracting the "month" part out of the 32bit timestamp*/
#define WIMOD_RTC_GET_MONTHS(t)   ((t >> 12) & 0x0F)
/** RTC macro for extracting the "hour" part out of the 32bit timestamp*/
#define WIMOD_RTC_GET_HOURS(t)    ((t >> 16) & 0x1F)
/** RTC macro for extracting the "day" part out of the 32bit timestamp*/
#define WIMOD_RTC_GET_DAYS(t)     ((t >> 21) & 0x1F)
/** RTC macro for extracting the "year" part out of the 32bit timestamp*/
#define WIMOD_RTC_GET_YEARS(t)    (((t >> 26) & 0x3F) + WiMOD_RTC_YEAR_OFFSET)

/** RTC macro for generating a 32 bit timestamp out of a human readable format*/
#define WIMOD_RTC_MAKE_DATETIME_U32(s,m,h,dd,mm,yy)         \
                    (((UINT32)s) | ((UINT32)m << 6)   \
                  | ((UINT32)mm << 12)                \
                  | ((UINT32)h << 16)                 \
                  | ((UINT32)dd << 21)                \
                  | ((UINT32)(yy-WiMOD_RTC_YEAR_OFFSET) << 26))



#endif /* ARDUINO_WIMOD_SAP_DEVMGMT_IDS_H_ */
