//------------------------------------------------------------------------------
//! @file WiMOD_SAP_GLOBALLINK24_IDs_WW2G4.h
//! @ingroup WiMOD_SAP_GLOBALLINK24
//! <!------------------------------------------------------------------------->
//! @brief Supporting IDs and definitions for the GlobalLink24 ServiceAccessPoint

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


/*
 * THIS IS AN EXAMPLE IMPLEMENTATION ACCORDING THE THE HCI SPEC: V0.01 for WW2G4 (V0.1)
 * FOR FIRMWARE: GlobalLink24_EndNode_Modem
 *
 * SEE FILE: WiMOD_GlobalLink24_EndNode_Modem_WW2G4_HCI_Spec_V0_1.pdf
 * for detailed information
 *
 */


#ifndef ARDUINO_SAP_WIMOD_SAP_GLOBALLINK24_IDS_WW2G4_H_

//------------------------------------------------------------------------------
//
// Section Includes Files
//
//------------------------------------------------------------------------------

//! @cond Doxygen_Suppress

#define WIMOD_GLOBALLINK24_WW2G4



// LoRa Radio Band Index
#define GLOBALLINK24_BAND_WW_2G4                         48U
#define GLOBALLINK24_BAND_WW_2G4_RX2                     176U //   (RX2: 2.422 GHz)
// LoRa Data Rate Index

#define GLOBALLINK24_DATA_RATE_WW2G4_LORA_SF12_812KHZ      0U
#define GLOBALLINK24_DATA_RATE_WW2G4_LORA_SF11_812KHZ      1U
#define GLOBALLINK24_DATA_RATE_WW2G4_LORA_SF10_812KHZ      2U
#define GLOBALLINK24_DATA_RATE_WW2G4_LORA_SF9_812KHZ       3U
#define GLOBALLINK24_DATA_RATE_WW2G4_LORA_SF8_812KHZ       4U
#define GLOBALLINK24_DATA_RATE_WW2G4_LORA_SF7_812KHZ       5U
#define GLOBALLINK24_DATA_RATE_WW2G4_LORA_SF6_812KHZ       6U
#define GLOBALLINK24_DATA_RATE_WW2G4_LORA_SF5_812KHZ       7U


#define IS_GLOBALLINK24_WW2G4_DATA_RATE(x)                 ( ((x) == GLOBALLINK24_DATA_RATE_WW2G4_LORA_SF12_812KHZ ) || \
														((x) == GLOBALLINK24_DATA_RATE_WW2G4_LORA_SF11_812KHZ  ) || \
														((x) == GLOBALLINK24_DATA_RATE_WW2G4_LORA_SF10_812KHZ  ) || \
														((x) == GLOBALLINK24_DATA_RATE_WW2G4_LORA_SF9_812KHZ  ) || \
														((x) == GLOBALLINK24_DATA_RATE_WW2G4_LORA_SF8_812KHZ) || \
														((x) == GLOBALLINK24_DATA_RATE_WW2G4_LORA_SF7_812KHZ ) || \
                                                        ((x) == GLOBALLINK24_DATA_RATE_WW2G4_LORA_SF6_812KHZ ) || \
														((x) == GLOBALLINK24_DATA_RATE_WW2G4_LORA_SF5_812KHZ  ) );

//! @endcond


/**
 * @brief LoRa data rate definitions for WW 2G4 MHz Band
 */
typedef enum TGlobalLink24DataRateWW2G4
{
    LoRaWAN_DataRate_WW2G4_LoRa_SF12_812kHz = 0,                                /*!< LoRa Spreading factor 12 using 812kHz bandwidth */
    LoRaWAN_DataRate_WW2G4_LoRa_SF11_812kHz,                                    /*!< LoRa Spreading factor 11 using 812kHz bandwidth */
    LoRaWAN_DataRate_WW2G4_LoRa_SF10_812kHz,                                    /*!< LoRa Spreading factor 10 using 812kHz bandwidth */
    LoRaWAN_DataRate_WW2G4_LoRa_SF9_812kHz,                                     /*!< LoRa Spreading factor  9 using 812kHz bandwidth */
    LoRaWAN_DataRate_WW2G4_LoRa_SF8_812kHz,                                     /*!< LoRa Spreading factor  8 using 812kHz bandwidth */
    LoRaWAN_DataRate_WW2G4_LoRa_SF7_812kHz,                                     /*!< LoRa Spreading factor  7 using 812kHz bandwidth */
    LoRaWAN_DataRate_WW2G4_LoRa_SF6_812kHz,                                     /*!< LoRa Spreading factor  6 using 812kHz bandwidth */
    LoRaWAN_DataRate_WW2G4_LoRa_SF5_812kHz,                                     /*!< LoRa Spreading factor  5 using 812kHz bandwidth */
} TGlobalLink24DataRateWW2G4;


//! @cond Doxygen_Suppress

// LoRa Channel Index ( WW2G4 )

#define GLOBALLINK24_CH_WW_2403_MHZ                0
#define GLOBALLINK24_CH_WW_2425_MHZ                1
#define GLOBALLINK24_CH_WW_2479_MHZ                2
#define GLOBALLINK24_CH_WW_2423_MHZ                128

#define IS_GLOBALLINK24_CH_IN(x)                       ( ((x) == GLOBALLINK24_CH_WW_2403_MHZ) || \
                                                  ((x) == GLOBALLINK24_CH_WW_2425_MHZ) || \
                                                  ((x) == GLOBALLINK24_CH_WW_2479_MHZ) || \
                                                  ((x) == GLOBALLINK24_CH_WW_2423_MHZ) )

//! @endcond

/**
 * @brief Channel definition for WW2G4
 */
typedef enum TGlobalLink24_Channel_WW2G4
{
    GlobalLink24_Channel_WW_2403_Mhz   = 0,                                          /*!< carrier frequency of 2403 MHz */
    GlobalLink24_Channel_WW_2425_Mhz   = 1,                                          /*!< carrier frequency of 2425 MHz */
    GlobalLink24_Channel_WW_2479_Mhz   = 2,                                          /*!< carrier frequency of 2479 MHz */
    GlobalLink24_Channel_WW_2423_Mhz   = 128,                                        /*!< carrier frequency of 2423 MHz */
} TLoRaWAN_Channel_WW2G4;



#endif /* ARDUINO_SAP_WIMOD_SAP_GLOBALLINK24_IDS_WW2G4_H_ */
