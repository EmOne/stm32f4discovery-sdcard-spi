//------------------------------------------------------------------------------
//! @file WiMOD_SAP_LORAWAN_IDs_AS923.h
//! @ingroup WiMOD_SAP_LORAWAN
//! <!------------------------------------------------------------------------->
//! @brief Supporting IDs and definitions for the LoRaWAN ServiceAccessPoint

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


/*
 * THIS IS AN EXAMPLE IMPLEMENTATION ACCORDING THE THE HCI SPEC: V1.22 for AS923 (V0.1)
 * FOR FIRMWARE: LoRaWAN_EndNode_Modem
 *
 * SEE FILE: WiMOD_LoRaWAN_EndNode_Modem_AS923_HCI_Spec_V0_1.pdf
 * for detailed information
 *
 */


#ifndef ARDUINO_SAP_WIMOD_SAP_LORAWAN_IDS_AS923_H_

//------------------------------------------------------------------------------
//
// Section Includes Files
//
//------------------------------------------------------------------------------

//! @cond Doxygen_Suppress

#define WIMOD_LORAWAN_AS923



// LoRa Radio Band Index
//#define LORAWAN_BAND_NZ_865                     4	// @deprecated removed in HCI spec V1_15
//#define LORAWAN_BAND_SG_923                     5 // @deprecated removed in HCI spec V1_15

#define LORAWAN_BAND_AS_923_BN_923              10 // Brunei                    only for iM980 module
#define LORAWAN_BAND_AS_923_KH_923              11 // Cambodia                  only for iM980 module
#define LORAWAN_BAND_AS_923_ID_923              12 // Indonesiae                only for iM980 module
#define LORAWAN_BAND_AS_923_JP_920              13 // Japan                     only for iM980 module
#define LORAWAN_BAND_AS_923_LA_920              14 // Laos                      only for iM980 module
#define LORAWAN_BAND_AS_923_LA_915              15 // New Zealand               only for iM980 module
#define LORAWAN_BAND_AS_923_SG_920              16 // Singapore                 only for iM980 module
#define LORAWAN_BAND_AS_923_TW_922              17 // Taiwan                    only for iM980 module
#define LORAWAN_BAND_AS_923_TH_920              18 // Thailand                  only for iM980 module
#define LORAWAN_BAND_AS_923_VN_920              19 // Vietnam                   only for iM980 module


// LoRa Data Rate Index

#define LORAWAN_DATA_RATE_AS923_LORA_SF12_125KHZ      0
#define LORAWAN_DATA_RATE_AS923_LORA_SF11_125KHZ      1
#define LORAWAN_DATA_RATE_AS923_LORA_SF10_125KHZ      2
#define LORAWAN_DATA_RATE_AS923_LORA_SF9_125KHZ       3
#define LORAWAN_DATA_RATE_AS923_LORA_SF8_125KHZ       4
#define LORAWAN_DATA_RATE_AS923_LORA_SF7_125KHZ       5
#define LORAWAN_DATA_RATE_AS923_LORA_SF7_250KHZ       6
#define LORAWAN_DATA_RATE_AS923_LORA_FSK              7

#define IS_LORAWAN_AS923_DATA_RATE(x)                 ( ((x) >= LORAWAN_DATA_RATE_AS923_LORA_SF12_125KHZ) && \
														((X) <= LORAWAN_DATA_RATE_AS923_LORA_FSK) )



//! @endcond

/**
 * @brief LoRa data rate definitions for Asia 923 MHz Band
 */

typedef enum TLoRaWANDataRateAS923
{
    LoRaWAN_DataRate_AS923_LoRa_SF12_125kHz = 0,                                /*!< LoRa Spreading factor 12 using 125kHz bandwidth */
    LoRaWAN_DataRate_AS923_LoRa_SF11_125kHz,                                    /*!< LoRa Spreading factor 11 using 125kHz bandwidth */
    LoRaWAN_DataRate_AS923_LoRa_SF10_125kHz,                                    /*!< LoRa Spreading factor 10 using 125kHz bandwidth */
    LoRaWAN_DataRate_AS923_LoRa_SF9_125kHz,                                     /*!< LoRa Spreading factor  9 using 125kHz bandwidth */
    LoRaWAN_DataRate_AS923_LoRa_SF8_125kHz,                                     /*!< LoRa Spreading factor  8 using 125kHz bandwidth */
    LoRaWAN_DataRate_AS923_LoRa_SF7_125kHz,                                     /*!< LoRa Spreading factor  7 using 125kHz bandwidth */
    LoRaWAN_DataRate_AS923_LoRa_SF7_250kHz,                                     /*!< LoRa Spreading factor  7 using 250kHz bandwidth */
    LoRaWAN_DataRate_AS923_LoRa_FSK,                                            /*!< FSK modulation; LoRa settings not applicable    */
} TLoRaWANDataRateAS923;



//! @cond Doxygen_Suppress

// @deprecated removed in HCI spec V1_15

//// LoRa Channel Index ( New Zealand )
//
//#define LORAWAN_CH_NZ_865_0_MHZ                    0
//#define LORAWAN_CH_NZ_865_2_MHZ                    1
//#define LORAWAN_CH_NZ_865_4_MHZ                    2
//#define LORAWAN_CH_NZ_866_2_MHZ                    3
//#define LORAWAN_CH_NZ_866_4_MHZ                    4
//#define LORAWAN_CH_NZ_866_6_MHZ                    5
//#define LORAWAN_CH_NZ_866_8_MHZ                    6
//#define LORAWAN_CH_NZ_867_0_MHZ                    7
//#define LORAWAN_CH_NZ_865_6_MHZ                    8
//#define LORAWAN_CH_NZ_865_9_MHZ                    9
//#define LORAWAN_CH_NZ_867_2_MHZ                    128
//
/*
//#define IS_LORAWAN_CH_NZ(x)                       ( ((x) == LORAWAN_CH_NZ_865_0_MHZ) || \
//                                                  ((x) == LORAWAN_CH_NZ_865_2_MHZ) || \
//                                                  ((x) == LORAWAN_CH_NZ_865_4_MHZ) || \
//                                                  ((x) == LORAWAN_CH_NZ_866_2_MHZ) || \
//                                                  ((x) == LORAWAN_CH_NZ_866_4_MHZ) || \
//                                                  ((x) == LORAWAN_CH_NZ_865_4_MHZ) || \
//                                                  ((x) == LORAWAN_CH_NZ_866_6_MHZ) || \
//                                                  ((x) == LORAWAN_CH_NZ_867_0_MHZ) || \
//                                                  ((x) == LORAWAN_CH_NZ_865_6_MHZ) || \
//                                                  ((x) == LORAWAN_CH_NZ_865_9_MHZ) || \
//                                                  ((x) == LORAWAN_CH_NZ_867_2_MHZ) )
*/
//
//! @endcond

// @deprecated removed in HCI spec V1_15
///**
// * @brief Channel definition for New Zealand
// */
//typedef enum TLoRaWAN_Channel_NewZealand
//{
//    LoRaWAN_Channel_NewZealand_865_0_Mhz   = 0,                                 /*!< carrier frequency of 865.000 MHz */
//    LoRaWAN_Channel_NewZealand_865_2_Mhz   = 1,                                 /*!< carrier frequency of 866.200 MHz */
//    LoRaWAN_Channel_NewZealand_865_4_Mhz   = 2,                                 /*!< carrier frequency of 865.400 MHz */
//    LoRaWAN_Channel_NewZealand_866_2_Mhz   = 3,                                 /*!< carrier frequency of 866.200 MHz */
//    LoRaWAN_Channel_NewZealand_866_4_Mhz   = 4,                                 /*!< carrier frequency of 866.400 MHz */
//    LoRaWAN_Channel_NewZealand_866_6_Mhz   = 5,                                 /*!< carrier frequency of 866.600 MHz */
//    LoRaWAN_Channel_NewZealand_866_8_Mhz   = 6,                                 /*!< carrier frequency of 866.800 MHz */
//    LoRaWAN_Channel_NewZealand_867_0_Mhz   = 7,                                 /*!< carrier frequency of 867.000 MHz */
//    LoRaWAN_Channel_NewZealand_865_6_Mhz   = 8,                                 /*!< carrier frequency of 865.600 MHz */
//    LoRaWAN_Channel_NewZealand_865_9_Mhz   = 9,                                 /*!< carrier frequency of 865.900 MHz */
//    LoRaWAN_Channel_NewZealand_867_2_Mhz   = 128,                               /*!< carrier frequency of 867.200 MHz */
//} TLoRaWAN_Channel_NewZealand;


//! @cond Doxygen_Suppress

// @deprecated removed in HCI spec V1_15
//// LoRa Channel Index ( Singapore )
//
//#define LORAWAN_CH_SG_923_5_MHZ                    0
//#define LORAWAN_CH_SG_923_7_MHZ                    1
//#define LORAWAN_CH_SG_923_9_MHZ                    2
//#define LORAWAN_CH_SG_924_1_MHZ                    3
//#define LORAWAN_CH_SG_924_3_MHZ                    4
//#define LORAWAN_CH_SG_924_5_MHZ                    5
//#define LORAWAN_CH_SG_924_7_MHZ                    6
//#define LORAWAN_CH_SG_924_9_MHZ                    7
//#define LORAWAN_CH_SG_923_8_MHZ                    8
//#define LORAWAN_CH_SG_922_5_MHZ                    128
//
/*
//#define IS_LORAWAN_CH_SG(x)                       ( ((x) == LORAWAN_CH_SG_923_5_MHZ) || \
//                                                  ((x) == LORAWAN_CH_SG_923_7_MHZ) || \
//                                                  ((x) == LORAWAN_CH_SG_923_9_MHZ) || \
//                                                  ((x) == LORAWAN_CH_SG_924_1_MHZ) || \
//                                                  ((x) == LORAWAN_CH_SG_924_3_MHZ) || \
//                                                  ((x) == LORAWAN_CH_SG_924_5_MHZ) || \
//                                                  ((x) == LORAWAN_CH_SG_924_7_MHZ) || \
//                                                  ((x) == LORAWAN_CH_SG_924_9_MHZ) || \
//                                                  ((x) == LORAWAN_CH_SG_923_8_MHZ) || \
//                                                  ((x) == LORAWAN_CH_SG_922_5_MHZ) )
*/
//! @endcond

// @deprecated removed in HCI spec V1_15
///**
// * @brief Channel definition for Singapore
// */
//typedef enum TLoRaWAN_Channel_Singapore
//{
//    LoRaWAN_Channel_Singapore_923_5_Mhz   = 0,                                  /*!< carrier frequency of 923.500 MHz */
//    LoRaWAN_Channel_Singapore_923_7_Mhz   = 1,                                  /*!< carrier frequency of 823.700 MHz */
//    LoRaWAN_Channel_Singapore_923_9_Mhz   = 2,                                  /*!< carrier frequency of 923.900 MHz */
//    LoRaWAN_Channel_Singapore_924_1_Mhz   = 3,                                  /*!< carrier frequency of 924.100 MHz */
//    LoRaWAN_Channel_Singapore_924_3_Mhz   = 4,                                  /*!< carrier frequency of 924.300 MHz */
//    LoRaWAN_Channel_Singapore_924_5_Mhz   = 5,                                  /*!< carrier frequency of 924.500 MHz */
//    LoRaWAN_Channel_Singapore_924_7_Mhz   = 6,                                  /*!< carrier frequency of 924.700 MHz */
//    LoRaWAN_Channel_Singapore_924_9_Mhz   = 7,                                  /*!< carrier frequency of 924.900 MHz */
//    LoRaWAN_Channel_Singapore_923_8_Mhz   = 8,                                  /*!< carrier frequency of 923.800 MHz */
//    LoRaWAN_Channel_Singapore_922_5_Mhz   = 128,                                /*!< carrier frequency of 922.500 MHz */
//} TLoRaWAN_Channel_Singapore;
//


//! @cond Doxygen_Suppress


//! @cond Doxygen_Suppress

// LoRa Channel Index ( AS923 )

#define LORAWAN_CH_AS_923_2_MHZ                    0
#define LORAWAN_CH_AS_923_4_MHZ                    1
#define LORAWAN_CH_AS_923_2_MHZ_2                  128

#define IS_LORAWAN_CH_AS(x)                       ( ((x) == LORAWAN_CH_AS_923_2_MHZ) || \
                                                  ((x) == LORAWAN_CH_AS_923_4_MHZ) || \
                                                  ((x) == LORAWAN_CH_AS_923_2_MHZ_2) )

//! @endcond




/**
 * @brief Channel definition for AS 923MHz Band
 */
typedef enum TLoRaWAN_Channel_AS
{
    LoRaWAN_Channel_AS_923_2_Mhz   = 0,                                         /*!< carrier frequency of 923.200 MHz */
    LoRaWAN_Channel_AS_923_4_Mhz   = 1,                                         /*!< carrier frequency of 923.400 MHz */
    LoRaWAN_Channel_AS_923_2_Mhz_2 = 128,                                       /*!< carrier frequency of 923.200 MHz */
} TLoRaWAN_Channel_AS;





#endif /* ARDUINO_SAP_WIMOD_SAP_LORAWAN_IDS_AS923_H_ */
