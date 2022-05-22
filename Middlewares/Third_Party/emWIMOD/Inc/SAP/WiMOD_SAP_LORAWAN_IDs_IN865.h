//------------------------------------------------------------------------------
//! @file WiMOD_SAP_LORAWAN_IDs_IN865.h
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
 * THIS IS AN EXAMPLE IMPLEMENTATION ACCORDING THE THE HCI SPEC: V1.22 for IN865 (V0.1)
 * FOR FIRMWARE: LoRaWAN_EndNode_Modem
 *
 * SEE FILE: WiMOD_LoRaWAN_EndNode_Modem_IN865_HCI_Spec_V0_1.pdf
 * for detailed information
 *
 */


#ifndef ARDUINO_SAP_WIMOD_SAP_LORAWAN_IDS_IN865_H_

//------------------------------------------------------------------------------
//
// Section Includes Files
//
//------------------------------------------------------------------------------

//! @cond Doxygen_Suppress

#define WIMOD_LORAWAN_IN865



// LoRa Radio Band Index
#define LORAWAN_BAND_IN_865                     3
#define LORAWAN_BAND_IN_865_RX2_SF8             131

// LoRa Data Rate Index

#define LORAWAN_DATA_RATE_IN865_LORA_SF12_125KHZ      0
#define LORAWAN_DATA_RATE_IN865_LORA_SF11_125KHZ      1
#define LORAWAN_DATA_RATE_IN865_LORA_SF10_125KHZ      2
#define LORAWAN_DATA_RATE_IN865_LORA_SF9_125KHZ       3
#define LORAWAN_DATA_RATE_IN865_LORA_SF8_125KHZ       4
#define LORAWAN_DATA_RATE_IN865_LORA_SF7_125KHZ       5
#define LORAWAN_DATA_RATE_IN865_LORA_FSK              7


#define IS_LORAWAN_IN865_DATA_RATE(x)                 ( ((x) == LORAWAN_DATA_RATE_IN865_LORA_SF12_125KHZ ) || \
                                                        ((x) == LORAWAN_DATA_RATE_IN865_LORA_SF11_125KHZ  ) || \
                                                        ((x) == LORAWAN_DATA_RATE_IN865_LORA_SF10_125KHZ  ) || \
                                                        ((x) == LORAWAN_DATA_RATE_IN865_LORA_SF9_125KHZ  ) || \
                                                        ((x) == LORAWAN_DATA_RATE_IN865_LORA_SF8_125KHZ) || \
                                                        ((x) == LORAWAN_DATA_RATE_IN865_LORA_SF7_125KHZ ) || \
                                                        ((x) == LORAWAN_DATA_RATE_IN865_LORA_FSK  ) );

//! @endcond


/**
 * @brief LoRa data rate definitions for India 865 MHz Band
 */
typedef enum TLoRaWANDataRateIN865
{
    LoRaWAN_DataRate_IN865_LoRa_SF12_125kHz = 0,                                /*!< LoRa Spreading factor 12 using 125kHz bandwidth */
    LoRaWAN_DataRate_IN865_LoRa_SF11_125kHz,                                    /*!< LoRa Spreading factor 11 using 125kHz bandwidth */
    LoRaWAN_DataRate_IN865_LoRa_SF10_125kHz,                                    /*!< LoRa Spreading factor 10 using 125kHz bandwidth */
    LoRaWAN_DataRate_IN865_LoRa_SF9_125kHz,                                     /*!< LoRa Spreading factor  9 using 125kHz bandwidth */
    LoRaWAN_DataRate_IN865_LoRa_SF8_125kHz,                                     /*!< LoRa Spreading factor  8 using 125kHz bandwidth */
    LoRaWAN_DataRate_IN865_LoRa_SF7_125kHz,                                     /*!< LoRa Spreading factor  7 using 125kHz bandwidth */
    LoRaWAN_DataRate_IN865_LoRa_FSK = 7,                                        /*!< FSK modulation; LoRa settings not applicable    */
} TLoRaWANDataRateIN865;


//! @cond Doxygen_Suppress

// LoRa Channel Index ( India )

#define LORAWAN_CH_IN_865_062_5_MHZ                0
#define LORAWAN_CH_IN_865_402_5_MHZ                1
#define LORAWAN_CH_IN_865_985_MHZ                  2
#define LORAWAN_CH_IN_866_550_MHZ                  128

#define IS_LORAWAN_CH_IN(x)                       ( ((x) == LORAWAN_CH_IN_865_062_5_MHZ) || \
                                                  ((x) == LORAWAN_CH_IN_865_402_5_MHZ) || \
                                                  ((x) == LORAWAN_CH_IN_865_985_MHZ) || \
                                                  ((x) == LORAWAN_CH_IN_866_550_MHZ) )

//! @endcond

/**
 * @brief Channel definition for India
 */
typedef enum TLoRaWAN_Channel_India
{
    LoRaWAN_Channel_India_865_062_5_Mhz   = 0,                                  /*!< carrier frequency of 865.062 500 MHz */
    LoRaWAN_Channel_India_865_402_5_Mhz   = 1,                                  /*!< carrier frequency of 865.402 500 MHz */
    LoRaWAN_Channel_India_865_985_Mhz     = 2,                                  /*!< carrier frequency of 865.985 MHz */
    LoRaWAN_Channel_India_866_550_Mhz     = 128,                                /*!< carrier frequency of 866.550 MHz */
} TLoRaWAN_Channel_India;

//! @cond Doxygen_Suppress
// LoRa Channel Index ( India RX2: SF8)
#define LORAWAN_CH_IN2_865_062_5_MHZ                0
#define LORAWAN_CH_IN2_865_402_5_MHZ                1
#define LORAWAN_CH_IN2_865_985_MHZ                  2
#define LORAWAN_CH_IN2_866_550_MHZ                  128

#define IS_LORAWAN_CH_IN2(x)                       ( ((x) == LORAWAN_CH_IN2_865_062_5_MHZ) || \
                                                  ((x) == LORAWAN_CH_IN2_865_402_5_MHZ) || \
                                                  ((x) == LORAWAN_CH_IN2_865_985_MHZ) || \
                                                  ((x) == LORAWAN_CH_IN2_866_550_MHZ) )

//! @endcond

/**
 * @brief Channel definition for India
 */
typedef enum TLoRaWAN_Channel_IndiaTwo
{
    LoRaWAN_Channel_India2_865_062_5_Mhz   = 0,                                 /*!< carrier frequency of 865.062 500 MHz */
    LoRaWAN_Channel_India2_865_402_5_Mhz   = 1,                                 /*!< carrier frequency of 865.402 500 MHz */
    LoRaWAN_Channel_India2_865_985_Mhz     = 2,                                 /*!< carrier frequency of 865.985 MHz */
    LoRaWAN_Channel_India2_866_550_Mhz     = 128,                               /*!< carrier frequency of 866.550 MHz */
} TLoRaWAN_Channel_IndiaTwo;


#endif /* ARDUINO_SAP_WIMOD_SAP_LORAWAN_IDS_IN865_H_ */
