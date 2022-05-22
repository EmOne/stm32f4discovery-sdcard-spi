//------------------------------------------------------------------------------
//! @file WiMOD_SAP_LORAWAN_IDs_EU868.h
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
//! @author Frank Brey (FB), IMST
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
 * THIS IS AN EXAMPLE IMPLEMENTATION ACCORDING THE THE HCI SPEC: V1.22 for EU868 (V0.1)
 * FOR FIRMWARE: LoRaWAN_EndNode_Modem
 *
 * SEE FILE: WiMOD_LoRaWAN_EndNode_Modem_EU868_HCI_Spec_V0_1.pdf
 * for detailed information
 *
 */


#ifndef ARDUINO_SAP_WIMOD_SAP_LORAWAN_IDS_EU868_H_

//------------------------------------------------------------------------------
//
// Section Includes Files
//
//------------------------------------------------------------------------------

//! @cond Doxygen_Suppress

#define WIMOD_LORAWAN_EU868



// LoRa Radio Band Index
#define LORAWAN_BAND_EU_868                     1
#define LORAWAN_BAND_EU_868_RX2_SF9             129


// LoRa Data Rate Index

#define LORAWAN_DATA_RATE_EU868_LORA_SF12_125KHZ      0
#define LORAWAN_DATA_RATE_EU868_LORA_SF11_125KHZ      1
#define LORAWAN_DATA_RATE_EU868_LORA_SF10_125KHZ      2
#define LORAWAN_DATA_RATE_EU868_LORA_SF9_125KHZ       3
#define LORAWAN_DATA_RATE_EU868_LORA_SF8_125KHZ       4
#define LORAWAN_DATA_RATE_EU868_LORA_SF7_125KHZ       5
#define LORAWAN_DATA_RATE_EU868_LORA_SF7_250KHZ       6
#define LORAWAN_DATA_RATE_EU868_LORA_FSK              7

#define IS_LORAWAN_EU868_DATA_RATE(x)                 ( ((x) >= LORAWAN_DATA_RATE_EU868_LORA_SF12_125KHZ) && \
														((X) <= LORAWAN_DATA_RATE_EU868_LORA_FSK) )

//! @endcond


/**
 * @brief LoRa data rate definitions for EU 868 MHz Band
 */
typedef enum TLoRaWANDataRateEU868
{
    LoRaWAN_DataRate_EU868_LoRa_SF12_125kHz = 0,                                /*!< LoRa Spreading factor 12 using 125kHz bandwidth */
    LoRaWAN_DataRate_EU868_LoRa_SF11_125kHz,                                    /*!< LoRa Spreading factor 11 using 125kHz bandwidth */
    LoRaWAN_DataRate_EU868_LoRa_SF10_125kHz,                                    /*!< LoRa Spreading factor 10 using 125kHz bandwidth */
    LoRaWAN_DataRate_EU868_LoRa_SF9_125kHz,                                     /*!< LoRa Spreading factor  9 using 125kHz bandwidth */
    LoRaWAN_DataRate_EU868_LoRa_SF8_125kHz,                                     /*!< LoRa Spreading factor  8 using 125kHz bandwidth */
    LoRaWAN_DataRate_EU868_LoRa_SF7_125kHz,                                     /*!< LoRa Spreading factor  7 using 125kHz bandwidth */
    LoRaWAN_DataRate_EU868_LoRa_SF7_250kHz,                                     /*!< LoRa Spreading factor  7 using 250kHz bandwidth */
    LoRaWAN_DataRate_EU868_LoRa_FSK,                                            /*!< FSK modulation; LoRa settings not applicable    */
} TLoRaWANDataRateEU868;

//! @cond Doxygen_Suppress

// LoRa Channel Index ( EU868 )

#define LORAWAN_CH_EU_868_1_MHZ                    0
#define LORAWAN_CH_EU_868_3_MHZ                    1
#define LORAWAN_CH_EU_868_5_MHZ                    2
#define LORAWAN_CH_EU_869_525_MHZ                  128

#define IS_LORAWAN_CH_EU(x)                       ( ((x) == LORAWAN_CH_EU_868_1_MHZ) || \
                                                  ((x) == LORAWAN_CH_EU_868_3_MHZ) || \
                                                  ((x) == LORAWAN_CH_EU_868_5_MHZ) || \
                                                  ((x) == LORAWAN_CH_EU_869_525_MHZ) )

//! @endcond

/**
 * @brief Channel definition for EU 868MHz Band
 */
typedef enum TLoRaWAN_Channel_EU
{
    LoRaWAN_Channel_EU_868_1_Mhz   = 0,                                         /*!< carrier frequency of 868.100 MHz */
    LoRaWAN_Channel_EU_868_3_Mhz   = 1,                                         /*!< carrier frequency of 868.300 MHz */
    LoRaWAN_Channel_EU_868_5_Mhz   = 2,                                         /*!< carrier frequency of 868.500 MHz */
    LoRaWAN_Channel_EU_869_525_Mhz = 128,                                       /*!< carrier frequency of 869.525 MHz */
} TLoRaWAN_Channel_EU;



#endif /* ARDUINO_SAP_WIMOD_SAP_LORAWAN_IDS_EU868_H_ */
