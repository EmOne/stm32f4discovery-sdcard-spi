//------------------------------------------------------------------------------
//! @file WiMOD_SAP_LORAWAN_IDs_IL915.h
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
 * THIS IS AN EXAMPLE IMPLEMENTATION ACCORDING THE THE HCI SPEC: V1.22 for IL915 (V0.1)
 * FOR FIRMWARE: LoRaWAN_EndNode_Modem
 *
 * SEE FILE: WiMOD_LoRaWAN_EndNode_Modem_IL915_HCI_Spec_V0_1.pdf
 * for detailed information
 *
 */


#ifndef ARDUINO_SAP_WIMOD_SAP_LORAWAN_IDS_IL915_H_

//------------------------------------------------------------------------------
//
// Section Includes Files
//
//------------------------------------------------------------------------------

//! @cond Doxygen_Suppress

#define WIMOD_LORAWAN_IL915



// LoRa Radio Band Index
#define LORAWAN_BAND_IL_915                     40


// LoRa Data Rate Index

#define LORAWAN_DATA_RATE_IL915_LORA_SF12_125KHZ      0
#define LORAWAN_DATA_RATE_IL915_LORA_SF11_125KHZ      1
#define LORAWAN_DATA_RATE_IL915_LORA_SF10_125KHZ      2
#define LORAWAN_DATA_RATE_IL915_LORA_SF9_125KHZ       3
#define LORAWAN_DATA_RATE_IL915_LORA_SF8_125KHZ       4
#define LORAWAN_DATA_RATE_IL915_LORA_SF7_125KHZ       5
#define LORAWAN_DATA_RATE_IL915_LORA_SF7_250KHZ       6
#define LORAWAN_DATA_RATE_IL915_LORA_FSK              7

#define IS_LORAWAN_IL915_DATA_RATE(x)                 ( ((x) >= LORAWAN_DATA_RATE_IL915_LORA_SF12_125KHZ) && \
														((X) <= LORAWAN_DATA_RATE_IL915_LORA_FSK) )

//! @endcond


/**
 * @brief LoRa data rate definitions for Israel 915 MHz Band
 */
typedef enum TLoRaWANDataRateIL915
{
    LoRaWAN_DataRate_IL915_LoRa_SF12_125kHz = 0,                                /*!< LoRa Spreading factor 12 using 125kHz bandwidth */
    LoRaWAN_DataRate_IL915_LoRa_SF11_125kHz,                                    /*!< LoRa Spreading factor 11 using 125kHz bandwidth */
    LoRaWAN_DataRate_IL915_LoRa_SF10_125kHz,                                    /*!< LoRa Spreading factor 10 using 125kHz bandwidth */
    LoRaWAN_DataRate_IL915_LoRa_SF9_125kHz,                                     /*!< LoRa Spreading factor  9 using 125kHz bandwidth */
    LoRaWAN_DataRate_IL915_LoRa_SF8_125kHz,                                     /*!< LoRa Spreading factor  8 using 125kHz bandwidth */
    LoRaWAN_DataRate_IL915_LoRa_SF7_125kHz,                                     /*!< LoRa Spreading factor  7 using 125kHz bandwidth */
    LoRaWAN_DataRate_IL915_LoRa_SF7_250kHz,                                     /*!< LoRa Spreading factor  7 using 250kHz bandwidth */
    LoRaWAN_DataRate_IL915_LoRa_FSK,                                            /*!< FSK modulation; LoRa settings not applicable    */
} TLoRaWANDataRateIL915;

//! @cond Doxygen_Suppress

// LoRa Channel Index ( IL915 )

#define LORAWAN_CH_IL_915_7_MHZ                    0
#define LORAWAN_CH_IL_915_9_MHZ                    1
#define LORAWAN_CH_IL_916_1_MHZ                    2
#define LORAWAN_CH_IL_916_3_MHZ                  128

#define IS_LORAWAN_CH_IL(x)                       ( ((x) == LORAWAN_CH_IL_915_7_MHZ) || \
                                                  ((x) == LORAWAN_CH_IL_915_9_MHZ) || \
                                                  ((x) == LORAWAN_CH_IL_916_1_MHZ) || \
                                                  ((x) == LORAWAN_CH_IL_916_3_MHZ) )

//! @endcond

/**
 * @brief Channel definition for IL 915MHz Band
 */
typedef enum TLoRaWAN_Channel_IL
{
    LoRaWAN_Channel_IL_915_7_Mhz   = 0,                                         /*!< carrier frequency of 915.700 MHz */
    LoRaWAN_Channel_IL_915_9_Mhz   = 1,                                         /*!< carrier frequency of 915.900 MHz */
    LoRaWAN_Channel_IL_916_1_Mhz   = 2,                                         /*!< carrier frequency of 916.100 MHz */
    LoRaWAN_Channel_IL_916_3_Mhz   = 128,                                       /*!< carrier frequency of 916.300 MHz */
} TLoRaWAN_Channel_IL;



#endif /* ARDUINO_SAP_WIMOD_SAP_LORAWAN_IDS_IL915_H_ */
