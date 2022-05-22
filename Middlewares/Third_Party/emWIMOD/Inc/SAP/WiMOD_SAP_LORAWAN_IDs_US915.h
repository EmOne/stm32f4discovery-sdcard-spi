//------------------------------------------------------------------------------
//! @file WiMOD_SAP_LORAWAN_IDs_US915.h
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
 * THIS IS AN EXAMPLE IMPLEMENTATION ACCORDING THE THE HCI SPEC: V1.22 for US915 (V0.2)
 * FOR FIRMWARE: LoRaWAN_EndNode_Modem
 *
 * SEE FILE: WiMOD_LoRaWAN_EndNode_Modem_US915_HCI_Spec_V0_2.pdf
 * for detailed information
 *
 */


#ifndef ARDUINO_SAP_WIMOD_SAP_LORAWAN_IDS_US915_H_

//------------------------------------------------------------------------------
//
// Section Includes Files
//
//------------------------------------------------------------------------------

//! @cond Doxygen_Suppress

#define WIMOD_LORAWAN_US915



// LoRa Radio Band Index
#define LORAWAN_BAND_US_915                     2                               /*!<  WiMOD LoRaWAN BAND ID for US 915 MHz Band */


// LoRa Data Rate Index

#define LORAWAN_DATA_RATE_US915_LORA_SF10_125KHZ      0                         /*!< LoRaWAN US DR 0 */
#define LORAWAN_DATA_RATE_US915_LORA_SF9_125KHZ       1                         /*!< LoRaWAN US DR 1 */
#define LORAWAN_DATA_RATE_US915_LORA_SF8_125KHZ       2                         /*!< LoRaWAN US DR 2 */
#define LORAWAN_DATA_RATE_US915_LORA_SF7_125KHZ       3                         /*!< LoRaWAN US DR 3 */
#define LORAWAN_DATA_RATE_US915_LORA_SF8_500KHZ_1     4                         /*!< LoRaWAN US DR 4 */
#define LORAWAN_DATA_RATE_US915_LORA_SF12_500KHZ      8                         /*!< LoRaWAN US DR 8 */
#define LORAWAN_DATA_RATE_US915_LORA_SF11_500KHZ      9                         /*!< LoRaWAN US DR 9 */
#define LORAWAN_DATA_RATE_US915_LORA_SF10_500KHZ      10                        /*!< LoRaWAN US DR 10 */
#define LORAWAN_DATA_RATE_US915_LORA_SF9_500KHZ       11                        /*!< LoRaWAN US DR 11 */
#define LORAWAN_DATA_RATE_US915_LORA_SF8_500KHZ_2     12                        /*!< LoRaWAN US DR 12 */
#define LORAWAN_DATA_RATE_US915_LORA_SF7_500KHZ       13                        /*!< LoRaWAN US DR 13 */


#define IS_LORAWAN_US915_DATA_RATE(x)                 ( ((x) == LORAWAN_DATA_RATE_US915_LORA_SF10_125KHZ ) || \
														((x) == LORAWAN_DATA_RATE_US915_LORA_SF9_125KHZ  ) || \
														((x) == LORAWAN_DATA_RATE_US915_LORA_SF8_125KHZ  ) || \
														((x) == LORAWAN_DATA_RATE_US915_LORA_SF7_125KHZ  ) || \
														((x) == LORAWAN_DATA_RATE_US915_LORA_SF8_500KHZ_1) || \
														((x) == LORAWAN_DATA_RATE_US915_LORA_SF12_500KHZ ) || \
														((x) == LORAWAN_DATA_RATE_US915_LORA_SF11_500KHZ ) || \
														((x) == LORAWAN_DATA_RATE_US915_LORA_SF10_500KHZ ) || \
														((x) == LORAWAN_DATA_RATE_US915_LORA_SF9_500KHZ  ) || \
														((x) == LORAWAN_DATA_RATE_US915_LORA_SF8_500KHZ_2) || \
														((x) == LORAWAN_DATA_RATE_US915_LORA_SF7_500KHZ  ) );
//! @endcond


/**
 * @brief LoRa data rate definitions for US 915 MHz Band
 */
typedef enum TLoRaWANDataRateUS915
{
	LoRaWAN_DataRate_US915_LoRa_SF10_125KHZ = 0,								/*!< LoRa Spreading factor 10 using 125kHz bandwidth */
	LoRaWAN_DataRate_US915_LoRa_SF9_125KHZ  = 1,								/*!< LoRa Spreading factor 9  using 125kHz bandwidth */
	LoRaWAN_DataRate_US915_LoRa_SF8_125KHZ  = 2,								/*!< LoRa Spreading factor 8  using 125kHz bandwidth */
	LoRaWAN_DataRate_US915_LoRa_SF7_125KHZ  = 3,								/*!< LoRa Spreading factor 7  using 125kHz bandwidth */
	LoRaWAN_DataRate_US915_LoRa_SF8_500KHZ_1= 4,								/*!< LoRa Spreading factor 8  using 500kHz bandwidth */
	LoRaWAN_DataRate_US915_LoRa_SF12_500KHZ = 8,								/*!< LoRa Spreading factor 12 using 500kHz bandwidth */
	LoRaWAN_DataRate_US915_LoRa_SF11_500KHZ = 9,								/*!< LoRa Spreading factor 11 using 500kHz bandwidth */
	LoRaWAN_DataRate_US915_LoRa_SF10_500KHZ = 10,								/*!< LoRa Spreading factor 10 using 500kHz bandwidth */
	LoRaWAN_DataRate_US915_LoRa_SF9_500KHZ  = 11,								/*!< LoRa Spreading factor 9  using 500kHz bandwidth */
	LoRaWAN_DataRate_US915_LoRa_SF8_500KHZ_2= 12,								/*!< LoRa Spreading factor 8  using 500kHz bandwidth */
	LoRaWAN_DataRate_US915_LoRa_SF7_500KHZ  = 13,								/*!< LoRa Spreading factor 7  using 500kHz bandwidth */
} TLoRaWANDataRateUS915;



//! @cond Doxygen_Suppress

// LoRa Channel Index ( US915 )

// uplink
#define LORAWAN_CH_US_UP_902_3_MHZ                    0                         /*!< LoRaWAN US uplink chanel */
#define LORAWAN_CH_US_UP_902_5_MHZ                    1                         /*!< LoRaWAN US uplink chanel */
#define LORAWAN_CH_US_UP_902_7_MHZ                    2                         /*!< LoRaWAN US uplink chanel */
#define LORAWAN_CH_US_UP_902_9_MHZ                    3                         /*!< LoRaWAN US uplink chanel */
#define LORAWAN_CH_US_UP_903_1_MHZ                    4                         /*!< LoRaWAN US uplink chanel */
#define LORAWAN_CH_US_UP_903_3_MHZ                    5                         /*!< LoRaWAN US uplink chanel */
#define LORAWAN_CH_US_UP_903_5_MHZ                    6                         /*!< LoRaWAN US uplink chanel */
#define LORAWAN_CH_US_UP_903_7_MHZ                    7                         /*!< LoRaWAN US uplink chanel */

#define LORAWAN_CH_US_UP_903_9_MHZ                    8                         /*!< LoRaWAN US uplink chanel */
#define LORAWAN_CH_US_UP_904_1_MHZ                    9                         /*!< LoRaWAN US uplink chanel */
#define LORAWAN_CH_US_UP_904_3_MHZ                    10                        /*!< LoRaWAN US uplink chanel */
#define LORAWAN_CH_US_UP_904_5_MHZ                    11                        /*!< LoRaWAN US uplink chanel */
#define LORAWAN_CH_US_UP_904_7_MHZ                    12                        /*!< LoRaWAN US uplink chanel */
#define LORAWAN_CH_US_UP_904_9_MHZ                    13                        /*!< LoRaWAN US uplink chanel */
#define LORAWAN_CH_US_UP_905_1_MHZ                    14                        /*!< LoRaWAN US uplink chanel */
#define LORAWAN_CH_US_UP_905_3_MHZ                    15                        /*!< LoRaWAN US uplink chanel */

#define LORAWAN_CH_US_UP_905_5_MHZ                    16                        /*!< LoRaWAN US uplink chanel */
#define LORAWAN_CH_US_UP_905_7_MHZ                    17                        /*!< LoRaWAN US uplink chanel */
#define LORAWAN_CH_US_UP_905_9_MHZ                    18                        /*!< LoRaWAN US uplink chanel */
#define LORAWAN_CH_US_UP_906_1_MHZ                    19                        /*!< LoRaWAN US uplink chanel */
#define LORAWAN_CH_US_UP_906_3_MHZ                    20                        /*!< LoRaWAN US uplink chanel */
#define LORAWAN_CH_US_UP_906_5_MHZ                    21                        /*!< LoRaWAN US uplink chanel */
#define LORAWAN_CH_US_UP_906_7_MHZ                    22                        /*!< LoRaWAN US uplink chanel */
#define LORAWAN_CH_US_UP_906_9_MHZ                    23                        /*!< LoRaWAN US uplink chanel */

#define LORAWAN_CH_US_UP_907_1_MHZ                    24                        /*!< LoRaWAN US uplink chanel */
#define LORAWAN_CH_US_UP_907_3_MHZ                    25                        /*!< LoRaWAN US uplink chanel */
#define LORAWAN_CH_US_UP_907_5_MHZ                    26                        /*!< LoRaWAN US uplink chanel */
#define LORAWAN_CH_US_UP_907_7_MHZ                    27                        /*!< LoRaWAN US uplink chanel */
#define LORAWAN_CH_US_UP_907_9_MHZ                    28                        /*!< LoRaWAN US uplink chanel */
#define LORAWAN_CH_US_UP_908_1_MHZ                    29                        /*!< LoRaWAN US uplink chanel */
#define LORAWAN_CH_US_UP_908_3_MHZ                    30                        /*!< LoRaWAN US uplink chanel */
#define LORAWAN_CH_US_UP_908_5_MHZ                    31                        /*!< LoRaWAN US uplink chanel */

#define LORAWAN_CH_US_UP_908_7_MHZ                    32                        /*!< LoRaWAN US uplink chanel */
#define LORAWAN_CH_US_UP_908_9_MHZ                    33                        /*!< LoRaWAN US uplink chanel */
#define LORAWAN_CH_US_UP_909_1_MHZ                    34                        /*!< LoRaWAN US uplink chanel */
#define LORAWAN_CH_US_UP_909_3_MHZ                    35                        /*!< LoRaWAN US uplink chanel */
#define LORAWAN_CH_US_UP_909_5_MHZ                    36                        /*!< LoRaWAN US uplink chanel */
#define LORAWAN_CH_US_UP_909_7_MHZ                    37                        /*!< LoRaWAN US uplink chanel */
#define LORAWAN_CH_US_UP_909_9_MHZ                    38                        /*!< LoRaWAN US uplink chanel */
#define LORAWAN_CH_US_UP_910_1_MHZ                    39                        /*!< LoRaWAN US uplink chanel */

#define LORAWAN_CH_US_UP_910_3_MHZ                    40                        /*!< LoRaWAN US uplink chanel */
#define LORAWAN_CH_US_UP_910_5_MHZ                    41                        /*!< LoRaWAN US uplink chanel */
#define LORAWAN_CH_US_UP_910_7_MHZ                    42                        /*!< LoRaWAN US uplink chanel */
#define LORAWAN_CH_US_UP_910_9_MHZ                    43                        /*!< LoRaWAN US uplink chanel */
#define LORAWAN_CH_US_UP_911_1_MHZ                    44                        /*!< LoRaWAN US uplink chanel */
#define LORAWAN_CH_US_UP_911_3_MHZ                    45                        /*!< LoRaWAN US uplink chanel */
#define LORAWAN_CH_US_UP_911_5_MHZ                    46                        /*!< LoRaWAN US uplink chanel */
#define LORAWAN_CH_US_UP_911_7_MHZ                    47                        /*!< LoRaWAN US uplink chanel */

#define LORAWAN_CH_US_UP_911_9_MHZ                    48                        /*!< LoRaWAN US uplink chanel */
#define LORAWAN_CH_US_UP_912_1_MHZ                    49                        /*!< LoRaWAN US uplink chanel */
#define LORAWAN_CH_US_UP_912_3_MHZ                    50                        /*!< LoRaWAN US uplink chanel */
#define LORAWAN_CH_US_UP_912_5_MHZ                    51                        /*!< LoRaWAN US uplink chanel */
#define LORAWAN_CH_US_UP_912_7_MHZ                    52                        /*!< LoRaWAN US uplink chanel */
#define LORAWAN_CH_US_UP_912_9_MHZ                    53                        /*!< LoRaWAN US uplink chanel */
#define LORAWAN_CH_US_UP_913_1_MHZ                    54                        /*!< LoRaWAN US uplink chanel */
#define LORAWAN_CH_US_UP_913_3_MHZ                    55                        /*!< LoRaWAN US uplink chanel */

#define LORAWAN_CH_US_UP_913_5_MHZ                    56                        /*!< LoRaWAN US uplink chanel */
#define LORAWAN_CH_US_UP_913_7_MHZ                    57                        /*!< LoRaWAN US uplink chanel */
#define LORAWAN_CH_US_UP_913_9_MHZ                    58                        /*!< LoRaWAN US uplink chanel */
#define LORAWAN_CH_US_UP_914_1_MHZ                    59                        /*!< LoRaWAN US uplink chanel */
#define LORAWAN_CH_US_UP_914_3_MHZ                    60                        /*!< LoRaWAN US uplink chanel */
#define LORAWAN_CH_US_UP_914_5_MHZ                    61                        /*!< LoRaWAN US uplink chanel */
#define LORAWAN_CH_US_UP_914_7_MHZ                    62                        /*!< LoRaWAN US uplink chanel */
#define LORAWAN_CH_US_UP_914_9_MHZ                    63                        /*!< LoRaWAN US uplink chanel */

#define LORAWAN_CH_US_UP_903_0_MHZ                    64                        /*!< LoRaWAN US uplink chanel */
#define LORAWAN_CH_US_UP_904_6_MHZ                    65                        /*!< LoRaWAN US uplink chanel */
#define LORAWAN_CH_US_UP_906_2_MHZ                    66                        /*!< LoRaWAN US uplink chanel */
#define LORAWAN_CH_US_UP_907_8_MHZ                    67                        /*!< LoRaWAN US uplink chanel */
#define LORAWAN_CH_US_UP_909_4_MHZ                    68                        /*!< LoRaWAN US uplink chanel */
#define LORAWAN_CH_US_UP_911_0_MHZ                    69                        /*!< LoRaWAN US uplink chanel */
#define LORAWAN_CH_US_UP_912_6_MHZ                    70                        /*!< LoRaWAN US uplink chanel */
#define LORAWAN_CH_US_UP_914_2_MHZ                    71                        /*!< LoRaWAN US uplink chanel */


//downlink
#define LORAWAN_CH_US_DN_923_3_MHZ                    0                         /*!< LoRaWAN US downlink chanel */
#define LORAWAN_CH_US_DN_923_9_MHZ                    1
#define LORAWAN_CH_US_DN_924_5_MHZ                    2
#define LORAWAN_CH_US_DN_925_1_MHZ                    3
#define LORAWAN_CH_US_DN_925_7_MHZ                    4
#define LORAWAN_CH_US_DN_926_3_MHZ                    5
#define LORAWAN_CH_US_DN_926_9_MHZ                    6
#define LORAWAN_CH_US_DN_927_5_MHZ                    7

#define LORAWAN_CH_US_DN_923_3_MHZ_2                  8
#define LORAWAN_CH_US_DN_923_9_MHZ_2                  9
#define LORAWAN_CH_US_DN_924_5_MHZ_2                  10
#define LORAWAN_CH_US_DN_925_1_MHZ_2                  11
#define LORAWAN_CH_US_DN_925_7_MHZ_2                  12
#define LORAWAN_CH_US_DN_926_3_MHZ_2                  13
#define LORAWAN_CH_US_DN_926_9_MHZ_2                  14
#define LORAWAN_CH_US_DN_927_5_MHZ_2                  15

#define LORAWAN_CH_US_DN_923_3_MHZ_3                  16
#define LORAWAN_CH_US_DN_923_9_MHZ_3                  17
#define LORAWAN_CH_US_DN_924_5_MHZ_3                  18
#define LORAWAN_CH_US_DN_925_1_MHZ_3                  19
#define LORAWAN_CH_US_DN_925_7_MHZ_3                  20
#define LORAWAN_CH_US_DN_926_3_MHZ_3                  21
#define LORAWAN_CH_US_DN_926_9_MHZ_3                  22
#define LORAWAN_CH_US_DN_927_5_MHZ_3                  23

#define LORAWAN_CH_US_DN_923_3_MHZ_4                  24
#define LORAWAN_CH_US_DN_923_9_MHZ_4                  25
#define LORAWAN_CH_US_DN_924_5_MHZ_4                  26
#define LORAWAN_CH_US_DN_925_1_MHZ_4                  27
#define LORAWAN_CH_US_DN_925_7_MHZ_4                  28
#define LORAWAN_CH_US_DN_926_3_MHZ_4                  29
#define LORAWAN_CH_US_DN_926_9_MHZ_4                  30
#define LORAWAN_CH_US_DN_927_5_MHZ_4                  31

#define LORAWAN_CH_US_DN_923_3_MHZ_5                  32
#define LORAWAN_CH_US_DN_923_9_MHZ_5                  33
#define LORAWAN_CH_US_DN_924_5_MHZ_5                  34
#define LORAWAN_CH_US_DN_925_1_MHZ_5                  35
#define LORAWAN_CH_US_DN_925_7_MHZ_5                  36
#define LORAWAN_CH_US_DN_926_3_MHZ_5                  37
#define LORAWAN_CH_US_DN_926_9_MHZ_5                  38
#define LORAWAN_CH_US_DN_927_5_MHZ_5                  39

#define LORAWAN_CH_US_DN_923_3_MHZ_6                  40
#define LORAWAN_CH_US_DN_923_9_MHZ_6                  41
#define LORAWAN_CH_US_DN_924_5_MHZ_6                  42
#define LORAWAN_CH_US_DN_925_1_MHZ_6                  43
#define LORAWAN_CH_US_DN_925_7_MHZ_6                  44
#define LORAWAN_CH_US_DN_926_3_MHZ_6                  45
#define LORAWAN_CH_US_DN_926_9_MHZ_6                  46
#define LORAWAN_CH_US_DN_927_5_MHZ_6                  47

#define LORAWAN_CH_US_DN_923_3_MHZ_7                  48
#define LORAWAN_CH_US_DN_923_9_MHZ_7                  49
#define LORAWAN_CH_US_DN_924_5_MHZ_7                  50
#define LORAWAN_CH_US_DN_925_1_MHZ_7                  51
#define LORAWAN_CH_US_DN_925_7_MHZ_7                  52
#define LORAWAN_CH_US_DN_926_3_MHZ_7                  53
#define LORAWAN_CH_US_DN_926_9_MHZ_7                  54
#define LORAWAN_CH_US_DN_927_5_MHZ_7                  55

#define LORAWAN_CH_US_DN_923_3_MHZ_8                  56
#define LORAWAN_CH_US_DN_923_9_MHZ_8                  57
#define LORAWAN_CH_US_DN_924_5_MHZ_8                  58
#define LORAWAN_CH_US_DN_925_1_MHZ_8                  59
#define LORAWAN_CH_US_DN_925_7_MHZ_8                  60
#define LORAWAN_CH_US_DN_926_3_MHZ_8                  61
#define LORAWAN_CH_US_DN_926_9_MHZ_8                  62
#define LORAWAN_CH_US_DN_927_5_MHZ_8                  63

#define LORAWAN_CH_US_DN_923_3_MHZ_9                  64
#define LORAWAN_CH_US_DN_923_9_MHZ_9                  65
#define LORAWAN_CH_US_DN_924_5_MHZ_9                  66
#define LORAWAN_CH_US_DN_925_1_MHZ_9                  67
#define LORAWAN_CH_US_DN_925_7_MHZ_9                  68
#define LORAWAN_CH_US_DN_926_3_MHZ_9                  69
#define LORAWAN_CH_US_DN_926_9_MHZ_9                  70
#define LORAWAN_CH_US_DN_927_5_MHZ_9                  71

#define LORAWAN_CH_US_DN_923_3_MHZ_10                 128


#define IS_LORAWAN_CH_US915_UP(x)               ( ((x) >= LORAWAN_CH_US_UP_902_3_MHZ) && \
                                                  ((x) == LORAWAN_CH_US_UP_914_2_MHZ) )


#define IS_LORAWAN_CH_US915_DN(x)                 ( ((x) >= LORAWAN_CH_US_DN_923_3_MHZ) || \
													((x) == LORAWAN_CH_US_DN_923_9_MHZ) || \
													((x) == LORAWAN_CH_US_DN_925_1_MHZ) || \
													((x) == LORAWAN_CH_US_DN_925_7_MHZ) || \
													((x) == LORAWAN_CH_US_DN_926_3_MHZ) || \
													((x) == LORAWAN_CH_US_DN_926_9_MHZ) || \
												((x) == LORAWAN_CH_US_DN_927_5_MHZ) )



//! @endcond

/**
 * @brief Channel definition for US 915MHz Band uplink
 */
typedef enum TLoRaWAN_Channel_US_UP
{
	LoRaWAN_Channel_US_UP_902_3_MHZ = 0,										/*!< carrier frequency of 902.3 MHz for uplink */
	LoRaWAN_Channel_US_UP_902_5_MHZ = 1,										/*!< carrier frequency of 902.5 MHz for uplink */
	LoRaWAN_Channel_US_UP_902_7_MHZ = 2,										/*!< carrier frequency of 902.7 MHz for uplink */
	LoRaWAN_Channel_US_UP_902_9_MHZ = 3,										/*!< carrier frequency of 902.9 MHz for uplink */
	LoRaWAN_Channel_US_UP_903_1_MHZ = 4,										/*!< carrier frequency of 903.1 MHz for uplink */
	LoRaWAN_Channel_US_UP_903_3_MHZ = 5,										/*!< carrier frequency of 903.3 MHz for uplink */
	LoRaWAN_Channel_US_UP_903_5_MHZ = 6,										/*!< carrier frequency of 903.5 MHz for uplink */
	LoRaWAN_Channel_US_UP_903_7_MHZ = 7,										/*!< carrier frequency of 903.7 MHz for uplink */

	LoRaWAN_Channel_US_UP_903_9_MHZ = 8,										/*!< carrier frequency of 903.9 MHz for uplink */
	LoRaWAN_Channel_US_UP_904_1_MHZ = 9,										/*!< carrier frequency of 904.1 MHz for uplink */
	LoRaWAN_Channel_US_UP_904_3_MHZ = 10,										/*!< carrier frequency of 904.3 MHz for uplink */
	LoRaWAN_Channel_US_UP_904_5_MHZ = 11,										/*!< carrier frequency of 904.5 MHz for uplink */
	LoRaWAN_Channel_US_UP_904_7_MHZ = 12,										/*!< carrier frequency of 904.7 MHz for uplink */
	LoRaWAN_Channel_US_UP_904_9_MHZ = 13,										/*!< carrier frequency of 904.9 MHz for uplink */
	LoRaWAN_Channel_US_UP_905_1_MHZ = 14,										/*!< carrier frequency of 905.1 MHz for uplink */
	LoRaWAN_Channel_US_UP_905_3_MHZ = 15,										/*!< carrier frequency of 905.3 MHz for uplink */

	LoRaWAN_Channel_US_UP_905_5_MHZ = 16,										/*!< carrier frequency of 905.5 MHz for uplink */
	LoRaWAN_Channel_US_UP_905_7_MHZ = 17,										/*!< carrier frequency of 905.7 MHz for uplink */
	LoRaWAN_Channel_US_UP_905_9_MHZ = 18,										/*!< carrier frequency of 905.9 MHz for uplink */
	LoRaWAN_Channel_US_UP_906_1_MHZ = 19,										/*!< carrier frequency of 906.1 MHz for uplink */
	LoRaWAN_Channel_US_UP_906_3_MHZ = 20,										/*!< carrier frequency of 906.3 MHz for uplink */
	LoRaWAN_Channel_US_UP_906_5_MHZ = 21,										/*!< carrier frequency of 906.5 MHz for uplink */
	LoRaWAN_Channel_US_UP_906_7_MHZ = 22,										/*!< carrier frequency of 906.7 MHz for uplink */
	LoRaWAN_Channel_US_UP_906_9_MHZ = 23,										/*!< carrier frequency of 906.9 MHz for uplink */

	LoRaWAN_Channel_US_UP_907_1_MHZ = 24,										/*!< carrier frequency of 907.1 MHz for uplink */
	LoRaWAN_Channel_US_UP_907_3_MHZ = 25,										/*!< carrier frequency of 907.3 MHz for uplink */
	LoRaWAN_Channel_US_UP_907_5_MHZ = 26,										/*!< carrier frequency of 907.5 MHz for uplink */
	LoRaWAN_Channel_US_UP_907_7_MHZ = 27,										/*!< carrier frequency of 907.7 MHz for uplink */
	LoRaWAN_Channel_US_UP_907_9_MHZ = 28,										/*!< carrier frequency of 907.9 MHz for uplink */
	LoRaWAN_Channel_US_UP_908_1_MHZ = 29,										/*!< carrier frequency of 908.1 MHz for uplink */
	LoRaWAN_Channel_US_UP_908_3_MHZ = 30,										/*!< carrier frequency of 908.3 MHz for uplink */
	LoRaWAN_Channel_US_UP_908_5_MHZ = 31,										/*!< carrier frequency of 908.5 MHz for uplink */

	LoRaWAN_Channel_US_UP_908_7_MHZ = 32,										/*!< carrier frequency of 908.7 MHz for uplink */
	LoRaWAN_Channel_US_UP_908_9_MHZ = 33,										/*!< carrier frequency of 908.9 MHz for uplink */
	LoRaWAN_Channel_US_UP_909_1_MHZ = 34,										/*!< carrier frequency of 909.1 MHz for uplink */
	LoRaWAN_Channel_US_UP_909_3_MHZ = 35,										/*!< carrier frequency of 909.3 MHz for uplink */
	LoRaWAN_Channel_US_UP_909_5_MHZ = 36,										/*!< carrier frequency of 909.5 MHz for uplink */
	LoRaWAN_Channel_US_UP_909_7_MHZ = 37,										/*!< carrier frequency of 909.7 MHz for uplink */
	LoRaWAN_Channel_US_UP_909_9_MHZ = 38,										/*!< carrier frequency of 909.9 MHz for uplink */
	LoRaWAN_Channel_US_UP_910_1_MHZ = 39,										/*!< carrier frequency of 910.1 MHz for uplink */

	LoRaWAN_Channel_US_UP_910_3_MHZ = 40,										/*!< carrier frequency of 910.3 MHz for uplink */
	LoRaWAN_Channel_US_UP_910_5_MHZ = 41,										/*!< carrier frequency of 910.5 MHz for uplink */
	LoRaWAN_Channel_US_UP_910_7_MHZ = 42,										/*!< carrier frequency of 910.7 MHz for uplink */
	LoRaWAN_Channel_US_UP_910_9_MHZ = 43,										/*!< carrier frequency of 910.9 MHz for uplink */
	LoRaWAN_Channel_US_UP_911_1_MHZ = 44,										/*!< carrier frequency of 911.1 MHz for uplink */
	LoRaWAN_Channel_US_UP_911_3_MHZ = 45,										/*!< carrier frequency of 911.3 MHz for uplink */
	LoRaWAN_Channel_US_UP_911_5_MHZ = 46,										/*!< carrier frequency of 911.5 MHz for uplink */
	LoRaWAN_Channel_US_UP_911_7_MHZ = 47,										/*!< carrier frequency of 911.7 MHz for uplink */

	LoRaWAN_Channel_US_UP_911_9_MHZ = 48,										/*!< carrier frequency of 911.9 MHz for uplink */
	LoRaWAN_Channel_US_UP_912_1_MHZ = 49,										/*!< carrier frequency of 912.1 MHz for uplink */
	LoRaWAN_Channel_US_UP_912_3_MHZ = 50,										/*!< carrier frequency of 912.3 MHz for uplink */
	LoRaWAN_Channel_US_UP_912_5_MHZ = 51,										/*!< carrier frequency of 912.5 MHz for uplink */
	LoRaWAN_Channel_US_UP_912_7_MHZ = 52,										/*!< carrier frequency of 912.7 MHz for uplink */
	LoRaWAN_Channel_US_UP_912_9_MHZ = 53,										/*!< carrier frequency of 912.9 MHz for uplink */
	LoRaWAN_Channel_US_UP_913_1_MHZ = 54,										/*!< carrier frequency of 913.1 MHz for uplink */
	LoRaWAN_Channel_US_UP_913_3_MHZ = 55,										/*!< carrier frequency of 913.3 MHz for uplink */

	LoRaWAN_Channel_US_UP_913_5_MHZ = 56,										/*!< carrier frequency of 913.5 MHz for uplink */
	LoRaWAN_Channel_US_UP_913_7_MHZ = 57,										/*!< carrier frequency of 913.7 MHz for uplink */
	LoRaWAN_Channel_US_UP_913_9_MHZ = 58,										/*!< carrier frequency of 913.9 MHz for uplink */
	LoRaWAN_Channel_US_UP_914_1_MHZ = 59,										/*!< carrier frequency of 914.1 MHz for uplink */
	LoRaWAN_Channel_US_UP_914_3_MHZ = 60,										/*!< carrier frequency of 914.3 MHz for uplink */
	LoRaWAN_Channel_US_UP_914_5_MHZ = 61,										/*!< carrier frequency of 914.5 MHz for uplink */
	LoRaWAN_Channel_US_UP_914_7_MHZ = 62,										/*!< carrier frequency of 914.7 MHz for uplink */
	LoRaWAN_Channel_US_UP_914_9_MHZ = 63,										/*!< carrier frequency of 914.9 MHz for uplink */

	LoRaWAN_Channel_US_UP_903_0_MHZ = 64,										/*!< carrier frequency of 903.0 MHz for uplink */
	LoRaWAN_Channel_US_UP_904_6_MHZ = 65,										/*!< carrier frequency of 904.6 MHz for uplink */
	LoRaWAN_Channel_US_UP_906_2_MHZ = 66,										/*!< carrier frequency of 906.2 MHz for uplink */
	LoRaWAN_Channel_US_UP_907_8_MHZ = 67,										/*!< carrier frequency of 907.8 MHz for uplink */
	LoRaWAN_Channel_US_UP_909_4_MHZ = 68,										/*!< carrier frequency of 909.4 MHz for uplink */
	LoRaWAN_Channel_US_UP_911_0_MHZ = 69,										/*!< carrier frequency of 911.0 MHz for uplink */
	LoRaWAN_Channel_US_UP_912_6_MHZ = 70,										/*!< carrier frequency of 912.6 MHz for uplink */
	LoRaWAN_Channel_US_UP_914_2_MHZ = 71,										/*!< carrier frequency of 914.2 MHz for uplink */
} TLoRaWAN_Channel_US_UP;

/**
 * @brief Channel definition for US 915MHz Band downlink
 */
typedef enum TLoRaWAN_Channel_US_DN
{
	LoRaWAN_Channel_US_DN_923_3_MHz = 0,										/*!< carrier frequency of 923.3 MHz for downlink */
	LoRaWAN_Channel_US_DN_923_9_MHz = 1,										/*!< carrier frequency of 923.9 MHz for downlink */
	LoRaWAN_Channel_US_DN_924_5_MHz = 2,										/*!< carrier frequency of 924.5 MHz for downlink */
	LoRaWAN_Channel_US_DN_925_1_MHz = 3,										/*!< carrier frequency of 925.1 MHz for downlink */
	LoRaWAN_Channel_US_DN_925_7_MHz = 4,										/*!< carrier frequency of 925.7 MHz for downlink */
	LoRaWAN_Channel_US_DN_926_3_MHz = 5,										/*!< carrier frequency of 926.3 MHz for downlink */
	LoRaWAN_Channel_US_DN_926_9_MHz = 6,										/*!< carrier frequency of 926.9 MHz for downlink */
	LoRaWAN_Channel_US_DN_927_5_MHz = 7,										/*!< carrier frequency of 927.5 MHz for downlink */

	LoRaWAN_Channel_US_DN_923_3_MHz_2 = 8,										/*!< carrier frequency of 923.3 MHz for downlink */
	LoRaWAN_Channel_US_DN_923_9_MHz_2 = 9,										/*!< carrier frequency of 923.9 MHz for downlink */
	LoRaWAN_Channel_US_DN_924_5_MHz_2 = 10,										/*!< carrier frequency of 924.5 MHz for downlink */
	LoRaWAN_Channel_US_DN_925_1_MHz_2 = 11,										/*!< carrier frequency of 925.1 MHz for downlink */
	LoRaWAN_Channel_US_DN_925_7_MHz_2 = 12,										/*!< carrier frequency of 925.7 MHz for downlink */
	LoRaWAN_Channel_US_DN_926_3_MHz_2 = 13,										/*!< carrier frequency of 926.3 MHz for downlink */
	LoRaWAN_Channel_US_DN_926_9_MHz_2 = 14,										/*!< carrier frequency of 926.9 MHz for downlink */
	LoRaWAN_Channel_US_DN_927_5_MHz_2 = 15,										/*!< carrier frequency of 927.5 MHz for downlink */

	LoRaWAN_Channel_US_DN_923_3_MHz_3 = 16,										/*!< carrier frequency of 923.3 MHz for downlink */
	LoRaWAN_Channel_US_DN_923_9_MHz_3 = 17,										/*!< carrier frequency of 923.9 MHz for downlink */
	LoRaWAN_Channel_US_DN_924_5_MHz_3 = 18,										/*!< carrier frequency of 924.5 MHz for downlink */
	LoRaWAN_Channel_US_DN_925_1_MHz_3 = 19,										/*!< carrier frequency of 925.1 MHz for downlink */
	LoRaWAN_Channel_US_DN_925_7_MHz_3 = 20,										/*!< carrier frequency of 925.7 MHz for downlink */
	LoRaWAN_Channel_US_DN_926_3_MHz_3 = 21,										/*!< carrier frequency of 926.3 MHz for downlink */
	LoRaWAN_Channel_US_DN_926_9_MHz_3 = 22,										/*!< carrier frequency of 926.9 MHz for downlink */
	LoRaWAN_Channel_US_DN_927_5_MHz_3 = 23,										/*!< carrier frequency of 927.5 MHz for downlink */

	LoRaWAN_Channel_US_DN_923_3_MHz_4 = 24,										/*!< carrier frequency of 923.3 MHz for downlink */
	LoRaWAN_Channel_US_DN_923_9_MHz_4 = 25,										/*!< carrier frequency of 923.9 MHz for downlink */
	LoRaWAN_Channel_US_DN_924_5_MHz_4 = 26,										/*!< carrier frequency of 924.5 MHz for downlink */
	LoRaWAN_Channel_US_DN_925_1_MHz_4 = 27,										/*!< carrier frequency of 925.1 MHz for downlink */
	LoRaWAN_Channel_US_DN_925_7_MHz_4 = 28,										/*!< carrier frequency of 925.7 MHz for downlink */
	LoRaWAN_Channel_US_DN_926_3_MHz_4 = 29,										/*!< carrier frequency of 926.3 MHz for downlink */
	LoRaWAN_Channel_US_DN_926_9_MHz_4 = 30,										/*!< carrier frequency of 926.9 MHz for downlink */
	LoRaWAN_Channel_US_DN_927_5_MHz_4 = 31,										/*!< carrier frequency of 927.5 MHz for downlink */

	LoRaWAN_Channel_US_DN_923_3_MHz_5 = 32,										/*!< carrier frequency of 923.3 MHz for downlink */
	LoRaWAN_Channel_US_DN_923_9_MHz_5 = 33,										/*!< carrier frequency of 923.9 MHz for downlink */
	LoRaWAN_Channel_US_DN_924_5_MHz_5 = 34,										/*!< carrier frequency of 924.5 MHz for downlink */
	LoRaWAN_Channel_US_DN_925_1_MHz_5 = 35,										/*!< carrier frequency of 925.1 MHz for downlink */
	LoRaWAN_Channel_US_DN_925_7_MHz_5 = 36,										/*!< carrier frequency of 925.7 MHz for downlink */
	LoRaWAN_Channel_US_DN_926_3_MHz_5 = 37,										/*!< carrier frequency of 926.3 MHz for downlink */
	LoRaWAN_Channel_US_DN_926_9_MHz_5 = 38,										/*!< carrier frequency of 926.9 MHz for downlink */
	LoRaWAN_Channel_US_DN_927_5_MHz_5 = 39,										/*!< carrier frequency of 927.5 MHz for downlink */

	LoRaWAN_Channel_US_DN_923_3_MHz_6 = 40,										/*!< carrier frequency of 923.3 MHz for downlink */
	LoRaWAN_Channel_US_DN_923_9_MHz_6 = 41,										/*!< carrier frequency of 923.9 MHz for downlink */
	LoRaWAN_Channel_US_DN_924_5_MHz_6 = 42,										/*!< carrier frequency of 924.5 MHz for downlink */
	LoRaWAN_Channel_US_DN_925_1_MHz_6 = 43,										/*!< carrier frequency of 925.1 MHz for downlink */
	LoRaWAN_Channel_US_DN_925_7_MHz_6 = 44,										/*!< carrier frequency of 925.7 MHz for downlink */
	LoRaWAN_Channel_US_DN_926_3_MHz_6 = 45,										/*!< carrier frequency of 926.3 MHz for downlink */
	LoRaWAN_Channel_US_DN_926_9_MHz_6 = 46,										/*!< carrier frequency of 926.9 MHz for downlink */
	LoRaWAN_Channel_US_DN_927_5_MHz_6 = 47,										/*!< carrier frequency of 927.5 MHz for downlink */

	LoRaWAN_Channel_US_DN_923_3_MHz_7 = 48,										/*!< carrier frequency of 923.3 MHz for downlink */
	LoRaWAN_Channel_US_DN_923_9_MHz_7 = 49,										/*!< carrier frequency of 923.9 MHz for downlink */
	LoRaWAN_Channel_US_DN_924_5_MHz_7 = 50,										/*!< carrier frequency of 924.5 MHz for downlink */
	LoRaWAN_Channel_US_DN_925_1_MHz_7 = 51,										/*!< carrier frequency of 925.1 MHz for downlink */
	LoRaWAN_Channel_US_DN_925_7_MHz_7 = 52,										/*!< carrier frequency of 925.7 MHz for downlink */
	LoRaWAN_Channel_US_DN_926_3_MHz_7 = 53,										/*!< carrier frequency of 926.3 MHz for downlink */
	LoRaWAN_Channel_US_DN_926_9_MHz_7 = 54,										/*!< carrier frequency of 926.9 MHz for downlink */
	LoRaWAN_Channel_US_DN_927_5_MHz_7 = 55,										/*!< carrier frequency of 927.5 MHz for downlink */

	LoRaWAN_Channel_US_DN_923_3_MHz_8 = 56,										/*!< carrier frequency of 923.3 MHz for downlink */
	LoRaWAN_Channel_US_DN_923_9_MHz_8 = 57,										/*!< carrier frequency of 923.9 MHz for downlink */
	LoRaWAN_Channel_US_DN_924_5_MHz_8 = 58,										/*!< carrier frequency of 924.5 MHz for downlink */
	LoRaWAN_Channel_US_DN_925_1_MHz_8 = 59,										/*!< carrier frequency of 925.1 MHz for downlink */
	LoRaWAN_Channel_US_DN_925_7_MHz_8 = 60,										/*!< carrier frequency of 925.7 MHz for downlink */
	LoRaWAN_Channel_US_DN_926_3_MHz_8 = 61,										/*!< carrier frequency of 926.3 MHz for downlink */
	LoRaWAN_Channel_US_DN_926_9_MHz_8 = 62,										/*!< carrier frequency of 926.9 MHz for downlink */
	LoRaWAN_Channel_US_DN_927_5_MHz_8 = 63,										/*!< carrier frequency of 927.5 MHz for downlink */

	LoRaWAN_Channel_US_DN_923_3_MHz_9 = 64,										/*!< carrier frequency of 923.3 MHz for downlink */
	LoRaWAN_Channel_US_DN_923_9_MHz_9 = 65,										/*!< carrier frequency of 923.9 MHz for downlink */
	LoRaWAN_Channel_US_DN_924_5_MHz_9 = 66,										/*!< carrier frequency of 924.5 MHz for downlink */
	LoRaWAN_Channel_US_DN_925_1_MHz_9 = 67,										/*!< carrier frequency of 925.1 MHz for downlink */
	LoRaWAN_Channel_US_DN_925_7_MHz_9 = 68,										/*!< carrier frequency of 925.7 MHz for downlink */
	LoRaWAN_Channel_US_DN_926_3_MHz_9 = 69,										/*!< carrier frequency of 926.3 MHz for downlink */
	LoRaWAN_Channel_US_DN_926_9_MHz_9 = 70,										/*!< carrier frequency of 926.9 MHz for downlink */
	LoRaWAN_Channel_US_DN_927_5_MHz_9 = 71,										/*!< carrier frequency of 927.5 MHz for downlink */

	LoRaWAN_Channel_US_DN_923_3_MHz_10 = 128,										/*!< carrier frequency of 923.3 MHz for downlink */
} TLoRaWAN_Channel_US_DN;


#endif /* ARDUINO_SAP_WIMOD_SAP_LORAWAN_IDS_US915_H_ */
