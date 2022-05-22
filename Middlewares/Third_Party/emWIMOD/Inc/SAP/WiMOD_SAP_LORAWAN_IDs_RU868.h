//------------------------------------------------------------------------------
//! @file WiMOD_SAP_LORAWAN_IDs_RU868.h
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
 * THIS IS AN EXAMPLE IMPLEMENTATION ACCORDING THE THE HCI SPEC: V1.22 for RU868 (V0.1)
 * FOR FIRMWARE: LoRaWAN_EndNode_Modem
 *
 * SEE FILE: WiMOD_LoRaWAN_EndNode_Modem_RU868_HCI_Spec_V0_1.pdf
 * for detailed information
 *
 */


#ifndef ARDUINO_SAP_WIMOD_SAP_LORAWAN_IDS_RU868_H_

//------------------------------------------------------------------------------
//
// Section Includes Files
//
//------------------------------------------------------------------------------

//! @cond Doxygen_Suppress

#define WIMOD_LORAWAN_RU868


// LoRa Radio Band Index
#define LORAWAN_BAND_RU1_868                    30
#define LORAWAN_BAND_RU2_868                    31
#define LORAWAN_BAND_RU3_868                    32
#define LORAWAN_BAND_RU4_868                    33
#define LORAWAN_BAND_RU5_868                    34
#define LORAWAN_BAND_RU6_868                    35
#define LORAWAN_BAND_RU7_868                    36


// LoRa Channel Index ( Russia )

#define LORAWAN_CH_RU1_868_78_MHZ                   0
#define LORAWAN_CH_RU2_868_95_MHZ                   1
#define LORAWAN_CH_RU1_869_12_MHZ                   2
#define LORAWAN_CH_RU1_864_1_MHZ                    3
#define LORAWAN_CH_RU1_864_3_MHZ                    4
#define LORAWAN_CH_RU1_864_5_MHZ                    5
#define LORAWAN_CH_RU1_864_7_MHZ                    6
#define LORAWAN_CH_RU1_864_9_MHZ                    128

#define IS_LORAWAN_CH_RU1(x)                       ( ((x) == LORAWAN_CH_RU1_868_78_MHZ) || \
                                                  ((x) == LORAWAN_CH_RU1_868_95_MHZ) || \
                                                  ((x) == LORAWAN_CH_RU1_869_12_MHZ) || \
                                                  ((x) == LORAWAN_CH_RU1_864_1_MHZ) || \
                                                  ((x) == LORAWAN_CH_RU1_864_3_MHZ) || \
                                                  ((x) == LORAWAN_CH_RU1_864_5_MHZ) || \
                                                  ((x) == LORAWAN_CH_RU1_864_7_MHZ) || \
                                                  ((x) == LORAWAN_CH_RU1_864_9_MHZ) )


//! @endcond


/**
 *  @brief LoRaWAN RUSSIA channel map ONE
 */
typedef enum TLoRaWAN_Channel_RussiaOne
{
    LoRaWAN_Channel_Russia1_868_78_Mhz   = 0,                                   /*!< carrier frequency of 864.100 MHz */
    LoRaWAN_Channel_Russia1_868_95_Mhz   = 1,                                   /*!< carrier frequency of 864.300 MHz */
	LoRaWAN_Channel_Russia1_869_12_Mhz   = 2,                                   /*!< carrier frequency of 864.500 MHz */
    LoRaWAN_Channel_Russia1_864_1_Mhz    = 3,                                   /*!< carrier frequency of 864.700 MHz */
    LoRaWAN_Channel_Russia1_864_3_Mhz    = 4,                                   /*!< carrier frequency of 868.780 MHz */
    LoRaWAN_Channel_Russia1_864_5_Mhz    = 5,                                   /*!< carrier frequency of 868.950 MHz */
    LoRaWAN_Channel_Russia1_864_7_Mhz    = 6,                                   /*!< carrier frequency of 869.120 MHz */
    LoRaWAN_Channel_Russia1_864_9_Mhz    = 128,                                 /*!< carrier frequency of 864.900 MHz */
} TLoRaWAN_Channel_RussiaOne;


//! @cond Doxygen_Suppress

// LoRa Channel Index ( Russia 2nd config)

#define LORAWAN_CH_RU2_868_9_MHZ                   0
#define LORAWAN_CH_RU2_868_7_MHZ                   1
#define LORAWAN_CH_RU2_869_1_MHZ                   128

#define IS_LORAWAN_CH_RU2(x)                       ( ((x) == LORAWAN_CH_RU2_868_9_MHZ) || \
                                                  ((x) == LORAWAN_CH_RU2_868_7_MHZ) || \
                                                  ((x) == LORAWAN_CH_RU2_869_1_MHZ) )


//! @endcond

/**
 *  @brief LoRaWAN RUSSIA channel map TWO
 */
typedef enum TLoRaWAN_Channel_RussiaTwo
{
    LoRaWAN_Channel_Russia2_868_9_Mhz   = 0,                                    /*!< carrier frequency of 868.900 MHz */
    LoRaWAN_Channel_Russia2_868_7_Mhz   = 1,                                    /*!< carrier frequency of 868.700 MHz */
    LoRaWAN_Channel_Russia2_869_1_Mhz   = 128,                                  /*!< carrier frequency of 869.100 MHz */
} TLoRaWAN_Channel_RussiaTwo;

//! @cond Doxygen_Suppress

// LoRa Channel Index ( Russia 3rd config)
#define LORAWAN_CH_RU3_868_78_MHZ                   0
#define LORAWAN_CH_RU3_868_95_MHZ                   1
#define LORAWAN_CH_RU3_869_12_MHZ                   128

#define IS_LORAWAN_CH_RU3(x)                       ( ((x) == LORAWAN_CH_RU3_868_78_MHZ) || \
                                                  ((x) == LORAWAN_CH_RU3_868_95_MHZ) || \
                                                  ((x) == LORAWAN_CH_RU3_869_12_MHZ) )

//! @endcond

/**
 *  @brief LoRaWAN RUSSIA channel map THREE
 */
typedef enum TLoRaWAN_Channel_RussiaThree
{
    LoRaWAN_Channel_Russia3_868_78_Mhz   = 0,                                   /*!< carrier frequency of 868.780 MHz */
    LoRaWAN_Channel_Russia3_868_95_Mhz   = 1,                                   /*!< carrier frequency of 868.950 MHz */
    LoRaWAN_Channel_Russia3_869_12_Mhz   = 128,                                 /*!< carrier frequency of 869.120 MHz */
} TLoRaWAN_Channel_RussiaThree;


//! @cond Doxygen_Suppress

// LoRa Channel Index ( Russia 4th config)
#define LORAWAN_CH_RU4_864_1_MHZ                   0
#define LORAWAN_CH_RU4_864_3_MHZ                   1
#define LORAWAN_CH_RU4_864_5_MHZ                   2
#define LORAWAN_CH_RU4_864_64_MHZ                  3
#define LORAWAN_CH_RU4_864_78_MHZ                  4
#define LORAWAN_CH_RU4_868_78_MHZ                  5
#define LORAWAN_CH_RU4_868_95_MHZ                  6
#define LORAWAN_CH_RU4_869_12_MHZ                  7
#define LORAWAN_CH_RU4_864_92_MHZ                  128


#define IS_LORAWAN_CH_RU4(x)                       ( ((x) ==  LORAWAN_CH_RU4_864_1_MHZ  || \
													 ((x) == LORAWAN_CH_RU4_864_3_MHZ   || \
													 ((x) == LORAWAN_CH_RU4_864_5_MHZ   || \
													 ((x) == LORAWAN_CH_RU4_864_64_MHZ  || \
												     ((x) == LORAWAN_CH_RU4_864_78_MHZ  || \
												     ((x) == LORAWAN_CH_RU4_868_78_MHZ  || \
												     ((x) == LORAWAN_CH_RU4_868_95_MHZ  || \
												     ((x) == LORAWAN_CH_RU4_869_12_MHZ  || \
													 ((x) == LORAWAN_CH_RU4_864_92_MHZ )

//! @endcond

/**
 *  @brief LoRaWAN RUSSIA channel map FOUR
 */
typedef enum TLoRaWAN_Channel_RussiaFour
{
    LoRaWAN_Channel_Russia4_864_1_Mhz   = 0,                                    /*!< carrier frequency of 864.100 MHz */
    LoRaWAN_Channel_Russia4_864_3_Mhz   = 1,                                    /*!< carrier frequency of 864.300 MHz */
    LoRaWAN_Channel_Russia4_869_5_Mhz   = 2,                                    /*!< carrier frequency of 864.500 MHz */

    LoRaWAN_Channel_Russia4_864_64_Mhz  = 3,                                    /*!< carrier frequency of 864.640 MHz */
    LoRaWAN_Channel_Russia4_864_78_Mhz  = 4,                                    /*!< carrier frequency of 864.780 MHz */
    LoRaWAN_Channel_Russia4_868_78_Mhz  = 5,                                    /*!< carrier frequency of 868.780 MHz */
    LoRaWAN_Channel_Russia4_868_95_Mhz  = 6,                                    /*!< carrier frequency of 868.950 MHz */
    LoRaWAN_Channel_Russia4_869_12_Mhz  = 7,                                    /*!< carrier frequency of 869.120 MHz */
    LoRaWAN_Channel_Russia4_864_92_Mhz  = 128,                                  /*!< carrier frequency of 864.920 MHz */

} TLoRaWAN_Channel_RussiaFour;

//! @cond Doxygen_Suppress

// LoRa Channel Index ( Russia 5th config)
#define LORAWAN_CH_RU5_864_5_MHZ                   0
#define LORAWAN_CH_RU5_864_7_MHZ                   1
#define LORAWAN_CH_RU5_864_9_MHZ                   2
#define LORAWAN_CH_RU5_864_1_MHZ                   3
#define LORAWAN_CH_RU5_864_3_MHZ                   4
#define LORAWAN_CH_RU5_868_8_MHZ                   5
#define LORAWAN_CH_RU5_869_05_MHZ                  128


#define IS_LORAWAN_CH_RU5(x)                       ( ((x) ==  LORAWAN_CH_RU5_864_5_MHZ  || \
													 ((x) == LORAWAN_CH_RU5_864_7_MHZ   || \
													 ((x) == LORAWAN_CH_RU5_864_9_MHZ   || \
													 ((x) == LORAWAN_CH_RU5_864_1_MHZ  || \
												     ((x) == LORAWAN_CH_RU5_864_3_MHZ  || \
												     ((x) == LORAWAN_CH_RU5_868_8_MHZ  || \
													 ((x) == LORAWAN_CH_RU5_869_05_MHZ )

//! @endcond

/**
 *  @brief LoRaWAN RUSSIA channel map FIVE
 */
typedef enum TLoRaWAN_Channel_RussiaFive
{
    LoRaWAN_Channel_Russia5_864_5_Mhz   = 0,                                    /*!< carrier frequency of 864.500 MHz */
    LoRaWAN_Channel_Russia5_864_7_Mhz   = 1,                                    /*!< carrier frequency of 864.700 MHz */
    LoRaWAN_Channel_Russia5_864_9_Mhz   = 2,                                    /*!< carrier frequency of 864.800 MHz */
    LoRaWAN_Channel_Russia5_864_1_Mhz   = 3,                                    /*!< carrier frequency of 864.100 MHz */
    LoRaWAN_Channel_Russia5_864_3_Mhz   = 4,                                    /*!< carrier frequency of 864.300 MHz */
    LoRaWAN_Channel_Russia5_868_8_Mhz   = 5,                                    /*!< carrier frequency of 868.800 MHz */
    LoRaWAN_Channel_Russia5_869_05_Mhz  = 128,                                  /*!< carrier frequency of 869.050 MHz */

} TLoRaWAN_Channel_RussiaFive;


//! @cond Doxygen_Suppress

// LoRa Channel Index ( Russia 6th config)
#define LORAWAN_CH_RU6_868_0_MHZ                   0
#define LORAWAN_CH_RU6_860_1_MHZ                   1
#define LORAWAN_CH_RU6_864_1_MHZ                   2
#define LORAWAN_CH_RU6_864_3_MHZ                   3
#define LORAWAN_CH_RU6_864_5_MHZ                   4
#define LORAWAN_CH_RU6_864_7_MHZ                   5
#define LORAWAN_CH_RU6_864_9_MHZ                   6
#define LORAWAN_CH_RU6_869_1_MHZ_2                   128


#define IS_LORAWAN_CH_RU6(x)                       ( ((x) ==  LORAWAN_CH_RU6_868_0_MHZ  || \
													 ((x) == LORAWAN_CH_RU6_860_1_MHZ   || \
													 ((x) == LORAWAN_CH_RU6_864_1_MHZ   || \
													 ((x) == LORAWAN_CH_RU6_864_3_MHZ  || \
												     ((x) == LORAWAN_CH_RU6_864_5_MHZ  || \
												     ((x) == LORAWAN_CH_RU6_864_7_MHZ  || \
												     ((x) == LORAWAN_CH_RU6_864_9_MHZ  || \
													 ((x) == LORAWAN_CH_RU6_869_1_MHZ_2 )

//! @endcond

/**
 *  @brief LoRaWAN RUSSIA channel map SIX
 */
typedef enum TLoRaWAN_Channel_RussiaSix
{
    LoRaWAN_Channel_Russia6_868_9_Mhz   = 0,                                    /*!< carrier frequency of 868.900 MHz */
    LoRaWAN_Channel_Russia6_869_1_Mhz   = 1,                                    /*!< carrier frequency of 869.100 MHz */
    LoRaWAN_Channel_Russia6_864_1_Mhz   = 2,                                    /*!< carrier frequency of 864.100 MHz */
    LoRaWAN_Channel_Russia6_864_3_Mhz   = 3,                                    /*!< carrier frequency of 864.300 MHz */
    LoRaWAN_Channel_Russia6_864_5_Mhz   = 4,                                    /*!< carrier frequency of 864.500 MHz */
    LoRaWAN_Channel_Russia6_864_7_Mhz   = 5,                                    /*!< carrier frequency of 864.700 MHz */
    LoRaWAN_Channel_Russia6_864_9_Mhz   = 6,                                    /*!< carrier frequency of 864.900 MHz */
    LoRaWAN_Channel_Russia6_869_1_Mhz_2   = 128,                                /*!< carrier frequency of 869.100 MHz */

} TLoRaWAN_Channel_RussiaSix;


//! @cond Doxygen_Suppress

// LoRa Channel Index ( Russia 7th config)
#define LORAWAN_CH_RU7_868_9_MHZ                   0
#define LORAWAN_CH_RU7_869_1_MHZ                   1
#define LORAWAN_CH_RU7_869_1_MHZ_2                 128

#define IS_LORAWAN_CH_RU7(x)                       ( ((x) ==  LORAWAN_CH_RU7_868_9_MHZ  || \
													 ((x) == LORAWAN_CH_RU7_869_1_MHZ   || \
													 ((x) == LORAWAN_CH_RU7_869_1_MHZ_2 )

//! @endcond

/**
 *  @brief LoRaWAN RUSSIA channel map SEVEN
 */
typedef enum TLoRaWAN_Channel_RussiaSeven
{
    LoRaWAN_Channel_Russia7_868_9_Mhz   = 0,                                    /*!< carrier frequency of 868.900 MHz */
    LoRaWAN_Channel_Russia7_869_1_Mhz   = 1,                                    /*!< carrier frequency of 869.100 MHz */
    LoRaWAN_Channel_Russia7_869_1_Mhz_2   = 128,                                /*!< carrier frequency of 869.100 MHz */

} TLoRaWAN_Channel_RussiaSeven;



/**
 * @brief LoRa data rate definitions for Russia 868 MHz Band
 */
typedef enum TLoRaWANDataRateRU868
{
    LoRaWAN_DataRate_RU868_LoRa_SF12_125kHz = 0,                                /*!< LoRa Spreading factor 12 using 125kHz bandwidth */
    LoRaWAN_DataRate_RU868_LoRa_SF11_125kHz,                                    /*!< LoRa Spreading factor 11 using 125kHz bandwidth */
    LoRaWAN_DataRate_RU868_LoRa_SF10_125kHz,                                    /*!< LoRa Spreading factor 10 using 125kHz bandwidth */
    LoRaWAN_DataRate_RU868_LoRa_SF9_125kHz,                                     /*!< LoRa Spreading factor  9 using 125kHz bandwidth */
    LoRaWAN_DataRate_RU868_LoRa_SF8_125kHz,                                     /*!< LoRa Spreading factor  8 using 125kHz bandwidth */
    LoRaWAN_DataRate_RU868_LoRa_SF7_125kHz,                                     /*!< LoRa Spreading factor  7 using 125kHz bandwidth */
    LoRaWAN_DataRate_RU868_LoRa_SF7_250kHz,                                     /*!< LoRa Spreading factor  7 using 250kHz bandwidth */
    LoRaWAN_DataRate_RU868_LoRa_FSK,                                            /*!< FSK modulation; LoRa settings not applicable    */
} TLoRaWANDataRateRU868;


#endif /* ARDUINO_SAP_WIMOD_SAP_LORAWAN_IDS_RU868_H_ */
