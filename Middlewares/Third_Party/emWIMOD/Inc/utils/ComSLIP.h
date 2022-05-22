//------------------------------------------------------------------------------
//
//  File:       ComSlip.h
//
//  Abstract:   SLIP Wrapper Class Declaration
//
//  Version:    0.1
//
//  Date:       09.12.2021
//
//  Disclaimer: This example code is provided by EmOne
//
//------------------------------------------------------------------------------
//! @file ComSlip.h
//! <!------------------------------------------------------------------------->
//! @brief SLIP layer for the HCI communication stack
//! @version 0.1
//! <!------------------------------------------------------------------------->



#ifndef COMSLIP_H
#define COMSLIP_H

//------------------------------------------------------------------------------
//
// Include Files
//
//------------------------------------------------------------------------------

#include <emod_uart.h>
#include "WMDefs.h"

//------------------------------------------------------------------------------
//
// General Definitions
//
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
//
// Class Declaration
//
//------------------------------------------------------------------------------

/**
 * @brief Class definition for enabling OO inheritance
 */
typedef struct{

    //! @cond Doxygen_Suppress
    UINT8* (*ProcessRxMessage)(UINT8* rxBuffer, UINT16 rxLength);
    //! @endcond
}TComSlipClient;

/**
 * @brief Class for handling SLIP encoding and decoding of HCI messages
 */
typedef struct{
//    public:
//    /*explicit*/ TComSlip(Stream& s);

    void (*begin)(void* handler, UINT32 baudrate);
    void (*end)(void);
    void (*RegisterClient)(TComSlipClient* client);
    bool (*SendMessage)(UINT8* msg, UINT16 msgLength);
    bool (*SetRxBuffer)(UINT8*  rxBuffer, UINT16 rxbufferSize);
    void (*DecodeData)(UINT8* rxData, UINT16 length);
    void (*SendWakeUpSequence)(UINT8 nbr);
    void (*StoreRxByte)(UINT8 rxByte);

    // receiver/decoder state
    int RxState;
    // rx buffer index
    UINT16 RxIndex;
    // size of RxBuffer
    UINT16 RxBufferSize;
    // pointer to RxBuffer
    UINT8 * RxBuffer;
    // client for received messages
    TComSlipClient* RxClient;

}TComSlip_t;

extern TComSlip_t TComSlip;
bool SetRxBuffer(UINT8* rxBuffer, UINT16  rxBufferSize);
void DecodeData(UINT8* rxData, UINT16 length);
void registerClient(TComSlipClient* client);

#endif // COMSLIP_H

//------------------------------------------------------------------------------
// end of file
//------------------------------------------------------------------------------
