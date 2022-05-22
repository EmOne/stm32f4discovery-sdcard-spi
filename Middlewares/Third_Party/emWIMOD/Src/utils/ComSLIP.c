//------------------------------------------------------------------------------
//
//  File:       ComSlip.c
//
//  Abstract:   SLIP Wrapper Class Implementation
//
//  Version:    0.1
//
//  Date:       09.12.2021
//
//  Disclaimer: This example code is provided by EmOne
//
//------------------------------------------------------------------------------
//! @file ComSlip.c
//! <!------------------------------------------------------------------------->
//! @brief SLIP layer for the HCI communication stack
//! @version 0.1
//! <!------------------------------------------------------------------------->

//------------------------------------------------------------------------------
//
//  Include Files
//
//------------------------------------------------------------------------------
#include "WiMODLRHCI.h"
#include "ComSLIP.h"

//------------------------------------------------------------------------------
//  Protocol Definitions
//------------------------------------------------------------------------------

static void Begin(void* handler, UINT32 baudrate);
static void End(void);
static bool SendMessage(UINT8* msg, UINT16 msgLength);
static void SendWakeUpSequence(UINT8 nbr);
static void StoreRxByte(UINT8 rxByte);
//------------------------------------------------------------------------------

// SLIP Protocol Characters
#define SLIP_END                    0xC0
#define SLIP_ESC                    0xDB
#define SLIP_ESC_END                0xDC
#define SLIP_ESC_ESC                0xDD

// SLIP Receiver/Decoder States
#define SLIPDEC_IDLE_STATE          0
#define SLIPDEC_START_STATE         1
#define SLIPDEC_IN_FRAME_STATE      2
#define SLIPDEC_ESC_STATE           3

//------------------------------------------------------------------------------
//TComSlipClient RegClient = {};

TComSlip_t TComSlip = {
		Begin,
		End,
		registerClient,
		SendMessage,
		SetRxBuffer,
		DecodeData,
		SendWakeUpSequence,
		StoreRxByte,
//		SLIPDEC_IDLE_STATE,
//		0x0,
//		0x0,
//		NULL,
//		NULL,
};
//-----------------------------------------------------------------------------
/**
 * @brief Constructor
 *
 * @param s     Reference to the Serial interface to use for communication
 */
//TComSlip TComSlip(Stream& s) {

//	Serial(s);
//    // init to idle state, no rx-buffer available
//    RxState         =   SLIPDEC_IDLE_STATE;
//    RxIndex         =   0;
//   RxBuffer        =   0;
//    RxBufferSize    =   0;
//    RxClient        =   0;
//}

//UINT8* ProcessRxMessage(UINT8* rxBuffer, UINT16 rxLength)
//{
//	UINT8* pbuffer;
//	pbuffer = rxBuffer;
//
//	return pbuffer;
//}

/**
 * @brief Init function that must be called once prior to any other function of this object
 * @Fon need to fix this further
 */
void Begin(void* handler, UINT32 baudrate)
{
    Serial.begin(handler, baudrate);

    // init to idle state, no rx-buffer available
    TComSlip.RxState         =   SLIPDEC_IDLE_STATE;
    TComSlip.RxIndex         =   0;
    TComSlip.RxBuffer        =   0;
    TComSlip.RxBufferSize    =   0;
    TComSlip.RxClient        =   0;
}


/**
 * @brief De-Init function
 */
void End(void) {

}

/**
 * @brief register a client for processing
 *
 * @param client  pointer to client that should handle a received SLIP frame
 */
void registerClient(TComSlipClient* client)
{
    TComSlip.RxClient = client;
}


//------------------------------------------------------------------------------
//
//  SendMessage
//
//  @brief: send a message as SLIP frame
//
//------------------------------------------------------------------------------

/**
 * @brief Generic function to transfer a message from the host to the module.
 *
 * This is a generic function to send out a stream of bytes. The bytes are
 * encoded into the SLIP format before transmission. Therefore a complete
 * message has to be given.
 *
 * @param msg       pointer to the bytes to encoded and send via Serial interface
 * @param msgLength number of bytes
 *
 */
bool SendMessage(UINT8* msg, UINT16 msgLength)
{
    // send start of SLIP message
    Serial.write(SLIP_END);

    // iterate over all message bytes
    while(msgLength--)
    {
        switch (*msg)
        {
            case SLIP_END:
                Serial.write(SLIP_ESC);
                Serial.write(SLIP_ESC_END);
                break;

            case SLIP_ESC:
                Serial.write(SLIP_ESC);
                Serial.write(SLIP_ESC_ESC);
                break;

            default:
                Serial.write(*msg);
                break;
        }
        // next byte
        msg++;
    }

    // send end of SLIP message
    Serial.write(SLIP_END);

    // always ok
    return true;
}


/**
 * @brief configure a rx-buffer and enable receiver/decoder
 *
 * @param rxBuffer  pointer to plain buffer
 *
 * @param rxBufferSize  size of buffer in bytes
 */
bool SetRxBuffer(UINT8* rxBuffer, UINT16  rxBufferSize)
{
    // receiver in IDLE state and client already registered ?
    if ((TComSlip.RxState == SLIPDEC_IDLE_STATE) && TComSlip.RxClient)
    {
        // same buffer params
    	TComSlip.RxBuffer        = rxBuffer;
    	TComSlip.RxBufferSize    = rxBufferSize;

        // enable decoder
    	TComSlip.RxState = SLIPDEC_START_STATE;

        return true;
    }
    return false;
}

//------------------------------------------------------------------------------
/**
 * @brief process received byte stream
 *
 * @param rxData    pointer to received bytes
 * @param length    number of bytes received
 */
void DecodeData(UINT8* rxData, UINT16 length)
{
    // iterate over all received bytes
    while(length--)
    {
        // get rxByte
        UINT8 rxByte = *rxData++;

        // decode according to current state
        switch(TComSlip.RxState)
        {
            case    SLIPDEC_START_STATE:
                    // start of SLIP frame ?
                    if(rxByte == SLIP_END)
                    {
                        // init read index
                    	TComSlip.RxIndex = 0;

                        // next state
                    	TComSlip.RxState = SLIPDEC_IN_FRAME_STATE;
                    }
                    break;

            case    SLIPDEC_IN_FRAME_STATE:
                    switch(rxByte)
                    {
                        case SLIP_END:
                                // data received ?
                                if(TComSlip.RxIndex > 0)
                                {
                                    // yes, return received decoded length
                                    if (TComSlip.RxClient)
                                    {
                                    	TWiMODLRHCI.Rx.SapID = *processRxMessage(TComSlip.RxBuffer, TComSlip.RxIndex);

                                        if (!TComSlip.RxBuffer)
                                        {
                                        	TComSlip.RxState = SLIPDEC_IDLE_STATE;
                                        }
                                        else
                                        {
                                        	TComSlip.RxState = SLIPDEC_START_STATE;
                                        }
                                    }
                                    else
                                    {
                                        // disable decoder, temp. no buffer available
                                    	TComSlip.RxState = SLIPDEC_IDLE_STATE;
                                    }
                                }
                                // init read index
                                TComSlip.RxIndex = 0;
                                break;

                        case SLIP_ESC:
                                // enter escape sequence state
                        		TComSlip.RxState = SLIPDEC_ESC_STATE;
                                break;

                        default:
                                // store byte
                                StoreRxByte(rxByte);
                                break;
                    }
                    break;

            case    SLIPDEC_ESC_STATE:
                    switch(rxByte)
                    {
                        case    SLIP_ESC_END:
                                StoreRxByte(SLIP_END);
                                // quit escape sequence state
                                TComSlip.RxState = SLIPDEC_IN_FRAME_STATE;
                                break;

                        case    SLIP_ESC_ESC:
                                StoreRxByte(SLIP_ESC);
                                // quit escape sequence state
                                TComSlip.RxState = SLIPDEC_IN_FRAME_STATE;
                                break;

                        default:
                                // abort frame reception
                        		TComSlip.RxState = SLIPDEC_START_STATE;
                                break;
                    }
                    break;

            default:
                    break;
        }
    }
}

//------------------------------------------------------------------------------
/**
 * @brief: store SLIP decoded rxByte
 *
 * @param rxByte    byte to store
 */
void StoreRxByte(UINT8 rxByte)
{
    if ((TComSlip.RxIndex < TComSlip.RxBufferSize) && TComSlip.RxBuffer)
    	TComSlip.RxBuffer[TComSlip.RxIndex++] = rxByte;
}



//------------------------------------------------------------------------------
/**
 * @brief: Send a sequence of dummy chars to give the WiMOD some time to wake up
 *
 * @param nbr    number of dummy bytes to send
 */
void SendWakeUpSequence(UINT8 nbr)
{
    while (nbr--) {
        Serial.write(SLIP_END);
    }
}


//------------------------------------------------------------------------------
// end of file
//------------------------------------------------------------------------------
