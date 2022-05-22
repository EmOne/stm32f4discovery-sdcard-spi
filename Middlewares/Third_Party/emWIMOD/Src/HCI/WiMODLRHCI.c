//------------------------------------------------------------------------------
//! @file WiMODLRHCI.cpp
//! @ingroup WiMODLRHCI
//! <!------------------------------------------------------------------------->
//! @brief Common low level HCI message processing base
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

//------------------------------------------------------------------------------
//
// Section Includes Files
//
//------------------------------------------------------------------------------

#include <emod_uart.h>
#include "WiMODLRHCI.h"
#include "../utils/CRC16.h"


//------------------------------------------------------------------------------
//
// Section public functions
//
//------------------------------------------------------------------------------
static void Begin(void* handler);
static void End(void);
static TWiMODLRResultCodes sendHCIMessage(UINT8 dstSapID, UINT8 msgID, UINT8 rxMsgID, UINT8* payload, UINT16 length);
static TWiMODLRResultCodes sendHCIMessageWithoutRx(UINT8 dstSapID, UINT8 msgID,  UINT8* payload, UINT16 length);
static void process(void);

static void sendWakeUpSequence(void);
static void registerStackErrorClient(TWiMODStackErrorClient cb);
static TWiMODLR_HCIMessage* getRxMessage(void);
static void enableWakeupSequence(bool flag);
static TWiMODLRResultCodes postMessage(UINT8 sapID, UINT8 msgID, UINT8* payload, UINT16 length);
static TWiMODLRResultCodes sendPacket(UINT8* txData, UINT16 length);
static bool waitForResponse(UINT8 rxSapID, UINT8 rxMsgID);
static void dispatchRxMessage(TWiMODLR_HCIMessage* rxMsg);

/**
 * @brief Constructor
 *
 * @param s     Reference to the serial interface the the WiMOD. The interface
 *              must be initialized before any other function of this class
 *              can be used.
 */

TWiMODLRHCI_t TWiMODLRHCI = {

		Begin,
		End,
		sendHCIMessage,
		sendHCIMessageWithoutRx,
		process,
		sendWakeUpSequence,
		registerStackErrorClient,
		getRxMessage,
		enableWakeupSequence,
		postMessage,
		sendPacket,
		waitForResponse,

};

//-----------------------------------------------------------------------------
/**
 * @brief Destructor
 */
//TWiMODLRHCI::~TWiMODLRHCI(void)
//{
//
//}

//------------------------------------------------------------------------------

/**
 * @brief Init function of the generic HCI message handler.
 *
 * This function must be called once before any other service can be used.
 */
void Begin(void* handler) {

//	TWiMODLRHCI.serial->begin(WIMODLR_SERIAL_BAUDRATE);
	TWiMODLRHCI.StackErrorClientCB = 0x00;

	TWiMODLRHCI.Rx.Active 		   = false;
	TWiMODLRHCI.Rx.Done   		   = false;
	TWiMODLRHCI.Rx.SapID           = 0x00;
	TWiMODLRHCI.Rx.MsgID           = 0x00;
	TWiMODLRHCI.Rx.Timeout 		   = WIMODLR_RESPOMSE_TIMEOUT_MS;

	TWiMODLRHCI.TxMessage.Length    = 0x00;
	TWiMODLRHCI.TxMessage.SapID     = 0x00;
	TWiMODLRHCI.DispatchRxMessage = dispatchRxMessage;

    // register for rx-messages
	TWiMODLRHCI.serial = &Serial;
	TWiMODLRHCI.comSlip = &TComSlip;
	TWiMODLRHCI.comSlip->begin(handler, WIMODLR_SERIAL_BAUDRATE);
	TWiMODLRHCI.comSlip->RegisterClient = registerClient;
	TWiMODLRHCI.comSlip->RegisterClient((TComSlipClient*) processRxMessage);
//	TWiMODLRHCI.comSlip->RxClient->ProcessRxMessage = processRxMessage;
	TWiMODLRHCI.comSlip->SetRxBuffer(&TWiMODLRHCI.Rx.Message.SapID, WIMODLR_HCI_RX_MESSAGE_SIZE);
	TWiMODLRHCI.wakeUp = true;
}

//void registerClient(TComSlipClient* client)
//{
//	RegisterClient
//}
//-----------------------------------------------------------------------------
/**
 * @brief shutdown function
 */
void End(void) {
	TWiMODLRHCI.comSlip->end();
}

//-----------------------------------------------------------------------------
/**
 * @brief Generic function for transferring a HCI message to the WiMOD module
 *
 * @param   dstSapID    the SAP endpoint to address
 * @param   msgID       the command ID to address within the SAP
 * @param   rxMsgID     the expected response ID according to the msgID
 * @param   payload     pointer to the payload bytes to send
 * @param   length      the number of payload bytes to send
 *
 * @retval WiMODLR_RESULT_OK    if everything was OK
 */
TWiMODLRResultCodes sendHCIMessage(UINT8 dstSapID, UINT8 msgID, UINT8 rxMsgID, UINT8* payload, UINT16 length) {

    // send wakeup sequence to get the WiMOD out of sleep ?
    if (TWiMODLRHCI.wakeUp) {
    	TWiMODLRHCI.SendWakeUpSequence();
    }

    // send message
    TWiMODLRResultCodes result = TWiMODLRHCI.PostMessage(dstSapID, msgID, payload, length);

    // message sent ?
    if (result == WiMODLR_RESULT_OK)
    {
        // yes, wait for response from radio module
        if (TWiMODLRHCI.WaitForResponse(dstSapID, rxMsgID))
        {
            return WiMODLR_RESULT_OK;
        }
        return WiMODLR_RESULT_NO_RESPONSE;
    }
    // return error
    return result;
}

//-----------------------------------------------------------------------------
/**
 * @brief Generic function for transferring a HCI message to the WiMOD module
 *
 * @param   dstSapID    the SAP endpoint to address
 * @param   msgID       the command ID to address within the SAP
 * @param   payload     pointer to the payload bytes to send
 * @param   length      the number of payload bytes to send
 *
 * @retval WiMODLR_RESULT_OK    if everything was OK
 */
TWiMODLRResultCodes sendHCIMessageWithoutRx(UINT8 dstSapID, UINT8 msgID,  UINT8* payload, UINT16 length) {

    // send message
    TWiMODLRResultCodes result = TWiMODLRHCI.PostMessage(dstSapID, msgID, payload, length);

    // return error
    return result;
}

//-----------------------------------------------------------------------------
/**
 * @brief Handle the receiver path; process all incomming bytes from the WiMOD
 *
 * This function checks if there are any bytes from the WiMOD available and
 * tries to start decoding the received data.
 *
 * @note this function must be called at regular base from the main loop.
 */
void process(void)
{
    // read data from comport
//    int numRxBytes = TWiMODLRHCI.serial->available();
    UINT16 rxByte;

    // bytes received ?
//    while(numRxBytes-- > 0) {
        rxByte = TWiMODLRHCI.serial->read();
        // yes, pass to SLIP Decoder
        // Complete SLIP messages will be forwarded via callback to
        // callback function "ProcessRxMessage" (see Receiver section)
        if(rxByte) {
              // init receiver struct
        	  TWiMODLRHCI.Rx.Active = true;
        	  TWiMODLRHCI.Rx.Done   = false;
        	  TWiMODLRHCI.Rx.Timeout = WIMODLR_RESPOMSE_TIMEOUT_MS;
        	  TWiMODLRHCI.comSlip->DecodeData(&Rx2_buffer[0], rxByte);
		}

//    }
}

//------------------------------------------------------------------------------
/**
 * @brief: Send a sequence of dummy chars to give the WiMOD some time to wake up
 */
void sendWakeUpSequence(void)
{
	TWiMODLRHCI.comSlip->SendWakeUpSequence(WIMODLR_NUMBER_OF_WAKEUP_CHARS);
}

//------------------------------------------------------------------------------
/**
 * @internal
 *
 * @brief: Enable / Disable transmitting a wakeup sequence before every command request
 *
 * @param flag  flag for enabling / disabling the wakeup sequence (true = enable)
 *
 * @endinternal
 */
void enableWakeupSequence(bool flag) {
	TWiMODLRHCI.wakeUp = flag;
}

//------------------------------------------------------------------------------
//
//  ProcessRxMessage
//------------------------------------------------------------------------------

/**
 * @internal
 *
 * @brief: handle received SLIP message
 *
 * @param rxBuffer  pointer to memory containing rx data bytes
 *
 * @param length    number of bytes in the buffer to process
 *
 * @return  pointer to memory that can store the next hci message for RX operation
 *
 * @endinternal
 */
UINT8* processRxMessage(UINT8* rxBuffer, UINT16 length)
{
    // 1. check CRC
    if (CRC16_Check(rxBuffer, length, CRC16_INIT_VALUE))
    {
        // 2. check min length, 2 bytes for SapID + MsgID + 2 bytes CRC16
        if(length >= (WIMODLR_HCI_MSG_HEADER_SIZE + WIMODLR_HCI_MSG_FCS_SIZE))
        {
            // 3. Hack: since only one RxMessage buffer is used,
            //          rxBuffer must point to RxMessage.SapId, thus
            //          memcpy to RxMessage structure is not needed here

            // add length
        	TWiMODLRHCI.Rx.Message.Length = length - (WIMODLR_HCI_MSG_HEADER_SIZE + WIMODLR_HCI_MSG_FCS_SIZE);
        	TWiMODLRHCI.Rx.SapID = TWiMODLRHCI.Rx.Message.SapID;
        	TWiMODLRHCI.Rx.MsgID = TWiMODLRHCI.Rx.Message.MsgID;
            // dispatch completed RxMessage
        	TWiMODLRHCI.DispatchRxMessage(&TWiMODLRHCI.Rx.Message);
        }
    }
    else
    {
        //CRC error ??

    }

    // return same buffer again, keep receiver enabled
    return &TWiMODLRHCI.Rx.Message.SapID;
}

//-----------------------------------------------------------------------------
/**
 * @internal
 *
 * @brief Registers a callback function for reporting an internal stack error
 *
 * @paramc  cb          pointer callback function
 *
 *@endinternal
 */
void registerStackErrorClient(TWiMODStackErrorClient cb)
{
	TWiMODLRHCI.StackErrorClientCB = cb;
}

//-----------------------------------------------------------------------------
/**
 * @internal
 *
 * @brief Returns the last HCI message received by the low level stack
 *
 * @return  reference to the last received HCI message
 *
 * @endinternal
 */
TWiMODLR_HCIMessage* getRxMessage(void)
{
#ifdef WIMOD_USE_2ND_RXBUFFER
// for non IRQ mode
    return &TWiMODLRHCI.Rx.ExpectedRsponseMsg;
#else
// for IRQ mode
    return Rx.Message;
#endif
}


//------------------------------------------------------------------------------
//
// Section protected functions
//
//------------------------------------------------------------------------------

//-----------------------------------------------------------------------------
/**
 * @internal
 *
 * @brief Construct a HCI message and send it to the WiMOD
 *
 * @param   sapID   the SAP-ID to target
 * @param   msgID   the Command ID to target
 * @param   payload pointer to the payload
 * @param   length  length of the payload field to use
 *
 * @endinternal
 */
TWiMODLRResultCodes postMessage(UINT8 sapID, UINT8 msgID, UINT8* payload, UINT16 length)
{
    // 1. check parameter
    //
    // 1.1 check length
    //
    if(length > WIMODLR_HCI_MSG_PAYLOAD_SIZE)
    {
        return WiMODLR_RESULT_PAYLOAD_LENGTH_ERROR;
    }
    // 1.2 check payload ptr
    //
    if(length && !payload)
    {
        return WiMODLR_RESULT_PAYLOAD_PTR_ERROR;
    }

    // 2.  init TxMessage
    //
    // 2.1 init SAP ID
    //
    TWiMODLRHCI.TxMessage.SapID = sapID;

    // 2.2 init Msg ID
    //
    TWiMODLRHCI.TxMessage.MsgID = msgID;

    // 2.3 copy payload, if present
    //
    if(payload && length)
    {
        UINT8*  dstPtr  = TWiMODLRHCI.TxMessage.Payload;
        int     n       = (int)length;

        // copy bytes
        while(n--)
            *dstPtr++ = *payload++;
    }

    // 3. Calculate CRC16 over header and optional payload
    //
    UINT16 crc16 = CRC16_Calc(&TWiMODLRHCI.TxMessage.SapID, length + WIMODLR_HCI_MSG_HEADER_SIZE, CRC16_INIT_VALUE);

    // 3.1 get 1's complement
    //
    crc16 = ~crc16;

    // 3.2 attach CRC16 and correct length, lobyte first
    //
    TWiMODLRHCI.TxMessage.Payload[length++] = LOBYTE(crc16);
    TWiMODLRHCI.TxMessage.Payload[length++] = HIBYTE(crc16);

    // 4. forward message to SLIP layer
    //    - start transmission with SAP ID
    //    - correct length by header size

    return TWiMODLRHCI.SendPacket(&TWiMODLRHCI.TxMessage.SapID, length + WIMODLR_HCI_MSG_HEADER_SIZE);
}


/**
 * @internal
 *
 * @brief Send a complete packet (Sap+Msg+Payload+CRC) to WiMOD.
 *
 * @param txData    pointer to COMPLETE HCI packet
 *
 * @param length    length of the HCI packet
 *
 * @endinternal
 */
TWiMODLRResultCodes sendPacket(UINT8* txData, UINT16 length)
{
    // call SLIP encoder
    // and send out data via serial interface
	TWiMODLRHCI.comSlip->SendMessage(txData, length);

    return WiMODLR_RESULT_OK;
}



//------------------------------------------------------------------------------
/**
 * @internal
 *
 * @brief: wait for response HCI message
 *
 * @param rxSapID   the SapID of the expected response message
 *
 * @param rxMsgID   the CmdID of the expected response message
 *
 * @return true     if the expected response message has been received within timeout
 *
 * @endinternal
 */
bool waitForResponse(UINT8 rxSapID, UINT8 rxMsgID)
{
    // init receiver struct
	TWiMODLRHCI.Rx.Active = true;
	TWiMODLRHCI.Rx.Done   = false;
	TWiMODLRHCI.Rx.SapID  = rxSapID;
	TWiMODLRHCI.Rx.MsgID  = rxMsgID;
	TWiMODLRHCI.Rx.Timeout = WIMODLR_RESPOMSE_TIMEOUT_MS;

    // wait for response ~1000ms
    while(TWiMODLRHCI.Rx.Timeout--)
    {
        // call receiver path
    	TWiMODLRHCI.Process();

        // response received  ?
        if(TWiMODLRHCI.Rx.Done)
        {
            // clear flag
        	TWiMODLRHCI.Rx.Active = false;

            // ok
            return true;
        }
//        delay(1);
    }
    // clear flag
    TWiMODLRHCI.Rx.Active = false;

    // error - timeout
    return false;
}

//------------------------------------------------------------------------------
//
// Section private functions
//
//------------------------------------------------------------------------------

/**
 * @internal
 *
 * @brief Dispatch a a CRC checked HCI message for further processing
 *
 * @param rxMsg     reference to the received HCI message
 *
 * @endinternal
 */
void dispatchRxMessage(TWiMODLR_HCIMessage* rxMsg)
{
    // 1. test if a response message is expected
    if(TWiMODLRHCI.Rx.Active)
    {
        // expected response received ?
        if ((rxMsg->SapID == TWiMODLRHCI.Rx.SapID) && (rxMsg->MsgID == TWiMODLRHCI.Rx.MsgID))
        {
            // yes
        	TWiMODLRHCI.Rx.Done = true;

#ifdef WIMOD_USE_2ND_RXBUFFER
            // store this message in case there are several messages at once in the uart fifo - only for non IRQ mode!
        	TWiMODLRHCI.Rx.ExpectedRsponseMsg = *rxMsg;
#endif
            // no further processing here !
            return;
        }
    }

    // 2. forward async received messages to corresponding SAP
    TWiMODLRHCI.RxMessageClient->ProcessRxMessage(rxMsg);
    TWiMODLRHCI.ProcessUnexpectedRxMessage(rxMsg);
}

//-----------------------------------------------------------------------------
// EOF
//-----------------------------------------------------------------------------



