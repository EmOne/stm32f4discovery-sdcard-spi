/*
 * emod_uart.h
 *
 *  Created on: Dec 19, 2021
 *      Author: HP
 */

#ifndef INC_EMOD_UART_H_
#define INC_EMOD_UART_H_

//------------------------------------------------------------------------------
//
// Include Files
//
//------------------------------------------------------------------------------

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "main.h"
#include "CRC16.h"


#define kMaxUARTPayloadSize			300U

typedef union{

    uint8_t b[kMaxUARTPayloadSize];
    uint16_t w[kMaxUARTPayloadSize/2];
    uint32_t lw[kMaxUARTPayloadSize/4];

}tCi;

typedef struct {

	tCi CmdFrmUser;
	uint16_t CmdFrmUserCount;
	uint16_t CmdFrmUserLen;
	uint8_t ReceiveFrmUserRequest;

}Ci;

extern Ci EmWimodData;

typedef struct {

	void (*begin)(void* handler, uint32_t baudrate);
	void (*end)(void);
	int (*read)(void);
	void (*write)(uint8_t data);
	void (*print)(uint8_t* data);
	int (*available)(void);

} Stream;

extern Stream  Serial;
extern uint8_t Rx2_buffer[kMaxUARTPayloadSize];
extern __IO uint8_t Rx2_byte;

/*-----------------------------------------main function-------------------------------------*/
void UART_Init (void* handler, uint32_t baudrate);
void UART_DeInit (void);
int UART_Read (void);
void UART_Write (uint8_t data);
void UART_Print (uint8_t* data);
int UART_Available( void );
uint8_t CmdFrmUserIsReceived(Ci * pData);
UINT8* ProcessRxMessage(UINT8* rxBuffer, UINT16 rxLength);

/*-----------------------------------------call back function-------------------------------------*/
void emod_RxCpltCallback(UART_HandleTypeDef *huart);
void emod_RxCpltCallbackDMA(DMA_HandleTypeDef *huart);
#endif /* INC_EMOD_UART_H_ */
