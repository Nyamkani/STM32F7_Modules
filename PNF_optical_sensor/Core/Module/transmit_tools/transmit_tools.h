/*
 * LLUart.h
 *
 *  Created on: Jul 6, 2022
 *      Author: studio3s
 */

#ifndef MODULE_TRANSMIT_TOOLS_TRANSMIT_TOOLS_H_
#define MODULE_TRANSMIT_TOOLS_TRANSMIT_TOOLS_H_


#include <stdio.h>
#include <string.h>
#include "stm32f746xx.h"
#include "stm32f7xx_hal.h"


#include <vector>
#include <queue>
#include <string>

enum CommTimeOut
{
	CommTimeOut_1ms = 1U,
	CommTimeOut_5ms = 5U,
	CommTimeOut_10ms = 10U,
};


//common transmit functions
//----------------------------------------------------------------------------------------UART
int HAL_UsartTransmit(UART_HandleTypeDef* huartx, uint8_t* todata, int datalength);
int HAL_UsartReceive(UART_HandleTypeDef* huartx, uint8_t* fromdata, int datalength);

//----------------------------------------------------------------------------------------CANOpen
int HAL_CANTransmit(CAN_HandleTypeDef* hcanx, const CAN_TxHeaderTypeDef *pHeader,
					const uint8_t *aData, uint32_t *pTxMailbox);

int HAL_CANReceive(CAN_HandleTypeDef *hcanx, CAN_RxHeaderTypeDef *pHeader, uint8_t *aData);




class HALModbusRtu
{
	//constructor & Decons.
	public:
		HALModbusRtu(UART_HandleTypeDef* huartx);
		~HALModbusRtu();


	private:
		std::queue<uint8_t*> send_data_;
		std::queue<uint8_t*> receive_data_;

		int data_size_;
		UART_HandleTypeDef* huartx_ = nullptr;


	public:
		void HALModbusRtuSingleRequest(int ids, int da, char data);
		void HALModbusRtuMultiRequest(int ids, int da, char* data, int datasize);
		int HALModbusRtuReceive();
		int HALModebusRtuSendData();

};





#endif /* MODULE_TRANSMIT_TOOLS_TRANSMIT_TOOLS_H_ */
