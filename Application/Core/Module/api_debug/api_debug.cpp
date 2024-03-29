/*
 * api_debug.cpp
 *
 *  Created on: Jul 26, 2022
 *      Author: studio3s
 */

#include "../api_debug/api_debug.h"

#include <stdio.h>
#include <stdarg.h>
#include <string.h>


/*NOTICE*/
/*We will use only USART1 for debugging*/

extern UART_HandleTypeDef huart1; 	//for debugging
extern UART_HandleTypeDef huart3;	//for Controller

static uint8_t str1_;
static uint8_t str3_;

void vprint(const char *fmt, va_list argp)
{
    char string[200];
    if(0 < vsprintf(string,fmt,argp)) // build string
    {
        //HAL_UART_Transmit(&huart1, (uint8_t*)string, strlen(string), 0xffffff); // send message via UART
    	  HAL_UART_Transmit(&huart1, (uint8_t*)string, strlen(string), 10); // send message via UART
    	//HAL_UART_Transmit(&huart3, (uint8_t*)string, strlen(string), 10); // send message via UART
    }
}

void Dprintf(const char *fmt, ...) // custom printf() function
{
    va_list argp;
    va_start(argp, fmt);
    vprint(fmt, argp);
    va_end(argp);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART1)
	{
		HAL_NVIC_DisableIRQ(USART1_IRQn); //Rx Callback 함수 Disable
		HAL_UART_Transmit_IT(&huart1, &str1_, sizeof(str1_));
		HAL_NVIC_EnableIRQ(USART1_IRQn);  //Rx callback 함수 enable
		HAL_UART_Receive_IT(&huart1, &str1_, sizeof(str1_));

	}
	else if(huart->Instance == USART3)
	{
		//HAL_NVIC_DisableIRQ(USART3_IRQn); //Rx Callback 함수 Disable
		//HAL_NVIC_EnableIRQ(USART3_IRQn);  //Rx callback 함수 enable
		HAL_NVIC_DisableIRQ(USART3_IRQn); //Rx Callback 함수 Disable
		HAL_UART_Transmit_IT(&huart3, &str3_, sizeof(str3_));
		HAL_NVIC_EnableIRQ(USART3_IRQn);  //Rx callback 함수 enable
		HAL_UART_Receive_IT(&huart3, &str3_, sizeof(str3_));

		//HAL_UART_Receive_IT(&huart3, &str3_, 1);
	}
}





void Debug_Uart_Init()
{
	HAL_UART_Receive_IT(&huart1, &str1_, 1); 	//for debugging
	HAL_UART_Receive_IT(&huart3, &str3_, 1);	//for Controlling
}


//--------------------------------------------------------------test
/* USER CODE BEGIN PV */

extern CAN_HandleTypeDef hcan1;

/* USER CODE END PV */
extern char char_ethernet;
extern BG95 bg95test;

int a_= 2000;
int testpos= 100000;

//for uart1 debug
void DebugDrive()
{
	int cmd = 0;
	int cmd1 = str1_;
	int cmd2 = str3_;
	int cmd3 = char_ethernet;

	if (cmd1 != 0) cmd = cmd1;
	else if (cmd2 !=0) cmd = cmd2;
	else if (cmd3 !=0) cmd = cmd3;

	switch(cmd)
	{
		case '1': break;
		case '2': break;

		case '0':
			  Dprintf("Firmware will be rebooted in 3 senconds.\r\n");

			  HAL_Delay(3000);

			  HAL_NVIC_SystemReset();

			break;

		default: break;
	}
	memset(&str1_, '\0', sizeof(str1_));
	memset(&str3_, '\0', sizeof(str3_));
	memset(&char_ethernet, '\0', sizeof(char_ethernet));
}

