/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include <ethernet/tcp_rtos/tcp_rtos.h>
#include <ethernet/udp_rtos/udp_rtos.h>
#include <api_debug/api_debug.h>
#include "main.h"
#include "cmsis_os.h"
#include "lwip.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "init_stm32f7/api_init.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

osThreadId InitTaskHandle;
osThreadId Task1Handle;
osThreadId Task2Handle;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void InitTask(void const * argument);
void Task1(void const * argument);
void Task2(void const * argument);
void Task3(void const * argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE BEGIN PM */
#define MAJOR 0   //APP Major version Number
#define MINOR 3   //APP Minor version Number
/* USER CODE END PM */
/* USER CODE BEGIN PV */
const uint8_t APP_Version[2] = { MAJOR, MINOR };

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* Enable I-Cache---------------------------------------------------------*/
	SCB_EnableICache();

	/* Enable D-Cache---------------------------------------------------------*/
	SCB_EnableDCache();

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */
	init_stm32f746();

	Debug_Uart_Init();
	/* USER CODE END SysInit */

	Dprintf("Starting Application(%d.%d)\n", APP_Version[0], APP_Version[1] );
	/* USER CODE BEGIN 2 */



  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of InitTask */
  osThreadDef(InitTask, InitTask, osPriorityNormal, 0, 512);
  InitTaskHandle = osThreadCreate(osThread(InitTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/* USER CODE BEGIN InitTask */
/**
  * @brief  Function implementing the InitTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END InitTask */
void InitTask(void const *argument)
{
	/* init code for LWIP */
	MX_LWIP_Init();

	//UdpServerInit();
	//UdpClientInit();
	//TcpClientInit();
	TcpServerInit();

	/* definition and creation of Task1 */
	osThreadDef(Task1, Task1, osPriorityLow, 0, 512);
	Task1Handle = osThreadCreate(osThread(Task1), NULL);

	/* definition and creation of Task2 */
	osThreadDef(Task2, Task2, osPriorityHigh, 0, 512);
	Task2Handle = osThreadCreate(osThread(Task2), NULL);

	/* Infinite loop */
	for(;;)
	{
		osDelay(1);

		vTaskDelete(NULL);
	}
	/* USER CODE END 5 */
}

/* USER CODE BEGIN Task1 */
/**
* @brief Function implementing the Task1 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Task1 */
void Task1(void const *argument)
{
  /* USER CODE BEGIN StartCommonSensorTask */

	const TickType_t xTime = pdMS_TO_TICKS(5);

	TickType_t xLastWakeTime = xTaskGetTickCount();

  /* Infinite loop */

  for(;;)
  {
    DebugDrive();

    osDelay(1);

	vTaskDelayUntil(&xLastWakeTime, xTime);

  }
  /* USER CODE END Task1 */
}

/* USER CODE BEGIN Task2 */
/**
* @brief Function implementing the Task2 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Task2 */

void Task2(void const *argument)
{
  /* USER CODE BEGIN Task2 */

	const TickType_t xTime = pdMS_TO_TICKS(2);

	TickType_t xLastWakeTime = xTaskGetTickCount();

  /* Infinite loop */
  for(;;)
  {

	osDelay(1);

	vTaskDelayUntil(&xLastWakeTime, xTime);

	//vTaskDelay(pdMS_TO_TICKS(80));
	}
  /* USER CODE END Task2 */
}


