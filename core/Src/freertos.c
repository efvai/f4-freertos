/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
typedef StaticQueue_t osStaticMessageQDef_t;
typedef StaticSemaphore_t osStaticSemaphoreDef_t;
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
typedef struct {
	char buffer[128];
} Message;
extern UART_HandleTypeDef huart1;
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
uint32_t defaultTaskBuffer[ 128 ];
osStaticThreadDef_t defaultTaskControlBlock;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .cb_mem = &defaultTaskControlBlock,
  .cb_size = sizeof(defaultTaskControlBlock),
  .stack_mem = &defaultTaskBuffer[0],
  .stack_size = sizeof(defaultTaskBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for LEDTask */
osThreadId_t LEDTaskHandle;
uint32_t LEDTaskBuffer[ 128 ];
osStaticThreadDef_t LEDTaskControlBlock;
const osThreadAttr_t LEDTask_attributes = {
  .name = "LEDTask",
  .cb_mem = &LEDTaskControlBlock,
  .cb_size = sizeof(LEDTaskControlBlock),
  .stack_mem = &LEDTaskBuffer[0],
  .stack_size = sizeof(LEDTaskBuffer),
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for BtnTask */
osThreadId_t BtnTaskHandle;
uint32_t BtnTaskBuffer[ 128 ];
osStaticThreadDef_t BtnTaskControlBlock;
const osThreadAttr_t BtnTask_attributes = {
  .name = "BtnTask",
  .cb_mem = &BtnTaskControlBlock,
  .cb_size = sizeof(BtnTaskControlBlock),
  .stack_mem = &BtnTaskBuffer[0],
  .stack_size = sizeof(BtnTaskBuffer),
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for LEDPWMTask */
osThreadId_t LEDPWMTaskHandle;
uint32_t LEDPWMTaskBuffer[ 128 ];
osStaticThreadDef_t LEDPWMTaskControlBlock;
const osThreadAttr_t LEDPWMTask_attributes = {
  .name = "LEDPWMTask",
  .cb_mem = &LEDPWMTaskControlBlock,
  .cb_size = sizeof(LEDPWMTaskControlBlock),
  .stack_mem = &LEDPWMTaskBuffer[0],
  .stack_size = sizeof(LEDPWMTaskBuffer),
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for uartTask */
osThreadId_t uartTaskHandle;
uint32_t uartTaskBuffer[ 128 ];
osStaticThreadDef_t uartTaskControlBlock;
const osThreadAttr_t uartTask_attributes = {
  .name = "uartTask",
  .cb_mem = &uartTaskControlBlock,
  .cb_size = sizeof(uartTaskControlBlock),
  .stack_mem = &uartTaskBuffer[0],
  .stack_size = sizeof(uartTaskBuffer),
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for uartQueue */
osMessageQueueId_t uartQueueHandle;
uint8_t uartQueueBuffer[ 8 * sizeof( Message ) ];
osStaticMessageQDef_t uartQueueControlBlock;
const osMessageQueueAttr_t uartQueue_attributes = {
  .name = "uartQueue",
  .cb_mem = &uartQueueControlBlock,
  .cb_size = sizeof(uartQueueControlBlock),
  .mq_mem = &uartQueueBuffer,
  .mq_size = sizeof(uartQueueBuffer)
};
/* Definitions for BtnSem */
osSemaphoreId_t BtnSemHandle;
osStaticSemaphoreDef_t BtnSemControlBlock;
const osSemaphoreAttr_t BtnSem_attributes = {
  .name = "BtnSem",
  .cb_mem = &BtnSemControlBlock,
  .cb_size = sizeof(BtnSemControlBlock),
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void LEDTaskEntry(void *argument);
void BtnTaskEntry(void *argument);
void LEDPWMTaskEntry(void *argument);
void uartTaskEntry(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of BtnSem */
  BtnSemHandle = osSemaphoreNew(1, 1, &BtnSem_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of uartQueue */
  uartQueueHandle = osMessageQueueNew (8, sizeof(Message), &uartQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of LEDTask */
  LEDTaskHandle = osThreadNew(LEDTaskEntry, NULL, &LEDTask_attributes);

  /* creation of BtnTask */
  BtnTaskHandle = osThreadNew(BtnTaskEntry, NULL, &BtnTask_attributes);

  /* creation of LEDPWMTask */
  LEDPWMTaskHandle = osThreadNew(LEDPWMTaskEntry, NULL, &LEDPWMTask_attributes);

  /* creation of uartTask */
  uartTaskHandle = osThreadNew(uartTaskEntry, NULL, &uartTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_LEDTaskEntry */
/**
* @brief Function implementing the LEDTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_LEDTaskEntry */
void LEDTaskEntry(void *argument)
{
  /* USER CODE BEGIN LEDTaskEntry */
  /* Infinite loop */
  for(;;)
  {
	HAL_GPIO_TogglePin(GPIOA, LED_Pin);
    osDelay(1000);
  }
  /* USER CODE END LEDTaskEntry */
}

/* USER CODE BEGIN Header_BtnTaskEntry */
/**
* @brief Function implementing the BtnTask thread.
* @param argument: Not used
* @retval None
*/

/* USER CODE END Header_BtnTaskEntry */
void BtnTaskEntry(void *argument)
{
  /* USER CODE BEGIN BtnTaskEntry */
	Message msg;
  /* Infinite loop */
  for(;;)
  {


	if (!HAL_GPIO_ReadPin(BTN1_GPIO_Port, BTN1_Pin)) {
		osSemaphoreRelease(BtnSemHandle);
		strcpy(msg.buffer, "Btn pressed\r\n\0");
		osMessageQueuePut(uartQueueHandle, &msg, 0, osWaitForever);
	}
    osDelay(250);
  }
  /* USER CODE END BtnTaskEntry */
}

/* USER CODE BEGIN Header_LEDPWMTaskEntry */
/**
* @brief Function implementing the LEDPWMTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_LEDPWMTaskEntry */
void LEDPWMTaskEntry(void *argument)
{
  /* USER CODE BEGIN LEDPWMTaskEntry */
  const int ccr[3] = {98, 75, 10};
  int ledState = 0;
  const int length = sizeof(ccr) / sizeof(int);
  /* Infinite loop */
  for(;;)
  {
	  if (osSemaphoreAcquire(BtnSemHandle, osWaitForever)
			  == osOK) {
		  TIM3->CCR2 = ccr[ledState++];
		  if (ledState >= length) {
			  ledState = 0;
		  }
	  }
    osDelay(100);
  }
  /* USER CODE END LEDPWMTaskEntry */
}

/* USER CODE BEGIN Header_uartTaskEntry */
/**
* @brief Function implementing the uartTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_uartTaskEntry */
void uartTaskEntry(void *argument)
{
  /* USER CODE BEGIN uartTaskEntry */
	Message msg;
  /* Infinite loop */
  for(;;)
  {
	  osMessageQueueGet(uartQueueHandle, &msg, 0, osWaitForever);
	  HAL_UART_Transmit(&huart1, (uint8_t*)msg.buffer, strlen(msg.buffer), osWaitForever);
	  osDelay(1);
  }
  /* USER CODE END uartTaskEntry */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

