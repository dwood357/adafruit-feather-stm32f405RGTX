/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "stdio.h"
#include "dac.h"
#include "bep_host_if.h"
#include "spi.h"

// #include "can.h"
//#include "gpio.h


/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CAN_TASK_SIZE 100
#define DAC_TASK_SIZE 100
#define SPI_TASK_SIZE 100

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
CAN_RxHeaderTypeDef   RxHeader;
CAN_TxHeaderTypeDef   TxHeader;

uint8_t               RxData[8];
uint8_t               TxData[8];

uint32_t              TxMailbox;

uint8_t               t=0;
uint32_t exec;

StaticTask_t  xCANTaskBuffer;
uint32_t  xCANStack[CAN_TASK_SIZE];

StaticTask_t  xADCTaskBuffer;
uint32_t  xADCStack[DAC_TASK_SIZE];

StaticTask_t  xSPITaskBuffer;
uint32_t  xSPIStack[SPI_TASK_SIZE];

/* USER CODE END Variables */
/* Definitions for defaultTask */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartDACTask(void *argument);
void StartSPITask(void *argument);


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

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
//  Timer1Start();
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
//  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);
  xTaskCreateStatic((TaskFunction_t)StartDefaultTask, "UARTTask", CAN_TASK_SIZE, NULL, osPriorityHigh, xCANStack, &xCANTaskBuffer);

  xTaskCreateStatic((TaskFunction_t)StartDACTask, "DACTask", DAC_TASK_SIZE, NULL, osPriorityHigh, xADCStack, &xADCTaskBuffer);

  xTaskCreateStatic((TaskFunction_t)StartSPITask, "SpiTask", DAC_TASK_SIZE, NULL, osPriorityHigh, xSPIStack, &xSPITaskBuffer);
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
	//setup the filters
  //   CAN_FilterTypeDef  CAN_FilterInitStructure;
  //   CAN_FilterInitStructure.FilterBank = 0;
  //   CAN_FilterInitStructure.SlaveStartFilterBank = 14;
  //   CAN_FilterInitStructure.FilterMode = CAN_FILTERMODE_IDMASK;
  //   CAN_FilterInitStructure.FilterScale = CAN_FILTERSCALE_32BIT;
  //   CAN_FilterInitStructure.FilterIdHigh = 0;
  //   CAN_FilterInitStructure.FilterIdLow = 0;
  //   CAN_FilterInitStructure.FilterMaskIdHigh = 0;
  //   CAN_FilterInitStructure.FilterMaskIdLow = 0;
  //   CAN_FilterInitStructure.FilterFIFOAssignment = 0;
  //   CAN_FilterInitStructure.FilterActivation = ENABLE;

  //   if(HAL_CAN_ConfigFilter(&hcan1, &CAN_FilterInitStructure) != HAL_OK)
	// {
  //   	//error
  //   	printf("CAN1 ERROR\n");
	// }

  //   CAN_FilterInitStructure.FilterIdHigh = 0;
  //   CAN_FilterInitStructure.FilterMaskIdHigh = 0;
  //   CAN_FilterInitStructure.FilterBank = 14;
  //   CAN_FilterInitStructure.FilterFIFOAssignment = 1;
  //   if(HAL_CAN_ConfigFilter(&hcan2, &CAN_FilterInitStructure) != HAL_OK)
	// {
	// 	//error
  //   	printf("CAN2 ERROR\n");
	// }

  //  	HAL_GPIO_WritePin(can1En_GPIO_Port, can1En_Pin, 1);
  //   HAL_GPIO_WritePin(can2En_GPIO_Port, can2En_Pin, 1);
	// HAL_CAN_Start(&hcan1);
	// HAL_CAN_Start(&hcan2);

	// HAL_CAN_ActivateNotification(&hcan1, (CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_ERROR));
	// HAL_CAN_ActivateNotification(&hcan2, (CAN_IT_RX_FIFO1_MSG_PENDING | CAN_IT_ERROR));

	// TxHeader.IDE = CAN_ID_EXT;
	// TxHeader.ExtId = 0x18FFB2F6;
	// TxHeader.RTR = CAN_RTR_DATA;
	// TxHeader.DLC = 8;

	//TxData[0] = 0xFF;
	//TxData[1] = 0xFF;
	//TxData[2] = 0xFF;
	//TxData[3] = 0xFF;
	//TxData[4] = 0xFF;
	//TxData[5] = 0xFF;
	//TxData[6] = 0xFF;
	//TxData[7] = 0xFF;

  /* Infinite loop */
  for(;;)
  {
//    HAL_CAN_RxFifo0MsgPendingCallback(&hcan1);
	  printf("Main task: %u \n", t);
	// TxData[7] = t;
	// HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);
//	printf("%c",RxHeader);
    osDelay(500);
    t+=1;
  }
  /* USER CODE END StartDefaultTask */
}

void StartDACTask(void *argument){
  
	for(;;){
    HAL_DAC_SetValue(&hdac, DAC1_CHANNEL_1, DAC_ALIGN_12B_R, 4000);
    osDelay(1000);
    HAL_DAC_SetValue(&hdac, DAC1_CHANNEL_1, DAC_ALIGN_12B_R, 100);
	}
}

void StartSPITask(void *argument){
  int state1 = HAL_SPI_GetState(&hspi1);
  int state2 = HAL_SPI_GetState(&hspi2);
  printf("%d , %d",state1, state2);

}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
// void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
// {
// 	if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
// 	  {
// 	    Error_Handler();
// 	  }

//     if ((RxHeader.ExtId == 0x18FF9981))
// 	  {
// //		  datacheck = 1;
// //		  printf("DLC:%c\n", RxHeader.DLC);
// 	  }
// }

// void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
// {

// }


/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
