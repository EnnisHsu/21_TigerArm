/**
  ***********************************************************************************
  * @file   : Service_Communication.cpp
  * @brief  : Communication support file.This file provides access ports to interface
  *           with connected devices.
  ***********************************************************************************
                                 ##### Port List #####
  =================================================================================
  |Port Name     Physical-Layer     Data-Link-Layer    Application-Layer    Number
  |————————————————————————————————————————
  |EXAMPLE_Port       CAN1               CAN                CUSTOM            0
  |CAN2_Port          CAN2               CAN                Custom            1
  |EBUG_Port         USART1             Custom              Custom            2
  |USART2_Port       USART2              DBUS               DJI-DR16          3
  *
**/
/* Includes ------------------------------------------------------------------*/
#include "Service_Communication.h"
#include "Service_MotoCtrl.h"
#include "Service_RobotCtrl.h"
#include <vector>
/* Private define ------------------------------------------------------------*/
void Task_CAN1Transmit(void *arg);
void Task_CAN2Transmit(void *arg);
void Task_UsartTransmit(void *arg);
void Task_CANReceive(void *arg);

/**
* @brief  Initialization of communication service
* @param  None.
* @return None.
*/
void Service_Communication_Init(void)
{
  
  /* CAN Filter Config*/
//  CAN_Filter_Mask_Config(&hcan1, CanFilter_1|CanFifo_0|Can_STDID|Can_DataType,0x001,0x3ff);//筛选器:|编号|FIFOx|ID类型|帧类型|ID|屏蔽位(0x3ff,0x1FFFFFFF)|
//  CAN_Filter_Mask_Config(&hcan2, CanFilter_15|CanFifo_0|Can_STDID|Can_DataType,0x002,0x3ff);//筛选器:|编号|FIFOx|ID类型|帧类型|ID|屏蔽位(0x3ff,0x1FFFFFFF)|
//  CAN_Filter_Mask_Config(&hcan2, CanFilter_14|CanFifo_0|Can_STDID|Can_DataType,0x003,0x3ff);//筛选器:|编号|FIFOx|ID类型|帧类型|ID|屏蔽位(0x3ff,0x1FFFFFFF)|
  CAN_Filter_Mask_Config(&hcan1, CanFilter_0|CanFifo_0|Can_STDID|Can_DataType, 0x200, 0x3f0);
  CAN_Filter_Mask_Config(&hcan1, CanFilter_1|CanFifo_0|Can_STDID|Can_DataType, 0x02, 0xff);
  CAN_Filter_Mask_Config(&hcan1, CanFilter_2|CanFifo_0|Can_STDID|Can_DataType, 0x00, 0xff);
  CAN_Filter_Mask_Config(&hcan1, CanFilter_3|CanFifo_0|Can_STDID|Can_DataType, 0x66, 0xff);
  CAN_Filter_Mask_Config(&hcan2, CanFilter_14|CanFifo_0|Can_STDID|Can_DataType, 0x01, 0xff);
  CAN_Filter_Mask_Config(&hcan2, CanFilter_17|CanFifo_0|Can_STDID|Can_DataType, 0x66, 0xff);
  CAN_Filter_Mask_Config(&hcan2, CanFilter_15|CanFifo_0|Can_STDID|Can_DataType, 0x200, 0x3f0);
  CAN_Filter_Mask_Config(&hcan2, CanFilter_16|CanFifo_0|Can_STDID|Can_DataType, 0x00, 0xff);
  xTaskCreate(Task_UsartTransmit,"Com.Usart TxPort" , Tiny_Stack_Size,    NULL, PriorityHigh,   &UartTransmitPort_Handle);
  xTaskCreate(Task_CAN1Transmit, "Com.CAN1 TxPort"  , Tiny_Stack_Size,    NULL, PrioritySuperHigh,   &CAN1SendPort_Handle);
  xTaskCreate(Task_CAN2Transmit, "Com.CAN2 TxPort"  , Tiny_Stack_Size,    NULL, PrioritySuperHigh,   &CAN2SendPort_Handle); 
  xTaskCreate(Task_CANReceive, "Com.CAN RxPort", Tiny_Stack_Size, NULL, PrioritySuperHigh, &CANReceivePort_Handle);
}

/*----------------------------------------------- CAN Manager ---------------------------------------------*/
/*Task Define ---------------------------*/
TaskHandle_t CAN1SendPort_Handle;
TaskHandle_t CAN2SendPort_Handle;
TaskHandle_t CANReceivePort_Handle;
static void Convert_Data(CAN_RxMessage* input, CAN_COB* output);

/*Function Prototypes--------------------*/
/**
* @brief  Tasks for CAN Management.
* @param  None.
* @return None.
*/
void Task_CAN1Transmit(void *arg)
{
  /* Cache for Task */
  uint8_t free_can_mailbox;
  CAN_COB CAN_TxMsg;
  /* Pre-Load for task */
  
  /* Infinite loop */
  
  for(;;)
  {
    /* CAN1 Send Port */
    if(xQueueReceive(CAN1_TxPort,&CAN_TxMsg,portMAX_DELAY) == pdPASS)
    {
      do{
        free_can_mailbox = HAL_CAN_GetTxMailboxesFreeLevel(&hcan1);
      }while(free_can_mailbox == 0);
      CANx_SendData(&hcan1,CAN_TxMsg.ID,CAN_TxMsg.Data,CAN_TxMsg.DLC);
    }
	vTaskDelay(1);
  }
}

void Task_CAN2Transmit(void *arg)
{
  /* Cache for Task */
  uint8_t free_can_mailbox;
  CAN_COB CAN_TxMsg;
  /* Pre-Load for task */
  
  /* Infinite loop */
  
  for(;;)
  {
    /* CAN2 Send Port */
    if(xQueueReceive(CAN2_TxPort,&CAN_TxMsg,portMAX_DELAY) == pdPASS)
    {
      do{
        free_can_mailbox = HAL_CAN_GetTxMailboxesFreeLevel(&hcan2);
      }while(free_can_mailbox == 0);
      CANx_SendData(&hcan2,CAN_TxMsg.ID,CAN_TxMsg.Data,CAN_TxMsg.DLC);
    }
	vTaskDelay(1);
  }
}

void Task_CANReceive(void *arg)
{
  /* Cache for Task */
  TickType_t xLastWakeTime=xTaskGetTickCount();
  static CAN_COB RX_COB;
  /* Pre-Load for task */

  /* Infinite loop */

  for(;;)
  {
    while (xQueueReceive(RMMotor_QueueHandle, &RX_COB, 0) == pdPASS)
    {
      if (yaw_controller.joint_motor.CheckID(RX_COB.ID)) yaw_controller.joint_motor.update(RX_COB.Data);
    }
    while (xQueueReceive(AK80Motor_QueueHandle, &RX_COB, 0) == pdPASS)
    {
			if (arm_controller.joint_motor.CheckID(RX_COB.ID)) arm_controller.joint_motor.Update(RX_COB.Data);
			if (elbow_controller.joint_motor.CheckID(RX_COB.ID)) elbow_controller.joint_motor.Update(RX_COB.Data);
      //if (Tigerarm_Shoulder.CheckID(RX_COB.ID)) Tigerarm_Shoulder.Update(RX_COB.Data);
      //if (Tigerarm_Elbow.CheckID(RX_COB.ID)) Tigerarm_Elbow.Update(RX_COB.Data);
    }
    vTaskDelayUntil(&xLastWakeTime, 1);
  }
}

/**
* @brief  Callback function in CAN Interrupt
* @param  None.
* @return None.
*/
void User_CAN1_RxCpltCallback(CAN_RxBuffer *CAN_RxMessage)
{
  static CAN_COB   CAN_RxCOB;
  Convert_Data(CAN_RxMessage,&CAN_RxCOB);
  //Send To CAN Receive Queue
  if(CAN_RxCOB.ID != 0x02 && CAN_RxCOB.ID != 0x01 && RMMotor_QueueHandle != NULL)
    xQueueSendFromISR(RMMotor_QueueHandle, &CAN_RxCOB, 0);
  if (CAN_RxCOB.ID == 0x02 && AK80Motor_QueueHandle != NULL)
	  xQueueSendFromISR(AK80Motor_QueueHandle, &CAN_RxCOB, 0);
  if (CAN_RxCOB.ID == 0x01 && AK80Motor_QueueHandle != NULL)
	  xQueueSendFromISR(AK80Motor_QueueHandle, &CAN_RxCOB, 0);
}

void User_CAN2_RxCpltCallback(CAN_RxBuffer *CAN_RxMessage)
{
  static CAN_COB   CAN_RxCOB;
  Convert_Data(CAN_RxMessage,&CAN_RxCOB);
  //Send To CAN Receive Queue
  if(CAN_RxCOB.ID != 0x02 && CAN_RxCOB.ID != 0x01 && RMMotor_QueueHandle != NULL)
    xQueueSendFromISR(RMMotor_QueueHandle, &CAN_RxCOB, 0);
  if (CAN_RxCOB.ID == 0x02 && AK80Motor_QueueHandle != NULL)
	  xQueueSendFromISR(AK80Motor_QueueHandle, &CAN_RxCOB, 0);
  if (CAN_RxCOB.ID == 0x01 && AK80Motor_QueueHandle != NULL)
	  xQueueSendFromISR(AK80Motor_QueueHandle, &CAN_RxCOB, 0);
}

/**
* @brief  Data convertion，provide lower layer access port 
          for application layer.
* @param  CAN_RxMsg：Lower layer CAN frame.
          CAN_RxCOB：Application layer CAN Frame.
* @return None.
*/
static void Convert_Data(CAN_RxMessage* input, CAN_COB* output)
{
	if (input->header.StdId==0x00 || input->header.StdId==0x66) output->ID=input->data[0];
	else output->ID = input->header.StdId;
  output->DLC = input->header.DLC;
  memcpy(output->Data, input->data, output->DLC);
}


/*---------------------------------------------- USART Manager --------------------------------------------*/
/*Task Define ---------------------------*/
TaskHandle_t UartTransmitPort_Handle;

/*Function Prototypes--------------------*/
/**
* @brief  Tasks for USART Management.
          Attention:In this version we passing the pointer of data not copying
          data itself and we only have one buff zone, so user need to process 
          the data as soon as possible in case of over-write.
* @param  None.
* @return None.
*/
void Task_UsartTransmit(void *arg)
{
  /* Cache for Task */
  UART_HandleTypeDef* pUart_x;
  static std::vector<uint8_t> Packed_COB;
  static USART_COB  Usart_TxCOB;
  /* Pre-Load for task */
  /* Infinite loop */
  for(;;)
  {
    /* Usart Receive Port*/
    if(xQueueReceive(USART_TxPort,&Usart_TxCOB,portMAX_DELAY) == pdPASS)
    {
      /* User Code Begin Here -------------------------------*/
      switch(Usart_TxCOB.port_num)
      {

        case 4:
          pUart_x = &huart4;
          break;
				case 6:
					pUart_x = &huart6;
					break;
      }
      /* User Code End Here ---------------------------------*/
      HAL_UART_Transmit_DMA(pUart_x,(uint8_t*)Usart_TxCOB.address,Usart_TxCOB.len);
    }
  }
}

/**
* @brief  Callback function in USART Interrupt
* @param  None.
* @return None.
*/
/*
  DR16
*/
uint32_t User_UART2_RxCpltCallback(uint8_t* Recv_Data,uint16_t ReceiveLen)
{
  static USART_COB Usart_RxCOB;
  //Send To UART Receive Queue
  if(DR16_QueueHandle != NULL)
  {
    Usart_RxCOB.port_num = 2;
    Usart_RxCOB.len      = ReceiveLen;
    Usart_RxCOB.address  = Recv_Data;
    xQueueSendFromISR(DR16_QueueHandle,&Usart_RxCOB,0);
		Recv_Data[19]=TigerArm.Get_Current_Mode();
		Usart_RxCOB.port_num = 6;
		xQueueSendFromISR(USART_TxPort,&Usart_RxCOB,0);
  }
  return 0;
}

uint32_t User_UART3_RxCpltCallback(uint8_t* Recv_Data, uint16_t ReceiveLen)
{
  return 0;
}

uint32_t User_UART4_RxCpltCallback(uint8_t* Recv_Data, uint16_t ReceiveLen)
{
	static USART_COB Usart_RxCOB;
	if (NUC_QueueHandle!=NULL)
	{
		Usart_RxCOB.port_num=4;
		Usart_RxCOB.len=ReceiveLen;
		Usart_RxCOB.address=Recv_Data;
		xQueueSendFromISR(NUC_QueueHandle,&Usart_RxCOB,0);
	}
	return 0;
}

uint32_t User_UART5_RxCpltCallback(uint8_t* Recv_Data, uint16_t ReceiveLen)
{

  return 0;
}
/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
