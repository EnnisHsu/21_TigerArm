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
void Task_BoardComRx(void *arg);
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
  xTaskCreate(Task_UsartTransmit,"Com.Usart TxPort" , Tiny_Stack_Size,    NULL, PriorityHigh,   &UartTransmitPort_Handle);
	xTaskCreate(Task_BoardComRx,"Com.Board Commu" , Normal_Stack_Size,    NULL, PriorityHigh,   &UartTransmitPort_Handle);
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
		while (xQueueReceive(RMMotor_QueueHandle, &RX_COB, 0)==pdPASS)
		{
			

		}

		vTaskDelayUntil(&xLastWakeTime, 1);
	}
}
void Task_BoardComRx(void *arg)
{
  /* Cache for Task */
	USART_COB _buffer;
	static TickType_t _xTicksToWait = pdMS_TO_TICKS(1);
  /* Pre-Load for task */

  /* Infinite loop */
  TickType_t xLastWakeTime_t;
  xLastWakeTime_t = xTaskGetTickCount();
	extern _BoardComRx BoardComRxData;
  for(;;)
  {
    

    if (xQueueReceive(BodCom_QueueHandle, &_buffer, _xTicksToWait) == pdTRUE)
    {

			static uint8_t* dr16_msg;
			memcpy(&BoardComRxData,_buffer.address,22);
    	DR16.DataCapture(&BoardComRxData.dr16Data);
			TigerArm.Switch_Mode((CEngineer::Engineer_Mode_Typedef)BoardComRxData.CtrlMode);
    }
    /* Pass control to the next task */
    vTaskDelayUntil(&xLastWakeTime_t,1);
  }
}
void Update_ChassisCurrent(uint8_t *msg_rece,uint8_t motor)
{
	/*The feedback and output values of the right two wheels should be reversed*/
		if(motor == RF || motor == RB)
		{
			
			Engineer_chassis.wheel_rpm[motor] = -(short)((unsigned char)msg_rece[2]<<8|(unsigned char)msg_rece[3]);	
			Engineer_chassis.wheel_torque[motor] = -(short)((unsigned char)msg_rece[4]<<8|(unsigned char)msg_rece[5]);
		}else if(motor == 4)
		{
			Camera_Motor.update(msg_rece);

		}
		else 
		{
			Engineer_chassis.wheel_rpm[motor] = (short)((unsigned char)msg_rece[2]<<8|(unsigned char)msg_rece[3]);	
			Engineer_chassis.wheel_torque[motor] = (short)((unsigned char)msg_rece[4]<<8|(unsigned char)msg_rece[5]);			
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
  if(((CAN_RxMessage->header.StdId)&0x200) == 0x200)
	{
		/* Calculate motor ID,and get the data of motor */
		uint8_t Motor_ID=CAN_RxMessage->header.StdId-0x201;	
		Update_ChassisCurrent(CAN_RxMessage->data, Motor_ID); 
	}

}

void User_CAN2_RxCpltCallback(CAN_RxBuffer *CAN_RxMessage)
{
  static CAN_COB   CAN_RxCOB;
  Convert_Data(CAN_RxMessage,&CAN_RxCOB);
  //Send To CAN Receive Queue

  if (CAN_RxCOB.ID==0x205 && RMMotor_QueueHandle != NULL)
	  xQueueSendFromISR(RMMotor_QueueHandle,&CAN_RxCOB,0);
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
	if (input->header.StdId==0x00) output->ID=input->data[0];
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
  
  return 0;
}

uint32_t User_UART3_RxCpltCallback(uint8_t* Recv_Data, uint16_t ReceiveLen)
{
  return 0;
}

uint32_t User_UART4_RxCpltCallback(uint8_t* Recv_Data, uint16_t ReceiveLen)
{
	return 0;
}

uint32_t User_UART5_RxCpltCallback(uint8_t* Recv_Data, uint16_t ReceiveLen)
{
  return 0;
}

uint32_t User_UART6_RxCpltCallback(uint8_t* Recv_Data, uint16_t ReceiveLen)
{
		static USART_COB Usart_RxCOB;
  //Send To UART Receive Queue
  if(ReceiveLen==22 && BodCom_QueueHandle != NULL)
  {
    Usart_RxCOB.port_num = 6;
    Usart_RxCOB.len      = ReceiveLen;
    Usart_RxCOB.address  = Recv_Data;
    xQueueSendFromISR(BodCom_QueueHandle,&Usart_RxCOB,0);
  }
  return 0;
}
/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
