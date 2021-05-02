#define _MotorCtrl_CPP_
#include "MotorCtrl.h"

#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>


Motor_GM6020 Motor_Yaw(1),Motor_Wrist(2);
MotorCascadeCtrl<myPID,myPID> Motor_Yaw_Ctrl(&Motor_Yaw),Motor_Wrist_Ctrl(&Motor_Wrist);
AK80_V3 Motor_Shoulder(0x0A,&hcan2),Motor_Elbow(0x01,&hcan2);

TaskHandle_t CAN1_TaskHandle,DogMotoCtrl_TaskHandle;

void Service_MotoCtrl_Init()
{
	xTaskCreate((TaskFunction_t)Task_CAN1Receive,"CAN1_Receive",Tiny_Stack_Size,NULL,PriorityHigh,&CAN1_TaskHandle);
	xTaskCreate((TaskFunction_t)Task_DogMotorCtrl,"DogMotoCtrl",Tiny_Stack_Size,NULL,PriorityHigh,&DogMotoCtrl_TaskHandle);
}

static void Convert_Data(CAN_RxMessage* input, CAN_COB* output)
{
  output->ID = input->header.StdId;
  output->DLC = input->header.DLC;
  memcpy(output->Data, input->data, output->DLC);
}

void CAN1_RxCpltCallback(CAN_RxBuffer *CAN_RxMessage)
{
	static CAN_COB CAN_RxCOB;
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	Convert_Data(CAN_RxMessage,&CAN_RxCOB);
	if ((CAN_RxMessage->data[1] &0x0F) !=0x00)
	{
		if (RMMotor_QueueHandle != NULL)
		{
			xQueueSendFromISR(RMMotor_QueueHandle,&CAN_RxCOB,&xHigherPriorityTaskWoken);
		}
	}
}

void Yaw_To(double deg)
{
	Motor_Yaw_Ctrl.setTarget(deg);
}

void Wrist_To(double deg)
{
	Motor_Wrist_Ctrl.setTarget(deg);
}

void TigerArm_Init(void)
{
	Motor_Shoulder.To_Into_Control();
	Motor_Elbow.To_Into_Control();
	MotorSpeedCtrl<myPID> Motor_Yaw_Init_Ctrl(&Motor_Yaw);
	//Motor_Yaw_Init_Ctrl.setTarget(3000.0f);
	//while (!PED_Yaw_Signal);
}

void Task_DogMotorCtrl(void)
{
	TigerArm_Init();
	TickType_t xLastWakeTime_MotoCtrl;
	for (;;)
	{
		xLastWakeTime_MotoCtrl = xTaskGetTickCount();
		
		vTaskDelayUntil(&xLastWakeTime_MotoCtrl,5);
	}
}


void Task_CAN1Receive(void)
{
	TickType_t xLastWakeTime_CAN1;
	static CAN_COB Rx_COB;
	for (;;)
	{
		xLastWakeTime_CAN1 = xTaskGetTickCount();
		while (xQueueReceive(RMMotor_QueueHandle,&Rx_COB,0) == pdPASS)
		{
//			if (Motor_Yaw.CheckID(Rx_COB.ID)) Motor_Yaw.update(Rx_COB.Data);
//			if (Motor_Shoulder.CheckID(Rx_COB.ID)) Motor_Shoulder.Update(Rx_COB.Data);
//			if (Motor_Elbow.CheckID(Rx_COB.ID)) Motor_Elbow.Update(Rx_COB.Data);
//			if (Motor_Wrist.CheckID(Rx_COB.ID)) Motor_Wrist.update(Rx_COB.Data);
//			switch (Rx_COB.ID)
//			{
//				case 0x01:
//					Motor_Shoulder.Update(Rx_COB.Data);
//					break;
//				case 0x03:
//					Motor_Elbow.Update(Rx_COB.Data);
//					break;
//				default:
//					break;
//			}
		}
		vTaskDelayUntil(&xLastWakeTime_CAN1,1);
	}
}

void Task_CAN2Receive(void)
{
	TickType_t xLastWakeTime_CAN2;
	static CAN_COB Rx_COB;
	for (;;)
	{
		xLastWakeTime_CAN2 = xTaskGetTickCount();
		while (xQueueReceive(RMMotor_QueueHandle,&Rx_COB,0) == pdPASS)
		{
			if (Motor_Yaw.CheckID(Rx_COB.ID)) Motor_Yaw.update(Rx_COB.Data);
			if (Motor_Shoulder.CheckID(Rx_COB.ID)) Motor_Shoulder.Update(Rx_COB.Data);
			if (Motor_Elbow.CheckID(Rx_COB.ID)) Motor_Elbow.Update(Rx_COB.Data);
			if (Motor_Wrist.CheckID(Rx_COB.ID)) Motor_Wrist.update(Rx_COB.Data);
		}
		vTaskDelayUntil(&xLastWakeTime_CAN2,1);
	}
}