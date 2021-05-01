#define _MotorCtrl_CPP_
#include "MotorCtrl.h"
#include "Middlewares/Module/motor_ctrl.h"

#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>


extern CAN_HandleTypeDef hcan1;

Motor_GM6020 Motor_Yaw(1),Motor_Wrist(2);
MotorCascadeCtrl Motor_Yaw_Ctrl(Motor_Yaw),Motor_Wrist_Ctrl(Motor_Wrist);
AK80_V3 Motor_Shoulder(0x1A,&hcan2),Motor_Elbow(0x01,&hcan1);

TaskHandle_t CAN1_TaskHandle,DogMotoCtrl_TaskHandle;

void Service_MotoCtrl_Init()
{
	xTaskCreate((TaskFunction_t)Task_CAN1Receive,"CAN1_Receive",Tiny_Stack_Size,NULL,PriorityHigh,&CAN1_TaskHandle);
	xTaskCreate((TaskFunction_t)Task_DogMotorCtrl,"DogMotoCtrl",Tiny_Stack_Size,NULL,PriorityHigh,&DogMotoCtrl_TaskHandle);
}

static void Convert_Data(CAN_RxMessage* input, COB_TypeDef* output)
{
  output->ID = input->header.StdId;
  output->DLC = input->header.DLC;
  memcpy(output->Data, input->data, output->DLC);
}

void CAN1_RxCpltCallback(CAN_RxBuffer *CAN_RxMessage)
{
	static COB_TypeDef CAN_RxCOB;
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
	
}

void Task_DogMotorCtrl(void)
{
	Motor_2.To_Into_Control();
	Motor_3.To_Into_Control();
	TickType_t xLastWakeTime_MotoCtrl;
	for (;;)
	{
		xLastWakeTime_MotoCtrl = xTaskGetTickCount();
		Motor_2.Out_Speed_Control(5,0.1);
		Motor_3.Out_Speed_Control(5,0);
		vTaskDelayUntil(&xLastWakeTime_MotoCtrl,5);
	}
}


void Task_CAN1Receive(void)
{
	TickType_t xLastWakeTime_CAN1;
	static COB_TypeDef Rx_COB;
	for (;;)
	{
		xLastWakeTime_CAN1 = xTaskGetTickCount();
		while (xQueueReceive(RMMotor_QueueHandle,&Rx_COB,0) == pdPASS)
		{
			switch (Rx_COB.ID)
			{
				case 0x01:
					Motor_2.Update(Rx_COB.Data);
					break;
				case 0x03:
					Motor_3.Update(Rx_COB.Data);
					break;
				default:
					break;
			}
		}
		vTaskDelayUntil(&xLastWakeTime_CAN1,1);
	}
}