#define _MotorCtrl_CPP_
#include "MotorCtrl.h"
#include "System_DataPool.h"

#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>


extern CAN_HandleTypeDef hcan1;

AK80_V3 Motor_2(0x0A,&hcan2),Motor_3(0x01,&hcan1);

TaskHandle_t CAN1_TaskHandle,DogMotoCtrl_TaskHandle;

void Service_MotoCtrl_Init()
{
	xTaskCreate((TaskFunction_t)Task_CAN1Receive,"CAN1_Receive",Tiny_Stack_Size,NULL,PriorityHigh,&CAN1_TaskHandle);
	xTaskCreate((TaskFunction_t)Task_DogMotorCtrl,"DogMotoCtrl",Tiny_Stack_Size,NULL,PriorityHigh,&DogMotoCtrl_TaskHandle);
}

static void Convert_Data(CAN_RxMessage* input, COB_TypeDef* output)
{
  output->ID = input->data[0];
  output->DLC = input->header.DLC;
  memcpy(output->Data, input->data, output->DLC);
}

void CAN1_RxCpltCallback(CAN_RxBuffer *CAN_RxMessage)
{
	static COB_TypeDef CAN_RxCOB;
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	Convert_Data(CAN_RxMessage,&CAN_RxCOB);
	if ((CAN_RxMessage->data[0] &0x0F) !=0x00)
	{
		if (RMMotor_QueueHandle != NULL)
		{
			xQueueSendFromISR(RMMotor_QueueHandle,&CAN_RxCOB,&xHigherPriorityTaskWoken);
		}
	}
}

void CAN2_RxCpltCallback(CAN_RxBuffer *CAN_RxMessage)
{
	static COB_TypeDef CAN_RxCOB;
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	Convert_Data(CAN_RxMessage,&CAN_RxCOB);
		if (RMMotor_QueueHandle2 != NULL)
		{
			xQueueSendFromISR(RMMotor_QueueHandle2,&CAN_RxCOB,&xHigherPriorityTaskWoken);
		}
}

double spd=0.0f,pos=-0.5f,spd_kd=5.0f,pos_kp=500.0f;//spd_kd[0.0f~10.0f] pos_kp[0.0f~500.0f]
double angle2=0.00f,angle3=0.00f;
double mt3_gap=0.02f;

void Task_DogMotorCtrl(void)
{
	Motor_2.To_Into_Control();
	Motor_2.Set_ZeroPosition();
	//HAL_Delay(1000);
	Motor_3.To_Into_Control();
	Motor_3.Set_ZeroPosition();
	TickType_t xLastWakeTime_MotoCtrl;
	Motor_2.Out_Mixed_Control(0.0f,spd,pos_kp,spd_kd);
	Motor_3.Out_Mixed_Control(0.0f,spd,pos_kp,spd_kd);
	static float target[2] = {Motor_2.get_Current_Pos(),Motor_3.get_Current_Pos()};
	for (;;)
	{
		
		xLastWakeTime_MotoCtrl = xTaskGetTickCount();
		//Motor_2.Out_Speed_Control(spd,spd_kd);
		//Motor_2.Out_Torque_Control(0.3f);
		//Motor_3.Out_Speed_Control(5,0);
		//Motor_2.Out_Mixed_Control(pos,spd,pos_kp,spd_kd);
		
		static float torg = 8;
		if (DR16.GetStatus()==Connection_Established)
		{
			if (DR16.Get_LY_Norm()<-0.5f) 
			{
				if (abs(Motor_2.get_Current_Pos()-target[0])<0.02 || (target[0]-Motor_2.get_Current_Pos()>0.0f)) target[0]-=0.02f;
			}
		else if (DR16.Get_LY_Norm()>0.5f) 
		{
			if (abs(Motor_2.get_Current_Pos()-target[0])<0.02 || (target[0]-Motor_2.get_Current_Pos()<0.0f)) target[0]+=0.02f;
		}
//		if (DR16.Get_LY_Norm()<-0.5f) Motor_3.Out_Mixed_Control(Motor_3.get_Current_Pos()-0.02f,spd,pos_kp,spd_kd);
//		else if (DR16.Get_LY_Norm()>0.5f) Motor_3.Out_Mixed_Control(Motor_3.get_Current_Pos()+0.02f,spd,pos_kp,spd_kd);
//			
//			if (DR16.Get_LX_Norm()<-0.5f) Motor_2.Out_Torque_Control(-torg);
//			else if (DR16.Get_LX_Norm()>0.5f) Motor_2.Out_Torque_Control(torg);
			Motor_2.Out_Mixed_Control(target[0],spd,pos_kp,spd_kd);
			if (DR16.Get_RY_Norm()<-0.5f) 
			{
				if (abs(Motor_3.get_Current_Pos()-target[1])<0.02 || (target[1]-Motor_3.get_Current_Pos()<0.0f)) target[1]+=mt3_gap;
			}
		else if (DR16.Get_RY_Norm()>0.5f) 
		{
			if (abs(Motor_3.get_Current_Pos()-target[1])<0.02 || (target[1]-Motor_3.get_Current_Pos()>0.0f)) target[1]-=mt3_gap;
		}
		Motor_3.Out_Mixed_Control(target[1],spd,pos_kp,spd_kd);
//			if (DR16.Get_LY_Norm()<-0.5f) Motor_3.Out_Torque_Control(-1*torg);
//			else if (DR16.Get_LY_Norm()>0.5f) Motor_3.Out_Torque_Control(torg);
			/*else{
				target[1] = Motor_3.get_Command_Pos();
				Motor_3.Out_Mixed_Control(target[1],spd,pos_kp,spd_kd);	
			
			}*/
		}
		else
		{
			Motor_2.Out_Mixed_Control(target[0],spd,pos_kp,spd_kd);
			Motor_3.Out_Mixed_Control(target[1],spd,pos_kp,spd_kd);
		}
		
		//Motor_2.Out_Mixed_Control(target[0],spd,pos_kp,spd_kd);
		//vTaskDelayUntil(&xLastWakeTime_MotoCtrl,500);
		//Motor_2.Out_Mixed_Control(-1.0f,spd,pos_kp,spd_kd);
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
		while(xQueueReceive(RMMotor_QueueHandle2,&Rx_COB,0) == pdPASS){
			switch (Rx_COB.ID)
			{
				case 0x0A:
					Motor_2.Update(Rx_COB.Data);
					break;
				default:
					break;
			}
		}
		while (xQueueReceive(RMMotor_QueueHandle,&Rx_COB,0) == pdPASS)
		{
			switch (Rx_COB.ID)
			{
				case 0x0A:
					Motor_2.Update(Rx_COB.Data);
					break;
				case 0x01:
					Motor_3.Update(Rx_COB.Data);
					break;
				default:
					break;
			}
		}
		vTaskDelayUntil(&xLastWakeTime_CAN1,1);
	}
}