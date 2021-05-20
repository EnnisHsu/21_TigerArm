#include "Service_MotoCtrl.h"

Motor_GM6020 Tigerarm_Yaw(1);
MotorCascadeCtrl<myPID,myPID> Tigerarm_Yaw_Ctrl(&Tigerarm_Yaw);
AK80_V3 Tigerarm_Shoulder(0x0A,&hcan1),Tigerarm_Elbow(0x01,&hcan2);

float Shoulder_kp=300.0f,Shoulder_kd=5.0f,Elbow_kp=300.0f,Elbow_kd=3.0f;
float spd=10.0f;
float Shoulder_target_pos,Elbow_target_pos;//	rad
float Yaw_target_pos;//		deg

TaskHandle_t ServiceMotoCtrl_Handle;

void Service_MotoCtrl_Init()
{
	xTaskCreate(Task_ArmMotoCtrl, "ArmMotorCtrl", Normal_Stack_Size, NULL, PriorityHigh, &ServiceMotoCtrl_Handle);
}

void ArmMotorSetZeroConfig()
{
	Tigerarm_Shoulder.Set_ZeroPosition();
	Tigerarm_Elbow.Set_ZeroPosition();
}

void ArmMotorInit()
{
	Tigerarm_Shoulder.To_Into_Control();
	Tigerarm_Elbow.To_Into_Control();
	Shoulder_target_pos=Tigerarm_Shoulder.get_current_position();
	Elbow_target_pos=Tigerarm_Elbow.get_current_position();
	Yaw_target_pos=Tigerarm_Yaw.getAngle();

}

void Task_ArmMotoCtrl(void *arg)
{
	  /* Cache for Task */
		Motor_CAN_COB Motor_TxMsg;
	  /* Pre-Load for task */
		ArmMotorInit();
	  /* Infinite loop */
	  TickType_t xLastWakeTime_t;
	  xLastWakeTime_t = xTaskGetTickCount();
	  for(;;)
	  {
		Tigerarm_Shoulder.Out_Mixed_Control(Shoulder_target_pos, spd, Shoulder_kp, Shoulder_kd);
		Tigerarm_Elbow.Out_Mixed_Control(Elbow_target_pos, spd, Elbow_kp, Elbow_kd);
		Tigerarm_Yaw_Ctrl.setTarget(Yaw_target_pos);
		MotorMsgPack(Motor_TxMsg, Tigerarm_Yaw);
		xQueueSendFromISR(CAN2_TxPort,&Motor_TxMsg.High,0);
	    /* Pass control to the next task */
	    vTaskDelayUntil(&xLastWakeTime_t,1);
	  }
}
