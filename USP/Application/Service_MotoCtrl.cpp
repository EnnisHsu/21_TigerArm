#include "Service_MotoCtrl.h"

Motor_GM6020 Tigerarm_Yaw(1);
MotorCascadeCtrl<myPID,myPID> Tigerarm_Yaw_Ctrl(&Tigerarm_Yaw);
AK80_V3 Tigerarm_Shoulder(0x02,&hcan1),Tigerarm_Elbow(0x01,&hcan1);

Asynchronous_Controller yaw_async_controller;
Asynchronous_Controller shoulder_async_controller;	//2nd joint
Asynchronous_Controller elbow_async_controller;		//3rd joint

float Shoulder_kp=50.0f,Shoulder_kd=2.0f,Elbow_kp=100.0f,Elbow_kd=1.0f;
float spd=10.0f;
float Shoulder_target_pos,Elbow_target_pos;//	rad
float Yaw_target_pos;//		deg
float Shoulder_current_target_pos,Elbow_current_target_pos;
float Wrist_Roll_pos,Wrist_Pitch_pos,Wrist_Yaw_pos;	//deg
float Shoulder_Zero_Offset,Yaw_Zero_Offset,Elbow_Zero_Offset;
int flag=0;

TaskHandle_t ServiceMotoCtrl_Handle;
TaskHandle_t LinearTargetCtrl_Handle;

void Service_MotoCtrl_Init()
{
	//ArmMotorInit();
	xTaskCreate(Task_ArmMotoCtrl, "ArmMotorCtrl", Normal_Stack_Size, NULL, PrioritySuperHigh, &ServiceMotoCtrl_Handle);
	xTaskCreate(Task_LinearTargetCtrl,"LinearTargetCtrl",Normal_Stack_Size,NULL,PriorityAboveNormal,&LinearTargetCtrl_Handle);
}

void ArmMotorSetZeroConfig()
{
	Tigerarm_Shoulder.Set_ZeroPosition();
	Tigerarm_Elbow.Set_ZeroPosition();
}

void ArmYawInit()
{
	Tigerarm_Yaw_Ctrl.AnglePID.SetPIDParam(100.0f,0.0f,0.0f,1000.0f,30000.0f);
	Tigerarm_Yaw_Ctrl.SpeedPID.SetPIDParam(80.0f,0.0f,0.0f,1000.0f,20000.0f);
	Yaw_target_pos=Tigerarm_Yaw.getAngle();
	Motor_CAN_COB Motor_TxMsgx;
	TickType_t xLastSetTime=xTaskGetTickCount();
	while (xTaskGetTickCount()-xLastSetTime<500)
	{
		if (abs(Yaw_target_pos-Tigerarm_Yaw.getAngle())<1.0f)
		{
			xLastSetTime=xTaskGetTickCount();
			Yaw_target_pos-=1.0f;	
		}
		Tigerarm_Yaw_Ctrl.setTarget(Yaw_target_pos);
		Tigerarm_Yaw_Ctrl.Adjust();
		MotorMsgPack(Motor_TxMsgx, Tigerarm_Yaw);
		xQueueSendFromISR(CAN1_TxPort,&Motor_TxMsgx.Low,0);
		vTaskDelay(1);
	}
	//ArmMotorSetZeroConfig();
	
	
}

void ArmShoulderInit()
{
	TickType_t xLastSetTime=xTaskGetTickCount();
	while (xTaskGetTickCount()-xLastSetTime<500)
	{
		if (fabs(Shoulder_target_pos-Tigerarm_Shoulder.get_current_position())<0.05f)
		{
			xLastSetTime=xTaskGetTickCount();
			Shoulder_target_pos+=0.05f;	
			Elbow_target_pos-=0.05f;
		}
		Tigerarm_Shoulder.Out_Mixed_Control(Shoulder_target_pos,spd,60.0f,2.0f);
		Tigerarm_Elbow.Out_Mixed_Control(Elbow_target_pos,spd,30.0f,1.0f);
		vTaskDelay(1);
	}
	//ArmMotorSetZeroConfig();
	//Shoulder_Zero_Offset=Tigerarm_Shoulder.get_current_position();
}

void ArmElbowInit()
{
	TickType_t xLastSetTime=xTaskGetTickCount();
	while (xTaskGetTickCount()-xLastSetTime<500)
	{
		if (fabs(Elbow_target_pos-Tigerarm_Elbow.get_current_position())<0.1f)
		{
			xLastSetTime=xTaskGetTickCount();
			Elbow_target_pos+=0.1f;	
		}
		Tigerarm_Shoulder.Out_Mixed_Control(Shoulder_target_pos,spd,50.0f,1.0f);
		Tigerarm_Elbow.Out_Mixed_Control(Elbow_target_pos,spd,30.0f,1.0f);
		vTaskDelay(1);
	}
	//ArmMotorSetZeroConfig();
	
}

void ArmMotorInit()
{
	vTaskDelay(2000);
	Tigerarm_Shoulder.To_Into_Control();
	vTaskDelay(500);
	Tigerarm_Elbow.To_Into_Control();
	//vTaskDelay(100);
	yaw_async_controller.setSpeedConstrain(75.0f);
	elbow_async_controller.setSpeedConstrain(1.0f);
	shoulder_async_controller.setSpeedConstrain(1.0f);
	//ArmMotorSetZeroConfig();
	
	
	
	Elbow_target_pos=Tigerarm_Elbow.get_current_position();
	Tigerarm_Elbow.Out_Mixed_Control(Elbow_target_pos,spd,80.0f,1.0f);
	Shoulder_target_pos=Tigerarm_Shoulder.get_current_position();
	ArmShoulderInit();	//slow
	
	Shoulder_target_pos=Tigerarm_Shoulder.get_current_position();
	Tigerarm_Shoulder.Out_Mixed_Control(Shoulder_target_pos, spd, Shoulder_kp, Shoulder_kd);
	//Tigerarm_Elbow.Set_ZeroPosition();
	Elbow_target_pos=Tigerarm_Elbow.get_current_position();
	ArmElbowInit();		//slow
	
	ArmYawInit(); //yaw slowly set zero config to foward 
	//set 3rd joint
	Elbow_target_pos=Tigerarm_Elbow.get_current_position()- 2.0f;
	elbow_async_controller.resetStepTarget(Elbow_target_pos, Tigerarm_Elbow.get_current_position(), xTaskGetTickCount());	//asynchorously set target
	while (fabs(Tigerarm_Elbow.get_current_position()-Elbow_target_pos)>0.1f)
	{ 
		Tigerarm_Shoulder.Out_Mixed_Control(Shoulder_target_pos, spd, Shoulder_kp, Shoulder_kd);
		Tigerarm_Elbow.Out_Mixed_Control(elbow_async_controller.getSteppingTarget(),spd,30.0f,1.0f);
		vTaskDelay(1);
	}
	flag=2;
	
	Elbow_Zero_Offset=Elbow_target_pos;
	Shoulder_target_pos=Tigerarm_Shoulder.get_current_position()-2.25f;
	shoulder_async_controller.resetStepTarget(Shoulder_target_pos, Tigerarm_Shoulder.get_current_position(), xTaskGetTickCount());
	while (fabs(Tigerarm_Shoulder.get_current_position()-Shoulder_target_pos)>0.1f) 
	{
		Tigerarm_Shoulder.Out_Mixed_Control(shoulder_async_controller.getSteppingTarget(), spd, Shoulder_kp, Shoulder_kd);
		Elbow_target_pos=Elbow_Zero_Offset+2.25f-fabs(Shoulder_target_pos-shoulder_async_controller.getSteppingTarget());
		Tigerarm_Elbow.Out_Mixed_Control(Elbow_target_pos,spd,30.0f,1.0f);
		vTaskDelay(1);
	}
	flag=3;
	Motor_CAN_COB Motor_TxMsg;
	Yaw_target_pos+=270.0f;
	yaw_async_controller.resetStepTarget(Yaw_target_pos,Tigerarm_Yaw.getAngle(),xTaskGetTickCount());
	Tigerarm_Yaw_Ctrl.setTarget(yaw_async_controller.getSteppingTarget());
	while (fabs(Tigerarm_Yaw.getAngle()-Yaw_target_pos)>1.0f)
	{
		Tigerarm_Yaw_Ctrl.setTarget(yaw_async_controller.getSteppingTarget());
		Tigerarm_Yaw_Ctrl.Adjust();
		MotorMsgPack(Motor_TxMsg, Tigerarm_Yaw);
		xQueueSendFromISR(CAN1_TxPort,&Motor_TxMsg.Low,0);
		vTaskDelay(1);
	}
	
	Elbow_Zero_Offset=Elbow_target_pos;
	Shoulder_Zero_Offset=Shoulder_target_pos;
	Yaw_Zero_Offset=Yaw_target_pos;
}

void Task_LinearTargetCtrl(void *arg)
{
	  /* Cache for Task */
		
	  /* Pre-Load for task */

	  /* Infinite loop */
	  TickType_t xLastWakeTime_t;
	  xLastWakeTime_t = xTaskGetTickCount();
	  for(;;)
	  {
		shoulder_async_controller.spinOnce(xTaskGetTickCount());
		elbow_async_controller.spinOnce(xTaskGetTickCount());
		yaw_async_controller.spinOnce(xTaskGetTickCount());
	    /* Pass control to the next task */
	    vTaskDelayUntil(&xLastWakeTime_t,1);
	  }
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
		if (fabs(Shoulder_target_pos-shoulder_async_controller.getTarget())>0.1f) shoulder_async_controller.resetStepTarget(Shoulder_target_pos, Tigerarm_Shoulder.get_current_position(), xTaskGetTickCount());
		Tigerarm_Shoulder.Out_Mixed_Control(shoulder_async_controller.getSteppingTarget(), spd, Shoulder_kp, Shoulder_kd);
		if (fabs(Elbow_target_pos-elbow_async_controller.getTarget())>0.1f) elbow_async_controller.resetStepTarget(Elbow_target_pos, Tigerarm_Elbow.get_current_position(), xTaskGetTickCount());
		Tigerarm_Elbow.Out_Mixed_Control(elbow_async_controller.getSteppingTarget(), spd, Elbow_kp, Elbow_kd);
		Tigerarm_Yaw_Ctrl.setTarget(yaw_async_controller.getSteppingTarget());
		Tigerarm_Yaw_Ctrl.Adjust();
		MotorMsgPack(Motor_TxMsg, Tigerarm_Yaw);
		xQueueSendFromISR(CAN1_TxPort,&Motor_TxMsg.Low,0);
	    /* Pass control to the next task */
	    vTaskDelayUntil(&xLastWakeTime_t,1);
	  }
}

void Task_ServoCtrl(void *arg)
{
	  /* Cache for Task */
		
	  /* Pre-Load for task */

	  /* Infinite loop */
	  TickType_t xLastWakeTime_t;
	  xLastWakeTime_t = xTaskGetTickCount();
	  for(;;)
	  {

	    /* Pass control to the next task */
	    vTaskDelayUntil(&xLastWakeTime_t,1);
	  }
}

void Task_JointSpdTest(void *arg)
{
	  /* Cache for Task */

	  /* Pre-Load for task */

	  /* Infinite loop */
	  TickType_t xLastWakeTime_t;
	  xLastWakeTime_t = xTaskGetTickCount();
	  for(;;)
	  {

	    /* Pass control to the next task */
	    vTaskDelayUntil(&xLastWakeTime_t,1);
	  }
}
