#include "Service_MotoCtrl.h" 
#include "Service_RobotCtrl.h"
#include "Service_RelayCtrl.h"
#define Fast_Mode 

#ifdef Fast_Mode
	#define Yaw_Limit_Spd 9.42f
	#define Arm_Limit_Spd 6.28f
	#define Elbow_Limit_Spd 6.28f
#else
	#define Yaw_Limit_Spd 6.28f
	#define Arm_Limit_Spd 2.5f
	#define Elbow_Limit_Spd 2.5f
#endif

Godzilla_Yaw_Controller yaw_controller(1,Yaw_Limit_Spd,-4.71f,12.56f,3.0f);
Godzilla_Arm_Controller arm_controller(0x02,&hcan2,Arm_Limit_Spd,-5.25f,2.25f,65.0f/30.0f);
Godzilla_Elbow_Controller elbow_controller(0x01,&hcan2,Elbow_Limit_Spd,-4.53f,1.32f,65.0f/30.0f);
Godzilla_Servo_Controller wristroll_controller(&htim2,TIM_CHANNEL_2,wristroll_controller.Servo360),
		wristpitch_controller(&htim3,TIM_CHANNEL_1,wristpitch_controller.Servo180,1190),
		wristyaw_controller(&htim3,TIM_CHANNEL_2,wristyaw_controller.Servo180,1450);



float arm_kp=100.0f,arm_kd=2.5f,elbow_kp=100.0f,elbow_kd=2.0f;
float Motor_Max_Speed=10.0f;

TaskHandle_t ServiceMotoCtrl_Handle;
TaskHandle_t MotorInit_Handle;
TaskHandle_t ServoCtrl_Handle;

void Service_MotoCtrl_Init()
{
  //ArmMotorInit();
  xTaskCreate(Task_ArmMotorCtrl, "ArmMotorCtrl", Normal_Stack_Size, NULL, PriorityNormal, &ServiceMotoCtrl_Handle);
  #ifndef _IngoreInit
		xTaskCreate(Task_ArmMotorInit,"ArmMotorInit", Normal_Stack_Size, NULL, PriorityAboveNormal, &MotorInit_Handle);
	#endif
  xTaskCreate(Task_ServoCtrl,"Servo.Ctrl",Normal_Stack_Size,NULL,PriorityNormal,&ServoCtrl_Handle);
}

void Task_ArmMotorInit(void *arg)
{

	
	for (;;)
	{
		/* Tigerarm Motor Init */
		vTaskSuspend(ServiceMotoCtrl_Handle);
		pump_controller.SetRelayStatus(pump_controller.Relay_Off);
		clamp_controller.SetRelayStatus(clamp_controller.Relay_Off);
		hook_controller.SetRelayStatus(hook_controller.Relay_Off);
		card_controller.SetRelayStatus(card_controller.Relay_Off);
		elbow_controller.joint_motor.To_Exit_Control();
		arm_controller.joint_motor.To_Exit_Control();
		vTaskDelay(2000);
		PID_Param_Typedef spd_pid_param = {70.0f, 0.0f, 0.0f, 0.0f, 1000.0f, 0.0f, 30000.0f};
		PID_Param_Typedef ang_pid_param = { 70.0f, 0.0f, 0.0f, 0.0f, 1000.0f, 0.0f, 30000.0f};
		yaw_controller.init(spd_pid_param, ang_pid_param);
		elbow_controller.init();
		vTaskDelay(100);
		arm_controller.init(&elbow_controller);
	
		/* Slowly Move to Limit */
		arm_controller.slowlyMoveToLimit();
		elbow_controller.slowlyMoveToLimit();
		yaw_controller.slowlyMoveToLimit();

		/* Set Limit as Target & Zero */
		yaw_controller.async_controller.setCurrent(yaw_controller.getCurrentAngle());
		arm_controller.async_controller.setCurrent(arm_controller.getCurrentAngle());
		elbow_controller.async_controller.setCurrent(elbow_controller.getCurrentAngle());
		yaw_controller.setCurrentAsZero();
		elbow_controller.setCurrentAsZero();
		arm_controller.setCurrentAsZero();
		elbow_controller.setCurrentAsTarget();
		arm_controller.setCurrentAsTarget();
		yaw_controller.setCurrentAsTarget();
	
		/* Turn to Prepare Position */
		yaw_controller.async_controller.setCubicConfig_tf(1000);
		elbow_controller.async_controller.setCubicConfig_tf(1000);
		arm_controller.async_controller.setCubicConfig_tf(1000);
		yaw_controller.async_controller.Spd_Limit=1;
		arm_controller.async_controller.Spd_Limit=1;
		elbow_controller.async_controller.Spd_Limit=1;
		vTaskResume(ServiceMotoCtrl_Handle);
		elbow_controller.setStepTarget(elbow_controller.getCurrentAngle()-2.2f);
		arm_controller.setStepTarget(arm_controller.getCurrentAngle()-2.4f);
		yaw_controller.setStepTarget(yaw_controller.getCurrentAngle()+4.71f);
		vTaskDelay(1500);
		yaw_controller.async_controller.Spd_Limit=0;
		arm_controller.async_controller.Spd_Limit=0;
		elbow_controller.async_controller.Spd_Limit=0;
		/* Set Prepare Postion as Target & Zero */
		vTaskSuspend(ServiceMotoCtrl_Handle);
		yaw_controller.async_controller.setCurrent(yaw_controller.getCurrentAngle());
		elbow_controller.async_controller.setCurrent(elbow_controller.getCurrentAngle());
		arm_controller.async_controller.setCurrent(arm_controller.getCurrentAngle());
		yaw_controller.setCurrentAsZero();
		elbow_controller.setCurrentAsZero();
		arm_controller.setCurrentAsZero();
		elbow_controller.setCurrentAsTarget();
		arm_controller.setCurrentAsTarget();
		yaw_controller.setCurrentAsTarget();	
		yaw_controller.async_controller.setCubicConfig_tf(100);
		elbow_controller.async_controller.setCubicConfig_tf(100);
		arm_controller.async_controller.setCubicConfig_tf(100);
		vTaskDelay(10);
		vTaskResume(ServiceMotoCtrl_Handle);
		vTaskSuspend(MotorInit_Handle);
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
		error_flag=70;
    	wristroll_controller.Output();
		//vTaskDelayUntil(&xLastWakeTime_t, 20);
		wristpitch_controller.Output();
		//vTaskDelayUntil(&xLastWakeTime_t, 20);
		wristyaw_controller.Output();
		vTaskDelayUntil(&xLastWakeTime_t, 20);
		
  }	
}

void Task_ArmMotorCtrl(void *arg)
{
	#ifndef _IngoreInit
		vTaskSuspend(ServiceMotoCtrl_Handle);
  #else
		yaw_controller.async_controller.setCubicConfig_tf(100);
		elbow_controller.async_controller.setCubicConfig_tf(100);
		arm_controller.async_controller.setCubicConfig_tf(100);
	#endif
	/* Cache for Task */
  Motor_CAN_COB Motor_TxMsg;

  /* Pre-Load for task */

  /* Infinite loop */
  TickType_t xLastWakeTime_t;
  xLastWakeTime_t = xTaskGetTickCount();
	static int cnt = 0;
  for(;;)
  {
    /* Spin linear interpolation */
		error_flag=75;
    	yaw_controller.spinOnce();
		arm_controller.spinOnce();
		elbow_controller.spinOnce();

    	MotorMsgPack(Motor_TxMsg, yaw_controller.joint_motor);
    	xQueueSendFromISR(CAN2_TxPort, &Motor_TxMsg.Low, 0);
		elbow_controller.Output=elbow_controller.async_controller.getSteppingTarget()-(arm_controller.async_controller.getSteppingTarget()-arm_controller.getZeroOffset());
		arm_controller.joint_motor.Out_Mixed_Control(arm_controller.async_controller.getSteppingTarget(),Motor_Max_Speed,arm_kp,arm_kd);
		elbow_controller.joint_motor.Out_Mixed_Control(elbow_controller.Output,Motor_Max_Speed,elbow_kp,elbow_kd);
    /* Pass control to the next task */
    vTaskDelayUntil(&xLastWakeTime_t, 1);
  }
}

