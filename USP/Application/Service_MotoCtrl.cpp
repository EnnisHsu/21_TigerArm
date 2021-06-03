#include "Service_MotoCtrl.h"
#include "Service_RobotCtrl.h"

Godzilla_Yaw_Controller yaw_controller(1, 6.28f,-4.71f,14.1f,3.0f);
Godzilla_Arm_Controller arm_controller(0x02,&hcan2,2.0f,-5.25f,2.25f,65.0f/30.0f);
Godzilla_Elbow_Controller elbow_controller(0x01,&hcan2,2.0f,-4.53f,1.32f,65.0f/30.0f);
Godzilla_Servo_Controller wristroll_controller(&htim2,TIM_CHANNEL_2),wristpitch_controller(&htim3,TIM_CHANNEL_1),wristyaw_controller(&htim3,TIM_CHANNEL_2);

float arm_kp=75.0f,arm_kd=2.0f,elbow_kp=100.0f,elbow_kd=1.0f;
float Motor_Max_Speed=10.0f;

TaskHandle_t ServiceMotoCtrl_Handle;
TaskHandle_t MotorInit_Handle;
TaskHandle_t ServoCtrl_Handle;

void Service_MotoCtrl_Init()
{
  //ArmMotorInit();
  xTaskCreate(Task_ArmMotorCtrl, "ArmMotorCtrl", Normal_Stack_Size, NULL, PrioritySuperHigh, &ServiceMotoCtrl_Handle);
  xTaskCreate(Task_ArmMotorInit,"ArmMotorInit", Normal_Stack_Size, NULL, PriorityAboveNormal, &MotorInit_Handle);
  xTaskCreate(Task_ServoCtrl,"Servo.Ctrl",Normal_Stack_Size,NULL,PriorityAboveNormal,&ServoCtrl_Handle);
}

void Task_ArmMotorInit(void *arg)
{
	vTaskDelay(2000);
  PID_Param_Typedef spd_pid_param = {50.0f, 0.0f, 0.0f, 0.0f, 1000.0f, 0.0f, 20000.0f};
  PID_Param_Typedef ang_pid_param = { 25.0f, 0.0f, 0.0f, 0.0f, 1000.0f, 0.0f, 30000.0f};
  yaw_controller.init(spd_pid_param, ang_pid_param);
	elbow_controller.init();
	vTaskDelay(500);
	arm_controller.init(&elbow_controller);
	
	
	
  /* Wait 1s for other tasks */
	
	arm_controller.slowlyMoveToLimit();
	elbow_controller.slowlyMoveToLimit();
	vTaskDelay(100);
  yaw_controller.slowlyMoveToLimit();
	//vTaskDelay(2000);
	yaw_controller.async_controller.setCurrent(yaw_controller.getCurrentAngle());
	arm_controller.async_controller.setCurrent(arm_controller.getCurrentAngle());
	elbow_controller.async_controller.setCurrent(elbow_controller.getCurrentAngle());
	elbow_controller.setCurrentAsTarget();
	arm_controller.setCurrentAsTarget();

	yaw_controller.setCurrentAsTarget();
	vTaskResume(ServiceMotoCtrl_Handle);
	elbow_controller.setStepTarget(elbow_controller.getCurrentAngle()-2.2f);
	arm_controller.setStepTarget(arm_controller.getCurrentAngle()-2.4f);
	yaw_controller.setStepTarget(yaw_controller.getCurrentAngle()+4.71f);
	vTaskDelay(2000);
	yaw_controller.setCurrentAsZero();
	elbow_controller.setCurrentAsZero();
	arm_controller.setCurrentAsZero();
	Service_RobotCtrl_Init();
	vTaskDelete(MotorInit_Handle);
	for (;;)
	{
		vTaskDelay(1);
	}
}







void Task_ServoCtrl(void *arg)
{ 
  /* Cache for Task */
	
  /* Pre-Load for task */
	yaw_controller.async_controller.setCurrent(yaw_controller.getCurrentAngle());
  /* Infinite loop */
  TickType_t xLastWakeTime_t;
  xLastWakeTime_t = xTaskGetTickCount();

  for(;;)
  {
    wristroll_controller.Output();
		vTaskDelayUntil(&xLastWakeTime_t, 20);
		wristpitch_controller.Output();
		vTaskDelayUntil(&xLastWakeTime_t, 20);
		wristyaw_controller.Output();
		vTaskDelayUntil(&xLastWakeTime_t, 20);
		
  }	
}

void Task_ArmMotorCtrl(void *arg)
{
	vTaskSuspend(ServiceMotoCtrl_Handle);
  /* Cache for Task */
  Motor_CAN_COB Motor_TxMsg;

  /* Pre-Load for task */

  /* Infinite loop */
  TickType_t xLastWakeTime_t;
  xLastWakeTime_t = xTaskGetTickCount();

  for(;;)
  {
    /* Spin linear interpolation */
    yaw_controller.spinOnce();
		arm_controller.spinOnce();
		elbow_controller.spinOnce();

    MotorMsgPack(Motor_TxMsg, yaw_controller.joint_motor);
    xQueueSendFromISR(CAN2_TxPort, &Motor_TxMsg.Low, 0);
		elbow_controller.Output=elbow_controller.async_controller.getSteppingTarget()-(arm_controller.async_controller.getSteppingTarget()-arm_controller.async_controller.getActCur());
		arm_controller.joint_motor.Out_Mixed_Control(arm_controller.async_controller.getSteppingTarget(),Motor_Max_Speed,arm_kp,arm_kd);
		elbow_controller.joint_motor.Out_Mixed_Control(elbow_controller.Output,Motor_Max_Speed,elbow_kp,elbow_kd);
		//TestServo();
    /* Pass control to the next task */
    vTaskDelayUntil(&xLastWakeTime_t, 1);
  }
}
