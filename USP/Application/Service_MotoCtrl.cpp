#include "Service_MotoCtrl.h"

Godzilla_Yaw_Controller yaw_controller(1, 75.0f);
Godzilla_Arm_Controller arm_controller(0x02,&hcan2,0.5f);
Godzilla_Elbow_Controller elbow_controller(0x01,&hcan2,0.5f);

float arm_kp=50.0f,arm_kd=2.0f,elbow_kp=100.0f,elbow_kd=1.0f;
float Motor_Max_Speed=10.0f;

float Wrist_Roll_pos, Wrist_Pitch_pos, Wrist_Yaw_pos;	//deg

TaskHandle_t ServiceMotoCtrl_Handle;
TaskHandle_t LinearTargetCtrl_Handle;
TaskHandle_t ServoCtrl_Handle;

void Service_MotoCtrl_Init()
{
  //ArmMotorInit();
  xTaskCreate(Task_ArmMotorCtrl, "ArmMotorCtrl", Normal_Stack_Size, NULL, PrioritySuperHigh, &ServiceMotoCtrl_Handle);
  xTaskCreate(Task_ArmMotorInit,"ArmMotorInit", Normal_Stack_Size, NULL, PriorityAboveNormal, &LinearTargetCtrl_Handle);
  //xTaskCreate(Task_ServoCtrl,"Servo.Ctrl",Normal_Stack_Size,NULL,PriorityAboveNormal,&ServoCtrl_Handle);
}

void Task_ArmMotorInit(void *arg)
{
  PID_Param_Typedef spd_pid_param = {100.0f, 0.0f, 0.0f, 0.0f, 1000.0f, 0.0f, 30000.0f};
  PID_Param_Typedef ang_pid_param = { 80.0f, 0.0f, 0.0f, 0.0f, 1000.0f, 0.0f, 20000.0f};
  yaw_controller.init(spd_pid_param, ang_pid_param);
	arm_controller.init();
	elbow_controller.init();
  /* Wait 1s for other tasks */
  vTaskDelay(1000);

  yaw_controller.slowlyMoveToLimit();
}

void Task_ArmMotorCtrl(void *arg)
{
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
		arm_controller.joint_motor.Out_Mixed_Control(arm_controller.async_controller.getSteppingTarget(),Motor_Max_Speed,arm_kp,arm_kd);
		elbow_controller.joint_motor.Out_Mixed_Control(elbow_controller.async_controller.getSteppingTarget(),Motor_Max_Speed,elbow_kp,elbow_kd);
		
    /* Pass control to the next task */
    vTaskDelayUntil(&xLastWakeTime_t, 1);
  }
}
