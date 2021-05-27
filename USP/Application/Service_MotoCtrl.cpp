#include "Service_MotoCtrl.h"

Godzilla_Yaw_Controller yaw_controller(1, 75.0f);

float Shoulder_kp=50.0f,Shoulder_kd=2.0f,Elbow_kp=100.0f,Elbow_kd=1.0f;
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

    MotorMsgPack(Motor_TxMsg, yaw_controller.joint_motor);
    xQueueSendFromISR(CAN2_TxPort, &Motor_TxMsg.Low, 0);

    /* Pass control to the next task */
    vTaskDelayUntil(&xLastWakeTime_t, 1);
  }
}
