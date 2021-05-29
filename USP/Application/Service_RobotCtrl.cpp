/**
  ******************************************************************************
  * Copyright (c) 2021 - ~, SCUT-RobotLab Development Team
  * @file    xxx.cpp
  * @author  EnnisKoh (8762322@qq.com) 
  * @brief   
  * @date    2021-05-02
  * @version 0.1.0
  * @par Change Log:
  * <table>
  * <tr><th>Date        <th>Version  <th>Author    				 <th>Description
  * <tr><td>2021-00-00  <td> 0.1.0   <td>EnnisKoh     <td>Creator
  * </table>
  *
  ==============================================================================
                            How to use this driver     
  ==============================================================================
    @note 
		
  ******************************************************************************
  * @attention
  * 
  * if you had modified this file, please make sure your code does not have any 
  * bugs, update the version Number, write dowm your name and the date. The most
  * important thing is make sure the users will have clear and definite under-
  * standing through your new brief.
  *
  * Copyright (c) 2021 - ~, SCUT-RobotLab Development Team.
  * All rights reserved.</center></h2>
  ******************************************************************************
  */
#include "Service_RobotCtrl.h"
#include "Service_MotoCtrl.h"

float deg[6];
TaskHandle_t Robot_ROSCtrl;
TaskHandle_t Robot_ArmSingleCtrl;
TaskHandle_t Robot_DR16Ctrl;
TaskHandle_t Robot_KeyboardCtrl;

void Service_RobotCtrl_Init()
{
	//xTaskCreate(Task_ArmSingleCtrl, "Robot.ArmSingleCtrl", Tiny_Stack_Size, NULL, PriorityNormal, &Robot_ArmSingleCtrl);
	xTaskCreate(Task_DR16Ctrl, "Robot.DR16Ctrl", Normal_Stack_Size, NULL, PrioritySuperHigh, &Robot_DR16Ctrl);
	//xTaskCreate(Task_ROSCtrl, "Robot.ROSCtrl", Normal_Stack_Size, NULL, PrioritySuperHigh, &Robot_ROSCtrl);
}

void Task_ArmSingleCtrl(void *arg)
{
	  /* Cache for Task */
		float move_step=0.02f;
		static TickType_t _xTicksToWait = pdMS_TO_TICKS(1);
	  /* Pre-Load for task */

	  /* Infinite loop */
	  TickType_t xLastWakeTime_t;
	  xLastWakeTime_t = xTaskGetTickCount();
	  for(;;)
	  {
		  if (xTaskNotifyWait(0x00000000, 0xffffffff, NULL, _xTicksToWait)==pdTRUE)
		  {
		    /*
			  if (DR16.Get_LX_Norm()<-0.5f && abs(Tigerarm_Elbow.get_current_position()-Elbow_target_pos)<move_step)
				  Elbow_target_pos-=move_step;
			  if (DR16.Get_LX_Norm()>0.5f && abs(Tigerarm_Elbow.get_current_position()-Elbow_target_pos)<move_step)
				  Elbow_target_pos+=move_step;
			  if (DR16.Get_LY_Norm()<-0.5f && abs(Tigerarm_Shoulder.get_current_position()-Shoulder_target_pos)<move_step)
				  Shoulder_target_pos-=move_step;
			  if (DR16.Get_LX_Norm()>0.5f && abs(Tigerarm_Shoulder.get_current_position()-Shoulder_target_pos)<move_step)
				  Shoulder_target_pos+=move_step;
			  if (DR16.Get_RX_Norm()<-0.5f && abs(Tigerarm_Yaw.getAngle()-Yaw_target_pos)<1.0f)
				  Yaw_target_pos-=1.0f;
			  if (DR16.Get_RX_Norm()>0.5f && abs(Tigerarm_Yaw.getAngle()-Yaw_target_pos)<1.0f)
				  Yaw_target_pos+=1.0f;
				 */
		  }
	    /* Pass control to the next task */
	    vTaskDelayUntil(&xLastWakeTime_t,5);
	  }
}

 void Task_DR16Ctrl(void *arg)
{
	  /* Cache for Task */

	  /* Pre-Load for task */

	  /* Infinite loop */
	  TickType_t xLastWakeTime_t;
	  xLastWakeTime_t = xTaskGetTickCount();
	  for(;;)
	  {
		  if (DR16.GetStatus()==DR16_ESTABLISHED)
		  {
			  if (DR16.GetS1()==DR16_SW_UP && DR16.GetS2()==DR16_SW_DOWN)
			  {
				  xTaskNotify(Robot_ArmSingleCtrl,(uint32_t)NULL,eSetBits);
			  }
			  if (DR16.GetS1()==DR16_SW_DOWN && DR16.GetS2()==DR16_SW_UP)
			  {

			  }
		  }

	    /* Pass control to the next task */
	    vTaskDelayUntil(&xLastWakeTime_t,1);
	  }
}

void Task_ROSCtrl(void *arg)
{
	  /* Cache for Task */
	USART_COB _buffer;
	static TickType_t _xTicksToWait = pdMS_TO_TICKS(1);
	  /* Pre-Load for task */

	  /* Infinite loop */
	  TickType_t xLastWakeTime_t;
	  xLastWakeTime_t = xTaskGetTickCount();
	  for(;;)
	  {
		if (xQueueReceive(NUC_QueueHandle, &_buffer, _xTicksToWait) == pdTRUE)
		{
		  
			memcpy(deg,_buffer.address,_buffer.len);
			yaw_controller.setStepTarget(yaw_controller.getZeroOffset()+rad2deg(deg[0]));
			arm_controller.setStepTarget(arm_controller.getZeroOffset()+deg[1]);
			elbow_controller.setStepTarget(elbow_controller.getZeroOffset()+deg[2]);
			wristroll_controller.SetTargetAngle(rad2deg(deg[3]));
			wristpitch_controller.SetTargetAngle(rad2deg(deg[4]));
			wristyaw_controller.SetTargetAngle(rad2deg(deg[5]));
		}
	    /* Pass control to the next task */
	    vTaskDelayUntil(&xLastWakeTime_t,1);
	  }	
}

 void Task_KeyboardCtrl(void *arg)
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
