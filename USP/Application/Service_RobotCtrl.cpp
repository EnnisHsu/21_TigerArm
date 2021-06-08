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
	pump_controller.SetRelayStatus(pump_controller.Relay_Off);
	//xTaskCreate(Task_ArmSingleCtrl, "Robot.ArmSingleCtrl", Tiny_Stack_Size, NULL, PriorityNormal, &Robot_ArmSingleCtrl);
	xTaskCreate(Task_DR16Ctrl, "Robot.DR16Ctrl", Tiny_Stack_Size, NULL, PrioritySuperHigh, &Robot_DR16Ctrl);
	xTaskCreate(Task_ROSCtrl, "Robot.ROSCtrl", Normal_Stack_Size, NULL, PrioritySuperHigh, &Robot_ROSCtrl);
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
				switch (DR16.GetS2())
				{
					case DR16_SW_UP:
						switch (DR16.GetS1())
						{
							case DR16_SW_UP:
								pump_controller.SetRelayStatus(pump_controller.Relay_On);
								break;
							case DR16_SW_MID:
								pump_controller.SetRelayStatus(pump_controller.Relay_Off);
								break;
							case DR16_SW_DOWN:
								break;
							default:
								break;
						}
					case DR16_SW_MID:
						switch (DR16.GetS1())
						{
							case DR16_SW_UP:
								break;
							case DR16_SW_MID:
								break;
							case DR16_SW_DOWN:
								break;
							default:
								break;
						}
					case DR16_SW_DOWN:
						switch (DR16.GetS1())
						{
							case DR16_SW_UP:
								break;
							case DR16_SW_MID:
								break;
							case DR16_SW_DOWN:
								break;
							default:
								break;
						}							
				}
		  }

	    /* Pass control to the next task */
	    vTaskDelayUntil(&xLastWakeTime_t,1);
	  }
}

void Task_ROSCtrl(void *arg)
{
	vTaskSuspend(Robot_ROSCtrl);
	  /* Cache for Task */
	USART_COB _buffer;
	static TickType_t _xTicksToWait = pdMS_TO_TICKS(1);
	  /* Pre-Load for task */

	  /* Infinite loop */
	  TickType_t xLastWakeTime_t;
	  xLastWakeTime_t = xTaskGetTickCount();
	  for(;;)
	  {
			error_flag=99;
		if (xQueueReceive(NUC_QueueHandle, &_buffer, _xTicksToWait) == pdTRUE)
		{
			memcpy(deg,_buffer.address,_buffer.len);
			yaw_controller.setStepTarget(yaw_controller.getZeroOffset()+deg[0]*yaw_controller.getReductionRatio());
			arm_controller.setStepTarget(arm_controller.getZeroOffset()+deg[1]*arm_controller.getReductionRatio());
			elbow_controller.setStepTarget(elbow_controller.getZeroOffset()+deg[2]*elbow_controller.getReductionRatio());
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
 
  /* BoardMsgArr
	[0][0] Mode:								
					ForwardChassis = 0xd0,			
					BackwardChassis =0xd1,		
					AutoCatch =0xd2,
					ManualCatch =0xd3,
					Rescure =0xd4
	[0][1] _w
	[0][2] _a
	[0][3] _s 
	[0][4] _d
	[0][5] _ctrl 
	[0][6] _shift
	[0][7] CRC 
	ID:0x600
*/

 uint8_t* Key_Pack()
 {
	 static uint8_t* Msg_Arr;
	 for (int i=DR16_KEY_W;i<=DR16_KEY_CTRL;i++)
	 {
		 Msg_Arr[i+1]=0xff;
	 }
	 Msg_Arr[7]=Msg_Arr[0] xor Msg_Arr[1] xor Msg_Arr[2] xor Msg_Arr[3] xor Msg_Arr[4] xor Msg_Arr[5] xor Msg_Arr[6];
	 return Msg_Arr;
 }
 
 void Task_BoardCommunication(void *arg)
 {
	  /* Cache for Task */
		USART_COB BoardMsgCOB;
		uint8_t* TxMsg;
	  /* Pre-Load for task */

	  /* Infinite loop */
	  TickType_t xLastWakeTime_t;
	  xLastWakeTime_t = xTaskGetTickCount();
	  for(;;)
	  {
			TxMsg=Key_Pack();
			BoardMsgCOB.port_num=6;
			BoardMsgCOB.len=sizeof(TxMsg);
			BoardMsgCOB.address=TxMsg;
			xQueueSendFromISR(USART_TxPort,&BoardMsgCOB,0);
	    /* Pass control to the next task */
	    vTaskDelayUntil(&xLastWakeTime_t,1);
	  }	 
 }