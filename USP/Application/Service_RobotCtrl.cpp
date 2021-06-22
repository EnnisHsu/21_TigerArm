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
#include "Service_Communication.h"



float deg[6];
CEngineer TigerArm;
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
	xTaskCreate(Task_KeyboardCtrl,"Robot.KeyboardCtrl",Tiny_Stack_Size,NULL,PrioritySuperHigh,&Robot_KeyboardCtrl);
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
								//TigerArm.Switch_Mode(TigerArm.ManualCatch);
								pump_controller.SetRelayStatus(pump_controller.Relay_On);
								break;
							case DR16_SW_MID:
								//TigerArm.Switch_Mode(TigerArm.ManualCatch);
								pump_controller.SetRelayStatus(pump_controller.Relay_Off);
								break;
							case DR16_SW_DOWN:
								break;
							default:
								break;
						}
						break;
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
						break;
					case DR16_SW_DOWN:
						
						switch (DR16.GetS1())
						{
							case DR16_SW_UP:
								break;
							case DR16_SW_MID:
								break;
							case DR16_SW_DOWN:
								xTaskNotify(Robot_KeyboardCtrl,NULL,eNoAction);
								break;
							default:
								break;
						}			
						break;
						default:
							break;
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
			error_flag=99;
			//if (TigerArm.Get_Current_Mode()==TigerArm.AutoCatch)
			//{
				if (xQueueReceive(NUC_QueueHandle, &_buffer, _xTicksToWait) == pdTRUE)
				{
					if (_buffer.len==24)
					{
						memcpy(deg,_buffer.address,_buffer.len);
						yaw_controller.setStepTarget(yaw_controller.getZeroOffset()+deg[0]*yaw_controller.getReductionRatio());
						arm_controller.setStepTarget(arm_controller.getZeroOffset()+deg[1]*arm_controller.getReductionRatio());
						elbow_controller.setStepTarget(elbow_controller.getZeroOffset()+deg[2]*elbow_controller.getReductionRatio());
						wristroll_controller.SetTargetAngle(rad2deg(deg[3]));
						wristpitch_controller.SetTargetAngle(rad2deg(deg[4]));
						wristyaw_controller.SetTargetAngle(rad2deg(deg[5]));
					}
					if (_buffer.len==2)
					{
						static int8_t ros_command[2];
						memcpy(ros_command,_buffer.address,_buffer.len);
						ros_command[0]==1?pump_controller.SetRelayStatus(pump_controller.Relay_Off):(void)NULL;
						ros_command[1]==1?TigerArm.Switch_CommandStatus(TigerArm.Engineer_CommandWait):(void)NULL;
					}
					if (_buffer.len==26)
					{
						static int8_t ros_command[2];
						static uint8_t msg_buff[26];
						memcpy(msg_buff,_buffer.address,_buffer.len);
						memcpy(deg,msg_buff,24);
						yaw_controller.setStepTarget(yaw_controller.getZeroOffset()+deg[0]*yaw_controller.getReductionRatio());
						arm_controller.setStepTarget(arm_controller.getZeroOffset()+deg[1]*arm_controller.getReductionRatio());
						elbow_controller.setStepTarget(elbow_controller.getZeroOffset()+deg[2]*elbow_controller.getReductionRatio());
						wristroll_controller.SetTargetAngle(rad2deg(deg[3]));
						wristpitch_controller.SetTargetAngle(rad2deg(deg[4]));
						wristyaw_controller.SetTargetAngle(rad2deg(deg[5]));
						memcpy(ros_command,msg_buff+24,2);
						ros_command[0]==1?pump_controller.SetRelayStatus(pump_controller.Relay_Off):(void)NULL;
						ros_command[1]==1?TigerArm.Switch_CommandStatus(TigerArm.Engineer_CommandWait):(void)NULL;
					}
				}
			//}
	    /* Pass control to the next task */
	    vTaskDelayUntil(&xLastWakeTime_t,1);
	  }	
}

void Send_Command_To_NUC(uint32_t command)
{
	static uint32_t Msg;
	Msg=command;
	TigerArm.Switch_CommandStatus(TigerArm.Engineer_CommandLock);
	(command==82)?pump_controller.SetRelayStatus(pump_controller.Relay_On):(void)NULL;
	Usart_Tx_Pack(USART_TxPort,4,sizeof(Msg),&Msg);
}

void Tigerarm_Space_Displacement()
{
	(DR16.IsKeyPress(DR16_KEY_W) && DR16.IsKeyPress(DR16_KEY_CTRL) && !DR16.IsKeyPress(DR16_KEY_SHIFT) && TigerArm.Get_Current_CommandStatus()==TigerArm.Engineer_CommandWait)?Send_Command_To_NUC(119):(void)NULL;
	(DR16.IsKeyPress(DR16_KEY_A) && DR16.IsKeyPress(DR16_KEY_CTRL) && !DR16.IsKeyPress(DR16_KEY_SHIFT) && TigerArm.Get_Current_CommandStatus()==TigerArm.Engineer_CommandWait)?Send_Command_To_NUC(97):(void)NULL;
	(DR16.IsKeyPress(DR16_KEY_S) && DR16.IsKeyPress(DR16_KEY_CTRL) && !DR16.IsKeyPress(DR16_KEY_SHIFT) && TigerArm.Get_Current_CommandStatus()==TigerArm.Engineer_CommandWait)?Send_Command_To_NUC(115):(void)NULL;
	(DR16.IsKeyPress(DR16_KEY_D) && DR16.IsKeyPress(DR16_KEY_CTRL) && !DR16.IsKeyPress(DR16_KEY_SHIFT) && TigerArm.Get_Current_CommandStatus()==TigerArm.Engineer_CommandWait)?Send_Command_To_NUC(100):(void)NULL;
	(DR16.IsKeyPress(DR16_KEY_Q) && DR16.IsKeyPress(DR16_KEY_CTRL) && !DR16.IsKeyPress(DR16_KEY_SHIFT) && TigerArm.Get_Current_CommandStatus()==TigerArm.Engineer_CommandWait)?Send_Command_To_NUC(87):(void)NULL;
	(DR16.IsKeyPress(DR16_KEY_C) && DR16.IsKeyPress(DR16_KEY_CTRL) && !DR16.IsKeyPress(DR16_KEY_SHIFT) && TigerArm.Get_Current_CommandStatus()==TigerArm.Engineer_CommandWait)?Send_Command_To_NUC(83):(void)NULL;
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
			if (xTaskNotifyWait(0x00,0xffffffff,NULL,0)==pdTRUE)
			{
				/*if (DR16.IsKeyPress(DR16_KEY_CTRL) && (DR16.IsKeyPress(DR16_KEY_Q) || DR16.IsKeyPress(DR16_KEY_E)))
				{
					if (DR16.IsKeyPress(DR16_KEY_Q)) TigerArm.Switch_Mode((CEngineer::Engineer_Mode_Typedef)((TigerArm.Get_Current_Mode()-0xd1)%6+0xd0));
					if (DR16.IsKeyPress(DR16_KEY_E)) TigerArm.Switch_Mode((CEngineer::Engineer_Mode_Typedef)((TigerArm.Get_Current_Mode()-0xcf)%6+0xd0));
				}
				else
				{*/
				if (DR16.IsKeyPress(DR16_KEY_B))
				{
					continue;
				}
				if (DR16.IsKeyPress(DR16_KEY_Q) && !DR16.IsKeyPress(DR16_KEY_CTRL) && !DR16.IsKeyPress(DR16_KEY_SHIFT) && TigerArm.Get_Current_CommandStatus()==TigerArm.Engineer_CommandWait) 
				{
					TigerArm.Switch_Mode(TigerArm.DrivingMode);
					Send_Command_To_NUC(114);
					continue;
				}
				if (DR16.IsKeyPress(DR16_KEY_E) && !DR16.IsKeyPress(DR16_KEY_CTRL) && !DR16.IsKeyPress(DR16_KEY_SHIFT) && TigerArm.Get_Current_CommandStatus()==TigerArm.Engineer_CommandWait) 
				{
					TigerArm.Switch_Mode(TigerArm.TaskingMode);
					Send_Command_To_NUC(114);
					continue;
				}
				if (DR16.IsKeyPress(DR16_KEY_Z) && !DR16.IsKeyPress(DR16_KEY_CTRL) && !DR16.IsKeyPress(DR16_KEY_SHIFT) && TigerArm.Get_Current_CommandStatus()==TigerArm.Engineer_CommandWait)
				{
					TigerArm.Switch_Mode(TigerArm.GoldenMineral);
					Send_Command_To_NUC(113);
					continue;
				}
				if (DR16.IsKeyPress(DR16_KEY_C) && !DR16.IsKeyPress(DR16_KEY_CTRL) && !DR16.IsKeyPress(DR16_KEY_SHIFT) && TigerArm.Get_Current_CommandStatus()==TigerArm.Engineer_CommandWait) 
				{
					TigerArm.Switch_Mode(TigerArm.ExchangeMode);
					Send_Command_To_NUC(113);
					continue;
				}
				switch (TigerArm.Get_Current_Mode())
				{
					case CEngineer::GoldenMineral:
						Tigerarm_Space_Displacement();
						(DR16.IsKeyPress(DR16_KEY_F) && !DR16.IsKeyPress(DR16_KEY_CTRL) && !DR16.IsKeyPress(DR16_KEY_SHIFT) && TigerArm.Get_Current_CommandStatus()==TigerArm.Engineer_CommandWait)?Send_Command_To_NUC(82):(void)NULL;
							(DR16.IsKeyPress(DR16_KEY_F) && !DR16.IsKeyPress(DR16_KEY_CTRL) && DR16.IsKeyPress(DR16_KEY_SHIFT) && TigerArm.Get_Current_CommandStatus()==TigerArm.Engineer_CommandWait)?Send_Command_To_NUC(122):(void)NULL;
						(DR16.IsKeyPress(DR16_KEY_G) && !DR16.IsKeyPress(DR16_KEY_CTRL) && !DR16.IsKeyPress(DR16_KEY_SHIFT) && TigerArm.Get_Current_CommandStatus()==TigerArm.Engineer_CommandWait)?Send_Command_To_NUC(118):(void)NULL;
						(DR16.IsKeyPress(DR16_KEY_F) && DR16.IsKeyPress(DR16_KEY_CTRL) && !DR16.IsKeyPress(DR16_KEY_SHIFT) && TigerArm.Get_Current_CommandStatus()==TigerArm.Engineer_CommandWait)?Send_Command_To_NUC(0):(void)NULL;//左pia
						(DR16.IsKeyPress(DR16_KEY_G) && DR16.IsKeyPress(DR16_KEY_CTRL) && !DR16.IsKeyPress(DR16_KEY_SHIFT) && TigerArm.Get_Current_CommandStatus()==TigerArm.Engineer_CommandWait)?Send_Command_To_NUC(0):(void)NULL;//右pia
						break;
					case CEngineer::ExchangeMode:
						/* Tigerarm Exchange Control */
						Tigerarm_Space_Displacement();
						(DR16.IsKeyPress(DR16_KEY_F) && !DR16.IsKeyPress(DR16_KEY_CTRL) && !DR16.IsKeyPress(DR16_KEY_SHIFT) && TigerArm.Get_Current_CommandStatus()==TigerArm.Engineer_CommandWait)?Send_Command_To_NUC(32):(void)NULL;//拿出矿舱矿石准备兑换
						(DR16.IsKeyPress(DR16_KEY_G) && !DR16.IsKeyPress(DR16_KEY_CTRL) && !DR16.IsKeyPress(DR16_KEY_SHIFT) && TigerArm.Get_Current_CommandStatus()==TigerArm.Engineer_CommandWait)?Send_Command_To_NUC(120):(void)NULL;//兑换矿石
						(DR16.IsKeyPress(DR16_KEY_F) && DR16.IsKeyPress(DR16_KEY_CTRL) && !DR16.IsKeyPress(DR16_KEY_SHIFT) && TigerArm.Get_Current_CommandStatus()==TigerArm.Engineer_CommandWait)?Send_Command_To_NUC(0):(void)NULL;//fuck一下
						(DR16.IsKeyPress(DR16_KEY_G) && DR16.IsKeyPress(DR16_KEY_CTRL) && !DR16.IsKeyPress(DR16_KEY_SHIFT) && TigerArm.Get_Current_CommandStatus()==TigerArm.Engineer_CommandWait)?Send_Command_To_NUC(0):(void)NULL;//翻转一下
						break;
					case CEngineer::Rescure:
							
						break;
					case CEngineer::Obstacles:
						//(DR16.IsKeyPress(DR16_KEY_Z) && TigerArm.Get_Current_CommandStatus()==TigerArm.Engineer_CommandWait)?Send_Command_To_NUC(116):(void)NULL;
						//(DR16.IsKeyPress(DR16_KEY_X) && TigerArm.Get_Current_CommandStatus()==TigerArm.Engineer_CommandWait)?Send_Command_To_NUC(113):(void)NULL;
						break;
					default:
						break;
				}
				
			}

	    /* Pass control to the next task */
	    vTaskDelayUntil(&xLastWakeTime_t,1);
	  }
 }
 