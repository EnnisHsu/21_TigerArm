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

CChassis Engineer_chassis(0,0,76,16000,9000);
CEngineer TigerArm;

float TargetVelocity_X,TargetVelocity_Y,TargetVelocity_Z;
float _div = 15.0f;
float _x_vel_div = 7000.0f;
float _y_vel_div = 7000.0f;
int is_RxNuc = 0;
	
myPID Chassis_Spd[4];
myPID Chassis_Pos[4];
myPID Chassis_Attitude_Yaw;
myPID TurnBack_Yaw;

TaskHandle_t Robot_ChassisCtrl;
TaskHandle_t Robot_OfflineCtrl;
TaskHandle_t Robot_GamepadCtrl;
TaskHandle_t Robot_Keyboard_Ctrl;
TaskHandle_t Robot_NUC_Ctrl;

_NUCComRx NUCComRxData;

void Service_RobotCtrl_Init()
{
	Engineer_Chassis_Init();
	xTaskCreate(Chassis_Ctrl, "Robot.ChassisCtrl", Normal_Stack_Size, NULL, PriorityNormal, &Robot_ChassisCtrl);
	//xTaskCreate(Offline_Ctrl, "Robot.OfflineCtrl", Normal_Stack_Size, NULL, PrioritySuperHigh, &Robot_OfflineCtrl);
	xTaskCreate(Gamepad_Ctrl, "Robot.GamepadCtrl", Normal_Stack_Size, NULL, PriorityNormal, &Robot_GamepadCtrl);
	xTaskCreate(Keyboard_Ctrl, "Robot.Keyboard_Ctrl", Normal_Stack_Size, NULL, PriorityNormal, &Robot_Keyboard_Ctrl);
	xTaskCreate(NUC_Ctrl, "Robot.NUC_Ctrl", Normal_Stack_Size, NULL, PriorityHigh, &Robot_NUC_Ctrl);
}

void Controller_PID_ParamTnit()
{
	
	/* _Kp _Ki,  _Kd,  _I_Term_Max, _Out_Max
	 * @note The whole restrict in class chassis's creation
	*/		
	Chassis_Spd[0].SetPIDParam(10 , 0 ,0 ,0 , 16000 );
	Chassis_Spd[1].SetPIDParam(10 , 0 ,0 ,0 , 16000 );
	Chassis_Spd[2].SetPIDParam(10 , 0 ,0 ,0 , 16000 );
	Chassis_Spd[3].SetPIDParam(10 , 0 ,0 ,0 , 16000 );
	
	/* vision alighment pid */	
	Chassis_Pos[0].SetPIDParam(15.0f	,0	,0	,0,1500);   //x
	Chassis_Pos[1].SetPIDParam(15.0f	,0	,0	,0,1500);	//y
	Chassis_Pos[2].SetPIDParam(25.0f	,0	,0	,0,1500);	//z

	/* attitude controller */
	Chassis_Attitude_Yaw.SetPIDParam(80.0f,0,0,0,1000);
	TurnBack_Yaw.SetPIDParam(0.8f,0,0,0,1000);

}

void Engineer_Chassis_Init()
{
	Engineer_chassis.Set_AccelerationParam(12000,30000,14000);
	Engineer_chassis.Set_SpeedGear(FAST_GEAR);
	Engineer_chassis.Set_TorqueOptimizeFlag(1);
	Engineer_chassis.Set_AttitudeOptimizeFlag(1);
	Engineer_chassis.Set_SpeedParam(0.35f,0.5f,1.0f,1.0f);
	Engineer_chassis.Switch_Mode(Normal_Speed);
	Engineer_chassis.Load_SpeedController(SpeedController);
	Engineer_chassis.Load_AttitudeController(AttitudeController);
	
	Controller_PID_ParamTnit();
}

int* SpeedController(const int16_t* current,const int16_t* target)
{
	static int Chassis_out[4];
	for (int index=0;index<4;index++)
	{
		Chassis_Spd[index].Target=(float)target[index];
		Chassis_Spd[index].Current=(float)current[index];
		Chassis_Spd[index].Adjust();
		Chassis_out[index]=Chassis_Spd[index].Out;
	}
	return Chassis_out;
}

_chassis_Velocity* AttitudeController(const _chassis_GlobalPos* Current_Pos,const _chassis_GlobalPos* Command_Pos)
{
	static _chassis_Velocity _chassis_Velocity_Attitude;
	Chassis_Attitude_Yaw.Current=Current_Pos->yaw;
	Chassis_Attitude_Yaw.Target=Command_Pos->yaw;
	Chassis_Attitude_Yaw.Adjust();
	_chassis_Velocity_Attitude.z_speed=std_lib::constrain((float)Chassis_Attitude_Yaw.Out,-1000.0f,1000.0f);
	return &_chassis_Velocity_Attitude;
}
float TurnBackController(float CurYaw,float TarYaw)
{
	TurnBack_Yaw.Current=CurYaw;
	TurnBack_Yaw.Target=TarYaw;
	TurnBack_Yaw.Adjust();
	return std_lib::constrain((float)TurnBack_Yaw.Out,-0.1f,0.1f);
}

_chassis_Velocity* PositionController(const _chassis_GlobalPos Current,const _chassis_GlobalPos Target)
{
	//static _chassis_Velocity chassis_
}

void NUC_Ctrl(void *arg)	
{
	USART_COB _buffer;
	int8_t longBuff[0x2a];
	int8_t shortBuff[0x1c];
	int8_t normalBuff[0x0e];
	static TickType_t _xTicksToWait = pdMS_TO_TICKS(1);
	/* Pre-Load for task */
	int overtime = 30;
	
	/* Infinite loop */
	TickType_t xLastWakeTime_t;
	xLastWakeTime_t = xTaskGetTickCount();
	for(;;)
	{
		vTaskDelayUntil(&xLastWakeTime_t,1);
		
		if (xQueueReceive(NUC_QueueHandle, &_buffer, _xTicksToWait) == pdTRUE)
		{
			is_RxNuc = 1;
			if(_buffer.len == 0x2a)  //42
			{
				memcpy(longBuff, _buffer.address, _buffer.len);
				memcpy(&NUCComRxData, longBuff, 14);//收12个 即为3*4个字节
			}
			else if(_buffer.len == 0x1c) //28
			{
				memcpy(shortBuff, _buffer.address, _buffer.len);
				memcpy(&NUCComRxData, shortBuff, 14);
			}
			else if(_buffer.len == 0x0e) //14
			{
				memcpy(normalBuff, _buffer.address, _buffer.len);
				memcpy(&NUCComRxData, normalBuff, 14);
			}
		}
		else
		{
			overtime--;
			if(overtime <= 0)
			{
				overtime = 30;
				is_RxNuc = 0;
			}
		}
	}
}

void Gamepad_Ctrl(void*arg)
{  
	/*Gamepad Mode*/
  static TickType_t _xTicksToWait = pdMS_TO_TICKS(1);
	static TickType_t _xPreviousWakeTime = xTaskGetTickCount();
  static TickType_t _xTimeIncrement = pdMS_TO_TICKS(1);
  for (;;)
  {
	  //拨杆双中---DR16遥控
		if (DR16.GetS1()== DR16_SW_MID && DR16.GetS2()== DR16_SW_MID)
		{
			if (TigerArm.Get_Current_Mode()==TigerArm.DrivingMode)
			{
				TargetVelocity_X=-DR16.Get_LX_Norm();
				TargetVelocity_Y=-DR16.Get_LY_Norm();
				TargetVelocity_Z=DR16.Get_RX_Norm()*0.7f;
			}
			else
			{
				TargetVelocity_X=DR16.Get_LX_Norm();
				TargetVelocity_Y=DR16.Get_LY_Norm();
				TargetVelocity_Z=-DR16.Get_RX_Norm()*0.7f;
			}
		}
		else//NUC控制模式
		{
			if(is_RxNuc)
			{
				TargetVelocity_X =  NUCComRxData.ROS_Target_X/_x_vel_div;
				TargetVelocity_Y = -NUCComRxData.ROS_Target_Y/_y_vel_div;
				TargetVelocity_Z = -NUCComRxData.ROS_Target_W/_div;//+-注意
				if(NUCComRxData.flag == 0)
				{
					TargetVelocity_X = 0;
					TargetVelocity_Y = 0;
					TargetVelocity_Z = 0;//+-注意
				}
			}
			else
			{
				TargetVelocity_X =  0;
				TargetVelocity_Y =  0;
				TargetVelocity_Z =  0;//+-注意
			}
		}
			vTaskDelayUntil(&_xPreviousWakeTime, _xTimeIncrement);
  }
}



void Chassis_Ctrl(void *arg)
{
	static TickType_t _xPreviousWakeTime = xTaskGetTickCount();
	static TickType_t _xTimeIncrement = 20;
	
	for (;;)
	{
		Engineer_chassis.Set_Target(TargetVelocity_X,TargetVelocity_Y,-TargetVelocity_Z);				
		Engineer_chassis.Chassis_Control();
		Chassis_MotorMsg_Send();
		vTaskDelayUntil(&_xPreviousWakeTime, 1);
	}
}

void Keyboard_Ctrl(void*arg)
{
	/*Keyboard Mode*/
	static TickType_t _xTicksToWait = pdMS_TO_TICKS(1);
	static TickType_t _xPreviousWakeTime = xTaskGetTickCount();
  static TickType_t _xTimeIncrement = pdMS_TO_TICKS(1);
	
	static float RecordYaw;
	static float TarYaw;
	static bool CameraisForward = false;
	static CEngineer::Engineer_Mode_Typedef DirMode;
	 for (;;)
  {
		vTaskDelayUntil(&_xPreviousWakeTime, _xTimeIncrement);
		if((DR16.GetS1()==DR16_SW_DOWN && DR16.GetS2()==DR16_SW_DOWN))
		{
			static float Speed_Fast_Gear=1.5;
			static float Speed_Gear=1.2;
			static float Speed_Slow_Gear=0.1;
			static float Mouse_Sensitivity_Gear=0.2;

			if(DR16.IsKeyPress(DR16_KEY_Q)){
			TigerArm.Switch_Mode(CEngineer::DrivingMode);
			__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_2,1550);
			}
			if(DR16.IsKeyPress(DR16_KEY_C)||DR16.IsKeyPress(DR16_KEY_Z)||DR16.IsKeyPress(DR16_KEY_E)){
			TigerArm.Switch_Mode(CEngineer::BackMode);
			__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_2,580);
			}

			DirMode = TigerArm.Get_Current_Mode() == CEngineer::DrivingMode?CEngineer::DrivingMode:CEngineer::BackMode;
			if(DirMode==TigerArm.DrivingMode)
			{
				DR16.IsKeyPress(DR16_KEY_S) ? (TargetVelocity_Y=-Speed_Gear):(TargetVelocity_Y=0);                  /*后*/
				DR16.IsKeyPress(DR16_KEY_W) ? (TargetVelocity_Y=Speed_Gear):(TargetVelocity_Y=TargetVelocity_Y);    /*前*/
				DR16.IsKeyPress(DR16_KEY_A) ? (TargetVelocity_X=-Speed_Gear):(TargetVelocity_X=0);                  /*左*/
				DR16.IsKeyPress(DR16_KEY_D) ? (TargetVelocity_X=Speed_Gear):(TargetVelocity_X=TargetVelocity_X);    /*右*/	

				(DR16.IsKeyPress(DR16_KEY_S)&&DR16.IsKeyPress(DR16_KEY_SHIFT)) ? (TargetVelocity_Y=-Speed_Slow_Gear):(TargetVelocity_Y=TargetVelocity_Y);                  /*后*/
				(DR16.IsKeyPress(DR16_KEY_W)&&DR16.IsKeyPress(DR16_KEY_SHIFT)) ? (TargetVelocity_Y=Speed_Slow_Gear):(TargetVelocity_Y=TargetVelocity_Y);    /*前*/
				(DR16.IsKeyPress(DR16_KEY_A)&&DR16.IsKeyPress(DR16_KEY_SHIFT)) ? (TargetVelocity_X=-Speed_Slow_Gear):(TargetVelocity_X=TargetVelocity_X);                  /*左*/
				(DR16.IsKeyPress(DR16_KEY_D)&&DR16.IsKeyPress(DR16_KEY_SHIFT)) ? (TargetVelocity_X=Speed_Slow_Gear):(TargetVelocity_X=TargetVelocity_X);    /*右*/	

				(DR16.IsKeyPress(DR16_KEY_S)&&DR16.IsKeyPress(DR16_KEY_CTRL)) ? (TargetVelocity_Y=-Speed_Fast_Gear):(TargetVelocity_Y=TargetVelocity_Y);                  /*后*/
				(DR16.IsKeyPress(DR16_KEY_W)&&DR16.IsKeyPress(DR16_KEY_CTRL)) ? (TargetVelocity_Y=Speed_Fast_Gear):(TargetVelocity_Y=TargetVelocity_Y);    /*前*/
				(DR16.IsKeyPress(DR16_KEY_A)&&DR16.IsKeyPress(DR16_KEY_CTRL)) ? (TargetVelocity_X=-Speed_Fast_Gear):(TargetVelocity_X=TargetVelocity_X);                  /*左*/
				(DR16.IsKeyPress(DR16_KEY_D)&&DR16.IsKeyPress(DR16_KEY_CTRL)) ? (TargetVelocity_X=Speed_Fast_Gear):(TargetVelocity_X=TargetVelocity_X);    /*右*/	

			}else if(TigerArm.Get_Current_Mode()==TigerArm.BackMode||TigerArm.TaskingMode||DirMode==TigerArm.GoldenMineral||DirMode==TigerArm.SilverMineral||DirMode==TigerArm.ExchangeMode||DirMode==TigerArm.Rescure||DirMode==TigerArm.Obstacles)
			{

				DR16.IsKeyPress(DR16_KEY_S) ? (TargetVelocity_Y=Speed_Gear):(TargetVelocity_Y=0);                  /*后*/
				DR16.IsKeyPress(DR16_KEY_W) ? (TargetVelocity_Y=-Speed_Gear):(TargetVelocity_Y=TargetVelocity_Y);    /*前*/
				DR16.IsKeyPress(DR16_KEY_A) ? (TargetVelocity_X=Speed_Gear):(TargetVelocity_X=0);                  /*左*/
				DR16.IsKeyPress(DR16_KEY_D) ? (TargetVelocity_X=-Speed_Gear):(TargetVelocity_X=TargetVelocity_X);    /*右*/	

				DR16.IsKeyPress(DR16_KEY_S)&&DR16.IsKeyPress(DR16_KEY_SHIFT) ? (TargetVelocity_Y=Speed_Slow_Gear):(TargetVelocity_Y=TargetVelocity_Y);                  /*后*/
				DR16.IsKeyPress(DR16_KEY_W)&&DR16.IsKeyPress(DR16_KEY_SHIFT) ? (TargetVelocity_Y=-Speed_Slow_Gear):(TargetVelocity_Y=TargetVelocity_Y);    /*前*/
				DR16.IsKeyPress(DR16_KEY_A)&&DR16.IsKeyPress(DR16_KEY_SHIFT) ? (TargetVelocity_X=Speed_Slow_Gear):(TargetVelocity_X=TargetVelocity_X);                  /*左*/
				DR16.IsKeyPress(DR16_KEY_D)&&DR16.IsKeyPress(DR16_KEY_SHIFT) ? (TargetVelocity_X=-Speed_Slow_Gear):(TargetVelocity_X=TargetVelocity_X);    /*右*/	

				DR16.IsKeyPress(DR16_KEY_S)&&DR16.IsKeyPress(DR16_KEY_CTRL) ? (TargetVelocity_Y=Speed_Fast_Gear):(TargetVelocity_Y=TargetVelocity_Y);                  /*后*/
				DR16.IsKeyPress(DR16_KEY_W)&&DR16.IsKeyPress(DR16_KEY_CTRL) ? (TargetVelocity_Y=-Speed_Fast_Gear):(TargetVelocity_Y=TargetVelocity_Y);    /*前*/
				DR16.IsKeyPress(DR16_KEY_A)&&DR16.IsKeyPress(DR16_KEY_CTRL) ? (TargetVelocity_X=Speed_Fast_Gear):(TargetVelocity_X=TargetVelocity_X);                  /*左*/
				DR16.IsKeyPress(DR16_KEY_D)&&DR16.IsKeyPress(DR16_KEY_CTRL) ? (TargetVelocity_X=-Speed_Fast_Gear):(TargetVelocity_X=TargetVelocity_X);    /*右*/	

			}

			TargetVelocity_Z=2.5f*DR16.Get_MouseX_Norm()*Mouse_Sensitivity_Gear;                        /*旋转*/

			if(DR16.IsKeyPress(DR16_KEY_Z) && DR16.IsKeyPress(DR16_KEY_CTRL))  /*相机翻转*/
			{
				CameraisForward = CameraisForward?false:true;
				__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_2,(CameraisForward?580:1550));//1500
				
			}
		//if(DR16.IsKeyPress(_Z)&&!DR16.IsKeyPress(_CTRL)) {Z_KeyFlag=1;}

		}
    
		
	}
}
