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

myPID Chassis_Spd[4];
myPID Chassis_Pos[4];
myPID Chassis_Attitude_Yaw;

TaskHandle_t Robot_ChassisCtrl;
TaskHandle_t Robot_OfflineCtrl;
TaskHandle_t Robot_GamepadCtrl;
TaskHandle_t Robot_Keyboard_Ctrl;

void Service_RobotCtrl_Init()
{
	Engineer_Chassis_Init();
	xTaskCreate(Chassis_Ctrl, "Robot.ChassisCtrl", Normal_Stack_Size, NULL, PriorityNormal, &Robot_ChassisCtrl);
	//xTaskCreate(Offline_Ctrl, "Robot.OfflineCtrl", Normal_Stack_Size, NULL, PrioritySuperHigh, &Robot_OfflineCtrl);
	xTaskCreate(Gamepad_Ctrl, "Robot.GamepadCtrl", Normal_Stack_Size, NULL, PriorityNormal, &Robot_GamepadCtrl);
	xTaskCreate(Keyboard_Ctrl, "Robot.Keyboard_Ctrl", Normal_Stack_Size, NULL, PriorityNormal, &Robot_Keyboard_Ctrl);

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

_chassis_Velocity* PositionController(const _chassis_GlobalPos Current,const _chassis_GlobalPos Target)
{
	//static _chassis_Velocity chassis_
}


void Gamepad_Ctrl(void*arg)
{  
	/*Gamepad Mode*/
  static TickType_t _xTicksToWait = pdMS_TO_TICKS(1);
	static TickType_t _xPreviousWakeTime = xTaskGetTickCount();
  static TickType_t _xTimeIncrement = pdMS_TO_TICKS(1);
  for (;;)
  {
    //if (xTaskNotifyWait(0x00000000, 0xFFFFFFFF, NULL, _xTicksToWait) == pdTRUE)
    //{
		if (DR16.GetStatus()==DR16_ESTABLISHED)
		{
			if (TigerArm.Get_Current_Mode()==TigerArm.DrivingMode)
			{
				TargetVelocity_X=-DR16.Get_LX_Norm();
				TargetVelocity_Y=-DR16.Get_LY_Norm();
				TargetVelocity_Z=DR16.Get_RX_Norm()*0.7f;
			}else
			{
				TargetVelocity_X=DR16.Get_LX_Norm();
				TargetVelocity_Y=DR16.Get_LY_Norm();
				TargetVelocity_Z=-DR16.Get_RX_Norm()*0.7f;
			}
			if(DR16.GetS2() == DR16_SW_MID){
				__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_2,580);//forward
			}else if(DR16.GetS2() == DR16_SW_DOWN){
				__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_2,1550);//screen
			}
		}
			vTaskDelayUntil(&_xPreviousWakeTime, _xTimeIncrement);
    //}
  }
}



void Chassis_Ctrl(void *arg)
{
  static TickType_t _xPreviousWakeTime = xTaskGetTickCount();
  static TickType_t _xTimeIncrement = pdMS_TO_TICKS(1);
	
	for (;;)
  {
		Engineer_chassis.Set_Target(TargetVelocity_X,TargetVelocity_Y,-TargetVelocity_Z);				
		Engineer_chassis.Chassis_Control();
		Chassis_MotorMsg_Send();
    vTaskDelayUntil(&_xPreviousWakeTime, _xTimeIncrement);
  }
}



void Keyboard_Ctrl(void*arg)
{
	/*Keyboard Mode*/
	static TickType_t _xTicksToWait = pdMS_TO_TICKS(1);
	static TickType_t _xPreviousWakeTime = xTaskGetTickCount();
  static TickType_t _xTimeIncrement = pdMS_TO_TICKS(1);
	
	static bool toggle;
	static CEngineer::Engineer_Mode_Typedef LastDirMode;
	static CEngineer::Engineer_Mode_Typedef DirMode;
	 for (;;)
  {
		    if (xTaskNotifyWait(0x00000000, 0xFFFFFFFF, NULL, _xTicksToWait) == pdTRUE)
    {
			static float Speed_Gear=0.5;
			static float Mouse_Sensitivity_Gear=0.5;
			static uint8_t F_KeyFlag=0;
			static uint8_t G_KeyFlag=0;
			static uint8_t Z_KeyFlag=0;
			static uint8_t F_Ctrl_KeyFlag=0;
			static uint8_t G_Ctrl_KeyFlag=0;
			
			DirMode = TigerArm.Get_Current_Mode() == CEngineer::DrivingMode?CEngineer::DrivingMode:CEngineer::BackMode;
			if(TigerArm.Get_Current_Mode()==TigerArm.DrivingMode)
			{
				DR16.IsKeyPress(DR16_KEY_S) ? (TargetVelocity_Y=-Speed_Gear):(TargetVelocity_Y=0);                  /*后*/
				DR16.IsKeyPress(DR16_KEY_W) ? (TargetVelocity_Y=Speed_Gear):(TargetVelocity_Y=TargetVelocity_Y);    /*前*/
				DR16.IsKeyPress(DR16_KEY_A) ? (TargetVelocity_X=-Speed_Gear):(TargetVelocity_X=0);                  /*左*/
				DR16.IsKeyPress(DR16_KEY_D) ? (TargetVelocity_X=Speed_Gear):(TargetVelocity_X=TargetVelocity_X);    /*右*/	
			}else
			{
				DR16.IsKeyPress(DR16_KEY_S) ? (TargetVelocity_Y=Speed_Gear):(TargetVelocity_Y=0);                  /*后*/
				DR16.IsKeyPress(DR16_KEY_W) ? (TargetVelocity_Y=-Speed_Gear):(TargetVelocity_Y=TargetVelocity_Y);    /*前*/
				DR16.IsKeyPress(DR16_KEY_A) ? (TargetVelocity_X=Speed_Gear):(TargetVelocity_X=0);                  /*左*/
				DR16.IsKeyPress(DR16_KEY_D) ? (TargetVelocity_X=-Speed_Gear):(TargetVelocity_X=TargetVelocity_X);    /*右*/	
			}
			
			if(LastDirMode != DirMode){
				__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_2,(TigerArm.Get_Current_Mode()==TigerArm.DrivingMode?580:1550));
				toggle = ~toggle;
			}
			
			TargetVelocity_Z=2.5f*DR16.Get_MouseX_Norm()*Mouse_Sensitivity_Gear;                        /*旋转*/
			//DR16.IsKeyPress(_V) ? (TargetVelocity_Z=TargetVelocity_Z-Mouse_Sensitivity_Gear):(TargetVelocity_Z=TargetVelocity_Z); /*按键控制逆时针旋转*/
			//DR16.IsKeyPress(_B) ? (TargetVelocity_Z=TargetVelocity_Z+Mouse_Sensitivity_Gear):(TargetVelocity_Z=TargetVelocity_Z); /*按键控制顺时针旋转*/
      TargetVelocity_Z=std_lib::constrain(TargetVelocity_Z,1.0f,-1.0f);
			
			
			if(DR16.IsKeyPress(DR16_KEY_Z) && DR16.IsKeyPress(DR16_KEY_CTRL))  /*相机翻转*/
				{
					__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_2,(toggle?580:1550));//1500
					toggle = ~toggle;
				}
		  //if(DR16.IsKeyPress(_Z)&&!DR16.IsKeyPress(_CTRL)) {Z_KeyFlag=1;}
			LastDirMode = (TigerArm.Get_Current_Mode() == CEngineer::DrivingMode?CEngineer::DrivingMode:CEngineer::BackMode);
			
    }
		vTaskDelayUntil(&_xPreviousWakeTime, _xTimeIncrement);
	}
}





