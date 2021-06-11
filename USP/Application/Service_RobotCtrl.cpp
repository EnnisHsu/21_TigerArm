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



void Service_RobotCtrl_Init()
{
	Engineer_Chassis_Init();
	xTaskCreate(Chassis_Ctrl, "Robot.ChassisCtrl", Tiny_Stack_Size, NULL, PriorityNormal, &Robot_ChassisCtrl);
	xTaskCreate(Offline_Ctrl, "Robot.OfflineCtrl", Tiny_Stack_Size, NULL, PrioritySuperHigh, &Robot_OfflineCtrl);
	xTaskCreate(Gamepad_Ctrl, "Robot.GamepadCtrl", Normal_Stack_Size, NULL, PrioritySuperHigh, &Robot_GamepadCtrl);
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
	Engineer_chassis.Set_AccelerationParam(12000,30000,2000);
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

void Offline_Ctrl(void*arg)
{
  static TickType_t _xTicksToWait = pdMS_TO_TICKS(1);
	static TickType_t _xPreviousWakeTime = xTaskGetTickCount();
  static TickType_t _xTimeIncrement = pdMS_TO_TICKS(1);
	
  for (;;)
  {
    if (xTaskNotifyWait(0x00000000, 0xFFFFFFFF, NULL, _xTicksToWait) == pdTRUE)
    {
			/*Gamepad Offline protection*/
      TargetVelocity_X =0;
			TargetVelocity_Y =0;
			TargetVelocity_Z =0;
			Engineer_chassis.Switch_Mode(Halt);
			
			
			vTaskDelayUntil(&_xPreviousWakeTime, _xTimeIncrement);
    }
  }
}

void Gamepad_Ctrl(void*arg)
{  
	/*Gamepad Mode*/
  static TickType_t _xTicksToWait = pdMS_TO_TICKS(1);
	static TickType_t _xPreviousWakeTime = xTaskGetTickCount();
  static TickType_t _xTimeIncrement = pdMS_TO_TICKS(1);
	extern _BoardComRx BoardComRxData;
  for (;;)
  {
    if (xTaskNotifyWait(0x00000000, 0xFFFFFFFF, NULL, _xTicksToWait) == pdTRUE)
    {
      TargetVelocity_X=BoardComRxData.TarSpdX?BoardComRxData.TarSpdX:-DR16.Get_LX_Norm();
			TargetVelocity_Y=BoardComRxData.TarSpdY?BoardComRxData.TarSpdY:-DR16.Get_LY_Norm();
			TargetVelocity_Z=BoardComRxData.TarSpdZ?BoardComRxData.TarSpdZ:DR16.Get_RX_Norm()*0.7f;
			
			vTaskDelayUntil(&_xPreviousWakeTime, _xTimeIncrement);
    }
  }
}

void Motor_SendESC()
{
	/*The feedback and output values of the right two wheels should be reversed*/
  Engineer_chassis.wheel_Out[1]= -Engineer_chassis.wheel_Out[1];
	Engineer_chassis.wheel_Out[2]= -Engineer_chassis.wheel_Out[2];
	
	uint8_t msg_send[8] = {0};	
	if(DR16.GetStatus() == DR16_ESTABLISHED)
	{
			msg_send[0] = (unsigned char)((short)Engineer_chassis.wheel_Out[0] >> 8);//低8位
			msg_send[1] = (unsigned char)(short)Engineer_chassis.wheel_Out[0];
			msg_send[2] = (unsigned char)((short)Engineer_chassis.wheel_Out[1]>> 8);
			msg_send[3] = (unsigned char)(short)Engineer_chassis.wheel_Out[1];
			msg_send[4] = (unsigned char)((short)Engineer_chassis.wheel_Out[2] >> 8);
			msg_send[5] = (unsigned char)(short)Engineer_chassis.wheel_Out[2];
			msg_send[6] = (unsigned char)((short)Engineer_chassis.wheel_Out[3] >> 8);
			msg_send[7] = (unsigned char)(short)Engineer_chassis.wheel_Out[3];	
	}
	else
	{
		msg_send[0] = 0;//低8位
		msg_send[1] = 0;
		msg_send[2] = 0;
		msg_send[3] = 0;
		msg_send[4] =	0;
		msg_send[5] = 0;
		msg_send[6] = 0;
		msg_send[7] = 0;			
	}
	
	CANx_SendData(&hcan1,0x200,msg_send,8);		
}

void Chassis_Ctrl(void *arg)
{
  static TickType_t _xPreviousWakeTime = xTaskGetTickCount();
  static TickType_t _xTimeIncrement = pdMS_TO_TICKS(1);
	
	for (;;)
  {
		Engineer_chassis.Set_Target(TargetVelocity_X,TargetVelocity_Y,-TargetVelocity_Z);				
		Engineer_chassis.Chassis_Control();
		Motor_SendESC();
    vTaskDelayUntil(&_xPreviousWakeTime, _xTimeIncrement);
  }
}

void Camera_Toggle()
{
	return;
}

void Keyboard_Ctrl(void*arg)
{
	/*Keyboard Mode*/
	static TickType_t _xTicksToWait = pdMS_TO_TICKS(1);
	static TickType_t _xPreviousWakeTime = xTaskGetTickCount();
  static TickType_t _xTimeIncrement = pdMS_TO_TICKS(1);
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

			DR16.IsKeyPress(DR16_KEY_S) ? (TargetVelocity_Y=-Speed_Gear):(TargetVelocity_Y=0);                  /*后*/
			DR16.IsKeyPress(DR16_KEY_W) ? (TargetVelocity_Y=Speed_Gear):(TargetVelocity_Y=TargetVelocity_Y);    /*前*/
      DR16.IsKeyPress(DR16_KEY_A) ? (TargetVelocity_X=-Speed_Gear):(TargetVelocity_X=0);                  /*左*/
			DR16.IsKeyPress(DR16_KEY_D) ? (TargetVelocity_X=Speed_Gear):(TargetVelocity_X=TargetVelocity_X);    /*右*/	
			
			TargetVelocity_Z=2.5f*DR16.Get_MouseX_Norm()*Mouse_Sensitivity_Gear;                        /*旋转*/
			//DR16.IsKeyPress(_V) ? (TargetVelocity_Z=TargetVelocity_Z-Mouse_Sensitivity_Gear):(TargetVelocity_Z=TargetVelocity_Z); /*按键控制逆时针旋转*/
			//DR16.IsKeyPress(_B) ? (TargetVelocity_Z=TargetVelocity_Z+Mouse_Sensitivity_Gear):(TargetVelocity_Z=TargetVelocity_Z); /*按键控制顺时针旋转*/
      TargetVelocity_Z=std_lib::constrain(TargetVelocity_Z,1.0f,-1.0f);
			
			
			if(DR16.IsKeyPress(DR16_KEY_Z) && DR16.IsKeyPress(DR16_KEY_CTRL))  {Camera_Toggle();Z_KeyFlag=0;}        /*相机翻转*/
			
		  //if(DR16.IsKeyPress(_Z)&&!DR16.IsKeyPress(_CTRL)) {Z_KeyFlag=1;}
			
			vTaskDelayUntil(&_xPreviousWakeTime, _xTimeIncrement);
    }
	}
}





