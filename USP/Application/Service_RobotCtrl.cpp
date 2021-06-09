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


void Service_RobotCtrl_Init()
{
	//xTaskCreate(Task_ArmSingleCtrl, "Robot.ArmSingleCtrl", Tiny_Stack_Size, NULL, PriorityNormal, &Robot_ArmSingleCtrl);
	//xTaskCreate(Task_DR16Ctrl, "Robot.DR16Ctrl", Normal_Stack_Size, NULL, PrioritySuperHigh, &Robot_DR16Ctrl);
//	xTaskCreate(Task_ROSCtrl, "Robot.ROSCtrl", Normal_Stack_Size, NULL, PrioritySuperHigh, &Robot_ROSCtrl);
}




