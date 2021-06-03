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

myPID Chassis_Spd[4];

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

void Engineer_Chassis_Init()
{
	Engineer_chassis.Set_AccelerationParam(12000,30000,2000);
	Engineer_chassis.Set_SpeedGear(FAST_GEAR);
	Engineer_chassis.Set_TorqueOptimizeFlag(1);
	Engineer_chassis.Set_AttitudeOptimizeFlag(1);
	Engineer_chassis.Set_SpeedParam(0.35f,0.5f,1.0f,1.0f);
	Engineer_chassis.Switch_Mode(Normal_Speed);
	
}


void Service_RobotCtrl_Init()
{
	//xTaskCreate(Task_ArmSingleCtrl, "Robot.ArmSingleCtrl", Tiny_Stack_Size, NULL, PriorityNormal, &Robot_ArmSingleCtrl);
	//xTaskCreate(Task_DR16Ctrl, "Robot.DR16Ctrl", Normal_Stack_Size, NULL, PrioritySuperHigh, &Robot_DR16Ctrl);
//	xTaskCreate(Task_ROSCtrl, "Robot.ROSCtrl", Normal_Stack_Size, NULL, PrioritySuperHigh, &Robot_ROSCtrl);
}




