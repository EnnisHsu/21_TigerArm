/**
  ******************************************************************************
  * Copyright (c) 2021 - ~, SCUT-RobotLab Development Team
  * @file    xxx.h
  * @author  EnnisKoh  8762322@qq.com 
  * @brief   
  * @date    2021-00-00
  * @attention
  * Copyright (c) 2021 - ~, SCUT-RobotLab Development Team 
*/
#ifndef APPLICATION_SERVICE_ROBOTCTRL_H_
#define APPLICATION_SERVICE_ROBOTCTRL_H_

#ifdef __cplusplus
	#include "SRML.h"
	#include "System_Datapool.h"
extern "C"
{
#endif
	
extern float deg[6];
extern TaskHandle_t Robot_ROSCtrl;
	
class CEngineer
{
	public:
		enum Engineer_Mode_Typedef
		{
					TigerarmNone = 0xee,
					ForwardChassis = 0xd0,			
					BackwardChassis =0xd1,		
					AutoCatch =0xd2,
					ManualCatch =0xd3,
					Rescure =0xd4,
					Auto_Obstacles =0xd5,
		};
		void Switch_Mode(Engineer_Mode_Typedef Target_Mode)
		{
			this->Engineer_Mode=Target_Mode;
		}
		Engineer_Mode_Typedef Get_Current_Mode()
		{
			return this->Engineer_Mode;
		}
	private:
		Engineer_Mode_Typedef Engineer_Mode=this->ForwardChassis;
		
	
};

extern CEngineer TigerArm;

void Service_RobotCtrl_Init();
void Task_ArmSingleCtrl(void *arg);
void Task_DR16Ctrl(void *arg);
void Task_ROSCtrl(void *arg);
void Task_KeyboardCtrl(void *arg);
 void Task_BoardCommunication(void *arg);

#ifdef __cplusplus
}
#endif

#endif
