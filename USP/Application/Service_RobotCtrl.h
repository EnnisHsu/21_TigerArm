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
	
//#define _IngoreInit
	
extern float deg[6];
extern TaskHandle_t Robot_ROSCtrl;
	
class CEngineer
{
	public:
		enum Engineer_Mode_Typedef
		{
					TigerarmNone = 0xee,
					DrivingMode = 0xd0,			
					TaskingMode =0xd1,		
					GoldenMineral =0xd2,
					SilverMineral =0xd3,
					ExchangeMode = 0xd4,
					Rescure =0xd5,
					Obstacles =0xd6,
		};
		enum Engineer_ComStatus_Typedef
		{
			Engineer_CommandWait=0x01,
			Engineer_CommandLock,
		};
		void Switch_Mode(Engineer_Mode_Typedef Target_Mode)
		{
			this->Engineer_Mode=Target_Mode;
		}
		Engineer_Mode_Typedef Get_Current_Mode()
		{
			return this->Engineer_Mode;
		}
		void Switch_CommandStatus(Engineer_ComStatus_Typedef command_status)
		{
			this->Engineer_ComStatus=command_status;
		}
		Engineer_ComStatus_Typedef Get_Current_CommandStatus()
		{
			return this->Engineer_ComStatus;
		}
	private:
		Engineer_Mode_Typedef Engineer_Mode=this->TigerarmNone;
		Engineer_ComStatus_Typedef Engineer_ComStatus=this->Engineer_CommandWait;
	
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
