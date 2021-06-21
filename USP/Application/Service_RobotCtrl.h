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
class CEngineer
{
 public:
  enum Engineer_Mode_Typedef
  {
     TigerarmNone = 0xee,
     DrivingMode = 0xd0,   //forward
     TaskingMode =0xd1,  //screen
     GoldenMineral =0xd2,
     SilverMineral =0xd3,
     ExchangeMode = 0xd4,
     Rescure =0xd5,
     Obstacles =0xd6,
			BackMode = 0xff,
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
  Engineer_Mode_Typedef Engineer_Mode=this->DrivingMode;
  
 
};

extern CEngineer TigerArm;
	
extern CChassis Engineer_chassis;
	
extern float TargetVelocity_X,TargetVelocity_Y,TargetVelocity_Z;

void Service_RobotCtrl_Init();
void Controller_PID_ParamTnit();
void Engineer_Chassis_Init();
int* SpeedController(const int16_t* current,const int16_t* target);
_chassis_Velocity* AttitudeController(const _chassis_GlobalPos* Current_Pos,const _chassis_GlobalPos* Command_Pos);
_chassis_Velocity* PositionController(const _chassis_GlobalPos Current,const _chassis_GlobalPos Target);

void Offline_Ctrl(void*arg);
void Gamepad_Ctrl(void*arg);
void Camera_Ctrl(void*arg);
void Chassis_Ctrl(void *arg);
void Keyboard_Ctrl(void *arg);


#ifdef __cplusplus
}
#endif

#endif
