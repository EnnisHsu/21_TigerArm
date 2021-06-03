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

void Service_RobotCtrl_Init();
void Task_ArmSingleCtrl(void *arg);
void Task_DR16Ctrl(void *arg);
void Task_ROSCtrl(void *arg);
void Task_KeyboardCtrl(void *arg);

#ifdef __cplusplus
}
#endif

#endif
