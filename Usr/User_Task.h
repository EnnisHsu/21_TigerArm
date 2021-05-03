/**
******************************************************************************
* @file   User_Task.h
* @brief  Header file of User Tasks.
******************************************************************************
* @note
*  - Before running your task, just do what you want ~
*  - More devices or using other classification is decided by yourself ~
*/
#pragma once
/* Includes ------------------------------------------------------------------*/
#include "System_DataPool.h"
/* Private macros ------------------------------------------------------------*/
/* Private type --------------------------------------------------------------*/
//shoulder_yaw	->shouder_pitch	->elbow	->wrist_roll->wrist_pitch	->wrist_yaw
//joint0		->joint1		->joint2->joint3	->joint4		->joint5
enum _Joint_Type
{
	Shoulder_yaw = 0U,
	Shouder_pitch,
	Elbow,
	Wrist_roll,
	Wrist_pitch,
	Wrist_yaw
};

enum _Wheel_Pos
{
  _LF = 0U,
  _RF,
  _RB,
  _LB
};
/* Exported macros -----------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
extern TaskHandle_t RobotCtrl_Handle;
extern TaskHandle_t DataDisplay_Handle;
/* Exported function declarations --------------------------------------------*/
void User_Tasks_Init(void);
void Task_DataDisplay(void *arg);

/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/

