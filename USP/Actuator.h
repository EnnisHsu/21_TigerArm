/**
  ******************************************************************************
  * Copyright (c) 2019 - ~, SCUT-RobotLab Development Team
  * @file    MotorCtrl.cpp
  * @author  EnnisKoh (8762322@qq.com) & Hehe
  * @brief   Code for resolve of armmodel like puma650
  * @date    2021-05-02
  * @version 0.4.1
  * @par Change Log:
  * <table>
  * <tr><th>Date        <th>Version  <th>Author    				 <th>Description
  * <tr><td>2021-05-01  <td> 0.4.1   <td>EnnisKoh&Hehe     <td>Creator
  * </table>
  *
	*/

/* Includes ------------------------------------------------------------------*/
#ifndef _Actuator_H_
#define _Actuator_H_


enum ACTUATOR_MODE_TYPE{
	PreCatching = 0U,
	Catching,
	Release
};
enum OPERATOR_TYPE{
	Cmd_Catch= 0U,
	Cmd_Release
};

enum SWITCH_MODE_TYPE{
	SwitchOff = 0U,
	SwitchOn
};

class Actuator_Classdef
{
	public:
		Actuator_Classdef();
		~Actuator_Classdef(){};
		
		int suckerAct();
	private:
		int turnMotor(SWITCH_MODE_TYPE);
		int airValve(SWITCH_MODE_TYPE);
		int getValve();
		int switchMode();
	
		ACTUATOR_MODE_TYPE Mode;
		OPERATOR_TYPE Cmd;
		float NegativePressure;
		const int NPThredshold;//thredshold of negative pressure

};

#endif




