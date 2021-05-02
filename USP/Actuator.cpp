/**
  ******************************************************************************
  * Copyright (c) 2019 - ~, SCUT-RobotLab Development Team
  * @file    Actuator.cpp
  * @author  EnnisKoh (8762322@qq.com) & Hehe(853856200@qq.com) 
  * @brief   Code for Actuator of armmodel like sucker
  * @date    2021-05-01
  * @version 0.4.1
  * @par Change Log:
  * <table>
  * <tr><th>Date        <th>Version  <th>Author    				 <th>Description
  * <tr><td>2021-05-01  <td> 0.4.1   <td>EnnisKoh&Hehe     <td>Creator
  * </table>
  */



/* Includes ------------------------------------------------------------------*/ 
#include "Actuator.h"
#include "stm32f4xx_hal.h"
/* function prototypes -------------------------------------------------------*/
/**
* @brief  actiong of actuator
* @param  None
* @retval 1 if success
*/
int Actuator_Classdef::suckerAct(){
	//if negative pressure
	//>threshold:slirp
	//<=threshold:stop
	switch(Mode)
	{
		PreCatching:
		{
			turnMotor(SwitchOn);
			airValve(SwitchOn);
			break;
		}
		Catching:
		{
			turnMotor(SwitchOff);
			airValve(SwitchOff);
			break;
		}
		Release:
		{
			turnMotor(SwitchOff);
			airValve(SwitchOn);
			break;
		}
	default:return 0;break;
	}
	getValve();
	switchMode();
	return 1;
}
/**
* @brief  
* @param  
* @retval 
*/
int Actuator_Classdef::turnMotor(SWITCH_MODE_TYPE mode){
	//mode off gpio write pin 0
	//mode on gpio write pin 1
	return 1;
}
/**
* @brief  
* @param  
* @retval 
*/
int Actuator_Classdef::airValve(SWITCH_MODE_TYPE mode){
	//mode off gpio write pin 0
	//mode on gpio write pin 1
	return 1;
}

/**
* @brief  
* @param  
* @retval 
*/
int Actuator_Classdef::getValve(){
	//get negative pressure
	//NegativePressure = 
	return 1;
}
/**
* @brief  
* @param  
* @retval 
*/
int Actuator_Classdef::switchMode(){
	if(NegativePressure<NPThredshold)Mode = Catching;
	else if(Cmd == Cmd_Catch)Mode = PreCatching;
	else if(Cmd == Cmd_Release)Mode = Release;
	else {return 0;}
	return 1;
}
