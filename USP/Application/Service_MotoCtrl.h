/*
 * Service_MotoCtrl.h
 *
 *  Created on: 2021年5月19日
 *      Author: EnnisKoh
 */

#ifndef APPLICATION_SERVICE_MOTOCTRL_H_
#define APPLICATION_SERVICE_MOTOCTRL_H_

#ifdef __cplusplus
#include "SRML.h"
#include "System_DataPool.h"
	extern "C"
	{
#endif
	extern Motor_GM6020 Tigerarm_Yaw;

	extern AK80_V3 Tigerarm_Shoulder,Tigerarm_Elbow;

	extern float Shoulder_target_pos,Elbow_target_pos;
	extern float Yaw_target_pos;//		deg

	void Service_MotoCtrl_Init();
	void Task_ArmMotoCtrl(void *arg);

#ifdef __cplusplus
	}
#endif



#endif /* APPLICATION_SERVICE_MOTOCTRL_H_ */
