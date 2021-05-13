#ifndef _MotorCtrl_H_
#define _MotorCtrl_H_

#ifdef __cplusplus
	#include "SRML.h"
	#include "Dog_Motor.h"
	#include "ArmResolve.h"
	extern "C"
{
#endif
 
#include "System_DataPool.h"
	
void Service_MotoCtrl_Init();
static void Convert_Data(CAN_RxMessage* input, CAN_COB* output);
void CAN1_RxCpltCallback(CAN_RxBuffer *CAN_RxMessage);
void Task_DogMotorCtrl(void);
void Task_CAN1Receive(void);
void TigerArm_Init(void);
void Task_JointCtrl(void* arg);
void Task_TigerArmUpdate(void* arg);
void TigerArm_move();
void Task_TigerArmCtrl(void* arg);

#ifdef __cplusplus
}
#endif
#endif
