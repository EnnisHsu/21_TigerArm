#ifndef _MotorCtrl_H_
#define _MotorCtrl_H_

#ifdef __cplusplus
	#include "SRML.h"
	#include "Dog_Motor.h"
	extern "C"
{
#endif

#include "System_DataPool.h"
	
extern AK80_V3 Motor_2,Motor_3;
	
void Service_MotoCtrl_Init();
static void Convert_Data(CAN_RxMessage* input, COB_TypeDef* output);
void CAN1_RxCpltCallback(CAN_RxBuffer *CAN_RxMessage);
void CAN2_RxCpltCallback(CAN_RxBuffer *CAN_RxMessage);
void Task_DogMotorCtrl(void);
void Task_CAN1Receive(void);


#ifdef __cplusplus
}
#endif
#endif
		