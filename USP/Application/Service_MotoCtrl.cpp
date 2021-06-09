#include "Service_MotoCtrl.h"
#include "Service_RobotCtrl.h"


void Service_MotoCtrl_Init()
{
	//ArmMotorInit();
	
}


void Chassis_MotorMsg_Send()
{
	uint8_t Chassis_MotorMsg[8] = {0};	
/*The feedback and output values of the right two wheels should be reversed*/
	Engineer_chassis.wheel_Out[1]= -Engineer_chassis.wheel_Out[1];
	Engineer_chassis.wheel_Out[2]= -Engineer_chassis.wheel_Out[2];

		
	Chassis_MotorMsg[0] = (unsigned char)((short)Engineer_chassis.wheel_Out[0] >> 8);//ตอ8ฮป
	Chassis_MotorMsg[1] = (unsigned char)(short)Engineer_chassis.wheel_Out[0];
	Chassis_MotorMsg[2] = (unsigned char)((short)Engineer_chassis.wheel_Out[1]>> 8);
	Chassis_MotorMsg[3] = (unsigned char)(short)Engineer_chassis.wheel_Out[1];
	Chassis_MotorMsg[4] = (unsigned char)((short)Engineer_chassis.wheel_Out[2] >> 8);
	Chassis_MotorMsg[5] = (unsigned char)(short)Engineer_chassis.wheel_Out[2];
	Chassis_MotorMsg[6] = (unsigned char)((short)Engineer_chassis.wheel_Out[3] >> 8);
	Chassis_MotorMsg[7] = (unsigned char)(short)Engineer_chassis.wheel_Out[3];	
	CANx_SendData(&hcan1,0x200,Chassis_MotorMsg,8);
}