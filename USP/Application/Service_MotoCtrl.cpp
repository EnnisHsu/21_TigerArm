#include "Service_MotoCtrl.h"
#include "Service_RobotCtrl.h"


void Service_MotoCtrl_Init()
{
	//ArmMotorInit();
	
}


void Chassis_MotorMsg_Send()
{
	/*The feedback and output values of the right two wheels should be reversed*/
  Engineer_chassis.wheel_Out[1]= -Engineer_chassis.wheel_Out[1];
	Engineer_chassis.wheel_Out[2]= -Engineer_chassis.wheel_Out[2];
	
	uint8_t msg_send[8] = {0};	
	
	msg_send[0] = (unsigned char)((short)Engineer_chassis.wheel_Out[0] >> 8);//ตอ8ฮป
	msg_send[1] = (unsigned char)(short)Engineer_chassis.wheel_Out[0];
	msg_send[2] = (unsigned char)((short)Engineer_chassis.wheel_Out[1]>> 8);
	msg_send[3] = (unsigned char)(short)Engineer_chassis.wheel_Out[1];
	msg_send[4] = (unsigned char)((short)Engineer_chassis.wheel_Out[2] >> 8);
	msg_send[5] = (unsigned char)(short)Engineer_chassis.wheel_Out[2];
	msg_send[6] = (unsigned char)((short)Engineer_chassis.wheel_Out[3] >> 8);
	msg_send[7] = (unsigned char)(short)Engineer_chassis.wheel_Out[3];	
	
	CANx_SendData(&hcan1,0x200,msg_send,8);		
}