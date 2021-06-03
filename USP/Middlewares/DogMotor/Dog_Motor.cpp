/**
  ******************************************************************************
  * Copyright (c) 2019 - ~, SCUT-RobotLab Development Team
  * @file    Motor.h
  * @author  lindongdong  1027221389@qq.com 
  * @brief   Motro for RC_Dog
  *			     Currently motor type only supports haitai motor AK80-01-03
  * @date    2021-02-21
  * @attention ʹ�÷����ο�Motor.h
*/
	
/* Includes ------------------------------------------------------------------*/
#include "Dog_Motor.h"

/* function prototypes -------------------------------------------------------*/

/**
* @brief  ���µ����ǰλ�á��ٶȡ�����ֵ
* @param  can_rx_data
*/
void AK80_V3::Update(uint8_t can_rx_data[])
{
//	int ID_temp = can_rx_data[0];
	int position_int = can_rx_data[1]<<8|can_rx_data[2];
	int speed_int = (can_rx_data[3]<<4)|(can_rx_data[4]>>4);
	int torque_int =((can_rx_data[4]&0xF)<<8)|can_rx_data[5];
	
	Current_Position = uint_to_float(position_int , POSITION_MIN(), POSITION_MAX(),16);
	Current_Speed    = uint_to_float(speed_int    , SPEED_MIN()   , SPEED_MAX()   ,12);
	Current_Torque   = uint_to_float(torque_int   , TORQUE_MIN()  , TORQUE_MAX()  ,12);
	
  Current_Angle = Current_Position*57.29578f;
	Current_Rpm   = Current_Speed/6.284f;
}


/**
* @brief  ���õ����ǰλ��Ϊ���(�����������ѽ������ģʽʱ)
* @param  NON
*/
void AK80_V3::Set_ZeroPosition()
{
	Command_Position = 0;
	Command_Speed   = 0;
	Command_Torque = 0;
  Command_Kp = 0;
  Command_Kd = 0;
	uint8_t buf[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE};
	CANx_SendData(Can, ID, buf, sizeof(buf));
}
	

/**
* @brief  ������Ԥ����ģʽ
* @param  NON
*/
void AK80_V3::To_Into_Control()
{
	Current_mode=INTO_CONTROL;
	Command_Position = 0;
	Command_Speed   = 0;
	Command_Torque = 0;
  Command_Kp = 0;
  Command_Kd = 0;
	uint8_t buf[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
	CANx_SendData(Can, ID, buf, sizeof(buf));
	HAL_Delay(100);
	Out ( Command_Torque, Command_Speed, Command_Position, Command_Kp , Command_Kd);
	HAL_Delay(100);
}

	
	
/**
* @brief  �˳��������ģʽ
* @param  NON
*/
void AK80_V3::To_Exit_Control()
{
	Current_mode=EXIT_CONTROL;
	Command_Position = 0;
	Command_Speed   = 0;
	Command_Torque = 0;
  Command_Kp = 0;
  Command_Kd = 0;
	uint8_t buf[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};
	CANx_SendData(Can, ID, buf, sizeof(buf));
}


/**
* @brief  ������ٶȡ�λ�á��������
* @param  current   �������
* @param  speed     ����ٶ�
* @param  position  ���λ��
* @param  kp        λ�û�Kpֵ
* @param  kd        �ٶȻ�Kdֵ
*/
void AK80_V3::Out (float torque,float speed,float position,float kp ,float kd)
{
	/*��ֵ*/
	Command_Position = position;
	Command_Speed   = speed;
	Command_Torque = torque;
  Command_Kp = kp;
  Command_Kd = kd;
	
	/* ��������Ĳ����ڶ���ķ�Χ�� */
	LIMIT_MIN_MAX(Command_Position, POSITION_MIN(), POSITION_MAX());
	LIMIT_MIN_MAX(Command_Speed,  SPEED_MIN(),  SPEED_MAX());
	LIMIT_MIN_MAX(Command_Kp,  KP_MIN(), KP_MAX());
	LIMIT_MIN_MAX(Command_Kd, KD_MIN(), KD_MAX());
	LIMIT_MIN_MAX(Command_Torque, TORQUE_MIN() , TORQUE_MAX());
	
	/* ����Э�飬��float��������ת�� */
	uint16_t P, V, KP, KD, C;
  uint8_t buf[8];
	P  = float_to_uint(Command_Position, POSITION_MIN(),  POSITION_MAX(), 16);            
	V  = float_to_uint(Command_Speed   , SPEED_MIN()   ,  SPEED_MAX()   , 12);
	KP = float_to_uint(Command_Kp      , KP_MIN()      ,  KP_MAX()      , 12);
	KD = float_to_uint(Command_Kd      , KD_MIN()      ,  KD_MAX()      , 12);
	C  = float_to_uint(Command_Torque , TORQUE_MIN()  ,  TORQUE_MAX()   , 12);
	
	/* ���ݴ���Э�飬������ת��ΪCAN���������ֶ� */
	buf[0] = P>>8;
	buf[1] = P&0xFF;
	buf[2] = V>>4;
	buf[3] = ((V&0xF)<<4)|(KP>>8);
	buf[4] = KP&0xFF;
	buf[5] = KD>>4;
	buf[6] = ((KD&0xF)<<4)|(C>>8);
	buf[7] = C&0xff;
	
	CANx_SendData(Can, ID, buf, sizeof(buf));
}
	

/**
* @brief  �����������
* @param  current
*/
void AK80_V3::Out_Torque_Control(float torque)
{
	if(Current_mode!=EXIT_CONTROL)
	{
		Current_mode=CURRENT_CONTROL;
		float position=0;
		float speed =0  ;
		float kp=0;
		float kd=0;
		Out ( torque, speed, position, kp , kd);
	}
	else{Current_mode=WARNING;}/*����δ����������ģʽ*/

}

/**
* @brief  ���ٶȻ����
* @param  speed
* @param  kd
*/
void AK80_V3::Out_Speed_Control  (float speed, float kd )
{
	if(Current_mode!=EXIT_CONTROL)
	{
		Current_mode=SPEED_CONTROL;
		float position=0;
		float torque =0;
		float kp=0;
		Out ( torque, speed, position, kp , kd);
	}
	else{Current_mode=WARNING;}/*����δ����������ģʽ*/
}


/**
* @brief  ��λ�û����
* @param  position
* @param  kp
*/
void AK80_V3::Out_Position_Control(float position ,float kp)
{
	if(Current_mode!=EXIT_CONTROL)
	{
		Current_mode=POSITION_CONTROL;
		float speed =0;
		float torque =0;
		float kd=0;
		Out ( torque, speed, position, kp , kd);
	}
	else{Current_mode=WARNING;}/*����δ����������ģʽ*/
}


/** 
* @brief  ˫�����
* @param  position
* @param  speed
* @param  kp
* @param  kd
*/
void AK80_V3::Out_Mixed_Control (float position ,float speed ,float kp ,float kd)
{
	if(Current_mode!=EXIT_CONTROL)
	{
		Current_mode=MIXED_CONTROL;
		float speed =speed;
		float torque =0;
		Out ( torque, speed, position, kp , kd);
	}
	else{Current_mode=WARNING;}/*����δ����������ģʽ*/
}
	



/**
* @brief  ����Э���floatת��Ϊuint16_t
* @param  x      ��ת����ֵ
* @param  x_min  ���ֵ
* @param  x_max  ��Сֵ
* @param  bits   ת�����λ��
*/
uint16_t AK80_V3::float_to_uint(float x, float x_min, float x_max, uint8_t bits)
{
	float span = x_max - x_min;
	float offset = x_min;
	
	return (uint16_t) ((x-offset)*((float)((1<<bits)-1))/span);
}


/**
* @brief  ����Э���uint16_tת��Ϊfloat
* @param  x      ��ת����ֵ
* @param  x_min  ���ֵ
* @param  x_max  ��Сֵ
* @param  bits   ת�����λ��
*/
float  AK80_V3::uint_to_float(int x_int, float x_min, float x_max, int bits)
{
	float span = x_max - x_min;
	float offset = x_min;
	return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}


/*-----------------------------------------------------------------------------------------------------*/

/**
* @brief  ���µ����ǰλ�á��ٶȡ�Ť��ֵ
* @param  NON
*/
void AK80_V1::Update(uint8_t can_rx_data[])
{
//	uint8_t slave_id=0x00;       //�ӻ�ID
	uint8_t function_code =0x00; //������
	uint8_t sign_bit =0;         //���ط���λ
//	uint8_t register_address =0; //�Ĵ�����ַ
//  uint8_t data_length =0;      //���ݳ���
//	uint16_t data =0;            //����
//  slave_id =can_rx_data[0];  
	function_code =can_rx_data[1];
	switch (function_code)
  {
		/*��д�Ӵ�*/
  	case 0x01:
		  sign_bit =can_rx_data[2];
		  sign_bit ?(Current_Torque=-can_rx_data[3]):(Current_Torque=can_rx_data[3]);
		  Current_Speed = can_rx_data[5]<<8|can_rx_data[4];
		  Current_Position = can_rx_data[7]<<8|can_rx_data[6];
  		break;
		/*����Ӵ�*/
  	case 0x02:
		  sign_bit =can_rx_data[2];
		  sign_bit ?(Current_Torque=-can_rx_data[3]):(Current_Torque=can_rx_data[3]);
		  Current_Temperature = can_rx_data[4];
		  Current_Position = can_rx_data[7]<<8|can_rx_data[6];
  		break;
		/*�Ĵ���д��*/
  	case 0x06:
//			register_address =can_rx_data[2];
//		  data_length =can_rx_data[3];
//		  data = can_rx_data[7]<<8|can_rx_data[6];
		/*Ӧ�������Ϊ�������͵����ݣ��˴����ݽ���ֻ������Debug������������*/
  		break;
  	default:
  		break;
  }
}


/**
* @brief  �޸Ĵӻ�ID(���ID)
* @param  slave_address Ŀ���ַ
*/
void AK80_V1::Modify_Slave_Address(uint8_t slave_address)
{
	uint8_t buf_1[6] = {Host_Address, 0x06, 0x15, 0x01, slave_address, 0x00};
	uint8_t buf_2[6] = {Host_Address, 0x06, 0x14, 0x01, 0x01, 0x00};
	CANx_SendData(Can, Slave_Address, buf_1, sizeof(buf_1));
	HAL_Delay(100);	
	CANx_SendData(Can, Slave_Address, buf_2, sizeof(buf_2));
}


/**
* @brief  ��ȡ�������(�¶ȡ�Ť�ء�λ��)
* @param  NON
*/
void AK80_V1::Read_Temperature()
{
	uint8_t buf[8] = {Host_Address, 0x02, 0x00, 0x00, 0x00, 0x00,0x00,0x00};
	CANx_SendData(Can, Slave_Address, buf, sizeof(buf));
}



/**
* @brief  �л�Ϊ�ٶȿ���ģʽ������
* @param  NON
*/
void AK80_V1::To_Speed_Control()
{
	Current_State=SPEED_CONTROL;
	uint8_t buf_1[6] = {Host_Address, 0x06, 0x19, 0x01, 0x03, 0x00};
	uint8_t buf_2[6] = {Host_Address, 0x06, 0x14, 0x01, 0x01, 0x00};
	CANx_SendData(Can, Slave_Address, buf_1, sizeof(buf_1));
	HAL_Delay(100);
	CANx_SendData(Can, Slave_Address, buf_2, sizeof(buf_2));
}

/**
* @brief  �л�Ϊλ�ÿ���ģʽ������
* @param  NON
*/
void AK80_V1::To_Position_Control()
{
	Current_State=POSITION_CONTROL;
	uint8_t buf_1[6] = {Host_Address, 0x06, 0x19, 0x01, 0x00, 0x00};
	uint8_t buf_2[6] = {Host_Address, 0x06, 0x14, 0x01, 0x01, 0x00};
	CANx_SendData(Can, Slave_Address, buf_1, sizeof(buf_1));
	HAL_Delay(100);
	CANx_SendData(Can, Slave_Address, buf_2, sizeof(buf_2));
}




/**
* @brief  ָ������ٶ�ת��
* @param  speed ����ٶ�Ŀ��ֵ
*/
void AK80_V1::Out_Speed(int16_t speed)
{
	Command_Speed=speed;
	speed=6*speed; //���Լ��ٱ�
	uint16_t speed_uint=0;
	
	if(speed>=0)
	{speed_uint=speed;}
	else
  {speed_uint=0x10000+speed;}
	
	uint8_t buf[6];
	buf[0] = Host_Address;
	buf[1] = 0x06;
	buf[2] = 0x02;
	buf[3] = 0x01;
	buf[4] = speed_uint&0xFF;
	buf[5] = speed_uint>>8;
	
  CANx_SendData(Can, Slave_Address, buf, sizeof(buf));
}


/**
* @brief  ָ�����ת����ָ��λ��
* @param  acceleration  ���ٶ�  
* @param  speed         �ٶ�  0~256
* @param  position      λ��  0~36000
*/
void AK80_V1::Out_Position(int16_t acceleration,int16_t speed,int16_t position)
{
	Command_Position=position;
	Command_Speed =speed;
	Command_Acceleration =acceleration;
	
	uint8_t speed_uint= ABS(speed);
	uint8_t acceleration_uint=ABS(acceleration);
	uint8_t buf[8];
	buf[0] = Host_Address;
	buf[1] = 0x01;
	buf[2] = 0x00;
	buf[3] = acceleration_uint;
	buf[4] = 0x00;
	buf[5] = speed_uint;
	buf[6] = position&0xFF;
	buf[7] = position>>8;
	
	CANx_SendData(Can, Slave_Address, buf, sizeof(buf));
}


/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/


