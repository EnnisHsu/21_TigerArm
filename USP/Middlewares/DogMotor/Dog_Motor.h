/**
  ******************************************************************************
  * Copyright (c) 2019 - ~, SCUT-RobotLab Development Team
  * @file    Motor.h
  * @author  lindongdong  1027221389@qq.com 
  * @brief   Motro for RC_Dog
  *			     Currently motor type only supports haitai motor AK80-01-03
  * @date    2021-02-21
  * @attention
  * @note 
      
	AK80_V3ʹ�÷�����
	
      -1. ʵ����AK80_V3��             �磺AK80_V3 Motor_1(0x01,&hcan1);
      -2. ���ú�������������ģʽ    �磺Motor_1.To_Into_Control();
      -3. ���õ�����(ѡ��)          �磺Motor_1.Set_ZeroPosition();
      -4. ��CAN�ж��������ݽ���       �磺Motor_1.Update(CAN_RxMessage->data);
			-5. Ť�ء��ٶȡ�λ�ÿ��Ƶ��    �磺Out_Torque_Control(float torque);
		                                      Out_Speed_Control  (float speed, float kd );
		                                      Out_Position_Control(float position ,float kp);
		                                      Out_Mixed_Control (float position ,float kp ,float kd);
																				 
			ע������IDĬ��Ϊ0x00�����IDĬ��Ϊ0x01�����ID������λ���޸ġ�
			
			
  AK80_V1ʹ�÷�����
			 
      -1. ʵ����AK80_V1��             �磺AK80_V3 Motor_2(0x01,&hcan1);
      -2. ���ú�������λ�á��ٶ�ģʽ  �磺Motor_2.To_Speed_Control();
		                                      Motor_2.To_Position_Control();
      -3. ��������                    �磺Motor_2.Save();
      -4. ��CAN�ж��������ݽ���       �磺Motor_2.Update(CAN_RxMessage->data);
			-5. �ٶȡ�λ�ÿ��Ƶ��          �磺Motor_2.Out_Speed(int16_t speed);
		                                      Motor_2.Out_Position(int16_t acceleration,int16_t speed,int16_t position);
																					
			ע������IDĬ��Ϊ0xff�����ID����CAN�������ָ���޸ģ��޸ĺ������ϵ����Ч�������֪�����ID��������0x00��¼�޸�ID
			
  ******************************************************************************
  */
#ifndef _MOTOR_H_
#define _MOTOR_H_

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "drv_can.h"

#ifdef __cplusplus

/* Private define-------------------------------------------------------------*/

#define LIMIT_MIN_MAX(x,min,max) (x) = (((x)<=(min))?(min):(((x)>=(max))?(max):(x)))
#define ABS(x) ((x)>=0?(x):-(x))
/* Private Enum---------------------------------------------------------------*/

enum Control_State {
	EXIT_CONTROL =0,     /*�˳��������ģʽ*/
	INTO_CONTROL =1,     /*���Ԥ����ģʽ  */
	CURRENT_CONTROL=2,   /*����������      */ 
	SPEED_CONTROL =3,    /*�ٶȵ�������(Kd)*/
	POSITION_CONTROL =4, /*λ�õ�������(Kd)*/
	MIXED_CONTROL =5,    /*˫������(Kp��Kd)*/
	WARNING=6            /*����δ����������ģʽ*/
};



/**
  @brief class for motors AK80_V3.
*/
class AK80_V3
{
public:
    AK80_V3(uint16_t id ,CAN_HandleTypeDef* hcan) {ID=id; Can=hcan;}
    ~AK80_V3(){}
		CAN_HandleTypeDef* Can;
    uint16_t ID = 0;
		Control_State Current_mode = EXIT_CONTROL;
	  float Command_Position = 0.0f;
		float Command_Speed = 0.0f;
		float Command_Torque = 0.0f;
	  float Command_Kp =0.0f;
		float Command_Kd =0.0f;
			
		void Update(uint8_t can_rx_data[]);
		void Set_ZeroPosition();
		void To_Exit_Control();
		void To_Into_Control();
			
		float get_Current_Torque(){return this->Current_Torque;}
		float get_Current_Pos(){return this->Current_Position;}
		float get_Current_Angle(){return this->Current_Angle;}
		float get_Current_Speed(){return this->Current_Speed;}
		float get_Command_Pos() {return this->Command_Position;}

		
		void Out_Torque_Control(float torque);
		void Out_Speed_Control  (float speed, float kd );
		void Out_Position_Control(float position ,float kp);
		void Out_Mixed_Control (float position ,float speed,float kp ,float kd);
			
private:
	  float Current_Position = 0.0f;  //(rad) 
		float Current_Speed = 0.0f;     //(rad/s)
		float Current_Torque= 0.0f;     //(Nm)

    float Current_Angle = 0.0f;     //(�� )
    float Current_Rpm = 0.0f;       //(Ȧ/s)

    float POSITION_MAX() const { return 95.5f ;}
		float POSITION_MIN() const { return -95.5f;}
		float SPEED_MAX()    const { return 30.0f ;}
		float SPEED_MIN()    const { return -30.0f;}
		float KP_MAX()       const { return 500.0f;}
		float KP_MIN()       const { return 0.0f  ;}
		float KD_MAX()       const { return 5.0f  ;}
	  float KD_MIN()       const { return 0.0f  ;}
		float TORQUE_MAX()   const { return 18.0f ;}
		float TORQUE_MIN()   const { return -18.0f;}
		
		void Out (float current,float speed,float position,float kp ,float kd);
		
		float uint_to_float(int x_int, float x_min, float x_max, int bits);
		uint16_t float_to_uint(float x, float x_min, float xmax, uint8_t bits);
		
};


class AK80_V1
{
public:
    AK80_V1(uint16_t id ,CAN_HandleTypeDef* hcan) {Slave_Address=id; Can=hcan;}
    ~AK80_V1(){}
		CAN_HandleTypeDef* Can;
		uint8_t Host_Address =0xff;//����ID
		uint8_t Slave_Address=0x00;//�ӻ�ID 
		Control_State Current_State =INTO_CONTROL;
		
		int16_t Command_Acceleration =0;//(rpm/s)
		int16_t Command_Speed =0;       //(rpm)
		uint16_t Command_Position =0;   //(0~36000)
		uint16_t Command_Angle=0;       //(0~360��)
		
		void Update(uint8_t can_rx_data[]);
		void Modify_Slave_Address(uint8_t slave_address);
		void Read_Temperature();
    void To_Speed_Control();
		void To_Position_Control();
		void Out_Speed(int16_t speed);
		void Out_Position(int16_t acceleration,int16_t speed,int16_t position);
			
private:
	  int16_t  Current_Torque =0;  //(N��m)
    int16_t  Current_Speed =0;   //(rpm)
    uint16_t Current_Position=0; //(0~36000)
    uint16_t Current_Angle =0;   //(0~360��)
    uint8_t  Current_Temperature=0;//(0~100��)

    uint16_t POSITION_MAX() const { return 36000;}
		uint16_t POSITION_MIN() const { return 0    ;}
		int16_t SPEED_MAX()    const { return  256 ;}
		int16_t SPEED_MIN()    const { return -256 ;}
//		int16_t TORQUE_MAX()   const { return  ;} 
//		int16_t TORQUE_MIN()   const { return  ;}
};


#endif
#endif
/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
	
	
	
	
	
	
	
	
	
	
	
