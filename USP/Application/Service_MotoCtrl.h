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
	class Asynchronous_Controller;
		
	extern Motor_GM6020 Tigerarm_Yaw;
	extern MotorCascadeCtrl<myPID,myPID> Tigerarm_Yaw_Ctrl;
	extern AK80_V3 Tigerarm_Shoulder,Tigerarm_Elbow;

	extern float Shoulder_target_pos,Elbow_target_pos;
	extern float Yaw_target_pos;//		deg
	
	extern Asynchronous_Controller yaw_async_controller;
	extern Asynchronous_Controller shoulder_async_controller;	//2nd joint
	extern Asynchronous_Controller elbow_async_controller;		//3rd joint
		
	extern TaskHandle_t ServiceMotoCtrl_Handle;
		
	extern float Shoulder_Zero_Offset,Yaw_Zero_Offset,Elbow_Zero_Offset;
	
	enum Curve_Type{
		LINEAR = 0,	//直线	
	};
		
	class Asynchronous_Controller{
	public:
		Asynchronous_Controller(){}
		float spinOnce(uint32_t time_stamp)
		{
			uint32_t interval_time = time_stamp - last_time_stamp;			//时间间隔
			float target_delta = getSpeedWithDirection() * float(interval_time) / 1000.0f;//计算目标值
			float stepping_target_temp;
			if (fabs(stepping_target)>=fabs(set_target) && fabs(stepping_target+target_delta)<=fabs(set_target))
				stepping_target_temp=set_target;
			else if (fabs(stepping_target)<=fabs(set_target) && fabs(stepping_target+target_delta)>=fabs(set_target))
				stepping_target_temp=set_target;
			else stepping_target_temp=stepping_target+target_delta;
			//stepping_target = fabs(stepping_target + target_delta) >= fabs(set_target) ? set_target : stepping_target + target_delta;
			stepping_target=stepping_target_temp;
			last_time_stamp = time_stamp;
			
			return stepping_target;
		}
		void resetStepTarget(float target, float current, uint32_t time_stamp)
		{
			setTarget(target);
			setCurrent(current);
			last_time_stamp = time_stamp;
		}
		void setSpeedConstrain(float val)	//设置最大速度 单位rad/s
		{
			max_speed = val;
		}
		void setAccerConstrain(float val)	//设置最大加速度 单位rad/s^2
		{
			max_accer = val;
		}
		void setCurveType(Curve_Type type);	
		float getSteppingTarget()
		{
			return stepping_target;
		}
		float getTarget()
		{
			return set_target;
		}
		float getActCur()
		{
			return actual_current;
		}
	private:
		float getSpeedWithDirection()
		{
			return max_speed * (set_target - actual_current) / fabs(set_target - actual_current);
		}
		void setTarget(float val)
		{
			set_target = val;
		}
		void setCurrent(float val)
		{
			actual_current = val;
			stepping_target = val;
		}
		float actual_current;	//当前实际值
		float set_target;		//设置的目标值
		float stepping_target;	//计算的目标值
		float max_speed;		//速度限制
		float max_accer;		//加速度限制
		uint32_t last_time_stamp;	//ms
	};

	void Task_LinearTargetCtrl(void *arg);
	void Service_MotoCtrl_Init();
		void ArmMotorInit();
	void Task_ArmMotoCtrl(void *arg);

#ifdef __cplusplus
	}
#endif



#endif /* APPLICATION_SERVICE_MOTOCTRL_H_ */
