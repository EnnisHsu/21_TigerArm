/*
 * Service_MotoCtrl.h
 *
 *  Created on: 2021年5月19日
 *      Author: EnnisKoh
 */

#ifndef APPLICATION_SERVICE_MOTOCTRL_H_
#define APPLICATION_SERVICE_MOTOCTRL_H_
#define _USE_ASYNCHRONOUS_
#ifdef __cplusplus
#include "SRML.h"
#include "System_DataPool.h"

#endif
void Task_ArmMotorInitCtrl(void *arg);
void Service_MotoCtrl_Init();
void Task_ArmMotorInit(void *arg);
void Task_ArmMotorCtrl(void *arg);
void Task_ServoCtrl(void *arg);

class Asynchronous_Controller;
	
enum Curve_Typedef{
  LINEAR = 0,	//直线
	Cubic_Polynomial,//三次多项式
};

struct PID_Param_Typedef{
  float kp;
  float ki;
  float kd;
  float p_term_max;
  float i_term_max;
  float d_term_max;
  float o_max;
};

struct CubicPoly_Param_Typedef{
	float a0,a1,a2,a3;
	float tf;
};
		
class Asynchronous_Controller{
public:
  Asynchronous_Controller(){}
	~Asynchronous_Controller(){}
	int CubicPoly_Init(){
		//用三次曲线插
		//theta = a0+a1*theta+a2*theta^2+a3*theta^3
		CubicPoly_Config.a0 = this->actual_current;
		CubicPoly_Config.a1 = 0;
		CubicPoly_Config.a2 = (this->set_target - this->actual_current)*3/(CubicPoly_Config.tf*CubicPoly_Config.tf);
		CubicPoly_Config.a3 = (this->set_target - this->actual_current)*2/(CubicPoly_Config.tf*CubicPoly_Config.tf*CubicPoly_Config.tf);
		return 0;
	}
	void spinOnce(uint32_t time_stamp)
	{
		(this->Curve_Type==LINEAR)?(void)LinearSpin(time_stamp):(void)NULL;
		(this->Curve_Type==Cubic_Polynomial)?(void)CubicPolySpin(time_stamp):(void)NULL;
	}
  void resetStepTarget(float target, float current)
  {
    setTarget(target);
		this->actual_current=current;
    last_time_stamp = xTaskGetTickCount();
		set_time_stamp = xTaskGetTickCount();
		(this->Curve_Type==Cubic_Polynomial)?(void)CubicPoly_Init():(void)NULL;
  }
  void setSpeedConstrain(float val)	//设置最大速度 单位rad/s
  {
    max_speed = val;
  }
  void setAccerConstrain(float val)	//设置最大加速度 单位rad/s^2
  {
    max_accer = val;
  }
  void setCurveType(Curve_Typedef type);
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
	void setCurrent(float val)
  {
    actual_current = val;
    stepping_target = val;
  }
private:
	float LinearSpin(uint32_t time_stamp)
  {
    uint32_t interval_time = time_stamp - last_time_stamp;			//时间间隔
    float target_delta = getSpeedWithDirection() * float(interval_time) / 1000.0f;//计算目标值
    float stepping_target_temp;
    if (stepping_target>=set_target && stepping_target+target_delta<=set_target)
      stepping_target_temp=set_target;
    else if (stepping_target<=set_target && stepping_target+target_delta>=set_target)
      stepping_target_temp=set_target;
    else stepping_target_temp=stepping_target+target_delta;
    //stepping_target = fabs(stepping_target + target_delta) >= fabs(set_target) ? set_target : stepping_target + target_delta;
    stepping_target=stepping_target_temp;
    last_time_stamp = time_stamp;

    return stepping_target;
  }
	float CubicPolySpin(uint32_t time_stamp)
	{
		uint32_t interval_time = time_stamp - set_time_stamp;
		stepping_target = CubicPoly_Config.a0+CubicPoly_Config.a1*interval_time+CubicPoly_Config.a2*interval_time*interval_time+CubicPoly_Config.a3*interval_time*interval_time*interval_time;
		last_time_stamp = time_stamp;
		return stepping_target;
	}
	/**新加入*/
  float getSpeedWithDirection()
  {
		if (fabs(set_target-stepping_target)>1e-5)
			return max_speed * (set_target - stepping_target) / fabs(set_target - stepping_target);
		else return 0.0f;
  }
  void setTarget(float val)
  {
    set_target = val;
  }

  float actual_current;	//当前实际值
  float set_target;		//设置的目标值
  float stepping_target;	//计算的目标值
  float max_speed;		//速度限制
  float max_accer;		//加速度限制
	CubicPoly_Param_Typedef CubicPoly_Config;
	Curve_Typedef Curve_Type;
  uint32_t last_time_stamp;	//ms
	uint32_t set_time_stamp;
};


class Godzilla_Servo_Controller{
	public:
		enum Servo_Typedef
		{
			Servo180,
			Servo360
		};
		Godzilla_Servo_Controller(TIM_HandleTypeDef* HTIM,uint8_t TIM_CHANNEL,Servo_Typedef Servo_T)
		{
			this->Servo_Type=Servo_T;
			this->htim=HTIM;
			this->tim_channel=TIM_CHANNEL;
		}
		Godzilla_Servo_Controller(TIM_HandleTypeDef* HTIM,uint8_t TIM_CHANNEL,Servo_Typedef Servo_T,uint16_t zerop)
		{
			this->zero_offset=zerop;
			this->Servo_Type=Servo_T;
			this->htim=HTIM;
			this->tim_channel=TIM_CHANNEL;
		}
		
		void SetTargetAngle(float target)
		{
			this->o_target=target;
		}
		
		float GetCurrentAngle()
		{
			return this->o_target;
		}
		
		void Output()
		{
			if (this->Servo_Type==Servo360) __HAL_TIM_SetCompare(htim,tim_channel,deg2pwm360(this->o_target));
				else __HAL_TIM_SetCompare(htim,tim_channel,deg2pwm180(this->o_target));
		}
		
		void ResetOutputConfig(float c_min,float c_max,float c_offset)
		{
			this->o_max=c_max;
			this->o_min=c_min;
			this->zero_offset=c_offset;
		}
		
	private:
		uint16_t deg2pwm180(float ang){return (uint16_t)(std_lib::constrain((1000.0f/90.0f)*ang+this->zero_offset,o_min,o_max));}
		uint16_t deg2pwm360(float ang){return (uint16_t)(std_lib::constrain((1000.0f/180.0f)*ang+this->zero_offset,o_min,o_max));}
		TIM_HandleTypeDef* htim;
		uint32_t tim_channel;
		Servo_Typedef Servo_Type;
		float o_target;
		float o_max=2500.0f,o_min=500.0f,zero_offset=1500.0f;
};

template <class motor>
class Godzilla_Joint_Controller{
public:
  Godzilla_Joint_Controller(int id, float speed, float i_min,float i_max,float reduction) : joint_motor(id){
    this->async_controller.setSpeedConstrain(speed);
		this->o_min=i_min;
		this->o_max=i_max;
		this->reduction_ratio=reduction;
  }
  Godzilla_Joint_Controller(int id, CAN_HandleTypeDef* hcan, float speed, float i_min,float i_max,float reduction) : joint_motor(id, hcan){
    this->async_controller.setSpeedConstrain(speed);
		this->o_min=i_min;
		this->o_max=i_max;
		this->reduction_ratio=reduction;
  }
  void init();
  void slowlyMoveToLimit();
	float getReductionRatio() {return this->reduction_ratio;}
	float getZeroOffset(){return this->zero_offset;}
  virtual float getCurrentAngle()
  {
    /* Please rewrite on your child class */
    return 0;
  }
  void setCurrentAsZero()
	{
		this->zero_offset=this->getCurrentAngle();
	}
	void setCurrentAsTarget()
	{
		//float curAng=this->getCurrentAngle();
		this->async_controller.resetStepTarget(this->getCurrentAngle(),this->getCurrentAngle());
	}
  void setStepTarget(float target)
  {
		if (target>=this->zero_offset+this->o_min && target<=this->zero_offset+this->o_max)
		{
			this->async_controller.resetStepTarget(target, this->getCurrentAngle());
			this->current_target=target;
		}
		else return;
  }
  void spinOnce();
  motor joint_motor;
  Asynchronous_Controller async_controller;
protected:
  float zero_offset,o_min,o_max;
	float reduction_ratio,current_target;
};


class Godzilla_Yaw_Controller : public Godzilla_Joint_Controller<Motor_GM6020>
{
public:
  Godzilla_Yaw_Controller(uint8_t id, float speed, float i_min,float i_max,float reduction) : Godzilla_Joint_Controller{id, speed, i_min,i_max,reduction}, joint_ctrl(&joint_motor){}
  void init(PID_Param_Typedef spd_pid_param, PID_Param_Typedef ang_pid_param)
  {
    //this->async_controller.setSpeedConstrain(3.14f);
		this->setStepTarget(this->getCurrentAngle());
    this->joint_ctrl.SpeedPID.SetPIDParam(spd_pid_param.kp, spd_pid_param.ki, spd_pid_param.kd, \
                                          spd_pid_param.i_term_max, spd_pid_param.o_max);
    this->joint_ctrl.AnglePID.SetPIDParam(ang_pid_param.kp, ang_pid_param.ki, ang_pid_param.kd, \
                                          ang_pid_param.i_term_max, ang_pid_param.o_max);
  }
  void slowlyMoveToLimit()
  {
    float slowly_moving_target = this->getCurrentAngle();
    auto xLastSetTime = xTaskGetTickCount();
    while (xTaskGetTickCount() - xLastSetTime < 500)
    {
      /* slowly change target */
      if (fabs(slowly_moving_target - this->getCurrentAngle()) < 0.05f)
      {
        xLastSetTime = xTaskGetTickCount();
        slowly_moving_target -= 0.05f;
      }

      /* Adjust PID */
      joint_ctrl.setTarget(rad2deg(slowly_moving_target));
      joint_ctrl.Adjust();
			Motor_CAN_COB Motor_TxMsg;
      MotorMsgPack(Motor_TxMsg, this->joint_motor);
      xQueueSendFromISR(CAN2_TxPort, &Motor_TxMsg.Low, 0);
      vTaskDelay(1);
    }
  }
  float getCurrentAngle()
  {
    return deg2rad(this->joint_motor.getAngle());
  }
  void spinOnce()
  {
    this->async_controller.spinOnce(xTaskGetTickCount());
		
		#ifdef _USE_ASYNCHRONOUS_
			this->Yaw_MF << rad2deg(this->async_controller.getSteppingTarget());
			static float joint_target;
			this->Yaw_MF >> joint_target;
			this->joint_ctrl.setTarget(joint_target);
		#else
			this->Yaw_MF << rad2deg(this->current_target);
			static float joint_target;
			this->Yaw_MF >> joint_target;
			this->joint_ctrl.setTarget(joint_target);
		#endif
    this->joint_ctrl.Adjust();
  }
	MotorCascadeCtrl<myPID, myPID> joint_ctrl;
private:
	MeanFilter<50> Yaw_MF;
  
};

class Godzilla_Elbow_Controller : public Godzilla_Joint_Controller<AK80_V3>
{
	public:
		Godzilla_Elbow_Controller(int id, CAN_HandleTypeDef* hcan, float speed, float i_min,float i_max,float reduction) : Godzilla_Joint_Controller(id, hcan, speed, i_min,i_max,reduction){}
		void init()
		{
			this->joint_motor.To_Into_Control();
			//this->setStepTarget(this->getCurrentAngle());
			this->joint_motor.Out_Mixed_Control(this->getCurrentAngle(),10.0f,mot_kp,mot_kd);
		}			
		void slowlyMoveToLimit()
		{
			float slowly_moving_target = this->joint_motor.get_current_position();
			auto xLastSetTime = xTaskGetTickCount();
			while (xTaskGetTickCount() - xLastSetTime < 500)
			{
				/* slowly change target */
				if (fabs(slowly_moving_target - this->joint_motor.get_current_position()) < 0.05f)
				{
					xLastSetTime = xTaskGetTickCount();
					slowly_moving_target += 0.05f;
					//this->setStepTarget(slowly_moving_target);
				}

				/* Send Control to Motor */
				this->joint_motor.Out_Mixed_Control(slowly_moving_target,10.0f,mot_kp,mot_kd);
				vTaskDelay(1);
			}			
		}
		/*void ResetPIDparma(float kp,float kd)
		{
			mot_kp=kp;
			mot_kd=kd;
		}*/
		float getCurrentAngle()
		{
			return this->joint_motor.get_current_position();
		}
		void spinOnce()
		{
			this->async_controller.spinOnce(xTaskGetTickCount());
			//this->joint_motor.Out_Mixed_Control(this->async_controller.getSteppingTarget(),10.0f,mot_kp,mot_kd);
		}
	float Output;
		
	private:
		float mot_kp=30.0f,mot_kd=1.0f;
};

class Godzilla_Arm_Controller : public Godzilla_Joint_Controller<AK80_V3>
{
	public:
		Godzilla_Arm_Controller(int id, CAN_HandleTypeDef* hcan, float speed, float i_min,float i_max,float reduction) : Godzilla_Joint_Controller(id, hcan, speed, i_min,i_max,reduction){}
		void init(Godzilla_Elbow_Controller* elbow_controller)
		{
			this->joint_motor.To_Into_Control();
			//this->setStepTarget(this->getCurrentAngle());
			elbow=elbow_controller;
			this->joint_motor.Out_Mixed_Control(this->getCurrentAngle(),10.0f,mot_kp,mot_kd);
		}
		void slowlyMoveToLimit()
		{
			float slowly_moving_target = this->joint_motor.get_current_position();
			float elbow_moving_target = elbow->joint_motor.get_current_position();
			auto xLastSetTime = xTaskGetTickCount();
			while (xTaskGetTickCount() - xLastSetTime < 500)
			{
				/* slowly change target */
				if (fabs(slowly_moving_target - this->joint_motor.get_current_position()) < 0.05f)
				{
					xLastSetTime = xTaskGetTickCount();
					slowly_moving_target += 0.05f;
					elbow_moving_target -= 0.05f;
					//this->setStepTarget(slowly_moving_target);
				}

				/* Send Control to Motor */
				this->joint_motor.Out_Mixed_Control(slowly_moving_target,10.0f,mot_kp,mot_kd);
				elbow->joint_motor.Out_Mixed_Control(elbow_moving_target,10.0f,mot_kp,mot_kd);
				vTaskDelay(1);
			}			
		}
		/*void ResetPIDparma(float kp,float kd)
		{
			mot_kp=kp;
			mot_kd=kd;
		}*/
		float getCurrentAngle()
		{
			return this->joint_motor.get_current_position();
		}
		void spinOnce()
		{
			this->async_controller.spinOnce(xTaskGetTickCount());
			//this->joint_motor.Out_Mixed_Control(this->async_controller.getSteppingTarget(),10.0f,mot_kp,mot_kd);
		}
		
	private:
		float mot_kp=50.0f,mot_kd=1.0f;
		Godzilla_Elbow_Controller* elbow;
};


//Actuator
class Godzilla_Relay_Controller{
public:
	enum Relay_Status_Typedef
	{
		Relay_On=0,
		Relay_Off,
	};
	Godzilla_Relay_Controller(GPIO_TypeDef* IOBase,uint16_t IOPinBase)
	{
		this->GPIO_Base=IOBase;
		this->GPIO_PIN_Base=IOPinBase;
	}
	void SetRelayStatus(Relay_Status_Typedef Target_Status)
	{
		this->Relay_Status=Target_Status;
		HAL_GPIO_WritePin(this->GPIO_Base,this->GPIO_PIN_Base,(GPIO_PinState)this->Relay_Status);
	}
	Relay_Status_Typedef GetRelayStatus()
	{
		return this->Relay_Status;
	}
	
private:
	Relay_Status_Typedef Relay_Status;
	GPIO_TypeDef* GPIO_Base;
	uint16_t GPIO_PIN_Base;
};

class Godzilla_Pump_Controller : public Godzilla_Relay_Controller{
public:
	enum Negetive_Pressure_Typedef
	{
		Negetive,
		Positive,
	};
	Godzilla_Pump_Controller(GPIO_TypeDef* IOBase,uint16_t IOPinBase) : Godzilla_Relay_Controller(IOBase,IOPinBase){};
	void Update_PressureValve(Negetive_Pressure_Typedef Valve_Status)
	{
		this->negetive_pressure_signal=Valve_Status;
	}		
	Negetive_Pressure_Typedef Get_PressureValve()
	{
		return this->negetive_pressure_signal;
	}
private:
	Negetive_Pressure_Typedef negetive_pressure_signal;
};


extern Godzilla_Yaw_Controller yaw_controller;
extern Godzilla_Arm_Controller arm_controller;
extern Godzilla_Elbow_Controller elbow_controller;
extern Godzilla_Servo_Controller wristroll_controller,wristpitch_controller,wristyaw_controller;
extern Godzilla_Pump_Controller pump_controller;



extern TaskHandle_t ServiceMotoCtrl_Handle;

#endif /* APPLICATION_SERVICE_MOTOCTRL_H_ */
