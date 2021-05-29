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

#endif

void Service_MotoCtrl_Init();
void Task_ArmMotorInit(void *arg);
void Task_ArmMotorCtrl(void *arg);
void Task_ServoCtrl(void *arg);

class Asynchronous_Controller;
	
enum Curve_Type{
  LINEAR = 0,	//直线
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
  void resetStepTarget(float target, float current)
  {
    setTarget(target);
    setCurrent(current);
    last_time_stamp = xTaskGetTickCount();
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
		if (fabs(set_target-actual_current)>1e-5)
			return max_speed * (set_target - actual_current) / fabs(set_target - actual_current);
		else return 0.0f;
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

template <class motor>
class Godzilla_Joint_Controller{
public:
  Godzilla_Joint_Controller(int id, float speed) : joint_motor(id){
    this->async_controller.setSpeedConstrain(speed);
  }
  Godzilla_Joint_Controller(int id, CAN_HandleTypeDef* hcan, float speed) : joint_motor(id, hcan){
    this->async_controller.setSpeedConstrain(speed);
  }
  void init();
  void slowlyMoveToLimit();
  virtual float getCurrentAngle()
  {
    /* Please rewrite on your child class */
    return 0;
  }
  void setCurrentAsZero()
	{
		this->zero_offset=this->getCurrentAngle();
	}
  void setStepTarget(float target)
  {
    this->async_controller.resetStepTarget(target, this->getCurrentAngle());
  }
	float getZeroOffset(){return this->zero_offset;}
  void spinOnce();
  motor joint_motor;
  Asynchronous_Controller async_controller;
protected:
  float zero_offset;
};

class Godzilla_Servo_Controller{
	public:
		Godzilla_Servo_Controller(TIM_HandleTypeDef* HTIM,uint8_t TIM_CHANNEL)
		{
			this->htim=HTIM;
			this->tim_channel=TIM_CHANNEL;
		}
		
		void SetTargetAngle(float target)
		{
			this->o_target=target;
		}
		
		void Output()
		{
			__HAL_TIM_SetCompare(htim,tim_channel,deg2pwm(this->o_target));
		}
		
		void ResetOutputConfig(float c_min,float c_max,float c_offset)
		{
			this->o_max=c_max;
			this->o_min=c_min;
			this->zero_offset=c_offset;
		}
		
	private:
		uint16_t deg2pwm(float ang){return (uint16_t)((1000.0f/90.0f)*ang+1500.0f);}
		TIM_HandleTypeDef* htim;
		uint32_t tim_channel;
		float o_target;
		float o_max=2500.0f,o_min=500.0f,zero_offset=1500.0f;
};

class Godzilla_Yaw_Controller : public Godzilla_Joint_Controller<Motor_GM6020>
{
public:
  Godzilla_Yaw_Controller(uint8_t id, float speed) : Godzilla_Joint_Controller{id, speed}, joint_ctrl(&joint_motor){}
  void init(PID_Param_Typedef spd_pid_param, PID_Param_Typedef ang_pid_param)
  {
    this->async_controller.setSpeedConstrain(135.0f);
		this->setStepTarget(this->getCurrentAngle());
    this->joint_ctrl.SpeedPID.SetPIDParam(spd_pid_param.kp, spd_pid_param.ki, spd_pid_param.kd, \
                                          spd_pid_param.i_term_max, spd_pid_param.o_max);
    this->joint_ctrl.AnglePID.SetPIDParam(ang_pid_param.kp, ang_pid_param.ki, ang_pid_param.kd, \
                                          ang_pid_param.i_term_max, ang_pid_param.o_max);
  }
  void slowlyMoveToLimit()
  {
    float slowly_moving_target = this->joint_motor.getAngle();
    auto xLastSetTime = xTaskGetTickCount();
    while (xTaskGetTickCount() - xLastSetTime < 500)
    {
      /* slowly change target */
      if (fabs(slowly_moving_target - this->joint_motor.getAngle()) < 1.0f)
      {
        xLastSetTime = xTaskGetTickCount();
        slowly_moving_target -= 1.0f;
      }

      /* Adjust PID */
      joint_ctrl.setTarget(slowly_moving_target);
      joint_ctrl.Adjust();
			Motor_CAN_COB Motor_TxMsg;
      MotorMsgPack(Motor_TxMsg, this->joint_motor);
      xQueueSendFromISR(CAN2_TxPort, &Motor_TxMsg.Low, 0);
      vTaskDelay(1);
    }
  }
  float getCurrentAngle()
  {
    return this->joint_motor.getAngle();
  }
  void spinOnce()
  {
    this->async_controller.spinOnce(xTaskGetTickCount());
    this->joint_ctrl.setTarget(this->async_controller.getSteppingTarget());
    this->joint_ctrl.Adjust();
  }
private:
  MotorCascadeCtrl<myPID, myPID> joint_ctrl;
};

class Godzilla_Elbow_Controller : public Godzilla_Joint_Controller<AK80_V3>
{
	public:
		Godzilla_Elbow_Controller(int id, CAN_HandleTypeDef* hcan, float speed) : Godzilla_Joint_Controller(id, hcan, speed){}
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
		
	private:
		float mot_kp=30.0f,mot_kd=1.0f;
};

class Godzilla_Arm_Controller : public Godzilla_Joint_Controller<AK80_V3>
{
	public:
		Godzilla_Arm_Controller(int id, CAN_HandleTypeDef* hcan, float speed) : Godzilla_Joint_Controller(id, hcan, speed){}
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






extern Godzilla_Yaw_Controller yaw_controller;
extern Godzilla_Arm_Controller arm_controller;
extern Godzilla_Elbow_Controller elbow_controller;
extern Godzilla_Servo_Controller wristroll_controller,wristpitch_controller,wristyaw_controller;



extern TaskHandle_t ServiceMotoCtrl_Handle;

#endif /* APPLICATION_SERVICE_MOTOCTRL_H_ */
