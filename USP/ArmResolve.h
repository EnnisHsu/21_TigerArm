#ifndef _ArmResolve_H_
#define _ArmResolve_H_

#include "Matrix.h"
#include <math.h> 
#include <stdint.h>
#define a(n) this->dh_model[n].a
#define d(n) this->dh_model[n].d
#define alpha(n) this->dh_model[n].alpha
#define theta(n) this->dh_model[n].theta

#define PI 3.14159265358979323846

struct DH_MODEL_Typedef
{
	double alpha, a, d, theta;
	double min_deg, max_deg;
};

struct theta_deg_pack
{
	double deg[6];
};

struct IP_data_pack
{
	double a[6];
};

enum GOAL_INPUT_METHOD_TYPE{
	World_Input,
	Vision_Input	
};

enum CREAVE_MODE_TYPE{
	Cubic_IP,
	Quintic_IP
};

class MechanicalArm
{
	public:
		MechanicalArm();
		//MechnicalArm(int n);
		~MechanicalArm();

		int Init(Matrix Tw_c,Matrix T6_g);
		bool Set_DHModel_Config(double a[6],double d[6],double interval[6][2]);
		bool IK_cal();
		int solveT0_6();
		int SetVision(Matrix Tc_g);
		int SetWorldGoal(Matrix Tw_g);
		theta_deg_pack get_IK_ans();
		theta_deg_pack get_curtarget_deg(uint32_t now_time);
		void Set_Cubic_IP_Config(theta_deg_pack* cur,uint32_t now_time);
		void update(theta_deg_pack* cur);
	private:
		//int n_axis;
		GOAL_INPUT_METHOD_TYPE goal_input_mode;
		CREAVE_MODE_TYPE creave_mode;
		DH_MODEL_Typedef dh_model[6];
		double theta[8][6];
		int tf=2000.0f;
		uint32_t last_IP_time;
		IP_data_pack joint_IP_data[6];
		theta_deg_pack target_deg,current_deg,curtarget_deg;
		Matrix T0_6;
		Matrix Tc_g;//camera_goal
		Matrix Tw_c;//world_camera
		Matrix T6_g;//6_goal
		Matrix Tw_g;//world_goal

};

#endif // !_ArmResolve_H_




