#ifndef _ArmResolve_H_
#define _ArmResolve_H_

//#include "Tool_Mat.h"
#include "Matrix.h"
#include "math.h"
#include "SRML/SRML.h"
#include "System_DataPool.h"
#include <stdint.h>
#define a(n) this->dh_model[n].a
#define d(n) this->dh_model[n].d
#define alpha(n) this->dh_model[n].alpha
#define theta(n) this->dh_model[n].theta
#define ENABLE 1
#define DISABLE 0

double rad2deg(double rad);
double deg2rad(double deg);

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

//template<class ArmMatrix>
class MechanicalArm
{
	public:
		MechanicalArm() {};
		MechanicalArm(int i):T0_6(4,4), Tc_g(4, 4),Tw_c(4, 4), T6_g(4, 4),Tw_g(4, 4){};
		//MechnicalArm(int n);
		~MechanicalArm() {};

		Matrix convert_DHModel_to_Matrix(DH_MODEL_Typedef& axis);
		bool FK_cal();
		int Init(Matrix Tw_c, Matrix T6_g);
		bool Set_DHModel_Config(double a[6],double d[6],double interval[6][2]);
		bool IK_cal();
		int solveT0_6();
		int SetVision(Matrix Tc_g);
		int SetWorldGoal(Matrix Tw_g);
		theta_deg_pack get_IK_ans();
		theta_deg_pack get_curtarget_deg(uint32_t now_time);
		bool ReachTargetDeg();
		void Set_Cubic_IP_Config(uint32_t now_time);
		void update(theta_deg_pack* cur);
		void SetTargetx(double x) { target_x = x; }
		void SetTargety(double y) { target_y = y; }
		void SetTargetz(double z) { target_z = z; }
		double GetWorldx() { return world_x; }
		double GetWorldy() { return world_y; }
		double GetWorldz() { return world_z; }
		double GetClawRoll() { return roll; }
		double GetClawPitch() { return pitch; }
		double GetClawYaw() { return yaw; }
		double GetTargetx() { return target_x; }
		double GetTargety() { return target_y; }
		double GetTargetz() { return target_z; }

		int NewTarget = DISABLE;
		double x_offset = 0.325f, y_offset = -0.425f, z_offset = 0.14f;
	private:
		//int n_axis;
		GOAL_INPUT_METHOD_TYPE goal_input_mode=World_Input;
		CREAVE_MODE_TYPE creave_mode=Cubic_IP ;
		int CP62g=DISABLE;
		
		DH_MODEL_Typedef dh_model[6];
		double theta[8][6];
		int tf=20.0f;
		uint32_t last_IP_time;
		IP_data_pack joint_IP_data[6];
		theta_deg_pack target_deg,current_deg,curtarget_deg;
		Matrix T0_6;
		//T0_6.Init(4, 4);
		Matrix Tc_g;//camera_goal
		Matrix Tw_c;//world_camera
		Matrix T6_g;//6_goal
		Matrix Tw_g;//world_goal
		double world_x, world_y, world_z, roll, pitch, yaw;
		double target_x, target_y, target_z, target_roll, target_pitch, target_yaw;
};

#endif // !_ArmResolve_H_




