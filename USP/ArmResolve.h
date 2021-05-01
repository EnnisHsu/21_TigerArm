#ifndef _ArmResolve_H_
#define _ArmResolve_H_

#include "Matrix.h"
#include <cmath> 

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

class MechanicalArm
{
	public:
		MechanicalArm();
		//MechnicalArm(int n);
		~MechanicalArm();

		int Init(Matrix Tw_c,Matrix T6_g);
		bool IKP_Input(double a[6],double d[6],double interval[6][2]);
		int cal();
		int solveT0_6();
		int getVision(Matrix Tc_g);
		theta_deg_pack match_solve();
	private:
		//int n_axis;
		DH_MODEL_Typedef dh_model[6];
		double theta[8][6];
		Matrix T0_6;
		Matrix Tc_g;//camera_goal
		Matrix Tw_c;//world_camera
		Matrix T6_g;//6_goal

};

#endif // !_ArmResolve_H_




