#ifndef _ArmResolve_H_
#define _ArmResolve_H_

#include "Matrix.h"

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

class MechnicalArm
{
	public:
		MechnicalArm();
		//MechnicalArm(int n);


		~MechnicalArm();

		bool IKP_Input(Matrix& Mat,double* a[6],double* d[6]);
		int cal();
		theta_deg_pack match_solve();
	private:
		//int n_axis;
		DH_MODEL_Typedef dh_model[6];
		double theta[8][6];
		Matrix T0_6;

};

#endif // !_ArmResolve_H_




