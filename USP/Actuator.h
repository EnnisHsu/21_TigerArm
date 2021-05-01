#ifndef _Actuator_H_
#define _Actuator_H_

#define theta(n) this->dh_model[n].theta

#define PI 3.14159265358979323846

enum ACTUATOR_MODE_TYPE{
	PreCatching = 0U,
	Catching,
	Release
};

class Actuator_Classdef
{
	public:
		Actuator_Classdef(){};
		~Actuator_Classdef(){};
		
		int suckerAct();
	private:
		int turnMotorOn();
		int turnMotorOff();
		int airValveOff();
		int airValveOn();
		
		ACTUATOR_MODE_TYE Mode;
		int NegativePressure;
		const int NPThredshold;//thredshold of negative pressure

};

#endif




