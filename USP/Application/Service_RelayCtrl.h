#ifndef _RELAYCTRL_H_
#define _RELAYCTRL_H_

#ifdef __cplusplus
#include "SRML.h"
#include "System_DataPool.h"

#endif

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
		NoneObject,
		SomeObject,
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

#endif

extern Godzilla_Pump_Controller pump_controller;
extern Godzilla_Relay_Controller clamp_controller,hook_controller,card_controller;