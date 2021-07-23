#include "Service_RelayCtrl.h"

Godzilla_Pump_Controller pump_controller(GPIOC,GPIO_PIN_5);
Godzilla_Relay_Controller clamp_controller(GPIOB,GPIO_PIN_0),hook_controller(GPIOB,GPIO_PIN_1),
		card_controller(GPIOC,GPIO_PIN_4);
