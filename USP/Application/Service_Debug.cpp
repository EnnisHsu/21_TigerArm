/**
  **********************************************************************************
  * @file   : Service_Debug.cpp
  * @brief  : Debug support file.This file provides access ports to debug.
  **********************************************************************************
  *  
**/
/* Includes ------------------------------------------------------------------*/
#include "Service_Debug.h"
#include "SRML.h"
#include "UpperMonitor.h"

/* Private define ------------------------------------------------------------*/
TaskHandle_t Debug_Handle;
/* Private variables ---------------------------------------------------------*/
/* Private function declarations ---------------------------------------------*/
void Task_Debug(void *arg);

/* Function prototypes -------------------------------------------------------*/
/**
* @brief  Initialize debug service based on Upper Monitor.
*/
void Service_Debug_Init(void)
{
  
  xTaskCreate(Task_Debug,       /* Task function. */
         "Debug_Service",       /* Task Name. */
       Normal_Stack_Size,       /* Stack depth. */
                    NULL,       /* Task parameter */
     PriorityBelowNormal,       /* Priority */
          &Debug_Handle);       /* Task handle */
}


/**
* @brief  Send the debug meaasge to Upper Monitor(Lowest priority)
* @param  None.
* @return None.
*/
void Task_Debug(void *arg)
{
    /* Cache for Task */
		TickType_t xLastWakeTime_t;
		xLastWakeTime_t = xTaskGetTickCount();
		//asuwave_init(&huart1,xTaskGetTickCount);
    /* Pre-Load for task */
    for(;;)
    {
      /* User porcess BEGIN. */

      /* User process END. */
      
      /* Transmit a message frame. */
      Sent_Contorl(&huart1);
			//asuwave_subscribe();
      /*Pass to next ready task*/
      vTaskDelayUntil(&xLastWakeTime_t,10);
    }
}

/************************ COPYRIGHT SCUT-ROBOTLAB *****END OF FILE*************/
