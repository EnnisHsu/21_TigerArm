/**
  ******************************************************************************
  * @file   Service_Devices.cpp
  * @brief  Devices service running file.
  ******************************************************************************
  * @note
  *  - Before running your devices, just do what you want ~ !
  *  - More devices or using other classification is decided by yourself ~ !
  ===============================================================================
                                    Task List
  ===============================================================================
  * <table>
  * <tr><th>Task Name     <th>Priority          <th>Frequency/Hz    <th>Stack/Byte
  * <tr><td>              <td>                  <td>                <td>    
  * </table>
  *
 */
/* Includes ------------------------------------------------------------------*/
#include "Service_Devices.h"
#include "Service_RobotCtrl.h"

/* Private define ------------------------------------------------------------*/
TaskHandle_t DeviceActuators_Handle;
TaskHandle_t DeviceDR16_Handle;
TaskHandle_t DeviceSensors_Handle;

/* Private variables ---------------------------------------------------------*/
/* Private function declarations ---------------------------------------------*/
void Device_Actuators(void *arg);
void Device_Sensors(void *arg);
void Device_DR16(void *arg);


/* Exported devices ----------------------------------------------------------*/
/* Motor & ESC & Other actuators*/

/* Remote control */

/* IMU & NUC & Other sensors */

/* Other boards */

/* Function prototypes -------------------------------------------------------*/
/**
* @brief  Initialization of device management service
* @param  None.
* @return None.
*/
void Service_Devices_Init(void)
{
  //xTaskCreate(Device_Actuators, "Dev.Actuator" , Tiny_Stack_Size,    NULL, PrioritySuperHigh,   &DeviceActuators_Handle);
  xTaskCreate(Device_DR16,      "Dev.DR16"     , Normal_Stack_Size,    NULL, PriorityHigh,        &DeviceDR16_Handle);
  //xTaskCreate(Device_Sensors,   "Dev.Sensors"  , Tiny_Stack_Size,    NULL, PriorityHigh,        &DeviceSensors_Handle);
}



void Device_Actuators(void *arg)
{
  /* Cache for Task */

  
  /* Pre-Load for task */

  
  /* Infinite loop */
  TickType_t xLastWakeTime_t;
  xLastWakeTime_t = xTaskGetTickCount();
  for(;;)
  {
    /* Connection check */

    /* Read Message */

    

    /* Send Message */
    

    /* Pass control to the next task */
    vTaskDelayUntil(&xLastWakeTime_t,1);
  }
}

void Device_DR16(void *arg)
{
  /* Cache for Task */
	USART_COB _buffer;
  static uint8_t Tx_Msg[22];	
	static TickType_t _xTicksToWait = pdMS_TO_TICKS(1);
  /* Pre-Load for task */

  /* Infinite loop */
  TickType_t xLastWakeTime_t;
  xLastWakeTime_t = xTaskGetTickCount();

  for(;;)
  {
    
		
    if (xQueueReceive(DR16_QueueHandle, &_buffer, _xTicksToWait) == pdTRUE)
    {
			DR16.Check_Link(xTaskGetTickCount());
    	DR16.DataCapture((DR16_DataPack_Typedef*)_buffer.address);
			memcpy(Tx_Msg,_buffer.address,18);
			Tx_Msg[18]=TigerArm.Get_Current_Mode();
			Tx_Msg[19]=0;
			Tx_Msg[20]=0;
			Tx_Msg[21]=0;
			error_flag=43;
			USART_COB Usart_RxCOB;
			Usart_RxCOB.port_num = 6;
			Usart_RxCOB.len=22;
			Usart_RxCOB.address=Tx_Msg;
			HAL_UART_Transmit_DMA(&huart6,Tx_Msg,22);;
			//_BoardComRx Tx_Msg_Pack;
			//memcpy(&Tx_Msg_Pack,Tx_Msg,22);
			//xQueueSendFromISR(USART_TxPort,&Usart_RxCOB,0);
    }
    /* Pass control to the next task */
    vTaskDelayUntil(&xLastWakeTime_t,1);
  }
}

void Device_Sensors(void *arg)
{
  /* Cache for Task */

  /* Pre-Load for task */

  /* Infinite loop */
  TickType_t xLastWakeTime_t;
  xLastWakeTime_t = xTaskGetTickCount();
  for(;;)
  {
    /* Read IMU Message */
    
    /* Exchange NUC Meaasge */
    
    /* Exchange Other board Message */
		
    
  /* Pass control to the next task ------------------------------------------*/
    vTaskDelayUntil(&xLastWakeTime_t,2);
  }
}

/* User Code End Here ---------------------------------*/

/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
