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
 // xTaskCreate(Device_Actuators, "Dev.Actuator" , Tiny_Stack_Size,    NULL, PrioritySuperHigh,   &DeviceActuators_Handle);
  //xTaskCreate(Device_DR16,      "Dev.DR16"     , Normal_Stack_Size,    NULL, PriorityHigh,        &DeviceDR16_Handle);
  xTaskCreate(Device_Sensors,   "Dev.Sensors"  , Normal_Stack_Size,    NULL, PriorityHigh,        &DeviceSensors_Handle);
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
			static uint8_t msg[22];
			memcpy(msg,_buffer.address,22);
			static uint8_t dr16_msg[18];
			memcpy(dr16_msg,_buffer.address,18);
    	DR16.DataCapture((DR16_DataPack_Typedef*)dr16_msg);
			TigerArm.Switch_Mode((CEngineer::Engineer_Mode_Typedef)msg[18]);
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
    mpu_dmp_get_data(&MPUData.roll,&MPUData.pitch,&MPUData.yaw);
		if(TurnBack)
			MPUData.yaw+=180;
		MPU_Get_Accelerometer(&MPU6050_IIC_PIN,&MPUData.ax,&MPUData.ay,&MPUData.az);
		if (!MPU_Get_Gyroscope(&MPU6050_IIC_PIN,&MPUData.gx,&MPUData.gy,&MPUData.gz))
		{
			MPUData.gx-=MPUData.gxoffset;
			MPUData.gy-=MPUData.gyoffset;
			MPUData.gz-=MPUData.gzoffset;
		}
		Engineer_chassis.Update_CurrentAttitude(MPUData.roll,MPUData.pitch,MPUData.yaw);
    /* Exchange NUC Meaasge */
    
    /* Read Other board Message */

    
  /* Pass control to the next task ------------------------------------------*/
    vTaskDelayUntil(&xLastWakeTime_t,2);
  }
}


/* User Code End Here ---------------------------------*/

/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
