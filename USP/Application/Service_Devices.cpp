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
Motor_AK80_9  Test_Motor(1, 0, 0);
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
  xTaskCreate(Device_Actuators, "Dev.Actuator" , Tiny_Stack_Size,    NULL, PrioritySuperHigh,   &DeviceActuators_Handle);
  xTaskCreate(Device_DR16,      "Dev.DR16"     , Tiny_Stack_Size,    NULL, PriorityHigh,        &DeviceDR16_Handle);
  xTaskCreate(Device_Sensors,   "Dev.Sensors"  , Tiny_Stack_Size,    NULL, PriorityHigh,        &DeviceSensors_Handle);
}



void Device_Actuators(void *arg)
{
  /* Cache for Task */
  CAN_COB Rx_Buff;
  CAN_COB Tx_Buff;
  
  /* Pre-Load for task */
  Tx_Buff.ID = 1;
  
//  Analyzer.set(30, 500, 0, 500, 3);
//  Analyzer.start(AUTO_FREQ);
  
  Test_Motor.enterCtrlMode_cmd(Tx_Buff.Data);
  xQueueSend(RMMotor_QueueHandle, &Tx_Buff, 0);
  
  /* Infinite loop */
  TickType_t xLastWakeTime_t;
  xLastWakeTime_t = xTaskGetTickCount();
  for(;;)
  {
    /* Connection check */

    /* Read Message */
//    Analyzer.sample_control(0.5f*Analyzer.val_signal,0.7f);
    
    xQueueReceive(CAN1_TxPort, &Rx_Buff, 0);
    if(Rx_Buff.ID == 1) Test_Motor.unpack_reply(Rx_Buff.Data);
    else {}
    /* Send Message */
    Test_Motor.pack_cmd(Tx_Buff.Data, 0, 5, 0);
    xQueueSend(RMMotor_QueueHandle, &Tx_Buff, 0);
    
    /* Pass control to the next task */
    vTaskDelayUntil(&xLastWakeTime_t,1);
  }
}

void Device_DR16(void *arg)
{
  /* Cache for Task */

  /* Pre-Load for task */

  /* Infinite loop */
  TickType_t xLastWakeTime_t;
  xLastWakeTime_t = xTaskGetTickCount();
  for(;;)
  {
    
    DR16.Check_Link(xTaskGetTickCount());
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
    
    /* Read Other board Message */

    
  /* Pass control to the next task ------------------------------------------*/
    vTaskDelayUntil(&xLastWakeTime_t,2);
  }
}

/* User Code End Here ---------------------------------*/

/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
