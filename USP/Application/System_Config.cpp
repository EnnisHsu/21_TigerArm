/**
  ******************************************************************************
  * @file   System_config.cpp
  * @brief  Deploy resources,tasks and services in this file.
  ******************************************************************************
  * @note
  *  - Before running your Task you should first include your headers and init- 
  *    ialize used resources in "System_Resource_Init()". This function will be 
  *    called before tasks Start.
  *    
  *  - All tasks should be created in "System_Tasks_Init()", this function will
  *    be called in 'defaultTask()'.
  *
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
#include "System_Config.h"
#include "System_DataPool.h"

/* Service */
#include "Service_Devices.h"
#include "Service_Debug.h"
//#include "Service_SwerveChassis.h"
#include "Service_Communication.h"
#include "Service_MotoCtrl.h"
#include "Service_RobotCtrl.h"
/* User support package & SRML */
#include <SRML.h>
#include "SEGGER_SYSVIEW.h"
#include "UpperMonitor.h"
/* Private variables ---------------------------------------------------------*/


/*Founctions------------------------------------------------------------------*/
/**
* @brief Load drivers ,modules, and data resources for tasks.
* @note  Edit this function to add Init-functions and configurations.
*/
void System_Resource_Init(void)
{
  /* Drivers Init ---------------------*/
  Timer_Init(&htim4, USE_HAL_DELAY);
  
  CAN_Init(&hcan1,User_CAN1_RxCpltCallback);
  CAN_Init(&hcan2,User_CAN2_RxCpltCallback);
  

  Uart_Init(&huart1, Uart1_Rx_Buff, USART1_RX_BUFFER_SIZE,RecHandle);
  Uart_Init(&huart2, Uart2_Rx_Buff, USART2_RX_BUFFER_SIZE,User_UART2_RxCpltCallback);
  //Uart_Init(&huart3, Uart3_Rx_Buff, USART3_RX_BUFFER_SIZE,NULL);
  Uart_Init(&huart4, Uart4_Rx_Buff, USART4_RX_BUFFER_SIZE,User_UART4_RxCpltCallback);
  Uart_Init(&huart5, Uart5_Rx_Buff, USART5_RX_BUFFER_SIZE,NULL);
  Uart_Init(&huart6, Uart6_Rx_Buff, USART6_RX_BUFFER_SIZE,User_UART6_RxCpltCallback);
  
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_4);
	
  SPI_Init(&hspi1,GPIOA,GPIO_PIN_4,100);
  
#ifdef USE_BMX055
  BMX055_Init(GPIOB, GPIO_PIN_10, GPIO_PIN_11, 10, 11, 0.002f);
#else
  MPU6050_Init(GPIOB, GPIO_PIN_6, GPIO_PIN_7);
#endif
  
  Flash.init();
  /* RTOS resources Init --------------*/
  USART_RxPort       = xQueueCreate(4,sizeof(USART_COB));
  USART_TxPort       = xQueueCreate(2,sizeof(USART_COB));
  CAN1_TxPort        = xQueueCreate(4,sizeof(CAN_COB));
  CAN2_TxPort        = xQueueCreate(4,sizeof(CAN_COB));
  RMMotor_QueueHandle= xQueueCreate(10,sizeof(CAN_COB));
  BodCom_QueueHandle = xQueueCreate(2,sizeof(USART_COB));
  IMU_QueueHandle    = xQueueCreate(2,sizeof(CAN_COB));
  NUC_QueueHandle    = xQueueCreate(2,sizeof(USART_COB));
  DR16_QueueHandle   = xQueueCreate(2,sizeof(USART_COB));
  Referee_QueueHandle = xQueueCreate(2,sizeof(USART_COB));
  
  /* Other resources Init -------------*/
  
  
  /* Modules Init ---------------------*/
  SEGGER_SYSVIEW_Conf();
  myPIDTimer::getMicroTick_regist(Get_SystemTimer);
  Analyzer.getMicroTick_regist(Get_SystemTimer);
  SysLog.getMilliTick_regist(HAL_GetTick);
  SysLog.filter_conf(DEFAULT_TAG, LOG_LVL_ALL,Uart1_Transmit);
  SysLog.filter_conf("Test Tag", LOG_LVL_ALL, Uart1_Transmit,&Filter_List[0]);
  SysLog.global_conf(LOG_LVL_ALL, false);
  
  /* Service configurations -----------*/
  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);
}  

extern int flag;
/**
* @brief Load and start User Tasks. 
* @note  Edit this function to add tasks into the activated tasks list.
*/
void System_Tasks_Init(void)
{ 
  /* Syetem Service init --------------*/
//  Service_Debug_Init();
  Service_Devices_Init();
  Service_Communication_Init();
  Service_RobotCtrl_Init();
  /* Applications Init ----------------*/
  //xTaskCreate(Task_SwerveChassis, "Dev.Actuator" , Tiny_Stack_Size,    NULL, PrioritySuperHigh,   &TestSwerveChassis_Handle);
}


/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/

