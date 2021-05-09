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
  *  - All tasks should be created in "System_Tasks_Init()".   
  *  - FreeRTOS scheduler will be started after tasks are created.
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
#include <iostream>
#include "inputInterface.h"
#include "System_Config.h"
#include "System_DataPool.h"

/* Service */
#include "CoppeliaSim/CoppeliaSim_service.h"
#include "simulation.h"
#include "win32_support.h"

/* User support package & SRML */
#include "User_Task.h"
#include "UpperMonitor.h"
/* Private variables ---------------------------------------------------------*/


/*Founctions------------------------------------------------------------------*/
/**
* @brief Load drivers ,modules, and data resources for tasks.
* @note  Edit this function to add Init-functions and configurations.
*/
void System_Resource_Init(void)
{
  std::cout << 
    "* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * \r\n"
    "*                                                                                   * \r\n"
    "*                     Robomaster Development Simulation Platform                    * \r\n"
    "*                          Supported By Coppeliasim Robotic                         * \r\n"
    "*                                                                                   * \r\n"
    "*                            COPYRIGHT(C) SCUT-ROBOTLAB                             * \r\n"
    "*                                                                                   * \r\n"
    "* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * \r\n" ;
  /* Drivers Init ---------------------*/

  /* RTOS resources Init --------------*/

  /* Other resources Init -------------*/
  LogOutputBacken_Init();

  /* Modules Init ---------------------*/
  
  /* Service configurations -----------*/
  SysLog.getMilliTick_regist(xTaskGetTickCount);
  SysLog.filter_conf(DEFAULT_TAG, LOG_LVL_ALL, LogConsole_Output);
  SysLog.filter_conf("Simulation", LOG_LVL_INFO, LogConsole_Output, &Filter_List[0]);
  SysLog.filter_conf("CoppeliaSim", LOG_LVL_INFO, LogConsole_Output, &Filter_List[1]);
  SysLog.filter_conf("TigerArm", LOG_LVL_INFO, LogConsole_Output, &Filter_List[2]);
  SysLog.global_conf(LOG_LVL_ALL, false);
  SysLog.Record(_INFO_, "SysLog initialized success...");

  SerialPort.Register_RecvCallBack(RecHandle);

}  


/**
* @brief Load and start User Tasks. This function run directly in "main()"
* @note  Edit this function to add tasks into the activated tasks list.
*/
void System_Tasks_Init(void)
{
  /* Syetem Service init --------------*/
  /* KeyBord & Mouse (Developed by MFC) Service */
  SysLog.Record(_INFO_, "Initializing Keyboard and mouse interfaces...");
  keyboard.init();
  //mouse.init();

  /*Serial Communication service*/
  SysLog.Record(_INFO_, "Initializing communication service...");
  Service_SerialPortCom_Init(1);

  /*CoppeliaSim service*/
  SysLog.Record(_INFO_, "Initializing CoppeliaSim simulation service...");
  //Coppeliasim_Client_Init();
  Service_CoppeliaSim_Init();

  //Remember! The following functions must be called only after 
  //CoppeliaSim-client connected successfully.
  Body = CoppeliaSim->Add_Object("chassis_respondable", OTHER_OBJECT, { SIM_ORIENTATION | CLIENT_RO, SIM_VELOCITY | CLIENT_RO });
  Joint[Shoulder_yaw] = CoppeliaSim->Add_Object("shoulder_yaw", JOINT, { SIM_FORCE | CLIENT_RO, SIM_POSITION | CLIENT_RW });
  Joint[Shoulder_pitch] = CoppeliaSim->Add_Object("shoulder_pitch", JOINT, { SIM_FORCE | CLIENT_RO, SIM_POSITION | CLIENT_RW });
  Joint[Elbow] = CoppeliaSim->Add_Object("Elbow", JOINT, { SIM_FORCE | CLIENT_RO, SIM_POSITION | CLIENT_RW });
  Joint[Wrist_roll] = CoppeliaSim->Add_Object("Wrist_roll", JOINT, { SIM_FORCE | CLIENT_RO, SIM_POSITION | CLIENT_RW });
  Joint[Wrist_pitch] = CoppeliaSim->Add_Object("Wrist_pitch", JOINT, { SIM_FORCE | CLIENT_RO, SIM_POSITION | CLIENT_RW });
  Joint[Wrist_yaw] = CoppeliaSim->Add_Object("Wrist_yaw", JOINT, { SIM_FORCE | CLIENT_RO, SIM_POSITION | CLIENT_RW });
  //Wheel[_LF] = CoppeliaSim->Add_Object("Force_Sensor0", FORCE_SENSOR, { SIM_FORCE | CLIENT_RO });
  //Wheel[_RF] = CoppeliaSim->Add_Object("Force_Sensor1", FORCE_SENSOR, { SIM_FORCE | CLIENT_RO });
  //Wheel[_RB] = CoppeliaSim->Add_Object("Force_Sensor2", FORCE_SENSOR, { SIM_FORCE | CLIENT_RO });
  //Wheel[_LB] = CoppeliaSim->Add_Object("Force_Sensor3", FORCE_SENSOR, { SIM_FORCE | CLIENT_RO });

  //Signal = CoppeliaSim->Add_Object("Name.fire", SIM_INTEGER_SIGNAL, { SIM_SIGNAL_OP | CLIENT_WO });
  //Signal->target = 0;
  
  //check
  std::string check_result = "";
  for (_simObjectHandle_Type* p : Joint) {
      if (p == nullptr) check_result.append("\n\rJoint pointer exploded");
  }
  for (_simObjectHandle_Type* p : Wheel) {
      if (p == nullptr) break;//check_result.append("\n\rWheel pointer exploded");
  }
  if (check_result.empty())
      std::cout << "------------------------> Objects added successfully ! <------------------------" << std::endl;
  else {
      std::cout << "----------------------------> Objects add failed ! <----------------------------";
      std::cout << check_result << std::endl;
      std::cout << "------------------------> Process will be suspended ! <-------------------------" << std::endl;
      Sleep(2000);
      exit(0);
  }
 
  /* Applications Init ----------------*/
  User_Tasks_Init();

  /* Start the tasks and timer running. */
  vTaskStartScheduler();
}


/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/

