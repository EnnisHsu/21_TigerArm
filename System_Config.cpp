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
		"* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * \r\n";
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
	SysLog.global_conf(LOG_LVL_ALL, false);
	SysLog.Record(_INFO_, "SysLog initialized success...");

	SerialPort.Register_RecvCallBack(RecHandle);
	SerialPort_Vision.Register_RecvCallBack(VisionSerial_RxCpltCallback);

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
	Service_SerialPortCom_Init();

	/*CoppeliaSim service*/
	SysLog.Record(_INFO_, "Initializing CoppeliaSim simulation service...");
	Service_CoppeliaSim_Init();

	Sentry_Body = CoppeliaSim->Add_Object("chassis_respondable2", OTHER_OBJECT, { SIM_POSITION | CLIENT_RO, SIM_VELOCITY | CLIENT_RO });
	Wheel[DrivenWheel_Back][Left] = CoppeliaSim->Add_Object("DrivenWheel_BL", JOINT, { SIM_FORCE | CLIENT_RO, SIM_VELOCITY | CLIENT_RO });
	Wheel[DrivenWheel_Back][Right] = CoppeliaSim->Add_Object("DrivenWheel_BR", JOINT, { SIM_FORCE | CLIENT_RO, SIM_VELOCITY | CLIENT_RO });
	Wheel[DrivenWheel_Front][Left] = CoppeliaSim->Add_Object("DrivenWheel_FL", JOINT, { SIM_FORCE | CLIENT_RO, SIM_VELOCITY | CLIENT_RO });
	Wheel[DrivenWheel_Front][Right] = CoppeliaSim->Add_Object("DrivenWheel_FR", JOINT, { SIM_FORCE | CLIENT_RO, SIM_VELOCITY | CLIENT_RO });
	Wheel[DrivenWheel_Up][Left] = CoppeliaSim->Add_Object("DrivenWheel_UL", JOINT, { SIM_FORCE | CLIENT_RO, SIM_VELOCITY | CLIENT_RO });
	Wheel[DrivenWheel_Up][Right] = CoppeliaSim->Add_Object("DrivenWheel_UR", JOINT, { SIM_FORCE | CLIENT_RO, SIM_VELOCITY | CLIENT_RO });
	Wheel[Driving_Wheel][Left] = CoppeliaSim->Add_Object("Wheel_L", JOINT, { SIM_FORCE | CLIENT_RO, SIM_VELOCITY | CLIENT_RW });
	Wheel[Driving_Wheel][Right] = CoppeliaSim->Add_Object("Wheel_R", JOINT, { SIM_FORCE | CLIENT_RO, SIM_VELOCITY | CLIENT_RW });
	Gimbal[Gimbal_Up][Pitch] = CoppeliaSim->Add_Object("UpGimbal_Pitch", JOINT, { SIM_FORCE | CLIENT_RO, SIM_POSITION | CLIENT_RW });
	Gimbal[Gimbal_Up][Yaw] = CoppeliaSim->Add_Object("UpGimbal_Yaw", JOINT, { SIM_FORCE | CLIENT_RO, SIM_POSITION | CLIENT_RW });
	Gimbal[Gimbal_Down][Pitch] = CoppeliaSim->Add_Object("DownGimbal_Pitch", JOINT, { SIM_FORCE | CLIENT_RO, SIM_POSITION | CLIENT_RW });
	Gimbal[Gimbal_Down][Yaw] = CoppeliaSim->Add_Object("DownGimbal_Yaw", JOINT, { SIM_FORCE | CLIENT_RO, SIM_POSITION | CLIENT_RW });

	FireData[Gimbal_Up][Fire] = CoppeliaSim->Add_Object("Up.fire", SIM_INTEGER_SIGNAL, { SIM_SIGNAL_OP | CLIENT_RW });
	FireData[Gimbal_Up][Fire]->target = 0;
	FireData[Gimbal_Up][Bullet_Speed] = CoppeliaSim->Add_Object("Up.bullet_speed", SIM_INTEGER_SIGNAL, { SIM_SIGNAL_OP | CLIENT_WO });
	FireData[Gimbal_Up][Bullet_Speed]->target = 0;
	FireData[Gimbal_Down][Fire] = CoppeliaSim->Add_Object("Down.fire", SIM_INTEGER_SIGNAL, { SIM_SIGNAL_OP | CLIENT_RW });
	FireData[Gimbal_Down][Fire]->target = 0;
	FireData[Gimbal_Down][Bullet_Speed] = CoppeliaSim->Add_Object("Down.bullet_speed", SIM_INTEGER_SIGNAL, { SIM_SIGNAL_OP | CLIENT_RW });
	FireData[Gimbal_Down][Bullet_Speed]->target = 0;

	GyroData[Gimbal_Up][Yaw][Vel] = CoppeliaSim->Add_Object("Sentry1_Up.YawVel", SIM_FLOAT_SIGNAL, { SIM_SIGNAL_OP | CLIENT_RO });
	GyroData[Gimbal_Up][Pitch][Vel] = CoppeliaSim->Add_Object("Sentry1_Up.PitchVel", SIM_FLOAT_SIGNAL, { SIM_SIGNAL_OP | CLIENT_RO });
	GyroData[Gimbal_Down][Yaw][Vel] = CoppeliaSim->Add_Object("Sentry1_Down.YawVel", SIM_FLOAT_SIGNAL, { SIM_SIGNAL_OP | CLIENT_RO });
	GyroData[Gimbal_Down][Pitch][Vel] = CoppeliaSim->Add_Object("Sentry1_Down.PitchVel", SIM_FLOAT_SIGNAL, { SIM_SIGNAL_OP | CLIENT_RO });
	GyroData[Gimbal_Up][Yaw][Angle] = CoppeliaSim->Add_Object("Sentry1_Up.YawAng", SIM_FLOAT_SIGNAL, { SIM_SIGNAL_OP | CLIENT_RO });
	GyroData[Gimbal_Up][Pitch][Angle] = CoppeliaSim->Add_Object("Sentry1_Up.PitchAng", SIM_FLOAT_SIGNAL, { SIM_SIGNAL_OP | CLIENT_RO });
	GyroData[Gimbal_Down][Yaw][Angle] = CoppeliaSim->Add_Object("Sentry1_Down.YawAng", SIM_FLOAT_SIGNAL, { SIM_SIGNAL_OP | CLIENT_RO });
	GyroData[Gimbal_Down][Pitch][Angle] = CoppeliaSim->Add_Object("Sentry1_Down.PitchAng", SIM_FLOAT_SIGNAL, { SIM_SIGNAL_OP | CLIENT_RO });
	/*check*/
	std::string check_result = "";
	if (Sentry_Body == nullptr)
		check_result.append("\n\rSentry_Body pointer exploded");
	for (int i = 0; i < Wheel_Group * Wheel_Pos; i++)
	{
		if (*(Wheel[0] + i) == nullptr)
			check_result.append("\n\rWheel pointer exploded");
	}
	for (int i = 0; i < Gimbal_Name * Axis_Num; i++)
	{
		if (*(Gimbal[0] + i) == nullptr)
		{
			check_result.append("\n\rGimbal pointer exploded");
		}
	}
	for (int i = 0; i < Gimbal_Name * Signal_Name; i++)
	{
		if (*(FireData[0] + i) == nullptr)
		{
			check_result.append("\n\rFireData pointer exploded");
			std::cout << i << "\n";
		}
	}
	for (int i = 0; i < Gimbal_Name * Axis_Num * Gyro_Signal_Name; i++)
	{
		if (*(GyroData[0][0] + i) == nullptr)
			check_result.append("\n\rGyroData pointer exploded");
	}
	if (!check_result.empty())
	{
		std::cout << "----------------------------> Objects add failed ! <----------------------------";
		std::cout << check_result << std::endl;
		std::cout << "------------------------> Process will be suspended ! <-------------------------" << std::endl;
		Sleep(2000);
		exit(0);
	}
	else
		std::cout << "------------------------> Objects added successfully ! <------------------------" << std::endl;
#ifndef SYNC_MODE
	/* Applications Init ----------------*/
	User_Tasks_Init();

	/* Start the tasks and timer running. */
	vTaskStartScheduler();
#endif
}


/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
