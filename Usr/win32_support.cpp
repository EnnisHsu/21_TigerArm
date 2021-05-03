/**
  ******************************************************************************
  * Copyright (c) 2019 - ~, SCUT-RobotLab Development Team
  * @file    win32_support.cpp
  * @brief   This file provide win32 services for MCU programm.
  * @date    2019-12-17
  * @version 1.0
  *
  ==============================================================================
                            How to use this library
  ==============================================================================
    @note
      The following Service is provied by this lib: \n
      - Program run timer.
      - IO stream for SysLog.
      - Serial port communication.
    @warning
      - Using object "SerialPort" to communication by COM1 in default.
        Also user can set other COM port by rewriting "Service_SerialPortCom_Init()".
  ******************************************************************************
  * @attention
  *
  * if you had modified this file, please make sure your code does not have any
  * bugs, update the version Number, write dowm your name and the date. The most
  * important thing is make sure the users will have clear and definite under-
  * standing through your new brief.
  *
  * <h2><center>&copy; Copyright (c) 2019 - ~, SCUT-RobotLab Development Team.
  * All rights reserved.</center></h2>
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include <win32_support.h>
#include <Windows.h>
#include <iostream>
#include <fstream>
#include <time.h>

/* Private define ------------------------------------------------------------*/
using namespace std;
CSerialPort SerialPort;

/* Private variables ---------------------------------------------------------*/
ofstream outFile;
char outMsg[256];
/* Private type --------------------------------------------------------------*/
/* Private function declarations ---------------------------------------------*/

/**
*  @brief Time Service 
*/
uint32_t GetMillSecond(void)
{
  return (uint32_t)clock();
}

uint32_t GetMicroSecond(void)
{
  return (uint32_t)(clock() * 1000);
}

/**
*  @brief IO stream for SysLog.
*         Output a log file named "sys_log.txt" or display in console.
*/
void LogOutputBacken_Init(void)
{
  /*Console init*/
  SetConsoleTitleA("System Log Record");

  /*Creat file*/
  outFile.open("sys_log.txt", ios::trunc);
}

uint32_t LogConsole_Output(uint8_t *msg, uint16_t len)
{
  memcpy(outMsg, msg,(size_t)len);
  outMsg[len] = 0;

  cout << outMsg;

  return 0;
}

uint32_t LogFile_Output(uint8_t *msg, uint16_t len)
{
  memcpy(outMsg, msg, len);
  outMsg[len - 1] = 0;

  outFile.open("sys_log.txt", ios::app);
  if(outFile.is_open())
    outFile << outMsg;
  else {};
  outFile.close();

  return 0;
}

void LogOutputBacken_DeInit(void)
{
  outFile.close();
}


/**
*  @brief Serial port communication service
*/
void Service_SerialPortCom_Init(int port_num)
{
  if (!SerialPort.InitPort(port_num))
  {
    std::cout << " - Serial Port init fail ..." << std::endl;
  }
  else
  { 
    std::cout << " - Serial Port init success !" << std::endl;
  }

  if (!SerialPort.OpenListenThread())
  {
    std::cout << " - Open Listen Thread fail ..." << std::endl;
  }
  else
  {
    std::cout << " - Open Listen Thread success !" << std::endl;
  }
}


/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
