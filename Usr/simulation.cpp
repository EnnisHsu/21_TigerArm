#include "simulation.h"
#include "CoppeliaSim/CoppeliaSim.h"
#include <iostream>
#include "System_DataPool.h"
using namespace std;
//CoppeliaSim_Client CoppeliaSim_old;
TaskHandle_t Sim_Handle;

void Service_CoppeliaSim_Init()
{
  if (!CoppeliaSim->Start("127.0.0.1", 5000, 5, simx_opmode_oneshot))
  {
    SysLog.Record(_INFO_, "Simulation", "CoppeliaSim failed to connect...");
    SysLog.Record(_INFO_, "Simulation", "Waiting for reconnecting...");
  }
  xTaskCreate(Task_Sim, "CoppeliaSim service", Huge_Stack_Size, NULL, PriorityHigh, &Sim_Handle);
}


void Task_Sim(void* arg)
{
  /* Cache for Task */
  TickType_t xLastWakeTime_t;
  const TickType_t xBlockTime = pdMS_TO_TICKS(5);
  /* Pre-Load for task */
  xLastWakeTime_t = xTaskGetTickCount();
  while (!CoppeliaSim->Start("127.0.0.1", 5000, 5, simx_opmode_oneshot)) {}
  SysLog.Record(_INFO_, "Simulation", "CoppeliaSim connection initialized successfully !");

  (void)arg;
  /* Infinite loop */
  for (;;)
  {
    if (CoppeliaSim->ComWithServer())
    {
      //SysLog.Record(_INFO_, "Simulation", "Simulation is running !");
    }
    else
      //SysLog.Record(_INFO_, "Simulation", "Simulation is stopped !");

    /* Pass control to the next task */
    Sleep(5);
  }
}
