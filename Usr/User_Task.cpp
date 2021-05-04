/**
******************************************************************************
* @file   User_Task.cpp
* @brief  User task running file. Please add your task in this file, and create
*         it in "System_Config.cpp"
******************************************************************************
* @note
*  - Before running your task, just do what you want ~ 
*  - More devices or using other classification is decided by yourself ~ 
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
#include "iostream"
#include "inputInterface.h"
#include "User_Task.h"
#include "simulation.h"
#include "UpperMonitor.h"
#include "win32_support.h"
#include "ArmResolve.h"
#include "windows.h"

/* Private define ------------------------------------------------------------*/
TaskHandle_t RobotCtrl_Handle;
TaskHandle_t DataDisplay_Handle;
TaskHandle_t JointCtrl_Handle;
TaskHandle_t TigerArmUpdate_Handle;

/* Private variables ---------------------------------------------------------*/
/* Private function declarations ---------------------------------------------*/
void Task_RobotCtrl(void *arg);
void Task_DataDisplay(void *arg);
void Task_JointCtrl(void *arg);
void Task_TigerArmUpdate(void *arg);

/* Exported devices ----------------------------------------------------------*/
/* Motor & ESC & Other actuators*/

/* Remote control */
enum Ctrl_Mode_Typedef {
    kb_input=0x01,
    dir_ctrl
} ArmCtrlMode;
/* IMU & NUC & Other sensors */

/* Other boards */
MechanicalArm TigerArm;
Matrix T6_g, Tw_c, Tw_g;
/* Function prototypes -------------------------------------------------------*/
/**
* @brief  Initialization of Tasks
* @param  None.
* @return None.
*/
void User_Tasks_Init(void)
{
  xTaskCreate(Task_JointCtrl, "Joint Control", Huge_Stack_Size, NULL, PrioritySuperHigh, &JointCtrl_Handle);
  xTaskCreate(Task_RobotCtrl, "Robot Control", Huge_Stack_Size, NULL, PrioritySuperHigh, &RobotCtrl_Handle);
  xTaskCreate(Task_DataDisplay, "Data Display", Huge_Stack_Size, NULL, PrioritySuperHigh, &DataDisplay_Handle);
  xTaskCreate(Task_TigerArmUpdate, "TigerArm Update", Huge_Stack_Size, NULL, PrioritySuperHigh, &TigerArmUpdate_Handle);
}

void TigerArm_Init(void)
{
    for (int i = 1; i <= 4; i++)
    {
        for (int j = 1; j <= 4; j++)
        {
            T6_g(i, j) = 1.0f;
            if (i == j) Tw_g(i, j) = 1.0f;
            else Tw_g(i, j) = 0.0f;
        }
    }
    TigerArm.Init(Tw_c, T6_g);
    
    double a[6] = { 0.0f,0.0f,0.21f,0.0f,0.0f,0.0f };
    double alpha[6] = { 0.0f,90.0f,0.0f,90.0f,-90.0f,-90.0f };
    double d[6] = { 0.0f,0.0f,0.0f,0.12f,0.0f,0.0f };
    
    double interval[6][2] = { {-90.0f,210.0f},{0.0f,115.0f},{35.0f,120.0f},{-180.0f,180.0f},{-33.0f,90.0f},{-45.0f,45.0f} };
    TigerArm.Set_DHModel_Config(a, d, interval);
}

void TigerArm_move(double x, double y, double z)
{
    Tw_g(1,4) = x; Tw_g(2, 4) = y; Tw_g(3, 4) = z;
    TigerArm.SetWorldGoal(Tw_g);
    TigerArm.solveT0_6();
    TigerArm.Set_Cubic_IP_Config(xTaskGetTickCount());
    TigerArm.IK_cal();
}

void Task_TigerArmUpdate(void* arg)
{
    /* Cache for Task */
    TickType_t xLastWakeTime_t;
    const TickType_t xBlockTime = pdMS_TO_TICKS(5);
    /* Pre-Load for task */
    xLastWakeTime_t = xTaskGetTickCount();
    (void)arg;
    /* Infinite loop */
    for (;;)
    {
        theta_deg_pack cur_deg;
        cur_deg.deg[0] = Joint[Shoulder_yaw]->obj_Data.angle_f;
        cur_deg.deg[1] = Joint[Shouder_pitch]->obj_Data.angle_f;
        cur_deg.deg[2] = Joint[Elbow]->obj_Data.angle_f;
        cur_deg.deg[3] = Joint[Wrist_roll]->obj_Data.angle_f;
        cur_deg.deg[4] = Joint[Wrist_pitch]->obj_Data.angle_f;
        cur_deg.deg[5] = Joint[Wrist_yaw]->obj_Data.angle_f;
        TigerArm.update(&cur_deg);
        vTaskDelayUntil(&xLastWakeTime_t, xBlockTime);
    }
}

/**
* @brief  User can run all your robot control code in this task.
* @param  None.
* @return None.
*/
void Task_RobotCtrl(void *arg)
{ 
  /* Cache for Task */
  TickType_t xLastWakeTime_t;
  const TickType_t xBlockTime = pdMS_TO_TICKS(5);
  /* Pre-Load for task */
  xLastWakeTime_t     = xTaskGetTickCount();
  (void)arg;
  TigerArm_Init();
  /* Infinite loop */
  for (;;)
  {
      if (keyboard.isKeyPressed(_SHIFT_KV))
      {
          if (ArmCtrlMode == kb_input)
          {
              ArmCtrlMode = dir_ctrl;
              std::cout << "Ctrl Mode change to dir_ctrl\n";
          }
          else if (ArmCtrlMode == dir_ctrl)
          {
              ArmCtrlMode = kb_input;
              std::cout << "Ctrl Mode change to kb_input\n";
          }
      }
      if (ArmCtrlMode == dir_ctrl)
      {
          //if (keyboard.isKeyPressed(_W_KV))
      }
   //mouse.resolveVelocity(5);
   //mouse.setExitFlag(false);
   //std:: cout << "Mouse speed :" << mouse.velocity.vx << "," << mouse.velocity.vy << std::endl;
      
      
   /* Pass control to the next task */
    vTaskDelayUntil(&xLastWakeTime_t, xBlockTime);
  }
}
/**
* @brief  Joint drive
* @param  None.
* @return None.
*/
void Task_JointCtrl(void* arg)
{
    /* Cache for Task */
    TickType_t xLastWakeTime_t;
    const TickType_t xBlockTime = pdMS_TO_TICKS(5);
    /* Pre-Load for task */
    xLastWakeTime_t = xTaskGetTickCount();
    (void)arg;

    /* Infinite loop */
    for (;;)
    {
        theta_deg_pack cur_target;
        cur_target = TigerArm.get_curtarget_deg(xTaskGetTickCount());
        Joint[Shoulder_yaw]->obj_Target.angle_f = cur_target.deg[0];
        Joint[Shouder_pitch]->obj_Target.angle_f = cur_target.deg[1];
        Joint[Elbow]->obj_Target.angle_f = cur_target.deg[2];
        Joint[Wrist_roll]->obj_Target.angle_f = cur_target.deg[3];
        Joint[Wrist_pitch]->obj_Target.angle_f = cur_target.deg[4];
        Joint[Wrist_yaw]->obj_Target.angle_f = cur_target.deg[5];
        vTaskDelayUntil(&xLastWakeTime_t, xBlockTime);
    }
}


/**
* @brief  User can collect and show datas in a lower frequency.
* @param  None.
* @return None.
*/
void Task_DataDisplay(void *arg)
{
  /* Cache for Task */
  _Unified_COB UpperMonitor_COB = {NULL,39};
  const TickType_t xBlockTime = pdMS_TO_TICKS(10);
  TickType_t xLastWakeTime_t;

  /* Pre-Load for task */
  xLastWakeTime_t = xTaskGetTickCount();
  (void)arg;

  /* Infinite loop */
  for (;;)
  {
    UpperMonitor_COB.pData = Get_SendPackage();

    if(UpperMonitor_COB.pData != NULL)
      while (!SerialPort.BuffSend(UpperMonitor_COB.pData, UpperMonitor_COB.len)) {};
    /* Pass control to the next task */
    vTaskDelayUntil(&xLastWakeTime_t, xBlockTime);
  }
}
/* User Code End Here ---------------------------------*/

/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
