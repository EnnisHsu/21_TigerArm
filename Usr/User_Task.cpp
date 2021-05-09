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
#include "conio.h"
//#include "windows.h"

/* Private define ------------------------------------------------------------*/
TaskHandle_t RobotCtrl_Handle;
TaskHandle_t DataDisplay_Handle;
TaskHandle_t JointCtrl_Handle;
TaskHandle_t TigerArmUpdate_Handle;
TaskHandle_t TigerArmCtrl_Handle;

/* Private variables ---------------------------------------------------------*/
/* Private function declarations ---------------------------------------------*/
void Task_RobotCtrl(void *arg);
void Task_DataDisplay(void *arg);
void Task_JointCtrl(void *arg);
void Task_TigerArmUpdate(void *arg);
void Task_TigerArmCtrl(void* arg);

/* Exported devices ----------------------------------------------------------*/
/* Motor & ESC & Other actuators*/

/* Remote control */
enum Ctrl_Mode_Typedef {
    dir_ctrl,
    kb_input,
} ArmCtrlMode;
/* IMU & NUC & Other sensors */

/* Other boards */
MechanicalArm TigerArm(1);
Matrix T6_g(4,4,0.0f), Tw_c(4,4,0.0f), Tw_g(4,4,0.0f);
double move_step = 0.05f;
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
  xTaskCreate(Task_TigerArmCtrl, "TigerArm Control", Huge_Stack_Size, NULL, PrioritySuperHigh, &TigerArmCtrl_Handle);
}

void TigerArm_Init(void)
{
    /*Tw_g(1,1) = 1.0f; Tw_g(2,2) = 1.0f;
    Tw_g(3,3) = 1.0f; Tw_g(4,4) = 1.0f;
    T6_g(1, 4) = 1.0f; T6_g(2, 4) = 1.0f;
    T6_g(3, 4) = 1.0f; T6_g(4, 4) = 1.0f;
    TigerArm.Init(Tw_c, T6_g);*/
    Joint[Shoulder_yaw]->obj_Target.angle_f = 0;
    Joint[Shoulder_pitch]->obj_Target.angle_f = -(PI-1)/2;
    Joint[Elbow]->obj_Target.angle_f = PI*90/180;
    Joint[Wrist_roll]->obj_Target.angle_f = 0;
    Joint[Wrist_pitch]->obj_Target.angle_f = 0;
    Joint[Wrist_yaw]->obj_Target.angle_f = 0;
    double a[6] = { 0.0f,0.0f,0.21f,0.0f,0.0f,0.0f };
    double alpha[6] = { 0.0f,90.0f,0.0f,90.0f,-90.0f,-90.0f };
    double d[6] = { 0.0f,0.0f,0.0f,0.12f,0.0f,0.0f };
    
    double interval[6][2] = { {-90.0f,210.0f},{-90.0f,0.0f},{35.0f,120.0f},{-180.0f,180.0f},{-33.0f,90.0f},{-45.0f,45.0f} };
    TigerArm.Set_DHModel_Config(a, d, interval);
}



void Task_TigerArmUpdate(void* arg)
{
    /* Cache for Task */
    TickType_t xLastWakeTime_t;
    const TickType_t xBlockTime = pdMS_TO_TICKS(5);
    /* Pre-Load for task */
    xLastWakeTime_t = xTaskGetTickCount();
    TickType_t LogLastOutputTime = xTaskGetTickCount();
    //vTaskDelayUntil(&xLastWakeTime_t, 1000);
    (void)arg;
    /* Infinite loop */
    for (;;)
    {
        theta_deg_pack cur_deg;
        cur_deg.deg[0] = rad2deg(Joint[Shoulder_yaw]->obj_Data.angle_f);
        cur_deg.deg[1] = rad2deg(Joint[Shoulder_pitch]->obj_Data.angle_f);
        cur_deg.deg[2] = rad2deg(Joint[Elbow]->obj_Data.angle_f);
        cur_deg.deg[3] = rad2deg(Joint[Wrist_roll]->obj_Data.angle_f);
        cur_deg.deg[4] = rad2deg(Joint[Wrist_pitch]->obj_Data.angle_f);
        cur_deg.deg[5] = rad2deg(Joint[Wrist_yaw]->obj_Data.angle_f);
        TigerArm.update(&cur_deg);
        if (xTaskGetTickCount() - LogLastOutputTime > 2000)
        {
            SysLog.Record(_INFO_, "TigerArm", "TigerArm joint deg now is theta1:%f theta2:%f theta3:%f theta4:%f theta5:%f theta6:%f...", cur_deg.deg[0], cur_deg.deg[1], cur_deg.deg[2], cur_deg.deg[3], cur_deg.deg[4], cur_deg.deg[5]);
            SysLog.Record(_INFO_, "TigerArm", "TigerArm now is at point(%f,%f,%f)...", TigerArm.GetWorldx(), TigerArm.GetWorldy(), TigerArm.GetWorldz());
            LogLastOutputTime = xTaskGetTickCount();
        }
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
  const TickType_t xHitTime = pdMS_TO_TICKS(200);
  /* Pre-Load for task */
  xLastWakeTime_t     = xTaskGetTickCount();
  (void)arg;
  TigerArm_Init();
  int isResume = 0;
  /* Infinite loop */
  for (;;)
  {
      if (GetKeyState(_CTRL_KV)<0)
      {
          if (ArmCtrlMode == kb_input)
          {
              ArmCtrlMode = dir_ctrl;
              //std::cout << "Ctrl Mode change to dir_ctrl\n";
              SysLog.Record(_INFO_, "TigerArm", "TigerArm control mode change to direction key control...");
              vTaskDelayUntil(&xLastWakeTime_t, xHitTime);
          }
          else if (ArmCtrlMode == dir_ctrl)
          {
              ArmCtrlMode = kb_input;
              SysLog.Record(_INFO_, "TigerArm", "TigerArm control mode change to keyboard input control...");
              vTaskDelayUntil(&xLastWakeTime_t, xHitTime);
          }
      }
      if (ArmCtrlMode == kb_input)
      {
          if (GetKeyState(_G_KV) < 0)
          {
              double wx, wy, wz;
              std::cout << "请输出世界坐标系:";
              std::cin >> wx >> wy >> wz;
              TigerArm.SetTargetx(wx);
              TigerArm.SetTargety(wy);
              TigerArm.SetTargetz(wz);
              vTaskResume(TigerArmCtrl_Handle);
              xTaskNotify(TigerArmCtrl_Handle, 1, eSetValueWithOverwrite);//1should be message to be send
              vTaskDelayUntil(&xLastWakeTime_t, xHitTime);
          }
      }
      if (ArmCtrlMode == dir_ctrl)
      {
          static BaseType_t* pxHigherPriorityTaskWoken;
          if (GetKeyState(_W_KV)<0) {
              isResume = 1;
              TigerArm.SetTargetx(TigerArm.GetWorldx() + move_step);
              std::cout << "w pressed" << std::endl;
          }
          if (GetKeyState(_S_KV)<0){
              isResume = 1;
              TigerArm.SetTargetx(TigerArm.GetWorldx() - move_step);
              std::cout << "s pressed" << std::endl;
          }
          if (GetKeyState(_A_KV)<0){
              isResume = 1;
              TigerArm.SetTargety(TigerArm.GetWorldy() + move_step);
              std::cout << "a pressed" << std::endl;}
          if (GetKeyState(_D_KV)<0){
              isResume = 1;
              TigerArm.SetTargety(TigerArm.GetWorldy() - move_step);
              std::cout << "d pressed" << std::endl;}
          if (GetKeyState(_R_KV)<0){
              isResume = 1;
              TigerArm.SetTargetz(TigerArm.GetWorldz() + move_step);
              std::cout << "r pressed" << std::endl;}
          if (GetKeyState(_C_KV)<0){
              isResume = 1;
              TigerArm.SetTargetz(TigerArm.GetWorldz() - move_step);
              std::cout << "c pressed" << std::endl;}
          //xTaskNotifyFromISR(TigerArmCtrl_Handle, 1, eSetValueWithOverwrite, pxHigherPriorityTaskWoken);
          if (isResume) {
              isResume = 0;
              vTaskResume(TigerArmCtrl_Handle);
              xTaskNotify(TigerArmCtrl_Handle, 1, eSetValueWithOverwrite);//1should be message to be send
              vTaskDelayUntil(&xLastWakeTime_t, xHitTime);
          }
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
        //cur_target = TigerArm.get_curtarget_deg(xTaskGetTickCount());
        cur_target = TigerArm.get_IK_ans();
        if (TigerArm.NewTarget == ENABLE)
        {
            Joint[Shoulder_yaw]->obj_Target.angle_f = deg2rad(cur_target.deg[0]);
            Joint[Shoulder_pitch]->obj_Target.angle_f = deg2rad(cur_target.deg[1]);
            Joint[Elbow]->obj_Target.angle_f = deg2rad(cur_target.deg[2]);
            Joint[Wrist_roll]->obj_Target.angle_f = deg2rad(cur_target.deg[3]);
            Joint[Wrist_pitch]->obj_Target.angle_f = deg2rad(cur_target.deg[4]);
            Joint[Wrist_yaw]->obj_Target.angle_f = deg2rad(cur_target.deg[5]);
            TigerArm.NewTarget = DISABLE;
        }
        vTaskDelayUntil(&xLastWakeTime_t, xBlockTime);
    }
}

void TigerArm_move()
{
    /*Tw_g(1, 4) = abs(TigerArm.GetTargetx()) < 1e5 ? TigerArm.GetWorldx() : TigerArm.GetTargetx();
    Tw_g(2, 4) = abs(TigerArm.GetTargety()) < 1e5 ? TigerArm.GetWorldy() : TigerArm.GetTargety();
    Tw_g(3, 4) = abs(TigerArm.GetTargetz()) < 1e5 ? TigerArm.GetWorldz() : TigerArm.GetTargetz();
    TigerArm.SetWorldGoal(Tw_g);
    TigerArm.solveT0_6();*/
    TigerArm.IK_cal();
    TigerArm.Set_Cubic_IP_Config(xTaskGetTickCount());
}

void Task_TigerArmCtrl(void* arg)
{
    /* Cache for Task */
    
    /* Pre-Load for task */
    static uint32_t i;
    (void)arg;
    TickType_t xLastWakeTime_t = xTaskGetTickCount();
    //TigerArm_Init();
    //vTaskSuspend(TigerArmCtrl_Handle);
    /* Infinite loop */
    for (;;)
    {
        SysLog.Record(_INFO_, "TigerArm", "Last runtime is %d", xTaskGetTickCount() - xLastWakeTime_t);
        vTaskSuspend(TigerArmCtrl_Handle);
        xLastWakeTime_t = xTaskGetTickCount();
        if (xTaskNotifyWait(0x00000000, 0xFFFFFFFF, &i, 0) == pdTRUE)
        {
            //std::cout << i << std::endl;
            SysLog.Record(_INFO_, "TigerArm", "TigerArm Ctrl Task was woken and set arm to point(%f,%f,%f)...", TigerArm.GetTargetx(), TigerArm.GetTargety(), TigerArm.GetTargetz());
        }
        TigerArm_move();
        
        

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
