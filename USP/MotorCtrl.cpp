/**
  ******************************************************************************
  * Copyright (c) 2019 - ~, SCUT-RobotLab Development Team
  * @file    MotorCtrl.cpp
  * @author  EnnisKoh (8762322@qq.com) & Hehe
  * @brief   Code for resolve of armmodel like puma650
  * @date    2021-05-02
  * @version 0.4.1
  * @par Change Log:
  * <table>
  * <tr><th>Date        <th>Version  <th>Author    				 <th>Description
  * <tr><td>2021-05-01  <td> 0.4.1   <td>EnnisKoh&Hehe     <td>Creator
  * <tr><td>2021-05-02  <td> 0.4.2   <td>EnnisKoh&Hehe     <td>Ìí¼Ó×¢ÊÍ
  * </table>
  *
  ==============================================================================
                            How to use this driver     
  ==============================================================================
    @note 
		
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
#define _MotorCtrl_CPP_
#include "MotorCtrl.h"

#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>

/* instantiation -------------------------------------------------------*/
Motor_GM6020 Motor_Yaw(1),Motor_Wrist(2);
MotorCascadeCtrl<myPID,myPID> Motor_Yaw_Ctrl(&Motor_Yaw),Motor_Wrist_Ctrl(&Motor_Wrist);
AK80_V3 Motor_Shoulder(0x0A,&hcan2),Motor_Elbow(0x01,&hcan2);
MechanicalArm TigerArm;
Matrix Tw_c,T6_g;
Matrix T0_6;
/* declare variable -------------------------------------------------------*/
TaskHandle_t CAN1_TaskHandle,DogMotoCtrl_TaskHandle;
TaskHandle_t JointCtrl_Handle;
TaskHandle_t TigerArmUpdate_Handle;
TaskHandle_t TigerArmCtrl_Handle;
double shoulder_kp=300.0f,shoulder_kd=5.0f,elbow_kp=300.0f,elbow_kd=5.0f;
/* function prototypes -------------------------------------------------------*/
/**
    * @brief  add task
    * @param  None
    * @retval None
    */
void Service_MotoCtrl_Init()
{
	xTaskCreate((TaskFunction_t)Task_CAN1Receive,"CAN1_Receive",Tiny_Stack_Size,NULL,PriorityHigh,&CAN1_TaskHandle);
	xTaskCreate((TaskFunction_t)Task_DogMotorCtrl,"DogMotoCtrl",Tiny_Stack_Size,NULL,PriorityHigh,&DogMotoCtrl_TaskHandle);
	xTaskCreate(Task_TigerArmUpdate, "TigerArm Update", Huge_Stack_Size, NULL, PrioritySuperHigh, &TigerArmUpdate_Handle);
	xTaskCreate(Task_TigerArmCtrl, "TigerArm Control", Huge_Stack_Size, NULL, PrioritySuperHigh, &TigerArmCtrl_Handle);
	xTaskCreate(Task_JointCtrl, "Joint Control", Huge_Stack_Size, NULL, PrioritySuperHigh, &JointCtrl_Handle);
}

void TigerArm_Init(void)
{
    /*Tw_g(1,1) = 1.0f; Tw_g(2,2) = 1.0f;
    Tw_g(3,3) = 1.0f; Tw_g(4,4) = 1.0f;
    T6_g(1, 4) = 1.0f; T6_g(2, 4) = 1.0f;
    T6_g(3, 4) = 1.0f; T6_g(4, 4) = 1.0f;
    TigerArm.Init(Tw_c, T6_g);*/
	Motor_Shoulder.To_Into_Control();
	Motor_Elbow.To_Into_Control();
	Motor_Yaw_Ctrl.setTarget(0.0f);
	Motor_Shoulder.Out_Mixed_Control(0.0f,8.0f,shoulder_kp,shoulder_kd);
	Motor_Elbow.Out_Mixed_Control(0.0f,8.0f,elbow_kp,elbow_kd);

    double a[6] = { 0.0f,0.0f,0.21f,0.0f,0.0f,0.0f };
    double alpha[6] = { 0.0f,90.0f,0.0f,90.0f,-90.0f,-90.0f };
    double d[6] = { 0.0f,0.0f,0.0f,0.12f,0.0f,0.0f };
    
    double interval[6][2] = { {-90.0f,210.0f},{-90.0f,0.0f},{35.0f,120.0f},{-180.0f,180.0f},{-33.0f,90.0f},{-45.0f,45.0f} };
    TigerArm.Set_DHModel_Config(a, d, interval);
}

/**
    * @brief  write target(both 2)
    * @param  deg
    * @retval None
    */
void Yaw_To(double deg)
{
	Motor_Yaw_Ctrl.setTarget(deg);
}

void Wrist_To(double deg)
{
	Motor_Wrist_Ctrl.setTarget(deg);
}

theta_deg_pack pack_cur_deg()
{
	theta_deg_pack pack;
	pack.deg[0]=Motor_Yaw.getAngle();
	pack.deg[1]=Motor_Shoulder.get_current_angle();
	pack.deg[2]=Motor_Elbow.get_current_angle();
	pack.deg[3]=Motor_Wrist.getAngle();
	pack.deg[4]=0.0f;
	pack.deg[5]=0.0f;
	return pack;
}

/**
    * @brief  motor control caculate&send
    * @param  None
    * @retval None
    */
void Task_DogMotorCtrl(void)
{
	TigerArm_Init();
	theta_deg_pack cur_pack;
	TickType_t xLastWakeTime_ArmCtrl;
	
	for (int i=1;i<=4;i++)
	{
		for (int j=1;j<=4;j++)
		{
			if (i==j && i!=4) T0_6(i,j)=1.0f;
				else T0_6(i,j)=0.0f;
			T6_g(i,j)=0.0f;
		}
	}
	TigerArm.Init(Tw_c,T6_g);
	for (;;)
	{
		xLastWakeTime_ArmCtrl = xTaskGetTickCount();
		TigerArm.SetWorldGoal(T0_6);
		cur_pack=pack_cur_deg();
		TigerArm.Set_Cubic_IP_Config(xLastWakeTime_ArmCtrl);
		TigerArm.IK_cal();
		taskYIELD();
	}
}

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
        
        if (TigerArm.NewTarget == ENABLE)
        {
            cur_target = TigerArm.get_curtarget_deg(xLastWakeTime_t);
            //cur_target = TigerArm.get_IK_ans();
			Motor_Yaw_Ctrl.setTarget(cur_target.deg[0]);
			Motor_Shoulder.Out_Mixed_Control(deg2rad(cur_target.deg[1]),8.0f,shoulder_kp,shoulder_kd);
			Motor_Elbow.Out_Mixed_Control(deg2rad(cur_target.deg[2]),8.0f,elbow_kp,elbow_kd);
			Motor_Wrist_Ctrl.setTarget(cur_target.deg[3]);
            if (TigerArm.ReachTargetDeg()) 
                TigerArm.NewTarget = DISABLE;
        }
        vTaskDelayUntil(&xLastWakeTime_t, xBlockTime);
    }
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
        cur_deg.deg[0] = Motor_Yaw.getAngle();
        cur_deg.deg[1] = Motor_Shoulder.get_current_angle();
        cur_deg.deg[2] = Motor_Elbow.get_current_angle();
        cur_deg.deg[3] = Motor_Wrist.getAngle();
        cur_deg.deg[4] = 0.0f;
        cur_deg.deg[5] = 0.0f;
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
    /* Infinite loop */
    for (;;)
    {
        SysLog.Record(_INFO_, "TigerArm", "Last runtime is %d", xTaskGetTickCount() - xLastWakeTime_t);
        vTaskSuspend(TigerArmCtrl_Handle);
        xLastWakeTime_t = xTaskGetTickCount();
        if (xTaskNotifyWait(0x00000000, 0xFFFFFFFF, &i, 0) == pdTRUE)
        {
            SysLog.Record(_INFO_, "TigerArm", "TigerArm Ctrl Task was woken and set arm to point(%f,%f,%f)...", TigerArm.GetTargetx(), TigerArm.GetTargety(), TigerArm.GetTargetz());
        }
        TigerArm_move();
        
        

    }
}



/**
    * @brief  rec chassis motor
    * @param  None
    * @retval None
    */
void Task_CAN1Receive(void)
{
	TickType_t xLastWakeTime_CAN1;
	static CAN_COB Rx_COB;
	for (;;)
	{
		xLastWakeTime_CAN1 = xTaskGetTickCount();
		while (xQueueReceive(RMMotor_QueueHandle,&Rx_COB,0) == pdPASS)
		{
//			if (Motor_Yaw.CheckID(Rx_COB.ID)) Motor_Yaw.update(Rx_COB.Data);
		}
		vTaskDelayUntil(&xLastWakeTime_CAN1,1);
	}
}

/**
    * @brief  rec arm motor
    * @param  None
    * @retval None
    */
void Task_CAN2Receive(void)
{
	TickType_t xLastWakeTime_CAN2;
	static CAN_COB Rx_COB;
	for (;;)
	{
		xLastWakeTime_CAN2 = xTaskGetTickCount();
		while (xQueueReceive(RMMotor_QueueHandle,&Rx_COB,0) == pdPASS)
		{
			if (Motor_Yaw.CheckID(Rx_COB.ID)) Motor_Yaw.update(Rx_COB.Data);
			if (Motor_Shoulder.CheckID(Rx_COB.ID)) Motor_Shoulder.Update(Rx_COB.Data);
			if (Motor_Elbow.CheckID(Rx_COB.ID)) Motor_Elbow.Update(Rx_COB.Data);
			if (Motor_Wrist.CheckID(Rx_COB.ID)) Motor_Wrist.update(Rx_COB.Data);
		}
		vTaskDelayUntil(&xLastWakeTime_CAN2,1);
	}
}

