/**
  ******************************************************************************
  * @file   System_DataPool.h
  * @brief  All used resources are contained in this file.
  ******************************************************************************
  * @note
  *  - User can define datas including variables ,structs ,and arrays in
  *    this file, which are used in deffrient tasks or services.
**/
#ifndef _DATA_POOL_H_
#define _DATA_POOL_H_

/* Includes ------------------------------------------------------------------*/
/* Middlewares & Drivers Support */
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <SRML.h>
#include <CoppeliaSim.h>
/* Macro Definitions ---------------------------------------------------------*/
#define Tiny_Stack_Size       64
#define Small_Stack_Size      128
#define Normal_Stack_Size     256
#define Large_Stack_Size      512
#define Huge_Stack_Size       1024
#define PriorityVeryLow       1
#define PriorityLow           2
#define PriorityBelowNormal   3
#define PriorityNormal        4
#define PriorityAboveNormal   5
#define PriorityHigh          6
#define PrioritySuperHigh     7
#define PriorityRealtime      8

/* RTOS Resources ------------------------------------------------------------*/
/* Queues */
/* Semaphores */
/* Mutexes */
/* Notifications */
/* Other Resources -----------------------------------------------------------*/
#define WHEEL_NUM         4
#define ARM_JOINT       6
extern CLogger SysLog;
extern LogFilter_t Filter_List[2];

extern _simObjectHandle_Type* Body;
extern _simObjectHandle_Type* Wheel[WHEEL_NUM];
extern _simObjectHandle_Type* Joint[ARM_JOINT];
extern _simSignalHandle_Type* Signal;
/*
  @brief Unified communication struct.
*/
typedef struct
{
  uint8_t *pData;
  uint16_t len;
}_Unified_COB;
#endif
/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
