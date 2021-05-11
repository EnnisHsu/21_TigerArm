/**
  ******************************************************************************
  * @file   System_DataPool.c
  * @brief  All used resources are contained in this file.
  ******************************************************************************
  * @note
  *  - User can define datas including variables ,structs ,and arrays in
  *    this file, which are used in deffrient tasks or services.
**/
#include "System_DataPool.h"

/* RTOS Resources ------------------------------------------------------------*/
/* Queues */
/* Semaphores */
/* Mutexes */
/* Notifications */

/* Other Resources -----------------------------------------------------------*/
CLogger SysLog;
LogFilter_t Filter_List[3];

_simObjectHandle_Type* Body;
_simObjectHandle_Type* Wheel[WHEEL_NUM];
_simObjectHandle_Type* Joint[ARM_JOINT];
_simObjectHandle_Type* Mineral[MINERAL_NUM];

_simSignalHandle_Type* Signal;
/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/



