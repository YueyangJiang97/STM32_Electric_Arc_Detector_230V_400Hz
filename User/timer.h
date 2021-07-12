#ifndef __TIMER_H
#define __TIMER_H
/*
 *!
 * \file       :timer.h
 * \brief      : 
 * \version    : 
 * \date       : 2021/06/01
 * \author     : Yueyang_Jiang
 *Last modified by Yueyang_Jiang 2021/06/01
 *Copyright (c) 2021 by Yueyang_Jiang. All Rights Reserved.
 */
   
/*Includes ----------------------------------------------*/
#include "stm32h7xx_hal.h"
/* \defgroup TIMER_Private_TypesDefinitions
 * \{
 */
   
/**
 * \}
 */
   
/* \defgroup TIMER_Private_Defines
 * \{
 */
extern volatile uint32_t timerCnt;  
/**
 * \}
 */
   
/* \defgroup TIMER_Private_Macros
 * \{
 */
   
/**
 * \}
 */
   
/* \defgroup TIMER_Private_Variables
 * \{
 */
extern TIM_HandleTypeDef htim4;  
extern uint32_t openTime;
/**
 * \}
 */
   
/* \defgroup TIMER_Private_FunctionPrototypes
 * \{
 */
   
/**
 * \}
 */
   
/* \defgroup TIMER_Private_Functions
 * \{
 */
   
/**
 * \}
 */
   
/* \addtogroup TIMER_Exported_Functions
 * \{
 */
void Timer_Start(void);  
void Timer_Stop(void);
uint32_t Get_SystemTimer(void);
/**
 * \}
 */
#endif  
/************************ (C) COPYRIGHT Yueyang_Jiang *****END OF FILE****/

