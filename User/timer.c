
/*
 *!
 * \file       :timer.c
 * \brief      : 
 * \version    : 
 * \date       : 2021/06/01
 * \author     : Yueyang_Jiang
 *Last modified by Yueyang_Jiang 2021/06/01
 *Copyright (c) 2021 by Yueyang_Jiang. All Rights Reserved.
 */
   
/*Includes ----------------------------------------------*/
#include "timer.h" 
/* \defgroup TIMER_Private_TypesDefinitions
 * \{
 */
   
/**
 * \}
 */
   
/* \defgroup TIMER_Private_Defines
 * \{
 */
   
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
void Timer_Start(void)
{
	openTime = 0;
	HAL_TIM_Base_Start_IT(&htim4);
	HAL_TIM_Base_Start(&htim4);
}

void Timer_Stop(void)
{
	HAL_TIM_Base_Stop_IT(&htim4);
	HAL_TIM_Base_Stop(&htim4);
}

uint32_t Get_SystemTimer(void)
{
	return htim4.Instance->CNT + timerCnt*0xffff;
}

/**
 * \}
 */
  
/************************ (C) COPYRIGHT Yueyang_Jiang *****END OF FILE****/

