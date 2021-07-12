
/*
 *!
 * \file       :fuzzy_logic.c
 * \brief      : 
 * \version    : 
 * \date       : 2021/04/13
 * \author     : Yueyang Jiang
 *Last modified by Yueyang Jiang 2021/04/13
 *Copyright (c) 2021 by Yueyang Jiang. All Rights Reserved.
 */
   
/*Includes ----------------------------------------------*/
#include "fuzzy_logic.h"   
/* \defgroup FUZZY_LOGIC_Private_TypesDefinitions
 * \{
 */
   
/**
 * \}
 */
   
/* \defgroup FUZZY_LOGIC_Private_Defines
 * \{
 */
   
/**
 * \}
 */
   
/* \defgroup FUZZY_LOGIC_Private_Macros
 * \{
 */
   
/**
 * \}
 */
   
/* \defgroup FUZZY_LOGIC_Private_Variables
 * \{
 */
   
/**
 * \}
 */
   
/* \defgroup FUZZY_LOGIC_Private_FunctionPrototypes
 * \{
 */
   
/**
 * \}
 */
   
/* \defgroup FUZZY_LOGIC_Private_Functions
 * \{
 */
/*Simple logic initiation*/
void Simple_FuzzyLogic_init(void)
{
	sl.round = 0;
	sl.majFault[sl.round] = 0;
	sl.minFault[sl.round] = 0;
	sl.threshold = 250;
	sl.SumMajFault = 0;
	sl.SumFault = 0;
	symptomCounter = 0;
	symptomTime = 0;
	pre_symptomTime = 0;
}

/*Update threshold*/
static float Threshold_Update(uint16_t size)
{
	float threshold;
	
	arm_max_no_idx_f32(Res_Buffer, size+3, &residueMax);
	arm_mean_f32(Res_Buffer, size+3, &residueMean);
	
	if(residueMax > 0.15f && residueMean < 0.3f)
	{	
		if(residueMax > 3*residueMean)
		{	
			rapport_threshold = 4.5;
			thresholdMin = THRESHOLD_MINOR_MIN;
			thresholdMax = THRESHOLD_MINOR_MAX;
		}
		else return THRESHOLD_MAX;
	}
	else if(residueMean > 0.3f && residueMean < 10)
	{	
		if(residueMax > 5*residueMean)
		{	
			rapport_threshold = 5;
			thresholdMin = THRESHOLD_MAJOR_MIN;
			thresholdMax = THRESHOLD_MAJOR_MAX;
		}
		else return THRESHOLD_MAX;
	}
	else return THRESHOLD_MAX;
	
	arm_mean_f32(diff3_Buffer, size, &threshold);
	threshold *= rapport_threshold;
	threshold = threshold < thresholdMin ? thresholdMin : threshold;
	threshold = threshold > thresholdMax ? thresholdMax : threshold;
	return threshold;
}

/*Update fault symptome*/
static uint8_t Fault_update(uint16_t size)
{
	uint8_t *pFalt; 
	uint8_t *pPreFalt; 
	uint8_t numMax;
	
	if(sl.threshold >= THRESHOLD_MAJOR)
	{
		pFalt = sl.majFault;
		pPreFalt = &sl.preMajFalt;
		numMax = MAJFAULT_MAX_NUM;
	}
	else
	{
		pFalt = sl.minFault;
		pPreFalt = &sl.preMinFalt;
		numMax = MINFAULT_MAX_NUM;
	}
	
	for(int i = 0; i < size; i++)
	{
		if((diff3_Buffer[i] > sl.threshold) && !(*pPreFalt))
		{
			*pPreFalt = 1;
			pFalt[sl.round]++;
			
			symptomTime = Get_SystemTimer(); //update time of symptome
			Symptom_timespan[symptomCounter] = symptomTime - pre_symptomTime; //timespn between symptome
			pre_symptomTime = symptomTime;
			symptomCounter++;
			symptomCounter %= FAULT_MAX_NUM;
			
			/*if the number of symptomes surpass the number maximum */
			if(pFalt[sl.round] > numMax) 
			{
				HAL_GPIO_WritePin(GPIOG, GPIO_PIN_0, GPIO_PIN_SET);
				commandTime = Get_SystemTimer();
				return 1;
			}
		}
		else *pPreFalt = 0;
	}
	return 0;
}

/*Simple logic decision*/
void Simple_FuzzyLogic(uint16_t size)
{ 
	/*update threshold*/
	sl.threshold = Threshold_Update(size);
	
	/*update symptomTime for the first time*/
	if(!pre_symptomTime) pre_symptomTime = Get_SystemTimer();
	
	/*update fault symptome ArcDetected_Flag = 1 when arc detected*/
	ArcDetected_Flag = Fault_update(size);
	
	/*calculate sum of symptome in 60ms*/
	sl.SumMajFault = 0;
	sl.SumFault = 0;
	for(int i = 0; i < FLROUND_MAX_NUM; i++)
	{
		sl.SumMajFault += sl.majFault[i];	
		sl.SumFault += (sl.minFault[i] + sl.majFault[i]);	
	}
	
	/*return when arc detected*/
	if(ArcDetected_Flag) return;
	
	if(!sl.SumFault)
	{
		pre_symptomTime = Get_SystemTimer();
		symptomCounter = 0;
		memset(Symptom_timespan, 0, FAULT_MAX_NUM);
	}
	else if(sl.SumMajFault > MAJFAULT_MAX_NUM || sl.SumFault > MINFAULT_MAX_NUM)
	{
		HAL_GPIO_WritePin(GPIOG, GPIO_PIN_0, GPIO_PIN_SET);
		commandTime = Get_SystemTimer();
		ArcDetected_Flag = 1;
		return;
	}

	/*if no arc detected update the round counter*/
	sl.round++;
	sl.round %= FLROUND_MAX_NUM;
	sl.majFault[sl.round] = 0;
	sl.minFault[sl.round] = 0;
}
/**
 * \}
 */
   
/* \addtogroup FUZZY_LOGIC_Exported_Functions
 * \{
 */
     
/**
 * \}
 */
  
/************************ (C) COPYRIGHT Yueyang Jiang *****END OF FILE****/

