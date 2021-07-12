
/*
 *!
 * \file       : ADC.c
 * \brief      : 
 * \version    : 
 * \date       : 2021/03/17
 * \author     : Yueyang Jiang
 * Last modified by Yueyang Jiang 2021/03/17
 * Copyright (c) 2021 by Yueyang Jiang. All Rights Reserved.
 */
   
/*Includes ----------------------------------------------*/
#include "ADC.h"   
/* \defgroup ADC_Exported_TypesDefinitions
 * \{
 */
   
/**
 * \}
 */
   
/* \defgroup ADC_Exported_Defines
 * \{
 */
   
/**
 * \}
 */
   
/* \defgroup ADC_Exported_Macros
 * \{
 */
   
/**
 * \}
 */
   
/* \defgroup ADC_Exported_Variables
 * \{
 */
   
/**
 * \}
 */
   
/* \defgroup ADC_Exported_Functions
 * \{
 */
static void ADC_FlagReset(void)
{
		ADC1_CurrentConvCplt_Flag = 0;
		ADC2_CurrentConvCplt_Flag = 0;
		ADC3_CurrentConvCplt_Flag = 0;
		ADCAcquisition_Flag = 0;
}

static void ADC_HalfFlagReset(void)
{
		ADC1_CurrentConvHalfCplt_Flag = 0;
		ADC2_CurrentConvHalfCplt_Flag = 0;
		ADC3_CurrentConvHalfCplt_Flag = 0;
		ADCHalfAcquisition_Flag = 0;
}

void ADC_Start(void) 
{
		ADC_FlagReset();	
		HAL_TIM_Base_Start(&htim2);
    HAL_ADC_Start_DMA(&hadc1, (uint32_t* )ADC1_Buffer, ADC_BUFFER_SIZE); 
		HAL_ADC_Start_DMA(&hadc2, (uint32_t* )ADC2_Buffer, ADC_BUFFER_SIZE); 
		HAL_ADC_Start_DMA(&hadc3, (uint32_t* )ADC3_Buffer, ADC_BUFFER_SIZE); 
}    

void ADC_Stop(void)
{
		HAL_TIM_Base_Stop(&htim2);
    HAL_ADC_Stop_DMA(&hadc1);
		HAL_ADC_Stop_DMA(&hadc2);
		HAL_ADC_Stop_DMA(&hadc3);		
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  /* Prevent unused argument(s) compilation warning */
	if(hadc == &hadc1) ADC1_CurrentConvCplt_Flag = 1;
	if(hadc == &hadc2) ADC2_CurrentConvCplt_Flag = 1;
	if(hadc == &hadc3) ADC3_CurrentConvCplt_Flag = 1;
	if(ADC1_CurrentConvCplt_Flag && ADC2_CurrentConvCplt_Flag && ADC3_CurrentConvCplt_Flag) ADCAcquisition_Flag = 1;
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
	if(hadc == &hadc1) ADC1_CurrentConvHalfCplt_Flag = 1;
	if(hadc == &hadc2) ADC2_CurrentConvHalfCplt_Flag = 1;
	if(hadc == &hadc3) ADC3_CurrentConvHalfCplt_Flag = 1;
	if(ADC1_CurrentConvHalfCplt_Flag && ADC2_CurrentConvHalfCplt_Flag && ADC3_CurrentConvHalfCplt_Flag) ADCHalfAcquisition_Flag = 1;
}

void ADC_CurrentConversion(uint16_t size)
{
	uint16_t startbit;
	if(ADCAcquisition_Flag) 
	{
		startbit = HALF_BUFFER_SIZE;
		ADC_FlagReset();
	}
	else if(ADCHalfAcquisition_Flag) 
	{
		startbit = 0;
		ADC_HalfFlagReset();
	}
	
//	if(!ArcDetected_Flag) 
//	{
//		
//		arm_scale_f32((float*)ADC3_Buffer, 0.00005, (float*)Current_Buffer, size);
//		arm_sub_f32((float*)ADC3_Buffer, (float*)ADC2_Buffer, Current_Buffer, size);
//	}
	
	for(int i = 0; i < size; i++)
	{
		if(!ArcDetected_Flag) 
		{
			Current_Buffer[i] = (ADC3_Buffer[i+startbit]-ADC2_Buffer[i+startbit])*0.00005f; /*0.0003 0.00005*/
		}
		if(!MeasureBegin_Flag || ArcDetected_Flag)
		{
			Voltage_Buffer[i] = (ADC1_Buffer[i+startbit]-ADC2_Buffer[i+startbit])*0.0375f;
		}
	}
}


uint8_t Mesure_Begin(uint16_t size)
{
	arm_rms_f32(Voltage_Buffer, size, &VlotageRMS);
	arm_rms_f32(Current_Buffer, size, &CurrentRMS);
	if(VlotageRMS > VOLTAGE_RMS_BEGIN /*&& CurrentRMS > CURRENT_RMS_BEGIN*/) 
	{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_RESET);
		return 1;
	}
	return 0;
}

uint8_t OpenConfirm(uint16_t size)
{
	arm_rms_f32(Voltage_Buffer, size, &VlotageRMS);
	if(VlotageRMS < VOLTAGE_RMS_CUT) /*VoltageRMS < 100V means circuit is opened*/
	{
		openTime = Get_SystemTimer();
		openDelay = openTime - commandTime;
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
		return 1;
	}
	return 0;
}
/**
 * \}
 */
  
/************************ (C) COPYRIGHT Yueyang Jiang *****END OF FILE****/

