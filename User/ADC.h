#ifndef __ADC_H
#define __ADC_H
/*
 *!
 * \file       : ADC.h
 * \brief      : 
 * \version    : 
 * \date       : 2021/03/17
 * \author     : Yueyang Jiang
 * Last modified by Yueyang Jiang 2021/03/17
 * Copyright (c) 2021 by Yueyang Jiang. All Rights Reserved.
 */
   
/*Includes ----------------------------------------------*/
#include "stm32h7xx_hal.h"
#include "string.h"
#include "main.h"
#include "arm_math.h"
#include "EventRecorder.h"
/* \defgroup ADC_Exported_TypesDefinitions
 * \{
 */
   
/**
 * \}
 */
   
/* \defgroup ADC_Exported_Defines
 * \{
 */
#define ADC_BUFFER_SIZE 250
#define HALF_BUFFER_SIZE ADC_BUFFER_SIZE/2
#define VOLTAGE_RMS_BEGIN 200
#define VOLTAGE_RMS_CUT   100
#define CURRENT_RMS_BEGIN 1
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
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern ADC_HandleTypeDef hadc3; 
extern TIM_HandleTypeDef htim2;

extern uint16_t ADC1_Buffer[ADC_BUFFER_SIZE]; 
extern uint16_t ADC2_Buffer[ADC_BUFFER_SIZE]; 
extern uint16_t ADC3_Buffer[ADC_BUFFER_SIZE];

extern float Current_Buffer[ADC_BUFFER_SIZE];
extern float Voltage_Buffer[ADC_BUFFER_SIZE];
extern float VlotageRMS;
extern float CurrentRMS;

extern uint32_t commandTime;
extern uint32_t openTime;
extern uint32_t openDelay;

extern volatile uint8_t ADCAcquisition_Flag;
extern volatile uint8_t ADCHalfAcquisition_Flag;
extern volatile uint8_t MeasureBegin_Flag;
extern volatile uint8_t ArcDetected_Flag;
extern volatile uint8_t DetectBegin_Flag;

static volatile uint8_t ADC1_CurrentConvCplt_Flag;
static volatile uint8_t ADC2_CurrentConvCplt_Flag;
static volatile uint8_t ADC3_CurrentConvCplt_Flag;

static volatile uint8_t ADC1_CurrentConvHalfCplt_Flag;
static volatile uint8_t ADC2_CurrentConvHalfCplt_Flag;
static volatile uint8_t ADC3_CurrentConvHalfCplt_Flag;
/**
 * \}
 */
   
/* \defgroup ADC_Exported_Functions
 * \{
 */
static void ADC_FlagReset(void);
static void ADC_HalfFlagReset(void);
void ADC_Start(void);
void ADC_Stop(void);
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc);
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc);
void ADC_CurrentConversion(uint16_t size);
uint8_t Mesure_Begin(uint16_t size);
uint8_t OpenConfirm(uint16_t size);
/**
 * \}
 */
 #endif 
/************************ (C) COPYRIGHT Yueyang Jiang *****END OF FILE****/

