
/*
 *!
 * \file       : data_save.h
 * \brief      : 
 * \version    : 
 * \date       : 2021/03/23
 * \author     : Yueyang Jiang
 * Last modified by Yueyang Jiang 2021/03/23
 * Copyright (c) 2021 by Yueyang Jiang. All Rights Reserved.
 */
#ifndef __DATA_SAVE_H
#define __DATA_SAVE_H
/*Includes ----------------------------------------------*/
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "arm_math.h"
#include "ADC.h" 
#include "fatfs.h"
#include "fatfs_sd.h"
/* \defgroup DATA_SAVE_Exported_TypesDefinitions
 * \{
 */

/**
 * \}
 */
   
/* \defgroup DATA_SAVE_Exported_Defines
 * \{
 */
#define CHAR_BUFFER_SIZE 	4096
#define MAX_VALUE_NUMBER 	500
#define MAX_LINE_SIZE 		30
#define MAX_FILNAME_SIZE 	256
#define MAX_LABEL_SIZE 		1024
/**
 * \}
 */
   
/* \defgroup DATA_SAVE_Exported_Macros
 * \{
 */
   
/**
 * \}
 */
   
/* \defgroup DATA_SAVE_Exported_Variables
 * \{
 */
static FIL fil;
static FATFS fs;
static FATFS *pfs;
static FRESULT fresult;
static UINT bw, br;
static TCHAR fileName[MAX_FILNAME_SIZE] = "/data.txt";
static TCHAR label[MAX_LABEL_SIZE] = "resault\n";
/**
 * \}
 */
   
/* \defgroup DATA_SAVE_Exported_Functions
 * \{
 */
/*conversion of value*/
void Value_float2Char(float *val, char *des);
void twoValues_float2Char(float *pSrc1, float *pSrc2, char *pDst);
void Buffer_float2Char(float *pSrc, char *pDst, uint16_t size);

void Value_halfword2Char(uint16_t *val, char *dst);
void Value_byte2Char(uint8_t *val, char *dst);
void uint2Char(uint32_t n, char *pDst);
void Val_uint322Char(uint32_t n, char *pDst);
void Buffer_uint322Char(uint32_t *pSrc, char *pDst, uint16_t size);
/*buffer operations*/
int bufsize(char* buf);
void clearbuf(char* buf, uint16_t size);
/*File operation*/
void SD_Inite(void);
void SD_Write(char *pStr);
void SD_FileRead(TCHAR *path, TCHAR *pSData, FIL* fil, float32_t* buff, UINT* br, uint16_t size);
/**
 * \}
 */
#endif  
/************************ (C) COPYRIGHT Yueyang Jiang *****END OF FILE****/

