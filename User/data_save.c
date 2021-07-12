
/*
 *!
 * \file       :data_save.c
 * \brief      : 
 * \version    : 
 * \date       : 2021/03/23
 * \author     : Yueyang Jiang
 *Last modified by Yueyang Jiang 2021/03/23
 *Copyright (c) 2021 by Yueyang Jiang. All Rights Reserved.
 */
   
/*Includes ----------------------------------------------*/
#include "data_save.h"   
/* \defgroup DATA_SAVE_Private_TypesDefinitions
 * \{
 */
   
/**
 * \}
 */
   
/* \defgroup DATA_SAVE_Private_Defines
 * \{
 */
   
/**
 * \}
 */
   
/* \defgroup DATA_SAVE_Private_Macros
 * \{
 */
   
/**
 * \}
 */
   
/* \defgroup DATA_SAVE_Private_Variables
 * \{
 */
   
/**
 * \}
 */
   
/* \defgroup DATA_SAVE_Private_FunctionPrototypes
 * \{
 */
   
/**
 * \}
 */
   
/* \defgroup DATA_SAVE_Private_Functions
 * \{
 */
   
/**
 * \}
 */
   
/* \addtogroup DATA_SAVE_Exported_Functions
 * \{
 */
 /**************Sd operations**************/
void SD_Inite(void)
{
  br = 0;
	bw = 0;
  fresult = f_mount(&fs, "/", 1);
	fresult = f_open(&fil, fileName, FA_OPEN_ALWAYS | FA_WRITE);
  fresult = f_lseek(&fil, fil.obj.objsize);
  f_puts(label, &fil);
	fresult = f_close(&fil);
}

void SD_Write(char *pStr)
{
  f_open(&fil, fileName, FA_WRITE);
  f_lseek(&fil, fil.obj.objsize);
  f_puts(pStr, &fil);
  f_close(&fil); 
	memset(pStr, 0, CHAR_BUFFER_SIZE);
}

void SD_FileRead(TCHAR *path, TCHAR *pSData, FIL* fil, float32_t* buff, UINT* br, uint16_t size)
{  
  f_open(fil, path, FA_OPEN_ALWAYS | FA_READ); 
  fil->fptr = *br;  
  for(int i = 0; i < size; i++)
  {
    f_lseek(fil, fil->fptr);
    f_gets(pSData, MAX_LINE_SIZE, fil);
    buff[i] = strtod(pSData, NULL);  
  }
  *br = fil->fptr;
  f_close(fil);
}
/**************conversion of data**************/
void Value_float2Char(float *val, char *dst)
{
    char pt[20] = "";
    sprintf(pt, "%.3f", *val);
    strcat(dst, pt);
    strcat(dst, ",\n");
}   

void Value_halfword2Char(uint16_t *val, char *dst)
{
    char pt[20] = "";
    sprintf(pt, "%u", *val);
    strcat(dst, pt);
    strcat(dst, ",\n");
}

void Value_byte2Char(uint8_t *val, char *dst)
{
    char pt[20] = "";
    sprintf(pt, "%u", *val);
    strcat(dst, pt);
    strcat(dst, ",\n");
}

void Buffer_float2Char(float *pSrc, char *pDst, uint16_t size)
{
    char pt[20] = "\0";
    for(int i = 0; i < size; i++)
    {
        sprintf(pt, "%.5f", pSrc[i]);
        strcat(pDst, pt);
        strcat(pDst, ",");
        strcat(pDst, "\n");
    }
} 

void uint2Char(uint32_t n, char *pDst)
{
		int i = 0, j;
    char pt[20] = "\0";
		do{
        pt[i++] = n % 10 + '0';
    } while ((n /= 10)>0);
		i-=1;
    for(j = 0; i >= 0; j++, i--)
    {
        pDst[j] = pt[i];
    }
		pDst[j] = '\0';
}

void Val_uint322Char(uint32_t n, char *pDst)
{
		int i = 0, j;
    char pt[20] = "\0";
		char pt2[20] = "\0";
	
		do{
        pt[i++] = n % 10 + '0';
    } while ((n /= 10)>0);
		i-=1;
    for(j = 0; i >= 0; j++, i--)
    {
        pt2[j] = pt[i];
    }
		pt2[j] = '\0';
		
		strcat(pDst, pt2);
		strcat(pDst, ",");
		strcat(pDst, "\n");
}

void Buffer_uint322Char(uint32_t *pSrc, char *pDst, uint16_t size)
{
	char pt[20] = "\0";
	
	for(int i = 0; i < size; i++)
	{
		if(pSrc[i] != 0)
		{
			uint2Char(pSrc[i], pt);
			strcat(pDst, pt);
			strcat(pDst, ",");
			strcat(pDst, "\n");
		}
		else return;		
	}
}

void twoValues_float2Char(float *pSrc1, float *pSrc2, char *pDst)
{
    char pt[20] = "\0";
    Value_float2Char(pSrc1, pt);
    strcat(pDst, pt);
    Value_float2Char(pSrc2, pt);
    strcat(pDst, pt);
    strcat(pDst, "\n");
}

int bufsize(char* buf)
{
    int i = 0;
    while(*buf++ != '\0') i++;
    return i;
}

void clearbuf(char* buf, uint16_t size)
{
    for(int i = 0; i < size; i++)
    {
        buf[i] = '\0';
    }
}


/**
 * \}
 */
  
/************************ (C) COPYRIGHT Yueyang Jiang *****END OF FILE****/

