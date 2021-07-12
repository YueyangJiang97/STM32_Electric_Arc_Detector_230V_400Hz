
/*
 *!
 * \file       : fatfs_sd.h
 * \brief      : 
 * \version    : 
 * \date       : 2021/03/23
 * \author     : Yueyang Jiang
 * Last modified by Yueyang Jiang 2021/03/23
 * Copyright (c) 2021 by Yueyang Jiang. All Rights Reserved.
 */
#ifndef INC_FATFS_SD_H_
#define INC_FATFS_SD_H_

#ifndef __FATFS_SD_H
#define __FATFS_SD_H  
/*Includes ----------------------------------------------*/
#include "integer.h"
#include "diskio.h"   
/* \defgroup FATFS_SD_Exported_TypesDefinitions
 * \{
 */
   
/**
 * \}
 */
   
/* \defgroup FATFS_SD_Exported_Defines
 * \{
 */
#define SPI_TIMEOUT 1000 

/**
 * \}
 */
   
/* \defgroup FATFS_SD_Exported_Macros
 * \{
 */
/* Definitions for MMC/SDC command */
#define CMD0     (0x40+0)     /* GO_IDLE_STATE */
#define CMD1     (0x40+1)     /* SEND_OP_COND */
#define CMD8     (0x40+8)     /* SEND_IF_COND */
#define CMD9     (0x40+9)     /* SEND_CSD */
#define CMD10    (0x40+10)    /* SEND_CID */
#define CMD12    (0x40+12)    /* STOP_TRANSMISSION */
#define CMD16    (0x40+16)    /* SET_BLOCKLEN */
#define CMD17    (0x40+17)    /* READ_SINGLE_BLOCK */
#define CMD18    (0x40+18)    /* READ_MULTIPLE_BLOCK */
#define CMD23    (0x40+23)    /* SET_BLOCK_COUNT */
#define CMD24    (0x40+24)    /* WRITE_BLOCK */
#define CMD25    (0x40+25)    /* WRITE_MULTIPLE_BLOCK */
#define CMD41    (0x40+41)    /* SEND_OP_COND (ACMD) */
#define CMD55    (0x40+55)    /* APP_CMD */
#define CMD58    (0x40+58)    /* READ_OCR */   
/**
 * \}
 */
   
/* \defgroup FATFS_SD_Exported_Variables
 * \{
 */
 
/**
 * \}
 */
   
/* \defgroup FATFS_SD_Exported_Functions
 * \{
 */
DSTATUS SD_disk_initialize (BYTE pdrv);
DSTATUS SD_disk_status (BYTE pdrv);
DRESULT SD_disk_read (BYTE pdrv, BYTE* buff, DWORD sector, UINT count);
DRESULT SD_disk_write (BYTE pdrv, const BYTE* buff, DWORD sector, UINT count);
DRESULT SD_disk_ioctl (BYTE pdrv, BYTE cmd, void* buff);     
/**
 * \}
 */
#endif


#endif /* INC_FATFS_SD_H_ */ 
/************************ (C) COPYRIGHT Yueyang Jiang *****END OF FILE****/












