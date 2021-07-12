
/*
 *!
 * \file       :kalman.c
 * \brief      : 
 * \version    : 
 * \date       : 2021/04/02
 * \author     : Yueyang Jiang
 *Last modified by Yueyang Jiang 2021/04/02
 *Copyright (c) 2021 by Yueyang Jiang. All Rights Reserved.
 */
   
/*Includes ----------------------------------------------*/
#include "kalman.h"   
/* \defgroup KALMAN_Private_TypesDefinitions
 * \{
 */
   
/**
 * \}
 */
   
/* \defgroup KALMAN_Private_Defines
 * \{
 */
   
/**
 * \}
 */
   
/* \defgroup KALMAN_Private_Macros
 * \{
 */
   
/**
 * \}
 */
   
/* \defgroup KALMAN_Private_Variables
 * \{
 */
   
/**
 * \}
 */
   
/* \defgroup KALMAN_Private_FunctionPrototypes
 * \{
 */
   
/**
 * \}
 */
   
/* \defgroup KALMAN_Private_Functions
 * \{
 */  
void Kalman_Inite(void)
{
	memcpy(&km.P, Ex_data, 16);
	memcpy(&km.Xest, X_est_init_data, 8);
	km.Ez = 1;
	km.Ex = Q;
	km.Inno = 0;
	km.Round = 0;
}

/**
 * \}
 */
   
/* \addtogroup KALMAN_Exported_Functions
 * \{
 */
static void Difference3rd(float* pSrc, uint16_t size)
{ 
    arm_sub_f32(&pSrc[1], &pSrc[0], diff1_Buffer, size+2);
    arm_sub_f32(&diff1_Buffer[1], &diff1_Buffer[0], diff2_Buffer, size+1);
    arm_sub_f32(&diff2_Buffer[1], &diff2_Buffer[0], diff3_Buffer, size);
		arm_abs_f32(diff3_Buffer, diff3_Buffer, size);
		arm_scale_f32(diff3_Buffer, 10000, diff3_Buffer, size);
}

static void Kalman_Update(float Y, float* pX2, float* Res)
{
	float tmp;
	float p0c0;
	float p2c1;
	float p1c0;
	float p3c1;
	float p3c1_p1c0;
	float p0c0_p2c1;
	//X_estimate = A*X_estimate
	
	//P = A*P*A' + Ex
	km.P[0] += km.Ex; 
	km.P[3] += km.Ex;
	
	p0c0 = km.P[0]*km.C[0];
	p2c1 = km.P[2]*km.C[1];
	p1c0 = km.P[1]*km.C[0];
	p3c1 = km.P[3]*km.C[1];
	p3c1_p1c0 = p1c0 + p3c1;
	p0c0_p2c1 = p0c0 + p2c1;
	
	tmp = p0c0_p2c1*km.C[0] + p3c1_p1c0*km.C[1] + km.Ez;
	//Kg = P*C'*inv(C*P*C' + Ez)
	km.kg[0] = (p0c0 + km.P[1]*km.C[1])/tmp;	
	km.kg[1] = (km.P[2]*km.C[0] + p3c1)/tmp;
	
	//r_inno = y - C*X_estimate
	km.Inno = Y - (km.C[0]*km.Xest[0] + km.C[1]*km.Xest[1]);
	
	//X_estimate = X_estimate + Kg*(r_inno)
	km.Xest[0] += km.kg[0]*km.Inno;
	km.Xest[1] += km.kg[1]*km.Inno;
	
	//P = (eye(2)-Kg*C)*P
	km.P[0] -= km.kg[0]*p0c0_p2c1;
	km.P[1] -= km.kg[0]*p3c1_p1c0;
	km.P[2] -= km.kg[1]*p0c0_p2c1;
	km.P[3] -= km.kg[1]*p3c1_p1c0;
	
	//Resualt
	*pX2 = km.Xest[1];
	*Res = km.Inno * km.Inno;
}

void Kalman_Filter(uint16_t size)
{
	for(int i = 0; i < size; i++)
	{
		km.C[0] = C1_data[km.Round];
		km.C[1] = C2_data[km.Round];
		Kalman_Update(Current_Buffer[i], &X2_Buffer[i], &Res_Buffer[i]);
		km.Round++;
		km.Round %= ONE_PERIOD_SAMPLES;
	}
	Difference3rd(X2_Buffer, size-3);
}


/*
 * \}
 */
  
/************************ (C) COPYRIGHT Yueyang Jiang *****END OF FILE****/

