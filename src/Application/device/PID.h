
/**
  ******************************************************************************
  * @file    PID.h
  *
  * @author  Firmware-Team
  *
  * @version xx.xx.xx
  *
  * @date    2016-07-28
  *
  * @brief   PID�㷨���������㷨ͷ�ļ�
  *
  ******************************************************************************
**/
#ifndef _PID_H_
#define _PID_H_

//#include "stm32f3xx.h"

#include  <uccpu.h>
#include "stm32f3xx_hal.h"

/*!  --------------eeprom��ַ---start--------------------*/
#define   PID_PARAM_P_A	          0x31     	 //PID ����1��Aͨ����
#define   PID_PARAM_I_A	          0x35	     //PID ����2��Aͨ����
#define   PID_PARAM_D_A	          0x39	     //PID ����3��Aͨ����
#define   PID_PARAM_P_B	      0x3F	     //PID ����4��Bͨ����
#define   PID_PARAM_I_B	      0x43	     //PID ����5��Bͨ����
#define   PID_PARAM_D_B	      0x47	     //PID ����6��Bͨ����
#define   PID_PARAM_P_C	          0x4D	     //PID ����1��Cͨ����
#define   PID_PARAM_I_C	          0x51	     //PID ����2��Cͨ����
#define   PID_PARAM_D_C	          0x55	     //PID ����3��Cͨ����
#define   PID_PARAM_P_D	      0x5B	     //PID ����4��Dͨ����
#define   PID_PARAM_I_D	      0x5F	     //PID ����5��Dͨ����
#define   PID_PARAM_D_D	      0x63	     //PID ����6��Dͨ����

#define   PIDOUTPUT_THRESHOLD_MAX        0x70	     //PID ������ֵ
#define   PIDOUTPUT_THRESHOLD_MIN	      0x74	     //PID �����Сֵ
#define   PID_INTEGRAL_MAX2	      0x78	     //PID ����������ֵ
#define   PID_INTEGRAL_MIN2	      0x7A	     //PID ���������Сֵ
#define   PID_I_SEPARATION	      0x7C
#define   TARGET_TEMPERATURE2	  0x7D	     //Ŀ���¶�

#define   PIDOUTPUT_THRESHOLD_MAX_B        0x83	     //PID ������ֵ
#define   PIDOUTPUT_THRESHOLD_MIN_B	      0x87	     //PID �����Сֵ
#define   PID_INTEGRAL_MAX2_B	      0x8B	     //PID ����������ֵ
#define   PID_INTEGRAL_MIN2_B	      0x8D	     //PID ���������Сֵ
#define   PID_I_SEPARATION_B	      0x8F
#define   TARGET_TEMPERATURE2_B	  0x90	     //Ŀ���¶�

#define   PIDOUTPUT_THRESHOLD_MAX_C        0x96	     //PID ������ֵ
#define   PIDOUTPUT_THRESHOLD_MIN_C	      0x9A	     //PID �����Сֵ
#define   PID_INTEGRAL_MAX2_C	      0x9E	     //PID ����������ֵ
#define   PID_INTEGRAL_MIN2_C	      0xA0	     //PID ���������Сֵ
#define   PID_I_SEPARATION_C	      0xA2
#define   TARGET_TEMPERATURE2_C	  0xA3	     //Ŀ���¶�

#define   PIDOUTPUT_THRESHOLD_MAX_D        0xA9	     //PID ������ֵ
#define   PIDOUTPUT_THRESHOLD_MIN_D	      0xAD	     //PID �����Сֵ
#define   PID_INTEGRAL_MAX2_D	      0xB1	     //PID ����������ֵ
#define   PID_INTEGRAL_MIN2_D	      0xB3	     //PID ���������Сֵ
#define   PID_I_SEPARATION_D	      0xB5
#define   TARGET_TEMPERATURE2_D	  0xB6	     //Ŀ���¶�





/*!  --------------eeprom��ַ---end--------------------*/


/*! --------------PID ���ڹ�����ʹ�õĲ����궨�� start --------- */
#define MAX_PID_OUTPUT       (220)    //(250)  //PID���ڿ�����������ֵ
#define MIN_PID_OUTPUT       (-120)   //(-150) //PID���ڿ����������Сֵ      

#define MAX_I_OUTPUT         (200)  //(500)
#define MIN_I_OUTPUT         (-200) //(-500)

#define THRESHOLD_TEMP       (2) //(2)   //7 
#define DEFAULT_HOLDER_TARGET_TEMPERATURE   (2500)

/*! --------------PID ���ڹ�����ʹ�õĲ����궨�� end--------- */

__packed
typedef struct
{
  s32 Kp ;
  s32 Ki ;
  s32 Kd ;
} PIDParam;

//PIDParam stPIDParam;

//PID���ڹ����еĽṹ��
__packed
typedef struct
{
  s32 TargetTemp;
  s32 maxOutput;
  s32 minOutput;
  s16 maxIntegralOutput;
  s16 minIntegralOutput;
  s8 threshold;
} PIDCalculate;


/*! from pidDeal.h */
typedef struct
{
  float SetTemperature;
  float err;
  float err_last;
  float Kp, Ki, Kd;
  float PidValue;
  float PidValueLast;
  float integral;
  float umax;
  float umin;
  float threshold;
} PidInfoDef;

extern void PID2_init( PidInfoDef *pPidInfo );
extern void PidParamInit( u8 channelNum, PidInfoDef *pPidInfo );
extern float PID2_realize( float CurrcentTemperature, float targetTemp, PidInfoDef *pPidInfo );
extern PidInfoDef PidInfoA, PidInfoB, PidInfoC, PidInfoD;
/*! from pidDeal.h */

extern PIDCalculate stParamOfPIDCal;

extern u16 SetPIDParameters( u8 channelNum, u8 segmentNum, PIDParam stPIDParam, PidInfoDef *pPidInfo );

/*****************************************************************************
* @name   s32 WriteDefaultPIDParam(PIDCalculate *pPidInfo)
*
* @brief  дPID ������ֻ���ڿ�����������ʼ��ʱʹ�ã�
*
* @param
*    Input   void
*    Output
*
* @return
*
*******************************************************************************/
extern s32 WriteDefaultPIDParam( u8 channelNum, PIDCalculate *pPidInfo, PidInfoDef *pfPidInfo );

/*****************************************************************************
* @name   s32 PIDOutput(s32 currentTemp,u32 targetTemp,PIDParam stPIDParam)
*
* @brief  PID���
*
* @param
*    Input  stPIDParam     PID������ַ
*    Input  TargetTemp     Ŀ���¶�
*
* @return  PID���������
*
*******************************************************************************/
//s32 PIDOutput(s32 currentTemp,u32 targetTemp,PIDParam stPIDParam);
//extern s32 CalPIDOutput(s32 currentTemp,s32 targetTemp,PIDCalculate* stPIDCalculate);


/*****************************************************************************
* @name   s32 GetPIDParam(u8 channelNum, PIDParam Target)
* @brief  ��ȡPID����

*******************************************************************************/
extern u16 GetPIDParam( u8 channelNum, u8 segment, PIDParam *pid, PidInfoDef pPidInfo );

/*****************************************************************************
* @name   PIDParam* SortPIDParam(s32 currentTemp,u32 targetValue)
* @brief  ���յ�ǰ�¶���Ŀ���¶ȵĲ�ֵ����PID��������
*
* @param
*    Input   Target      PID����
*    Output  ��
*
* @return
*
*******************************************************************************/
PIDParam SortPIDParam_A( s32 currentTemp, u32 targetValue );

/*****************************************************************************
* @name   PIDParam* SortPIDParam(s32 currentTemp,u32 targetValue)
* @brief  ���յ�ǰ�¶���Ŀ���¶ȵĲ�ֵ����PID��������
*
* @param
*    Input   Target      PID����
*    Output  ��
*
* @return
*
*******************************************************************************/
PIDParam SortPIDParam_B( s32 currentTemp, u32 targetValue );

u16 SetPidConfigParam( u8 channelNum, PidInfoDef *pfPidInfo, PIDCalculate *pPidInfo );
u16 GetPidConfigParam( u8 channelNum, PidInfoDef *pfPidInfo, PIDCalculate *pPidInfo );
#endif
