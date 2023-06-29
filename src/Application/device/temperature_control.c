
/***********************************************************
Module Name: temperature_control.c
Description: 温度控制模块。实现升温，降温功能
Module Date: 27-07-2016
Module Author: Firmware-Team
Others:
***********************************************************/

/*
*********************************************************************************************************
*                                             INCLUDE FILES
*********************************************************************************************************
*/
#include "temperature_control.h"
#include "includes.h"
#include "stm32f3xx.h"
#include "PID.h"
#include "AD7124.h"
#include "errcode.h"
#include "gpio.h"
#include "AD7124_temp.h"
#include <stdlib.h>
#include  "protocolAnalysis_app.h"
//#include "pidDeal.h"
/*
*********************************************************************************************************
*                                            LOCAL DEFINES
*********************************************************************************************************
*/

//typedef enum
//{
//	CHA = 0,           //A通道
//	CHB,               //B通道
//	INVALIDCHNUM,      //无效通道
//}channelNum;

#define  TARGET_TEMP_MAX		(11000ul)
#define  TARGET_TEMP_MIN    (100ul)
#define  LIMIT_WARNING_T		(200ul)
#define  LIMIT_WARNING_T_2	(100ul)

#define  MAX_ERROR_T        (10000ul)
#define  MIN_ERROR_T        (100u)
#define  MAX_CHANNEL_NUM    (4ul)
#define  MIN_CHANNEL_NUM    (1ul)

#define  BRI_TARGET_TEMP_MAX    (1500ul)
#define  BRI_TARGET_TEMP_MIN    (0ul)

#define  BRI_MAX_ERROR_T        (800)
#define  BRI_MIN_ERROR_T        (200)

//#define abs(x)   (x) >=0 ? (x) :(-(x))

#define  TEST_TIME           (2000)
#define  TEST_TARGET_60      (6000)
#define  TEST_TARGET_20      (2000)
#define  TEST_TARGET_40      (4000)
#define  TEST_TARGET_55      (5500)
#define  TEST_TARGET_57      (5700)
#define  TEST_TARGET_4       (400)
#define  TEST_TARGET_25      (2500)


#define ERROR_MAX_TARGET					(0x10)
#define ERROR_MIN_TARGET					(0x14)

#define ERROR_MAX_TARGET_B				(0x110)
#define ERROR_MIN_TARGET_B				(0x114)

#define ERROR_MAX_TARGET_C				(0x124)
#define ERROR_MIN_TARGET_C				(0x128)

#define ERROR_MAX_TARGET_D				(0x138)
#define ERROR_MIN_TARGET_D				(0x13C)

#define  WARNING_LIMIT						(0x20)
#define  ERROR_LIMIT							(0x22)
#define  HOT_NUM									(0x100)
#define  COOL_NUM									(0x104)
#define  CYCLE_NUM								(0x108)

#define    CAL_ENABLE							0x150
#define    TARGET_TEMP0						0x151
#define    CAL_TEMP0							0x155
#define    TARGET_TEMP1						0x159
#define    CAL_TEMP1							0x15D
#define    TARGET_TEMP2						0x161
#define    CAL_TEMP2							0x165
#define    TARGET_TEMP3						0x169
#define    CAL_TEMP3							0x16D
#define    TARGET_TEMP4						0x171
#define    CAL_TEMP4							0x175
#define    TARGET_TEMP5						0x179
#define    CAL_TEMP5							0x17D
#define    TARGET_TEMP6						0x181
#define    CAL_TEMP6							0x185
#define    TARGET_TEMP7						0x189
#define    CAL_TEMP7							0x18D
#define    TARGET_TEMP8						0x191
#define    CAL_TEMP8							0x195
#define    TARGET_TEMP9						0x199
#define    CAL_TEMP9							0x19D
#define    CAL_CHECKSUM						0x1A1

#define    BLACK_BOARD_FLAG				0x0C
#define    CHECKSUM								0x0D


#define    MAX_CURRENT_T					0xF0
#define    MIN_CURRENT_T					0xF4

#define    MAX_CURRENT_T_B	    	0x11A
#define    MIN_CURRENT_T_B				0x11E

#define    MAX_CURRENT_T_C	    	0x12e
#define    MIN_CURRENT_T_C				0x132

#define    MAX_CURRENT_T_D	    	0x142
#define    MIN_CURRENT_T_D				0x146
#define		 NEW_BOARD							0x00
#define    OLD_BOARD							0x01

#define  TDATA_1         				  (100)
#define  TDATA_4        				  (400)


#define TEC_STOP_MAX_TIME     180 //180s
#define TEC_STOP_MIN_TIME     60  //60s
#define TEC_STOP_ERROR_TIME   0xFFFF

calTarTempParam stCalTarTempParam;
tempControl stTemperatureControlA, stTemperatureControlB, stTemperatureControlC, stTemperatureControlD;

typedef struct
{
  u32 hotNum;
  u32 coolNum;
  u32 cycle;
} calcTECWorkNum;
calcTECWorkNum stTECWorkCount;

//static u8 channelEnableFlag_A = 0;   // 0:不使能；  1：使能
//static u8 channelEnableFlag_B = 1;   // 0:不使能；  1：使能
//static u8 channelEnableFlag_C = 0;   // 0:不使能；  1：使能
//static u8 channelEnableFlag_D = 0;   // 0:不使能；  1：使能
//static s32 targetTemperature_A = TEST_TARGET_20,targetTemperature_B=TEST_TARGET_4,targetTemperature_C=TEST_TARGET_4,targetTemperature_D = TEST_TARGET_20 ;        //目标温度


typedef struct
{
  bool enableFlag;
  s32 targTemperature;
} controlFlag;

__packed
typedef struct
{
  u8 blankBoardFlag;
  u16 CheckSum;
} blankBoard;

u16 activeReportFrameIndex = 0x00;
/*
*********************************************************************************************************
*                                      LOCAL FUNCTION PROTOTYPES
*********************************************************************************************************
*/


static s32 DutyCycle( u8 channelNum, s32 PIDOutput1 );

s32 Heating( u8 channelNum, bool ableFlag, u8 percent );
s32 Cool( u8 channelNum, bool ableFlag, u8 percent );
s32 StopTec( u8 channelNum, bool stopFlag );
static s32 VaryingTempControl( u8 channelNum, s32 currentTemp, s32 targetValue, tempControl *stTempControl, PidInfoDef *pPidInfo );
static s32 ShockTempControl( u8 channelNum, s32 currentTemp, s32 targetValue, tempControl *stTempControl, PidInfoDef *pPidInfo );
static s32 SteadyControl( u8 channelNum, s32 currentTemp, s32 targetValue, tempControl *stTempControl, PidInfoDef *pPidInfo );
void WriteDefaultParam( u8 channelNum, tempControl *stTempControl );




/*******************************************************************************************
* @Name: s32 PIDRegulator(s32 currentTemp,u32 targetValue,u8 channelNum)
*
* @brief:      PID??
*
* @param  :
     Inputs:  ?????
     Outputs: PID??????(PWM?)

* @return   True ??   False ??
*
**********************************************************************************/
s32 PIDRegulator( s32 currentTemp, s32 targetValue, u8 channelNum, PidInfoDef *pPidInfo )
{
  s32 ret = SUCCESS1;
  s32 PIDTempValue;

  PIDTempValue = ( s32 )PID2_realize( ( float )currentTemp/100.0f, ( float )targetValue/100.0f, pPidInfo );
  DutyCycle( channelNum, PIDTempValue );

  return  ret;
}

u16 GetCurrentTemperature( u8 channelNum, s32* pData )
{
  u16 ret = SUCCESS1;
  s32 TmpData[DIFF_CHN_MAX][10];
  s32 result = 0;
  if( ( MAX_CHANNEL_NUM >= channelNum ) && ( channelNum >=MIN_CHANNEL_NUM ) )
  {
    result = TEMP_GetTempData( channelNum-1, 1, TmpData[channelNum-1] );

    if( result == AD7124_OK )
    {
      *pData = TmpData[channelNum -1][0];
    }
    else
    {
      EepromSaveLog( 0x03, "get temp err!!" );
      ret = ERR_ADC_ABNORMAL;
    }
  }
  else
  {
    ret = ERR_CHANNEL_NUM;
  }

  return ret;

}


/**************************************************************************************
** 函数名称: GetSensorValue
** 参数    : channelNum：       pData：				stTempControl
** 函数功能: 获取传感器的数据
** 返回值  : u16(ret)：
**************************************************************************************/
u16 GetSensorValue( u8 channelNum, s32* pData, tempControl stTempControl )
{
  u16 ret = SUCCESS1;

  ret = GetCurrentTemperature( channelNum, pData );

  if( SUCCESS1 == ret )
  {
    if( ( *pData  > stTempControl.maxErrorT ) || ( *pData < stTempControl.minErrorT ) )
    {
      ret=( *pData > stTempControl.maxErrorT ) ?  ERR_TEMP_OVER_W_UPPER_LIMIT : ERR_TEMP_OVER_W_LOWER_LIMIT;
      EepromSaveLog( 0x03, "get temp err!!" );
    }
  }

  return ret;
}

/**************************************************************************************
** 函数名称: CalibrationTargetTemperature
** 参数    : stTemp
** 函数功能:
** 返回值  : u16(ret)：
**************************************************************************************/
u16 CalibrationTargetTemperature( calTarTempParam *stTemp )
{
  u16 ret = SUCCESS1;
  EepromStatusDef EepromStatus;

  if( stTemp->enableFlag >1 )
    return ERR_PARAM_ABNOMAL;
  else
  {
    EepromStatus = EepromWriteProgramter( CAL_ENABLE, ( u8* )stTemp, sizeof( calTarTempParam ) );

    if( EEPROM_PROGRAM_WRITE_OK == EepromStatus )
    {

      memcpy( &stCalTarTempParam, ( u8* )stTemp, sizeof( calTarTempParam ) );

      return ret;
    }
    else
    {
      return ERR_DRIVER_EEPROM;

    }
  }
}

s32 PlatformTempCal( u8 channelNum, s32* PT100Param, s32* boradParam, s32* platformParam )
{

  return true;
}

static s32 DutyCycle( u8 channelNum, s32 PIDOutput1 )
{
  u8 Duty;

  if( PIDOutput1 > 0 )
  {

    if( PIDOutput1 >= MAX_PID_OUTPUT )
    {
      Duty = 100;

    }
    else
    {
      Duty = abs( ( PIDOutput1*100 )/MAX_PID_OUTPUT );

    }
    //Debug_Printf("HEAT==> +PIDOutput1:%d  Duty:%d ", PIDOutput1,Duty);
    Heating( channelNum, true, Duty );

  }
  else
  {
    if( PIDOutput1 <= MIN_PID_OUTPUT )
    {
      Duty = 100;

    }
    else
    {
      Duty = abs( ( PIDOutput1*100 )/MIN_PID_OUTPUT );

    }
    //Debug_Printf("COOL==> -PIDOutput1:%d  Duty:%d ", PIDOutput1,Duty);
    Cool( channelNum, true, Duty );


  }

  return SUCCESS1;
}

/**************************************************************************************
** 函数名称: Heating
** 参数    : channelNum：TEC通道   ableFlag：是否加热标志   percent：参数
** 函数功能: 加热
** 返回值  :
**************************************************************************************/
#if 0
s32 Heating( u8 channelNum, bool ableFlag, u8 percent )
{
  s32 ret = SUCCESS1;

  if( ( MAX_CHANNEL_NUM < channelNum ) || ( channelNum < MIN_CHANNEL_NUM ) )
  {

    SendAbnormalData( ERR_CHANNEL_NUM, &protocolAnalysisHandle, &stActiveRepResult, &activeReportFrameIndex, 0 );
    return SUCCESS1;
  }

  if( CHA == channelNum )
  {
    TecControl( IN_1_A, false, 0 );
    TecControl( IN_2_A, ableFlag, percent );

    TecControl( IN_1_B, false, 0 );
    TecControl( IN_2_B, ableFlag, percent );
  }
  else if( CHB == channelNum )
  {
    //		TecControl(IN_1_B,false,0);
    //		TecControl(IN_2_B,ableFlag,percent);
  }
  else if( CHC == channelNum )
  {
    TecControl( IN_1_C, false, 0 );
    TecControl( IN_2_C, ableFlag, percent );
    TecControl( IN_1_D, false, 0 );
    TecControl( IN_2_D, ableFlag, percent );

  }
  else
  {
    //		TecControl(IN_1_D,false,0);
    //		TecControl(IN_2_D,ableFlag,percent);
  }

  return ret;
}
#endif

s32 Heating( u8 channelNum, bool ableFlag, u8 percent )
{
  s32 ret = SUCCESS1;

  if( ( MAX_CHANNEL_NUM < channelNum ) || ( channelNum < MIN_CHANNEL_NUM ) )
  {

    SendAbnormalData( ERR_CHANNEL_NUM, &protocolAnalysisHandle, &stActiveRepResult, &activeReportFrameIndex, 0 );
    return SUCCESS1;
  }

  if( CHA == channelNum )
  {
    LiquidFanContrl( CHA, true ); //开启风扇
    LiquidFanContrl( CHB, true ); //开启风扇

    TecControl( IN_1_A, false, 0 );
    TecControl( IN_2_A, ableFlag, percent );
    TecControl( IN_1_B, false, 0 );
    TecControl( IN_2_B, ableFlag, percent );
  }
  else if( CHB == channelNum )
  {
    //LiquidFanContrl(CHB,true); //开启风扇

    //TecControl(IN_1_B,false,0);
    //TecControl(IN_2_B,ableFlag,percent);
  }
  else if( CHC == channelNum )
  {
    LiquidFanContrl( CHC, true ); //开启风扇
    LiquidFanContrl( CHD, true ); //开启风扇

    TecControl( IN_1_C, false, 0 );
    TecControl( IN_2_C, ableFlag, percent );
    TecControl( IN_1_D, false, 0 );
    TecControl( IN_2_D, ableFlag, percent );
  }
  else
  {
    //LiquidFanContrl(CHD,true); //开启风扇

    //TecControl( IN_1_D, false, 0 );
    //TecControl( IN_2_D, ableFlag, percent );
  }

  return ret;
}

/*****************************************************************************
* @name   s32 StopHeating(void)
* @brief  停止加热
*
* @param
*    Input   value       PID调节后的输出
*    Output  无
*
* @return
*
*******************************************************************************/
/**************************************************************************************
** 函数名称: StopTec
** 参数    : channelNum：TEC通道   ableFlag：是否加热标志   percent：参数
** 函数功能: 停止加热
** 返回值  :
**************************************************************************************/
s32 StopTec( u8 channelNum, bool stopFlag )
{
  s32 ret = SUCCESS1;

  if( ( MAX_CHANNEL_NUM < channelNum ) || ( channelNum < MIN_CHANNEL_NUM ) )
  {
    //上报通道号异常
    SendAbnormalData( ERR_CHANNEL_NUM, &protocolAnalysisHandle, &stActiveRepResult, &activeReportFrameIndex, 0 );
    return SUCCESS1;
  }

  if( CHA == channelNum )
  {
    TecControl( IN_2_A, false, stopFlag );
    TecControl( IN_1_A, false, stopFlag );
    TecControl( IN_2_B, false, stopFlag );
    TecControl( IN_1_B, false, stopFlag );
  }
  else if( CHB == channelNum )
  {
    //		TecControl(IN_2_B,false,stopFlag);
    //		TecControl(IN_1_B,false,stopFlag);
  }
  else if( CHC == channelNum )
  {
    TecControl( IN_2_C, false, stopFlag );
    TecControl( IN_1_C, false, stopFlag );
    TecControl( IN_2_D, false, stopFlag );
    TecControl( IN_1_D, false, stopFlag );
  }
  else
  {
    //TecControl(IN_2_D,false,stopFlag);
    //TecControl(IN_1_D,false,stopFlag);
  }

  return ret;

}


/*****************************************************************************
* @name   s32 Cool(u8 channelNum,bool ableFlag,u8 percent)
* @brief  制冷函数
*
* @param
*    Input   channelNum  通道号
*    Input   value       PID调节后的输出
*    Output  无
*
* @return
*
*******************************************************************************/
s32 Cool( u8 channelNum, bool ableFlag, u8 percent )
{
  s32 ret = SUCCESS1;

  if( ( MAX_CHANNEL_NUM < channelNum ) || ( channelNum < MIN_CHANNEL_NUM ) )
  {
    //上报通道号异常

    SendAbnormalData( ERR_CHANNEL_NUM, &protocolAnalysisHandle, &stActiveRepResult, &activeReportFrameIndex, 0 );
    return SUCCESS1;
  }

  if( CHA == channelNum )
  {
    LiquidFanContrl( CHA, true ); //开启风扇
    LiquidFanContrl( CHB, true ); //开启风扇

    TecControl( IN_2_A, false, 0 );
    TecControl( IN_1_A, ableFlag, percent );

    TecControl( IN_2_B, false, 0 );
    TecControl( IN_1_B, ableFlag, percent );
    /*! 开启制冷泵 */
    //		HAL_GPIO_WritePin(GPIOB,PUMP_A, GPIO_PIN_RESET);

  }
  else if( CHB == channelNum )
  {
    //LiquidFanContrl(CHB,true); //开启风扇

    //		TecControl(IN_2_B,false,0);
    //		TecControl(IN_1_B,ableFlag,percent);
    //		HAL_GPIO_WritePin(GPIOB,PUMP_B, GPIO_PIN_SET);
  }
  else if( CHC == channelNum )
  {
    LiquidFanContrl( CHC, true ); //开启风扇
    LiquidFanContrl( CHD, true ); //开启风扇

    //		HAL_GPIO_WritePin(GPIOA,PUMP_C, GPIO_PIN_RESET);
    TecControl( IN_2_C, false, 0 );
    TecControl( IN_1_C, ableFlag, percent );
    TecControl( IN_2_D, false, 0 );
    TecControl( IN_1_D, ableFlag, percent );
  }
  else
  {
    //LiquidFanContrl(CHD,true); //开启风扇

    //		TecControl(IN_2_D,false,0);
    //		TecControl(IN_1_D,ableFlag,percent);
    //		HAL_GPIO_WritePin(GPIOB,PUMP_D, GPIO_PIN_RESET);
  }

  return ret;

}


/*****************************************************************************
* @name   bool StopAllOfTec(void)
* @brief  停止所有的TEC
*
* @param
*    Input   无
*    Output  无
*
* @return true：成功  false：失败
*
*******************************************************************************/
s32 StopAllOfTec( void )
{
  //	s32 ret = true;

  TecControl( IN_1_A, false, false );
  TecControl( IN_2_A, false, false );

  TecControl( IN_1_B, false, false );
  TecControl( IN_2_B, false, false );

  TecControl( IN_1_C, false, false );
  TecControl( IN_2_C, false, false );


  TecControl( IN_1_D, false, false );
  TecControl( IN_2_D, false, false );

  return true;
}

/*****************************************************************************
* @name   s32 SetChannelEnableFlag(u8 channelNum ,bool value, bool* channelEnableFlag)
* @brief  设置通道使能标示
*
* @param
*    Input   channelNum  通道号
             value       False：不使能  True：使能
             channelEnableFlag  通道使能（不使能）标示
*    Output
*
* @return
*
*******************************************************************************/
u16 SetChannelEnableFlag( u8 channelNum, u8 channelEnableFlag, tempControl *stTempControl )
{
  u16 ret = SUCCESS1;

  if( ( MAX_CHANNEL_NUM < channelNum ) || ( channelNum < MIN_CHANNEL_NUM ) )
    return ERR_CHANNEL_NUM;

  if( channelEnableFlag> 1 )
  {
    /*! 参数异常 */
    return ERR_PARAM_ABNOMAL;
  }

  stTempControl->channelEnableFlag = channelEnableFlag;
  if( 0 == stTempControl->channelEnableFlag )
  {
    StopTec( channelNum, false );

    resetTCParamer( stTempControl );

  }
  return ret;

}

/*****************************************************************************
* @name   s32 GetChannelEnableFlag(u8 channelNum , u8 *pFlag)
* @brief  获取通道使能标示
*
* @param
*    Input   channelNum  通道号
             value       False：不使能  True：使能
             channelEnableFlag  通道使能（不使能）标示
*    Output
*
* @return
*
*******************************************************************************/
u16 GetChannelEnableFlag( u8 channelNum, u8 *pFlag, tempControl *stTempControl )
{
  u16 ret =SUCCESS1;

  if( ( MAX_CHANNEL_NUM < channelNum ) || ( channelNum < MIN_CHANNEL_NUM ) )
    return ERR_CHANNEL_NUM;

  *pFlag = stTempControl->channelEnableFlag;

  return ret;

}


/*****************************************************************************
* @name   s32 SetTargetTemperature(u8 channelNum,u32 target,s32* pTarget )
* @brief  设置目标温度
*
* @param
*    Input   channelNum   通道号
             target       目标温度
             pTarget      存储目标温度的指针

*    Output  无
*
* @return
*
*******************************************************************************/
//u16 SetTargetTemperature(u8 channelNum,s32 target)
u16 SetTargetTemperature( u8 channelNum, s32 target, tempControl *stTempControl, PidInfoDef *pPidInfo )
{
  u16 ret = SUCCESS1;
  u8 i = 0, j=0;
  float tempData;

  tempData = pPidInfo->SetTemperature;

  /*! 参数判断 */
  if( ( MAX_CHANNEL_NUM < channelNum ) || ( channelNum < MIN_CHANNEL_NUM ) )
    return ERR_CHANNEL_NUM;

  if( target > stTempControl->targetTempMax )
  {
    return ERR_TARGET_TEMP_OVER_UPPER_LIMIT;
  }
  else if( target < stTempControl->targetTempMin )
  {
    return ERR_TARGET_TEMP_OVER_LOWER_LIMIT;
  }


  if( stCalTarTempParam.enableFlag == true )
  {

    for( i= 0; i<MAX_CALI_NUM; i++ )
    {
      if( target == stCalTarTempParam.calData[i][0] )
      {
        pPidInfo->SetTemperature = ( float )stCalTarTempParam.calData[i][1]/100;
        j = 0;
      }
      else
      {
        j++;
      }
    }
    if( j > 0 )
    {

      pPidInfo->SetTemperature = ( float )target/100;
    }
  }
  else
  {
    pPidInfo->SetTemperature = ( float )target/100;
  }

  if( ( tempData >= pPidInfo->SetTemperature +0.5f ) ||( tempData <= pPidInfo->SetTemperature -0.5f )  || ( stTempControl->firstSetTargetFlag == false ) )
  {
    stTempControl->stTempProcessFlag.sCurTempControlFlag = true;

    stTempControl->stTempProcessFlag.sCurTempShockFlag = 0;
    stTempControl->stTempProcessFlag.sCurTempSteadyFlag = 0;
    stTempControl->stTempCount.changeStageCount = 0;
    stTempControl->stTempCount.changeStageExcepCount =0;
    stTempControl->stTempCount.shockCount = 0;
    stTempControl->firstSetTargetFlag = true;
  }
  else
  {

  }

  return ret;
}


/*****************************************************************************
* @name   u16 GetTargetTemperature(u8 channelNum,s32 *targTemp)
* @brief  获取目标温度
*
* @param
*    Input   channelNum

*    Output  Target      目标温度
*
* @return
*
*******************************************************************************/
u16 GetTargetTemperature_test( u8 channelNum, s32 *targTemp, PidInfoDef *pPidInfo )
{
  u16 ret =SUCCESS1;
  s32 target;

  if( ( MAX_CHANNEL_NUM < channelNum ) || ( channelNum < MIN_CHANNEL_NUM ) )
    return ERR_CHANNEL_NUM;

  target = ( s32 )( pPidInfo->SetTemperature	* 100 );
  *targTemp = target;

  return ret;
}

u16 GetTargetTemperature( u8 channelNum, s32 *targTemp, PidInfoDef pPidInfo )
{
  u16 ret =SUCCESS1;
  s32 target = 0;

  if( ( MAX_CHANNEL_NUM < channelNum ) || ( channelNum < MIN_CHANNEL_NUM ) )
	{
    return ERR_CHANNEL_NUM;
	}
	
  target = ( s32 )( pPidInfo.SetTemperature * 100 );
	
  *targTemp = target ;

  return ret;
}

/*****************************************************************************
* @name   s32 SetTargetTempRange(u8 channelNum,s32 upperLimit,s32 lowerLimit,tempControl *stTempControl)
* @brief  设置目标温度范围
*
* @param
*    Input   channelNum  通道号
             upperLimit  上限（最大值）
             lowerLimit  下限（最小值）
             tempControl *stTempControl
*    Output  eeprom的读取状态
*
* @return
*
*******************************************************************************/
u16 SetTargetTempRange( u8 channelNum, s32 upperLimit, s32 lowerLimit, tempControl *stTempControl )
{
  u16 ret = SUCCESS1;
  EepromStatusDef EepromStatus;
  //    temperatureAbnormalParam stTemp;
  targetTemperatureErrParam stTargetRange;
  u32 EepromReadAddr;

  /*! 通道号判断 */
  if( ( MAX_CHANNEL_NUM < channelNum ) || ( channelNum < MIN_CHANNEL_NUM ) )
    return ERR_CHANNEL_NUM;


  stTargetRange.targetMaxValue = upperLimit;
  stTargetRange.targetMinValue = lowerLimit;

#if 0
  if( ( CHD == channelNum ) || ( CHA == channelNum ) )
  {

    EepromStatus = EepromWriteProgramter( ERROR_MAX_TARGET, ( u8* )&stTargetRange, sizeof( stTargetRange ) );
  }
  else
  {
    EepromStatus = EepromWriteProgramter( ERROR_MAX_TARGET_B, ( u8* )&stTargetRange, sizeof( stTargetRange ) );
  }
#endif
  switch( channelNum )
  {
    case CHA:
    {
      EepromReadAddr = ERROR_MAX_TARGET;
    }
    break;

    case CHB:
    {
      EepromReadAddr = ERROR_MAX_TARGET_B;
    }
    break;

    case CHC:
    {
      EepromReadAddr = ERROR_MAX_TARGET_C;
    }
    break;

    case CHD:
    {
      EepromReadAddr = ERROR_MAX_TARGET_D;
    }
    break;

    default:
      //			ret = ERR_CHANNEL_NUM;
      break;
  }
  EepromStatus = EepromWriteProgramter( EepromReadAddr, ( u8* )&stTargetRange, sizeof( stTargetRange ) );

  if( EEPROM_PROGRAM_WRITE_OK == EepromStatus )
  {
    stTempControl->targetTempMax = upperLimit;
    stTempControl->targetTempMin = lowerLimit;
    return ret;
  }
  else
  {

    return ERR_DRIVER_EEPROM;
  }



}
/*****************************************************************************
* @name   s32 GetTargetTempRange(u8 channelNum,tempControl* stTempControl)
* @brief  获取目标温度上下限
*
* @param
*    Input   channelNum

*    Output  Target      目标温度
*
* @return
*
*******************************************************************************/
u16 GetTargetTempRange( u8 channelNum, tempControl* stTempControl )
{
  u16 ret = SUCCESS1;
  EepromStatusDef EepromStatus;
  targetTemperatureErrParam stTemp;
  u32 EepromReadAddr;

  /*! 通道号判断 */
  if( ( MAX_CHANNEL_NUM < channelNum ) || ( channelNum < MIN_CHANNEL_NUM ) )
    return ERR_CHANNEL_NUM;

#if 0
  if( ( CHD == channelNum ) || ( CHA == channelNum ) )
  {

    EepromStatus = EepromReadProgramter( ERROR_MAX_TARGET, ( u8* )&stTemp, sizeof( stTemp ) );
  }
  else
  {
    EepromStatus = EepromReadProgramter( ERROR_MAX_TARGET_B, ( u8* )&stTemp, sizeof( stTemp ) );
  }
#endif

  switch( channelNum )
  {
    case CHA:
    {
      EepromReadAddr = ERROR_MAX_TARGET;
    }
    break;

    case CHB:
    {
      EepromReadAddr = ERROR_MAX_TARGET_B;
    }
    break;

    case CHC:
    {
      EepromReadAddr = ERROR_MAX_TARGET_C;
    }
    break;

    case CHD:
    {
      EepromReadAddr = ERROR_MAX_TARGET_D;
    }
    break;

    default:
      //			ret = ERR_CHANNEL_NUM;
      break;
  }
  EepromStatus = EepromReadProgramter( EepromReadAddr, ( u8* )&stTemp, sizeof( stTemp ) );

  if( EEPROM_PROGRAM_READ_OK == EepromStatus )
  {
    stTempControl->targetTempMax = stTemp.targetMaxValue;
    stTempControl->targetTempMin = stTemp.targetMinValue;

  }
  else
  {

    ret = ERR_DRIVER_EEPROM;

  }

  return ret;
}


/*****************************************************************************
* @name   u16 SetErrorLimitedTemperature(u8 channelNum)
* @brief  设置报错温度上下限（预留）@fixme for test
*
* @param
*    Input   channelNum

*    Output  Target      目标温度
*
* @return
*
*******************************************************************************/
u16 SetCurTemperRange( u8 channelNum, s32 upperLimit, s32 lowerLimit, tempControl *stTempControl )
{
  u16 ret = SUCCESS1;
  EepromStatusDef EepromStatus;
  //   temperatureAbnormalParam stTemp;
  curTempRange stTemp;
  u32 EepromReadAddr;
  /*! 通道号判断 */
  if( ( MAX_CHANNEL_NUM < channelNum ) || ( channelNum < MIN_CHANNEL_NUM ) )
    return ERR_CHANNEL_NUM;

  stTemp.maxCurTemp = upperLimit;
  stTemp.minCurTemp = lowerLimit;
#if 0
  if( ( CHD == channelNum ) || ( CHA == channelNum ) )
  {

    EepromStatus = EepromWriteProgramter( MAX_CURRENT_T, ( u8* )&stTemp, sizeof( stTemp ) ); //MAX_CURRENT_T
  }
  else
  {
    EepromStatus = EepromWriteProgramter( MAX_CURRENT_T_B, ( u8* )&stTemp, sizeof( stTemp ) ); //MAX_CURRENT_T
  }
#endif
  switch( channelNum )
  {
    case CHA:
    {
      EepromReadAddr = MAX_CURRENT_T;
    }
    break;

    case CHB:
    {
      EepromReadAddr = MAX_CURRENT_T_B;
    }
    break;

    case CHC:
    {
      EepromReadAddr = MAX_CURRENT_T_C;
    }
    break;

    case CHD:
    {
      EepromReadAddr = MAX_CURRENT_T_D;
    }
    break;

    default:
      ret = ERR_CHANNEL_NUM;
      break;
  }

  EepromStatus = EepromWriteProgramter( EepromReadAddr, ( u8* )&stTemp, sizeof( stTemp ) ); //MAX_CURRENT_T

  if( EEPROM_PROGRAM_WRITE_OK == EepromStatus )
  {
    stTempControl->maxErrorT = upperLimit;
    stTempControl->minErrorT = lowerLimit;

  }
  else
  {
    /*! 记入日志 */
    ret = ERR_DRIVER_EEPROM;
  }

  return ret;
}


/*****************************************************************************
* @name   u32 GetErrorLimitedTemperature(u8 channelNum)
* @brief  获取报错温度上下限（预留）
*
* @param
*    Input   channelNum

*    Output  Target      目标温度
*
* @return
*
*******************************************************************************/

u16 GetCurTemperRange( u8 channelNum, tempControl *stTempControl )
{
  s32 ret = SUCCESS1;
  EepromStatusDef EepromStatus;

  curTempRange stTemp;
  u32 EepromReadAddr;

  /*! 通道号判断 */
  if( ( MAX_CHANNEL_NUM < channelNum ) || ( channelNum < MIN_CHANNEL_NUM ) )
    return ERR_CHANNEL_NUM;

#if 0
  if( ( CHD == channelNum ) || ( CHA == channelNum ) )
  {

    EepromStatus = EepromReadProgramter( MAX_CURRENT_T, ( u8* )&stTemp, sizeof( stTemp ) ); //MAX_CURRENT_T
  }
  else
  {
    EepromStatus = EepromReadProgramter( MAX_CURRENT_T_B, ( u8* )&stTemp, sizeof( stTemp ) ); //MAX_CURRENT_T
  }
#endif
  switch( channelNum )
  {
    case CHA:
    {
      EepromReadAddr = MAX_CURRENT_T;
    }
    break;

    case CHB:
    {
      EepromReadAddr = MAX_CURRENT_T_B;
    }
    break;

    case CHC:
    {
      EepromReadAddr = MAX_CURRENT_T_C;
    }
    break;

    case CHD:
    {
      EepromReadAddr = MAX_CURRENT_T_D;
    }
    break;

    default:
      ret = ERR_CHANNEL_NUM;
      break;
  }
  EepromStatus = EepromReadProgramter( EepromReadAddr, ( u8* )&stTemp, sizeof( stTemp ) ); //MAX_CURRENT_T

  if( EEPROM_PROGRAM_READ_OK == EepromStatus )
  {
    stTempControl->maxErrorT = stTemp.maxCurTemp;
    stTempControl->minErrorT = stTemp.minCurTemp;

  }
  else
  {
    /*! 记入日志 */
    ret = ERR_DRIVER_EEPROM;

  }
  return ret;
}

/*****************************************************************************
* @name   s32 TemperatureFilterProcess(s32 currentTemp,u32 targetValue)
* @brief  温度滤波处理函数
*
* @param
*    Input   currentTemp   当前温度值
*    Input   targetValue   目标温度值
*    Output
*
* @return
*
*******************************************************************************/
s32 FilterProcessing( u8 channelNum, s32 currentTemp, tempControl *stTempControl )
{
  s32 ret = true;
  s16 errorCode = 0;

  stTempControl->pFlag.filterUpperLimit  = 10;

  if( ( currentTemp  > stTempControl->maxErrorT ) ||( currentTemp < stTempControl->minErrorT ) )
  {

    stTempControl->pFlag.filterCount ++;
    if( stTempControl->pFlag.filterCount >= stTempControl->pFlag.filterUpperLimit )
    {
#if 0

      if( stTempControl->pFlag.filterCount == stTempControl->pFlag.filterUpperLimit )
      {
        stTempControl->pFlag.filterCount = stTempControl->pFlag.filterUpperLimit + 1;
        errorCode = ERR_TEMPRETURE_SENSOR;


        SendAbnormalData( errorCode, &protocolAnalysisHandle, &stActiveRepResult, &activeReportFrameIndex, 0 );
        //			stTemperatureControl.stTempProcessFlag.sErrorStatusFlag = true;

        /*! @fixme 记入日志 */
        EepromSaveLog( 0x03, "sensor err!!" );

        //				ret =false;
      }
      else
      {
        stTempControl->pFlag.filterCount = stTempControl->pFlag.filterUpperLimit + 1;
        //				ret = false;

      }
#endif

      errorCode = ERR_TEMPRETURE_SENSOR_A + channelNum-1;
#if 0   //not report error
      SendAbnormalData( errorCode, &protocolAnalysisHandle, &stActiveRepResult, &activeReportFrameIndex, 0 );
      StopTec( channelNum, false );
#endif
      stTempControl->pFlag.filterCount = 0;
      /*! 记入日志 */
      EepromSaveLog( 0x03, "sensor err!!" );

      //			ret = false;
    }
    ret = false;

  }
  else
  {
    if( abs( stTempControl->LastT - currentTemp )>=TDATA_4 && ( stTempControl->LastT !=0 ) )
    {

      errorCode = ERR_TEMPRETURE_SENSOR_A + channelNum-1;
      //			SendAbnormalData( errorCode,&protocolAnalysisHandle,&stActiveRepResult,&activeReportFrameIndex,0);
    }

    stTempControl->pFlag.filterCount = 0;
    ret = true;
  }


  return ret;

}


/*****************************************************************************
* @name   s32 TestTemperature(s32 currentTemp,u32 targetValue)
* @brief  温度测试函数
*
* @param
*    Input   currentTemp   当前温度值
*    Input   targetValue   目标温度值
*    Input   channelNum    通道号
*    Output
*
* @return  true:正常 false:异常
*
*******************************************************************************/
s32 TestTemperature( s32 currentTemp, s32 targetValue, s32 index )
{
  s32 ret = true;

  /*! for test */
  if( ( ( targetValue - 240 )< currentTemp ) && ( currentTemp < ( 250+ targetValue ) ) )
  {
    index ++;
    if( index >= TEST_TIME )
    {
      index = 0;

      switch( targetValue )
      {
        case TEST_TARGET_20:
        {
          targetValue = TEST_TARGET_60;

        }
        break;

        case TEST_TARGET_60:
        {
          targetValue = TEST_TARGET_57;

        }
        break;

        case TEST_TARGET_40:
        {
          targetValue = TEST_TARGET_57;
        }
        break;

        case TEST_TARGET_57:
        {
          targetValue = TEST_TARGET_20;
        }
        break;

      }
    }
  }
  return ret;

}



/*****************************************************************************
* @name   static s32 VaryingTempControl(s32 currentTemp,s32 targetValue,tempControl stTemperatureControl)
* @brief  温度升/降温阶段控制函数
*
* @param
*    Input   currentTemp   当前温度值
*    Input   targetValue   目标温度值
*    Input   stTemperatureControl   温度控制参数
*    Output
*
* @return  true:正常 false:异常
*
*******************************************************************************/
static s32 VaryingTempControl( u8 channelNum, s32 currentTemp, s32 targetValue, tempControl *stTempControl, PidInfoDef *pPidInfo )
{
  s32 ret = SUCCESS1;
  u16 errorCode;

  /*! @fixme 是否需要上报生降温异常?  */
  if( stTempControl->stTempCount.changeStageCount >= stTempControl->stTempCount.changeCountUpLimit )
    //	if(stTempControl->stTempCount.changeStageCount >= 300)
  {
    stTempControl->stTempCount.changeStageCount = 0;
    stTempControl->stTempProcessFlag.sCurTempControlFlag = false;
    //		stTempControl->stTempProcessFlag.sCurTempControlFlag = true;	//for test
    stTempControl->stTempProcessFlag.sCurTempShockFlag = true;
    stTempControl->stTempCount.changeStageExcepCount = 0;
    //     	Debug_Printf(" VaryingOK ");
  }
  else
  {
    stTempControl->stTempCount.changeStageCount++;
    //	}


    if( abs( currentTemp - stTempControl->LastT ) == 0 )
    {
      stTempControl->stTempCount.changeStageExcepCount++;

    }
    else
    {

      stTempControl->stTempCount.changeStageExcepCount  =0;

      if( abs( targetValue - currentTemp )<=50 )
      {
        stTempControl->stTempProcessFlag.sFirstReachTargetFlag = true;

        stTempControl->stTempCount.changeStageCount = 0;
        stTempControl->stTempProcessFlag.sCurTempControlFlag = false;
        stTempControl->stTempProcessFlag.sCurTempShockFlag = true;
      }

    }

    if( stTempControl->stTempCount.changeStageExcepCount >= 100 )
    {
      /*! 温度传感器异常 */
      stTempControl->stTempCount.changeStageExcepCount  =0;

      errorCode = ERR_TEMPRETURE_SENSOR_A + channelNum-1;
#if 0   // not report the err
      SendAbnormalData( errorCode, &protocolAnalysisHandle, &stActiveRepResult, &activeReportFrameIndex, 0 );
      StopTec( stTempControl->channelNum, false );

      resetTCParamer( stTempControl );	//add zxh 20180726
#endif
    }
    else
    {

      PIDRegulator( currentTemp, targetValue, stTempControl->channelNum, pPidInfo );
    }
  }

  return ret;
}

/*****************************************************************************
* @name  static  s32 ShockTempControl(s32 currentTemp,s32 targetValue,tempControl stTemperatureControl)
* @brief  震荡阶段温度控制函数
*
* @param
*    Input   currentTemp   当前温度值
*    Input   targetValue   目标温度值
*    Input   stTemperatureControl   温度控制参数
*    Output
*
* @return  true:正常 false:异常
*
*******************************************************************************/
static s32 ShockTempControl( u8 channelNum, s32 currentTemp, s32 targetValue, tempControl *stTempControl, PidInfoDef *pPidInfo )
{
  s32 ret = SUCCESS1;
  u16 errorCode ;

  stTempControl->stTempCount.shockCount++;

  if( ( currentTemp>targetValue +stTempControl->limitWarningT_1 ) || \
      ( currentTemp<targetValue -stTempControl->limitWarningT_1 ) )
  {
    /*! 记入日志 */

  }

  PIDRegulator( currentTemp, targetValue, stTempControl->channelNum, pPidInfo );

  if( stTempControl->stTempCount.shockCount > stTempControl->stTempCount.shockCountUpLimit )
    //   if(stTempControl->stTempCount.shockCount > 400 )  //for test
  {
    stTempControl->stTempCount.shockCount = 0;
    stTempControl->stTempProcessFlag.sCurTempShockFlag = false;

    if( ( stTempControl->stTempProcessFlag.sFirstReachTargetFlag == true )   &&
        abs( targetValue - currentTemp )>TDATA_1 )
    {

    }
    else if( targetValue - currentTemp >TDATA_1 )
    {

      stTempControl->stTempProcessFlag.sFirstReachTargetFlag = false;

      errorCode = ERR_HOT_OVERTIME + channelNum*2-2;
      //			SendAbnormalData( errorCode,&protocolAnalysisHandle,&stActiveRepResult,&activeReportFrameIndex,0);
    }
    else if( currentTemp - targetValue >TDATA_1 )
    {

      stTempControl->stTempProcessFlag.sFirstReachTargetFlag = false;


      errorCode = ERR_COOL_OVERTIME + channelNum*2-2;
      //			SendAbnormalData( errorCode,&protocolAnalysisHandle,&stActiveRepResult,&activeReportFrameIndex,0);
    }
    else
    {

      stTempControl->stTempProcessFlag.sFirstReachTargetFlag = false;

    }
    stTempControl->stTempProcessFlag.sCurTempSteadyFlag = true;
  }

  return ret;
}

/*****************************************************************************
* @name  static  s32 SteadyControl(s32 currentTemp,s32 targetValue,tempControl stTemperatureControl)
* @brief  温度平稳阶段控制函数
*
* @param
*    Input   currentTemp   当前温度值
*    Input   targetValue   目标温度值
*    Input   stTemperatureControl   温度控制参数
*    Output
*
* @return  true:正常 false:异常
*
*******************************************************************************/
static s32 SteadyControl( u8 channelNum, s32 currentTemp, s32 targetValue, tempControl *stTempControl, PidInfoDef *pPidInfo )
{
  s32 ret = SUCCESS1;

  if( ( currentTemp>( targetValue +stTempControl->limitWarningT_2 ) ) || \
      ( currentTemp<( targetValue -stTempControl->limitWarningT_2 ) ) )
  {
    //上报超过目标温度上/下限故障，并记入日志

  }
  else if( ( currentTemp>( targetValue +stTempControl->limitWarningT ) ) || \
           ( currentTemp<( targetValue -stTempControl->limitWarningT ) ) )
  {
    //记入日志

  }

  ret = PIDRegulator( currentTemp, targetValue, stTempControl->channelNum, pPidInfo );

  return ret;

}


/*****************************************************************************
* @name   s32 TargetTempCalParamInit(void)
* @brief  目标温度校准参数初始化
*
* @param  无
*
* @return  true:正常 false:异常
*
*******************************************************************************/
s32 TargetTempCalParamInit( void )
{
  s32 ret = SUCCESS1;
  calTarTempParam stTemp;
  EepromStatusDef EepromStatus;

  memset( &stTemp, 0x00, sizeof( stTemp ) );

  /*! 从eeprom中读取目标温度校准参数 */
  EepromStatus = EepromReadProgramter( CAL_ENABLE, ( u8* )&stTemp, sizeof( stTemp ) );
  if( EepromStatus == EEPROM_PROGRAM_READ_OK )
  {
    //		stCalTarTempParam.enableFlag = 	stTemp.enableFlag;
    memcpy( &stCalTarTempParam, &stTemp, sizeof( calTarTempParam ) );
    return ret;
  }
  else
  {
    /*! 记入日志 */
    return EepromStatus;
  }

}

/*****************************************************************************
* @name   s32 TargetTempWarningParamInit(void)
* @brief  目标温度上下限参数初始化
*
* @param  无
*
* @return  true:正常 false:异常
*
*******************************************************************************/
s32 TargetTempWarningParamInit( u8 channelNum, tempControl *stTempControl )
{
  s32 ret = SUCCESS1;
  EepromStatusDef EepromStatus;
  targetTemperatureErrParam targetParam;
  u32 EepromReadAddr;

  memset( &targetParam, 0x00, sizeof( targetParam ) );

  switch( channelNum )
  {
    case CHA:
    {
      EepromReadAddr = ERROR_MAX_TARGET;
    }
    break;

    case CHB:
    {
      EepromReadAddr = ERROR_MAX_TARGET_B;
    }
    break;

    case CHC:
    {
      EepromReadAddr = ERROR_MAX_TARGET_C;
    }
    break;

    case CHD:
    {
      EepromReadAddr = ERROR_MAX_TARGET_D;
    }
    break;

    default:
      ret = ERR_CHANNEL_NUM;
      break;
  }

  //	/*! 读取目标温度报警范围 */
  EepromStatus = EepromReadProgramter( EepromReadAddr, ( u8* )&targetParam, sizeof( targetParam ) );

  if( EepromStatus == EEPROM_PROGRAM_READ_OK )
  {
    stTempControl->targetTempMax = targetParam.targetMaxValue;
    stTempControl->targetTempMin = targetParam.targetMinValue;

    return ret;
  }
  else
  {
    //记入日志
    return EepromStatus;
  }
}

/*****************************************************************************
* @name   s32 CurTempAbnomalParamInit(void)
* @brief  当前温度异常参数初始化
*
* @param  无
*
* @return  true:正常 false:异常
*
*******************************************************************************/
s32 CurTempAbnomalParamInit( void )
{
  s32 ret = SUCCESS1;
  EepromStatusDef EepromStatus;
  temperatureAbnormalParam readBuff;

  memset( &readBuff, 0x00, sizeof( readBuff ) );
  /*! 读取当前温度异常时的范围 */
  EepromStatus = EepromReadProgramter( WARNING_LIMIT, ( u8* )&readBuff, sizeof( readBuff ) );
  if( EepromStatus == EEPROM_PROGRAM_READ_OK )
  {
    stTemperatureControlD.limitWarningT = readBuff.warningLimit;
    stTemperatureControlD.limitWarningT_1 =readBuff.errorLimit ;
    return ret;
  }
  else
  {
    //记入日志
    return EepromStatus;
  }
}


/*****************************************************************************
* @name   s32 CalTecWorkNumInit(void)
* @brief  TEC使用次数参数初始化
*
* @param  无
*
* @return  true:正常 false:异常
*
*******************************************************************************/
s32 CalTecWorkNumInit( void )
{
  s32 ret = SUCCESS1;
  EepromStatusDef EepromStatus;
  TecWorkParam  TecParam;

  memset( &TecParam, 0x00, sizeof( TecParam ) );

  EepromStatus = EepromReadProgramter( HOT_NUM, ( u8* )&TecParam, sizeof( TecParam ) );

  if( EepromStatus == EEPROM_PROGRAM_READ_OK )
  {
    stTECWorkCount.hotNum = TecParam.hotCount;
    stTECWorkCount.coolNum= TecParam.coolCount;
    stTECWorkCount.cycle = TecParam.cycleCount;
    return ret;
  }
  else
  {
    //记入日志
    return EepromStatus;
  }

}


/*****************************************************************************
* @name   s32 CurTempRangeInit(void)
* @brief  当前温度报错范围初始化
*
* @param  无
*
* @return  true:正常 false:异常
*
*******************************************************************************/
s32 CurTempRangeInit( u8 channelNum, tempControl *stTempControl )
{
  s32 ret = SUCCESS1;
  EepromStatusDef EepromStatus;
  curTempRange stCurTempRange;
  u32 EepromReadAddr;

  memset( &stCurTempRange, 0x00, sizeof( stCurTempRange ) );

  switch( channelNum )
  {
    case CHA:
    {
      EepromReadAddr = MAX_CURRENT_T;
    }
    break;

    case CHB:
    {
      EepromReadAddr = MAX_CURRENT_T_B;
    }
    break;

    case CHC:
    {
      EepromReadAddr = MAX_CURRENT_T_C;
    }
    break;

    case CHD:
    {
      EepromReadAddr = MAX_CURRENT_T_D;
    }
    break;

    default:
      ret = ERR_CHANNEL_NUM;
      break;
  }

  EepromStatus = EepromReadProgramter( EepromReadAddr, ( u8* )&stCurTempRange, sizeof( stCurTempRange ) );

  if( EepromStatus == EEPROM_PROGRAM_READ_OK )
  {
    stTempControl->maxErrorT = stCurTempRange.maxCurTemp;
    stTempControl->minErrorT = stCurTempRange.minCurTemp;
    return ret;
  }
  else
  {
    //记入日志
    return EepromStatus;
  }

}
/*****************************************************************************
* @name   s32 TemperatureParamInit(void)
* @brief  温度参数初始化
*
* @param  无
*
* @return  true:正常 false:异常
*
*******************************************************************************/
s32 TemperatureParamInit( void )
{
  s32 ret = SUCCESS1;
  EepromStatusDef EepromStatus;
  blankBoard stBlankBoard;

  EepromStatus = EepromReadProgramter( BLACK_BOARD_FLAG, ( u8* )&stBlankBoard, sizeof( stBlankBoard ) );
  //stBlankBoard.blankBoardFlag = NEW_BOARD;
  if( EEPROM_PROGRAM_READ_OK == EepromStatus )
  {
    if( stBlankBoard.blankBoardFlag == OLD_BOARD )
    {
      TargetTempCalParamInit();

      /*! 目标温度上下限参数初始化 */
      TargetTempWarningParamInit( CHD, &stTemperatureControlD );
      TargetTempWarningParamInit( CHC, &stTemperatureControlC );
      TargetTempWarningParamInit( CHB, &stTemperatureControlB );
      TargetTempWarningParamInit( CHA, &stTemperatureControlA );

      CurTempAbnomalParamInit();

      CalTecWorkNumInit();

      /*! 当前温度上下限参数初始化 */
      CurTempRangeInit( CHD, &stTemperatureControlD );
      CurTempRangeInit( CHC, &stTemperatureControlC );
      CurTempRangeInit( CHB, &stTemperatureControlB );
      CurTempRangeInit( CHA, &stTemperatureControlA );

      PidParamInit( CHA, &PidInfoA );
      PidParamInit( CHB, &PidInfoB );
      PidParamInit( CHC, &PidInfoC );
      PidParamInit( CHD, &PidInfoD );

      stTemperatureControlA.stTempCount.changeCountUpLimit = 500;
      stTemperatureControlA.stTempCount.shockCountUpLimit  = 400;
			
      stTemperatureControlB.stTempCount.changeCountUpLimit = 500; 
      stTemperatureControlB.stTempCount.shockCountUpLimit  =400;

      stTemperatureControlC.stTempCount.changeCountUpLimit = 500;
      stTemperatureControlC.stTempCount.shockCountUpLimit  =400;

      stTemperatureControlD.stTempCount.changeCountUpLimit = 500;
      stTemperatureControlD.stTempCount.shockCountUpLimit  =400;
    }
    else
    {
      WriteDefaultParam( CHA, &stTemperatureControlA );
      WriteDefaultParam( CHB, &stTemperatureControlB );
      WriteDefaultParam( CHC, &stTemperatureControlC );
      WriteDefaultParam( CHD, &stTemperatureControlD );

      WriteDefaultPIDParam( CHA, &stParamOfPIDCal, &PidInfoA );
      WriteDefaultPIDParam( CHB, &stParamOfPIDCal, &PidInfoB );
      WriteDefaultPIDParam( CHC, &stParamOfPIDCal, &PidInfoC );
      WriteDefaultPIDParam( CHD, &stParamOfPIDCal, &PidInfoD );
      //WriteDefaultPIDParam(5, &stParamOfPIDCal,&PidInfoD);

      if( stBlankBoard.blankBoardFlag != OLD_BOARD )
      {
        stBlankBoard.blankBoardFlag = OLD_BOARD;
        EepromStatus = EepromWriteProgramter( BLACK_BOARD_FLAG, ( u8 * )&stBlankBoard, sizeof( stBlankBoard ) );
      }
    }
  }
  else
  {
    WriteDefaultParam( CHA, &stTemperatureControlA );
    WriteDefaultParam( CHB, &stTemperatureControlB );
    WriteDefaultParam( CHC, &stTemperatureControlC );
    WriteDefaultParam( CHD, &stTemperatureControlD );

    WriteDefaultPIDParam( CHA, &stParamOfPIDCal, &PidInfoA );
    WriteDefaultPIDParam( CHB, &stParamOfPIDCal, &PidInfoB );
    WriteDefaultPIDParam( CHC, &stParamOfPIDCal, &PidInfoC );
    WriteDefaultPIDParam( CHD, &stParamOfPIDCal, &PidInfoD );

    if( stBlankBoard.blankBoardFlag != OLD_BOARD )
    {
      stBlankBoard.blankBoardFlag = OLD_BOARD;
      EepromStatus = EepromWriteProgramter( BLACK_BOARD_FLAG, ( u8 * )&stBlankBoard, sizeof( stBlankBoard ) );
    }
  }
  //	stTemperatureControlB.channelEnableFlag = 1;
  //	stTemperatureControlB.stTempProcessFlag.sCurTempControlFlag = true;	//add 20180123

  return ret;
}


/*! 写默认参数 */
void WriteDefaultParam( u8 channelNum, tempControl *stTempControl )
{

  EepromStatusDef EepromStatus;
  temperatureAbnormalParam stTemp;
  TecWorkParam  stTecParam;
  targetTemperatureErrParam stTargetParam;
  //	blankBoard stBlankBoard;
  curTempRange stCurTempRange;
  calTarTempParam stReadBuffer;
  u32 EepromReadAddr;


  memset( &stReadBuffer, 0x00, sizeof( stReadBuffer ) );
  memset( &stTemp, 0x00, sizeof( stTemp ) );
  memset( &stTecParam, 0x00, sizeof( stTecParam ) );
  memset( &stTargetParam, 0x00, sizeof( stTargetParam ) );

  /*! 写入默认参数 */
  EepromStatus = EepromWriteProgramter( CAL_ENABLE, ( u8 * )&stReadBuffer, sizeof( stReadBuffer ) );

  stTemp.errorLimit = LIMIT_WARNING_T_2 ;
  stTemp.warningLimit = LIMIT_WARNING_T;
  EepromStatus = EepromWriteProgramter( WARNING_LIMIT, ( u8* )&stTemp, sizeof( stTemp ) );

  stTempControl->limitWarningT = stTemp.warningLimit;
  stTempControl->limitWarningT_1 =stTemp.errorLimit ;

  /*! 默认目标温度上下限[10,70],[2,8] */
  switch( channelNum )
  {
    case CHA:
    {
      stTargetParam.targetMaxValue = TARGET_TEMP_MAX ;
      stTargetParam.targetMinValue = TARGET_TEMP_MIN;
      EepromReadAddr = ERROR_MAX_TARGET;

      stTempControl->stTempCount.changeCountUpLimit = 500;
      stTempControl->stTempCount.shockCountUpLimit = 400;
    }
    break;

    case CHB:
    {
      stTargetParam.targetMaxValue = TARGET_TEMP_MAX ;
      stTargetParam.targetMinValue = TARGET_TEMP_MIN;
      EepromReadAddr = ERROR_MAX_TARGET_B;

      stTempControl->stTempCount.changeCountUpLimit = 500;
      stTempControl->stTempCount.shockCountUpLimit = 400;
    }
    break;

    case CHC:
    {
      stTargetParam.targetMaxValue = TARGET_TEMP_MAX ;
      stTargetParam.targetMinValue = TARGET_TEMP_MIN;
      EepromReadAddr = ERROR_MAX_TARGET_C;
      stTempControl->stTempCount.changeCountUpLimit = 500;
      stTempControl->stTempCount.shockCountUpLimit = 400;
    }
    break;

    case CHD:
    {
      stTargetParam.targetMaxValue = TARGET_TEMP_MAX ;
      stTargetParam.targetMinValue = TARGET_TEMP_MIN;
      EepromReadAddr = ERROR_MAX_TARGET_D;

      stTempControl->stTempCount.changeCountUpLimit = 500;
      stTempControl->stTempCount.shockCountUpLimit = 400 ; 
    }
    break;

    default:
      //			ret = ERR_CHANNEL_NUM;
      break;
  }

  EepromStatus = EepromWriteProgramter( EepromReadAddr, ( u8* )&stTargetParam, sizeof( stTargetParam ) );

  EepromStatus = EepromStatus;
  //		if(EepromStatus == EEPROM_PROGRAM_WRITE_OK)
  //		{
  stTempControl->targetTempMax = stTargetParam.targetMaxValue;
  stTempControl->targetTempMin = stTargetParam.targetMinValue;
  //		}

  /*! TEC工作次数统计 */
  stTecParam.coolCount = 0;
  stTecParam.cycleCount = 0;
  stTecParam.hotCount = 0;
  EepromStatus = EepromWriteProgramter( HOT_NUM, ( u8* )&stTecParam, sizeof( stTecParam ) );

  /*! 当前温度上下限 [1,75],[0,35] */
  switch( channelNum )
  {
    case CHA:
    {
      stCurTempRange.maxCurTemp = MAX_ERROR_T;
      stCurTempRange.minCurTemp = MIN_ERROR_T;
      EepromReadAddr = MAX_CURRENT_T;
    }
    break;

    case CHB:
    {
      stCurTempRange.maxCurTemp = MAX_ERROR_T;
      stCurTempRange.minCurTemp = MIN_ERROR_T;
      EepromReadAddr = MAX_CURRENT_T_B;
    }
    break;

    case CHC:
    {
      stCurTempRange.maxCurTemp = MAX_ERROR_T;
      stCurTempRange.minCurTemp = MIN_ERROR_T;
      EepromReadAddr = MAX_CURRENT_T_C;
    }
    break;

    case CHD:
    {
      stCurTempRange.maxCurTemp = MAX_ERROR_T;
      stCurTempRange.minCurTemp = MIN_ERROR_T;
      EepromReadAddr = MAX_CURRENT_T_D;
    }
    break;

    default:
      //			ret = ERR_CHANNEL_NUM;
      break;
  }

  EepromStatus = EepromWriteProgramter( EepromReadAddr, ( u8* )&stCurTempRange, sizeof( stCurTempRange ) );
  EepromStatus = EepromStatus;
  stTempControl->maxErrorT = stCurTempRange.maxCurTemp;
  stTempControl->minErrorT = stCurTempRange.minCurTemp;


}

u16 LiquidPumpContrl( u8 channelNum, bool enableFlag )
{
  u16 ret = SUCCESS1;

  /*! 通道号判断 */
  if( ( MAX_CHANNEL_NUM < channelNum ) || ( channelNum < MIN_CHANNEL_NUM ) )
    return ERR_CHANNEL_NUM;

  if( enableFlag>1 )
  {
    return ERR_PARAM_ABNOMAL;
  }

  if( channelNum == CHA )
  {
    if( enableFlag )
    {
      HAL_GPIO_WritePin( GPIOB, PUMP_A, GPIO_PIN_RESET );
      HAL_GPIO_WritePin( GPIOB, PUMP_B, GPIO_PIN_RESET );
    }
    else
    {
      HAL_GPIO_WritePin( GPIOB, PUMP_A, GPIO_PIN_SET );
      HAL_GPIO_WritePin( GPIOB, PUMP_B, GPIO_PIN_SET );
    }
  }
  else if( channelNum == CHB )
  {

  }

  else if( channelNum == CHC )
  {

    if( enableFlag )
    {
      HAL_GPIO_WritePin( GPIOB, PUMP_C, GPIO_PIN_RESET );

    }
    else
    {
      HAL_GPIO_WritePin( GPIOB, PUMP_C, GPIO_PIN_SET );

    }
  }
  else if( channelNum == CHD )
  {

  }

  return ret;
}


u16 LiquidFanContrl( u8 channelNum, bool enableFlag )
{
  u16 ret = SUCCESS1;

  /*! 通道号判断 */
  if( ( MAX_CHANNEL_NUM < channelNum ) || ( channelNum < MIN_CHANNEL_NUM ) )
    return ERR_CHANNEL_NUM;

  if( enableFlag>1 )
  {
    return ERR_PARAM_ABNOMAL;
  }

  if( channelNum == CHA )
  {
    /*! enableFlag 1:工作  0：不工作 */
    if( enableFlag )     //打开
    {
      HAL_GPIO_WritePin( GPIOB, PUMP_A, GPIO_PIN_RESET );
      //HAL_GPIO_WritePin( GPIOB, PUMP_B, GPIO_PIN_RESET ); //exit zzq 20220424
    }
    else  if( stTemperatureControlA.channelEnableFlag )        //关闭
    {
      return WAR_NOT_CLOSE_FANA;
    }
    else
    {
      HAL_GPIO_WritePin( GPIOB, PUMP_A, GPIO_PIN_SET );
      //HAL_GPIO_WritePin( GPIOB, PUMP_B, GPIO_PIN_SET ); //exit zzq 20220424
    }
  }
  /*! 水冷风扇B */
  else if( channelNum == CHB )
  {
    /*! 1:工作  0：不工作 */
    if( enableFlag )     //打开
    {
      HAL_GPIO_WritePin( GPIOB, PUMP_B, GPIO_PIN_RESET );

    }
    else  if( stTemperatureControlB.channelEnableFlag )        //关闭
    {
      return WAR_NOT_CLOSE_FANB;
    }
    else           //关闭
    {
      HAL_GPIO_WritePin( GPIOB, PUMP_B, GPIO_PIN_SET );

    }
  }
  else if( channelNum == CHC ) //add zzq 20220424
  {
    /*! enableFlag 1:工作  0：不工作 */
    if( enableFlag )     //打开
    {
      HAL_GPIO_WritePin( GPIOA, PUMP_C, GPIO_PIN_RESET );
    }
    else  if( stTemperatureControlC.channelEnableFlag )        //关闭
    {
      return WAR_NOT_CLOSE_FANC;
    }
    else
    {
      HAL_GPIO_WritePin( GPIOA, PUMP_C, GPIO_PIN_SET );
    }
  }

  /*! 水冷风扇A */
  else if( channelNum == CHD )
  {
    /*! enableFlag 1:工作  0：不工作 */
    if( enableFlag )     //打开
    {
      HAL_GPIO_WritePin( GPIOB, PUMP_D, GPIO_PIN_RESET );
    }
    else  if( stTemperatureControlD.channelEnableFlag )        //关闭
    {
      return WAR_NOT_CLOSE_FAND;
    }
    else
    {
      HAL_GPIO_WritePin( GPIOB, PUMP_D, GPIO_PIN_SET );
    }
  }

  return ret;
}


s32 TmpData[DIFF_CHN_MAX][5];
static s32 TemperatureControlDeal( u8 channelNum, s32 currentTemp, s32 targetTemp, tempControl *stTempControl, PidInfoDef *pPidInfo );
#if 0
s32 TempControlSystem( u8 channelNum, tempControl *stTempControl, PidInfoDef *pPidInfo )
{
  //	u16 errorCode;
  s8  getTempStatus = 0;

  /*! 参数判断 */
  if( ( MAX_CHANNEL_NUM < channelNum ) || ( channelNum < MIN_CHANNEL_NUM ) )
  {
    /*! 上报该故障 */
    SendAbnormalData( ERR_CHANNEL_NUM, &protocolAnalysisHandle, &stActiveRepResult, &activeReportFrameIndex, 0 );
    //stTempControl->stTempProcessFlag.sErrorStatusFlag = true;
    return SUCCESS1;
  }

  if( stTempControl->channelEnableFlag == true )
  {
    /*! 获取温度值 */
    getTempStatus = TEMP_GetTempData( channelNum-1, 1, TmpData[channelNum-1] );

    if( getTempStatus == AD7124_OK )
    {
      if( FilterProcessing( channelNum, TmpData[channelNum-1][0], stTempControl ) )
      {
        Debug_Printf( " CH:%d , data: %d\r\n", channelNum, TmpData[channelNum-1][0] );
        TemperatureControlDeal( channelNum, TmpData[channelNum-1][0], pPidInfo->SetTemperature*100, stTempControl, pPidInfo );
        stTempControl->LastT = TmpData[channelNum-1][0];
      }
    }
    else
    {
      Debug_Printf( " CH_2:%d, data: %d\r\n", channelNum, TmpData[channelNum-1][0] );
      EepromSaveLog( 0x03, "read ADC status err!" );
    }
  }
  return SUCCESS1;
}
#endif


s32 TempControlSystem( u8 channelNum, tempControl *stTempControl, PidInfoDef *pPidInfo )
{
  //	u16 errorCode;
  s8  getTempStatus = 0;
  static u16 TimeCount = 0;

  /*! 参数判断 */
  if( ( MAX_CHANNEL_NUM < channelNum ) || ( channelNum < MIN_CHANNEL_NUM ) )
  {
    /*! 上报该故障 */
    SendAbnormalData( ERR_CHANNEL_NUM, &protocolAnalysisHandle, &stActiveRepResult, &activeReportFrameIndex, 0 );
    //stTempControl->stTempProcessFlag.sErrorStatusFlag = true;
    return SUCCESS1;
  }

  if( stTempControl->channelEnableFlag == true )
  {
    stTempControl->channelTECSTopTime[channelNum-1] = 0;
    stTempControl->channelTECFirstRunFlag[channelNum-1] = 1;

    /*! 获取温度值 */
    getTempStatus = TEMP_GetTempData( channelNum-1, 1, TmpData[channelNum-1] );
    if( getTempStatus == AD7124_OK )
    {
      if( FilterProcessing( channelNum, TmpData[channelNum-1][0], stTempControl ) )
      {
        //Debug_Printf( " CH:%d , data: %d\r\n", channelNum, TmpData[channelNum-1][0] );
        Debug_Printf( "%d,%d,%d,\r\n", channelNum, TmpData[channelNum-1][0], ( uint32_t )( ( float )( pPidInfo->SetTemperature*100 ) ) );
        TemperatureControlDeal( channelNum, TmpData[channelNum-1][0], pPidInfo->SetTemperature*100, stTempControl, pPidInfo );
        stTempControl->LastT = TmpData[channelNum-1][0];
      }
    }
    else
    {
      Debug_Printf( " CH_2:%d, data: %d\r\n", channelNum, TmpData[channelNum-1][0] );
      EepromSaveLog( 0x03, "read ADC status err!" );
    }
  }
  else //stTempControl->channelEnableFlag == false 停止工作
  {
    if( stTempControl->channelTECFirstRunFlag[channelNum-1] == 1 )
    {
      stTempControl->channelTECSTopTime[channelNum-1] ++;
      Debug_Printf( "STopTime[%d]:%d\r\n", channelNum, stTempControl->channelTECSTopTime[channelNum-1] );

      /*! 获取温度值 */
      getTempStatus = TEMP_GetTempData( channelNum-1, 1, TmpData[channelNum-1] );
      if( getTempStatus == AD7124_OK )
      {
        if( FilterProcessing( channelNum, TmpData[channelNum-1][0], stTempControl ) )
        {
          Debug_Printf( " CH:%d , data: %lf\r\n", channelNum, ( float )( ( float )TmpData[channelNum-1][0]/( float )100 ) );
          if( ( stTempControl->channelTECSTopTime[channelNum-1] >= TEC_STOP_MIN_TIME ) && ( stTempControl->channelTECSTopTime[channelNum-1] <= TEC_STOP_MAX_TIME ) ) //1min--3min
          {
            if( ( ( float )( ( float )TmpData[channelNum-1][0]/( float )100 ) > ( float )20 )  && ( ( float )( ( float )TmpData[channelNum-1][0]/( float )100 ) < ( float )30 ) )
            {
              LiquidFanContrl( channelNum, false ); //关闭风扇
							LiquidFanContrl( channelNum + 1, false ); //关闭风扇
              stTempControl->channelTECFirstRunFlag[channelNum-1] = 2;
            }
          }
          if( stTempControl->channelTECSTopTime[channelNum-1] > TEC_STOP_MAX_TIME )
          {
            LiquidFanContrl( channelNum, false ); //关闭风扇
						LiquidFanContrl( channelNum + 1, false ); //关闭风扇
            stTempControl->channelTECFirstRunFlag[channelNum-1] = 2;
          }
        }
      }
    }
  }
  return SUCCESS1;
}








s32 TempControlSystem_test( u8 channelNum, tempControl *stTempControl, PidInfoDef *pPidInfo )
{
  static u8 i;

  for( i=0; i<=16; i++ )
  {
    //		TemperatureControlDeal(channelNum,TestData[i],pPidInfo->SetTemperature*100,stTempControl,pPidInfo);

    //				stTempControl->LastT = TmpData[channelNum-1][0];
  }
  return SUCCESS1;

}
static s32 TemperatureControlDeal( u8 channelNum, s32 currentTemp, s32 targetTemp, tempControl *stTempControl, PidInfoDef *pPidInfo )
{
  s32 ret = SUCCESS1;


  stTempControl->channelNum = channelNum;	 //通道号赋值

  if( stTempControl->stTempProcessFlag.sErrorStatusFlag == false )
  {

    if( stTempControl->stTempProcessFlag.sCurTempControlFlag == true )
    {
      ret = VaryingTempControl( channelNum, currentTemp, targetTemp, stTempControl, pPidInfo );
      Debug_Printf( "0000 CH:%d , curr: %d  targ: %d \r\n", channelNum, currentTemp, targetTemp );
    }
    else if( stTempControl->stTempProcessFlag.sCurTempShockFlag == true )
    {
      ret = ShockTempControl( channelNum, currentTemp, targetTemp, stTempControl, pPidInfo );
      Debug_Printf( "1111 CH:%d , curr: %d  targ: %d \r\n", channelNum, currentTemp, targetTemp );
    }
    else if( stTempControl->stTempProcessFlag.sCurTempSteadyFlag == true )
    {
      ret = SteadyControl( channelNum, currentTemp, targetTemp, stTempControl, pPidInfo );
      Debug_Printf( "2222 CH:%d , curr: %d  targ: %d \r\n", channelNum, currentTemp, targetTemp );
    }
    else
    {

    }

  }

  return ret;

}

void resetTCParamer( tempControl *stTempControl )
{

  stTempControl->firstSetTargetFlag = false;
  stTempControl->pFlag.filterCount =0;
  stTempControl->stTempCount.changeStageCount = 0;
  stTempControl->stTempCount.changeStageExcepCount = 0;
  stTempControl->stTempCount.shockCount = 0;
  stTempControl->LastT  = 0;
  stTempControl->stTempProcessFlag.sCurTempControlFlag = false;
  stTempControl->stTempProcessFlag.sCurTempShockFlag = false;
  stTempControl->stTempProcessFlag.sCurTempSteadyFlag = false;
  stTempControl->stTempProcessFlag.sFirstReachTargetFlag = false;
  stTempControl->stTempProcessFlag.sErrorStatusFlag = false;

}

