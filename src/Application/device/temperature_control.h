
/***********************************************************
Module Name: temperature_control.c
Description: 温度控制模块。实现升温，降温、温度异常故障上报功能
Module Date: 27-07-2016
Module Auth: Firmware-Team
Others:
***********************************************************/


#ifndef _TEMPERATURE_CONTROL_H_
#define _TEMPERATURE_CONTROL_H_


#include "stm32f3xx.h"
#include "pid.h"
 
#define CHA      (1)    //A通道
#define CHB      (2)    //B通道 
#define CHC      (3)    //C通道
#define CHD      (4)    //D通道 

//#define  MAX_CALI_NUM  (10)
#define  MAX_CALI_NUM  (10)


#define    HOLDER_A                (CHD)
#define    HOLDER_B                (CHA)
#define    REFRIGERATOR      (CHB)
#define    HOLDER_ENVIR       (CHC)

__packed
typedef struct
{
  u8 sCurTempControlFlag;   //温度升/降温标示
  u8 sCurTempSteadyFlag;     //温度平稳标示
  u8 sFirstReachTargetFlag;  //首次达到目标温度标示
  u8 sCurTempShockFlag;       //震荡标示
  u8 sErrorStatusFlag;      //故障状态标示
} stTempProcess;

//stTempProcess  stTempProcessFlag;

__packed
typedef struct
{
  s32  changeStageCount;              //升降温阶段的总计数时间
  s32  changeStageExcepCount;         //升降温阶段异常数据计数
  s32  shockCount;                    //震荡阶段的计数
  s32  changeCountUpLimit;            //升降温阶段的计数值上限
  s32  shockCountUpLimit;             //震荡阶段的计数值上限
} Count;

/*! ------------存储在eeprom中的参数  start ----------------------------*/

/*! 目标温度范围 */
__packed
typedef struct
{
  s32 targetMaxValue;	 //目标温度最大值
  s32 targetMinValue;  //目标温度最小值
  u16 checkSum;		 //校验值
} targetTemperatureErrParam;

/* 温度异常参数 */
__packed
typedef struct
{
  s16 warningLimit;	 //报警范围
  s16 errorLimit;      //报错范围
  u16 checkSum;		 //校验值
} temperatureAbnormalParam;

/*! TEC加热/制冷次数参数 */
__packed
typedef struct
{
  u32 hotCount;	    //加热次数计数
  u32 coolCount;      //制冷次数计数
  u32 cycleCount;     //TEC循环使用次数计数
  u16 checkSum;		 //校验值
} TecWorkParam;

/*! 目标温度点对点校准参数 */
__packed
typedef struct
{
  u8 enableFlag;
  s32  calData[MAX_CALI_NUM][2];
  u16  checkSum;
} calTarTempParam;

/*! 当前温度报错 最大/最小值（即报错范围） */
__packed
typedef struct
{
  s32 maxCurTemp;      //当前温度 上限
  s32 minCurTemp;      //当前温度 下限
  u16 checkSum;        //校验和
} curTempRange;


/*! ------------存储在eeprom中的参数  end ----------------------------*/

__packed
typedef struct
{
  s32 filterCount;
  s32 filterUpperLimit;

} filterFlag;

#if 1

__packed
typedef struct
{
  u8  channelNum;
  s32 targetTempMax;      //目标温度最大值
  s32 targetTempMin;      //目标温度最小值
  s32 maxErrorT;          //当前温度故障范围 （最大值）
  s32 minErrorT;          //当前温度故障范围 （最小值）
  s32 limitWarningT_1;    //温度报警范围(+-1°)
  s32 limitWarningT_2;    //温度报警范围(+-2°)
  s32 limitWarningT;      //温度报警范围(+-0.5°)
  stTempProcess  stTempProcessFlag;  //温度处理标示
  Count stTempCount;	               //温度计数
  s32 LastT;       //last temperature
  filterFlag pFlag;
  u8 channelEnableFlag;
  bool firstSetTargetFlag;
	
	s32 channelTECSTopTime[4]; //TEC关闭时间
	u8 channelTECFirstRunFlag[4];
} tempControl;

#endif


//extern filterFlag stFilterVariA,stFilterVariB,stFilterVariC,stFilterVariD;

//extern tempControl stTemperatureControl;
extern tempControl stTemperatureControlA, stTemperatureControlB, stTemperatureControlC, stTemperatureControlD;

extern calTarTempParam stCalTarTempParam;
extern u16 activeReportFrameIndex;

/*****************************************************************************
* @name   s32 RequireTempBoardVersion(void)
* @brief  获取温控板版本
*******************************************************************************/
extern s32 RequireTempBoardVersion( void );

/*****************************************************************************
* @name s32 temperatureControl(u8 channelNum,s32 currentTemp,tempControl *stTempControl);

* @brief  温度控制函数
*
* @param
*    Input   通道号
*
*    Output
*
* @return
*
*******************************************************************************/
//extern s32 temperatureControl(u8 channelNum);
//extern s32 TemperatureControl(u8 channelNum,s32 currentTemp,tempControl *stTempControl);

/*****************************************************************************
* @name   s32 TemperatureFilterProcess(s32 currentTemp,u32 targetValue)
* @brief  温度滤波处理函数
*
* @param
*    Input   currentTemp   当前温度值
*    Input   targetValue   目标温度值
*    Input   lastTemp      上次温度值
*    Output
*
* @return
*
*******************************************************************************/
//extern s32 TemperatureFilterProcess_test(s32 currentTemp,s32 *lastTemp,filterFlag *filterParam);
//extern s32 TemperatureFilterProcess(s32 currentTemp,tempControl *stTempControl);
/*******************************************************************************************
* @Name: s32 PIDRegulator(u32 targetValue)
*
* @brief:      PID调节
*
* @param  :
     Inputs:  目标温度值
     Outputs: PID调节后的输出（PWM波）

* @return   True 成功   False 失败
*
**********************************************************************************/
extern s32 PIDRegulator( s32 currentTemp, s32 targetValue, u8 channelNum, PidInfoDef *pPidInfo );
//extern s32 PIDRegulator(u8 channelNum,s32 currentTemp,s32 targetValue);

/*****************************************************************************
* @name   s32 SetTargetTemperature(u8 channelNum,u32 Target)
* @brief  设置目标温度
*
* @param
*    Input   Target      目标温度
*    Output  无
*
* @return
*
*******************************************************************************/
//extern u16 SetTargetTemperature(u8 channelNum,s32 target);
//extern u16 SetTargetTemperature(u8 channelNum,s32 target,tempControl *stTempControl);
extern u16 SetTargetTemperature( u8 channelNum, s32 target, tempControl *stTempControl, PidInfoDef *pPidInfo );
/*****************************************************************************
* @name   u32 GetTargetTemperature(u8 channelNum)
* @brief  获取目标温度
*
* @param
*    Input   channelNum

*    Output  Target      目标温度
*
* @return
*
*******************************************************************************/
//extern s32 GetTargetTemperature(u8 channelNum);
u16 GetTargetTemperature_test( u8 channelNum, s32 *targTemp, PidInfoDef *pPidInfo );
u16 GetTargetTemperature( u8 channelNum, s32 *targTemp, PidInfoDef pPidInfo );
/*****************************************************************************
* @name  u16 GetCurrentTemperature(u8 channelNum，s32* pData)
* @brief  获取当前通道温度值
*
* @param
*    Input   channelNum  通道号
*    Output  无
*
* @return   True 成功   False 失败
*
*******************************************************************************/
extern u16 GetCurrentTemperature( u8 channelNum, s32* pData );
extern u16 GetSensorValue( u8 channelNum, s32* pData, tempControl stTempControl );
/*****************************************************************************
* @name   s32 GetChannelEnableFlag(u8 channelNum ,bool* channelEnableFlag)
* @brief  获取通道使能（不使能）标示
*
* @param
*    Input   channelNum             通道号
             channelEnableFlag      通道使能标示
*    Output  bool
*
* @return
*
*******************************************************************************/
//extern u16 GetChannelEnableFlag(u8 channelNum,u8 *pFlag);
u16 GetChannelEnableFlag( u8 channelNum, u8 *pFlag, tempControl *stTempControl );

//extern u16 SetChannelEnableFlag(u8 channelNum ,u8 channelEnableFlag);
extern u16 SetChannelEnableFlag( u8 channelNum, u8 channelEnableFlag, tempControl *stTempControl );


/*****************************************************************************
* @name   s32 SetWarningLimitedTemperature(u8 channelNum)
* @brief  设置报警温度上下限
*
*******************************************************************************/
extern u16 SetTargetTempRange( u8 channelNum, s32 upperLimit, s32 lowerLimit, tempControl *stTempControl );

/*****************************************************************************
* @name   u32 GetWarningLimitedTemperature(u8 channelNum)
* @brief  获取报警温度上下限
*
*******************************************************************************/
//extern s32 GetWarningLimitedTemperature(u8 channelNum,s32 upperLimit,s32 lowerLimit,tempControl stTempControl);
extern u16 GetTargetTempRange( u8 channelNum, tempControl* stTempControl );
/*****************************************************************************
* @name   u32 SetErrorLimitedTemperature(u8 channelNum)
* @brief  设置报错温度上下限
*
*******************************************************************************/
extern u16 SetCurTemperRange( u8 channelNum, s32 upperLimit, s32 lowerLimit, tempControl *stTempControl );

/*****************************************************************************
* @name   u32 GetErrorLimitedTemperature(u8 channelNum)
* @brief  获取报错温度上下限
*
*******************************************************************************/
extern u16 GetCurTemperRange( u8 channelNum, tempControl* stTempControl );
/*****************************************************************************
* @name s32 CalibrationTargetTemperature1(bool enableFlag, s32* targetTemp,s32* calibrationTemperature,s32 *pTargetValue)

* @brief  目标温度校准(校准后的值通过SBC设置，存储在firmware的eeprom中)
*
* @param
*    Input   targetTemp       读取到的目标温度


*    Output  *pTargetValue    校准后的目标温度
*
* @return   校准后的目标温度
*
*******************************************************************************/
extern u16 CalibrationTargetTemperature( calTarTempParam *stTemp );

/*****************************************************************************
* @name   s32 TemperatureParamInit(void)
* @brief  温度参数初始化
*
* @param  无
*
* @return  true:正常 false:异常
*
*******************************************************************************/
extern s32 TemperatureParamInit( void );

extern u16 LiquidPumpContrl( u8 channelNum, bool enableFlag );

extern u16 LiquidFanContrl( u8 channelNum, bool enableFlag );

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
extern s32 StopAllOfTec( void );


extern s32 Heating( u8 channelNum, bool ableFlag, u8 percent );
extern s32 Cool( u8 channelNum, bool ableFlag, u8 percent );


extern s32 TempControlSystem( u8 channelNum, tempControl *stTempControl, PidInfoDef *pPidInfo );
void resetTCParamer( tempControl *stTempControl );
#endif

