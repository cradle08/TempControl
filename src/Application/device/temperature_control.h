
/***********************************************************
Module Name: temperature_control.c
Description: �¶ȿ���ģ�顣ʵ�����£����¡��¶��쳣�����ϱ�����
Module Date: 27-07-2016
Module Auth: Firmware-Team
Others:
***********************************************************/


#ifndef _TEMPERATURE_CONTROL_H_
#define _TEMPERATURE_CONTROL_H_


#include "stm32f3xx.h"
#include "pid.h"
 
#define CHA      (1)    //Aͨ��
#define CHB      (2)    //Bͨ�� 
#define CHC      (3)    //Cͨ��
#define CHD      (4)    //Dͨ�� 

//#define  MAX_CALI_NUM  (10)
#define  MAX_CALI_NUM  (10)


#define    HOLDER_A                (CHD)
#define    HOLDER_B                (CHA)
#define    REFRIGERATOR      (CHB)
#define    HOLDER_ENVIR       (CHC)

__packed
typedef struct
{
  u8 sCurTempControlFlag;   //�¶���/���±�ʾ
  u8 sCurTempSteadyFlag;     //�¶�ƽ�ȱ�ʾ
  u8 sFirstReachTargetFlag;  //�״δﵽĿ���¶ȱ�ʾ
  u8 sCurTempShockFlag;       //�𵴱�ʾ
  u8 sErrorStatusFlag;      //����״̬��ʾ
} stTempProcess;

//stTempProcess  stTempProcessFlag;

__packed
typedef struct
{
  s32  changeStageCount;              //�����½׶ε��ܼ���ʱ��
  s32  changeStageExcepCount;         //�����½׶��쳣���ݼ���
  s32  shockCount;                    //�𵴽׶εļ���
  s32  changeCountUpLimit;            //�����½׶εļ���ֵ����
  s32  shockCountUpLimit;             //�𵴽׶εļ���ֵ����
} Count;

/*! ------------�洢��eeprom�еĲ���  start ----------------------------*/

/*! Ŀ���¶ȷ�Χ */
__packed
typedef struct
{
  s32 targetMaxValue;	 //Ŀ���¶����ֵ
  s32 targetMinValue;  //Ŀ���¶���Сֵ
  u16 checkSum;		 //У��ֵ
} targetTemperatureErrParam;

/* �¶��쳣���� */
__packed
typedef struct
{
  s16 warningLimit;	 //������Χ
  s16 errorLimit;      //����Χ
  u16 checkSum;		 //У��ֵ
} temperatureAbnormalParam;

/*! TEC����/����������� */
__packed
typedef struct
{
  u32 hotCount;	    //���ȴ�������
  u32 coolCount;      //�����������
  u32 cycleCount;     //TECѭ��ʹ�ô�������
  u16 checkSum;		 //У��ֵ
} TecWorkParam;

/*! Ŀ���¶ȵ�Ե�У׼���� */
__packed
typedef struct
{
  u8 enableFlag;
  s32  calData[MAX_CALI_NUM][2];
  u16  checkSum;
} calTarTempParam;

/*! ��ǰ�¶ȱ��� ���/��Сֵ��������Χ�� */
__packed
typedef struct
{
  s32 maxCurTemp;      //��ǰ�¶� ����
  s32 minCurTemp;      //��ǰ�¶� ����
  u16 checkSum;        //У���
} curTempRange;


/*! ------------�洢��eeprom�еĲ���  end ----------------------------*/

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
  s32 targetTempMax;      //Ŀ���¶����ֵ
  s32 targetTempMin;      //Ŀ���¶���Сֵ
  s32 maxErrorT;          //��ǰ�¶ȹ��Ϸ�Χ �����ֵ��
  s32 minErrorT;          //��ǰ�¶ȹ��Ϸ�Χ ����Сֵ��
  s32 limitWarningT_1;    //�¶ȱ�����Χ(+-1��)
  s32 limitWarningT_2;    //�¶ȱ�����Χ(+-2��)
  s32 limitWarningT;      //�¶ȱ�����Χ(+-0.5��)
  stTempProcess  stTempProcessFlag;  //�¶ȴ����ʾ
  Count stTempCount;	               //�¶ȼ���
  s32 LastT;       //last temperature
  filterFlag pFlag;
  u8 channelEnableFlag;
  bool firstSetTargetFlag;
	
	s32 channelTECSTopTime[4]; //TEC�ر�ʱ��
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
* @brief  ��ȡ�¿ذ�汾
*******************************************************************************/
extern s32 RequireTempBoardVersion( void );

/*****************************************************************************
* @name s32 temperatureControl(u8 channelNum,s32 currentTemp,tempControl *stTempControl);

* @brief  �¶ȿ��ƺ���
*
* @param
*    Input   ͨ����
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
* @brief  �¶��˲�������
*
* @param
*    Input   currentTemp   ��ǰ�¶�ֵ
*    Input   targetValue   Ŀ���¶�ֵ
*    Input   lastTemp      �ϴ��¶�ֵ
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
* @brief:      PID����
*
* @param  :
     Inputs:  Ŀ���¶�ֵ
     Outputs: PID���ں�������PWM����

* @return   True �ɹ�   False ʧ��
*
**********************************************************************************/
extern s32 PIDRegulator( s32 currentTemp, s32 targetValue, u8 channelNum, PidInfoDef *pPidInfo );
//extern s32 PIDRegulator(u8 channelNum,s32 currentTemp,s32 targetValue);

/*****************************************************************************
* @name   s32 SetTargetTemperature(u8 channelNum,u32 Target)
* @brief  ����Ŀ���¶�
*
* @param
*    Input   Target      Ŀ���¶�
*    Output  ��
*
* @return
*
*******************************************************************************/
//extern u16 SetTargetTemperature(u8 channelNum,s32 target);
//extern u16 SetTargetTemperature(u8 channelNum,s32 target,tempControl *stTempControl);
extern u16 SetTargetTemperature( u8 channelNum, s32 target, tempControl *stTempControl, PidInfoDef *pPidInfo );
/*****************************************************************************
* @name   u32 GetTargetTemperature(u8 channelNum)
* @brief  ��ȡĿ���¶�
*
* @param
*    Input   channelNum

*    Output  Target      Ŀ���¶�
*
* @return
*
*******************************************************************************/
//extern s32 GetTargetTemperature(u8 channelNum);
u16 GetTargetTemperature_test( u8 channelNum, s32 *targTemp, PidInfoDef *pPidInfo );
u16 GetTargetTemperature( u8 channelNum, s32 *targTemp, PidInfoDef pPidInfo );
/*****************************************************************************
* @name  u16 GetCurrentTemperature(u8 channelNum��s32* pData)
* @brief  ��ȡ��ǰͨ���¶�ֵ
*
* @param
*    Input   channelNum  ͨ����
*    Output  ��
*
* @return   True �ɹ�   False ʧ��
*
*******************************************************************************/
extern u16 GetCurrentTemperature( u8 channelNum, s32* pData );
extern u16 GetSensorValue( u8 channelNum, s32* pData, tempControl stTempControl );
/*****************************************************************************
* @name   s32 GetChannelEnableFlag(u8 channelNum ,bool* channelEnableFlag)
* @brief  ��ȡͨ��ʹ�ܣ���ʹ�ܣ���ʾ
*
* @param
*    Input   channelNum             ͨ����
             channelEnableFlag      ͨ��ʹ�ܱ�ʾ
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
* @brief  ���ñ����¶�������
*
*******************************************************************************/
extern u16 SetTargetTempRange( u8 channelNum, s32 upperLimit, s32 lowerLimit, tempControl *stTempControl );

/*****************************************************************************
* @name   u32 GetWarningLimitedTemperature(u8 channelNum)
* @brief  ��ȡ�����¶�������
*
*******************************************************************************/
//extern s32 GetWarningLimitedTemperature(u8 channelNum,s32 upperLimit,s32 lowerLimit,tempControl stTempControl);
extern u16 GetTargetTempRange( u8 channelNum, tempControl* stTempControl );
/*****************************************************************************
* @name   u32 SetErrorLimitedTemperature(u8 channelNum)
* @brief  ���ñ����¶�������
*
*******************************************************************************/
extern u16 SetCurTemperRange( u8 channelNum, s32 upperLimit, s32 lowerLimit, tempControl *stTempControl );

/*****************************************************************************
* @name   u32 GetErrorLimitedTemperature(u8 channelNum)
* @brief  ��ȡ�����¶�������
*
*******************************************************************************/
extern u16 GetCurTemperRange( u8 channelNum, tempControl* stTempControl );
/*****************************************************************************
* @name s32 CalibrationTargetTemperature1(bool enableFlag, s32* targetTemp,s32* calibrationTemperature,s32 *pTargetValue)

* @brief  Ŀ���¶�У׼(У׼���ֵͨ��SBC���ã��洢��firmware��eeprom��)
*
* @param
*    Input   targetTemp       ��ȡ����Ŀ���¶�


*    Output  *pTargetValue    У׼���Ŀ���¶�
*
* @return   У׼���Ŀ���¶�
*
*******************************************************************************/
extern u16 CalibrationTargetTemperature( calTarTempParam *stTemp );

/*****************************************************************************
* @name   s32 TemperatureParamInit(void)
* @brief  �¶Ȳ�����ʼ��
*
* @param  ��
*
* @return  true:���� false:�쳣
*
*******************************************************************************/
extern s32 TemperatureParamInit( void );

extern u16 LiquidPumpContrl( u8 channelNum, bool enableFlag );

extern u16 LiquidFanContrl( u8 channelNum, bool enableFlag );

/*****************************************************************************
* @name   bool StopAllOfTec(void)
* @brief  ֹͣ���е�TEC
*
* @param
*    Input   ��
*    Output  ��
*
* @return true���ɹ�  false��ʧ��
*
*******************************************************************************/
extern s32 StopAllOfTec( void );


extern s32 Heating( u8 channelNum, bool ableFlag, u8 percent );
extern s32 Cool( u8 channelNum, bool ableFlag, u8 percent );


extern s32 TempControlSystem( u8 channelNum, tempControl *stTempControl, PidInfoDef *pPidInfo );
void resetTCParamer( tempControl *stTempControl );
#endif

