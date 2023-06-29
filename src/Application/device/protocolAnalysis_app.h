/***********************************************************
Module Name: protocolAnalysis_app.h
Description: Э���飬���� ---Ӧ�ò�ʵ��
Module Date: 12-08-2016
Module Author: Firmware-Team
Others:
***********************************************************/

#ifndef  _PROTOCOLANALYSIS_APP_H_
#define  _PROTOCOLANALYSIS_APP_H_

#include "stm32f3xx.h"
#include "protocolDeal.h"
#include "PID.h"

extern ProtocolDealHandle  protocolAnalysisHandle, protocolHandle;
extern Frame stActiveRepResult;


#define  PROTOCAL_PARAM1      0xFF

/*! *************** ���ڴ��ݲ����Ľṹ�� ***************** */
/*! ����PID���� */
__packed
typedef struct
{
  u8 parameter1;          //����1
  u8 channelNum;          //ͨ����
  u8 segmentNum;
  PIDParam stParam;

} setPIDParam;

/*! ����Ŀ���¶������� */
__packed
typedef struct
{
  u8 parameter1;          //����1
  u8 channelNum;          //ͨ����
  s32 upperLimit;         //����
  s32 lowerLimit;         //����
} SetTargetRangeLimit;

/*! �����¶ȹ���(ERROR)�����ޣ�Ԥ����  */
__packed
typedef struct
{
  u8 parameter1;          //����1
  u8 channelNum;          //ͨ����
  s32 upperLimit;         //����
  s32 lowerLimit;         //����
} SetErrorLimit;

/*! ����Ŀ���¶� */
__packed
typedef struct
{
  u8 parameter1;          //����1
  u8 channelNum;          //ͨ����
  s32 targetTempValue;     //Ŀ���¶�

} SetTargetTempParam;

/*! ʹ��/��ʹ���¶ȿ��� */
__packed
typedef struct
{
  u8 parameter1;          //����1
  u8 channelNum;          //ͨ����
  u8 ableFlag;            //ʹ�ܱ�ʾ

} EnableParam;


/*! ��/�ر������ */
__packed
typedef struct
{
  u8 parameter1;          //����1 ������ͬ�ı�
  u8 ableFlag;            //ʹ�ܱ�ʾ

} EnablePump;

/*! ��/�ر�������� */
__packed
typedef struct
{
  u8 parameter1;          //����1  ������ͬ�ı�
  u8 ableFlag;            //ʹ�ܱ�ʾ

} EnableFan;

/*! ��/�رճ�����ˮ�� */
__packed
typedef struct
{
  u8 parameter1;          //����1  ������ͬ�ı�
  u8 ableFlag;            //ʹ�ܱ�ʾ

} EnableRefrigPump;


/*! **************** ���������֡�Ľṹ�� *********************** */

/*! ��ѯ�¿ذ�汾 */
__packed
typedef struct
{
  u8   parameter1;          //����1
  u8   channelNum;          //ͨ����
  u16  warningDiff;         //������ֵ
  u16  errDiff;	          //������ֵ

} SetSteadyDiff;

/*! ����PID���ò��� */
__packed
typedef struct
{
  u8  parameter1;          //����1
  u8  channelNum;
  //	s32 maxOutput;           //PID�������
  //	s32 minOutput;           //PID�������
  //	s16 maxIntegralOutput;
  //	s16 minIntegralOutput;
  //	u8  thresholdValue;
  //	s32 targetTemperature;   //Ŀ���¶�
  PIDCalculate  stPidCalParam;
} SetConfigParam;

/*! **************** ���������֡�Ľṹ�� *********************** */

/*! ��ѯ�¿ذ�汾 */
//__packed
//typedef struct
//{
//	u8 status;
//	u8 parameter1;          //����1
//	u8 mainVer;            //���汾��
//	u8 subVer;             //�ΰ汾��
//	u8 editVer;            //�޸ĺ�
//	u32 SVNVer;            //SVN�汾��
//	u8 proVerMain;         //ͨ��Э�����汾��
//	u8 proVersec;          //ͨ��Э��ΰ汾��
//	u8 configVerMain;      //�����ļ����汾��
//	u8 configVerSec;       //�����ļ��ΰ汾��
//}RequireVerResult;

/*! ��ȡPID����  */
__packed
typedef struct
{
  u8 status;
  u8 parameter1;          //����1
  u8 channelNum;          //ͨ����
  u8 segemetNum;
  PIDParam stParam;

} GetPIDParameter;

/*! ��ȡĿ���¶�������  */
__packed
typedef struct
{
  u8 status;
  u8 parameter1;          //����1
  u8 channelNum;          //ͨ����
  s32 upperLimit;         //����
  s32 lowerLimit;         //����

} GetTargetRangeLimit;

/*! ��ȡ�����¶ȹ��������� */
__packed
typedef struct
{
  u8 status;
  u8 parameter1;          //����1
  u8 channelNum;          //ͨ����
  s32 upperLimit;         //����
  s32 lowerLimit;         //����

} GetErrorLimit;

/*! ��ȡȫ��ͨ���¶ȣ�˳���壬�ֽڶ��壬����Լ���� */
__packed
typedef struct
{
  u8 status;
  u8 parameter1;          //����1
  u8 channelNum;          //ͨ����
  s32 currentTemp;        //��ȡ�����¶�ֵ

} GetCurTemp;


/*! ��ȡĿ���¶� */
__packed
typedef struct
{
  u8 status;
  u8 parameter1;          //����1
  u8 channelNum;          //ͨ����
  s32 targetTemp;        //��ȡ�����¶�ֵ

} GetTargetTemp;

/*! ��ȡ��̬ʱ������/�����¶Ȳ�ֵ */
__packed
typedef struct
{
  u8   status;              //״̬
  u8   parameter1;          //����1
  u8   channelNum;          //ͨ����
  u16  warningDiff;         //������ֵ
  u16  errDiff;	          //������ֵ

} GetSteadyDiff;

/*! ��ȡTEC�������� */
__packed
typedef struct
{
  u8   status;              //״̬
  u8   parameter1;          //����1
  u8   channelNum;          //ͨ����
  u32  hotCount;            //���ȴ���
  u32  coolCount;           //�������
  u32  cycleCount;          //ѭ������

} GetTecParam;

/*! ��ȡPID���ò��� */
__packed
typedef struct
{
  u8  status;              //״̬
  u8  parameter1;          //����1
  u8  channelNum;          //ͨ����
  //	s32 maxOutput;           //PID�������
  //	s32 minOutput;           //PID�������
  //	s16 maxIntegralOutput;
  //	s16 minIntegralOutput;
  //	u8  thresholdValue;
  PIDCalculate  stPidCalParam;
} GetConfigParam;

/*! ��ȡ�¶ȿ���ʹ�ܱ�ʾ */
__packed
typedef struct
{
  u8 status;           //״̬
  u8 parameter1;       //����1
  u8 channelNum;       //ͨ����
  u8 flag;             //ʹ������ʾ

} GetTcFlag;


/*��******************************** */


typedef enum
{
  RESULT_LENTH_ERR = 2,                //֡�����쳣
  RESULT_CHANNEL_ERR,                  //ͨ���쳣
  RESULT_ADC_ERR,                      //
  RESULT_TARGET_UPPER_ERR,             //
  RESULT_TARGET_LOWER_ERR,             //
  RESULT_TARGET_UPPER_ERR_B,           //
  RESULT_TARGET_LOWER_ERR_B,           //
  RESULT_CUR_UPPER_ERR,                //
  RESULT_CUR_LOWER_ERR,                //
  RESULT_CUR_UPPER_ERR_B,              //
  RESULT_CUR_LOWER_ERR_B,              //
  RESULT_FRAME_INEXISTENCE,            //����֡������
  RESULT_PARAM1_ABNOMAL,                //
  RESULT_PARAM_ABNOMAL,                //֡�����쳣

} ResultStatusDef;

typedef enum
{
  STATUS_OK,
  STATUS_INVALID_ERRORCODE,        //��Ч�Ĵ�����
  STATUS_INVALID_FRAMESTATUS,	     //��Ч��֡״̬

} ReqErrCodeStatusDef;

typedef enum
{
  RESULT_FRAME_BUILD_OK,					//���֡��������
  RESULT_FRAME_BUILD_ERROR,				//���֡��������
  RESULT_TX_PARAMETER_ERROR,			//֡���ͺ��������������Ӧ�ô���
  RESULT_TX_ERROR,								//֡���ʹ���Ӧ�ô���
  RESULT_FRAME_TX_OK,							//֡������ȷ��Ӧ�ô���
  RESULT_TX_TIMEOUT,							//֡���ͳ�ʱ��Ӧ�ô���
  RESULT_ACTIVE_REPORT_FRAME_OK,	//���������ϱ�֡OK
  RESULT_ACTIVE_REPORT_FRAME_ERR,	//���������ϱ�֡����

} BuildAppResultStatusDef;

extern void ConfigProtocolParam( void );

/*! ����Command Code  FrameTypeΪ0x04  */
BuildAppResultStatusDef AnalysisCommandCodeNoResult( Frame *RxFrame );

extern BuildAppResultStatusDef AnalysisCommandCode( Frame *RxFrame );
/*! ����FrameType  */
extern s32 AnalysisFrameType( Frame *RxFrame );

extern BuildAppResultStatusDef BuildAppResultFrame_test( u8 FramStatus, const Frame * InputRxFrame, void *InputData, Frame * OutputTxFrame );
extern BuildAppResultStatusDef BuildAppResultFrame( u16 FramStatus, const Frame * InputRxFrame, void *InputData, Frame * OutputTxFrame );

/*
*********************************************************************************************************
*                                          SendNormalData
*
* Description : �ϱ���������
*
* Arguments   : FrameStatus  ״̬
*				InputRxFrame     ���յ�������
*				OutputTxFrame    ���������
*				pProtocolDealHandle    �ֽ���Э����
*				timeOut ��ʱʱ��
*
* Returns     : FrameStatusDef ֡״̬
*
* Notes       :
*********************************************************************************************************
*/
BuildAppResultStatusDef SendNormalData( u16 FrameStatus, const Frame * InputRxFrame, void *InputData, Frame *OutputTxFrame, ProtocolDealHandle *pProtocolDealHandle, u32 timeOut );
/*
*********************************************************************************************************
*                                          SendAbnormalData
*
* Description : �ϱ��쳣����
*
* Arguments   : ErrorCode  ������
*				pProtocolDealHandle �ֽ���Э����
*				OutputRxFrame ���յ�������
*				FrameIndex    ֡���к�
*				timeOut ��ʱʱ��
*
* Returns     : FrameStatusDef ֡״̬
*
* Notes       :
*********************************************************************************************************
*/
extern BuildAppResultStatusDef SendAbnormalData( s16 ErrorCode, ProtocolDealHandle *pProtocolDealHandle, Frame *OutputTxFrame, u16 *FrameIndex, u32 timeOut );

#endif