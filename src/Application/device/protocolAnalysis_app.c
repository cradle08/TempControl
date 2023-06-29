/***********************************************************
Module Name: protocolAnalysis_app.c
Description: 协议检查，解析
Module Date: 12-08-2016
Module Author: Firmware-Team
Others:
***********************************************************/

#include "includes.h"

#include  "protocolAnalysis_app.h"
#include  "temperature_control.h"
#include  "PID.h"
#include  "Version.h"
#include  "errcode.h"
#define  OBJECT_UNIT      (0x0B)
#define  SBC              (0x01)


/**************************************************************************************
** 通讯协议命令
**************************************************************************************/
#define  BUFF_SIZE_MAX    (256ul)
//#define  REQUIRE_VERSION                  (0x01)      //获取通信协议的版本
#define  SET_PID_PARAMETERS	                (0x02)      //设置PID参数
#define  SET_TARGET_TEMP_RANGE	            (0x03)      //设置目标温度上下限
#define  SET_CURRENT_TEMP_RANGE	            (0x04)      //设置当前温度上下限
#define  SET_TARGET_TEMPERATURE	            (0x05)      //设置目标温度
#define  ENABLE_DISABLE_TC	                (0x06)      //使能/不使能温度控制
#define  GET_PID_PARAMETERS	                (0x07)      //获取PID参数
#define  GET_TARGET_TEMP_RANGE	            (0x08)      //获取目标温度上下限
#define  GET_CUR_TEMP_RANGE	        		    (0x09)      //获取出现温度故障上下限
#define  GET_TARGET_TEMPERATURE	            (0x0A)      //获取目标温度
#define  GET_CURRENT_TEMPERATURE	          (0x0B)      //获取当前通道温度
#define  SET_PT100_ORIGIN_TABLE	            (0x0C)      //设置PT100原始值     
#define  SET_PT100_ADJUST_TABLE	            (0x0D)      //设置PT100校准值    
#define  SET_BOARD_ADJUST_TABLE             (0x0E)      //设置板卡校准值    
#define  SET_STAGE_ADJUST_TABLE	            (0x0F)      //设置平台校准值 
#define  SET_DELTA	                        (0x10)      //设置单位时间内温度变化差值     
#define  SET_TEMPERATURE_DELTA_LIMIT	      (0x11)      //设置单位时间内温度变化差值
#define  TARGET_CALIBRATION                 (0x12)      //目标温度校准(主要是点对点的)
#define  PLATFORM_CALIBRATION               (0x13)      //平台校准                        
#define  GET_ENVIRONMENT_TEMPERATURE        (0x14)      //获取环境温度   

#define  CONTROL_LIQUID_PUMP                (0x15)      //控制液泵
#define  CONTROL_LIQUID_FAN                 (0x16)      //控制制冷风扇
#define  SET_STEADY_ERROR                   (0x17)      //设置稳态时，报警/报错温度差值（报警：0.5°，报错：1°）
#define  GET_STEADY_ERROR                   (0x18)      //获取稳态时，报警/报错温度差值（报警：0.5°，报错：1°）
#define  GET_TEC_WORK_COUNT                 (0x19)      //获取TEC工作次数
#define  SET_PIDCONFIG_PARAM                (0x1A)      //设置PID配置参数
#define  GET_PIDCONFIG_PARAM                (0x1B)      //获取PID配置参数
#define  GET_TC_FLAG                        (0x1C)      //获取温度控制使能标示 
#define  CONTROL_REFRIG_PUMP                (0x1D)      //控制抽冷凝水泵

#define  ANOMALY_REPORT                     (0xDF)
#define  REQUIRE_VERSION                    (0xE0)      //获取版本
#define  UPGRADE                            (0xE1)

#define  FRAME_ACTIVE_REPORT_LENTH          (0x08u)


/*数据协议结构体*/
ProtocolDealHandle  protocolAnalysisHandle;

u8 uartReadBuff[BUFF_SIZE_MAX];
u8 uartWriteBuff[BUFF_SIZE_MAX];

Frame stResult, stActiveRepResult;

ReqErrCodeStatusDef RequireErrCode( u8 FramStatus, u16 *pErrCode );

/**************************************************************************************
** 函数名称: Uart1ReadData
** 参数    : InputBuff：输入buff  InputLen：输入长度   OutputLen：输出长度
** 函数功能: UART1读取数据
** 返回值  : u32(status): 返回状态
**************************************************************************************/
u32 Uart1ReadData( u8 *InputBuff, u32 InputLen, u32 *OutputLen )
{
  BspStatusTypeDef status;

  status = BspUartReceiveIT( &BspUart1Handle, InputBuff, InputLen, OutputLen );
  switch( status )
  {
    case BSP_PARAMETER_ERROR:
    {
      *OutputLen = 0;
    }
    break;
		
    case BSP_RX_HAL_ERROR:
    {
      *OutputLen = 0;
    }
    break;
		
    case BSP_RX_QUEUE_EMPTY:
    {
      *OutputLen = 0;
    }
    break;
		
    case BSP_RX_QUEUE_OVER:
    {
      SaveLog( NULL, 0 );
    }
    break;
		
    case BSP_OK:
    {
    }
    break;

    default:
      *OutputLen = 0;
      break;
  }
  return status;
}

/**************************************************************************************
** 函数名称: Uart1SendData
** 参数     : InputBuff：输入的数据buff    InPutLen:输出的数据长度
** 函数功能: Uart1发送数据Buff
** 返回值   : u32(status): 返回状态
**************************************************************************************/
u32 Uart1SendData( u8 *InputBuff, u32 InPutLen )
{
  BspStatusTypeDef status;
  status = BspUartTransmitIT( &BspUart1Handle, InputBuff, InPutLen );
  return status;
}

/**************************************************************************************
** 函数名称: ConfigProtocolParam
** 参数     : NULL
** 函数功能: 初始化协议信息
** 返回值   : NULL
**************************************************************************************/
void ConfigProtocolParam( void )
{
  protocolAnalysisHandle.LocalObjectUnit = OBJECT_UNIT;
  protocolAnalysisHandle.TagetObjectUnit = SBC;

  /*协议结构体里面的读取UART1 ReadBuff数据*/
  protocolAnalysisHandle.ReadBuff     = uartReadBuff;
  protocolAnalysisHandle.ReadBuffSize = BUFF_SIZE_MAX ;
  protocolAnalysisHandle.ReadData     = Uart1ReadData;
  protocolAnalysisHandle.ReadDataLen  = 0;

  /*协议结构体里面的写UART1 SendBuff数据*/
  protocolAnalysisHandle.WriteData     = Uart1SendData;
  protocolAnalysisHandle.WriteDataLen  = 0;
  protocolAnalysisHandle.WriteBuff     = uartWriteBuff;
  protocolAnalysisHandle.WriteBuffSize = BUFF_SIZE_MAX;

  /*协议结构体里面的时间数据*/
  protocolAnalysisHandle.TimeoutBetweenBytes.TimeValue 	= 0x0a;
  protocolAnalysisHandle.TimeoutBetweenBytes.TimeUnit  	= 0x02;
  protocolAnalysisHandle.TimeoutBetweenBytes.TimingFlag = 0x00;
  protocolAnalysisHandle.TimeoutBetweenBytes.time				= 0xFFFFFFFF;

  /*协议结构体里面的CRC数据*/
  protocolAnalysisHandle.CrcInit        = crcInit;
  protocolAnalysisHandle.Crc16          = ( CrcFun )crcFast;

  protocolAnalysisHandle.LastFrameIndex = 0xFFFF;

  FrameDealInit( &protocolAnalysisHandle );
}

/**************************************************************************************
** 函数名称: AnalysisFrameType
** 参数     : NULL
** 函数功能: 分析协议框架类型
** 返回值   : NULL
**************************************************************************************/
s32 AnalysisFrameType( Frame *RxFrame )
{
  BuildAppResultStatusDef	 status = RESULT_FRAME_BUILD_OK; /*结果帧建立正常*/

  switch( RxFrame->Header.FrameType )
  {
    case LAUNCH_FRAME_1_TYPE: /*发送类型1*/
    {
      status = AnalysisCommandCode( RxFrame );
    }
    break;

    case RESPONSE_FRAME_TYPE: /*回复类型*/
    {

    }
    break;

    case RESULT_FRAME_TYPE: 	/*结果类型*/
    {


    }
    break;

    case LAUNCH_FRAME_4_TYPE:/*发送类型4*/
    {
      status = AnalysisCommandCodeNoResult( RxFrame );
    }
    break;

    default:
      break;
  }

  return status;
}

/**************************************************************************************
** 函数名称: AnalysisCommandCodeNoResult
** 参数    : RxFrame：协议结构体指针
** 函数功能: 分析命令代码无结果
** 返回值  : BuildAppResultStatusDef：返回分析的结果类型
**************************************************************************************/
BuildAppResultStatusDef AnalysisCommandCodeNoResult( Frame *RxFrame )
{
  BuildAppResultStatusDef	 status = RESULT_FRAME_BUILD_OK;

  if( OBJECT_UNIT == RxFrame->Header.ObjectUnit )
  {
    switch( RxFrame->Header.CommandCode )
    {
      default:
        break;
    }
  }
  return status;

}


/**************************************************************************************
** 函数名称: AnalysisCommandCode
** 参数    : RxFrame：协议结构体指针
** 函数功能: 分析命令代码
** 返回值  : BuildAppResultStatusDef：返回分析的结果类型
**************************************************************************************/
BuildAppResultStatusDef AnalysisCommandCode( Frame *RxFrame )
{
  BuildAppResultStatusDef	 status = RESULT_FRAME_BUILD_OK;
  u16 funStatus = 0;

  if( OBJECT_UNIT == RxFrame->Header.ObjectUnit )
  {
    switch( RxFrame->Header.CommandCode )
    {
      case REQUIRE_VERSION:       //获取版本
      {
        u32 verLen = 0;
        if( RxFrame->Header.FrameLen != 6 )
        {
          funStatus = ERR_FRAME_LENTH_ERROR;

          status = SendNormalData( funStatus, RxFrame, NULL, &stResult, &protocolAnalysisHandle, 0 );
        }
        else
        {

          switch( RxFrame->FramDate[0] )
          {
            case 0x01:
            {
              unsigned char requireVersion[11] ;

              verLen = GetSoftwareVersion( requireVersion );

              if( verLen <= 0 )
              {
                funStatus = ERR_VIRSION;
              }
              else
              {
                funStatus  = SUCCESS1;
              }

              status = SendNormalData( funStatus, RxFrame, &requireVersion, &stResult, &protocolAnalysisHandle, 0 );

            }
            break;

            case 0x02:
            {

            }
            break;

            case 0x03:
            {

            }
            break;

            default:

              status = SendNormalData( ERR_PARAM1_ABNOMAL, RxFrame, NULL, &stResult, &protocolAnalysisHandle, 0 );
              break;
          }
        }
      }
      break;

      case SET_PID_PARAMETERS:    //设置PID参数
      {
        //				s32 funStatus = 0;
        setPIDParam *temp = NULL;

        if( RxFrame->Header.FrameLen != 20 )
        {

          funStatus = ERR_FRAME_LENTH_ERROR;

          status = SendNormalData( funStatus, RxFrame, NULL, &stResult, &protocolAnalysisHandle, 0 );

        }
        else
        {
          temp = ( setPIDParam * )RxFrame->FramDate;

          switch( temp->channelNum )
          {
            case CHA:
            {
              funStatus = SetPIDParameters( temp->channelNum, temp->segmentNum, temp->stParam, &PidInfoA );
            }
            break;

            case CHB:
            {
              funStatus = SetPIDParameters( temp->channelNum, temp->segmentNum, temp->stParam, &PidInfoB );
            }
            break;

            case CHC:
            {
              funStatus = SetPIDParameters( temp->channelNum, temp->segmentNum, temp->stParam, &PidInfoC );
            }
            break;

            case CHD:
            {
              funStatus = SetPIDParameters( temp->channelNum, temp->segmentNum, temp->stParam, &PidInfoD );
            }
            break;

            default:
              funStatus = ERR_CHANNEL_NUM;
              break;
          }
          status = SendNormalData( funStatus, RxFrame, NULL, &stResult, &protocolAnalysisHandle, 0 );
        }
      }
      break;

      case SET_TARGET_TEMP_RANGE: //设置目标温度上下限
      {
        SetTargetRangeLimit *stSetWarningLimit = NULL;

        if( RxFrame->Header.FrameLen != 15 )
        {

          funStatus = ERR_FRAME_LENTH_ERROR;

          status = SendNormalData( funStatus, RxFrame, NULL, &stResult, &protocolAnalysisHandle, 0 );
        }
        else
        {
          stSetWarningLimit = ( SetTargetRangeLimit * )RxFrame->FramDate;
          switch( stSetWarningLimit->channelNum )
          {
            case CHA:
            {
              funStatus = SetTargetTempRange( stSetWarningLimit->channelNum, stSetWarningLimit->upperLimit,
                                              stSetWarningLimit->lowerLimit, &stTemperatureControlA );

            }
            break;

            case CHB:
            {
              funStatus = SetTargetTempRange( stSetWarningLimit->channelNum, stSetWarningLimit->upperLimit,
                                              stSetWarningLimit->lowerLimit, &stTemperatureControlB );

            }
            break;

            case CHC:
            {
              funStatus = SetTargetTempRange( stSetWarningLimit->channelNum, stSetWarningLimit->upperLimit,
                                              stSetWarningLimit->lowerLimit, &stTemperatureControlC );

            }
            break;

            case CHD:
            {
              funStatus = SetTargetTempRange( stSetWarningLimit->channelNum, stSetWarningLimit->upperLimit,
                                              stSetWarningLimit->lowerLimit, &stTemperatureControlD );

            }
            break;

            default:
              funStatus = ERR_CHANNEL_NUM;
              break;

          }

          status = SendNormalData( funStatus, RxFrame, NULL, &stResult, &protocolAnalysisHandle, 0 );
        }
      }

      break;

      case SET_CURRENT_TEMP_RANGE: //设置当前温度上下限
      {
        SetErrorLimit *stSetErrorLimit;

        if( RxFrame->Header.FrameLen != 15 )
        {
          funStatus = ERR_FRAME_LENTH_ERROR;

          status = SendNormalData( funStatus, RxFrame, NULL, &stResult, &protocolAnalysisHandle, 0 );
        }
        else
        {
          stSetErrorLimit = ( SetErrorLimit * )RxFrame->FramDate;
          switch( stSetErrorLimit->channelNum )
          {
            case CHA:
            {
              funStatus = SetCurTemperRange( stSetErrorLimit->channelNum, stSetErrorLimit->upperLimit,
                                             stSetErrorLimit->lowerLimit, &stTemperatureControlA );

            }
            break;

            case CHB:
            {
              funStatus = SetCurTemperRange( stSetErrorLimit->channelNum, stSetErrorLimit->upperLimit,
                                             stSetErrorLimit->lowerLimit, &stTemperatureControlB );

            }
            break;

            case CHC:
            {
              funStatus = SetCurTemperRange( stSetErrorLimit->channelNum, stSetErrorLimit->upperLimit,
                                             stSetErrorLimit->lowerLimit, &stTemperatureControlC );

            }
            break;

            case CHD:
            {
              funStatus = SetCurTemperRange( stSetErrorLimit->channelNum, stSetErrorLimit->upperLimit,
                                             stSetErrorLimit->lowerLimit, &stTemperatureControlD );

            }
            break;

            default:
              funStatus = ERR_CHANNEL_NUM;
              break;

          }

          status = SendNormalData( funStatus, RxFrame, NULL, &stResult, &protocolAnalysisHandle, 0 );
        }
      }
      break;

      case SET_TARGET_TEMPERATURE: //设置目标温度
      {

        SetTargetTempParam *SetTargetTempParamTemp;

        if( RxFrame->Header.FrameLen != 11 )
        {
          funStatus = ERR_FRAME_LENTH_ERROR;

          SendNormalData( funStatus, RxFrame, NULL, &stResult, &protocolAnalysisHandle, 0 );
        }
        else
        {
          SetTargetTempParamTemp = ( SetTargetTempParam * )RxFrame->FramDate;
          switch( SetTargetTempParamTemp->channelNum )
          {
            case CHA:
            {
              funStatus = SetTargetTemperature( SetTargetTempParamTemp->channelNum, SetTargetTempParamTemp->targetTempValue,
                                                &stTemperatureControlA, &PidInfoA );
            }
            break;

            case CHB:
            {
              funStatus = SetTargetTemperature( SetTargetTempParamTemp->channelNum, SetTargetTempParamTemp->targetTempValue,
                                                &stTemperatureControlB, &PidInfoB );
              if( funStatus != SUCCESS1 )
                funStatus =( funStatus ==ERR_TARGET_TEMP_OVER_UPPER_LIMIT )? ERR_TARGET_TEMP_OVER_UPPER_LIMIT_B:ERR_TARGET_TEMP_OVER_LOWER_LIMIT_B;

            }
            break;

            case CHC:
            {
              funStatus = SetTargetTemperature( SetTargetTempParamTemp->channelNum, SetTargetTempParamTemp->targetTempValue,
                                                &stTemperatureControlC, &PidInfoC );

              if( funStatus != SUCCESS1 )
                funStatus =( funStatus ==ERR_TARGET_TEMP_OVER_UPPER_LIMIT )? ERR_TARGET_TEMP_OVER_UPPER_LIMIT_C:ERR_TARGET_TEMP_OVER_LOWER_LIMIT_C;
            }
            break;

            case CHD:
            {
              funStatus = SetTargetTemperature( SetTargetTempParamTemp->channelNum, SetTargetTempParamTemp->targetTempValue,
                                                &stTemperatureControlD, &PidInfoD );
              if( funStatus != SUCCESS1 )
                funStatus = ( funStatus ==ERR_TARGET_TEMP_OVER_UPPER_LIMIT )? ERR_TARGET_TEMP_OVER_UPPER_LIMIT_D:ERR_TARGET_TEMP_OVER_LOWER_LIMIT_D;
            }
            break;

            default:
              funStatus = ERR_CHANNEL_NUM;
              break;

          }
          status = SendNormalData( funStatus, RxFrame, NULL, &stResult, &protocolAnalysisHandle, 0 );
        }

      }
      break;

      case ENABLE_DISABLE_TC:			 //使能/不使能温度控制
      {
        //				s32 funStatus = 0;
        EnableParam *stEnableFlag;
				
        if( RxFrame->Header.FrameLen != 8 )
        {

          funStatus = ERR_FRAME_LENTH_ERROR;

          SendNormalData( funStatus, RxFrame, NULL, &stResult, &protocolAnalysisHandle, 0 );
        }
        else
        {
          stEnableFlag = ( EnableParam * )RxFrame->FramDate;
          switch( stEnableFlag->channelNum )
          {
            case CHA:
            {
              funStatus = SetChannelEnableFlag( stEnableFlag->channelNum, stEnableFlag->ableFlag, &stTemperatureControlA );
            }
            break;

            case CHB:
            {
              funStatus = SetChannelEnableFlag( stEnableFlag->channelNum, stEnableFlag->ableFlag, &stTemperatureControlB );
            }
            break;

            case CHC:
            {
              funStatus = SetChannelEnableFlag( stEnableFlag->channelNum, stEnableFlag->ableFlag, &stTemperatureControlC );
            }
            break;

            case CHD:
            {
              funStatus = SetChannelEnableFlag( stEnableFlag->channelNum, stEnableFlag->ableFlag, &stTemperatureControlD );
            }
            break;

            default:
              funStatus = ERR_CHANNEL_NUM;
              break;
          }
          status = SendNormalData( funStatus, RxFrame, NULL, &stResult, &protocolAnalysisHandle, 0 );
        }
      }

      break;

      case GET_PID_PARAMETERS:    //获取PID参数
      {
        //				s32 funStatus = 0;
        GetPIDParameter  stGetPIDParameter= {0};

        if( RxFrame->Header.FrameLen != 8 )
        {
          funStatus = ERR_FRAME_LENTH_ERROR;

          status = SendNormalData( funStatus, RxFrame, NULL, &stResult, &protocolAnalysisHandle, 0 );
        }
        else
        {
          switch( RxFrame->FramDate[1] )
          {
            case CHA:
            {
              funStatus = GetPIDParam( RxFrame->FramDate[1], RxFrame->FramDate[2], &stGetPIDParameter.stParam, PidInfoA );
            }
            break;

            case CHB:
            {
              funStatus = GetPIDParam( RxFrame->FramDate[1], RxFrame->FramDate[2], &stGetPIDParameter.stParam, PidInfoB );
            }
            break;

            case CHC:
            {
              funStatus = GetPIDParam( RxFrame->FramDate[1], RxFrame->FramDate[2], &stGetPIDParameter.stParam, PidInfoC );
            }
            break;

            case CHD:
            {
              funStatus = GetPIDParam( RxFrame->FramDate[1], RxFrame->FramDate[2], &stGetPIDParameter.stParam, PidInfoD );
            }
            break;

            default:
              funStatus = ERR_CHANNEL_NUM;
              break;
          }

          stGetPIDParameter.parameter1 = RxFrame->FramDate[0];
          stGetPIDParameter.channelNum = RxFrame->FramDate[1];
          stGetPIDParameter.segemetNum = RxFrame->FramDate[2];
          status = SendNormalData( funStatus, RxFrame, &stGetPIDParameter, &stResult, &protocolAnalysisHandle, 0 );
        }
      }
      break;

      case GET_TARGET_TEMP_RANGE: //获取目标温度上下限
      {
        //				s32 funStatus = 0;
        GetTargetRangeLimit stGetWarningLimit= {0};

        if( RxFrame->Header.FrameLen != 7 )
        {

          funStatus = ERR_FRAME_LENTH_ERROR;

          status = SendNormalData( funStatus, RxFrame, NULL, &stResult, &protocolAnalysisHandle, 0 );
        }
        else
        {
          switch( RxFrame->FramDate[1] )
          {
            case CHA:
            {
              funStatus = GetTargetTempRange( RxFrame->FramDate[1], &stTemperatureControlA );
              stGetWarningLimit.upperLimit = stTemperatureControlA.targetTempMax;
              stGetWarningLimit.lowerLimit = stTemperatureControlA.targetTempMin;
            }
            break;
            case CHB:
            {
              funStatus = GetTargetTempRange( RxFrame->FramDate[1], &stTemperatureControlB );
              stGetWarningLimit.upperLimit = stTemperatureControlB.targetTempMax;
              stGetWarningLimit.lowerLimit = stTemperatureControlB.targetTempMin;
            }
            break;
            case CHC:
            {
              funStatus = GetTargetTempRange( RxFrame->FramDate[1], &stTemperatureControlC );
              stGetWarningLimit.upperLimit = stTemperatureControlC.targetTempMax;
              stGetWarningLimit.lowerLimit = stTemperatureControlC.targetTempMin;
            }
            break;

            case CHD:
            {
              funStatus = GetTargetTempRange( RxFrame->FramDate[1], &stTemperatureControlD );
              stGetWarningLimit.upperLimit = stTemperatureControlD.targetTempMax;
              stGetWarningLimit.lowerLimit = stTemperatureControlD.targetTempMin;
            }
            break;

            default:
              funStatus = ERR_CHANNEL_NUM;
              break;
          }
          stGetWarningLimit.parameter1 = RxFrame->FramDate[0];
          stGetWarningLimit.channelNum = RxFrame->FramDate[1];
          status = SendNormalData( funStatus, RxFrame, &stGetWarningLimit, &stResult, &protocolAnalysisHandle, 0 );
        }
      }
      break;

      case GET_CUR_TEMP_RANGE:    //获取出现温度故障上下限
      {
        GetErrorLimit stGetErrorLimit;

        if( RxFrame->Header.FrameLen != 7 )
        {
          funStatus = ERR_FRAME_LENTH_ERROR;

          status = SendNormalData( funStatus, RxFrame, NULL, &stResult, &protocolAnalysisHandle, 0 );
        }
        else
        {
          switch( RxFrame->FramDate[1] )
          {
            case CHA:
            {
              funStatus = GetCurTemperRange( RxFrame->FramDate[1], &stTemperatureControlA );
              stGetErrorLimit.lowerLimit = stTemperatureControlA.minErrorT;
              stGetErrorLimit.upperLimit = stTemperatureControlA.maxErrorT;
            }
            break;

            case CHB:
            {
              funStatus = GetCurTemperRange( RxFrame->FramDate[1], &stTemperatureControlB );
              stGetErrorLimit.lowerLimit = stTemperatureControlB.minErrorT;
              stGetErrorLimit.upperLimit = stTemperatureControlB.maxErrorT;
            }
            break;

            case CHC:
            {
              funStatus = GetCurTemperRange( RxFrame->FramDate[1], &stTemperatureControlC );
              stGetErrorLimit.lowerLimit = stTemperatureControlC.minErrorT;
              stGetErrorLimit.upperLimit = stTemperatureControlC.maxErrorT;
            }
            break;

            case CHD:
            {
              funStatus = GetCurTemperRange( RxFrame->FramDate[1], &stTemperatureControlD );
              stGetErrorLimit.lowerLimit = stTemperatureControlD.minErrorT;
              stGetErrorLimit.upperLimit = stTemperatureControlD.maxErrorT;
            }
            break;

            default:
              funStatus = ERR_CHANNEL_NUM;
              break;
          }

          stGetErrorLimit.parameter1 = RxFrame->FramDate[0];
          stGetErrorLimit.channelNum  = RxFrame->FramDate[1];

          status = SendNormalData( funStatus, RxFrame, &stGetErrorLimit, &stResult, &protocolAnalysisHandle, 0 );
        }

      }
      break;

      case GET_TARGET_TEMPERATURE:  //获取目标温度
      {
        //				s32 funStatus = 0;
        GetTargetTemp	stGetTargetTemp = {0};

        if( RxFrame->Header.FrameLen != 7 )
        {
          funStatus = ERR_FRAME_LENTH_ERROR;

          status = SendNormalData( funStatus, RxFrame, NULL, &stResult, &protocolAnalysisHandle, 0 );
        }
        else
        {
          stGetTargetTemp.channelNum = RxFrame->FramDate[1];
          switch( stGetTargetTemp.channelNum )
          {
            case CHA:
            {
              funStatus = GetTargetTemperature( stGetTargetTemp.channelNum, ( s32 * )&stGetTargetTemp.targetTemp, PidInfoA );
            }
            break;

            case CHB:
            {
              funStatus = GetTargetTemperature( stGetTargetTemp.channelNum, ( s32 * )&stGetTargetTemp.targetTemp, PidInfoB );
            }
            break;

            case CHC:
            {
              funStatus = GetTargetTemperature( stGetTargetTemp.channelNum, ( s32 * )&stGetTargetTemp.targetTemp, PidInfoC );
            }
            break;

            case CHD:
            {
              funStatus = GetTargetTemperature( stGetTargetTemp.channelNum, ( s32 * )&stGetTargetTemp.targetTemp, PidInfoD );
            }
            break;


            default:
              funStatus = ERR_CHANNEL_NUM;
              break;
          }
          stGetTargetTemp.parameter1 = RxFrame->FramDate[0];
          status = SendNormalData( funStatus, RxFrame, &stGetTargetTemp, &stResult, &protocolAnalysisHandle, 0 );
        }

      }
      break;

      case GET_CURRENT_TEMPERATURE: //获取当前通道温度
      {
        GetCurTemp stGetCurrentTemp = {0};

        if( RxFrame->Header.FrameLen != 7 )
        {

          funStatus = ERR_FRAME_LENTH_ERROR;

          status = SendNormalData( funStatus, RxFrame, &stGetCurrentTemp, &stResult, &protocolAnalysisHandle, 0 );
        }
        else
        {
          stGetCurrentTemp.channelNum = RxFrame->FramDate[1];

          switch( stGetCurrentTemp.channelNum )
          {
            case CHA:
            {
              funStatus = GetSensorValue( stGetCurrentTemp.channelNum, ( s32* )&stGetCurrentTemp.currentTemp, stTemperatureControlA );

            }
            break;
            case CHB:
            {
              funStatus = GetSensorValue( stGetCurrentTemp.channelNum, ( s32* )&stGetCurrentTemp.currentTemp, stTemperatureControlB );
              if( ( funStatus == ERR_TEMP_OVER_W_UPPER_LIMIT ) || ( funStatus == ERR_TEMP_OVER_W_LOWER_LIMIT ) )
                funStatus =( funStatus == ERR_TEMP_OVER_W_UPPER_LIMIT )? ERR_TEMP_OVER_W_UPPER_LIMIT_B:ERR_TEMP_OVER_W_LOWER_LIMIT_B;

            }
            break;
            case CHC:
            {
              funStatus = GetSensorValue( stGetCurrentTemp.channelNum, ( s32* )&stGetCurrentTemp.currentTemp, stTemperatureControlC );
              if( ( funStatus == ERR_TEMP_OVER_W_UPPER_LIMIT ) || ( funStatus == ERR_TEMP_OVER_W_LOWER_LIMIT ) )
                funStatus = ( funStatus ==ERR_TEMP_OVER_W_UPPER_LIMIT )? ERR_TEMP_OVER_W_UPPER_LIMIT_C:ERR_TEMP_OVER_W_LOWER_LIMIT_C;

            }
            break;
            case CHD:
            {
              funStatus = GetSensorValue( stGetCurrentTemp.channelNum, ( s32* )&stGetCurrentTemp.currentTemp, stTemperatureControlD );
              if( ( funStatus == ERR_TEMP_OVER_W_UPPER_LIMIT ) || ( funStatus == ERR_TEMP_OVER_W_LOWER_LIMIT ) )
                funStatus =( funStatus ==ERR_TEMP_OVER_W_UPPER_LIMIT )? ERR_TEMP_OVER_W_UPPER_LIMIT_D:ERR_TEMP_OVER_W_LOWER_LIMIT_D;

            }
            break;

            default:
              funStatus = ERR_CHANNEL_NUM;
              break;

          }
          stGetCurrentTemp.parameter1 = RxFrame->FramDate[0];
          status = SendNormalData( funStatus, RxFrame, &stGetCurrentTemp, &stResult, &protocolAnalysisHandle, 0 );
        }

      }
      break;

      case TARGET_CALIBRATION:			//目标温度校准(主要是点对点的)
      {
        calTarTempParam *stSetCalTarParam;

        if( RxFrame->Header.FrameLen != 87 )
        {
          funStatus = ERR_FRAME_LENTH_ERROR;

          status = SendNormalData( funStatus, RxFrame, NULL, &stResult, &protocolAnalysisHandle, 0 );
        }
        else
        {

          stSetCalTarParam = ( calTarTempParam * )&( RxFrame->FramDate[1] );
          funStatus = CalibrationTargetTemperature( stSetCalTarParam );
          status = SendNormalData( funStatus, RxFrame, NULL, &stResult, &protocolAnalysisHandle, 0 );
        }
      }
      break;

      case UPGRADE:	//UPGRADE升级
      {
        UpgradeDef stUpgradeInfo;

        if( RxFrame->Header.FrameLen != 6 )
        {
          funStatus = ERR_FRAME_LENTH_ERROR;

          status = SendNormalData( funStatus, RxFrame, NULL, &stResult, &protocolAnalysisHandle, 0 );
        }
        else
        {
          if( RxFrame->FramDate[0] <= ETH_AUTO_UPGRADE )
          {
            StopAllOfTec();
            SendNormalData( 0, RxFrame, NULL, &stResult, &protocolAnalysisHandle, 0 );

            stUpgradeInfo.UpdateType = RxFrame->FramDate[0];
            JumpToIap( &stUpgradeInfo );
          }
          else
          {
            status = SendNormalData( ERR_PARAM1_ABNOMAL, RxFrame, NULL, &stResult, &protocolAnalysisHandle, 0 );
          }
        }
      }
      break;

      case CONTROL_LIQUID_PUMP: //控制液泵
      {

        EnablePump *stEnablePump;

        if( RxFrame->Header.FrameLen != 7 )
        {
          funStatus = ERR_FRAME_LENTH_ERROR;

          status = SendNormalData( funStatus, RxFrame, NULL, &stResult, &protocolAnalysisHandle, 0 );
        }
        else
        {

          stEnablePump = ( EnablePump * )RxFrame->FramDate;

          funStatus = LiquidPumpContrl( stEnablePump->parameter1, stEnablePump->ableFlag );
          status = SendNormalData( funStatus, RxFrame, NULL, &stResult, &protocolAnalysisHandle, 0 );
        }
      }
      break;

      case CONTROL_LIQUID_FAN:  //控制制冷风扇
      {
        //				u8 funStatus = 0;
        EnableFan *stEnableFun;

        if( RxFrame->Header.FrameLen != 7 )
        {
          funStatus = ERR_FRAME_LENTH_ERROR;

          status = SendNormalData( funStatus, RxFrame, NULL, &stResult, &protocolAnalysisHandle, 0 );
        }
        else
        {
          stEnableFun = ( EnableFan * )RxFrame->FramDate;

          funStatus = LiquidFanContrl( stEnableFun->parameter1, stEnableFun->ableFlag );
          status = SendNormalData( funStatus, RxFrame, NULL, &stResult, &protocolAnalysisHandle, 0 );
        }
      }
      break;

      case SET_STEADY_ERROR:   //设置稳态时，报警/报错温度差值（报警：0.5°，报错：1°）
      {

      }
      break;

      case GET_STEADY_ERROR:   //获取稳态时，报警/报错温度差值（报警：0.5°，报错：1°）
      {

      }
      break;

      case GET_TEC_WORK_COUNT: //获取TEC工作次数
      {

      }
      break;

      case SET_PIDCONFIG_PARAM: //设置PID配置参数
      {
        SetConfigParam *stSetConfigParam;

        if( RxFrame->Header.FrameLen != 24 )
        {

          funStatus = ERR_FRAME_LENTH_ERROR;

          status = SendNormalData( funStatus, RxFrame, NULL, &stResult, &protocolAnalysisHandle, 0 );
        }
        else
        {
          stSetConfigParam = ( SetConfigParam * )RxFrame->FramDate;
          switch( stSetConfigParam->channelNum )
          {
            case CHA:
            {
              funStatus = SetPidConfigParam( stSetConfigParam->channelNum, &PidInfoA, &stSetConfigParam->stPidCalParam );
            }
            break;

            case CHB:
            {
              funStatus = SetPidConfigParam( stSetConfigParam->channelNum, &PidInfoB, &stSetConfigParam->stPidCalParam );
            }
            break;

            case CHC:
            {
              funStatus = SetPidConfigParam( stSetConfigParam->channelNum, &PidInfoC, &stSetConfigParam->stPidCalParam );
            }
            break;

            case CHD:
            {
              funStatus = SetPidConfigParam( stSetConfigParam->channelNum, &PidInfoD, &stSetConfigParam->stPidCalParam );
            }
            break;

            default:
              funStatus = ERR_CHANNEL_NUM;
              break;
          }
          status = SendNormalData( funStatus, RxFrame, NULL, &stResult, &protocolAnalysisHandle, 0 );
        }
      }
      break;

      case GET_PIDCONFIG_PARAM: //获取PID配置参数
      {
        GetConfigParam stGetConfigParam;
        if( RxFrame->Header.FrameLen != 7 )
        {
          funStatus = ERR_FRAME_LENTH_ERROR;

          status = SendNormalData( funStatus, RxFrame, NULL, &stResult, &protocolAnalysisHandle, 0 );
        }
        else
        {
          stGetConfigParam.parameter1= RxFrame->FramDate[0];
          stGetConfigParam.channelNum = RxFrame->FramDate[1];
          switch( stGetConfigParam.channelNum )
          {
            case CHA:
            {
              funStatus = GetPidConfigParam( stGetConfigParam.channelNum = RxFrame->FramDate[1], &PidInfoA, &stGetConfigParam.stPidCalParam );

            }
            break;

            case CHB:
            {
              funStatus = GetPidConfigParam( stGetConfigParam.channelNum = RxFrame->FramDate[1], &PidInfoB, &stGetConfigParam.stPidCalParam );

            }
            break;

            case CHC:
            {
              funStatus = GetPidConfigParam( stGetConfigParam.channelNum = RxFrame->FramDate[1], &PidInfoC, &stGetConfigParam.stPidCalParam );

            }
            break;

            case CHD:
            {
              funStatus = GetPidConfigParam( stGetConfigParam.channelNum = RxFrame->FramDate[1], &PidInfoD, &stGetConfigParam.stPidCalParam );
            }
            break;

            default:
              funStatus = ERR_CHANNEL_NUM;
              break;
          }
          status = SendNormalData( funStatus, RxFrame, &stGetConfigParam, &stResult, &protocolAnalysisHandle, 0 );


        }
      }
      break;

      case GET_TC_FLAG:					//获取温度控制使能标示
      {
        GetTcFlag  stGetTcFlag;

        if( RxFrame->Header.FrameLen != 7 )
        {
          funStatus = ERR_FRAME_LENTH_ERROR;

          status = SendNormalData( funStatus, RxFrame, NULL, &stResult, &protocolAnalysisHandle, 0 );
        }
        else
        {
          stGetTcFlag.channelNum = RxFrame->FramDate[1];

          switch( stGetTcFlag.channelNum )
          {
            case CHA:
            {
              funStatus = GetChannelEnableFlag( stGetTcFlag.channelNum, ( u8* )&stGetTcFlag.flag, &stTemperatureControlA );
            }
            break;

            case CHB:
            {
              funStatus = GetChannelEnableFlag( stGetTcFlag.channelNum, ( u8* )&stGetTcFlag.flag, &stTemperatureControlB );
            }
            break;

            case CHC:
            {
              funStatus = GetChannelEnableFlag( stGetTcFlag.channelNum, ( u8* )&stGetTcFlag.flag, &stTemperatureControlC );
            }
            break;

            case CHD:
            {
              funStatus = GetChannelEnableFlag( stGetTcFlag.channelNum, ( u8* )&stGetTcFlag.flag, &stTemperatureControlD );
            }
            break;

            default:
              funStatus = ERR_CHANNEL_NUM;
              break;

          }

          stGetTcFlag.parameter1 = RxFrame->FramDate[0];
          status = SendNormalData( funStatus, RxFrame, &stGetTcFlag, &stResult, &protocolAnalysisHandle, 0 );
        }


      }
      break;

      case CONTROL_REFRIG_PUMP://控制抽冷凝水泵
      {
        EnableRefrigPump *stEnableRefrigPump;

        if( RxFrame->Header.FrameLen != 7 )
        {
          funStatus = ERR_FRAME_LENTH_ERROR;

          SendNormalData( funStatus, RxFrame, NULL, &stResult, &protocolAnalysisHandle, 0 );
        }
        else
        {

          stEnableRefrigPump = ( EnableRefrigPump * )RxFrame->FramDate;

          funStatus = LiquidPumpContrl( stEnableRefrigPump->parameter1, stEnableRefrigPump->ableFlag );
          SendNormalData( funStatus, RxFrame, NULL, &stResult, &protocolAnalysisHandle, 0 );
        }

      }
      break;

      default:
        status = SendNormalData( ERR_FRAME_INEXISTENCE, RxFrame, NULL, &stResult, &protocolAnalysisHandle, 0 );
        break;
    }

  } // end of  if( OBJECT_UNIT == RxFrame->Header.ObjectUnit)

  return status;
}


/**************************************************************************************
** 函数名称: BuildActiveReportFrame
** 参数    : ErrorCode：   pProtocolDealHandle：  OutputTxFrame：  FrameIndex：
** 函数功能: 构建协议报表框架
** 返回值  : BuildAppResultStatusDef：返回分析的结果类型
**************************************************************************************/
BuildAppResultStatusDef BuildActiveReportFrame( s16 ErrorCode, ProtocolDealHandle *pProtocolDealHandle, Frame *OutputTxFrame, u16 *FrameIndex )
{
  OutputTxFrame->Header.Indicator.Header1 = FRAME_HEAD1;
  OutputTxFrame->Header.Indicator.Header2 = FRAME_HEAD2;
  OutputTxFrame->Header.FrameLen 					= FRAME_ACTIVE_REPORT_LENTH;
  OutputTxFrame->Header.FrameType 				= LAUNCH_FRAME_1_TYPE;
  OutputTxFrame->Header.FrameIndex			 	= *FrameIndex;

  ( *FrameIndex ) ++;

  if( *FrameIndex >= 0xFFFF )
  {
    *FrameIndex  = 0;
  }

  OutputTxFrame->Header.ObjectUnit  = pProtocolDealHandle->TagetObjectUnit;
  OutputTxFrame->Header.CommandCode = ANOMALY_REPORT;

  OutputTxFrame->FramDate[0] = 0;
  OutputTxFrame->FramDate[2] = ( ( ErrorCode>>8 ) );
  OutputTxFrame->FramDate[1] = ErrorCode;

  return RESULT_ACTIVE_REPORT_FRAME_OK;
}

/**************************************************************************************
** 函数名称: SendAbnormalData
** 参数    : ErrorCode：   pProtocolDealHandle：  OutputTxFrame：  FrameIndex：    timeOut：
** 函数功能: 发送异常数据
** 返回值  : BuildAppResultStatusDef：返回分析的结果类型
**************************************************************************************/
BuildAppResultStatusDef SendAbnormalData( s16 ErrorCode, ProtocolDealHandle *pProtocolDealHandle, Frame *OutputTxFrame, u16 *FrameIndex, u32 timeOut )
{
  BuildAppResultStatusDef BuildStatus;
  //	Frame TxResponseFrame;

  BuildStatus = BuildActiveReportFrame( ErrorCode, pProtocolDealHandle, OutputTxFrame, FrameIndex );
  if( BuildStatus == RESULT_ACTIVE_REPORT_FRAME_OK )
  {
    if( FRAME_TX_OK != SendFrame( pProtocolDealHandle, OutputTxFrame, timeOut ) )
    {

    }
  }
  else
  {
    return RESULT_ACTIVE_REPORT_FRAME_ERR;

  }

  return BuildStatus;
}


/**************************************************************************************
** 函数名称: SendNormalData
** 参数    : FrameStatus：   InputRxFrame：   InputData：    OutputTxFrame：   pProtocolDealHandle：   timeOut：
** 函数功能: 发送正常数据
** 返回值  : BuildAppResultStatusDef：返回分析的结果类型
**************************************************************************************/
BuildAppResultStatusDef SendNormalData( u16 FrameStatus, const Frame * InputRxFrame, void *InputData, Frame *OutputTxFrame, ProtocolDealHandle *pProtocolDealHandle, u32 timeOut )
{
  BuildAppResultStatusDef	 BuildStatus;

  BuildStatus = BuildAppResultFrame( FrameStatus, InputRxFrame, InputData, OutputTxFrame );

  if( BuildStatus == RESULT_FRAME_BUILD_OK )
  {

    if( FRAME_TX_OK != SendFrame( pProtocolDealHandle, OutputTxFrame, timeOut ) )
    {


    }
  }
  else
  {
    BuildStatus = RESULT_FRAME_BUILD_ERROR;
  }

  return BuildStatus;
}

/**************************************************************************************
** 函数名称: BuildAppResultFrame_test
** 参数    : FrameStatus：   InputRxFrame：   InputData：    OutputTxFrame：
** 函数功能: 构建应用程序协议结果框架_测试
** 返回值  : BuildAppResultStatusDef：返回分析的结果类型
**************************************************************************************/
BuildAppResultStatusDef BuildAppResultFrame_test( u8 FramStatus, const Frame * InputRxFrame, void *InputData, Frame * OutputTxFrame )
{
  u32 len;
  u16 ErrorCode;
  u8 resultStatusTemp;

  if( FramStatus != true )
  {
    resultStatusTemp = true;

    OutputTxFrame->Header.FrameLen = 0x06 + InputRxFrame->Header.FrameLen;

    if( FramStatus == ERR_FRAME_LENTH_ERROR )
    {
      len = 0;
      OutputTxFrame->FramDate[len] = resultStatusTemp;
      len += 1;

      OutputTxFrame->FramDate[len] = PROTOCAL_PARAM1;
      len +=1;
    }
    else
    {
      len = 0;
      OutputTxFrame->FramDate[len] = resultStatusTemp;
      len += 1;

      OutputTxFrame->FramDate[len] = InputRxFrame->FramDate[0];
      len +=1;
    }

    RequireErrCode( FramStatus, ( u16 * )&ErrorCode );
    OutputTxFrame->FramDate[len] = ( u8 )ErrorCode ;
    len++;
    OutputTxFrame->FramDate[len] = ( u8 )( ErrorCode>>8 ) ;
    len++;
    memcpy( OutputTxFrame->FramDate + len, InputRxFrame, OutputTxFrame->Header.FrameLen -2 );
    len += OutputTxFrame->Header.FrameLen - 2;
    memcpy( OutputTxFrame->FramDate + len, ( u8* )&InputRxFrame->CheckCode, 2 );
    OutputTxFrame->Header.FrameLen += 0x09;
    OutputTxFrame->Header.Indicator.Header1 = FRAME_HEAD1;
    OutputTxFrame->Header.Indicator.Header2 = FRAME_HEAD2;
    OutputTxFrame->Header.FrameType = RESULT_FRAME_TYPE;
    OutputTxFrame->Header.FrameIndex = InputRxFrame->Header.FrameIndex;
    OutputTxFrame->Header.ObjectUnit = SBC;
    OutputTxFrame->Header.CommandCode = InputRxFrame->Header.CommandCode;

    return RESULT_FRAME_BUILD_OK;
  }
  else
  {
    resultStatusTemp = false;
    switch( InputRxFrame->Header.CommandCode )
    {
      case REQUIRE_VERSION:
      {

        OutputTxFrame->Header.FrameLen = 0x05 + 13;
        memcpy( &OutputTxFrame->FramDate[2], InputData, 12 );
        OutputTxFrame->FramDate[0] = resultStatusTemp;
        OutputTxFrame->FramDate[1] = InputRxFrame->FramDate[0];
      }
      break;

      case SET_PID_PARAMETERS:
      {
        OutputTxFrame->FramDate[0] = resultStatusTemp;
        OutputTxFrame->FramDate[1] = InputRxFrame->FramDate[0];

        OutputTxFrame->Header.FrameLen = 7;
      }
      break;

      case SET_TARGET_TEMP_RANGE:
      {
        OutputTxFrame->FramDate[0] = resultStatusTemp; //
        OutputTxFrame->FramDate[1] = InputRxFrame->FramDate[0];
        OutputTxFrame->Header.FrameLen = 7;
      }
      break;

      case SET_CURRENT_TEMP_RANGE:
      {
        OutputTxFrame->FramDate[0] = resultStatusTemp; //
        OutputTxFrame->FramDate[1] = InputRxFrame->FramDate[0];
        OutputTxFrame->Header.FrameLen = 7;
      }
      break;

      case SET_TARGET_TEMPERATURE:
      {
        OutputTxFrame->FramDate[0] = resultStatusTemp;
        OutputTxFrame->FramDate[1] = InputRxFrame->FramDate[0];
        OutputTxFrame->Header.FrameLen = 7;
      }
      break;

      case ENABLE_DISABLE_TC:
      {
        OutputTxFrame->FramDate[0] = resultStatusTemp;
        OutputTxFrame->FramDate[1] = InputRxFrame->FramDate[0];
        OutputTxFrame->Header.FrameLen = 7;
      }
      break;

      case GET_PID_PARAMETERS:
      {
        GetPIDParameter	*stGetPIDParam;

        stGetPIDParam = ( GetPIDParameter * )InputData;

        OutputTxFrame->Header.FrameLen = 0x05 + sizeof( GetPIDParameter );
        memcpy( OutputTxFrame->FramDate, stGetPIDParam, sizeof( GetPIDParameter ) );
        OutputTxFrame->FramDate[0] = resultStatusTemp;
      }
      break;

      case GET_TARGET_TEMP_RANGE:
      {
        GetTargetRangeLimit *stGetWarningLimit;

        stGetWarningLimit = ( GetTargetRangeLimit * )InputData;
        OutputTxFrame->Header.FrameLen = 0x05 + sizeof( GetTargetRangeLimit );
        memcpy( OutputTxFrame->FramDate, stGetWarningLimit, sizeof( GetTargetRangeLimit ) );
        OutputTxFrame->FramDate[0] = resultStatusTemp;
      }
      break;

      case GET_CUR_TEMP_RANGE:
      {
        GetErrorLimit *stGetErrorLimit;

        stGetErrorLimit = ( GetErrorLimit * )InputData;
        OutputTxFrame->Header.FrameLen = 0x05 + sizeof( GetErrorLimit );
        memcpy( OutputTxFrame->FramDate, stGetErrorLimit, sizeof( GetErrorLimit ) );
        OutputTxFrame->FramDate[0] = resultStatusTemp;
      }
      break;

      case GET_TARGET_TEMPERATURE:
      {
        GetTargetTemp *stGetTarTemp;

        stGetTarTemp = ( GetTargetTemp * )InputData;
        OutputTxFrame->Header.FrameLen = 0x05 + sizeof( GetTargetTemp );
        memcpy( OutputTxFrame->FramDate, stGetTarTemp, sizeof( GetTargetTemp ) );
        OutputTxFrame->FramDate[0] = resultStatusTemp;
      }
      break;

      case GET_CURRENT_TEMPERATURE:
      {
        GetCurTemp *pGetCurTemp;
        pGetCurTemp = ( GetCurTemp * )InputData;
        OutputTxFrame->Header.FrameLen = 0x05 + sizeof( GetCurTemp );
        memcpy( OutputTxFrame->FramDate, pGetCurTemp, sizeof( GetCurTemp ) );
        OutputTxFrame->FramDate[0] = resultStatusTemp;
      }
      break;

      case TARGET_CALIBRATION:
      {
        OutputTxFrame->FramDate[0] = resultStatusTemp;
        OutputTxFrame->FramDate[1] = InputRxFrame->FramDate[0];
        OutputTxFrame->Header.FrameLen = 7;
      }
      break;

      case UPGRADE:
      {
        OutputTxFrame->FramDate[0] = resultStatusTemp;
        OutputTxFrame->FramDate[1] = InputRxFrame->FramDate[0];
        OutputTxFrame->Header.FrameLen = 7;
      }
      break;

      case CONTROL_LIQUID_PUMP:
      {
        OutputTxFrame->FramDate[0] = resultStatusTemp;
        OutputTxFrame->FramDate[1] = InputRxFrame->FramDate[0];
        OutputTxFrame->Header.FrameLen = 7;
      }
      break;

      case CONTROL_LIQUID_FAN:
      {
        OutputTxFrame->FramDate[0] = resultStatusTemp;
        OutputTxFrame->FramDate[1] = InputRxFrame->FramDate[0];
        OutputTxFrame->Header.FrameLen = 7;
      }
      break;

      case CONTROL_REFRIG_PUMP:
      {
        OutputTxFrame->FramDate[0] = resultStatusTemp;
        OutputTxFrame->FramDate[1] = InputRxFrame->FramDate[0];
        OutputTxFrame->Header.FrameLen = 7;

      }
      break;

      default:
        break;
    }

    OutputTxFrame->Header.Indicator.Header1 = FRAME_HEAD1;
    OutputTxFrame->Header.Indicator.Header2 = FRAME_HEAD2;
    OutputTxFrame->Header.FrameType = RESULT_FRAME_TYPE;
    OutputTxFrame->Header.FrameIndex = InputRxFrame->Header.FrameIndex;
    OutputTxFrame->Header.ObjectUnit = SBC;
    OutputTxFrame->Header.CommandCode = InputRxFrame->Header.CommandCode;
  }

  return RESULT_FRAME_BUILD_OK;
}


/**************************************************************************************
** 函数名称: BuildAppResultFrame_test
** 参数    : FrameStatus：   InputRxFrame：   InputData：    OutputTxFrame：
** 函数功能: 构建应用程序协议结果框架
** 返回值  : BuildAppResultStatusDef：返回分析的结果类型
**************************************************************************************/
BuildAppResultStatusDef BuildAppResultFrame( u16 FramStatus, const Frame * InputRxFrame, void *InputData, Frame * OutputTxFrame )
{
  u32 len;
  u8 resultStatusTemp =0;

  if( FramStatus != 0 )
  {
    resultStatusTemp = 1;

    OutputTxFrame->Header.FrameLen = 0x06 + InputRxFrame->Header.FrameLen;

    if( FramStatus == ERR_FRAME_LENTH_ERROR )
    {
      len = 0;
      OutputTxFrame->FramDate[len] = resultStatusTemp;
      len += 1;

      OutputTxFrame->FramDate[len] = PROTOCAL_PARAM1;
      len +=1;
    }
    else
    {
      len = 0;
      OutputTxFrame->FramDate[len] = resultStatusTemp;
      len += 1;

      OutputTxFrame->FramDate[len] = InputRxFrame->FramDate[0];
      len +=1;
    }

    OutputTxFrame->FramDate[len] = ( u8 )FramStatus ;
    len++;
    OutputTxFrame->FramDate[len] = ( u8 )( FramStatus>>8 ) ;
    len++;
    memcpy( OutputTxFrame->FramDate + len, InputRxFrame, OutputTxFrame->Header.FrameLen -2 );
    len += OutputTxFrame->Header.FrameLen - 2;
    memcpy( OutputTxFrame->FramDate + len, ( u8* )&InputRxFrame->CheckCode, 2 );
    OutputTxFrame->Header.FrameLen += 0x09;
    OutputTxFrame->Header.Indicator.Header1 = FRAME_HEAD1;
    OutputTxFrame->Header.Indicator.Header2 = FRAME_HEAD2;
    OutputTxFrame->Header.FrameType = RESULT_FRAME_TYPE;
    OutputTxFrame->Header.FrameIndex = InputRxFrame->Header.FrameIndex;
    OutputTxFrame->Header.ObjectUnit = SBC;
    OutputTxFrame->Header.CommandCode = InputRxFrame->Header.CommandCode;

    return RESULT_FRAME_BUILD_OK;
  }
  else
  {
    resultStatusTemp = 0;
    switch( InputRxFrame->Header.CommandCode )
    {
      case REQUIRE_VERSION:
      {
        OutputTxFrame->Header.FrameLen = 0x05 + 13;
        memcpy( &OutputTxFrame->FramDate[2], InputData, 11 );
        OutputTxFrame->FramDate[0] = resultStatusTemp;
        OutputTxFrame->FramDate[1] = InputRxFrame->FramDate[0];
      }
      break;

      case SET_PID_PARAMETERS:
      {
        OutputTxFrame->FramDate[0] = resultStatusTemp;
        OutputTxFrame->FramDate[1] = InputRxFrame->FramDate[0];

        OutputTxFrame->Header.FrameLen = 7;
      }
      break;

      case SET_TARGET_TEMP_RANGE:
      {
        OutputTxFrame->FramDate[0] = resultStatusTemp;
        OutputTxFrame->FramDate[1] = InputRxFrame->FramDate[0];
        OutputTxFrame->Header.FrameLen = 7;
      }
      break;

      case SET_CURRENT_TEMP_RANGE:
      {
        OutputTxFrame->FramDate[0] = resultStatusTemp;
        OutputTxFrame->FramDate[1] = InputRxFrame->FramDate[0];
        OutputTxFrame->Header.FrameLen = 7;
      }
      break;

      case SET_TARGET_TEMPERATURE:
      {
        OutputTxFrame->FramDate[0] = resultStatusTemp;
        OutputTxFrame->FramDate[1] = InputRxFrame->FramDate[0];
        OutputTxFrame->Header.FrameLen = 7;
      }
      break;

      case ENABLE_DISABLE_TC:
      {
        OutputTxFrame->FramDate[0] = resultStatusTemp;
        OutputTxFrame->FramDate[1] = InputRxFrame->FramDate[0];
        OutputTxFrame->Header.FrameLen = 7;
      }
      break;

      case GET_PID_PARAMETERS:
      {
        GetPIDParameter	*stGetPIDParam;

        stGetPIDParam = ( GetPIDParameter * )InputData;

        OutputTxFrame->Header.FrameLen = 0x05 + sizeof( GetPIDParameter );
        memcpy( OutputTxFrame->FramDate, stGetPIDParam, sizeof( GetPIDParameter ) );
        OutputTxFrame->FramDate[0] = resultStatusTemp;
      }
      break;

      case GET_TARGET_TEMP_RANGE:
      {
        GetTargetRangeLimit *stGetWarningLimit;

        stGetWarningLimit = ( GetTargetRangeLimit * )InputData;
        OutputTxFrame->Header.FrameLen = 0x05 + sizeof( GetTargetRangeLimit );
        memcpy( OutputTxFrame->FramDate, stGetWarningLimit, sizeof( GetTargetRangeLimit ) );
        OutputTxFrame->FramDate[0] = resultStatusTemp;
      }
      break;

      case GET_CUR_TEMP_RANGE:
      {
        GetErrorLimit *stGetErrorLimit;

        stGetErrorLimit = ( GetErrorLimit * )InputData;
        OutputTxFrame->Header.FrameLen = 0x05 + sizeof( GetErrorLimit );
        memcpy( OutputTxFrame->FramDate, stGetErrorLimit, sizeof( GetErrorLimit ) );
        OutputTxFrame->FramDate[0] = resultStatusTemp;
      }
      break;

      case GET_TARGET_TEMPERATURE:
      {
        GetTargetTemp *stGetTarTemp;

        stGetTarTemp = ( GetTargetTemp * )InputData;
        OutputTxFrame->Header.FrameLen = 0x05 + sizeof( GetTargetTemp );
        memcpy( OutputTxFrame->FramDate, stGetTarTemp, sizeof( GetTargetTemp ) );
        OutputTxFrame->FramDate[0] = resultStatusTemp;
      }
      break;

      case GET_CURRENT_TEMPERATURE:
      {
        GetCurTemp *pGetCurTemp;
        pGetCurTemp = ( GetCurTemp * )InputData;
        OutputTxFrame->Header.FrameLen = 0x05 + sizeof( GetCurTemp );
        memcpy( OutputTxFrame->FramDate, pGetCurTemp, sizeof( GetCurTemp ) );
        OutputTxFrame->FramDate[0] = resultStatusTemp;
      }
      break;

      case TARGET_CALIBRATION:
      {
        OutputTxFrame->FramDate[0] = resultStatusTemp;
        OutputTxFrame->FramDate[1] = InputRxFrame->FramDate[0];
        OutputTxFrame->Header.FrameLen = 7;
      }
      break;

      case UPGRADE:
      {
        OutputTxFrame->FramDate[0] = resultStatusTemp;
        OutputTxFrame->FramDate[1] = InputRxFrame->FramDate[0];
        OutputTxFrame->Header.FrameLen = 7;
      }
      break;

      case CONTROL_LIQUID_PUMP:
      {
        OutputTxFrame->FramDate[0] = resultStatusTemp;
        OutputTxFrame->FramDate[1] = InputRxFrame->FramDate[0];
        OutputTxFrame->Header.FrameLen = 7;
      }
      break;

      case CONTROL_LIQUID_FAN:
      {
        OutputTxFrame->FramDate[0] = resultStatusTemp;
        OutputTxFrame->FramDate[1] = InputRxFrame->FramDate[0];
        OutputTxFrame->Header.FrameLen = 7;
      }
      break;

      case SET_STEADY_ERROR:
      {

      }
      break;

      case GET_STEADY_ERROR:
      {

      }
      break;

      case GET_TEC_WORK_COUNT:
      {

      }
      break;

      case SET_PIDCONFIG_PARAM:
      {
        OutputTxFrame->FramDate[0] = resultStatusTemp;
        OutputTxFrame->FramDate[1] = InputRxFrame->FramDate[0];

        OutputTxFrame->Header.FrameLen = 7;
      }
      break;

      case GET_PIDCONFIG_PARAM:
      {
        GetConfigParam	*stGetConfigParam;

        stGetConfigParam = ( GetConfigParam * )InputData;

        OutputTxFrame->Header.FrameLen = 0x05 + sizeof( GetConfigParam );
        memcpy( OutputTxFrame->FramDate, stGetConfigParam, sizeof( GetConfigParam ) );
        OutputTxFrame->FramDate[0] = resultStatusTemp;
      }
      break;

      case GET_TC_FLAG:
      {
        GetTcFlag *stGetFlag;

        stGetFlag = ( GetTcFlag * )InputData;
        OutputTxFrame->Header.FrameLen = 0x05 + sizeof( GetTcFlag );
        memcpy( OutputTxFrame->FramDate, stGetFlag, sizeof( GetTcFlag ) );
        OutputTxFrame->FramDate[0] = resultStatusTemp;
      }
      break;

      case CONTROL_REFRIG_PUMP:
      {
        OutputTxFrame->FramDate[0] = resultStatusTemp;
        OutputTxFrame->FramDate[1] = InputRxFrame->FramDate[0];
        OutputTxFrame->Header.FrameLen = 7;

      }
      break;

      default:
        break;
    }

    OutputTxFrame->Header.Indicator.Header1 = FRAME_HEAD1;
    OutputTxFrame->Header.Indicator.Header2 = FRAME_HEAD2;
    OutputTxFrame->Header.FrameType = RESULT_FRAME_TYPE;
    OutputTxFrame->Header.FrameIndex = InputRxFrame->Header.FrameIndex;
    OutputTxFrame->Header.ObjectUnit = SBC;
    OutputTxFrame->Header.CommandCode = InputRxFrame->Header.CommandCode;
  }

  return RESULT_FRAME_BUILD_OK;
}


/**************************************************************************************
** 函数名称: RequireErrCode
** 参数    : FrameStatus：   pErrCode：
** 函数功能: 错误代码
** 返回值  : BuildAppResultStatusDef：返回分析的结果类型
**************************************************************************************/
ReqErrCodeStatusDef RequireErrCode( u8 FramStatus, u16 *pErrCode )
{
  switch( FramStatus )
  {
    case RESULT_LENTH_ERR:
    {
      *pErrCode = ERR_FRAME_LENTH_ERROR;
    }
    break;

    case RESULT_CHANNEL_ERR:
    {
      *pErrCode =ERR_CHANNEL_NUM ;
    }
    break;
    case RESULT_ADC_ERR:
    {

    }
    break;

    case RESULT_TARGET_UPPER_ERR:
    {
      *pErrCode =ERR_TARGET_TEMP_OVER_UPPER_LIMIT ;
    }
    break;

    case RESULT_TARGET_LOWER_ERR:
    {
      *pErrCode =ERR_TARGET_TEMP_OVER_LOWER_LIMIT ;
    }
    break;

    case RESULT_TARGET_UPPER_ERR_B:
    {

    }
    break;

    case RESULT_TARGET_LOWER_ERR_B:
    {

    }
    break;

    case RESULT_CUR_UPPER_ERR:
    {

    }
    break;

    case RESULT_CUR_LOWER_ERR:
    {

    }
    break;

    case RESULT_CUR_UPPER_ERR_B:
    {

    }
    break;

    case RESULT_CUR_LOWER_ERR_B:
    {

    }
    break;

    case RESULT_FRAME_INEXISTENCE:
    {
      *pErrCode = ERR_FRAME_INEXISTENCE;
    }
    break;

    case RESULT_PARAM1_ABNOMAL:
    {
      *pErrCode = ERR_PARAM1_ABNOMAL;

    }
    break;

    case RESULT_PARAM_ABNOMAL:
    {
      *pErrCode = ERR_PARAM_ABNOMAL;
    }
    break;

    default :
      break;
  }
  return STATUS_OK;
}


