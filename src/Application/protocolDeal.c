/**
  ******************************************************************************
  * @file    protocolDeal.c
  * @author  Firmware-Team
  * @version V1.0.0
  * @date    13-06-2016
  * @brief
  ******************************************************************************
**/
/*
*********************************************************************************************************
*                                             INCLUDE FILES
*********************************************************************************************************
*/
#include  <includes.h>
#include <string.h>

/*
*********************************************************************************************************
*                                               INT DEFINES
*********************************************************************************************************
*/
#define FRAME_DATA_MIN_SIZE 	0x06u
#define FRAME_RESPONSE_LENTH	0x06u
/*
*********************************************************************************************************
*                                            LOCAL DEFINES
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                        LOCAL GLOBAL VARIABLES
*********************************************************************************************************
*/



/*
*********************************************************************************************************
*                                         FUNCTION PROTOTYPES
*********************************************************************************************************
*/

FrameStatusDef FindFrameHeader( u8 *pBuffer, s32 length, u8 **HeadAddr )
{
  u8 *pSrc = pBuffer;
  u32 i;

  *HeadAddr = NULL;

  for( i = 0; i < length; i++ )
  {
    if( length - i < 2 )
    {
      return 	FRAME_LENTH_NOT_ENOUGH;
    }
    if( *( u16* )( pSrc + i ) == FRAME_HEAD )
    {
      *HeadAddr = pSrc + i;
      return FRAME_HEADER_OK;
    }
    else
    {
      if( ( length - i ) == 2 )
      {
        return FRAME_HEADER_NOT_FINE;
      }
    }
  }
  return FRAME_LENTH_NOT_ENOUGH;
}


FrameStatusDef FrameLenthIsOK( u8 *pBuffer, u32 length, u32 *FrameLenthOutPut )
{
  FrameHeader * pHeader;

  *FrameLenthOutPut = 0;

  if( length < FRAME_MIN_SIZE )
  {
    return 	FRAME_LENTH_NOT_ENOUGH;
  }

  pHeader = ( FrameHeader * )pBuffer;


  if( ( pHeader->FrameLen > ( PROTOCOL_DATA_SIZE + FRAME_MIN_SIZE - 6 ) ) || ( pHeader->FrameLen < 5 ) )
  {
    return FRAME_LENTH_ERROR;
  }
  else
  {
    *FrameLenthOutPut = pHeader->FrameLen;
    return FRAME_LENTH_OK;
  }

}

void FrameLoadData( Frame *pFrame, u8 *pBuffer )
{
  FrameHeader * pHeader;

  pHeader = ( FrameHeader * )pBuffer;

  memcpy( pFrame, pHeader, sizeof( FrameHeader ) );

  memcpy( pFrame->FramDate, pBuffer + sizeof( FrameHeader ), pFrame->Header.FrameLen + sizeof( pFrame->Header.Indicator ) + sizeof( pFrame->Header.FrameLen ) - sizeof( FrameHeader ) );

  pFrame->CheckCode = *( u16 * )( pBuffer + pFrame->Header.FrameLen + sizeof( pFrame->Header.Indicator ) + sizeof( pFrame->Header.FrameLen ) );
}

FrameStatusDef FrameCheckCodeIsOK( Frame *pFrame )
{
  u16 checkCode;

  checkCode = crcFast( ( u8 * )pFrame + sizeof( pFrame->Header.Indicator ) + sizeof( pFrame->Header.FrameLen ), pFrame->Header.FrameLen );

  if( checkCode == pFrame->CheckCode )
  {
    return FRAME_CHECKCODE_OK;
  }
  else
  {
    return FRAME_CHECKCODE_ERROR;
  }
}

FrameStatusDef FrameDealInit( ProtocolDealHandle *pProtocolDealHandle )
{
  if( pProtocolDealHandle == NULL )
  {
    return PROTOCOL_HANDLE_PARAMETER_ERROR;
  }

  pProtocolDealHandle->CrcInit();

  return FRAME_INIT_OK;
}

FrameStatusDef BuildFrame( ProtocolDealHandle *pProtocolDealHandle, const Frame *InputTxFrame )
{
  Frame *TxFrame = ( Frame * )pProtocolDealHandle->WriteBuff;
  u32 parameterLen = 0; //参数长度


  memcpy( &TxFrame->Header, &InputTxFrame->Header, sizeof( FrameHeader ) );

  TxFrame->Header.Indicator.Header1 = FRAME_HEAD1;
  TxFrame->Header.Indicator.Header2 = FRAME_HEAD2;

  //有参数
  parameterLen = TxFrame->Header.FrameLen + sizeof( InputTxFrame->Header.Indicator ) + sizeof( InputTxFrame->Header.FrameLen ) - sizeof( FrameHeader ) ;
  if( TxFrame->Header.FrameLen > 0 && parameterLen < PROTOCOL_DATA_SIZE )
  {
    memcpy( TxFrame->FramDate, InputTxFrame->FramDate, parameterLen );
  }
  else
  {
    return FRAME_BUILD_ERROR;
  }

  //计算校验和
  *( u16* )( ( u8* )TxFrame + sizeof( FrameHeader ) + parameterLen ) = pProtocolDealHandle->Crc16( ( u8* )&InputTxFrame->Header.FrameType, InputTxFrame->Header.FrameLen );

  pProtocolDealHandle->WriteDataLen = TxFrame->Header.FrameLen + sizeof( InputTxFrame->Header.Indicator ) + sizeof( InputTxFrame->Header.FrameLen ) + sizeof( InputTxFrame->CheckCode );
  return FRAME_BUILD_OK;
}

FrameStatusDef FrameDeal( ProtocolDealHandle *pProtocolDealHandle, Frame *OutputRxFrame, u32 timeOut )
{
  u32 frameLen = 0, readRealLen = 0;
  u8 *ReadBuff = NULL;
  u8 *headAddr;
  ReadDataFun ReceiveData;
  u32 status;
  u32 SystemTick;

  if( pProtocolDealHandle == NULL )
  {
    return PROTOCOL_HANDLE_PARAMETER_ERROR;
  }

  if( OutputRxFrame == NULL )
  {
    return FRAME_RX_PARAMETER_ERROR;
  }

  SystemTick = HAL_GetTick();

  ReadBuff = pProtocolDealHandle->ReadBuff;
  ReceiveData = pProtocolDealHandle->ReadData;

  ReceiveData( ReadBuff + pProtocolDealHandle->ReadDataLen, pProtocolDealHandle->ReadBuffSize - pProtocolDealHandle->ReadDataLen, &readRealLen );
  if( readRealLen > 0 )
  {
    pProtocolDealHandle->ReadDataLen += readRealLen;

    if( pProtocolDealHandle->TimeoutBetweenBytes.TimingFlag == TIM_BYTE_TIMEOUT_START )
    {
      pProtocolDealHandle->TimeoutBetweenBytes.time = SystemTick;
    }
  }

  //字节间超时
  if( pProtocolDealHandle->TimeoutBetweenBytes.TimingFlag == TIM_BYTE_TIMEOUT_START )
  {
    if( SystemTick - pProtocolDealHandle->TimeoutBetweenBytes.time > pProtocolDealHandle->TimeoutBetweenBytes.TimeValue )
    {
      //丢掉头部
      if( pProtocolDealHandle->ReadDataLen > 1 )
      {
        pProtocolDealHandle->ReadDataLen -= sizeof( OutputRxFrame->Header.Indicator );
        memmove( ReadBuff, ReadBuff + sizeof( OutputRxFrame->Header.Indicator ), pProtocolDealHandle->ReadDataLen );
        SaveLog( NULL, 0 );
      }
      pProtocolDealHandle->TimeoutBetweenBytes.TimingFlag = TIM_BYTE_TIMEOUT_STOP;
      pProtocolDealHandle->TimeoutBetweenBytes.time = 0xFFFFFFFF;
      return FRAME_BYTE_TIMEOUT;
    }
  }

  if( pProtocolDealHandle->ReadDataLen > 1 )
  {
    status = FindFrameHeader( ReadBuff, pProtocolDealHandle->ReadDataLen, &headAddr );

    //找到帧头
    if( status == FRAME_HEADER_OK )
    {
      //开启对字节之间超时进行检测
      if( pProtocolDealHandle->TimeoutBetweenBytes.TimingFlag != TIM_BYTE_TIMEOUT_START )
      {
        pProtocolDealHandle->TimeoutBetweenBytes.TimingFlag = TIM_BYTE_TIMEOUT_START;
        pProtocolDealHandle->TimeoutBetweenBytes.time = SystemTick;
      }

      //帧头位置不是在缓冲区首位
      if( ReadBuff != headAddr )
      {
        //帧头前面的乱码移除
        pProtocolDealHandle->ReadDataLen = pProtocolDealHandle->ReadDataLen - ( headAddr - ReadBuff );

        memmove( ReadBuff, headAddr, pProtocolDealHandle->ReadDataLen );

        SaveLog( NULL, 0 );
      }

      if( pProtocolDealHandle->ReadDataLen < FRAME_MIN_SIZE )
      {
        return FRAME_LENTH_NOT_ENOUGH;
      }
    }
    //没有找到帧头
    else if( status == FRAME_HEADER_NOT_FINE )
    {
      //缓冲区只保留最后一个字节
      ReadBuff[0] = ReadBuff[pProtocolDealHandle->ReadDataLen - 1];
      pProtocolDealHandle->ReadDataLen = 1;

      SaveLog( NULL, 0 );
      return FRAME_HEADER_NOT_FINE;
    }
    //数据长度不够
    else
    {
      return FRAME_LENTH_NOT_ENOUGH;
    }

    status = FrameLenthIsOK( ReadBuff, pProtocolDealHandle->ReadDataLen, &frameLen );

    //数据长度不够
    if( status == FRAME_LENTH_NOT_ENOUGH )
    {
      return FRAME_LENTH_NOT_ENOUGH;
    }

    //帧长正确
    else if( status == FRAME_LENTH_OK )
    {
      frameLen += sizeof( OutputRxFrame->Header.Indicator ) + sizeof( OutputRxFrame->Header.FrameLen ) + sizeof( OutputRxFrame->CheckCode );

      if( pProtocolDealHandle->ReadDataLen >= frameLen )
      {
        //填充帧数据
        FrameLoadData( OutputRxFrame, ReadBuff );
      }
      else
      {
        return FRAME_LENTH_NOT_ENOUGH;
      }
    }
    //帧长有误
    else
    {
      //丢掉头部
      pProtocolDealHandle->ReadDataLen -= sizeof( OutputRxFrame->Header.Indicator );
      memmove( ReadBuff, ReadBuff + sizeof( OutputRxFrame->Header.Indicator ), pProtocolDealHandle->ReadDataLen );
      SaveLog( NULL, 0 );
      return FRAME_LENTH_ERROR;
    }

    //关闭对字节之间超时进行检测
    if( pProtocolDealHandle->TimeoutBetweenBytes.TimingFlag != TIM_BYTE_TIMEOUT_STOP )
    {
      pProtocolDealHandle->TimeoutBetweenBytes.TimingFlag = TIM_BYTE_TIMEOUT_STOP;
      pProtocolDealHandle->TimeoutBetweenBytes.time = 0xFFFFFFFF;
    }

    status = FrameCheckCodeIsOK( OutputRxFrame );

    if( status == FRAME_CHECKCODE_OK )
    {
      //检查Frame Type
      if( OutputRxFrame->Header.FrameType != LAUNCH_FRAME_1_TYPE &&\
          OutputRxFrame->Header.FrameType != RESPONSE_FRAME_TYPE &&\
          OutputRxFrame->Header.FrameType != RESULT_FRAME_TYPE &&\
          OutputRxFrame->Header.FrameType != LAUNCH_FRAME_4_TYPE )
      {
        //丢掉头部
        pProtocolDealHandle->ReadDataLen -= sizeof( OutputRxFrame->Header.Indicator );
        memmove( ReadBuff, ReadBuff + sizeof( OutputRxFrame->Header.Indicator ), pProtocolDealHandle->ReadDataLen );
        SaveLog( NULL, 0 );
        return FRAME_TYPE_ERROR;
      }

      //检查object unit
      if( OutputRxFrame->Header.ObjectUnit != pProtocolDealHandle->LocalObjectUnit )
      {
        //丢掉头部
        pProtocolDealHandle->ReadDataLen -= sizeof( OutputRxFrame->Header.Indicator );
        memmove( ReadBuff, ReadBuff + sizeof( OutputRxFrame->Header.Indicator ), pProtocolDealHandle->ReadDataLen );
        SaveLog( NULL, 0 );
        return FRAME_OBJECTUNIT_ERROR;
      }
      //判断重复帧
      if( OutputRxFrame->Header.FrameIndex == pProtocolDealHandle->LastFrameIndex )
      {
        //丢掉头部
        pProtocolDealHandle->ReadDataLen -= sizeof( OutputRxFrame->Header.Indicator );
        memmove( ReadBuff, ReadBuff + sizeof( OutputRxFrame->Header.Indicator ), pProtocolDealHandle->ReadDataLen );
        SaveLog( NULL, 0 );
        return FRAME_REPEAT_ERROR;
      }
      else
      {
        pProtocolDealHandle->LastFrameIndex = OutputRxFrame->Header.FrameIndex;
      }
      //删除正确部分数据
      pProtocolDealHandle->ReadDataLen -= frameLen;
      memmove( ReadBuff, ReadBuff + frameLen, pProtocolDealHandle->ReadDataLen );

      return FRAME_RX_OK;
    }
    else
    {
      //丢掉头部
      pProtocolDealHandle->ReadDataLen -= sizeof( OutputRxFrame->Header.Indicator );
      memmove( ReadBuff, ReadBuff + sizeof( OutputRxFrame->Header.Indicator ), pProtocolDealHandle->ReadDataLen );
      SaveLog( NULL, 0 );
      return FRAME_CHECKCODE_ERROR;
    }
  }

  return FRAME_LENTH_NOT_ENOUGH;
}

static FrameStatusDef BuildResponseFrame( ProtocolDealHandle *pProtocolDealHandle, FrameStatusDef ErrorType, const Frame *InputRxFrame, Frame *OutputTxFrame )
{
  u8 errorCode;
  bool flag = false;

  switch( ErrorType )
  {
    case FRAME_RX_OK:
    {
      errorCode = RESPONSE_OK;
      flag = true;
    }
    break;
    case FRAME_BYTE_TIMEOUT:
    {
      errorCode = RESPONSE_TIME_OUT;
      flag = true;
    }
    break;
    case FRAME_REPEAT_ERROR:
    {
      errorCode = RESPONSE_REPEAT_ERR;
      flag = true;
    }
    break;
    case FRAME_LENTH_ERROR:
    {
      errorCode = RESPONSE_LENTH_ERR;
      //flag = true;
    }
    break;
    case FRAME_OBJECTUNIT_ERROR:
    {
      errorCode = RESPONSE_OBJECTUNIT_ERR;
      flag = true;
    }
    break;
    case FRAME_CHECKCODE_ERROR:
    {
      errorCode = RESPONSE_CHECKCODE_ERR;
      flag = true;
    }
    break;

    default:
    {
      flag = false;
    }
    break;
  }

  if( flag )
  {
    OutputTxFrame->Header.Indicator.Header1 = FRAME_HEAD1;
    OutputTxFrame->Header.Indicator.Header2 = FRAME_HEAD2;
    OutputTxFrame->Header.FrameLen = FRAME_RESPONSE_LENTH;
    OutputTxFrame->Header.FrameType = RESPONSE_FRAME_TYPE;
    if( InputRxFrame != NULL )
    {
      OutputTxFrame->Header.FrameIndex = InputRxFrame->Header.FrameIndex;
    }
    OutputTxFrame->Header.ObjectUnit = pProtocolDealHandle->TagetObjectUnit;
    OutputTxFrame->Header.CommandCode = InputRxFrame->Header.CommandCode;
    OutputTxFrame->FramDate[0] = errorCode;
    return FRAME_BUILD_RESPONSE_OK;
  }
  else
  {
    return FRAME_BUILD_RESPONSE_ERROR;
  }
}

static FrameStatusDef BuildResultFrame( ProtocolDealHandle *pProtocolDealHandle, FrameStatusDef ErrorType, const Frame *InputRxFrame, Frame *OutputTxFrame )
{
  u8 errorCode;
  u32 len = 0;
  bool flag = false;

  switch( ErrorType )
  {
    case FRAME_REPEAT_ERROR:
    {
      flag = true;
      errorCode = RESPONSE_REPEAT_ERR;
    }
    break;
    case FRAME_TYPE_ERROR:
    {
      flag = true;
      errorCode = RESPONSE_TYPE_ERR;
    }
    break;
    case FRAME_OBJECTUNIT_ERROR:
    {
      flag = true;
      errorCode = RESPONSE_OBJECTUNIT_ERR;
    }
    break;
    case FRAME_CHECKCODE_ERROR:
    {
      flag = true;
      errorCode = RESPONSE_CHECKCODE_ERR;
    }
    break;

    default:
    {
      flag = false;
    }
    break;
  }

  if( flag == true )
  {
    OutputTxFrame->Header.Indicator.Header1 = FRAME_HEAD1;
    OutputTxFrame->Header.Indicator.Header2 = FRAME_HEAD2;
    OutputTxFrame->Header.FrameType = RESULT_FRAME_TYPE;
    OutputTxFrame->Header.FrameIndex = InputRxFrame->Header.FrameIndex;
    OutputTxFrame->Header.ObjectUnit = pProtocolDealHandle->TagetObjectUnit;
    OutputTxFrame->Header.CommandCode = InputRxFrame->Header.CommandCode;

    OutputTxFrame->Header.FrameLen = FRAME_DATA_MIN_SIZE + InputRxFrame->Header.FrameLen + 3;
    //拷贝数据段
    len = 0;
    OutputTxFrame->FramDate[len] = 0x01; //status
    len += 1;
    OutputTxFrame->FramDate[len] = InputRxFrame->FramDate[0];
    len += 1;
    OutputTxFrame->FramDate[len] = errorCode;
    len += 1;
    OutputTxFrame->FramDate[len] = 0x0B;
    len += 1;
    memcpy( OutputTxFrame->FramDate + len, InputRxFrame, InputRxFrame->Header.FrameLen + FRAME_DATA_MIN_SIZE - 2 );
    len += InputRxFrame->Header.FrameLen + FRAME_DATA_MIN_SIZE - 2;
    memcpy( OutputTxFrame->FramDate + len, ( u8* )&InputRxFrame->CheckCode, 2 );

    //加上本帧自己的数据长度
    OutputTxFrame->Header.FrameLen += FRAME_DATA_MIN_SIZE;
    if( OutputTxFrame->Header.FrameLen > PROTOCOL_DATA_SIZE )
    {
      return FRAME_BUILD_AUTO_RESULT_ERROR;
    }

    return FRAME_BUILD_AUTO_RESULT_OK;
  }
  else
  {
    return FRAME_BUILD_AUTO_RESULT_ERROR;
  }

}
/*
*********************************************************************************************************
*                                          SendFrame
*
* Description : 发送一帧数据
*
* Arguments   : pProtocolDealHandle 字节流协议句柄
*				InputTxFrame 要发送的数据
*				timeOut 超时时间
*
* Returns     : FrameStatusDef 帧状态
*
* Notes       :
*********************************************************************************************************
*/
FrameStatusDef SendFrame( ProtocolDealHandle *pProtocolDealHandle, const Frame *InputTxFrame, u32 timeOut )
{
  WriteDataFun TransmitData = pProtocolDealHandle->WriteData;

  u32 status;
  u32 tick;
  if( pProtocolDealHandle == NULL )
  {
    return PROTOCOL_HANDLE_PARAMETER_ERROR;
  }

  if( InputTxFrame == NULL )
  {
    return FRAME_TX_PARAMETER_ERROR;
  }

  status = BuildFrame( pProtocolDealHandle, InputTxFrame );
  if( status != FRAME_BUILD_OK )
  {
    return FRAME_BUILD_ERROR;
  }

  if( timeOut > 0 )
  {
    tick = HAL_GetTick();

    do
    {
      if( HAL_GetTick() - tick > timeOut )
      {
        return FRAME_TX_TIMEOUT;
      }

      status = TransmitData( pProtocolDealHandle->WriteBuff, pProtocolDealHandle->WriteDataLen );

    } while( status == BSP_TX_QUEUE_FULL );
  }
  else
  {
    status = TransmitData( pProtocolDealHandle->WriteBuff, pProtocolDealHandle->WriteDataLen );
  }

  if( status == BSP_OK )
  {
    return FRAME_TX_OK;
  }

  return FRAME_TX_ERROR;
}

/*
*********************************************************************************************************
*                                          ReadFrame
*
* Description : 接收一帧数据
*
* Arguments   : pProtocolDealHandle 字节流协议句柄
*				OutputRxFrame 接收到的数据
*				timeOut 超时时间
*
* Returns     : FrameStatusDef 帧状态
*
* Notes       :
*********************************************************************************************************
*/

FrameStatusDef ReadFrame( ProtocolDealHandle *pProtocolDealHandle, Frame *OutputRxFrame, u32 timeOut )
{
  FrameStatusDef Dealstatus, BuildStatus;
  Frame TxResponseFrame;
  u32 SystemTick;

  SystemTick = HAL_GetTick();
  do
  {
    memset( ( u8* )OutputRxFrame, 0, sizeof( Frame ) );
    Dealstatus = FrameDeal( pProtocolDealHandle, OutputRxFrame, timeOut );
    if( HAL_GetTick() - SystemTick > timeOut )
    {
      break;
    }
  } while( Dealstatus == FRAME_LENTH_NOT_ENOUGH );

  if( ( OutputRxFrame->Header.FrameType == LAUNCH_FRAME_1_TYPE ) || ( OutputRxFrame->Header.FrameType == LAUNCH_FRAME_4_TYPE ) )
  {
    BuildStatus = BuildResponseFrame( pProtocolDealHandle, Dealstatus, OutputRxFrame, &TxResponseFrame );
    if( BuildStatus == FRAME_BUILD_RESPONSE_OK )
    {
      SendFrame( pProtocolDealHandle, &TxResponseFrame, 0 );

      if( OutputRxFrame->Header.FrameType == LAUNCH_FRAME_1_TYPE )
      {
        BuildStatus = BuildResultFrame( pProtocolDealHandle, Dealstatus, OutputRxFrame, &TxResponseFrame );
        if( BuildStatus == FRAME_BUILD_AUTO_RESULT_OK )
        {
          SendFrame( pProtocolDealHandle, &TxResponseFrame, 0 );
        }

      }
    }
    else
    {
      return FRAME_BUILD_RESPONSE_ERROR;
    }
  }
  if( Dealstatus != FRAME_RX_OK )
  {
    SaveLog( NULL, 0 );
  }
  return Dealstatus;
}




