/**
  ******************************************************************************
  * @file    protocolTest.c
  * @author  linwenxiang
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


/*
*********************************************************************************************************
*                                               INT DEFINES
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                            LOCAL DEFINES
*********************************************************************************************************
*/
#define UART1_READ_BUFF_SIZE	512
#define UART1_SEND_BUFF_SIZE	512



/*
*********************************************************************************************************
*                                        LOCAL GLOBAL VARIABLES
*********************************************************************************************************
*/
ProtocolDealHandle	UartProtocolDealHandle;

u8 Uart1ReadBuff[UART1_READ_BUFF_SIZE];
u8 Uart1SendBuff[UART1_SEND_BUFF_SIZE];




/*
*********************************************************************************************************
*                                         FUNCTION PROTOTYPES
*********************************************************************************************************
*/
u32 UartReadData( u8 *InputBuff, u32 InputLen, u32 *OutputLen )
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
      OutputLen = 0;
      break;
  }
  return status;
}

u32 UartSendData( u8 *InputBuff, u32 InPutLen )
{
  BspStatusTypeDef status;
  status = BspUartTransmitIT( &BspUart1Handle, InputBuff, InPutLen );
  return status;
}

void protocolHandleConfig( void )
{
  UartProtocolDealHandle.LocalObjectUnit = 0x01;
  UartProtocolDealHandle.TagetObjectUnit = 0x02;
  UartProtocolDealHandle.ReadData = UartReadData;
  UartProtocolDealHandle.ReadBuff = Uart1ReadBuff;
  UartProtocolDealHandle.ReadBuffSize = UART1_READ_BUFF_SIZE;
  UartProtocolDealHandle.WriteData = UartSendData;
  UartProtocolDealHandle.WriteBuff = Uart1SendBuff;
  UartProtocolDealHandle.WriteBuffSize = UART1_SEND_BUFF_SIZE;
  UartProtocolDealHandle.CrcInit = crcInit;
  UartProtocolDealHandle.Crc16 = ( CrcFun )crcFast;
  UartProtocolDealHandle.TimeoutBetweenBytes.TimeValue = 10;
  UartProtocolDealHandle.TimeoutBetweenBytes.TimeUnit = 0x02;
  UartProtocolDealHandle.TimeoutBetweenBytes.TimingFlag = TIM_BYTE_TIMEOUT_STOP;
  UartProtocolDealHandle.LastFrameIndex = 0xffff;
  FrameDealInit( &UartProtocolDealHandle );
}

//例程
void FrameTest( void )
{
  u32 status;
  Frame RxFrame;
  //轮询接收数据
  status = ReadFrame( &UartProtocolDealHandle, &RxFrame, 0 );
  if( status == FRAME_RX_OK )
  {
    //接收到完整一帧数据
    //返回确认帧
  }
}
