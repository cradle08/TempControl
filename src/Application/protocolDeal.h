/**
  ******************************************************************************
  * @file    protocolDeal.h
  * @author  Firmware-Team
  * @version V1.0.0
  * @date    13-06-2016
  * @brief
  ******************************************************************************
**/

#ifndef  __PROTOCOL_DEAL__
#define  __PROTOCOL_DEAL__

/*
*********************************************************************************************************
*                                              INCLUDE FILES
*********************************************************************************************************
*/
#include  <uccpu.h>
#include "stm32f3xx_hal.h"
#include "bsp_msp_port.h"

/*
*********************************************************************************************************
*                                               INT DEFINES
*********************************************************************************************************
*/


#define FRAME_HEAD1 			0x55
#define FRAME_HEAD2				0xaa
#define FRAME_HEAD				0xaa55
#define FRAME_MIN_SIZE			0x0B

#define PROTOCOL_DATA_SIZE		512 - FRAME_MIN_SIZE


#define TIM_BYTE_TIMEOUT_START 0x01
#define TIM_BYTE_TIMEOUT_STOP 0x00

#define BigtoLittle16(A)   (( ((uint16)(A) & 0xff00) >> 8)    |   \
                           (( (uint16)(A) & 0x00ff) << 8))

#define BigtoLittle32(A)   ((( (uint32)(A) & 0xff000000) >> 24) |  \
                           (( (uint32)(A) & 0x00ff0000) >> 8)   |  \
                           (( (uint32)(A) & 0x0000ff00) << 8)   |  \
                           (( (uint32)(A) & 0x000000ff) << 24))



/*
********************************************************************************************************
*                                        LOCAL GLOBAL VARIABLES
*********************************************************************************************************
*/
/*定义函数指针*/
typedef u32( *ReadDataFun )( u8 *InputBuff, u32 InputLen, u32 *OutputLen );
typedef u32( *WriteDataFun )( u8 *InputBuff, u32 InPutLen );
typedef void ( *InitFun )( void );
typedef u16( *CrcFun )( u8 *InputBuff, s32 InputLen );
typedef BspStatusTypeDef( *TimDealFun )( BspTimHandle *BspTimxHandle );

typedef enum
{
  FRAME_HEADER_OK = 1,
  FRAME_LENTH_NOT_ENOUGH,
  FRAME_HEADER_NOT_FINE,
  FRAME_BYTE_TIMEOUT,
  FRAME_REPEAT_ERROR,
  FRAME_LENTH_ERROR,
  FRAME_LENTH_OK,
  FRAME_CHECKCODE_OK,
  FRAME_CHECKCODE_ERROR,
  FRAME_TYPE_ERROR,
  FRAME_OBJECTUNIT_ERROR,
  PROTOCOL_HANDLE_PARAMETER_ERROR,
  FRAME_INIT_OK,
  FRAME_RX_PARAMETER_ERROR,
  FRAME_RX_ERROR,
  FRAME_RX_OK,
  FRAME_RX_TIMEOUT,
  FRAME_BUILD_ERROR,
  FRAME_BUILD_OK,
  FRAME_BUILD_RESPONSE_ERROR,
  FRAME_BUILD_RESPONSE_OK,
  FRAME_BUILD_AUTO_RESULT_ERROR,
  FRAME_BUILD_AUTO_RESULT_OK,
  FRAME_TX_PARAMETER_ERROR,
  FRAME_TX_ERROR,
  FRAME_TX_OK,
  FRAME_TX_TIMEOUT
} FrameStatusDef;

typedef enum
{
  LAUNCH_FRAME_1_TYPE = 0x01,  /*发送类型1*/
  RESPONSE_FRAME_TYPE,         /*回复类型*/
  RESULT_FRAME_TYPE,           /*结果类型*/
  LAUNCH_FRAME_4_TYPE          /*发送类型1*/
} FrameTypeDef;

typedef enum
{
  PC_TYPE = 0,
  IO_TYPE,
  TEMPEREATURE_TYPE
} ObjectUnitDef;

typedef enum
{
  RESPONSE_OK = 0,
  RESPONSE_TIME_OUT,
  RESPONSE_TYPE_ERR,
  RESPONSE_OBJECTUNIT_ERR,
  RESPONSE_PROTOCOL_ERR,
  RESPONSE_PARAMETER_ERR,
  RESPONSE_REPEAT_ERR,
  RESPONSE_LENTH_ERR,
  RESPONSE_CHECKCODE_ERR = 0xff
} ResponseErrorCodeDef;


//__packed是字节对齐,按照结构体内部定义的数据类型对齐
__packed
typedef struct
{
  u8 TimeValue;
  u8 TimeUnit;
  u8 TimingFlag;
  u32 time;
} TimeoutBetweenBytesDef;

__packed
typedef struct
{
  u8 Header1;
  u8 Header2;
} Heard;

__packed
typedef struct
{
  Heard Indicator;
  u16 FrameLen;
  u8 	FrameType;
  u16 FrameIndex;
  u8 	ObjectUnit;
  u8  CommandCode;
} FrameHeader;

__packed
typedef struct
{
  FrameHeader Header;
  u8 	FramDate[PROTOCOL_DATA_SIZE];
  u16 CheckCode;
} Frame;

typedef struct
{
  u8 LocalObjectUnit;    /*本地对象单元*/
  u8 TagetObjectUnit;    /*目标对象单元*/

  ReadDataFun ReadData;  /*协议结构体里面的读取数据*/
  u8 *ReadBuff;
  u32 ReadBuffSize;
  u32 ReadDataLen;

  WriteDataFun WriteData;/*协议结构体里面的写数据*/
  u8 *WriteBuff;
  u32 WriteBuffSize;
  u32 WriteDataLen;

  InitFun CrcInit;       /*协议结构体里面的CRC数据*/
  CrcFun Crc16;

  TimeoutBetweenBytesDef TimeoutBetweenBytes; /*字节之间的超时定义*/
  u16 LastFrameIndex;   /*最后一帧索引*/
} ProtocolDealHandle;


/*
*********************************************************************************************************
*                                           FUNCTION PROTOTYPES
*********************************************************************************************************
*/
extern FrameStatusDef FrameDealInit( ProtocolDealHandle *pProtocolDealHandle );
extern FrameStatusDef ReadFrame( ProtocolDealHandle *pProtocolDealHandle, Frame *OutputRxFrame, u32 timeOut );
extern FrameStatusDef SendFrame( ProtocolDealHandle *pProtocolDealHandle, const Frame *InputTxFrame, u32 timeOut );


/*
*********************************************************************************************************
*                                             MODULE END
*********************************************************************************************************
*/




#endif /* End */

