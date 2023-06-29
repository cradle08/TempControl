#ifndef __AD7124_TEMP_H__
#define __AD7124_TEMP_H__

enum tagAD7124_ERROR
{
  AD7124_OK               = 0,
  AD7124_ERR_NO           = -1,     // AD7124联接异常
  AD7124_ERR_MEM_LESS     = -2,     // 内存不足
  AD7124_ERR_PARA_INPUT   = -3,     // 传输参数错误
  AD7124_ERR_QUEUE_NONE   = -4,     // 无数据
};

typedef struct TempSample
{
  uint32_t TrigErrTimes;            // 触发采样错误次数
  uint32_t ReadErrTimes;            // 读取数据错误次数
  uint8_t  ConvStat;                // 转换状态
  uint8_t  BufReading;              // 正在读取数据
  uint8_t  WriteIndex;			  // 读取数据次数
} TEMP_SAMPLE, *P_TEMP_SAMPLE;



// functions
void  TEMP_TimerGetTempConfig( void );
s32   TEMP_ParameterInit( void );
s32   TEMP_GetTempData( u8 chnidx, u8 DatNum, s32 *tempbuff );
float TEMP_GetResisterValue( u32 adcValue );
float TEMP_GetTempValue( float R );
s32   TEMP_InitAd7124( void );
void  TEMP_GetAd7124Settings( uint8_t len, uint32_t *RegSet );

#endif

