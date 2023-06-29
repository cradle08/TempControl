/*******************************************************************************
  * @file    AD7124_temp.c
  * @author  Yan xiangwen
  * @version V1.0.0
  * @date    26-09-2016
  * @brief   温度读取
  *****************************************************************************/

/*
*********************************************************************************************************
*                                             INCLUDE FILES
********************************************************************************************************/
#include "AD7124.h"
#include "includes.h"
#include "AD7124_temp.h"
#include "string.h"
#include "loopbuff4b.h"

/*********************************************************************************************************
 *                                            LOCAL DEFINES
 ********************************************************************************************************/
#define TMP_QUEUE_MAX    5
//#define  PrintRef

static Queue4b TmpQueueHandle[DIFF_CHN_MAX];
static u32     TmpQueueBuffer[DIFF_CHN_MAX][TMP_QUEUE_MAX];
/********************************************************************************************************
 *                                        LOCAL GLOBAL VARIABLES
 ********************************************************************************************************/
static u32 gTempSampTms = 0;
static u8  gCurrTempChn = DIFF_CHN_1;

TEMP_SAMPLE stTempSample[DIFF_CHN_MAX];

extern uint32_t AD7124_ReadRegOut( unsigned char reg );
/*********************************************************************************
 *Function Name: AD7124_TimerGetTempConfig
 *  Description: 初始化定时器15,用于采样中断
 *  Arguments
         Inputs: 无
        Outputs: 无
 *  Note(s)    :
**********************************************************************************/
s32 TEMP_ParameterInit( void )
{
  u8 idx;

  for( idx=DIFF_CHN_1; idx<DIFF_CHN_MAX; idx++ )
  {
    stTempSample[idx].TrigErrTimes = 0;
    stTempSample[idx].ReadErrTimes = 0;
    stTempSample[idx].ConvStat     = FALSE;
    stTempSample[idx].BufReading   = FALSE;
    stTempSample[idx].WriteIndex   = 0;

    memset( ( u32 * )TmpQueueBuffer[idx], 0x00, TMP_QUEUE_MAX*4 );

    if( !initQueue4b( ( Queue4b * )&TmpQueueHandle[idx], ( u32 * )TmpQueueBuffer[idx], TMP_QUEUE_MAX ) )
    {
      // save log
      return AD7124_ERR_MEM_LESS;
    }

  }

  return AD7124_OK;
}

/*********************************************************************************
 *Function Name: Tim15ClkEnable
 *  Description: 定时器15时钟使能
 *  Arguments
         Inputs: 无
        Outputs: 无
 *  Note(s)    :
**********************************************************************************/
static void Tim15ClkEnable( void )
{
  __HAL_RCC_TIM15_CLK_ENABLE();
}


void quickSort( u32 *a, int n )
{

  int i, j, k;
  u32 temp = 0;
  for( i=0; i<n-1; i++ )
  {
    k=i;

    for( j=i+1; j<n; j++ )

      if( a[k]>a[j] ) k=j;

    if( i!=k )
    {

      temp=a[i];

      a[i]=a[k];

      a[k]=temp;
    }
  }
}

u32 TempBuffDeal( u32 *InputBuf, u32 InputLen )
{
  u32 MidLen;

  quickSort( InputBuf, InputLen );

  MidLen = InputLen/2 ;

  return InputBuf[MidLen];

}

/*********************************************************************************
 *Function Name: Tim15CallBackIT
 *  Description: 采样中断响应程序
 *  Arguments
         Inputs: 无
        Outputs: 无
 *  Note(s)    :
**********************************************************************************/
static void Tim15CallBackIT( void )
{
  s32  adcValue;

  //	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);

  if( gTempSampTms > 0 )
  {
    adcValue = 0;

    // 读取ADC数据
    if( stTempSample[gCurrTempChn].ConvStat )
      adcValue = AD7124_GetAdcValue();
    else
      adcValue = 0;

    //		if( adcValue>=0xFFFFFF )
    if( adcValue > 0xFFFFFF )
      adcValue = 0;

    //		if( adcValue>0 )
    //		{
    if( isQueueFull4b( ( Queue4b * )&TmpQueueHandle[gCurrTempChn] ) )
    {
      deQueue4b( ( Queue4b * )&TmpQueueHandle[gCurrTempChn] );

      if( false == isQueueFull4b( ( Queue4b * )&TmpQueueHandle[gCurrTempChn] ) )
      {
        enQueue4b( ( Queue4b * )&TmpQueueHandle[gCurrTempChn], adcValue );
      }
    }
    else
    {
      enQueue4b( ( Queue4b * )&TmpQueueHandle[gCurrTempChn], adcValue );
    }
    //		}


    gCurrTempChn += 1;
    if( gCurrTempChn > DIFF_CHN_4 )
      gCurrTempChn = DIFF_CHN_1;
  }
  else
  {
    gCurrTempChn = DIFF_CHN_1;
  }


  if( gTempSampTms < 0xFFFFFFFF )
    gTempSampTms += 1;


  if( AD7124_Convert( gCurrTempChn )<0 )
  {

    stTempSample[gCurrTempChn].ConvStat = FALSE;

    stTempSample[gCurrTempChn].TrigErrTimes += 1;
  }
  else
  {

    stTempSample[gCurrTempChn].ConvStat = TRUE;
  }

  //	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
}

/*********************************************************************************
 *Function Name: TEMP_GetTempData
 *  Description: 读取数据,要求读取频率大于25毫秒
 *  Arguments
         Inputs: chnidx: 通道选择,(0/1/2/3)
                 tempbuff: 数据缓存,10个无符号长整形数组
        Outputs: 通道异常返回-1,
               : 缓存数据小于要求读取的数量,返回-2
               : 正常则返回0
 *  Note(s)    : 外部调用接口
**********************************************************************************/
s32 TEMP_GetTempData( u8 chnidx, u8 DatNum, s32 *tempbuff )
{
  u8  datidx;
  u32 buff[TMP_QUEUE_MAX];
  float temp;

  *tempbuff = 0;
  if( chnidx > DIFF_CHN_4 )
    return AD7124_ERR_PARA_INPUT;

  if( 0==DatNum )
    return AD7124_OK;

  datidx = queueFullN4b( ( Queue4b * )&TmpQueueHandle[chnidx] );

  // 数据空,返回错误
  if( 0 == datidx )
    return AD7124_ERR_QUEUE_NONE;

  if( DatNum > datidx )
    DatNum = datidx;


  stTempSample[chnidx].BufReading = TRUE;
#if 0
  for( datidx=0; datidx<DatNum; datidx++ )
  {
    tempbuff[datidx] = deQueue4b( &TmpQueueBuffer[chnidx], );

  }
#endif

#if 1
  memcpy( ( u8* )buff, ( u8* )&TmpQueueBuffer[chnidx], TMP_QUEUE_MAX * 4 );
  for( datidx=0; datidx<DatNum; datidx++ )
  {
    tempbuff[datidx] = TempBuffDeal( buff, TMP_QUEUE_MAX );
#if 1
    if( ( tempbuff[datidx] <= 0 ) || ( tempbuff[datidx] >= 0xFFFFFF ) )
      return  AD7124_ERR_QUEUE_NONE;
#endif
  }
#endif

  stTempSample[chnidx].BufReading = FALSE;

  // 计算
  for( datidx=0; datidx<DatNum; datidx++ )
    //	for(datidx=0; datidx<1; datidx++)
  {
    temp = ( float )tempbuff[datidx];

    if( temp>0 )
    {
      temp = TEMP_GetResisterValue( temp );
      temp = TEMP_GetTempValue( temp );
    }
    else
    {
      temp = 0;
    }

    tempbuff[datidx] = ( s32 )( ( double )temp*100+0.5 );
  }

  return AD7124_OK;
}

/*********************************************************************************
 *Function Name: AD7124_TimerGetTempConfig
 *  Description: 初始化定时器15,用于采样中断
 *  Arguments
         Inputs: 无
        Outputs: 无
 *  Note(s)    :
**********************************************************************************/
void TEMP_TimerGetTempConfig( void )
{
  uint32_t  apb2clk = HAL_RCC_GetPCLK2Freq();
  uint32_t  timerms = 25;

#if TIMER15_ADC_MS==TIM_MS_10
  timerms = TIM_MS_10;
#endif

  Tim15Handle.TimHandleInit.Instance 				    = TIM15;
  Tim15Handle.TimHandleInit.Init.ClockDivision 	= 0;
  Tim15Handle.TimHandleInit.Init.Prescaler	 	  = 49;
  Tim15Handle.TimHandleInit.Init.Period		 	    = apb2clk/( Tim15Handle.TimHandleInit.Init.ClockDivision+1 )/1000*timerms/( Tim15Handle.TimHandleInit.Init.Prescaler+1 );
  Tim15Handle.TimHandleInit.Init.CounterMode  	= TIM_COUNTERMODE_UP;

  Tim15Handle.TimClkEnable 						          = Tim15ClkEnable;

  Tim15Handle.IrqInit.Irqn						          = TIM15_IRQn;
  Tim15Handle.IrqInit.PreemptPriority				    = 15;
  Tim15Handle.IrqInit.SubPriority					      = 0;

  Tim15Handle.TimCallBack 						          = Tim15CallBackIT;

  if( BspTimHandleInit( &Tim15Handle ) != HAL_OK )
  {
    // Initialization Error
    SaveLog( NULL, 0 );
  }
  else
  {
    BspTimHandleStartIT( &Tim15Handle );
  }
}


/*********************************************************************************
 *Function Name: GetResisterValue
 *  Description: 输入AD7124的数字值,转换成PT100的电阻值
 *  Arguments
         Inputs: adcValue, AD数字值
        Outputs: 电阻值
 *  Note(s)    : GetTempValue
**********************************************************************************/
float TEMP_GetResisterValue( u32 adcValue )
{

#if TIMER15_ADC_MS == TIM_MS_10
  return ( double )adcValue*250/0xFFFFFF;

#else
  return ( ( double )adcValue*2/( 0xFFFFFF+1 )-1 )*2.005*1000/8;
  //	return (double)adcValue*250/0x7FFFFF;
#endif    // #if TIMER15_ADC_MS==TIM_MS_10
}


#define  A   3.9083e-3
#define  B   -5.775e-7
#define  C   -4.183e-12
/*********************************************************************************
 *Function Name: GetTempValue
 *  Description: 输入电阻值,计算相应的温度值
 *  Arguments
         Inputs: R, 电阻值
        Outputs: 温度值
 *  Note(s)    :
**********************************************************************************/
float TEMP_GetTempValue( float R )
{

  //	return ((double)(R/100 - 1)/0.003851);


  double  pttemp;


  if( ( R >= 100 )&&( R <= 390.481f ) ) // 0-850
  {
    pttemp = A*A- 4 * B * ( 1 - R/100 );
    pttemp = pttemp*1000000;
    pttemp = sqrt( pttemp )/1000;
    pttemp = ( pttemp-A ) / ( 2 * B );

    return ( float )pttemp;
  }
  else
  {
    return ( ( double )( R/100 - 1 )/0.003851f );
  }
}


/*********************************************************************************
 *Function Name: TEMP_InitAd7124
 *  Description: 初始化AD7124,中断定时器和相关数据变量
 *  Arguments
         Inputs: 无
        Outputs: 状态,正常返回0
 *  Note(s)    :
**********************************************************************************/
s32 TEMP_InitAd7124( void )
{

  if( 0!=ConfigAD7124() )
    return AD7124_ERR_NO;

  if( AD7124_OK!=TEMP_ParameterInit() )
    return AD7124_ERR_MEM_LESS;


  if( 0!=AD7124_UpdateDevSpiSettings() )
    return AD7124_ERR_NO;

  TEMP_TimerGetTempConfig();

  return AD7124_OK;
}

