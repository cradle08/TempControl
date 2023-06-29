/**
  ******************************************************************************
  * @file    main.c
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
#include  "includes.h"
#include  "peripheralInit.h"
#include  "protocolAnalysis_app.h"
#include  "temperature_control.h"
#include  "AD7124_regs.h"
#include  "AD7124.h"
#include  "protocolDeal.h"
#include  "AD7124_temp.h"
#include  "PID.h"
#include  "appTest.h"
#include  "gpio.h"
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
*                                       TASK LOCAL GLOBAL VARIABLES
*********************************************************************************************************
*/





/*
*********************************************************************************************************
*                                         FUNCTION PROTOTYPES
*********************************************************************************************************
*/

/*
*********************************************************************************************************
* Description : This is the standard entry point for C code.  It is assumed that your code will call
*               main() once you have performed all necessary initialization.
*
* Notes       : 1) STM32F4xx HAL library initialization:
*                      a) Configures the Flash prefetch, intruction and data caches.
*                      b) Configures the Systick to generate an interrupt. However, the function ,
*                         HAL_InitTick(), that initializes the Systick has been overwritten since Micrium's
*                         RTOS has its own Systick initialization and it is recommended to initialize the
*                         Systick after multitasking has started.
*********************************************************************************************************
*/

Frame RxFrame;
int main( void )
{
  //	Frame RxFrame;
  u32 RxStatus  = 0;
  s32 TxStatus  = 0;
  u32 tickStart = HAL_GetTick();  //获取时钟数

  /*外部设备初始化*/
  HAL_Init();						/* See Note 1. */

  PeripheralInit();    //外设初始化
	
  /*! 配置通信协议参数 参考协议：《Communication Protocol-V2-wenxiang - 3.xls》 */
  ConfigProtocolParam();

  /*! 应用层 参数初始化 */
  TemperatureParamInit();
	
	Debug_Printf("Version=%s [%s %s %s]", "V1.0.0",__FILE__,__func__,__LINE__);
	
#if 1
  while( DEF_ON )
  {
    /*! 通过串口获取数据 */
    RxStatus = ReadFrame( &protocolAnalysisHandle, &RxFrame, 1 );

    /*! 协议解析 */
    if( FRAME_RX_OK == RxStatus )
    {
      TxStatus = AnalysisFrameType( &RxFrame ); //分析协议框架类型
      if( TxStatus != RESULT_FRAME_BUILD_OK )
      {

      }
    }
    else
    {

    }

    /*! 温度控制  每1s更新一次PID温度*/
    if( ( HAL_GetTick() - tickStart ) >= 1000 )
    {
      tickStart = HAL_GetTick(); /*tickStart获取最新的Tick*/
			
      /*TempControlSystem PID设置温度*/
      TempControlSystem( CHA, &stTemperatureControlA, &PidInfoA ); //A平台 channelNum=1 控制CON4口 CON3口
			TempControlSystem( CHC, &stTemperatureControlC, &PidInfoC ); //C平台 channelNum=3 控制CON2口 CON2口				
      // AD7124_DisplayRegSettings();
      /*! for test */
      // FunctionTest();
    }
  }
#endif	
}


