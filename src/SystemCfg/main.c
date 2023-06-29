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
  u32 tickStart = HAL_GetTick();  //��ȡʱ����

  /*�ⲿ�豸��ʼ��*/
  HAL_Init();						/* See Note 1. */

  PeripheralInit();    //�����ʼ��
	
  /*! ����ͨ��Э����� �ο�Э�飺��Communication Protocol-V2-wenxiang - 3.xls�� */
  ConfigProtocolParam();

  /*! Ӧ�ò� ������ʼ�� */
  TemperatureParamInit();
	
	Debug_Printf("Version=%s [%s %s %s]", "V1.0.0",__FILE__,__func__,__LINE__);
	
#if 1
  while( DEF_ON )
  {
    /*! ͨ�����ڻ�ȡ���� */
    RxStatus = ReadFrame( &protocolAnalysisHandle, &RxFrame, 1 );

    /*! Э����� */
    if( FRAME_RX_OK == RxStatus )
    {
      TxStatus = AnalysisFrameType( &RxFrame ); //����Э��������
      if( TxStatus != RESULT_FRAME_BUILD_OK )
      {

      }
    }
    else
    {

    }

    /*! �¶ȿ���  ÿ1s����һ��PID�¶�*/
    if( ( HAL_GetTick() - tickStart ) >= 1000 )
    {
      tickStart = HAL_GetTick(); /*tickStart��ȡ���µ�Tick*/
			
      /*TempControlSystem PID�����¶�*/
      TempControlSystem( CHA, &stTemperatureControlA, &PidInfoA ); //Aƽ̨ channelNum=1 ����CON4�� CON3��
			TempControlSystem( CHC, &stTemperatureControlC, &PidInfoC ); //Cƽ̨ channelNum=3 ����CON2�� CON2��				
      // AD7124_DisplayRegSettings();
      /*! for test */
      // FunctionTest();
    }
  }
#endif	
}


