/**
  ******************************************************************************
  * @file    appTest.c
  * @author  Firmware-Team
  * @version
  * @date    05-01-2017
  * @brief
  ******************************************************************************
**/

/*
*********************************************************************************************************
*                                             INCLUDE FILES
*********************************************************************************************************
*/
#include  <includes.h>
#include "appTest.h"
#include "temperature_control.h"
#include "AD7124.h"
#include "AD7124_temp.h"
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
#define   TEST_SET_TARGET        false
#define   TEST_SET_PARAM         false
#define   TEST_SET_TARGET_CAL    false

/*
*********************************************************************************************************
*                                           FUNCTION PROTOTYPES
*********************************************************************************************************
*/

s32 FunctionTest( void )
{
  s32 ret = 0;
  //	u32 tickStart;
  //	s32 TempData[4][10];


#if BSP_USART1_EN
  Usart1Test( NULL );
#endif

#if BSP_USART3_EN
  Usart3Test( NULL );
#endif

#if I2C1_EN
  I2c1Test();
#endif

#if I2C3_EN
  I2c3Test();
#endif

#if SPI1_EN
  TestForSpi1( NULL );
#endif

#if SPI2_EN
  Spi2Test( NULL );
#endif

#if SPI4_EN
  Spi4Test( NULL );
#endif

#if SPI5_EN
  Spi5Test( NULL );
#endif

#if TIM1_EN
  //PwmTest();
#endif

#if CRC_EN
  CrcTest();
#endif

#if ADC_TEST

  if( ( HAL_GetTick() - tickStart )>= 1000 )
  {
    tickStart = HAL_GetTick();
    /* Read data from the ADC */
    TEMP_GetTempData( 0, 1, TempData[0] );
    Debug_Printf( "CH:%d,data : %d \r", 0, TempData[0][0] );
    TEMP_GetTempData( 1, 1, TempData[1] );
    Debug_Printf( "CH:%d,data is : %d \r", 1, TempData[1][0] );
    TEMP_GetTempData( 2, 1, TempData[2] );
    Debug_Printf( "CH:%d,data is : %d \r", 2, TempData[2][0] );
    TEMP_GetTempData( 3, 1, TempData[3] );
    Debug_Printf( "CH:%d,data is : %d \r\n", 3, TempData[3][0] );

  }

#endif
#if PRINT_DATA_TEST

  //			AD7124_DisplayRegSettings();

#endif

#if GPIO_TEST


#endif

#if TEC_TEST
  TecTest();
#endif

#if SET_TARGET
  /*! 设置目标温度 */
  ret = SetTargetTemperature( 1, 2000 );
  if( ret ==false )
  {
    Debug_Printf( "target value abnormal !!!\r\n" );

  }
#endif

  return ret ;
}
