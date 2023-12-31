/***********************************************************
Module Name: peripheralinit.c
Description: 外设初始化
Module Date: 12-08-2016
Module Author: Firmware-Team
Others:
***********************************************************/


#include "includes.h"


#include "AD7124.h"
#include "AD7124_regs.h"
#include "gpio.h"
#include "AD7124_temp.h"
#include "pid.h"

void PeripheralInit( void )
{
  BSP_RCC_PLL_Config();

  /* STM32F405x/407x/415x/417x Revision Z devices: ...... */
  /* ....prefetch is supported                            */
  if( HAL_GetREVID() == 0x1001 ) /*获取芯片UID*/
  {
    //Debug_Printf("HAL_GetREVID:0x%04X\r\n",HAL_GetREVID());

    /* Enable the Flash prefetch */
    __HAL_FLASH_PREFETCH_BUFFER_ENABLE(); /*使能片上FLIASH*/
  }

  /*! 配置IO口 */
  ConfigGpio();

  /*! 初始化串口1 作为RS232主串口进行上位机通信*/
  Uart1Config();

  /*! 初始化串口3 作为Dubug串口调试打印数据*/
  Uart3Config();

  /*! 初始化SPI1 */
  ConfigSPI1();

  /*! 初始化SPI2==》 AD7124-8有一个3线或4线SPI接口，该接口以SPI模式3工作*/
  Spi2Config();

  /*! 初始化I2C1  初始化EEPROM*/
  I2C1Config();

  /*! eeprom 初始化 */
  EepromInit( &I2c1Handle );

  /*! 初始化AD7124 */
  TEMP_InitAd7124();

  /*! TEC初始化*/
  TecInit();
}

