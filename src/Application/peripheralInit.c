/***********************************************************
Module Name: peripheralinit.c
Description: �����ʼ��
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
  if( HAL_GetREVID() == 0x1001 ) /*��ȡоƬUID*/
  {
    //Debug_Printf("HAL_GetREVID:0x%04X\r\n",HAL_GetREVID());

    /* Enable the Flash prefetch */
    __HAL_FLASH_PREFETCH_BUFFER_ENABLE(); /*ʹ��Ƭ��FLIASH*/
  }

  /*! ����IO�� */
  ConfigGpio();

  /*! ��ʼ������1 ��ΪRS232�����ڽ�����λ��ͨ��*/
  Uart1Config();

  /*! ��ʼ������3 ��ΪDubug���ڵ��Դ�ӡ����*/
  Uart3Config();

  /*! ��ʼ��SPI1 */
  ConfigSPI1();

  /*! ��ʼ��SPI2==�� AD7124-8��һ��3�߻�4��SPI�ӿڣ��ýӿ���SPIģʽ3����*/
  Spi2Config();

  /*! ��ʼ��I2C1  ��ʼ��EEPROM*/
  I2C1Config();

  /*! eeprom ��ʼ�� */
  EepromInit( &I2c1Handle );

  /*! ��ʼ��AD7124 */
  TEMP_InitAd7124();

  /*! TEC��ʼ��*/
  TecInit();
}
