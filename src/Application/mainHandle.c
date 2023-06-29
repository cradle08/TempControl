/**
  ******************************************************************************
  * @file    mainHandle.c
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
//#define SPI2_BUFF_SIZE 64
#define SPI4_BUFF_SIZE 1024
#define SPI5_BUFF_SIZE 64



/*
*********************************************************************************************************
*                                        LOCAL GLOBAL VARIABLES
*********************************************************************************************************
*/
//u8 gSpi2TxBuff[SPI4_BUFF_SIZE];
//u8 gSpi2RxBuff[SPI4_BUFF_SIZE];
u8 gSpi4TxBuff[SPI4_BUFF_SIZE];
u8 gSpi4RxBuff[SPI4_BUFF_SIZE];
u8 gSpi5TxBuff[SPI4_BUFF_SIZE];
u8 gSpi5RxBuff[SPI4_BUFF_SIZE];




/*
*********************************************************************************************************
*                                         FUNCTION PROTOTYPES
*********************************************************************************************************
*/
u32 gSpi4TestTxCount;
u32 gSpi4TestRxCount;
u32 gSpi4TestErrCount;
void PwmTest( void );



void  Spi4Test( void *p_arg )
{

}


void  Spi5Test( void *p_arg )
{

}

/**************************************************************************************
** ��������: Buffercmp
** ����    : pBuffer1������ָ��1  pBuffer2������ָ��2  BufferLength���Ƚϵĳ���
** ��������: BUFF�ȽϺ���
** ����ֵ  : ����ȫ����һ���ͷ��س���  ��һ���ͷ���0
**************************************************************************************/
uint16_t Buffercmp( uint8_t* pBuffer1, uint8_t* pBuffer2, uint16_t BufferLength )
{
  while( BufferLength-- )
  {
    if( ( *pBuffer1 ) != *pBuffer2 )
    {
      return BufferLength;
    }
    pBuffer1++;
    pBuffer2++;
  }

  return 0;
}

void GpioInit( void )
{
  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __GPIOH_CLK_ENABLE();

  GPIO_InitStruct.Pin   = GPIO_PIN_6;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init( GPIOH, &GPIO_InitStruct );
}

void CrcTest( void )
{
  u8 crcTestBuff[5] = {0x01, 0x02, 0x03, 0x04, 0x05};
  crc result;

  result = crcSlow( crcTestBuff, 5 );

  result = crcFast( crcTestBuff, 5 );

  result = result;
}

