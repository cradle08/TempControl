/***************************************************************************//**
*   @file    AD7124.c
*   @brief   AD7124 implementation file.
*   @devices AD7124-4, AD7124-8
*
********************************************************************************
* Copyright 2015(c) Analog Devices, Inc.
*
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without modification,
* are permitted provided that the following conditions are met:
*  - Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
*  - Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in
*    the documentation and/or other materials provided with the
*    distribution.
*  - Neither the name of Analog Devices, Inc. nor the names of its
*    contributors may be used to endorse or promote products derived
*    from this software without specific prior written permission.
*  - The use of this software may or may not infringe the patent rights
*    of one or more patent holders.  This license does not release you
*    from the requirement that you obtain separate licenses from these
*    patent holders to use this software.
*  - Use of the software either in source or binary form, must be run
*    on or directly connected to an Analog Devices Inc. component.
*
* THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES "AS IS" AND ANY EXPRESS OR IMPLIED
* WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT, MERCHANTABILITY
* AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
* IN NO EVENT SHALL ANALOG DEVICES BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
* SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
* INTELLECTUAL PROPERTY RIGHTS, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
* ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/

//#include "Communication.h"
#include "AD7124.h"
#include "includes.h"
#include "stm32f3xx_hal_spi.h"
#include "AD7124_regs.h"


/* Error codes */
#define INVALID_VAL -1         /* Invalid argument */
#define COMM_ERR    -2         /* Communication error on receive */
#define TIMEOUT     -3         /* A timeout has occured */
#define AD7124_NO   -4         /* A timeout has occured */


const ad7124_st_reg ad7124_regs[] =
{
  {0x00, 0x00,   1, 2}, /* AD7124_Status */
  {0x01, 0x0000, 2, 1}, /* AD7124_ADC_Control */
  {0x02, 0x0000, 3, 2}, /* AD7124_Data */
  {0x03, 0x0000, 3, 1}, /* AD7124_IOCon1 */
  {0x04, 0x0000, 2, 1}, /* AD7124_IOCon2 */
  {0x05, 0x02,   1, 2}, /* AD7124_ID */
  {0x06, 0x0000, 3, 2}, /* AD7124_Error */
  {0x07, 0x0044, 3, 1}, /* AD7124_Error_En */
  {0x08, 0x00,   1, 2}, /* AD7124_Mclk_Count */
  {0x09, 0x8001, 2, 1}, /* AD7124_Channel_0 */
  {0x0A, 0x0001, 2, 1}, /* AD7124_Channel_1 */
  {0x0B, 0x0001, 2, 1}, /* AD7124_Channel_2 */
  {0x0C, 0x0001, 2, 1}, /* AD7124_Channel_3 */
  {0x0D, 0x0001, 2, 1}, /* AD7124_Channel_4 */
  {0x0E, 0x0001, 2, 1}, /* AD7124_Channel_5 */
  {0x0F, 0x0001, 2, 1}, /* AD7124_Channel_6 */
  {0x10, 0x0001, 2, 1}, /* AD7124_Channel_7 */
  {0x11, 0x0001, 2, 1}, /* AD7124_Channel_8 */
  {0x12, 0x0001, 2, 1}, /* AD7124_Channel_9 */
  {0x13, 0x0001, 2, 1}, /* AD7124_Channel_10 */
  {0x14, 0x0001, 2, 1}, /* AD7124_Channel_11 */
  {0x15, 0x0001, 2, 1}, /* AD7124_Channel_12 */
  {0x16, 0x0001, 2, 1}, /* AD7124_Channel_13 */
  {0x17, 0x0001, 2, 1}, /* AD7124_Channel_14 */
  {0x18, 0x0001, 2, 1}, /* AD7124_Channel_15 */
  {0x19, 0x0860, 2, 1}, /* AD7124_Config_0 */
  {0x1A, 0x0860, 2, 1}, /* AD7124_Config_1 */
  {0x1B, 0x0860, 2, 1}, /* AD7124_Config_2 */
  {0x1C, 0x0860, 2, 1}, /* AD7124_Config_3 */
  {0x1D, 0x0860, 2, 1}, /* AD7124_Config_4 */
  {0x1E, 0x0860, 2, 1}, /* AD7124_Config_5 */
  {0x1F, 0x0860, 2, 1}, /* AD7124_Config_6 */
  {0x20, 0x0860, 2, 1}, /* AD7124_Config_7 */
  {0x21, 0x060180, 3, 1}, /* AD7124_Filter_0 */
  {0x22, 0x060180, 3, 1}, /* AD7124_Filter_1 */
  {0x23, 0x060180, 3, 1}, /* AD7124_Filter_2 */
  {0x24, 0x060180, 3, 1}, /* AD7124_Filter_3 */
  {0x25, 0x060180, 3, 1}, /* AD7124_Filter_4 */
  {0x26, 0x060180, 3, 1}, /* AD7124_Filter_5 */
  {0x27, 0x060180, 3, 1}, /* AD7124_Filter_6 */
  {0x28, 0x060180, 3, 1}, /* AD7124_Filter_7 */
  {0x29, 0x800000, 3, 1}, /* AD7124_Offset_0 */
  {0x2A, 0x800000, 3, 1}, /* AD7124_Offset_1 */
  {0x2B, 0x800000, 3, 1}, /* AD7124_Offset_2 */
  {0x2C, 0x800000, 3, 1}, /* AD7124_Offset_3 */
  {0x2D, 0x800000, 3, 1}, /* AD7124_Offset_4 */
  {0x2E, 0x800000, 3, 1}, /* AD7124_Offset_5 */
  {0x2F, 0x800000, 3, 1}, /* AD7124_Offset_6 */
  {0x30, 0x800000, 3, 1}, /* AD7124_Offset_7 */
  {0x31, 0x500000, 3, 1}, /* AD7124_Gain_0 */
  {0x32, 0x500000, 3, 1}, /* AD7124_Gain_1 */
  {0x33, 0x500000, 3, 1}, /* AD7124_Gain_2 */
  {0x34, 0x500000, 3, 1}, /* AD7124_Gain_3 */
  {0x35, 0x500000, 3, 1}, /* AD7124_Gain_4 */
  {0x36, 0x500000, 3, 1}, /* AD7124_Gain_5 */
  {0x37, 0x500000, 3, 1}, /* AD7124_Gain_6 */
  {0x38, 0x500000, 3, 1}, /* AD7124_Gain_7 */
};

ad7124_st_chn stAd7124_chn[4];
//#ifdef AD_13
//const ad7124_st_chn stAd7124_chn[] =
//{
//	{1,  2,  0,  3},           // diff channel 1
//	{5,  6,  4,  7},           // diff channel 2
//	{9, 10,  8, 11},           // diff channel 3
//	{14,13, 12, 15},           // diff channel 4
//};
//#else
//const ad7124_st_chn stAd7124_chn[] =
//{
//	{1,  2,  0,  3},           // diff channel 1
//	{5,  6,  4,  7},           // diff channel 2
//	{9, 10,  8, 11},           // diff channel 3
//	{13,14, 12, 15},           // diff channel 4
//};
////#endif
static u32  AdcGain0Default = 0x0;

extern void SPI2_CS_LOW( void );
extern void SPI2_CS_HIGH( void );

s32 GetADControlConfig( ad7124_st_chn *stAd7124_chn );

/***************************************************************************//**
* @brief Reads the value of the specified register without checking if the
*        device is ready to accept user requests.
*
* @param device - The handler of the instance of the driver.
* @param pReg - Pointer to the register structure holding info about the
*               register to be read. The read value is stored inside the
*               register structure.
*
* @return Returns 0 for success or negative error code.
*******************************************************************************/
int32_t AD7124_NoCheckReadRegister( uint8_t RegIdx, uint32_t *RegValue )
{
  int32_t ret       = 0;
  uint8_t buffer[8] = {0, 0, 0, 0, 0, 0, 0, 0};
  uint8_t i         = 0;
  //	uint8_t check8    = 0;
  uint8_t msgBuf[8] = {0, 0, 0, 0, 0, 0, 0, 0};
  uint8_t bufsize;
  uint32_t tickStart;
  if( RegIdx>=AD7124_REG_NO )
    return INVALID_VAL;

  bufsize = ad7124_regs[RegIdx].size;

  /* Build the Command word */
  buffer[0] = AD7124_COMM_REG_WEN | AD7124_COMM_REG_RD | AD7124_COMM_REG_RA( ad7124_regs[RegIdx].addr );

  SPI2_CS_LOW();
  ret = BspSpiTransmitReceiveIT( &BspSpi2Handle, ( u8* )buffer, ( u8 * )msgBuf, ad7124_regs[RegIdx].size + 1 );	//edit 20171019

#if 1
  if( ret != BSP_OK )
  {
    return ret;
  }
  else
  {
    // 不处理
  }

  tickStart = HAL_GetTick();
  while( BspSpi2Handle.SpiFlag.RxCpltFlag != true )
  {
    if( HAL_GetTick() - tickStart > AD7124_WAIT_TIME )
    {
      return BSP_HAL_BUSY;
    }
  }

#endif

  SPI2_CS_HIGH();
  // Build the result
  *RegValue = 0;
  for( i = 1; i <bufsize  + 1; i++ )
  {
    *RegValue <<= 8;
    *RegValue += msgBuf[i];
  }

  return ret;
}

/***************************************************************************//**
* @brief Writes the value of the specified register without checking if the
*        device is ready to accept user requests.
*
* @param device - The handler of the instance of the driver.
* @param reg - Register structure holding info about the register to be written
*
* @return Returns 0 for success or negative error code.
*******************************************************************************/
int32_t AD7124_NoCheckWriteRegister( uint8_t RegIdx, uint32_t RegValue )
{
  int32_t ret      = 0;
  int32_t regValue = 0;
  uint8_t wrBuf[8] = {0, 0, 0, 0, 0, 0, 0, 0};
  uint8_t rdBuf[8] = {0, 0, 0, 0, 0, 0, 0, 0};
  uint8_t i        = 0;
  //	uint8_t crc8     = 0;
  uint8_t bufsize;
  u32 tickStart;

  if( RegIdx>=AD7124_REG_NO )
    return INVALID_VAL;

  bufsize = ad7124_regs[RegIdx].size;

  /* Build the Command word */
  wrBuf[0] = AD7124_COMM_REG_WEN | AD7124_COMM_REG_WR | AD7124_COMM_REG_RA( ad7124_regs[RegIdx].addr );

  /* Fill the write buffer */
  regValue = RegValue;
  for( i = 0; i < bufsize; i++ )
  {
    wrBuf[bufsize - i] = regValue & 0xFF;
    regValue >>= 8;
  }

  // Write data to the device
  SPI2_CS_LOW();

  ret = BspSpiTransmitReceiveIT( &BspSpi2Handle, ( u8* )wrBuf, ( u8 * )rdBuf, ad7124_regs[RegIdx].size + 1 ); //edit 20171019
#if 1
  if( ret != BSP_OK )
  {
    return ret;
  }
  else
  {
    // 不处理
  }

  tickStart = HAL_GetTick();
  while( BspSpi2Handle.SpiFlag.TxCpltFlag != true )
  {
    if( HAL_GetTick() - tickStart > AD7124_WAIT_TIME )
    {
      return BSP_HAL_BUSY;
    }
  }

#endif

  SPI2_CS_HIGH();

  return ret;
}

/***************************************************************************//**
* @brief Reads the value of the specified register only when the device is ready
*        to accept user requests. If the device ready flag is deactivated the
*        read operation will be executed without checking the device state.
*
* @param device - The handler of the instance of the driver.
* @param pReg - Pointer to the register structure holding info about the
*               register to be read. The read value is stored inside the
*               register structure.
*
* @return Returns 0 for success or negative error code.
*******************************************************************************/
int32_t AD7124_ReadRegister( uint8_t RegIdx, uint32_t *RegValue )
{
  int32_t ret;

  if( RegIdx>=AD7124_REG_NO )
    return INVALID_VAL;

  //	if (ad7124_regs[RegIdx].addr != AD7124_ERR_REG && device->check_ready)
  if( ( ad7124_regs[RegIdx].addr != AD7124_ERR_REG )&&0 )
  {
    ret = AD7124_WaitForSpiReady( );
    if( ret < 0 )
      return ret;
  }

  ret = AD7124_NoCheckReadRegister( RegIdx, RegValue );

  return ret;
}

/***************************************************************************//**
* @brief Writes the value of the specified register only when the device is
*        ready to accept user requests. If the device ready flag is deactivated
*        the write operation will be executed without checking the device state.
*
* @param device - The handler of the instance of the driver.
* @param reg - Register structure holding info about the register to be written
*
* @return Returns 0 for success or negative error code.
*******************************************************************************/
int32_t AD7124_WriteRegister( uint8_t RegIdx, uint32_t RegValue )
{
  int32_t  ret;

  if( RegIdx>=AD7124_REG_NO )
    return INVALID_VAL;

  if( 0 )
  {
    ret = AD7124_WaitForSpiReady( );
    if( ret < 0 )
      return ret;
  }

  ret = AD7124_NoCheckWriteRegister( RegIdx, RegValue );

  return ret;
}

/***************************************************************************//**
* @brief Resets the device.
*
* @param device - The handler of the instance of the driver.
*
* @return Returns 0 for success or negative error code.
*******************************************************************************/
int32_t AD7124_Reset( void )
{
  int32_t ret = 0;
  uint8_t wrBuf[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  uint8_t rdBuf[8];
  u32 tickStart;

  SPI2_CS_LOW();

  ret = BspSpiTransmitReceiveIT( &BspSpi2Handle, ( u8* )wrBuf, ( u8 * )rdBuf, 8 );
#if 1
  if( ret != BSP_OK )
  {
    return ret;
  }
  else
  {
    // 不处理
  }
  tickStart = HAL_GetTick();
  while( BspSpi2Handle.SpiFlag.TxCpltFlag != true )
  {
    if( HAL_GetTick() - tickStart > AD7124_WAIT_TIME )
    {
      return BSP_HAL_BUSY;
    }
  }

#endif
  SPI2_CS_HIGH();

  return ret;
}

/***************************************************************************//**
* @brief Waits until the device can accept read and write user actions.
*
* @param device - The handler of the instance of the driver.
* @param timeout - Count representing the number of polls to be done until the
*                  function returns.
*
* @return Returns 0 for success or negative error code.
*******************************************************************************/
int32_t AD7124_WaitForSpiReady( void )
{

  int32_t  ret;
  int8_t   ready = 0;
  uint32_t timeout = AD7124_WAIT_TIME;
  uint32_t regvalue;

  while( !ready && --timeout )
  {
    /* Read the value of the Error Register */
    ret = AD7124_NoCheckReadRegister( AD7124_Error, &regvalue );
    if( ret < 0 )
      return ret;

    /* Check the SPI IGNORE Error bit in the Error Register */
    ready = ( ( regvalue & AD7124_ERR_REG_SPI_IGNORE_ERR ) == 0 )?1:0;
  }

  return timeout ? 0 : TIMEOUT;
}

/***************************************************************************//**
* @brief Waits until a new conversion result is available.
*
* @param device - The handler of the instance of the driver.
* @param timeout - Count representing the number of polls to be done until the
*                  function returns if no new data is available.
*
* @return Returns 0 for success or negative error code.
*******************************************************************************/
int32_t AD7124_WaitForConvReady( uint8_t RegIdx, uint32_t *RegValue )
{
  int32_t ret;
  int8_t ready = 0;

  uint32_t timeout = AD7124_WAIT_TIME;

  if( RegIdx>=AD7124_REG_NO )
    return INVALID_VAL;


  while( !ready && --timeout )
  {
    /* Read the value of the Status Register */
    ret = AD7124_NoCheckReadRegister( AD7124_Status, RegValue );
    if( ret < 0 )
      return ret;

    /* Check the RDY bit in the Status Register */
    ready = ( *RegValue & AD7124_STATUS_REG_RDY ) == 0;
  }

  return timeout ? 0 : TIMEOUT;
}



/***************************************************************************//**
* @brief Updates the device SPI interface settings.
*
* @param device - The handler of the instance of the driver.
*
* @return None.
*******************************************************************************/
int32_t AD7124_UpdateDevSpiSettings( void )
{
  int32_t  ret;
  uint32_t RegValue;

  ret = AD7124_NoCheckReadRegister( AD7124_Error_En, &RegValue );
  if( ret < 0 )
    return ret;

  if( RegValue & AD7124_ERREN_REG_SPI_IGNORE_ERR_EN )
  {
    ret = 0;
  }
  else
  {
    ret = AD7124_NO;
  }

  return ret;
}

/*********************************************************************************
 *Function Name: AD7124_Setup
 *  Description: AD7124寄存器设置指令
 *  Arguments
         Inputs: device, AD7124操作结构体
               : chn   , 相应的差分通道
        Outputs:
 *  Note(s)    :
**********************************************************************************/
void AD7124_CommandDelayTime( void )
{
  uint32_t systick;

  systick = HAL_GetTick();

  while( ( HAL_GetTick()-systick )<1 )  ;
}

/*********************************************************************************
 *Function Name: AD7124_Setup
 *  Description: AD7124寄存器设置指令
 *  Arguments
         Inputs: device, AD7124操作结构体
               : chn   , 相应的差分通道
        Outputs:
 *  Note(s)    :
**********************************************************************************/

int32_t AD7124_Setup( u8 chn )
{
  int32_t  ret;
  uint32_t RegValue;
  //	uint32_t systick;


  if( chn>=DIFF_CHN_MAX )
    return INVALID_VAL;

  // Register init

  RegValue = ( AD7124_CH_MAP_REG_CH_ENABLE | AD7124_CH_MAP_REG_SETUP( chn ) |         \
               AD7124_CH_MAP_REG_AINP( stAd7124_chn[chn].AinP ) |                     \
               AD7124_CH_MAP_REG_AINM( stAd7124_chn[chn].AinM ) );
  ret = AD7124_WriteRegister( AD7124_CH0_MAP_REG+chn, RegValue );

  AD7124_CommandDelayTime();


  RegValue = AD7124_CFG_REG_BIPOLAR    | AD7124_CFG_REG_BURNOUT( 0 ) |               \
             AD7124_CFG_REG_REF_BUFP   | AD7124_CFG_REG_REF_BUFM   |               \
             AD7124_CFG_REG_AIN_BUFP   | AD7124_CFG_REG_AINN_BUFM  |               \
             AD7124_CFG_REG_REF_SEL( 0 ) | AD7124_CFG_REG_PGA( AD7124_GAIN_PGA );
  ret = AD7124_WriteRegister( AD7124_CFG0_REG+chn, RegValue );

  AD7124_CommandDelayTime();

  //	RegValue = AD7124_FILT_REG_FILTER(0) | AD7124_FILT_REG_POST_FILTER(0)| AD7124_FILT_REG_FS(AD7124_FILTER_FS);
  RegValue = AD7124_FILT_REG_FILTER( 0 ) | AD7124_FILT_REG_POST_FILTER( 3 )| AD7124_FILT_REG_FS( AD7124_FILTER_FS );
  ret = AD7124_WriteRegister( AD7124_FILT0_REG+chn, RegValue );

  //	AD7124_CommandDelayTime();
  //
  //	RegValue = ad7124_regs[AD7124_OFFS0_REG].value;
  //    ret = AD7124_WriteRegister(AD7124_OFFS0_REG+chn, RegValue);
  //
  //	AD7124_CommandDelayTime();
  //
  //	RegValue = ad7124_regs[AD7124_GAIN0_REG].value;
  //	ret = AD7124_WriteRegister(AD7124_GAIN0_REG+chn, RegValue);

  return ret;
}

/*********************************************************************************
 *Function Name: AD7124_Convert
 *  Description: AD7124转换指令
 *  Arguments
         Inputs: 无
        Outputs:
 *  Note(s)    :
**********************************************************************************/
int32_t AD7124_Convert( u8 chn )
{
  s32      ret;
  uint32_t RegValue;


  if( chn>=DIFF_CHN_MAX )
    return -1;

  // channel pins setup
  RegValue = ( AD7124_CH_MAP_REG_CH_ENABLE | AD7124_CH_MAP_REG_SETUP( 0 ) |                         \
               AD7124_CH_MAP_REG_AINP( stAd7124_chn[chn].AinP ) | AD7124_CH_MAP_REG_AINM( stAd7124_chn[chn].AinM ) );
  ret = AD7124_WriteRegister( AD7124_CH0_MAP_REG, RegValue );

  if( ret<0 )
    return ret;


  RegValue = AD7124_IO_CTRL1_REG_IOUT_CH0( stAd7124_chn[chn].Io_CH0 ) | AD7124_IO_CTRL1_REG_IOUT0( 4 )|
             AD7124_IO_CTRL1_REG_IOUT_CH1( stAd7124_chn[chn].Io_CH1 ) | AD7124_IO_CTRL1_REG_IOUT1( 4 );
  ret = AD7124_WriteRegister( AD7124_IO_CTRL1_REG, RegValue );

  if( ret<0 )
    return ret;

  RegValue = AD7124_ADC_CTRL_REG_CS_EN | AD7124_ADC_CTRL_REG_DATA_STATUS|           \
             /* 		       AD7124_ADC_CTRL_REG_DOUT_RDY_DEL | AD7124_ADC_CTRL_REG_MODE(1)|        \     */
             //			   AD7124_ADC_CTRL_REG_CONT_READ    |
             AD7124_ADC_CTRL_REG_DOUT_RDY_DEL | AD7124_ADC_CTRL_REG_MODE( 1 )|        \
             AD7124_ADC_CTRL_REG_POWER_MODE( 3 )| AD7124_ADC_CTRL_REG_CLK_SEL( 0 );
  ret = AD7124_WriteRegister( AD7124_ADC_CTRL_REG, RegValue );
  if( ret<0 )
    return ret;

  return 0;
}

/*********************************************************************************
 *Function Name: AD7124_GetAdcValue
 *  Description: 读取AD7124数据寄存器
 *  Arguments
         Inputs: 无
        Outputs: 1为转换完成, 0则未完成转换
 *  Note(s)    :
**********************************************************************************/
u32 gGain;
int32_t AD7124_GetAdcValue( void )
{
  uint32_t RegValue;

  //	if( GPIO_PIN_SET==HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14) )
  //		return INVALID_VAL;

  /*	if( 0!=AD7124_ReadRegister(AD7124_Status, &RegValue) )
  		return INVALID_VAL;

  	// Check the RDY bit in the Status Register
  	if( (AD7124_STATUS_REG_RDY==(RegValue & AD7124_STATUS_REG_RDY))||
  		(AD7124_STATUS_REG_ERROR_FLAG!=(RegValue & AD7124_STATUS_REG_ERROR_FLAG)) )
  	{
  		return INVALID_VAL;
  	}
  */
  if( 0 != AD7124_ReadRegister( AD7124_GAIN0_REG, &RegValue ) )
    return INVALID_VAL;

  if( ( AdcGain0Default >= 0x00500000 ) && ( AdcGain0Default != RegValue ) )
  {
    AD7124_WriteRegister( AD7124_GAIN0_REG, AdcGain0Default );

    gGain = RegValue;

    return INVALID_VAL;
  }
  RegValue = 0;

  if( 0 != AD7124_ReadRegister( AD7124_DATA_REG, &RegValue ) )
    return INVALID_VAL;

  return ( int32_t )RegValue;
}

/*********************************************************************************
 *Function Name: AD7124_AdcDataReady
 *  Description: 查询AD7124状态寄存器
 *  Arguments
         Inputs: 无
        Outputs: 1为转换完成, 0则未完成转换
 *  Note(s)    :
**********************************************************************************/
int32_t AD7124_AdcDataReady( void )
{
  int32_t  ret;
  uint32_t RegValue;

  ret = AD7124_NoCheckReadRegister( AD7124_Status, &RegValue );
  if( ret < 0 )
    return ret;

  if( ( RegValue & AD7124_STATUS_REG_RDY ) == 0 )
    return 1;
  else
    return 0;
}

/*********************************************************************************
 *Function Name: ConfigAD7124
 *  Description: 配置AD7124
 *  Arguments
         Inputs: 无
        Outputs: 无
 *  Note(s)    : 外部接口
**********************************************************************************/
int32_t ConfigAD7124( void )
{
  int32_t ret;
  u32 systick;

  //  Reset the device interface.
  ret = AD7124_Reset();
  if( ret < 0 )
    return ret;

  systick = HAL_GetTick( );
  while( ( HAL_GetTick( )-systick )<10 )
    ;

  GetADControlConfig( stAd7124_chn );
  ret = AD7124_Setup( DIFF_CHN_1 );
  ret = AD7124_Convert( DIFF_CHN_1 );

  return ret;
}


/*********************************************************************************
 *Function Name: AD7124_DisplayRegSettings
 *  Description: 读取寄存器设置参数
 *  Arguments
         Inputs: len,     数据个数
               : RegValue,数据缓存
        Outputs:
 *  Note(s)    :
**********************************************************************************/
void TEMP_GetAd7124Settings( uint8_t len, uint32_t *RegSet )
{
  uint32_t regvalue;
  uint8_t  idx;

  AD7124_ReadRegister( AD7124_ID_REG, &regvalue );
  Debug_Printf( "[ID,%08x]", regvalue );

  idx = 0;

  if( idx<len )
    RegSet[idx] = regvalue;
  idx += 1;

  AD7124_ReadRegister( AD7124_IO_CTRL1_REG, &regvalue );
  Debug_Printf( "[IO_CTRL1,%08x]", regvalue );

  if( idx<len )
    RegSet[idx] = regvalue;
  idx += 1;

  AD7124_ReadRegister( AD7124_CH0_MAP_REG, &regvalue );
  Debug_Printf( "[CH0,%08x]",	regvalue );

  if( idx<len )
    RegSet[idx] = regvalue;
  idx += 1;

  AD7124_ReadRegister( AD7124_CFG0_REG, &regvalue );
  Debug_Printf( "[CFG0,%08x]",	regvalue );

  if( idx<len )
    RegSet[idx] = regvalue;
  idx += 1;

  AD7124_ReadRegister( AD7124_FILT0_REG, &regvalue );
  Debug_Printf( "[FILT0,%08x]", regvalue );

  if( idx<len )
    RegSet[idx] = regvalue;
  idx += 1;

  AD7124_ReadRegister( AD7124_OFFS0_REG, &regvalue );
  Debug_Printf( "[OFFS0,%08x]", regvalue );

  if( idx<len )
    RegSet[idx] = regvalue;
  idx += 1;

  AD7124_ReadRegister( AD7124_GAIN0_REG, &regvalue );
  Debug_Printf( "[GAIN0,%08x]", regvalue );

  // 存储GAIN0的默认值
  AdcGain0Default = regvalue;

  gGain = AdcGain0Default;

  if( idx<len )
    RegSet[idx] = regvalue;
  idx += 1;

  Debug_Printf( "\r\n" );
}


s32 GetADControlConfig( ad7124_st_chn *stAd7124_chn )
{
  uint32_t valueOfID;
  uint8_t i = 0;

  AD7124_ReadRegister( AD7124_ID_REG, &valueOfID );
  Debug_Printf( "[ID,%08x]", valueOfID );

  if( valueOfID >= 0x14 )
  {

    for( i = 0; i<4; i++ )
    {
      stAd7124_chn[i].AinP = ( 1+4*i );
      stAd7124_chn[i].AinM = ( 2+4*i );
      stAd7124_chn[i].Io_CH0 = ( 4*i );
      stAd7124_chn[i].Io_CH1 = ( 3+4*i );
    }
  }
  else
  {
    for( i = 0; i<4; i++ )
    {
      stAd7124_chn[i].AinP = ( 1+4*i );
      stAd7124_chn[i].AinM = ( 2+4*i );
      stAd7124_chn[i].Io_CH0 = ( 4*i );
      stAd7124_chn[i].Io_CH1 = ( 3+4*i );
    }
    stAd7124_chn[3].AinP = 14;
    stAd7124_chn[3].AinM = 13;
  }

  return true;
}

uint32_t AD7124_ReadRegOut( unsigned char reg )
{
  uint32_t regvalue;
  AD7124_ReadRegister( ( uint8_t )reg, &regvalue );

  return  regvalue;
}


