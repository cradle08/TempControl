/***************************************************************************//**
*   @file    AD7124.h
*   @brief   AD7124 header file.
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

#ifndef __AD7124_H__
#define __AD7124_H__

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include <stdint.h>
#include "AD7124_regs.h"

#define TIM_MS_10            10
#define TIM_MS_25            25

#define TIMER15_ADC_MS       TIM_MS_25

#define AD7124_GAIN_PGA      4

#if TIMER15_ADC_MS==TIM_MS_10
  #define AD7124_FILTER_FS    33   //35
#else
  #define AD7124_FILTER_FS    58   //60
#endif

#define AD7124_WAIT_TIME     0x1000
#define AD7124_CRC           AD7124_DISABLE_CRC


/*
 * The structure describes the device and is used with the ad7124 driver.
 * @slave_select_id: The ID of the Slave Select to be passed to the SPI calls.
 * @regs: A reference to the register list of the device that the user must
 *       provide when calling the Setup() function.
 * @userCRC: Whether to do or not a cyclic redundancy check on SPI transfers.
 * @check_ready: When enabled all register read and write calls will first wait
 *               until the device is ready to accept user requests.
 * @spi_rdy_poll_cnt: Number of times the driver should read the Error register
 *                    to check if the device is ready to accept user requests,
 *                    before a timeout error will be issued.
 */

// BGI
enum tagDiffChn
{
  DIFF_CHN_1    = 0,
  DIFF_CHN_2,
  DIFF_CHN_3,
  DIFF_CHN_4,
  DIFF_CHN_MAX,
};

// channel config register
typedef struct _ad7124_st_chn
{
  int8_t AinP;
  int8_t AinM;
  int8_t Io_CH0;
  int8_t Io_CH1;
} ad7124_st_chn;

#define IsAdOutPinLow()     (GPIO_PIN_RESET==GPIO_HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15))


/******************************************************************************/
/******************* AD7124 Constants *****************************************/
/******************************************************************************/
#define AD7124_CRC8_POLYNOMIAL_REPRESENTATION  0x07 /* x8 + x2 + x + 1 */
#define AD7124_DISABLE_CRC                     0
#define AD7124_USE_CRC                         1

/******************************************************************************/
/************************ Functions Declarations ******************************/
/******************************************************************************/
/*! Reads the value of the specified register. */
int32_t AD7124_ReadRegister( uint8_t RegIdx, uint32_t *RegValue );

/*! Writes the value of the specified register. */
int32_t AD7124_WriteRegister( uint8_t RegIdx, uint32_t RegValue );

/*! Reads the value of the specified register without a device state check. */
int32_t AD7124_NoCheckReadRegister( uint8_t RegIdx, uint32_t *RegValue );

/*! Writes the value of the specified register without a device state check. */
int32_t AD7124_NoCheckWriteRegister( uint8_t RegIdx, uint32_t RegValue );

/*! Resets the device. */
int32_t AD7124_Reset( void );

/*! Waits until the device can accept read and write user actions. */
int32_t AD7124_WaitForSpiReady( void );

/*! Waits until a new conversion result is available. */
int32_t AD7124_WaitForConvReady( uint8_t RegIdx, uint32_t *RegValue );

/*! Updates the device SPI interface settings. */
int32_t AD7124_UpdateDevSpiSettings( void );

/*! Initializes the AD7124. */
int32_t AD7124_Setup( uint8_t channel );

int32_t ConfigAD7124( void );
int32_t AD7124_GetAdcValue( void );
int32_t AD7124_Convert( uint8_t chn );
int32_t AD7124_AdcDataReady( void );

#endif /* __AD7124_H__ */

