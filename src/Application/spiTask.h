/**
  ******************************************************************************
  * @file    spiTask.h
  * @author  Firmware-Team
  * @version V1.0.0
  * @date    13-06-2016
  * @brief
  ******************************************************************************
**/

#ifndef  __SPI_TASK__
#define  __SPI_TASK__

/*
*********************************************************************************************************
*                                              INCLUDE FILES
*********************************************************************************************************
*/
#include  <uccpu.h>
#include "stm32f3xx_hal.h"
#include "bsp_msp_port.h"
/*
*********************************************************************************************************
*                                               INT DEFINES
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*                                           FUNCTION PROTOTYPES
*********************************************************************************************************
*/
extern BspSpiHandle BspSpi1Handle, BspSpi2Handle, BspSpi4Handle, BspSpi5Handle;

/*
*********************************************************************************************************
*                                        LOCAL GLOBAL VARIABLES
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*                                         FUNCTION PROTOTYPES
*********************************************************************************************************
*/
extern void Spi2Config( void );
extern void SPI2_CS_LOW( void );
extern void SPI2_CS_HIGH( void );
extern void ConfigSPI1( void );

extern void  TestForSpi1( void *p_arg );
extern void  Spi2Test( void *p_arg );
/*
*********************************************************************************************************
*                                             MODULE END
*********************************************************************************************************
*/




#endif /* End */
