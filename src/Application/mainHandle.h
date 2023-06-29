/**
  ******************************************************************************
  * @file    task.h
  * @author  Firmware-Team
  * @version V1.0.0
  * @date    13-06-2016
  * @brief
  ******************************************************************************
**/

#ifndef  __TASK__
#define  __TASK__

/*
*********************************************************************************************************
*                                              INCLUDE FILES
*********************************************************************************************************
*/
#include  <uccpu.h>
#include "stm32f3xx_hal.h"

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
extern void  MainHandleTask( void *p_arg );
extern uint16_t Buffercmp( uint8_t* pBuffer1, uint8_t* pBuffer2, uint16_t BufferLength );
extern void I2c1Test( void );
extern void I2c3Test( void );
//extern void  Spi2Test(void *p_arg);
extern void  Spi4Test( void *p_arg );
extern void  Spi5Test( void *p_arg );
extern void PwmInit( void );
extern void PwmTest( void );
extern void GpioInit( void );
extern void CrcTest( void );
/*
*********************************************************************************************************
*                                        LOCAL GLOBAL VARIABLES
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                             MODULE END
*********************************************************************************************************
*/




#endif /* End */

