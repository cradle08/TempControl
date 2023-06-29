/**
  ******************************************************************************
  * @file    stm32f4xx_hal_msp.h
  * @author  Firmware-Team
  * @version V1.0.0
  * @date    27-06-2016
  * @brief
  ******************************************************************************
**/

#ifndef  __STM32F4XX_HAL_MSP__
#define  __STM32F4XX_HAL_MSP__


/*
*********************************************************************************************************
*                                              INCLUDE FILES
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*                                               INT DEFINES
*********************************************************************************************************
*/
#define DEBUG_EN							true
#define BSP_USART1_EN					false
#define BSP_USART3_EN					false
#if DEBUG_EN
  #define BSP_USART7_EN				false
#endif

#define SPI1_EN								false
#define SPI2_EN								false
#define SPI4_EN								false
#define SPI5_EN								false

#define TIM1_EN								false
#define TIM3_EN								false
#define EXIT4_EN							false
#define EXIT9_5_EN						false
#define I2C1_EN								false
#define I2C3_EN								false

#define CRC_EN								false
#define ADC_TEST							false
#define GPIO_TEST							false
#define	TEC_TEST							false
#define PRINT_DATA_TEST				false

/*i2c1***************************************************************************/


/*
*********************************************************************************************************
*                                           FUNCTION PROTOTYPES
*********************************************************************************************************
*/
extern void SaveLog( const char *pFormat, ... );
extern void BSP_MspInit( void );
extern void HAL_UART_MspInit( UART_HandleTypeDef* huart );
extern void HAL_UART_MspDeInit( UART_HandleTypeDef* huart );
extern void Debug_Printf( const char *pFormat, ... );

extern void TimxPulseValueSet( TIM_HandleTypeDef *htim, u32 Channel, u32 Value );

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


/**
  * @}
  */

/**
  * @}
  */


#endif

