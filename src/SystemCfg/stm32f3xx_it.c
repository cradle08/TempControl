/**
  ******************************************************************************
  * @file    stm32f3xx_it.c
  * @author  Firmware-Team
  * @version V1.0.0
  * @date    13-06-2016
  * @brief
  ******************************************************************************
**/
/* Includes ------------------------------------------------------------------*/
#include "includes.h"


/* External variables --------------------------------------------------------*/





/******************************************************************************/
/*            Cortex-M4 Processor Interruption and Exception Handlers         */
/******************************************************************************/







/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

void USART1_IRQHandler( void )
{
  HAL_UART_IRQHandler( &BspUart1Handle.UartHandleInit );

}


void USART3_IRQHandler( void )
{
  HAL_UART_IRQHandler( &BspUart3Handle.UartHandleInit );
}

void USART6_IRQHandler( void )
{
  HAL_UART_IRQHandler( &BspUart6Handle.UartHandleInit );
}


void UART7_IRQHandler( void )
{
  HAL_UART_IRQHandler( &BspUart7Handle.UartHandleInit );
}

void DMA1_Stream3_IRQHandler( void )
{
  ;
}

/**
  * @brief  This function handles DMA TX interrupt request.
  * @param  None
  * @retval None
  */
void DMA1_Stream1_IRQHandler( void )
{
  ;
}

void SPI1_IRQHandler( void )
{
  HAL_SPI_IRQHandler( &BspSpi1Handle.SpiHandleInit );
}

void SPI2_IRQHandler( void )
{
  HAL_SPI_IRQHandler( &BspSpi2Handle.SpiHandleInit );
}

void SPI4_IRQHandler( void )
{
  HAL_SPI_IRQHandler( &BspSpi4Handle.SpiHandleInit );
}

void SPI5_IRQHandler( void )
{
  HAL_SPI_IRQHandler( &BspSpi5Handle.SpiHandleInit );
}

void EXTI4_IRQHandler( void )
{
  HAL_GPIO_EXTI_IRQHandler( GPIO_PIN_4 );
}

void EXTI9_5_IRQHandler( void )
{
  HAL_GPIO_EXTI_IRQHandler( GPIO_PIN_6 );
}

/**
  * @brief  This function handles I2C event interrupt request.
  * @param  None
  * @retval None
  * @Note   This function is redefined in "main.h" and related to I2C data transmission
  */
void I2C1_EV_IRQHandler( void )
{
  HAL_I2C_EV_IRQHandler( &I2c1Handle.I2cHandleInit );
}

/**
  * @brief  This function handles I2C error interrupt request.
  * @param  None
  * @retval None
  * @Note   This function is redefined in "main.h" and related to I2C error
  */
void I2C1_ER_IRQHandler( void )
{
  HAL_I2C_ER_IRQHandler( &I2c1Handle.I2cHandleInit );
}

/**
  * @brief  This function handles I2C event interrupt request.
  * @param  None
  * @retval None
  * @Note   This function is redefined in "main.h" and related to I2C data transmission
  */
void I2C3_EV_IRQHandler( void )
{
  HAL_I2C_EV_IRQHandler( &I2c3Handle.I2cHandleInit );
}

/**
  * @brief  This function handles I2C error interrupt request.
  * @param  None
  * @retval None
  * @Note   This function is redefined in "main.h" and related to I2C error
  */
void I2C3_ER_IRQHandler( void )
{
  HAL_I2C_ER_IRQHandler( & I2c3Handle.I2cHandleInit );
}

void TIM2_IRQHandler( void )
{
  HAL_TIM_IRQHandler( &Tim2Handle.TimHandleInit );
}

void TIM3_IRQHandler( void )
{
  HAL_TIM_IRQHandler( &Tim3Handle.TimHandleInit );
}

//void TIM4_IRQHandler(void)
//{
//	HAL_TIM_IRQHandler(&Tim4Handle.TimHandleInit);
//}

void TIM5_IRQHandler( void )
{
  HAL_TIM_IRQHandler( &Tim5Handle.TimHandleInit );
}

void TIM6_IRQHandler( void )
{
  HAL_TIM_IRQHandler( &Tim6Handle.TimHandleInit );
}

//void TIM1_BRK_TIM15_IRQHandler(void)
void TIM15_IRQHandler( void )
{
  HAL_TIM_IRQHandler( &Tim15Handle.TimHandleInit );
}

void SysTick_Handler()
{
  HAL_IncTick();
}





/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
