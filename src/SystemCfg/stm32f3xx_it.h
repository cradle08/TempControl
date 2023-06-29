/**
  ******************************************************************************
  * @file    stm32f3xx_it.h
  * @author  Firmware-Team
  * @version V1.0.0
  * @date    13-06-2016
  * @brief
  ******************************************************************************
**/
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F3xx_IT_H
#define __STM32F3xx_IT_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

extern void SysTick_Handler( void );
extern void USART1_IRQHandler( void );
extern void UART7_IRQHandler( void );
extern void DMA1_Stream1_IRQHandler( void );
extern void DMA1_Stream3_IRQHandler( void );
extern void SPI4_IRQHandler( void );
extern void EXTI4_IRQHandler( void );
extern void EXTI9_5_IRQHandler( void );
extern void I2C1_EV_IRQHandler( void );
extern void I2C1_ER_IRQHandler( void );
extern void SPI1_IRQHandler( void );
extern void USART2_IRQHandler( void );

#ifdef __cplusplus
}
#endif

#endif /* __STM32F4xx_IT_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
