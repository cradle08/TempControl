/**
  ******************************************************************************
  * @file    bsp_common.h
  * @author  Firmware-Team
  * @version V1.0.0
  * @date    13-06-2016
  * @brief   
  ******************************************************************************
**/

#ifndef  __BSP_COMMON__
#define  __BSP_COMMON__

/*
*********************************************************************************************************
*                                              INCLUDE FILES
*********************************************************************************************************
*/
#include "stm32f3xx_hal.h"

/*
*********************************************************************************************************
*                                                 MACRO'S
*********************************************************************************************************
*/
#define  BSP_GPIOG_LED_ALL                     0
#define  BSP_GPIOG_LED_RED                     1
#define  BSP_GPIOI_LED_YELLOW                  2


/*
*********************************************************************************************************
*                                               INT DEFINES
*********************************************************************************************************
*/
#define  BSP_INT_ID_WWDG                                   0u   
#define  BSP_INT_ID_PVD                                    1u  
#define  BSP_INT_ID_TAMP_STAMP                             2u   
#define  BSP_INT_ID_RTC_WKUP                               3u  
#define  BSP_INT_ID_FLASH                                  4u   
#define  BSP_INT_ID_RCC                                    5u  
#define  BSP_INT_ID_EXTI0                                  6u  
#define  BSP_INT_ID_EXTI1                                  7u   
#define  BSP_INT_ID_EXTI2                                  8u 
#define  BSP_INT_ID_EXTI3                                  9u  
#define  BSP_INT_ID_EXTI4                                 10u  
#define  BSP_INT_ID_DMA1_CH0                              11u  
#define  BSP_INT_ID_DMA1_CH1                              12u  
#define  BSP_INT_ID_DMA1_CH2                              13u  
#define  BSP_INT_ID_DMA1_CH3                              14u  
#define  BSP_INT_ID_DMA1_CH4                              15u 
#define  BSP_INT_ID_DMA1_CH5                              16u   
#define  BSP_INT_ID_DMA1_CH6                              17u  
#define  BSP_INT_ID_ADC                                   18u  
#define  BSP_INT_ID_CAN1_TX                               19u   
#define  BSP_INT_ID_CAN1_RX0                              20u   
#define  BSP_INT_ID_CAN1_RX1                              21u  
#define  BSP_INT_ID_CAN1_SCE                              22u   
#define  BSP_INT_ID_EXTI9_5                               23u  
#define  BSP_INT_ID_TIM1_BRK_TIM9                         24u   
#define  BSP_INT_ID_TIM1_UP_TIM10                         25u   
#define  BSP_INT_ID_TIM1_TRG_COM_TIM11                    26u   
#define  BSP_INT_ID_TIM1_CC                               27u 
#define  BSP_INT_ID_TIM2                                  28u   
#define  BSP_INT_ID_TIM3                                  29u   
#define  BSP_INT_ID_TIM4                                  30u   
#define  BSP_INT_ID_I2C1_EV                               31u   
#define  BSP_INT_ID_I2C1_ER                               32u   
#define  BSP_INT_ID_I2C2_EV                               33u   
#define  BSP_INT_ID_I2C2_ER                               34u   
#define  BSP_INT_ID_SPI1                                  35u  
#define  BSP_INT_ID_SPI2                                  36u   
#define  BSP_INT_ID_USART1                                37u   
#define  BSP_INT_ID_USART2                                38u   
#define  BSP_INT_ID_USART3                                39u   
#define  BSP_INT_ID_EXTI15_10                             40u   
#define  BSP_INT_ID_RTC_ALARM                             41u   
#define  BSP_INT_ID_OTG_FS_WKUP                           42u   

#define  BSP_INT_ID_TIM8_BRK_TIM12                        43u   
#define  BSP_INT_ID_TIM8_UP_TIM13                         44u  
#define  BSP_INT_ID_TIM8_TRG_COM_TIM14                    45u   
#define  BSP_INT_ID_TIM8_CC                               46u   
#define  BSP_INT_ID_DMA1_STREAM7                          47u   
#define  BSP_INT_ID_FSMC                                  48u   
#define  BSP_INT_ID_SDIO                                  49u   

#define  BSP_INT_ID_TIM5                                  50u   
#define  BSP_INT_ID_SPI3                                  51u  
#define  BSP_INT_ID_USART4                                52u   
#define  BSP_INT_ID_USART5                                53u  
#define  BSP_INT_ID_TIM6_DAC                              54u   /* TIM6 global Interrupt, DAC1 & DAC2 underrun err int. */
#define  BSP_INT_ID_TIM7                                  55u   /* TIM7 global Interrupt                                */
#define  BSP_INT_ID_DMA2_CH0                              56u   /* DMA2 Channel 0 global Interrupt                      */
#define  BSP_INT_ID_DMA2_CH1                              57u   /* DMA2 Channel 1 global Interrupt                      */
#define  BSP_INT_ID_DMA2_CH2                              58u   /* DMA2 Channel 2 global Interrupt                      */
#define  BSP_INT_ID_DMA2_CH3                              59u   /* DMA2 Channel 3 global Interrupt                      */
#define  BSP_INT_ID_DMA2_CH4                              60u   /* DMA2 Channel 4 global Interrupt                      */

#define  BSP_INT_ID_ETH                                   61u   /* ETH  global Interrupt                                */
#define  BSP_INT_ID_ETH_WKUP                              62u   /* ETH  WakeUp from EXTI line interrupt                 */
#define  BSP_INT_ID_CAN2_TX                               63u   /* CAN2 TX Interrupts                                   */
#define  BSP_INT_ID_CAN2_RX0                              64u   /* CAN2 RX0 Interrupts                                  */
#define  BSP_INT_ID_CAN2_RX1                              65u   /* CAN2 RX1 Interrupt                                   */
#define  BSP_INT_ID_CAN2_SCE                              66u   /* CAN2 SCE Interrupt                                   */
#define  BSP_INT_ID_OTG_FS                                67u   /* OTG global Interrupt                                 */

#define  BSP_INT_ID_DMA2_CH5                              68u   /* DMA2 Channel 5 global Interrupt                      */
#define  BSP_INT_ID_DMA2_CH6                              69u   /* DMA2 Channel 6 global Interrupt                      */
#define  BSP_INT_ID_DMA2_CH7                              70u   /* DMA2 Channel 7 global Interrupt                      */
#define  BSP_INT_ID_USART6                                71u   /* USART5 global Interrupt                              */
#define  BSP_INT_ID_I2C3_EV                               72u   /* I2C3 Event  Interrupt                                */
#define  BSP_INT_ID_I2C3_ER                               73u   /* I2C3 Error  Interrupt                                */
#define  BSP_INT_ID_OTG_HS_EP1_OUT                        74u   /* OTG HS EP1 OUT global Interrupt                      */
#define  BSP_INT_ID_OTG_HS_EP1_IN                         75u   /* OTG HS EP1 IN global Interrupt                       */
#define  BSP_INT_ID_OTG_HS_WKUP                           76u   /* OTG HS Wakeup Interrupt                              */
#define  BSP_INT_ID_OTG_HS                                77u   /* OTG HS global Interrupt                              */
#define  BSP_INT_ID_DCMI                                  78u   /* DCMI global Interrupt                                */
#define  BSP_INT_ID_CRYP                                  79u   /* CRYPT crypto global Interrupt                        */
#define  BSP_INT_ID_HASH_RNG                              80u   /* HASH and RNG global Interrupt                        */
#define  BSP_INT_ID_FPU                                   81u   /* FPU global Interrupt                                 */

#define	 BSP_INT_ID_USART7              				  82u  
#define  BSP_INT_ID_USART8              				  83u
#define  BSP_INT_ID_SPI4                				  84u
#define  BSP_INT_ID_SPI5                				  85u
#define  BSP_INT_ID_SPI6                				  86u
#define  BSP_INT_ID_SAI1         		  				  87u
#define  BSP_INT_ID_LTDC        		  				  88u
#define  BSP_INT_ID_LTDC_ER			  					  89u
#define  BSP_INT_ID_DMA2D				  				  90u

#define  BSP_INT_SRC_NBR                                  91u


/*
*********************************************************************************************************
*                                           FUNCTION PROTOTYPES
*********************************************************************************************************
*/
void        BSP_Init       (void);
void        BSP_IntDisAll  (void);
void 		BSP_RCC_PLL_Config(void);
void        BSP_Tick_Init  (void);





/*
*********************************************************************************************************
*                                             MODULE END
*********************************************************************************************************
*/

#endif /* End of module include */




