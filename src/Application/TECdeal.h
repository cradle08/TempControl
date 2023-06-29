/**
  ******************************************************************************
  * @file    TECdeal.h
  * @author  Firmware-Team
  * @version V1.0.0
  * @date    13-06-2016
  * @brief
  ******************************************************************************
**/

#ifndef  __TECDEAL__
#define  __TECDEAL__

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

/**************************************************************************************
**TECA   NSLEEP引脚:设备休眠:将逻辑拉低使设备进入低功耗休眠模式  IN1 IN2：PWM输入
**NSLEEP_A:PC0  IN1_A:PA0   IN2_A:PA1
**************************************************************************************/
#define NSLEEP_PORT_A					GPIOC //NSLEEP_A:PC0
#define NSLEEP_PIN_A 					GPIO_PIN_0
#define NSLEEP_A_GPIO_CLK()		__HAL_RCC_GPIOC_CLK_ENABLE()

#define IN1_PORT_A 						GPIOA //IN1_A:PA0 
#define IN1_PIN_A 						GPIO_PIN_0
#define IN1_A_GPIO_CLK()			__HAL_RCC_GPIOA_CLK_ENABLE()
#define IN1_A_CHANNEL					TIM_CHANNEL_1

#define IN2_PORT_A						GPIOA //IN2_A:PA1
#define IN2_PIN_A							GPIO_PIN_1
#define IN2_A_GPIO_CLK()			__HAL_RCC_GPIOA_CLK_ENABLE()
#define IN2_A_CHANNEL					TIM_CHANNEL_2




/**************************************************************************************
**TECB   NSLEEP引脚:设备休眠:将逻辑拉低使设备进入低功耗休眠模式  IN1 IN2：PWM输入
**NSLEEP_B:PB0  IN1_B:PA2   IN2_B:PA3
**************************************************************************************/
#define NSLEEP_PORT_B					GPIOB
#define NSLEEP_PIN_B 					GPIO_PIN_0
#define NSLEEP_B_GPIO_CLK()		__HAL_RCC_GPIOB_CLK_ENABLE()

#define IN1_PORT_B						GPIOA
#define IN1_PIN_B							GPIO_PIN_2
#define IN1_B_GPIO_CLK()			__HAL_RCC_GPIOA_CLK_ENABLE()
#define IN1_B_CHANNEL					TIM_CHANNEL_3

#define IN2_PORT_B						GPIOA
#define IN2_PIN_B							GPIO_PIN_3
#define IN2_B_GPIO_CLK()			__HAL_RCC_GPIOA_CLK_ENABLE()
#define IN2_B_CHANNEL					TIM_CHANNEL_4




/**************************************************************************************
**TECC   NSLEEP引脚:设备休眠:将逻辑拉低使设备进入低功耗休眠模式  IN1 IN2：PWM输入
**NSLEEP_C:PA8  IN1_C:PC6   IN2_C:PC7
**************************************************************************************/
#define NSLEEP_PORT_C 				GPIOA
#define NSLEEP_PIN_C 					GPIO_PIN_8
#define NSLEEP_C_GPIO_CLK()		__HAL_RCC_GPIOA_CLK_ENABLE()

#define IN1_PORT_C						GPIOC
#define IN1_PIN_C							GPIO_PIN_6
#define IN1_C_GPIO_CLK()			__HAL_RCC_GPIOC_CLK_ENABLE()
#define IN1_C_CHANNEL					TIM_CHANNEL_1

#define IN2_PORT_C						GPIOC
#define IN2_PIN_C							GPIO_PIN_7
#define IN2_C_GPIO_CLK()			__HAL_RCC_GPIOC_CLK_ENABLE()
#define IN2_C_CHANNEL					TIM_CHANNEL_2



/**************************************************************************************
**TECD   NSLEEP引脚:设备休眠:将逻辑拉低使设备进入低功耗休眠模式  IN1 IN2：PWM输入
**NSLEEP_C:PC12  IN1_C:PC8   IN2_C:PC9
**************************************************************************************/
#define NSLEEP_PORT_D 				GPIOC
#define NSLEEP_PIN_D 					GPIO_PIN_12
#define NSLEEP_D_GPIO_CLK()		__HAL_RCC_GPIOC_CLK_ENABLE()

#define IN1_PORT_D						GPIOC
#define IN1_PIN_D							GPIO_PIN_8
#define IN1_D_GPIO_CLK()			__HAL_RCC_GPIOC_CLK_ENABLE()
#define IN1_D_CHANNEL					TIM_CHANNEL_3

#define IN2_PORT_D						GPIOC
#define IN2_PIN_D							GPIO_PIN_9
#define IN2_D_GPIO_CLK()			__HAL_RCC_GPIOC_CLK_ENABLE()
#define IN2_D_CHANNEL					TIM_CHANNEL_4






/*
*********************************************************************************************************
*                                        LOCAL GLOBAL VARIABLES
*********************************************************************************************************
*/

typedef enum
{
  IN_1_A = 0,
  IN_2_A,
  IN_1_B,
  IN_2_B,
  IN_1_C,
  IN_2_C,
  IN_1_D,
  IN_2_D
} TecChannelDef;

typedef enum
{
  TEC_INI_OK = 0,
  TEC_OK,
  TEC_PERCENT_ERR,
  TEC_CHANNEL_ERR,
  HAL_PWM_ERR
} TecStatusDef;

/*
*********************************************************************************************************
*											FUNCTION PROTOTYPES
*********************************************************************************************************
*/
TecStatusDef TecInit( void );
TecStatusDef TecControl( TecChannelDef TecChannel, bool EnableFlag, u8 percent );

void TecTest( void );


/*
*********************************************************************************************************
*                                             MODULE END
*********************************************************************************************************
*/




#endif /* End */

