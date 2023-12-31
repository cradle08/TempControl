/**
  ******************************************************************************
  * @file    TECdeal.c
  * @author  Firmware-Team
  * @version V1.0.0
  * @date    24-09-2016
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

/* Definition for TIM3 */
#define HAL_TIM2                         TIM2
#define TIM2_CLK_ENABLE()                __HAL_RCC_TIM2_CLK_ENABLE()

/* Compute the prescaler value to have TIM3 counter clock equal to 10 MHz */
/* Initialize TIMx peripheral as follow:
	 + Prescaler = (SystemCoreClock/2)/10000000
	 + Period = 1000  (to have an output frequency equal to 10 KHz)
	 + ClockDivision = 0
	 + Counter direction = Up
*/

#define TIM2_CLOCK_EQUAL	15000000
#define TIM2_PRESCALER_VALUE ((SystemCoreClock /2) / TIM2_CLOCK_EQUAL) - 1;
#define TIM2_PERIOD_VALUE       (1000)  /* Period Value  */

#define TIM2_PULSE1_VALUE       500        /* Capture Compare 1 Value  */
#define TIM2_PULSE2_VALUE       600         /* Capture Compare 2 Value  */
#define TIM2_PULSE3_VALUE       400         /* Capture Compare 3 Value  */
#define TIM2_PULSE4_VALUE       200        /* Capture Compare 4 Value  */


#define TIM2_CHANNEL1_EN 				   		true
#if TIM2_CHANNEL1_EN
  #define TIM2_CHANNEL1_GPIO_CLK()		IN1_A_GPIO_CLK()
  #define TIM2_GPIO_TYPE_CHANNEL1			IN1_PORT_A
  #define TIM2_GPIO_PIN_CHANNEL1			IN1_PIN_A
#endif

#define TIM2_CHANNEL2_EN 				   		true
#if TIM2_CHANNEL2_EN
  #define TIM2_CHANNEL2_GPIO_CLK()		IN2_A_GPIO_CLK()
  #define TIM2_GPIO_TYPE_CHANNEL2			IN2_PORT_A
  #define TIM2_GPIO_PIN_CHANNEL2			IN2_PIN_A
#endif

#define TIM2_CHANNEL3_EN 				   		true
#if TIM2_CHANNEL3_EN
  #define TIM2_CHANNEL3_GPIO_CLK()		IN1_B_GPIO_CLK()
  #define TIM2_GPIO_TYPE_CHANNEL3			IN1_PORT_B
  #define TIM2_GPIO_PIN_CHANNEL3			IN1_PIN_B
#endif

#define TIM2_CHANNEL4_EN 				   		true
#if TIM2_CHANNEL4_EN
  #define TIM2_CHANNEL4_GPIO_CLK()		IN2_B_GPIO_CLK()
  #define TIM2_GPIO_TYPE_CHANNEL4			IN2_PORT_B
  #define TIM2_GPIO_PIN_CHANNEL4			IN2_PIN_B
#endif

#define TIM2_CHANNEL_MODE							GPIO_MODE_AF_PP
#define TIM2_CHANNEL_PULL							GPIO_PULLDOWN
#define TIM2_CHANNEL_SPEED 						GPIO_SPEED_FREQ_LOW
#define TIM2_CHANNEL_AF								GPIO_AF1_TIM2


/* Definition for SPI2's NVIC */
#define TIM2_HAL_IRQn									TIM2_IRQn
#define TIM2_PREEMPT_PRIORITY			 		3
#define TIM2_SUB_PRIORITY				 			0


/* Definition for TIM3 */
#define HAL_TIM3											TIM3
#define TIM3_CLK_ENABLE()							__HAL_RCC_TIM3_CLK_ENABLE()

/* Compute the prescaler value to have TIM3 counter clock equal to 10 MHz */
/* Initialize TIMx peripheral as follow:
	 + Prescaler = (SystemCoreClock/2)/10000000
	 + Period = 1000  (to have an output frequency equal to 10 KHz)
	 + ClockDivision = 0
	 + Counter direction = Up
*/

#define TIM3_CLOCK_EQUAL	15000000
#define TIM3_PRESCALER_VALUE ((SystemCoreClock /2) / TIM3_CLOCK_EQUAL) - 1;
#define TIM3_PERIOD_VALUE       (1000)  /* Period Value  */

#define TIM3_PULSE1_VALUE       1350        /* Capture Compare 1 Value  */
#define TIM3_PULSE2_VALUE       900         /* Capture Compare 2 Value  */
#define TIM3_PULSE3_VALUE       600         /* Capture Compare 3 Value  */
#define TIM3_PULSE4_VALUE       450         /* Capture Compare 4 Value  */


#define TIM3_CHANNEL1_EN 				   		true
#if TIM3_CHANNEL1_EN
  #define TIM3_CHANNEL1_GPIO_CLK()		IN1_C_GPIO_CLK()
  #define TIM3_GPIO_TYPE_CHANNEL1			IN1_PORT_C
  #define TIM3_GPIO_PIN_CHANNEL1			IN1_PIN_C
#endif

#define TIM3_CHANNEL2_EN 				   		true
#if TIM3_CHANNEL2_EN
  #define TIM3_CHANNEL2_GPIO_CLK()		IN2_C_GPIO_CLK()
  #define TIM3_GPIO_TYPE_CHANNEL2			IN2_PORT_C
  #define TIM3_GPIO_PIN_CHANNEL2			IN2_PIN_C
#endif

#define TIM3_CHANNEL3_EN 				   		true
#if TIM3_CHANNEL3_EN
  #define TIM3_CHANNEL3_GPIO_CLK()		IN1_D_GPIO_CLK()
  #define TIM3_GPIO_TYPE_CHANNEL3			IN1_PORT_D
  #define TIM3_GPIO_PIN_CHANNEL3			IN1_PIN_D
#endif

#define TIM3_CHANNEL4_EN 				   		true
#if TIM3_CHANNEL4_EN
  #define TIM3_CHANNEL4_GPIO_CLK()		IN2_D_GPIO_CLK()
  #define TIM3_GPIO_TYPE_CHANNEL4			IN2_PORT_D
  #define TIM3_GPIO_PIN_CHANNEL4			IN2_PIN_D
#endif

#define TIM3_CHANNEL_MODE							GPIO_MODE_AF_PP
#define TIM3_CHANNEL_PULL							GPIO_PULLDOWN
#define TIM3_CHANNEL_SPEED						GPIO_SPEED_FREQ_LOW
#define TIM3_CHANNEL_AF								GPIO_AF2_TIM3


/* Definition for SPI2's NVIC */
#define TIM3_HAL_IRQn									TIM3_IRQn
#define TIM3_PREEMPT_PRIORITY			 		3
#define TIM3_SUB_PRIORITY				 			0


#define TEC_NSLEEP_MODE								GPIO_MODE_OUTPUT_PP
#define TEC_NSLEEP_PULL								GPIO_PULLDOWN
#define TEC_NSLEEP_SPEED 							GPIO_SPEED_FREQ_LOW


/**************************************************************************************
设置DRV8701芯片的    NSLEEP引脚:设备休眠:将逻辑拉低使设备进入低功耗休眠模式
**************************************************************************************/
#define NSLEEP_A_ENABLE(x)	HAL_GPIO_WritePin(NSLEEP_PORT_A,NSLEEP_PIN_A,x)
#define NSLEEP_B_ENABLE(x)	HAL_GPIO_WritePin(NSLEEP_PORT_B,NSLEEP_PIN_B,x)
#define NSLEEP_C_ENABLE(x)	HAL_GPIO_WritePin(NSLEEP_PORT_C,NSLEEP_PIN_C,x)
#define NSLEEP_D_ENABLE(x)	HAL_GPIO_WritePin(NSLEEP_PORT_D,NSLEEP_PIN_D,x)







/*
*********************************************************************************************************
*                                            LOCAL DEFINES
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                        LOCAL GLOBAL VARIABLES
*********************************************************************************************************
*/
BspPwmHandle INABPwmhandle, INCDPwmhandle;



/*
*********************************************************************************************************
*                                         FUNCTION PROTOTYPES
*********************************************************************************************************
*/

static void Tim2ClkEnable( void )
{
  TIM2_CLK_ENABLE();
}

#if TIM2_CHANNEL1_EN
static void Tim2Channel1ClkEnable( void )
{
  TIM2_CHANNEL1_GPIO_CLK();
}
#endif

#if TIM2_CHANNEL2_EN
static void Tim2Channel2ClkEnable( void )
{
  TIM2_CHANNEL2_GPIO_CLK();
}
#endif

#if TIM2_CHANNEL3_EN
static void Tim2Channel3ClkEnable( void )
{
  TIM2_CHANNEL3_GPIO_CLK();
}
#endif

#if TIM2_CHANNEL4_EN
static void Tim2Channel4ClkEnable( void )
{
  TIM2_CHANNEL4_GPIO_CLK();
}
#endif

static void Tim2CallBackIT( void )
{

}


static void Tim3ClkEnable( void )
{
  TIM3_CLK_ENABLE();
}

#if TIM3_CHANNEL1_EN
static void Tim3Channel1ClkEnable( void )
{
  TIM3_CHANNEL1_GPIO_CLK();
}
#endif

#if TIM3_CHANNEL2_EN
static void Tim3Channel2ClkEnable( void )
{
  TIM3_CHANNEL2_GPIO_CLK();
}
#endif

#if TIM3_CHANNEL3_EN
static void Tim3Channel3ClkEnable( void )
{
  TIM3_CHANNEL3_GPIO_CLK();
}
#endif

#if TIM3_CHANNEL4_EN
static void Tim3Channel4ClkEnable( void )
{
  TIM3_CHANNEL4_GPIO_CLK();
}
#endif

void Tim2Config( void )
{
  Tim2Handle.TimHandleInit.Instance 					= HAL_TIM2;
  Tim2Handle.TimHandleInit.Init.Prescaler	 		= TIM2_PRESCALER_VALUE;
  Tim2Handle.TimHandleInit.Init.Period		 		= TIM2_PERIOD_VALUE;
  Tim2Handle.TimHandleInit.Init.ClockDivision	= 0;
  Tim2Handle.TimHandleInit.Init.CounterMode		= TIM_COUNTERMODE_UP;

  Tim2Handle.TimClkEnable 						= Tim2ClkEnable;
  Tim2Handle.IrqInit.Irqn							= TIM2_HAL_IRQn;
  Tim2Handle.IrqInit.PreemptPriority	= TIM2_PREEMPT_PRIORITY;
  Tim2Handle.IrqInit.SubPriority			= TIM2_SUB_PRIORITY;
  Tim2Handle.TimCallBack 							= Tim2CallBackIT;

  if( BspTimHandleInit( &Tim2Handle ) != HAL_OK )
  {
    /* Initialization Error */
    SaveLog( NULL, 0 );
  }

  BspTimHandleStartIT( &Tim2Handle );
}


/**************************************************************************************
** 函数名称: INPwmConfig
** 参数    : 无
** 函数功能: 配置TIM2 TIM3为PWM输出一共输出8路PWM
** 返回值  : 无
**************************************************************************************/
void INPwmConfig( void )
{
  /**************************************************************************************
  配置TIM2定时器为PWM模式 输出4路PWM
  **************************************************************************************/
  INABPwmhandle.TimHandleInit.Instance						= HAL_TIM2;
  INABPwmhandle.TimHandleInit.Init.Prescaler			= TIM2_PRESCALER_VALUE;
  INABPwmhandle.TimHandleInit.Init.Period					= TIM2_PERIOD_VALUE;
  INABPwmhandle.TimHandleInit.Init.ClockDivision	= 0;
  INABPwmhandle.TimHandleInit.Init.CounterMode		= TIM_COUNTERMODE_UP;
  INABPwmhandle.TimClkEnable 											= Tim2ClkEnable;

  /**通道使能，相关参数配置**/
#if TIM2_CHANNEL1_EN
  INABPwmhandle.PwmChannel1.EnableFlag					= TIM2_CHANNEL1_EN;
  INABPwmhandle.PwmChannel1.GpioClkInit					= Tim2Channel1ClkEnable;

  INABPwmhandle.PwmChannel1.GpioPort						= TIM2_GPIO_TYPE_CHANNEL1;
  INABPwmhandle.PwmChannel1.GpioInit.Pin 				= TIM2_GPIO_PIN_CHANNEL1;
  INABPwmhandle.PwmChannel1.GpioInit.Mode				= TIM2_CHANNEL_MODE;
  INABPwmhandle.PwmChannel1.GpioInit.Pull				= TIM2_CHANNEL_PULL;
  INABPwmhandle.PwmChannel1.GpioInit.Speed			= TIM2_CHANNEL_SPEED;
  INABPwmhandle.PwmChannel1.GpioInit.Alternate	= TIM2_CHANNEL_AF;

  INABPwmhandle.PwmChannel1.OConfig.OCMode			= TIM_OCMODE_PWM1;
  INABPwmhandle.PwmChannel1.OConfig.OCPolarity	= TIM_OCPOLARITY_HIGH;
  INABPwmhandle.PwmChannel1.OConfig.OCFastMode	= TIM_OCFAST_DISABLE;
  INABPwmhandle.PwmChannel1.OConfig.Pulse				= TIM2_PULSE1_VALUE;
#endif

  /**通道使能，相关参数配置**/
#if TIM2_CHANNEL2_EN
  INABPwmhandle.PwmChannel2.EnableFlag					= TIM2_CHANNEL2_EN;
  INABPwmhandle.PwmChannel2.GpioClkInit					= Tim2Channel2ClkEnable;

  INABPwmhandle.PwmChannel2.GpioPort						= TIM2_GPIO_TYPE_CHANNEL2;
  INABPwmhandle.PwmChannel2.GpioInit.Pin 				= TIM2_GPIO_PIN_CHANNEL2;
  INABPwmhandle.PwmChannel2.GpioInit.Mode				= TIM2_CHANNEL_MODE;
  INABPwmhandle.PwmChannel2.GpioInit.Pull				= TIM2_CHANNEL_PULL;
  INABPwmhandle.PwmChannel2.GpioInit.Speed			= TIM2_CHANNEL_SPEED;
  INABPwmhandle.PwmChannel2.GpioInit.Alternate	= TIM2_CHANNEL_AF;

  INABPwmhandle.PwmChannel2.OConfig.OCMode			= TIM_OCMODE_PWM1;
  INABPwmhandle.PwmChannel2.OConfig.OCPolarity	= TIM_OCPOLARITY_HIGH;
  INABPwmhandle.PwmChannel2.OConfig.OCFastMode	= TIM_OCFAST_DISABLE;
  INABPwmhandle.PwmChannel2.OConfig.Pulse				= TIM2_PULSE2_VALUE;
#endif

  /**通道使能，相关参数配置**/
#if TIM2_CHANNEL3_EN
  INABPwmhandle.PwmChannel3.EnableFlag					= TIM2_CHANNEL3_EN;
  INABPwmhandle.PwmChannel3.GpioClkInit					= Tim2Channel3ClkEnable;

  INABPwmhandle.PwmChannel3.GpioPort						= TIM2_GPIO_TYPE_CHANNEL3;
  INABPwmhandle.PwmChannel3.GpioInit.Pin 				= TIM2_GPIO_PIN_CHANNEL3;
  INABPwmhandle.PwmChannel3.GpioInit.Mode				= TIM2_CHANNEL_MODE;
  INABPwmhandle.PwmChannel3.GpioInit.Pull				= TIM2_CHANNEL_PULL;
  INABPwmhandle.PwmChannel3.GpioInit.Speed			= TIM2_CHANNEL_SPEED;
  INABPwmhandle.PwmChannel3.GpioInit.Alternate	= TIM2_CHANNEL_AF;

  INABPwmhandle.PwmChannel3.OConfig.OCMode			= TIM_OCMODE_PWM1;
  INABPwmhandle.PwmChannel3.OConfig.OCPolarity	= TIM_OCPOLARITY_HIGH;
  INABPwmhandle.PwmChannel3.OConfig.OCFastMode	= TIM_OCFAST_DISABLE;
  INABPwmhandle.PwmChannel3.OConfig.Pulse				= TIM2_PULSE3_VALUE;
#endif

  /**通道使能，相关参数配置**/
#if TIM2_CHANNEL4_EN
  INABPwmhandle.PwmChannel4.EnableFlag					= TIM2_CHANNEL4_EN;
  INABPwmhandle.PwmChannel4.GpioClkInit					= Tim2Channel4ClkEnable;

  INABPwmhandle.PwmChannel4.GpioPort						= TIM2_GPIO_TYPE_CHANNEL4;
  INABPwmhandle.PwmChannel4.GpioInit.Pin				= TIM2_GPIO_PIN_CHANNEL4;
  INABPwmhandle.PwmChannel4.GpioInit.Mode				= TIM2_CHANNEL_MODE;
  INABPwmhandle.PwmChannel4.GpioInit.Pull				= TIM2_CHANNEL_PULL;
  INABPwmhandle.PwmChannel4.GpioInit.Speed			= TIM2_CHANNEL_SPEED;
  INABPwmhandle.PwmChannel4.GpioInit.Alternate	= TIM2_CHANNEL_AF;

  INABPwmhandle.PwmChannel4.OConfig.OCMode			= TIM_OCMODE_PWM1;
  INABPwmhandle.PwmChannel4.OConfig.OCPolarity	= TIM_OCPOLARITY_HIGH;
  INABPwmhandle.PwmChannel4.OConfig.OCFastMode	= TIM_OCFAST_DISABLE;
  INABPwmhandle.PwmChannel4.OConfig.Pulse				= TIM2_PULSE4_VALUE;
#endif
  //if(HAL_TIM_Base_Init(&Pwm1handle.TimHandleInit) == HAL_OK)
  if( BspPwmHandleInit( &INABPwmhandle ) != HAL_OK )
  {
  }


  /**************************************************************************************
  配置TIM4定时器为PWM模式 输出4路PWM
  **************************************************************************************/
  INCDPwmhandle.TimHandleInit.Instance						= HAL_TIM3;
  INCDPwmhandle.TimHandleInit.Init.Prescaler			= TIM3_PRESCALER_VALUE;
  INCDPwmhandle.TimHandleInit.Init.Period					= TIM3_PERIOD_VALUE;
  INCDPwmhandle.TimHandleInit.Init.ClockDivision	= 0;
  INCDPwmhandle.TimHandleInit.Init.CounterMode		= TIM_COUNTERMODE_UP;
  INCDPwmhandle.TimClkEnable 											= Tim3ClkEnable;

  /**通道使能，相关参数配置**/
#if TIM3_CHANNEL1_EN
  INCDPwmhandle.PwmChannel1.EnableFlag					= TIM3_CHANNEL1_EN;
  INCDPwmhandle.PwmChannel1.GpioClkInit					= Tim3Channel1ClkEnable;

  INCDPwmhandle.PwmChannel1.GpioPort						= TIM3_GPIO_TYPE_CHANNEL1;
  INCDPwmhandle.PwmChannel1.GpioInit.Pin				= TIM3_GPIO_PIN_CHANNEL1;
  INCDPwmhandle.PwmChannel1.GpioInit.Mode				= TIM3_CHANNEL_MODE;
  INCDPwmhandle.PwmChannel1.GpioInit.Pull				= TIM3_CHANNEL_PULL;
  INCDPwmhandle.PwmChannel1.GpioInit.Speed			= TIM3_CHANNEL_SPEED;
  INCDPwmhandle.PwmChannel1.GpioInit.Alternate	= TIM3_CHANNEL_AF;

  INCDPwmhandle.PwmChannel1.OConfig.OCMode			= TIM_OCMODE_PWM1;
  INCDPwmhandle.PwmChannel1.OConfig.OCPolarity	= TIM_OCPOLARITY_HIGH;
  INCDPwmhandle.PwmChannel1.OConfig.OCFastMode	= TIM_OCFAST_DISABLE;
  INCDPwmhandle.PwmChannel1.OConfig.Pulse				= TIM3_PULSE1_VALUE;
#endif

  /**通道使能，相关参数配置**/
#if TIM3_CHANNEL2_EN
  INCDPwmhandle.PwmChannel2.EnableFlag					= TIM3_CHANNEL2_EN;
  INCDPwmhandle.PwmChannel2.GpioClkInit					= Tim3Channel2ClkEnable;

  INCDPwmhandle.PwmChannel2.GpioPort						= TIM3_GPIO_TYPE_CHANNEL2;
  INCDPwmhandle.PwmChannel2.GpioInit.Pin 				= TIM3_GPIO_PIN_CHANNEL2;
  INCDPwmhandle.PwmChannel2.GpioInit.Mode				= TIM3_CHANNEL_MODE;
  INCDPwmhandle.PwmChannel2.GpioInit.Pull				= TIM3_CHANNEL_PULL;
  INCDPwmhandle.PwmChannel2.GpioInit.Speed			= TIM3_CHANNEL_SPEED;
  INCDPwmhandle.PwmChannel2.GpioInit.Alternate	= TIM3_CHANNEL_AF;

  INCDPwmhandle.PwmChannel2.OConfig.OCMode			= TIM_OCMODE_PWM1;
  INCDPwmhandle.PwmChannel2.OConfig.OCPolarity	= TIM_OCPOLARITY_HIGH;
  INCDPwmhandle.PwmChannel2.OConfig.OCFastMode	= TIM_OCFAST_DISABLE;
  INCDPwmhandle.PwmChannel2.OConfig.Pulse				= TIM3_PULSE2_VALUE;
#endif

  /**通道使能，相关参数配置**/
#if TIM3_CHANNEL3_EN
  INCDPwmhandle.PwmChannel3.EnableFlag					= TIM3_CHANNEL3_EN;
  INCDPwmhandle.PwmChannel3.GpioClkInit					= Tim3Channel3ClkEnable;

  INCDPwmhandle.PwmChannel3.GpioPort						= TIM3_GPIO_TYPE_CHANNEL3;
  INCDPwmhandle.PwmChannel3.GpioInit.Pin 				= TIM3_GPIO_PIN_CHANNEL3;
  INCDPwmhandle.PwmChannel3.GpioInit.Mode				= TIM3_CHANNEL_MODE;
  INCDPwmhandle.PwmChannel3.GpioInit.Pull				= TIM3_CHANNEL_PULL;
  INCDPwmhandle.PwmChannel3.GpioInit.Speed			= TIM3_CHANNEL_SPEED;
  INCDPwmhandle.PwmChannel3.GpioInit.Alternate	= TIM3_CHANNEL_AF;

  INCDPwmhandle.PwmChannel3.OConfig.OCMode			= TIM_OCMODE_PWM1;
  INCDPwmhandle.PwmChannel3.OConfig.OCPolarity	= TIM_OCPOLARITY_HIGH;
  INCDPwmhandle.PwmChannel3.OConfig.OCFastMode	= TIM_OCFAST_DISABLE;
  INCDPwmhandle.PwmChannel3.OConfig.Pulse				= TIM3_PULSE3_VALUE;
#endif
  /**通道使能，相关参数配置**/
#if TIM3_CHANNEL4_EN
  INCDPwmhandle.PwmChannel4.EnableFlag					= TIM3_CHANNEL4_EN;
  INCDPwmhandle.PwmChannel4.GpioClkInit					= Tim3Channel4ClkEnable;

  INCDPwmhandle.PwmChannel4.GpioPort						= TIM3_GPIO_TYPE_CHANNEL4;
  INCDPwmhandle.PwmChannel4.GpioInit.Pin 				= TIM3_GPIO_PIN_CHANNEL4;
  INCDPwmhandle.PwmChannel4.GpioInit.Mode				= TIM3_CHANNEL_MODE;
  INCDPwmhandle.PwmChannel4.GpioInit.Pull				= TIM3_CHANNEL_PULL;
  INCDPwmhandle.PwmChannel4.GpioInit.Speed			= TIM3_CHANNEL_SPEED;
  INCDPwmhandle.PwmChannel4.GpioInit.Alternate	= TIM3_CHANNEL_AF;

  INCDPwmhandle.PwmChannel4.OConfig.OCMode			= TIM_OCMODE_PWM1;
  INCDPwmhandle.PwmChannel4.OConfig.OCPolarity	= TIM_OCPOLARITY_HIGH;
  INCDPwmhandle.PwmChannel4.OConfig.OCFastMode	= TIM_OCFAST_DISABLE;
  INCDPwmhandle.PwmChannel4.OConfig.Pulse				= TIM3_PULSE4_VALUE;
#endif

  if( BspPwmHandleInit( &INCDPwmhandle ) != HAL_OK )
  {
  }

  //BspPwmHandleStart(&INCDPwmhandle,IN1_C_CHANNEL);
  //BspPwmHandleStart(&INCDPwmhandle,IN2_C_CHANNEL);
  //BspPwmHandleStart(&INCDPwmhandle,IN1_D_CHANNEL);
  //BspPwmHandleStart(&INCDPwmhandle,IN2_D_CHANNEL);
}


/**************************************************************************************
** 函数名称: TecGpioConfig
** 参数    : 无
** 函数功能: TEC模块电路芯片DRV8701的 NSLEEP引脚配置
** 返回值  : 无
**************************************************************************************/
void TecGpioConfig( void )
{
  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  NSLEEP_A_GPIO_CLK();
  GPIO_InitStruct.Pin		= NSLEEP_PIN_A;
  GPIO_InitStruct.Mode	= TEC_NSLEEP_MODE;
  GPIO_InitStruct.Pull 	= TEC_NSLEEP_PULL;
  GPIO_InitStruct.Speed = TEC_NSLEEP_SPEED;
  HAL_GPIO_Init( NSLEEP_PORT_A, &GPIO_InitStruct );

  /* GPIO Ports Clock Enable */
  NSLEEP_B_GPIO_CLK();
  GPIO_InitStruct.Pin 	= NSLEEP_PIN_B;
  GPIO_InitStruct.Mode 	= TEC_NSLEEP_MODE;
  GPIO_InitStruct.Pull 	= TEC_NSLEEP_PULL;
  GPIO_InitStruct.Speed = TEC_NSLEEP_SPEED;
  HAL_GPIO_Init( NSLEEP_PORT_B, &GPIO_InitStruct );

  /* GPIO Ports Clock Enable */
  NSLEEP_C_GPIO_CLK();
  GPIO_InitStruct.Pin 	= NSLEEP_PIN_C;
  GPIO_InitStruct.Mode 	= TEC_NSLEEP_MODE;
  GPIO_InitStruct.Pull 	= TEC_NSLEEP_PULL;
  GPIO_InitStruct.Speed = TEC_NSLEEP_SPEED;
  HAL_GPIO_Init( NSLEEP_PORT_C, &GPIO_InitStruct );

  /* GPIO Ports Clock Enable */
  NSLEEP_D_GPIO_CLK();
  GPIO_InitStruct.Pin 	= NSLEEP_PIN_D;
  GPIO_InitStruct.Mode 	= TEC_NSLEEP_MODE;
  GPIO_InitStruct.Pull 	= TEC_NSLEEP_PULL;
  GPIO_InitStruct.Speed = TEC_NSLEEP_SPEED;
  HAL_GPIO_Init( NSLEEP_PORT_D, &GPIO_InitStruct );

#if 0 //PWM引脚在pwm模式里面配置了不需要在配置
  /* GPIO Ports Clock Enable */
  IN1_A_GPIO_CLK();
  GPIO_InitStruct.Pin 	= IN1_PIN_A;
  GPIO_InitStruct.Mode 	= TEC_NSLEEP_MODE;
  GPIO_InitStruct.Pull 	= TEC_NSLEEP_PULL;
  GPIO_InitStruct.Speed = TEC_NSLEEP_SPEED;
  HAL_GPIO_Init( IN1_PORT_A, &GPIO_InitStruct );
#endif
}


/*
*********************************************************************************************************
*                                          TecInit
*
* Description :TecInit() 初始化TEC,初始化了T2、T3定时器共八通道pwm，初始化四组全桥芯片nsleep为低电平
*

*
* Returns     : TecStatusDef 状态
*
* Notes       : 动态调整，不需要关闭定时器, 打开某个通道时，必须要先关闭相对通道，否则pwm不会输出
*********************************************************************************************************
*/

TecStatusDef TecInit( void )
{
  TecGpioConfig(); //配置NSLEEP引脚
  INPwmConfig();   //配置TIM2 TIM3输出PWM

  //HAL_GPIO_WritePin(IN1_PORT_A,IN1_PIN_A,true);
  //HAL_GPIO_WritePin(IN1_PORT_A,IN1_PIN_A,false);
  //HAL_GPIO_WritePin(IN1_PORT_A,IN1_PIN_A,true);
  //HAL_GPIO_WritePin(IN1_PORT_A,IN1_PIN_A,false);
  //Tim2Config();


  NSLEEP_A_ENABLE( GPIO_PIN_RESET ); //配置为休眠模式
  NSLEEP_B_ENABLE( GPIO_PIN_RESET );
  NSLEEP_C_ENABLE( GPIO_PIN_RESET );
  NSLEEP_D_ENABLE( GPIO_PIN_RESET );

  return TEC_INI_OK;
}


/*
*********************************************************************************************************
*                                          TecControl
*
* Description :TecControl() 打开或关闭一个通道的pwm输出
*
* Arguments   : TecChannel TEC通道
				EnableFlag 使能标志 true:打开 false:关闭
				percent 百分比，范围:0~100

*
* Returns     : TecStatusDef 错误类型
*
* Notes       : 动态调整，不需要关闭定时器, 打开某个通道时，必须要先关闭相对通道，否则pwm不会输出
*********************************************************************************************************
*/
TecStatusDef TecControl1( TecChannelDef TecChannel, bool EnableFlag, u8 percent )
{
  u32 channelPulse;
  TecStatusDef tecStatus = HAL_PWM_ERR;
  BspStatusTypeDef pwmStatus;

  if( percent > 100 )
  {
    return TEC_PERCENT_ERR;
  }


  switch( TecChannel )
  {
    case IN_1_A:
    {
      pwmStatus = BSP_OK;

      if( EnableFlag == true )
      {
        NSLEEP_A_ENABLE( GPIO_PIN_SET );
        pwmStatus = BspPwmHandleStop( &INABPwmhandle, IN2_A_CHANNEL );
        channelPulse = percent * TIM2_PERIOD_VALUE / 100;
      }
      else
      {
        NSLEEP_A_ENABLE( GPIO_PIN_RESET );
        channelPulse = 0;
      }

      if( pwmStatus == BSP_OK )
      {
        BspPwmPulseValueSet( &INABPwmhandle, IN1_A_CHANNEL, channelPulse );
        pwmStatus = BspPwmHandleStart( &INABPwmhandle, IN1_A_CHANNEL );
        if( pwmStatus == BSP_OK )
        {
          tecStatus = TEC_OK;
        }
        else
        {
          return HAL_PWM_ERR;
        }
      }
      else
      {
        return HAL_PWM_ERR;
      }
    }
    break;
    case IN_2_A:
    {
      pwmStatus = BSP_OK;

      if( EnableFlag == true )
      {
        NSLEEP_A_ENABLE( GPIO_PIN_SET );
        pwmStatus = BspPwmHandleStop( &INABPwmhandle, IN1_A_CHANNEL );
        channelPulse = percent * TIM2_PERIOD_VALUE / 100;
      }
      else
      {
        NSLEEP_A_ENABLE( GPIO_PIN_RESET );
        channelPulse = 0;
      }

      if( pwmStatus == BSP_OK )
      {
        BspPwmPulseValueSet( &INABPwmhandle, IN2_A_CHANNEL, channelPulse );
        pwmStatus = BspPwmHandleStart( &INABPwmhandle, IN2_A_CHANNEL );
        if( pwmStatus == BSP_OK )
        {
          tecStatus = TEC_OK;
        }
        else
        {
          return HAL_PWM_ERR;
        }
      }
      else
      {
        return HAL_PWM_ERR;
      }
    }
    break;
    case IN_1_B:
    {
      pwmStatus = BSP_OK;

      if( EnableFlag == true )
      {
        NSLEEP_B_ENABLE( GPIO_PIN_SET );
        pwmStatus = BspPwmHandleStop( &INABPwmhandle, IN2_B_CHANNEL );
        channelPulse = percent * TIM2_PERIOD_VALUE / 100;
      }
      else
      {
        NSLEEP_B_ENABLE( GPIO_PIN_RESET );
        channelPulse = 0;
      }

      if( pwmStatus == BSP_OK )
      {
        BspPwmPulseValueSet( &INABPwmhandle, IN1_B_CHANNEL, channelPulse );
        pwmStatus = BspPwmHandleStart( &INABPwmhandle, IN1_B_CHANNEL );
        if( pwmStatus == BSP_OK )
        {
          tecStatus = TEC_OK;
        }
        else
        {
          return HAL_PWM_ERR;
        }
      }
      else
      {
        return HAL_PWM_ERR;
      }
    }
    break;
    case IN_2_B:
    {
      pwmStatus = BSP_OK;

      if( EnableFlag == true )
      {
        NSLEEP_B_ENABLE( GPIO_PIN_SET );
        pwmStatus = BspPwmHandleStop( &INABPwmhandle, IN1_B_CHANNEL );
        channelPulse = percent * TIM2_PERIOD_VALUE / 100;
      }
      else
      {
        NSLEEP_B_ENABLE( GPIO_PIN_RESET );
        channelPulse = 0;
      }

      if( pwmStatus == BSP_OK )
      {
        BspPwmPulseValueSet( &INABPwmhandle, IN2_A_CHANNEL, channelPulse );
        pwmStatus = BspPwmHandleStart( &INABPwmhandle, IN2_B_CHANNEL );
        if( pwmStatus == BSP_OK )
        {
          tecStatus = TEC_OK;
        }
        else
        {
          return HAL_PWM_ERR;
        }
      }
      else
      {
        return HAL_PWM_ERR;
      }
    }
    break;
    case IN_1_C:
    {
      pwmStatus = BSP_OK;

      if( EnableFlag == true )
      {
        NSLEEP_C_ENABLE( GPIO_PIN_SET );
        pwmStatus = BspPwmHandleStop( &INCDPwmhandle, IN2_C_CHANNEL );
        channelPulse = percent * TIM3_PERIOD_VALUE / 100;
      }
      else
      {
        NSLEEP_C_ENABLE( GPIO_PIN_RESET );
        channelPulse = 0;
      }

      if( pwmStatus == BSP_OK )
      {
        BspPwmPulseValueSet( &INCDPwmhandle, IN1_C_CHANNEL, channelPulse );
        pwmStatus = BspPwmHandleStart( &INCDPwmhandle, IN1_C_CHANNEL );
        if( pwmStatus == BSP_OK )
        {
          tecStatus = TEC_OK;
        }
        else
        {
          return HAL_PWM_ERR;
        }
      }
      else
      {
        return HAL_PWM_ERR;
      }
    }
    break;
    case IN_2_C:
    {
      pwmStatus = BSP_OK;

      if( EnableFlag == true )
      {
        NSLEEP_C_ENABLE( GPIO_PIN_SET );
        pwmStatus = BspPwmHandleStop( &INCDPwmhandle, IN1_C_CHANNEL );
        channelPulse = percent * TIM2_PERIOD_VALUE / 100;
      }
      else
      {
        NSLEEP_C_ENABLE( GPIO_PIN_RESET );
        channelPulse = 0;
      }

      if( pwmStatus == BSP_OK )
      {
        BspPwmPulseValueSet( &INCDPwmhandle, IN2_C_CHANNEL, channelPulse );
        pwmStatus = BspPwmHandleStart( &INCDPwmhandle, IN2_C_CHANNEL );
        if( pwmStatus == BSP_OK )
        {
          tecStatus = TEC_OK;
        }
        else
        {
          return HAL_PWM_ERR;
        }
      }
      else
      {
        return HAL_PWM_ERR;
      }
    }
    break;
    case IN_1_D:
    {
      pwmStatus = BSP_OK;

      if( EnableFlag == true )
      {
        NSLEEP_D_ENABLE( GPIO_PIN_SET );
        pwmStatus = BspPwmHandleStop( &INCDPwmhandle, IN2_D_CHANNEL );
        channelPulse = percent * TIM3_PERIOD_VALUE / 100;
      }
      else
      {
        NSLEEP_D_ENABLE( GPIO_PIN_RESET );
        channelPulse = 0;
      }

      if( pwmStatus == BSP_OK )
      {
        BspPwmPulseValueSet( &INCDPwmhandle, IN1_D_CHANNEL, channelPulse );
        pwmStatus = BspPwmHandleStart( &INCDPwmhandle, IN1_D_CHANNEL );
        if( pwmStatus == BSP_OK )
        {
          tecStatus = TEC_OK;
        }
        else
        {
          return HAL_PWM_ERR;
        }
      }
      else
      {
        return HAL_PWM_ERR;
      }
    }
    break;
    case IN_2_D:
    {
      pwmStatus = BSP_OK;

      if( EnableFlag == true )
      {
        NSLEEP_D_ENABLE( GPIO_PIN_SET );
        pwmStatus = BspPwmHandleStop( &INCDPwmhandle, IN1_D_CHANNEL );
        channelPulse = percent * TIM2_PERIOD_VALUE / 100;
      }
      else
      {
        NSLEEP_D_ENABLE( GPIO_PIN_RESET );
        channelPulse = 0;
      }

      if( pwmStatus == BSP_OK )
      {
        BspPwmPulseValueSet( &INCDPwmhandle, IN2_D_CHANNEL, channelPulse );
        pwmStatus = BspPwmHandleStart( &INCDPwmhandle, IN2_D_CHANNEL );
        if( pwmStatus == BSP_OK )
        {
          tecStatus = TEC_OK;
        }
        else
        {
          return HAL_PWM_ERR;
        }
      }
      else
      {
        return HAL_PWM_ERR;
      }
    }
    break;

    default:
    {
      tecStatus = TEC_CHANNEL_ERR;
    }
    break;
  }

  return tecStatus;

}
/*
*********************************************************************************************************
*                                          TecControl
*
* Description :TecControl() 打开或关闭一个通道的pwm输出
*
* Arguments   : TecChannel TEC通道
				EnableFlag 使能标志 true:打开 false:关闭
				percent 百分比，范围:0~100

*
* Returns     : TecStatusDef 错误类型
*
* Notes       : 动态调整，不需要关闭定时器, 打开某个通道时，必须要先关闭相对通道，否则pwm不会输出
*********************************************************************************************************
*/
TecStatusDef TecControl( TecChannelDef TecChannel, bool EnableFlag, u8 percent )
{
  u32 channelPulse;
  TecStatusDef tecStatus = HAL_PWM_ERR;
  BspStatusTypeDef pwmStatus;

  if( percent > 100 )
  {
    return TEC_PERCENT_ERR;
  }


  switch( TecChannel )
  {
    case IN_1_A:
    {
      pwmStatus = BSP_OK;

      if( EnableFlag == true )
      {
        NSLEEP_A_ENABLE( GPIO_PIN_SET );

        channelPulse = percent * TIM2_PERIOD_VALUE / 100;

        pwmStatus = BspPwmHandleStop( &INABPwmhandle, IN2_A_CHANNEL );

        if( pwmStatus == BSP_OK )
        {
          BspPwmPulseValueSet( &INABPwmhandle, IN1_A_CHANNEL, channelPulse );
          pwmStatus = BspPwmHandleStart( &INABPwmhandle, IN1_A_CHANNEL );
          if( pwmStatus == BSP_OK )
          {
            tecStatus = TEC_OK;
          }
          else
          {
            return HAL_PWM_ERR;
          }
        }
        else
        {
          return HAL_PWM_ERR;
        }
      }
      else
      {
        NSLEEP_A_ENABLE( GPIO_PIN_RESET );
        channelPulse = 0;
        pwmStatus = BspPwmHandleStop( &INABPwmhandle, IN1_A_CHANNEL );
        if( pwmStatus == BSP_OK )
        {
          tecStatus = TEC_OK;

        }
        else
        {
          return HAL_PWM_ERR;
        }
      }

    }
    break;
		
    case IN_2_A:
    {
      pwmStatus = BSP_OK;

      if( EnableFlag == true )
      {
        NSLEEP_A_ENABLE( GPIO_PIN_SET );
        channelPulse = percent * TIM2_PERIOD_VALUE / 100;

        pwmStatus = BspPwmHandleStop( &INABPwmhandle, IN1_A_CHANNEL );

        if( pwmStatus == BSP_OK )
        {
          BspPwmPulseValueSet( &INABPwmhandle, IN2_A_CHANNEL, channelPulse );
          pwmStatus = BspPwmHandleStart( &INABPwmhandle, IN2_A_CHANNEL );
          if( pwmStatus == BSP_OK )
          {
            tecStatus = TEC_OK;
          }
          else
          {
            return HAL_PWM_ERR;
          }
        }
        else
        {
          return HAL_PWM_ERR;
        }
      }
      else
      {
        NSLEEP_A_ENABLE( GPIO_PIN_RESET );
        channelPulse = 0;
        pwmStatus = BspPwmHandleStop( &INABPwmhandle, IN2_A_CHANNEL );
        if( pwmStatus == BSP_OK )
        {
          tecStatus = TEC_OK;

        }
        else
        {
          return HAL_PWM_ERR;
        }
      }

    }
    break;
		
    case IN_1_B:
    {
      pwmStatus = BSP_OK;

      if( EnableFlag == true )
      {
        NSLEEP_B_ENABLE( GPIO_PIN_SET );
        pwmStatus = BspPwmHandleStop( &INABPwmhandle, IN2_B_CHANNEL );
        channelPulse = percent * TIM2_PERIOD_VALUE / 100;
        if( pwmStatus == BSP_OK )
        {
          BspPwmPulseValueSet( &INABPwmhandle, IN1_B_CHANNEL, channelPulse );
          pwmStatus = BspPwmHandleStart( &INABPwmhandle, IN1_B_CHANNEL );
          if( pwmStatus == BSP_OK )
          {
            tecStatus = TEC_OK;
          }
          else
          {
            return HAL_PWM_ERR;
          }
        }
        else
        {
          return HAL_PWM_ERR;
        }
      }
      else
      {
        NSLEEP_B_ENABLE( GPIO_PIN_RESET );
        channelPulse = 0;
        pwmStatus = BspPwmHandleStop( &INABPwmhandle, IN1_B_CHANNEL );
        if( pwmStatus == BSP_OK )
        {
          tecStatus = TEC_OK;

        }
        else
        {
          return HAL_PWM_ERR;
        }
      }


    }
    break;
		
    case IN_2_B:
    {
      pwmStatus = BSP_OK;

      if( EnableFlag == true )
      {
        NSLEEP_B_ENABLE( GPIO_PIN_SET );
        pwmStatus = BspPwmHandleStop( &INABPwmhandle, IN1_B_CHANNEL );
        channelPulse = percent * TIM2_PERIOD_VALUE / 100;
        if( pwmStatus == BSP_OK )
        {
          BspPwmPulseValueSet( &INABPwmhandle, IN2_B_CHANNEL, channelPulse );
          pwmStatus = BspPwmHandleStart( &INABPwmhandle, IN2_B_CHANNEL );
          if( pwmStatus == BSP_OK )
          {
            tecStatus = TEC_OK;
          }
          else
          {
            return HAL_PWM_ERR;
          }
        }
        else
        {
          return HAL_PWM_ERR;
        }
      }
      else
      {
        NSLEEP_B_ENABLE( GPIO_PIN_RESET );
        pwmStatus = BspPwmHandleStop( &INABPwmhandle, IN2_B_CHANNEL );
        channelPulse = 0;
        if( pwmStatus == BSP_OK )
        {
          tecStatus = TEC_OK;

        }
        else
        {
          return HAL_PWM_ERR;
        }
      }


    }
    break;
		
    case IN_1_C:
    {
      pwmStatus = BSP_OK;

      if( EnableFlag == true )
      {
        NSLEEP_C_ENABLE( GPIO_PIN_SET );
        pwmStatus = BspPwmHandleStop( &INCDPwmhandle, IN2_C_CHANNEL );
        channelPulse = percent * TIM3_PERIOD_VALUE / 100;
        if( pwmStatus == BSP_OK )
        {
          BspPwmPulseValueSet( &INCDPwmhandle, IN1_C_CHANNEL, channelPulse );
          pwmStatus = BspPwmHandleStart( &INCDPwmhandle, IN1_C_CHANNEL );
          if( pwmStatus == BSP_OK )
          {
            tecStatus = TEC_OK;
          }
          else
          {
            return HAL_PWM_ERR;
          }
        }
        else
        {
          return HAL_PWM_ERR;
        }
      }
      else
      {
        NSLEEP_C_ENABLE( GPIO_PIN_RESET );
        channelPulse = 0;
        pwmStatus = BspPwmHandleStop( &INCDPwmhandle, IN1_C_CHANNEL );
        if( pwmStatus == BSP_OK )
        {
          tecStatus = TEC_OK;

        }
        else
        {
          return HAL_PWM_ERR;
        }
      }
    }
    break;
		
    case IN_2_C:
    {
      pwmStatus = BSP_OK;

      if( EnableFlag == true )
      {
        NSLEEP_C_ENABLE( GPIO_PIN_SET );
        pwmStatus = BspPwmHandleStop( &INCDPwmhandle, IN1_C_CHANNEL );
        channelPulse = percent * TIM3_PERIOD_VALUE / 100;
        if( pwmStatus == BSP_OK )
        {
          BspPwmPulseValueSet( &INCDPwmhandle, IN2_C_CHANNEL, channelPulse );
          pwmStatus = BspPwmHandleStart( &INCDPwmhandle, IN2_C_CHANNEL );
          if( pwmStatus == BSP_OK )
          {
            tecStatus = TEC_OK;
          }
          else
          {
            return HAL_PWM_ERR;
          }
        }
        else
        {
          return HAL_PWM_ERR;
        }
      }
      else
      {
        NSLEEP_C_ENABLE( GPIO_PIN_RESET );
        channelPulse = 0;
        pwmStatus = BspPwmHandleStop( &INCDPwmhandle, IN2_C_CHANNEL );
        channelPulse = 0;
        if( pwmStatus == BSP_OK )
        {
          tecStatus = TEC_OK;

        }
        else
        {
          return HAL_PWM_ERR;
        }
      }
    }
    break;
		
    case IN_1_D:
    {
      pwmStatus = BSP_OK;

      if( EnableFlag == true )
      {
        NSLEEP_D_ENABLE( GPIO_PIN_SET );
        pwmStatus = BspPwmHandleStop( &INCDPwmhandle, IN2_D_CHANNEL );
        channelPulse = percent * TIM3_PERIOD_VALUE / 100;
        if( pwmStatus == BSP_OK )
        {
          BspPwmPulseValueSet( &INCDPwmhandle, IN1_D_CHANNEL, channelPulse );
          pwmStatus = BspPwmHandleStart( &INCDPwmhandle, IN1_D_CHANNEL );
          if( pwmStatus == BSP_OK )
          {
            tecStatus = TEC_OK;
          }
          else
          {
            return HAL_PWM_ERR;
          }
        }
        else
        {
          return HAL_PWM_ERR;
        }
      }
      else
      {
        NSLEEP_D_ENABLE( GPIO_PIN_RESET );
        channelPulse = 0;
        pwmStatus = BspPwmHandleStop( &INCDPwmhandle, IN1_D_CHANNEL );
        channelPulse = 0;
        if( pwmStatus == BSP_OK )
        {
          tecStatus = TEC_OK;

        }
        else
        {
          return HAL_PWM_ERR;
        }
      }

    }
    break;
		
    case IN_2_D:
    {
      pwmStatus = BSP_OK;

      if( EnableFlag == true )
      {
        NSLEEP_D_ENABLE( GPIO_PIN_SET );
        pwmStatus = BspPwmHandleStop( &INCDPwmhandle, IN1_D_CHANNEL );
        channelPulse = percent * TIM3_PERIOD_VALUE / 100;
        if( pwmStatus == BSP_OK )
        {
          BspPwmPulseValueSet( &INCDPwmhandle, IN2_D_CHANNEL, channelPulse );
          pwmStatus = BspPwmHandleStart( &INCDPwmhandle, IN2_D_CHANNEL );
          if( pwmStatus == BSP_OK )
          {
            tecStatus = TEC_OK;
          }
          else
          {
            return HAL_PWM_ERR;
          }
        }
        else
        {
          return HAL_PWM_ERR;
        }
      }
      else
      {
        NSLEEP_D_ENABLE( GPIO_PIN_RESET );
        channelPulse = 0;
        pwmStatus = BspPwmHandleStop( &INCDPwmhandle, IN2_D_CHANNEL );
        channelPulse = 0;
        if( pwmStatus == BSP_OK )
        {
          tecStatus = TEC_OK;

        }
        else
        {
          return HAL_PWM_ERR;
        }
      }

    }
    break;

    default:
    {
      tecStatus = TEC_CHANNEL_ERR;
    }
    break;
  }

  return tecStatus;

}


void TecTest( void )
{
  u8 i;
  for( i = 1; i < 100; i ++ )
  {
    TecControl( IN_1_A, true, i );

    TecControl( IN_1_B, true, i );

    TecControl( IN_1_C, true, i );

    TecControl( IN_1_D, true, i );

  }
  for( i = 1; i < 100; i ++ )
  {

    TecControl( IN_2_A, true, i );

    TecControl( IN_2_B, true, i );

    TecControl( IN_2_C, true, i );

    TecControl( IN_2_D, true, i );
  }
}


