/**
  ******************************************************************************
  * @file    bsp_common.c
  * @author  Firmware-Team
  * @version V1.0.0
  * @date    13-06-2016
  * @brief   
  ******************************************************************************
**/
/*
*********************************************************************************************************
*                                             INCLUDE FILES
*********************************************************************************************************
*/
#include "includes.h"
#include  <bsp_common.h>

/*
*********************************************************************************************************
*                                            LOCAL DEFINES
*********************************************************************************************************
*/
#define  BSP_BIT_RCC_PLLCFGR_PLLM               8u
#define  BSP_BIT_RCC_PLLCFGR_PLLN              360u
#define  BSP_BIT_RCC_PLLCFGR_PLLP                2u
#define  BSP_BIT_RCC_PLLCFGR_PLLQ                4u


/*
*********************************************************************************************************
*                                      LOCAL FUNCTION PROTOTYPES
*********************************************************************************************************
*/

/*
*********************************************************************************************************
* void BSP_Init (void)
* Description : Initialize the Board Support Package (BSP).
*********************************************************************************************************
*/
void BSP_Init (void)
{
//	u32 hclk_freq;
	
	BSP_RCC_PLL_Config();

   if (HAL_GetREVID() == 0x1001){
		__HAL_FLASH_PREFETCH_BUFFER_ENABLE();
    }
	
	BSP_MspInit();
    //BSP_LED_Init();
	//BSP_Beep_Init();

	
	
}

/*
*********************************************************************************************************
*                                            BSP_RCC_init()
* Description : Initialize the Board clock.
* Return(s)   : 
*********************************************************************************************************
*/
/**
  * @brief  System Clock Configuration
  *        
  * @param  None
  * @retval None
  */
void BSP_RCC_PLL_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;
  

  RCC_OscInitStruct.OscillatorType	= RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState				= RCC_HSE_ON;

  RCC_OscInitStruct.PLL.PLLState		= RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource 	= RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL 			= RCC_PLL_MUL3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct)!= HAL_OK)
  {
    while(1); 
  }
    	

  RCC_ClkInitStruct.ClockType 			= (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource 		= RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider		= RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider	= RCC_HCLK_DIV2;  
  RCC_ClkInitStruct.APB2CLKDivider 	= RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2)!= HAL_OK)
  {
    while(1); 
  }
}
/*
*********************************************************************************************************
*                                            HAL_InitTick()
* Description : This function has been overwritten from the STM32F4xx HAL libraries because Micrium's RTOS
*               has its own Systick initialization and because it is recomended to initialize the tick after
*               multi-tasking has started.
* Argument(s) : TickPriority          Tick interrupt priority.
* Return(s)   : HAL_OK.
* Note(s)     : none.
*********************************************************************************************************
*/
HAL_StatusTypeDef HAL_InitTickOld(uint32_t TickPriority)
{

    BSP_Tick_Init();

    return (HAL_OK);
}

/*
*********************************************************************************************************
*                                            BSP_Tick_Init()
* Description : Initialize all the peripherals that required OS Tick services (OS initialized)

*********************************************************************************************************
*/
void BSP_Tick_Init (void)
{
}

/*
*********************************************************************************************************
*                                           BSP_Beep_Init()
* Description : Initialize the beep on the board.
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*                                           BSP_Beep_Init()
* Description : turn on the beep on the board for specified time
* Argument(s) : ontime(mS), The time of the beep to on
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                           BSP_LED_Init()
* Description : Initialize any or all the LEDs on the board.
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                             BSP_LED_On()
* Description : Turn ON any or all the LEDs on the board.
* Argument(s) : led     The ID of the LED to control:
*                       BSP_GPIOG_LED_ALL    	turns ON ALL  LEDs
*                       BSP_GPIOG_LED_RED    	turns ON RED LED1
*                       BSP_GPIOI_LED_YELLOW    turns ON YELLOW LED2
*********************************************************************************************************
*/
void  BSP_LED_On (CPU_INT08U  led)
{
    switch (led) {
        case BSP_GPIOG_LED_ALL:
             HAL_GPIO_WritePin(GPIOF, GPIO_PIN_9|GPIO_PIN_10, GPIO_PIN_SET);
             break;
        case BSP_GPIOG_LED_RED:
             HAL_GPIO_WritePin(GPIOF, GPIO_PIN_9, GPIO_PIN_SET);
             break;
        case BSP_GPIOI_LED_YELLOW:
             HAL_GPIO_WritePin(GPIOF, GPIO_PIN_10, GPIO_PIN_SET);
             break;
        default:
             break;
    }
}

/*
*********************************************************************************************************
*                                              BSP_LED_Off()
* Description : Turn OFF any or all the LEDs on the board.
* Argument(s) : led     The ID of the LED to control:
*                       BSP_GPIOG_LED_ALL    	turns ON ALL  LEDs
*                       BSP_GPIOG_LED_RED    	turns ON RED LED1
*                       BSP_GPIOI_LED_YELLOW    turns ON YELLOW LED2
*********************************************************************************************************
*/
void  BSP_LED_Off (CPU_INT08U led)
{
    switch (led) {
        case BSP_GPIOG_LED_ALL:
             HAL_GPIO_WritePin(GPIOF, GPIO_PIN_9|GPIO_PIN_10, GPIO_PIN_RESET);
             break;
        case BSP_GPIOG_LED_RED:
             HAL_GPIO_WritePin(GPIOF, GPIO_PIN_9, GPIO_PIN_RESET);
             break;
        case BSP_GPIOI_LED_YELLOW:
             HAL_GPIO_WritePin(GPIOF, GPIO_PIN_10, GPIO_PIN_RESET);
             break;
        default:
             break;
    }
}

/*
*********************************************************************************************************
*                                            BSP_LED_Toggle()
* Description : TOGGLE any or all the LEDs on the board.
* Argument(s) : led     The ID of the LED to control:
*                       BSP_GPIOG_LED_ALL    	turns ON ALL  LEDs
*                       BSP_GPIOG_LED_RED    	turns ON RED LED1
*                       BSP_GPIOI_LED_YELLOW    turns ON YELLOW LED2
*********************************************************************************************************
*/
void  BSP_LED_Toggle (CPU_INT08U  led)
{
    switch (led) {
        case BSP_GPIOG_LED_ALL:
             HAL_GPIO_TogglePin(GPIOF, GPIO_PIN_9|GPIO_PIN_10);
             break;
        case BSP_GPIOG_LED_RED:
             HAL_GPIO_TogglePin(GPIOF, GPIO_PIN_9);
             break;
        case BSP_GPIOI_LED_YELLOW:
             HAL_GPIO_TogglePin(GPIOF, GPIO_PIN_10);
             break;
        default:
             break;
    }
}
