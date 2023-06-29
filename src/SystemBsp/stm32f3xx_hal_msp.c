/**
  ******************************************************************************
  * @file    stm32f3xx_hal_msp.c
  * @author  Firmware-Team
  * @version V1.0.1
  * @date    13-06-2016
  * @brief   
  ******************************************************************************
**/

/* Includes ------------------------------------------------------------------*/
#include "includes.h"
#include "stm32f3xx_hal.h"
#include "stm32f3xx_hal_msp.h"
#include <stdarg.h>

/*
*********************************************************************************************************
*                                               INT DEFINES
*********************************************************************************************************
*/



/* SPI4*/
#define SPI4_HAL_PORT                    SPI4
#define SPI4_CLK_ENABLE()                __HAL_RCC_SPI4_CLK_ENABLE()
#define SPI4_SCK_GPIO_CLK_ENABLE()       __HAL_RCC_GPIOE_CLK_ENABLE()
#define SPI4_MISO_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOE_CLK_ENABLE() 
#define SPI4_MOSI_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOE_CLK_ENABLE() 

#define SPI4_FORCE_RESET()               __HAL_RCC_SPI4_FORCE_RESET()
#define SPI4_RELEASE_RESET()             __HAL_RCC_SPI4_RELEASE_RESET()


#define SPI4_SCK_PIN                     GPIO_PIN_2
#define SPI4_SCK_GPIO_PORT               GPIOE
#define SPI4_SCK_AF                      GPIO_AF5_SPI4
#define SPI4_MISO_PIN                    GPIO_PIN_5
#define SPI4_MISO_GPIO_PORT              GPIOE
#define SPI4_MISO_AF                     GPIO_AF5_SPI4
#define SPI4_MOSI_PIN                    GPIO_PIN_6
#define SPI4_MOSI_GPIO_PORT              GPIOE
#define SPI4_MOSI_AF                     GPIO_AF5_SPI4


#define SPI4_HAL_IRQn                    SPI4_IRQn





/*i2c3************************************************************************/

#define I2C3_HAL_PORT                    I2C3
#define I2C3_CLK_ENABLE()                __HAL_RCC_I2C3_CLK_ENABLE()
#define I2C3_SDA_GPIO_CLK_ENABLE()       __HAL_RCC_GPIOH_CLK_ENABLE()
#define I2C3_SCL_GPIO_CLK_ENABLE()       __HAL_RCC_GPIOA_CLK_ENABLE() 

#define I2C3_FORCE_RESET()               __HAL_RCC_I2C3_FORCE_RESET()
#define I2C3_RELEASE_RESET()             __HAL_RCC_I2C3_RELEASE_RESET()
#define I2C3_SCL_PIN                    GPIO_PIN_8
#define I2C3_SCL_GPIO_PORT              GPIOA
#define I2C3_SCL_AF                     GPIO_AF4_I2C3
#define I2C3_SDA_PIN                    GPIO_PIN_8
#define I2C3_SDA_GPIO_PORT              GPIOH
#define I2C3_SDA_AF                     GPIO_AF4_I2C3
#define I2C3_EV_IRQn                    I2C3_EV_IRQn
#define I2C3_EV_IRQHandler              I2C3_EV_IRQHandler
#define I2C3_ER_IRQn                    I2C3_ER_IRQn
#define I2C3_ER_IRQHandler              I2C3_ER_IRQHandler



#define UART6_PREEMPT_PRIORITY							1
#define UART6_SUB_PRIORITY								0



#define UART7_DMA1_CHA1_PREEMPT_PRIORITY				3
#define UART7_DMA1_CHA1_SUB_PRIORITY					2
#define UART7_DMA1_CHA3_PREEMPT_PRIORITY				3
#define UART7_DMA1_CHA3_SUB_PRIORITY					1



#define SPI4_PREEMPT_PRIORITY							2
#define SPI4_SUB_PRIORITY								1


#define I2C3_ER_PREEMPT_PRIORITY						0
#define I2C3_ER_SUB_PRIORITY							2
#define I2C3_EV_PREEMPT_PRIORITY						0
#define I2C3_EV_SUB_PRIORITY							3


#define EXIT_PA4_PREEMPT_PRIORITY						1
#define EXIT_PA4_SUB_PRIORITY							0
#define EXIT_PH6_PREEMPT_PRIORITY						1
#define EXIT_PH6_SUB_PRIORITY							0








/*debug info define*************************************************************************************/
#define DEBUG_BUFF_SIZE 				1024


/*
*********************************************************************************************************
*                                        LOCAL GLOBAL VARIABLES
*********************************************************************************************************
*/


s8 DebugDataTemp[DEBUG_BUFF_SIZE];




/*
*********************************************************************************************************
*                                           FUNCTION PROTOTYPES
*********************************************************************************************************
*/



void SaveLog(const char *pFormat, ...)
{
	;
}

void BSP_MspInit(void)
{
#if BSP_USART1_EN
	Uart1Config();  
#endif

#if BSP_USART6_EN
	Uart6Config();
#endif
#if BSP_USART7_EN
	Uart7Config();
#endif

#if SPI2_EN
	Spi2Config();
#endif
	
#if SPI4_EN
	Spi4Config();
#endif

#if SPI5_EN
	Spi5Config();
#endif

#if TIM1_EN
	Pwm1Config();
#endif

#if TIM3_EN
	Tim3Config();
#endif

#if EXIT4_EN
	EXTILine4_Config();
#endif
		
#if EXIT9_5_EN
	EXTILine9_5_Config();
#endif

#if I2C1_EN
	I2C1Config();
#endif

#if I2C3_EN
	I2C3Config();
#endif

#if CRC_EN
	crcInit();
#endif
}


void HAL_UART_MspInit(UART_HandleTypeDef* huart)
{
  	GPIO_InitTypeDef GPIO_InitStruct;
	BspUartHandle *pUartHandle;
	
	pUartHandle = PortMapGet(AllPortMap,PROT_NUM,huart->Instance);
	if(pUartHandle == NULL)
	{
		return;
	}


	pUartHandle->UartClkSet.ftTxGpioClkEnable();
	

	pUartHandle->UartClkSet.ftRxGpioClkEnable();
	

	pUartHandle->UartClkSet.ftPortClkEnable();


	GPIO_InitStruct.Pin       = pUartHandle->TxGpioInit.Pin;
	GPIO_InitStruct.Mode      = pUartHandle->TxGpioInit.Mode;
	GPIO_InitStruct.Pull      = pUartHandle->TxGpioInit.Pull;
	GPIO_InitStruct.Speed     = pUartHandle->TxGpioInit.Speed;
	GPIO_InitStruct.Alternate = pUartHandle->TxGpioInit.Alternate;

	HAL_GPIO_Init(pUartHandle->UartPortInit.TxGpioPort, &GPIO_InitStruct);


	GPIO_InitStruct.Pin 	    = pUartHandle->RxGpioInit.Pin;
	GPIO_InitStruct.Mode      = pUartHandle->RxGpioInit.Mode;//GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull      = pUartHandle->RxGpioInit.Pull;//GPIO_NOPULL;
	GPIO_InitStruct.Speed     = pUartHandle->RxGpioInit.Speed;//GPIO_SPEED_FAST;
	GPIO_InitStruct.Alternate = pUartHandle->RxGpioInit.Alternate;//USART1_RX_AF;

	HAL_GPIO_Init(pUartHandle->UartPortInit.RxGpioPort, &GPIO_InitStruct);


	HAL_NVIC_SetPriority(pUartHandle->IrqInit.Irqn, pUartHandle->IrqInit.PreemptPriority, pUartHandle->IrqInit.SubPriority);
	HAL_NVIC_EnableIRQ(pUartHandle->IrqInit.Irqn);
}


void HAL_UART_MspDeInit(UART_HandleTypeDef* huart)
{
	BspUartHandle *pUartHandle;
	
	pUartHandle = PortMapGet(AllPortMap,PROT_NUM,huart->Instance);
	if(pUartHandle == NULL)
	{
		return;
	}

	pUartHandle->UartClkSet.ftPortForceReset();
	

	pUartHandle->UartClkSet.ftPortReleaseReset();


	HAL_GPIO_DeInit(pUartHandle->UartPortInit.TxGpioPort, pUartHandle->TxGpioInit.Pin);
	HAL_GPIO_DeInit(pUartHandle->UartPortInit.RxGpioPort, pUartHandle->RxGpioInit.Pin);

	HAL_NVIC_DisableIRQ(pUartHandle->IrqInit.Irqn);
}



/**
  * @brief  Tx Transfer completed callback
  * @param  UartHandle: UART handle. 
  * @note   
  *       
  * @retval None
  */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *UartHandle)
{
	HAL_StatusTypeDef status;
	BspUartHandle *pUartHandle;
	
	pUartHandle = PortMapGet(AllPortMap,PROT_NUM,UartHandle->Instance);
	if(pUartHandle == NULL)
	{
		return;
	}
	pUartHandle->UartFlag.TxCpltFlag = true;
	if(isQueueEmpty(pUartHandle->UartQuque.TxQueue) == false)
	{
		pUartHandle->TxData = deQueue(pUartHandle->UartQuque.TxQueue);
		if(HAL_UART_GetError(UartHandle) == HAL_UART_ERROR_NONE)
		{
			status = HAL_UART_Transmit_IT(UartHandle,&pUartHandle->TxData,1);
			if(status == HAL_ERROR)
			{
				pUartHandle->UartFlag.TxHalErrorFlag = true;
			}
			pUartHandle->UartFlag.TxCpltFlag = false;
		}
		else
		{

		}
	}
	
	if(HAL_UART_GetState(UartHandle) == HAL_UART_STATE_READY)
	{
		status = HAL_UART_Receive_IT(UartHandle,&pUartHandle->RxData,1);
		if(status != HAL_OK)
		{
			pUartHandle->UartFlag.RxHalErrorFlag = true;
		}
	}


}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
	HAL_StatusTypeDef status;
	BspUartHandle *pUartHandle;
	
	pUartHandle = PortMapGet(AllPortMap,PROT_NUM,UartHandle->Instance);
	if(pUartHandle == NULL)
	{
		return;
	}

	if(isQueueFull(pUartHandle->UartQuque.RxQueue) == false)
	{
		enQueue(pUartHandle->UartQuque.RxQueue,pUartHandle->RxData);
	}
	else
	{
		pUartHandle->UartFlag.RxQueueOverFlag = true;
	}
	
	if(HAL_UART_GetError(UartHandle) == HAL_UART_ERROR_NONE)
	{
		status = HAL_UART_Receive_IT(UartHandle,&pUartHandle->RxData,1);
		if(status != HAL_OK)
		{
			pUartHandle->UartFlag.RxHalErrorFlag = true;
		}
	}
	else
	{
		pUartHandle->UartFlag.RxHalErrorFlag = true;
	}
		
}


/**
  * @brief  UART error callbacks
  * @param  UartHandle: UART handle
  * @note   
  *         
  * @retval None
  */
 void HAL_UART_ErrorCallback(UART_HandleTypeDef *UartHandle)
{
	BspUartHandle *pUartHandle;
	
	pUartHandle = PortMapGet(AllPortMap,PROT_NUM,UartHandle->Instance);
	if(pUartHandle == NULL)
	{
		return;
	}

	if(HAL_UART_GetError(UartHandle) == HAL_UART_ERROR_ORE)
	{
		pUartHandle->UartFlag.RxQueueOverFlag = true;
		//HAL_UART_Receive(UartHandle,&pUartHandle->RxData,1,1);
		if(HAL_UART_Receive_IT(UartHandle,&pUartHandle->RxData,1) != HAL_OK)
		{
			SaveLog(NULL,0);
		}
		//pUartHandle->UartFlag.TxCpltFlag = true;
	}
  	else if(UartHandle->ErrorCode != HAL_UART_ERROR_NONE)
	{
		HAL_UART_MspDeInit(UartHandle);
		UartHandle->gState = HAL_UART_STATE_RESET;
		if(HAL_UART_Init(UartHandle) != HAL_OK)
		{
			SaveLog(NULL,0);
		}
		else
		{
		    HAL_UART_Receive_IT(UartHandle,&pUartHandle->RxData,1);
			pUartHandle->UartFlag.TxCpltFlag = true;
		}
	}

}



void Debug_Printf(const char *pFormat, ...)
{
#if DEBUG_EN

	u32  len;
	//CPU_SR_ALLOC();
	//OS_ERR err;
	va_list ap;
	
	va_start(ap, pFormat);
	len = vsprintf((char *)DebugDataTemp, pFormat, ap);
	va_end(ap);

	if(len > 0 && len < DEBUG_BUFF_SIZE)
	{
		BspUartTransmitIT(&BspUart3Handle,(u8 *)DebugDataTemp,len);
	}
	
#endif
}


/**
  * @brief SPI MSP Initialization 
  *        This function configures the hardware resources used in this example: 
  *           - Peripheral's clock enable
  *          
  * @param hspi: SPI handle pointer
  * @retval None
  */
void HAL_SPI_MspInit(SPI_HandleTypeDef *hspi)
{
	BspSpiHandle *pSpiHandle;
	GPIO_InitTypeDef  GPIO_InitStruct;
	
	pSpiHandle = PortMapGet(AllPortMap,PROT_NUM,hspi->Instance);
	if(pSpiHandle == NULL)
	{
		return;
	}
	

	pSpiHandle->SpiClkSet.ftSckGpioClkEnable();
	//SPI2_MISO_GPIO_CLK_ENABLE();
	pSpiHandle->SpiClkSet.ftMisoGpioClkEnable();
	//SPI2_MOSI_GPIO_CLK_ENABLE();
	pSpiHandle->SpiClkSet.ftMosiGpioClkEnable();

	if(hspi->Init.NSS != SPI_NSS_SOFT)
	{
		pSpiHandle->SpiClkSet.ftCsGpioClkEnable();
	}

	//SPI2_CLK_ENABLE(); 
	pSpiHandle->SpiClkSet.ftPortClkEnable();

	GPIO_InitStruct.Pin       = pSpiHandle->SckGpioInit.Pin;//SPI2_SCK_PIN;
	GPIO_InitStruct.Mode      = pSpiHandle->SckGpioInit.Mode;//GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull      = pSpiHandle->SckGpioInit.Pull;//GPIO_PULLUP;
	GPIO_InitStruct.Speed     = pSpiHandle->SckGpioInit.Speed;//GPIO_SPEED_FAST;
	GPIO_InitStruct.Alternate = pSpiHandle->SckGpioInit.Alternate;//SPI2_SCK_AF;

	HAL_GPIO_Init(pSpiHandle->SpiPortInit.SckGpioPort, &GPIO_InitStruct);

	GPIO_InitStruct.Pin 	  = pSpiHandle->MisoGpioInit.Pin;
	GPIO_InitStruct.Mode      = pSpiHandle->MisoGpioInit.Mode;
	GPIO_InitStruct.Pull      = pSpiHandle->MisoGpioInit.Pull;
	GPIO_InitStruct.Speed     = pSpiHandle->MisoGpioInit.Speed;
	GPIO_InitStruct.Alternate = pSpiHandle->MisoGpioInit.Alternate;

	HAL_GPIO_Init(pSpiHandle->SpiPortInit.MisoGpioPort, &GPIO_InitStruct);

	GPIO_InitStruct.Pin 	  = pSpiHandle->MosiGpioInit.Pin;
	GPIO_InitStruct.Mode      = pSpiHandle->MosiGpioInit.Mode;
	GPIO_InitStruct.Pull      = pSpiHandle->MosiGpioInit.Pull;
	GPIO_InitStruct.Speed     = pSpiHandle->MosiGpioInit.Speed;
	GPIO_InitStruct.Alternate = pSpiHandle->MosiGpioInit.Alternate;
	HAL_GPIO_Init(pSpiHandle->SpiPortInit.MosiGpioPort, &GPIO_InitStruct);

	if(hspi->Init.NSS != SPI_NSS_SOFT)
	{
		GPIO_InitStruct.Pin 	  = pSpiHandle->CsGpioInit.Pin;
		GPIO_InitStruct.Mode      = pSpiHandle->CsGpioInit.Mode;
		GPIO_InitStruct.Pull      = pSpiHandle->CsGpioInit.Pull;
		GPIO_InitStruct.Speed     = pSpiHandle->CsGpioInit.Speed;
		GPIO_InitStruct.Alternate = pSpiHandle->CsGpioInit.Alternate;
		HAL_GPIO_Init(pSpiHandle->SpiPortInit.CsGpioPort, &GPIO_InitStruct);
	}

	HAL_NVIC_SetPriority(pSpiHandle->IrqInit.Irqn, pSpiHandle->IrqInit.PreemptPriority, pSpiHandle->IrqInit.SubPriority);
	HAL_NVIC_EnableIRQ(pSpiHandle->IrqInit.Irqn);
}

/**
  * @brief SPI MSP De-Initialization 
  *        This function frees the hardware resources used in this example:
  *          - Disable the Peripheral's clock
  *   
  * @param hspi: SPI handle pointer
  * @retval None
  */
void HAL_SPI_MspDeInit(SPI_HandleTypeDef *hspi)
{

	BspSpiHandle *pSpiHandle;
	
	pSpiHandle = PortMapGet(AllPortMap,PROT_NUM,hspi->Instance);
	if(pSpiHandle == NULL)
	{
		return;
	}


	pSpiHandle->SpiClkSet.ftPortForceReset();
	//SPI2_RELEASE_RESET();
	pSpiHandle->SpiClkSet.ftPortReleaseReset();

	//HAL_GPIO_DeInit(SPI2_SCK_GPIO_PORT, SPI2_SCK_PIN);
	HAL_GPIO_DeInit(pSpiHandle->SpiPortInit.SckGpioPort, pSpiHandle->SckGpioInit.Pin);

	HAL_GPIO_DeInit(pSpiHandle->SpiPortInit.MisoGpioPort, pSpiHandle->MisoGpioInit.Pin);

	HAL_GPIO_DeInit(pSpiHandle->SpiPortInit.MosiGpioPort, pSpiHandle->MosiGpioInit.Pin);
	
	if(hspi->Init.NSS != SPI_NSS_SOFT)
	{
		HAL_GPIO_DeInit(pSpiHandle->SpiPortInit.MosiGpioPort, pSpiHandle->CsGpioInit.Pin);
	}

	HAL_NVIC_DisableIRQ(pSpiHandle->IrqInit.Irqn);
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
	BspSpiHandle *pSpiHandle;
	
	pSpiHandle = PortMapGet(AllPortMap,PROT_NUM,hspi->Instance);
	if(pSpiHandle == NULL)
	{
		return;
	}

	pSpiHandle->SpiFlag.TxCpltFlag = true;
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
	BspSpiHandle *pSpiHandle;
	
	pSpiHandle = PortMapGet(AllPortMap,PROT_NUM,hspi->Instance);
	if(pSpiHandle == NULL)
	{
		return;
	}
	pSpiHandle->SpiFlag.RxCpltFlag = true;
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
	BspSpiHandle *pSpiHandle;
	
	pSpiHandle = PortMapGet(AllPortMap,PROT_NUM,hspi->Instance);
	if(pSpiHandle == NULL)
	{
		return;
	}
	pSpiHandle->SpiFlag.TxCpltFlag = true;
	pSpiHandle->SpiFlag.RxCpltFlag = true;

	if(pSpiHandle->SpiHandleInit.Init.Mode == SPI_MODE_SLAVE)
	{
		if(isQueueFull(pSpiHandle->SpiQuque.RxQueue) != true)
		{
			enQueue(pSpiHandle->SpiQuque.RxQueue,pSpiHandle->RxData);
		}
		else
		{
			pSpiHandle->SpiFlag.RxQueueOverFlag = true;
		}

		if(isQueueEmpty(pSpiHandle->SpiQuque.TxQueue) != true)
		{
			pSpiHandle->TxData = deQueue(pSpiHandle->SpiQuque.TxQueue);
		}
		else
		{
			pSpiHandle->TxData = 0;
		}

		if(BspSpiTransmitReceiveIT(pSpiHandle,&pSpiHandle->TxData,&pSpiHandle->RxData,1) != BSP_OK)
		{
			pSpiHandle->SpiFlag.RxHalErrorFlag = true;
			pSpiHandle->SpiFlag.TxHalErrorFlag = true;
		}
	}
}

/**
  * @brief  SPI error callbacks.
  * @param  hspi: SPI handle
  * @note   
  *         
  * @retval None
  */
 void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
	BspSpiHandle *pSpiHandle;
	
	pSpiHandle = PortMapGet(AllPortMap,PROT_NUM,hspi->Instance);
	if(pSpiHandle == NULL)
	{
		return;
	}
	
	if(hspi->ErrorCode != HAL_SPI_ERROR_NONE)
	{
		//Spi4TxCpltFlagItSet(true);
	}
	pSpiHandle->SpiFlag.TxCpltFlag = true;
	pSpiHandle->SpiFlag.RxCpltFlag = true;
}



/**
  * @brief TIM MSP Initialization 
  *        This function configures the hardware resources used in this example: 
  *           - Peripheral's clock enable
  *           - Peripheral's GPIO Configuration  
  * @param htim: TIM handle pointer
  * @retval None
  */
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim)
{
	BspTimHandle *pTimHandle;
	pTimHandle = PortMapGet(AllPortMap,PROT_NUM,htim->Instance);
	if(pTimHandle == NULL)
	{
		return;
	}

	//TIMx_CLK_ENABLE();
	pTimHandle->TimClkEnable();
	
	//HAL_NVIC_SetPriority(TIMx_IRQn, 0, 1);
	HAL_NVIC_SetPriority(pTimHandle->IrqInit.Irqn, pTimHandle->IrqInit.PreemptPriority, pTimHandle->IrqInit.SubPriority);

	//HAL_NVIC_EnableIRQ(TIMx_IRQn);
	HAL_NVIC_EnableIRQ(pTimHandle->IrqInit.Irqn);
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @param  htim: TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	BspTimHandle *pTimHandle;
	pTimHandle = PortMapGet(AllPortMap,PROT_NUM,htim->Instance);
	if(pTimHandle == NULL)
	{
		return;
	}
	pTimHandle->TimCallBack();
}

/**
  * @brief TIM MSP Initialization 
  *        This function configures the hardware resources used in this example: 
  *           - Peripheral's clock enable
  *           - Peripheral's GPIO Configuration  
  * @param htim: TIM handle pointer
  * @retval None
  */
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim)
{
	GPIO_InitTypeDef   GPIO_InitStruct;

	BspPwmHandle *pPwmHandle;
	pPwmHandle = PortMapGet(AllPortMap,PROT_NUM,htim->Instance);
	if(pPwmHandle == NULL)
	{
		return;
	}
	
	pPwmHandle->TimClkEnable();
	
	if(pPwmHandle->PwmChannel1.EnableFlag == true)
	{
		pPwmHandle->PwmChannel1.GpioClkInit();
		GPIO_InitStruct.Pin = pPwmHandle->PwmChannel1.GpioInit.Pin;//TIM3_GPIO_PIN_CHANNEL1;
		GPIO_InitStruct.Mode = pPwmHandle->PwmChannel1.GpioInit.Mode;//GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = pPwmHandle->PwmChannel1.GpioInit.Pull;//GPIO_PULLDOWN;
		GPIO_InitStruct.Speed = pPwmHandle->PwmChannel1.GpioInit.Speed;//GPIO_SPEED_HIGH;
		GPIO_InitStruct.Alternate = pPwmHandle->PwmChannel1.GpioInit.Alternate;//GPIO_AF2_TIM3;
		HAL_GPIO_Init(pPwmHandle->PwmChannel1.GpioPort, &GPIO_InitStruct);
	}

	if(pPwmHandle->PwmChannel2.EnableFlag == true)
	{
		pPwmHandle->PwmChannel2.GpioClkInit();
		GPIO_InitStruct.Pin = pPwmHandle->PwmChannel2.GpioInit.Pin;
		GPIO_InitStruct.Mode = pPwmHandle->PwmChannel2.GpioInit.Mode;//GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = pPwmHandle->PwmChannel2.GpioInit.Pull;//GPIO_PULLDOWN;
		GPIO_InitStruct.Speed = pPwmHandle->PwmChannel2.GpioInit.Speed;//GPIO_SPEED_HIGH;
		GPIO_InitStruct.Alternate = pPwmHandle->PwmChannel2.GpioInit.Alternate;//GPIO_AF2_TIM3;
		HAL_GPIO_Init(pPwmHandle->PwmChannel2.GpioPort, &GPIO_InitStruct);
	}

	if(pPwmHandle->PwmChannel3.EnableFlag == true)
	{
		pPwmHandle->PwmChannel3.GpioClkInit();
		GPIO_InitStruct.Pin = pPwmHandle->PwmChannel3.GpioInit.Pin;
		GPIO_InitStruct.Mode = pPwmHandle->PwmChannel3.GpioInit.Mode;//GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = pPwmHandle->PwmChannel3.GpioInit.Pull;//GPIO_PULLDOWN;
		GPIO_InitStruct.Speed = pPwmHandle->PwmChannel3.GpioInit.Speed;//GPIO_SPEED_HIGH;
		GPIO_InitStruct.Alternate = pPwmHandle->PwmChannel3.GpioInit.Alternate;//GPIO_AF2_TIM3;
		HAL_GPIO_Init(pPwmHandle->PwmChannel3.GpioPort, &GPIO_InitStruct);
	}

	if(pPwmHandle->PwmChannel4.EnableFlag == true)
	{
		pPwmHandle->PwmChannel4.GpioClkInit();
		GPIO_InitStruct.Pin = pPwmHandle->PwmChannel4.GpioInit.Pin;
		GPIO_InitStruct.Mode = pPwmHandle->PwmChannel4.GpioInit.Mode;//GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = pPwmHandle->PwmChannel4.GpioInit.Pull;//GPIO_PULLDOWN;
		GPIO_InitStruct.Speed = pPwmHandle->PwmChannel4.GpioInit.Speed;//GPIO_SPEED_HIGH;
		GPIO_InitStruct.Alternate = pPwmHandle->PwmChannel4.GpioInit.Alternate;//GPIO_AF2_TIM3;
		HAL_GPIO_Init(pPwmHandle->PwmChannel4.GpioPort, &GPIO_InitStruct);
	}
}

/**
  * @brief EXTI line detection callbacks
  * @param GPIO_Pin: Specifies the pins connected EXTI line
  * @retval None
  */
static void EXTILine4_Config(void)
{
  GPIO_InitTypeDef   GPIO_InitStructure;

  __HAL_RCC_GPIOA_CLK_ENABLE();
  
  GPIO_InitStructure.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStructure.Pull = GPIO_NOPULL;
  GPIO_InitStructure.Pin = GPIO_PIN_4;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);

  HAL_NVIC_SetPriority(EXTI4_IRQn, EXIT_PA4_PREEMPT_PRIORITY, EXIT_PA4_SUB_PRIORITY);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);
}

static void EXTILine9_5_Config(void)
{
  GPIO_InitTypeDef   GPIO_InitStructure;


  __HAL_RCC_GPIOH_CLK_ENABLE();
  
  GPIO_InitStructure.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStructure.Pull = GPIO_NOPULL;
  GPIO_InitStructure.Pin = GPIO_PIN_6;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStructure);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, EXIT_PH6_PREEMPT_PRIORITY, EXIT_PH6_SUB_PRIORITY);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
}

static u32 exit4Count1,exit4Count2;
static u32 exit6Count1,exit6Count2;
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == GPIO_PIN_4)
	{
		if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_4) == GPIO_PIN_RESET)
		{
			exit4Count1++;
		}
		if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_4) == GPIO_PIN_SET)
		{
			exit4Count2++;	  
		}
	}
	if(GPIO_Pin == GPIO_PIN_6)
	{
		if(HAL_GPIO_ReadPin(GPIOH,GPIO_PIN_6) == GPIO_PIN_RESET)
		{
			exit6Count1++;
		}
		if(HAL_GPIO_ReadPin(GPIOH,GPIO_PIN_6) == GPIO_PIN_SET)
		{
			exit6Count2++;	  
		}
	}
}




/**
  * @brief I2C MSP Initialization 
  *        This function configures the hardware resources used in this example: 
  *           - Peripheral's clock enable
  *           - Peripheral's GPIO Configuration  
  * @param hi2c: I2C handle pointer
  * @retval None
  */
void HAL_I2C_MspInit(I2C_HandleTypeDef *hi2c)
{
	BspI2cHandle *pI2cHandle;
	GPIO_InitTypeDef  GPIO_InitStruct;
	
	pI2cHandle = PortMapGet(AllPortMap,PROT_NUM,hi2c->Instance);
	if(pI2cHandle == NULL)
	{
		return;
	}
	

	pI2cHandle->I2cClkSet.ftSclpioClkEnable();
	//I2C1_SDA_GPIO_CLK_ENABLE();
	pI2cHandle->I2cClkSet.ftSdaGpioClkEnable();
	//I2C1_CLK_ENABLE(); 
	pI2cHandle->I2cClkSet.ftPortClkEnable();

	GPIO_InitStruct.Pin       = pI2cHandle->SclGpioInit.Pin;//I2C1_SCL_PIN;
	GPIO_InitStruct.Mode      = pI2cHandle->SclGpioInit.Mode;//GPIO_MODE_AF_OD;
	GPIO_InitStruct.Pull      = pI2cHandle->SclGpioInit.Pull;//GPIO_PULLUP;
	GPIO_InitStruct.Speed     = pI2cHandle->SclGpioInit.Speed;//GPIO_SPEED_FAST;
	GPIO_InitStruct.Alternate = pI2cHandle->SclGpioInit.Alternate;//I2C1_SCL_AF;

	HAL_GPIO_Init(pI2cHandle->I2cPortInit.SclGpioPort, &GPIO_InitStruct);

	GPIO_InitStruct.Pin 	    = pI2cHandle->SdaGpioInit.Pin;//I2C1_SDA_PIN;
	GPIO_InitStruct.Mode      = pI2cHandle->SdaGpioInit.Mode;//GPIO_MODE_AF_OD;
	GPIO_InitStruct.Pull      = pI2cHandle->SdaGpioInit.Pull;//GPIO_PULLUP;
	GPIO_InitStruct.Speed     = pI2cHandle->SdaGpioInit.Speed;//GPIO_SPEED_FAST;
	GPIO_InitStruct.Alternate = pI2cHandle->SdaGpioInit.Alternate;//I2C1_SDA_AF;

	HAL_GPIO_Init(pI2cHandle->I2cPortInit.SdaGpioPort, &GPIO_InitStruct);

	//HAL_NVIC_SetPriority(I2C1_ER_IRQn, I2C1_ER_PREEMPT_PRIORITY, I2C1_ER_SUB_PRIORITY);
	HAL_NVIC_SetPriority(pI2cHandle->IrqInit.ErIrqn, pI2cHandle->IrqInit.ErPreemptPriority, pI2cHandle->IrqInit.ErSubPriority);
	HAL_NVIC_EnableIRQ(pI2cHandle->IrqInit.ErIrqn);
	HAL_NVIC_SetPriority(pI2cHandle->IrqInit.EvIrqn, pI2cHandle->IrqInit.ErPreemptPriority, pI2cHandle->IrqInit.EvSubPriority);
	HAL_NVIC_EnableIRQ(pI2cHandle->IrqInit.EvIrqn);
}

/**
  * @brief I2C MSP De-Initialization 
  *        This function frees the hardware resources used in this example:
  *          - Disable the Peripheral's clock
  *          - Revert GPIO, DMA and NVIC configuration to their default state
  * @param hi2c: I2C handle pointer
  * @retval None
  */
void HAL_I2C_MspDeInit(I2C_HandleTypeDef *hi2c)
{
	BspI2cHandle *pI2cHandle;
	
	pI2cHandle = PortMapGet(AllPortMap,PROT_NUM,hi2c->Instance);
	if(pI2cHandle == NULL)
	{
		return;
	}

	pI2cHandle->I2cClkSet.ftPortForceReset();
	
	pI2cHandle->I2cClkSet.ftPortReleaseReset();

	HAL_GPIO_DeInit(pI2cHandle->I2cPortInit.SclGpioPort, pI2cHandle->SclGpioInit.Pin);

	HAL_GPIO_DeInit(pI2cHandle->I2cPortInit.SdaGpioPort, pI2cHandle->SdaGpioInit.Pin);

	HAL_NVIC_DisableIRQ(pI2cHandle->IrqInit.ErIrqn);
	HAL_NVIC_DisableIRQ(pI2cHandle->IrqInit.EvIrqn);
}

/**
  * @brief  Tx Transfer completed callback.
  * @param  I2cHandle: I2C handle 
  * @note   This example shows a simple way to report end of IT Tx transfer, and 
  *         you can add your own implementation. 
  * @retval None
  */
void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *I2cHandle)
{
	BspI2cHandle *pI2cHandle;
	
	pI2cHandle = PortMapGet(AllPortMap,PROT_NUM,I2cHandle->Instance);
	if(pI2cHandle == NULL)
	{
		return;
	}

	pI2cHandle->I2cFlag.TxCpltFlag = true;
}

/**
  * @brief  Rx Transfer completed callback.
  * @param  I2cHandle: I2C handle
  * @note   This example shows a simple way to report end of IT Rx transfer, and 
  *         you can add your own implementation.
  * @retval None
  */
void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *I2cHandle)
{
	BspI2cHandle *pI2cHandle;
	
	pI2cHandle = PortMapGet(AllPortMap,PROT_NUM,I2cHandle->Instance);
	if(pI2cHandle == NULL)
	{
		return;
	}

	pI2cHandle->I2cFlag.RxCpltFlag = true;


}

/** @brief  Slave Tx Transfer completed callbacks.
  * @param  hi2c: pointer to a I2C_HandleTypeDef structure that contains
  *         the configuration information for I2C module
  * @retval None
  */
void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	BspI2cHandle *pI2cHandle;
	
	pI2cHandle = PortMapGet(AllPortMap,PROT_NUM,hi2c->Instance);
	if(pI2cHandle == NULL)
	{
		return;
	}

	pI2cHandle->I2cFlag.TxCpltFlag = true;


}

/**
  * @brief  Slave Rx Transfer completed callbacks.
  * @param  hi2c: pointer to a I2C_HandleTypeDef structure that contains
  *         the configuration information for I2C module
  * @retval None
  */
void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	BspI2cHandle *pI2cHandle;
	
	pI2cHandle = PortMapGet(AllPortMap,PROT_NUM,hi2c->Instance);
	if(pI2cHandle == NULL)
	{
		return;
	}

	pI2cHandle->I2cFlag.RxCpltFlag = true;

}

void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	BspI2cHandle *pI2cHandle;
	
	pI2cHandle = PortMapGet(AllPortMap,PROT_NUM,hi2c->Instance);
	if(pI2cHandle == NULL)
	{
		return;
	}

	pI2cHandle->I2cFlag.TxCpltFlag = true;
}

/**
  * @brief  Memory Rx Transfer completed callbacks.
  * @param  hi2c: pointer to a I2C_HandleTypeDef structure that contains
  *         the configuration information for I2C module
  * @retval None
  */
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	BspI2cHandle *pI2cHandle;
	
	pI2cHandle = PortMapGet(AllPortMap,PROT_NUM,hi2c->Instance);
	if(pI2cHandle == NULL)
	{
		return;
	}

	pI2cHandle->I2cFlag.RxCpltFlag = true;
}

/**
  * @brief  I2C error callbacks.
  * @param  I2cHandle: I2C handle
  * @note   This example shows a simple way to report transfer error, and you can
  *         add your own implementation.
  * @retval None
  */
 void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *I2cHandle)
{
	BspI2cHandle *pI2cHandle;
	u32 i2cError;
	pI2cHandle = PortMapGet(AllPortMap,PROT_NUM,I2cHandle->Instance);
	if(pI2cHandle == NULL)
	{
		return;
	}

	i2cError = HAL_I2C_GetError(I2cHandle);

	if(i2cError == HAL_I2C_ERROR_OVR)
	{
		pI2cHandle->I2cFlag.RxQueueOverFlag = true;
	}
	pI2cHandle->I2cFlag.TxCpltFlag = true;
	pI2cHandle->I2cFlag.RxCpltFlag = true;
	SaveLog(NULL,0);
}




/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
