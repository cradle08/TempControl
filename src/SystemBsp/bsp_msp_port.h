/**
  ******************************************************************************
  * @file    bsp_msp_port.h
  * @author  Firmware-Team
  * @version V1.0.0
  * @date    13-06-2016
  * @brief
  ******************************************************************************
**/

#ifndef  __BSP_MSP_PROT__
#define  __BSP_MSP_PROT__

/*
*********************************************************************************************************
*                                              INCLUDE FILES
*********************************************************************************************************
*/
#include "stm32f3xx_hal_msp.h"
#include "loopBuff.h"


/*
*********************************************************************************************************
*                                               INT DEFINES
*********************************************************************************************************
*/
#define PROT_NUM 20

/*
*********************************************************************************************************
*                                        LOCAL GLOBAL VARIABLES
*********************************************************************************************************
*/

typedef enum
{
  BSP_OK       							= 0x00,
  BSP_HAL_ERROR    						= 0x01,
  BSP_HAL_BUSY     						,
  BSP_HAL_TIMEOUT  						,
  BSP_PARAMETER_ERROR 					,
  BSP_PORTMAP_ERROR 					,
  BSP_RX_QUEUE_INIT_ERROR 				,
  BSP_TX_QUEUE_INIT_ERROR 				,
  BSP_RX_QUEUE_EMPTY 					,
  BSP_TX_QUEUE_FULL 					,
  BSP_RX_QUEUE_OVER 					,
  BSP_TX_QUEUE_OVER 					,
  BSP_RX_HAL_ERROR 						,
  BSP_TX_HAL_ERROR
} BspStatusTypeDef;

typedef enum
{
  MASTER_I2C,
  SLAVE_I2C
} BspI2cTypeDef;

typedef struct
{
  void                 *PortInstance;
  void 				 *PortHandle;
} PortMap;

typedef void( *ClkSet )( void );

typedef void( *timCallBackFun )( void );
typedef struct
{
  IRQn_Type Irqn;
  u32 PreemptPriority;
  u32 SubPriority;
} PortIrq;

typedef struct
{
  IRQn_Type ErIrqn;
  u32 ErPreemptPriority;
  u32 ErSubPriority;
  IRQn_Type EvIrqn;
  u32 EvPreemptPriority;
  u32 EvSubPriority;
} I2cPortIrq;


typedef struct
{
  bool TxCpltFlag;
  bool RxCpltFlag;
  bool TxQueueOverFlag;
  bool RxQueueOverFlag;
  bool RxHalErrorFlag;
  bool TxHalErrorFlag;
} BspFlag;


typedef struct
{
  GPIO_TypeDef * TxGpioPort;
  GPIO_TypeDef * RxGpioPort;
} UartPort;

typedef struct
{
  GPIO_TypeDef * MosiGpioPort;
  GPIO_TypeDef * MisoGpioPort;
  GPIO_TypeDef * SckGpioPort;
  GPIO_TypeDef * CsGpioPort;
} SpiPort;

typedef struct
{
  GPIO_TypeDef * SdaGpioPort;
  GPIO_TypeDef * SclGpioPort;
} I2cPort;




typedef struct
{
  ClkSet ftPortClkEnable;
  ClkSet ftRxGpioClkEnable;
  ClkSet ftTxGpioClkEnable;
  ClkSet ftPortForceReset;
  ClkSet ftPortReleaseReset;
} UartClk;


typedef struct
{
  ClkSet ftPortClkEnable;
  ClkSet ftMisoGpioClkEnable;
  ClkSet ftMosiGpioClkEnable;
  ClkSet ftSckGpioClkEnable;
  ClkSet ftCsGpioClkEnable;
  ClkSet ftPortForceReset;
  ClkSet ftPortReleaseReset;
} SpiClk;

typedef struct
{
  ClkSet ftPortClkEnable;
  ClkSet ftSdaGpioClkEnable;
  ClkSet ftSclpioClkEnable;
  ClkSet ftPortForceReset;
  ClkSet ftPortReleaseReset;
} I2cClk;


typedef struct
{
  Queue *RxQueue;
  u8 *RxBuff;
  u32	RxBuffSize;
  Queue *TxQueue;
  u8 *TxBuff;
  u32 TxBuffSize;
} PortQueue;

typedef struct
{
  UART_HandleTypeDef				UartHandleInit;

  GPIO_InitTypeDef				RxGpioInit;

  GPIO_InitTypeDef				TxGpioInit;

  UartPort						UartPortInit;

  UartClk 						UartClkSet;

  PortIrq							IrqInit;

  BspFlag 						UartFlag;

  PortQueue 						UartQuque;

  u8								RxData;
  u8								TxData;
} BspUartHandle;


typedef struct
{
  SPI_HandleTypeDef				SpiHandleInit;

  GPIO_InitTypeDef				MisoGpioInit;

  GPIO_InitTypeDef				MosiGpioInit;

  GPIO_InitTypeDef				SckGpioInit;

  GPIO_InitTypeDef				CsGpioInit;

  SpiPort							SpiPortInit;

  SpiClk 							SpiClkSet;

  PortIrq							IrqInit;

  BspFlag 						SpiFlag;

  PortQueue 						SpiQuque;

  u8								RxData;
  u8								TxData;
} BspSpiHandle;


typedef struct
{
  I2C_HandleTypeDef				I2cHandleInit;

  GPIO_InitTypeDef				SdaGpioInit;

  GPIO_InitTypeDef				SclGpioInit;

  I2cPort							I2cPortInit;

  I2cClk 							I2cClkSet;

  I2cPortIrq						IrqInit;

  BspI2cTypeDef 					I2cType;

  BspFlag 						I2cFlag;

  PortQueue 						I2cQuque;

  u8								RxData;

  u8								TxData;

} BspI2cHandle;

typedef struct
{
  TIM_HandleTypeDef				TimHandleInit;

  ClkSet							TimClkEnable;

  PortIrq							IrqInit;

  timCallBackFun					TimCallBack;

} BspTimHandle;


typedef struct
{
  bool							EnableFlag;

  ClkSet							GpioClkInit;

  GPIO_InitTypeDef				GpioInit;

  GPIO_TypeDef					*GpioPort;

  TIM_OC_InitTypeDef 				OConfig;
} BspPwmChannel;

typedef struct
{
  TIM_HandleTypeDef				TimHandleInit;

  ClkSet							TimClkEnable;


  BspPwmChannel 					PwmChannel1;

  BspPwmChannel 					PwmChannel2;

  BspPwmChannel 					PwmChannel3;

  BspPwmChannel 					PwmChannel4;
} BspPwmHandle;




extern PortMap AllPortMap[PROT_NUM];
/*
*********************************************************************************************************
*                                           FUNCTION PROTOTYPES
*********************************************************************************************************
*/
extern void *PortMapGet( PortMap *pInputPortMap, u32 InputPortNum, void *pInputPortType );

extern BspStatusTypeDef BspUartHandleInit( BspUartHandle *InputUartHandle );
extern BspStatusTypeDef BspUartReceiveIT( BspUartHandle *InputUartHandle, u8 *RxBuff, u32 RxLen, u32 *OutPutLen );
extern BspStatusTypeDef BspUartTransmitIT( BspUartHandle *InputUartHandle, u8 *TxBuff, u32 TxLen );

extern BspStatusTypeDef BspSpiHandleInit( BspSpiHandle *InputSpiHandle );
extern BspStatusTypeDef BspSpiRecieveIT( BspSpiHandle *InputSpiHandle, u8 *RxBuff, u32 RxLen, u32 *OutPutLen );
extern BspStatusTypeDef BspSpiTransmitIT( BspSpiHandle *InputSpiHandle, u8 *TxBuff, u32 TxLen );
extern BspStatusTypeDef BspSpiTransmitReceiveIT( BspSpiHandle *InputSpiHandle, u8 *TxBuff, u8 *RxBuff, u32 TxLen );

extern BspStatusTypeDef BspI2cHandleInit( BspI2cHandle *InputI2cHandle );
extern BspStatusTypeDef BspI2cRecieveIT( BspI2cHandle *InputI2cHandle, u16 DevAddress, u8 *RxBuff, u32 RxLen, u32 *OutPutLen );
extern BspStatusTypeDef BspI2cTransmitIT( BspI2cHandle *InputI2cHandle, u16 DevAddress, u8 *TxBuff, u32 TxLen );
extern BspStatusTypeDef BspI2cMemRecieveIT( BspI2cHandle *InputI2cHandle, u16 DevAddress, u32 MemAddress, u32 MemAddSize, u8 *RxBuff, u32 RxLen, u32 *OutPutLen );
extern BspStatusTypeDef BspI2cMemTransmitIT( BspI2cHandle *InputI2cHandle, u16 DevAddress, u32 MemAddress, u32 MemAddSize, u8 *TxBuff, u32 TxLen );

extern BspStatusTypeDef BspTimHandleInit( BspTimHandle *BspTimxHandle );
extern BspStatusTypeDef BspTimHandleStartIT( BspTimHandle *BspTimxHandle );
extern BspStatusTypeDef BspTimHandleStopIT( BspTimHandle *BspTimxHandle );

extern BspStatusTypeDef BspPwmHandleInit( BspPwmHandle *BspPwmxHandle );
extern BspStatusTypeDef BspPwmHandleStart( BspPwmHandle *BspPwmxHandle, u32 Channel );
extern BspStatusTypeDef BspPwmHandleStop( BspPwmHandle *BspPwmxHandle, u32 Channel );
extern BspStatusTypeDef BspPwmPulseValueSet( BspPwmHandle *BspPwmxHandle, u32 Channel, u32 Value );
/*
*********************************************************************************************************
*                                             MODULE END
*********************************************************************************************************
*/




#endif /* End */

