/**
  ******************************************************************************
  * @file    bsp_msp_port.c
  * @author  Firmware-Team
  * @version V1.0.1
  * @date    13-06-2016
  * @brief
  ******************************************************************************
**/
/*
*********************************************************************************************************
*                                             INCLUDE FILES
*********************************************************************************************************
*/
#include  <includes.h>
#include  <string.h>


/*
*********************************************************************************************************
*                                               INT DEFINES
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                            LOCAL DEFINES
*********************************************************************************************************
*/

PortMap AllPortMap[PROT_NUM];


typedef struct
{
  u32                 *Instance;

} HandleTypeDef;


typedef struct
{
  HandleTypeDef				HandleInit;
} BspHandle;


/*
*********************************************************************************************************
*                                        LOCAL GLOBAL VARIABLES
*********************************************************************************************************
*/



/*
*********************************************************************************************************
*                                         FUNCTION PROTOTYPES
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*                                          PortMapInit
*
* Description : InitPortMap
*
* Arguments   : pPortMapInput
*
* Returns     : true 成功  false 失败
********************************************************************************************************
*/
bool PortMapInit( PortMap *pInputPortMap, u32 InputPortNum )
{
  if( pInputPortMap == NULL )
  {
    return false;
  }
  if( InputPortNum == 0 || InputPortNum > PROT_NUM )
  {
    return false;
  }
  memset( ( void * )pInputPortMap, 0l, ( u32 )InputPortNum * sizeof( PortMap ) );
  return true;
}

/*
*********************************************************************************************************
*                                          PortMapInit
*
* Description : InitPortMap
*
* Arguments   : pPortMapInput
*
* Returns     : true 成功  false 失败
*********************************************************************************************************
*/
bool PortMapSet( PortMap *pInputPortMap, u32 InputPortNum, void *InputPortHandle )
{
  u32 i;
  BspHandle *handle = ( BspHandle* )InputPortHandle;
  if( pInputPortMap == NULL )
  {
    return false;
  }

  if( InputPortNum == 0 || InputPortNum > PROT_NUM )
  {
    return false;
  }

  if( InputPortHandle == NULL )
  {
    return false;
  }

  for( i = 0; i < InputPortNum; i++ )
  {
    if( pInputPortMap[i].PortInstance == NULL || pInputPortMap[i].PortInstance == handle->HandleInit.Instance )
    {
      pInputPortMap[i].PortInstance = handle->HandleInit.Instance;
      pInputPortMap[i].PortHandle = InputPortHandle;
      return true;
    }
  }
  return false;
}

/*
*********************************************************************************************************
*                                          PortMapInit
*
* Description : InitPortMap
*
* Arguments   : pPortMapInput
*
* Returns     : true 成功  false 失败
*********************************************************************************************************
*/
void *PortMapGet( PortMap *pInputPortMap, u32 InputPortNum, void *pInputPortType )
{
  u32 i;
  if( pInputPortMap == NULL )
  {
    return NULL;
  }

  if( InputPortNum == 0 || InputPortNum > PROT_NUM )
  {
    return NULL;
  }

  if( pInputPortType == NULL )
  {
    return NULL;
  }
  for( i = 0; i < InputPortNum; i++ )
  {
    if( pInputPortMap[i].PortInstance == pInputPortType )
    {
      return pInputPortMap[i].PortHandle;
    }
  }
  return NULL;
}


/*
*********************************************************************************************************
*                                          PortMapInit
*
* Description : InitPortMap
*
* Arguments   : pPortMapInput
*
* Returns     : true 成功  false 失败
*********************************************************************************************************
*/

BspStatusTypeDef BspUartHandleInit( BspUartHandle *InputUartHandle )
{
  bool result;
  HAL_StatusTypeDef status;
  if( InputUartHandle == NULL )
  {
    return BSP_PARAMETER_ERROR;
  }

  result = PortMapSet( AllPortMap, PROT_NUM, InputUartHandle );

  if( result == false )
  {
    return BSP_PORTMAP_ERROR;
  }

  HAL_UART_MspDeInit( &InputUartHandle->UartHandleInit );

  status = HAL_UART_Init( &InputUartHandle->UartHandleInit );

  if( status != HAL_OK )
  {
    return ( BspStatusTypeDef )status;
  }

  result = initQueue( InputUartHandle->UartQuque.RxQueue, InputUartHandle->UartQuque.RxBuff, InputUartHandle->UartQuque.RxBuffSize );

  if( result == false )
  {
    return BSP_RX_QUEUE_INIT_ERROR;
  }

  result = initQueue( InputUartHandle->UartQuque.TxQueue, InputUartHandle->UartQuque.TxBuff, InputUartHandle->UartQuque.TxBuffSize );

  if( result == false )
  {
    return BSP_TX_QUEUE_INIT_ERROR;
  }


  status = HAL_UART_Receive_IT( &InputUartHandle->UartHandleInit, &InputUartHandle->RxData, 1 );

  if( status != HAL_OK )
  {
    return ( BspStatusTypeDef )status;
  }

  return BSP_OK;

}



/*
*********************************************************************************************************
*                                          PortMapInit
*
* Description : InitPortMap
*
* Arguments   : pPortMapInput
*
* Returns     : true 成功  false 失败
*********************************************************************************************************
*/

BspStatusTypeDef BspUartReceiveIT( BspUartHandle *InputUartHandle, u8 *RxBuff, u32 RxLen, u32 *OutPutLen )
{
  u32 QueueRxLen, i;
  //	HAL_StatusTypeDef status;

  if( InputUartHandle == NULL )
  {
    return BSP_PARAMETER_ERROR;
  }
  if( RxBuff == NULL )
  {
    return BSP_PARAMETER_ERROR;
  }
  if( RxLen <= 0 || RxLen >= InputUartHandle->UartQuque.RxBuffSize )
  {
    return BSP_PARAMETER_ERROR;
  }
  if( OutPutLen == NULL )
  {
    return BSP_PARAMETER_ERROR;
  }

  if( InputUartHandle->UartFlag.RxHalErrorFlag == true )
  {

    //status = HAL_UART_Receive_IT(&InputUartHandle->UartHandleInit,&InputUartHandle->RxData,1);

    //if(status == HAL_OK)
    //{
    InputUartHandle->UartFlag.RxHalErrorFlag = false;
    //}

    return BSP_RX_HAL_ERROR;
  }

  QueueRxLen = queueFullN( InputUartHandle->UartQuque.RxQueue );

  if( QueueRxLen > 0 )
  {
    for( i = 0; i < RxLen; i++ )
    {
      if( isQueueEmpty( InputUartHandle->UartQuque.RxQueue ) )
      {
        break;
      }
      else
      {
        RxBuff[i] = deQueue( InputUartHandle->UartQuque.RxQueue );
      }
    }

    *OutPutLen = ( i );

  }
  else
  {
    *OutPutLen = 0;
    return BSP_RX_QUEUE_EMPTY;
  }

  if( InputUartHandle->UartFlag.RxQueueOverFlag )
  {

    if( isQueueEmpty( InputUartHandle->UartQuque.RxQueue ) )
    {
      InputUartHandle->UartFlag.RxQueueOverFlag = false;
    }
    return BSP_RX_QUEUE_OVER;
  }

  return BSP_OK;
}


/*
*********************************************************************************************************
*                                          BspUartTransmitIT
*
* Description : 串口中断发送函数
*
* Arguments   : InputUartHandle
*				        TxBuff
*				        TxLen
*
* Returns     : BspStatusTypeDef
*
* Notes       :
*********************************************************************************************************
*/

BspStatusTypeDef BspUartTransmitIT( BspUartHandle *InputUartHandle, u8 *TxBuff, u32 TxLen )
{
  u32 TxQueueEmptyLen, i;
  HAL_UART_StateTypeDef uartState;
  HAL_StatusTypeDef status;

  if( InputUartHandle == NULL )
  {
    return BSP_PARAMETER_ERROR;
  }
  if( TxBuff == NULL )
  {
    return BSP_PARAMETER_ERROR;
  }
  if( TxLen <= 0 || TxLen >= InputUartHandle->UartQuque.TxBuffSize )
  {
    return BSP_PARAMETER_ERROR;
  }

  TxQueueEmptyLen = queueEmptyN( InputUartHandle->UartQuque.TxQueue );

  uartState = HAL_UART_GetState( &InputUartHandle->UartHandleInit );
  uartState = uartState;
  //if((uartState == HAL_UART_STATE_READY) || (uartState == HAL_UART_STATE_BUSY_RX))
  if( 1 )
  {
    if( TxQueueEmptyLen >= TxLen )
    {
      for( i = 0; i < TxLen; i++ )
      {
        enQueue( InputUartHandle->UartQuque.TxQueue, TxBuff[i] );
      }
    }
    else
    {
      if( InputUartHandle->UartFlag.TxCpltFlag == false )
      {
        return BSP_TX_QUEUE_FULL;
      }
      if( isQueueEmpty( InputUartHandle->UartQuque.TxQueue ) == false )
      {
        InputUartHandle->TxData = deQueue( InputUartHandle->UartQuque.TxQueue );

        __HAL_UART_DISABLE_IT( &InputUartHandle->UartHandleInit, UART_IT_RXNE );

        status = HAL_UART_Transmit_IT( &InputUartHandle->UartHandleInit, &InputUartHandle->TxData, 1 );
        InputUartHandle->UartFlag.TxCpltFlag = false;

        __HAL_UART_ENABLE_IT( &InputUartHandle->UartHandleInit, UART_IT_RXNE );
        if( status != HAL_OK )
        {
          InputUartHandle->UartFlag.TxCpltFlag = true;
          return ( BspStatusTypeDef )status;
        }
      }
      return BSP_TX_QUEUE_FULL;
    }
  }
  else
  {
    return BSP_HAL_BUSY;
  }



  if( InputUartHandle->UartFlag.TxCpltFlag == false )
  {
    return BSP_OK;
  }
  if( isQueueEmpty( InputUartHandle->UartQuque.TxQueue ) == false )
  {
    InputUartHandle->TxData = deQueue( InputUartHandle->UartQuque.TxQueue );

    __HAL_UART_DISABLE_IT( &InputUartHandle->UartHandleInit, UART_IT_RXNE );

    status = HAL_UART_Transmit_IT( &InputUartHandle->UartHandleInit, &InputUartHandle->TxData, 1 );

    InputUartHandle->UartFlag.TxCpltFlag = false;

    __HAL_UART_ENABLE_IT( &InputUartHandle->UartHandleInit, UART_IT_RXNE );
    if( status != HAL_OK )
    {
      InputUartHandle->UartFlag.TxCpltFlag = true;
      return ( BspStatusTypeDef )status;
    }
  }
  return BSP_OK;
}




BspStatusTypeDef BspSpiHandleInit( BspSpiHandle *InputSpiHandle )
{
  bool result;
  HAL_StatusTypeDef status;
  BspStatusTypeDef bspStatus;
  if( InputSpiHandle == NULL )
  {
    return BSP_PARAMETER_ERROR;
  }

  result = PortMapSet( AllPortMap, PROT_NUM, InputSpiHandle );

  if( result == false )
  {
    return BSP_PORTMAP_ERROR;
  }

  HAL_SPI_MspDeInit( &InputSpiHandle->SpiHandleInit );

  status = HAL_SPI_Init( &InputSpiHandle->SpiHandleInit );

  if( status != HAL_OK )
  {
    return ( BspStatusTypeDef )status;
  }

  result = initQueue( InputSpiHandle->SpiQuque.RxQueue, InputSpiHandle->SpiQuque.RxBuff, InputSpiHandle->SpiQuque.RxBuffSize );

  if( result == false )
  {
    return BSP_RX_QUEUE_INIT_ERROR;
  }

  result = initQueue( InputSpiHandle->SpiQuque.TxQueue, InputSpiHandle->SpiQuque.TxBuff, InputSpiHandle->SpiQuque.TxBuffSize );

  if( result == false )
  {
    return BSP_TX_QUEUE_INIT_ERROR;
  }

  if( InputSpiHandle->SpiHandleInit.Init.Mode == SPI_MODE_SLAVE )
  {
    bspStatus = BspSpiTransmitReceiveIT( InputSpiHandle, &InputSpiHandle->TxData, &InputSpiHandle->RxData, 1 );
    if( bspStatus != BSP_OK )
    {
      return bspStatus;
    }
  }
  return BSP_OK;

}



BspStatusTypeDef BspSpiRecieveIT( BspSpiHandle *InputSpiHandle, u8 *RxBuff, u32 RxLen, u32 *OutPutLen )
{
  HAL_StatusTypeDef status;


  if( InputSpiHandle == NULL )
  {
    return BSP_PARAMETER_ERROR;
  }
  if( RxBuff == NULL )
  {
    return BSP_PARAMETER_ERROR;
  }
  if( RxLen <= 0 || RxLen >= InputSpiHandle->SpiQuque.RxBuffSize )
  {
    return BSP_PARAMETER_ERROR;
  }
  if( OutPutLen == NULL )
  {
    return BSP_PARAMETER_ERROR;
  }

  *OutPutLen = 0;

  InputSpiHandle->SpiFlag.RxCpltFlag = false;

  status = HAL_SPI_Receive_IT( &InputSpiHandle->SpiHandleInit, RxBuff, RxLen );

  if( status == HAL_OK )
  {
    *OutPutLen = RxLen;
  }
  else
  {
    InputSpiHandle->SpiFlag.RxCpltFlag = true;
  }

  return ( BspStatusTypeDef )status;

}

BspStatusTypeDef BspSpiTransmitIT( BspSpiHandle *InputSpiHandle, u8 *TxBuff, u32 TxLen )
{
  HAL_StatusTypeDef status;

  if( InputSpiHandle == NULL )
  {
    return BSP_PARAMETER_ERROR;
  }
  if( TxBuff == NULL )
  {
    return BSP_PARAMETER_ERROR;
  }
  if( TxLen <= 0 || TxLen >= InputSpiHandle->SpiQuque.TxBuffSize )
  {
    return BSP_PARAMETER_ERROR;
  }


  InputSpiHandle->SpiFlag.TxCpltFlag = false;

  status = HAL_SPI_Transmit_IT( &InputSpiHandle->SpiHandleInit, TxBuff, TxLen );

  if( status != HAL_OK )
  {
    InputSpiHandle->SpiFlag.TxCpltFlag = true;
  }
  return ( BspStatusTypeDef )status;
}


BspStatusTypeDef BspSpiTransmitReceiveIT( BspSpiHandle *InputSpiHandle, u8 *TxBuff, u8 *RxBuff, u32 TxLen )
{
  u32 i;
  HAL_StatusTypeDef status;

  if( InputSpiHandle == NULL )
  {
    return BSP_PARAMETER_ERROR;
  }
  if( TxBuff == NULL )
  {
    return BSP_PARAMETER_ERROR;
  }
  if( RxBuff == NULL )
  {
    return BSP_PARAMETER_ERROR;
  }
  if( TxLen <= 0 || TxLen >= InputSpiHandle->SpiQuque.TxBuffSize )
  {
    return BSP_PARAMETER_ERROR;
  }

  if( InputSpiHandle->SpiHandleInit.Init.Mode == SPI_MODE_SLAVE )
  {
    for( i = 0; i < TxLen; i++ )
    {

      if( isQueueEmpty( InputSpiHandle->SpiQuque.RxQueue ) )
      {
        RxBuff[i] = 0;
      }
      else
      {
        RxBuff[i] = deQueue( InputSpiHandle->SpiQuque.RxQueue );
      }

      if( isQueueFull( InputSpiHandle->SpiQuque.TxQueue ) )
      {
        return BSP_TX_QUEUE_FULL;
      }
      else
      {
        enQueue( InputSpiHandle->SpiQuque.TxQueue, TxBuff[i] );
      }

      InputSpiHandle->SpiFlag.TxCpltFlag = false;
      InputSpiHandle->SpiFlag.RxCpltFlag = false;
      status = HAL_SPI_TransmitReceive_IT( &InputSpiHandle->SpiHandleInit, &InputSpiHandle->TxData, &InputSpiHandle->RxData, 1 );

      if( status != HAL_OK )
      {
        InputSpiHandle->SpiFlag.TxCpltFlag = true;
        InputSpiHandle->SpiFlag.RxCpltFlag = true;
      }
    }
  }
  else
  {
    InputSpiHandle->SpiFlag.TxCpltFlag = false;
    InputSpiHandle->SpiFlag.RxCpltFlag = false;
    status = HAL_SPI_TransmitReceive_IT( &InputSpiHandle->SpiHandleInit, TxBuff, RxBuff, TxLen );

    if( status != HAL_OK )
    {
      InputSpiHandle->SpiFlag.TxCpltFlag = true;
      InputSpiHandle->SpiFlag.RxCpltFlag = true;
    }
  }
  return ( BspStatusTypeDef )status;

}


BspStatusTypeDef BspI2cHandleInit( BspI2cHandle *InputI2cHandle )
{
  bool result;
  HAL_StatusTypeDef status;

  if( InputI2cHandle == NULL )
  {
    return BSP_PARAMETER_ERROR;
  }

  result = PortMapSet( AllPortMap, PROT_NUM, InputI2cHandle );

  if( result == false )
  {
    return BSP_PORTMAP_ERROR;
  }

  HAL_I2C_MspDeInit( &InputI2cHandle->I2cHandleInit );

  status = HAL_I2C_Init( &InputI2cHandle->I2cHandleInit );

  if( status != HAL_OK )
  {
    return ( BspStatusTypeDef )status;
  }

  result = initQueue( InputI2cHandle->I2cQuque.RxQueue, InputI2cHandle->I2cQuque.RxBuff, InputI2cHandle->I2cQuque.RxBuffSize );

  if( result == false )
  {
    return BSP_RX_QUEUE_INIT_ERROR;
  }

  result = initQueue( InputI2cHandle->I2cQuque.TxQueue, InputI2cHandle->I2cQuque.TxBuff, InputI2cHandle->I2cQuque.TxBuffSize );

  if( result == false )
  {
    return BSP_TX_QUEUE_INIT_ERROR;
  }

  return BSP_OK;
}


BspStatusTypeDef BspI2cRecieveIT( BspI2cHandle *InputI2cHandle, u16 DevAddress, u8 *RxBuff, u32 RxLen, u32 *OutPutLen )
{
  HAL_StatusTypeDef status;


  if( InputI2cHandle == NULL )
  {
    return BSP_PARAMETER_ERROR;
  }
  if( RxBuff == NULL )
  {
    return BSP_PARAMETER_ERROR;
  }
  if( RxLen <= 0 || RxLen >= InputI2cHandle->I2cQuque.RxBuffSize )
  {
    return BSP_PARAMETER_ERROR;
  }
  if( OutPutLen == NULL )
  {
    return BSP_PARAMETER_ERROR;
  }

  *OutPutLen = 0;

  InputI2cHandle->I2cFlag.RxCpltFlag = false;

  if( InputI2cHandle->I2cType == SLAVE_I2C )
  {

    status = HAL_I2C_Slave_Receive_IT( &InputI2cHandle->I2cHandleInit, RxBuff, RxLen );
  }
  else
  {
    status = HAL_I2C_Master_Receive_IT( &InputI2cHandle->I2cHandleInit, DevAddress, RxBuff, RxLen );
  }

  if( status == HAL_OK )
  {
    *OutPutLen = RxLen;
  }
  else
  {
    InputI2cHandle->I2cFlag.RxCpltFlag = true;
  }

  return ( BspStatusTypeDef )status;

}




BspStatusTypeDef BspI2cTransmitIT( BspI2cHandle *InputI2cHandle, u16 DevAddress, u8 *TxBuff, u32 TxLen )
{
  HAL_StatusTypeDef status;

  if( InputI2cHandle == NULL )
  {
    return BSP_PARAMETER_ERROR;
  }
  if( TxBuff == NULL )
  {
    return BSP_PARAMETER_ERROR;
  }
  if( TxLen <= 0 || TxLen >= InputI2cHandle->I2cQuque.TxBuffSize )
  {
    return BSP_PARAMETER_ERROR;
  }

  InputI2cHandle->I2cFlag.TxCpltFlag = false;

  if( InputI2cHandle->I2cType == SLAVE_I2C )
  {
    status = HAL_I2C_Slave_Transmit_IT( &InputI2cHandle->I2cHandleInit, TxBuff, TxLen );
  }
  else
  {
    status = HAL_I2C_Master_Transmit_IT( &InputI2cHandle->I2cHandleInit, DevAddress, TxBuff, TxLen );
  }

  if( status != HAL_OK )
  {
    InputI2cHandle->I2cFlag.TxCpltFlag = true;
  }
  return ( BspStatusTypeDef )status;
}


BspStatusTypeDef BspI2cMemRecieveIT( BspI2cHandle *InputI2cHandle, u16 DevAddress, u32 MemAddress, u32 MemAddSize, u8 *RxBuff, u32 RxLen, u32 *OutPutLen )
{
  HAL_StatusTypeDef status;


  if( InputI2cHandle == NULL )
  {
    return BSP_PARAMETER_ERROR;
  }
  if( RxBuff == NULL )
  {
    return BSP_PARAMETER_ERROR;
  }
  if( RxLen <= 0 || RxLen >= InputI2cHandle->I2cQuque.RxBuffSize )
  {
    return BSP_PARAMETER_ERROR;
  }
  if( OutPutLen == NULL )
  {
    return BSP_PARAMETER_ERROR;
  }

  *OutPutLen = 0;

  InputI2cHandle->I2cFlag.RxCpltFlag = false;

  status = HAL_I2C_Mem_Read_IT( &InputI2cHandle->I2cHandleInit, DevAddress, MemAddress, MemAddSize, RxBuff, RxLen );

  if( status == HAL_OK )
  {
    *OutPutLen = RxLen;
  }
  else
  {
    InputI2cHandle->I2cFlag.RxCpltFlag = true;
  }

  return ( BspStatusTypeDef )status;

}


BspStatusTypeDef BspI2cMemTransmitIT( BspI2cHandle *InputI2cHandle, u16 DevAddress, u32 MemAddress, u32 MemAddSize, u8 *TxBuff, u32 TxLen )
{
  HAL_StatusTypeDef status;

  if( InputI2cHandle == NULL )
  {
    return BSP_PARAMETER_ERROR;
  }
  if( TxBuff == NULL )
  {
    return BSP_PARAMETER_ERROR;
  }
  if( TxLen <= 0 || TxLen >= InputI2cHandle->I2cQuque.TxBuffSize )
  {
    return BSP_PARAMETER_ERROR;
  }

  InputI2cHandle->I2cFlag.TxCpltFlag = false;

  status = HAL_I2C_Mem_Write_IT( &InputI2cHandle->I2cHandleInit, DevAddress, MemAddress, MemAddSize, TxBuff, TxLen );

  if( status != HAL_OK )
  {
    InputI2cHandle->I2cFlag.TxCpltFlag = true;
  }

  return ( BspStatusTypeDef )status;
}



BspStatusTypeDef BspTimHandleInit( BspTimHandle *BspTimxHandle )
{
  bool result;
  HAL_StatusTypeDef status;

  if( BspTimxHandle == NULL )
  {
    return BSP_PARAMETER_ERROR;
  }

  result = PortMapSet( AllPortMap, PROT_NUM, BspTimxHandle );

  if( result == false )
  {
    return BSP_PORTMAP_ERROR;
  }

  status = HAL_TIM_Base_Init( &BspTimxHandle->TimHandleInit );

  return ( BspStatusTypeDef )status;
}


BspStatusTypeDef BspTimHandleStartIT( BspTimHandle *BspTimxHandle )
{
  HAL_StatusTypeDef status;

  if( BspTimxHandle == NULL )
  {
    return BSP_PARAMETER_ERROR;
  }

  status = HAL_TIM_Base_Start_IT( &BspTimxHandle->TimHandleInit );

  return ( BspStatusTypeDef )status;
}

BspStatusTypeDef BspTimHandleStopIT( BspTimHandle *BspTimxHandle )
{
  HAL_StatusTypeDef status;

  if( BspTimxHandle == NULL )
  {
    return BSP_PARAMETER_ERROR;
  }

  status = HAL_TIM_Base_Stop_IT( &BspTimxHandle->TimHandleInit );

  return ( BspStatusTypeDef )status;
}



BspStatusTypeDef BspPwmHandleInit( BspPwmHandle *BspPwmxHandle )
{
  bool result;
  HAL_StatusTypeDef status;

  if( BspPwmxHandle == NULL )
  {
    return BSP_PARAMETER_ERROR;
  }

  result = PortMapSet( AllPortMap, PROT_NUM, BspPwmxHandle );

  if( result == false )
  {
    return BSP_PORTMAP_ERROR;
  }

  status = HAL_TIM_PWM_Init( &BspPwmxHandle->TimHandleInit );

  if( status != HAL_OK )
  {
    return ( BspStatusTypeDef )status;
  }


  if( BspPwmxHandle->PwmChannel1.EnableFlag == true )
  {
    status = HAL_TIM_PWM_ConfigChannel( &BspPwmxHandle->TimHandleInit, &BspPwmxHandle->PwmChannel1.OConfig, TIM_CHANNEL_1 );
    if( status != HAL_OK )
    {
      return ( BspStatusTypeDef )status;
    }
  }

  if( BspPwmxHandle->PwmChannel2.EnableFlag == true )
  {
    status = HAL_TIM_PWM_ConfigChannel( &BspPwmxHandle->TimHandleInit, &BspPwmxHandle->PwmChannel2.OConfig, TIM_CHANNEL_2 );
    if( status != HAL_OK )
    {
      return ( BspStatusTypeDef )status;
    }
  }


  if( BspPwmxHandle->PwmChannel3.EnableFlag == true )
  {
    status = HAL_TIM_PWM_ConfigChannel( &BspPwmxHandle->TimHandleInit, &BspPwmxHandle->PwmChannel3.OConfig, TIM_CHANNEL_3 );
    if( status != HAL_OK )
    {
      return ( BspStatusTypeDef )status;
    }
  }


  if( BspPwmxHandle->PwmChannel4.EnableFlag == true )
  {
    status = HAL_TIM_PWM_ConfigChannel( &BspPwmxHandle->TimHandleInit, &BspPwmxHandle->PwmChannel4.OConfig, TIM_CHANNEL_4 );
    if( status != HAL_OK )
    {
      return ( BspStatusTypeDef )status;
    }
  }

  return BSP_OK;

}


BspStatusTypeDef BspPwmHandleStart( BspPwmHandle *BspPwmxHandle, u32 Channel )
{
  HAL_StatusTypeDef status = HAL_ERROR;

  if( BspPwmxHandle == NULL )
  {
    return BSP_PARAMETER_ERROR;
  }

  switch( Channel )
  {
    case TIM_CHANNEL_1:
    {
      if( BspPwmxHandle->PwmChannel1.EnableFlag == true )
      {
        status = HAL_TIM_PWM_Start( &BspPwmxHandle->TimHandleInit, TIM_CHANNEL_1 );
        if( status != HAL_OK )
        {
          return ( BspStatusTypeDef )status;
        }
      }
      else
      {
        return BSP_HAL_ERROR;
      }
    }
    break;
    case TIM_CHANNEL_2:
    {
      if( BspPwmxHandle->PwmChannel2.EnableFlag == true )
      {
        status = HAL_TIM_PWM_Start( &BspPwmxHandle->TimHandleInit, TIM_CHANNEL_2 );
        if( status != HAL_OK )
        {
          return ( BspStatusTypeDef )status;
        }
      }
      else
      {
        return BSP_HAL_ERROR;
      }
    }
    break;
    case TIM_CHANNEL_3:
    {
      if( BspPwmxHandle->PwmChannel3.EnableFlag == true )
      {
        status = HAL_TIM_PWM_Start( &BspPwmxHandle->TimHandleInit, TIM_CHANNEL_3 );
        if( status != HAL_OK )
        {
          return ( BspStatusTypeDef )status;
        }
      }
      else
      {
        return BSP_HAL_ERROR;
      }
    }
    break;
    case TIM_CHANNEL_4:
    {
      if( BspPwmxHandle->PwmChannel4.EnableFlag == true )
      {
        status = HAL_TIM_PWM_Start( &BspPwmxHandle->TimHandleInit, TIM_CHANNEL_4 );
        if( status != HAL_OK )
        {
          return ( BspStatusTypeDef )status;
        }
      }
      else
      {
        return BSP_HAL_ERROR;
      }
    }
    break;
    default:
      return BSP_PARAMETER_ERROR;
  }
  return BSP_OK;
}


BspStatusTypeDef BspPwmHandleStop( BspPwmHandle *BspPwmxHandle, u32 Channel )
{
  HAL_StatusTypeDef status = HAL_ERROR;

  if( BspPwmxHandle == NULL )
  {
    return BSP_PARAMETER_ERROR;
  }

  switch( Channel )
  {
    case TIM_CHANNEL_1:
    {
      if( BspPwmxHandle->PwmChannel1.EnableFlag == true )
      {
        status = HAL_TIM_PWM_Stop( &BspPwmxHandle->TimHandleInit, TIM_CHANNEL_1 );
        if( status != HAL_OK )
        {
          return ( BspStatusTypeDef )status;
        }
      }
      else
      {
        return BSP_HAL_ERROR;
      }
    }
    break;
    case TIM_CHANNEL_2:
    {
      if( BspPwmxHandle->PwmChannel2.EnableFlag == true )
      {
        status = HAL_TIM_PWM_Stop( &BspPwmxHandle->TimHandleInit, TIM_CHANNEL_2 );
        if( status != HAL_OK )
        {
          return ( BspStatusTypeDef )status;
        }
      }
      else
      {
        return BSP_HAL_ERROR;
      }
    }
    break;
    case TIM_CHANNEL_3:
    {
      if( BspPwmxHandle->PwmChannel3.EnableFlag == true )
      {
        status = HAL_TIM_PWM_Stop( &BspPwmxHandle->TimHandleInit, TIM_CHANNEL_3 );
        if( status != HAL_OK )
        {
          return ( BspStatusTypeDef )status;
        }
      }
      else
      {
        return BSP_HAL_ERROR;
      }
    }
    break;
    case TIM_CHANNEL_4:
    {
      if( BspPwmxHandle->PwmChannel4.EnableFlag == true )
      {
        status = HAL_TIM_PWM_Stop( &BspPwmxHandle->TimHandleInit, TIM_CHANNEL_4 );
        if( status != HAL_OK )
        {
          return ( BspStatusTypeDef )status;
        }
      }
      else
      {
        return BSP_HAL_ERROR;
      }
    }
    break;
    default:
      return BSP_PARAMETER_ERROR;
  }
  return BSP_OK;
}



BspStatusTypeDef BspPwmPulseValueSet( BspPwmHandle *BspPwmxHandle, u32 Channel, u32 Value )
{
  if( BspPwmxHandle == NULL )
  {
    return BSP_PARAMETER_ERROR;
  }

  switch( Channel )
  {
    case TIM_CHANNEL_1:
    {
      BspPwmxHandle->TimHandleInit.Instance->CCR1 = Value;
    }
    break;
    case TIM_CHANNEL_2:
    {
      BspPwmxHandle->TimHandleInit.Instance->CCR2 = Value;
    }
    break;
    case TIM_CHANNEL_3:
    {
      BspPwmxHandle->TimHandleInit.Instance->CCR3 = Value;
    }
    break;
    case TIM_CHANNEL_4:
    {
      BspPwmxHandle->TimHandleInit.Instance->CCR4 = Value;
    }
    break;
    default:
      return BSP_PARAMETER_ERROR;
  }
  return BSP_OK;
}



