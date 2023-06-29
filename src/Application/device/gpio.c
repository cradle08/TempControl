/***********************************************************
Module Name: gpio.c
Description: gpio��ʼ�� ��/д
Module Date: 12-08-2016
Module Author: Firmware-Team
Others:
***********************************************************/

#include   "includes.h"
#include   "gpio.h"



void ConfigGpio( void )
{
  GPIO_InitTypeDef GPIO_InitStruct;

  /**************************************************************************************
  	AD7124����PGA�ͻ�׼��ѹԴ��8ͨ��ADCоƬ PB11:ͬ�����롣��������һ���߼����룬
  	ʹ�ö��AD7124-8����ʱ��������ʹ�����˲�����ģ�������ͬ����
  **************************************************************************************/
  __GPIOB_CLK_ENABLE();
  GPIO_InitStruct.Pin   = AD_SYNC; //PB11
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init( GPIOB, &GPIO_InitStruct );

  HAL_GPIO_WritePin( GPIOB, AD_SYNC, GPIO_PIN_SET );


  /**************************************************************************************
    	4·����: PUMPA:PB1   PUMPB:PB2   PUMPC:PA4    PUMPD:PB10
  **************************************************************************************/
  __GPIOB_CLK_ENABLE();
  GPIO_InitStruct.Pin   = PUMP_A; //PUMPA:PB1
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init( GPIOB, &GPIO_InitStruct );


  __GPIOB_CLK_ENABLE();
  GPIO_InitStruct.Pin   = PUMP_B; //PUMPB:PB2
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init( GPIOB, &GPIO_InitStruct );

  __GPIOA_CLK_ENABLE();
  GPIO_InitStruct.Pin   = PUMP_C; //PUMPC:PA4
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init( GPIOA, &GPIO_InitStruct );

  __GPIOB_CLK_ENABLE();
  GPIO_InitStruct.Pin   = PUMP_D;  //PUMPD:PB10
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init( GPIOB, &GPIO_InitStruct );

//  /*����4·����*/
//  HAL_GPIO_WritePin( GPIOB, PUMP_A, GPIO_PIN_RESET );
//  HAL_GPIO_WritePin( GPIOB, PUMP_B, GPIO_PIN_RESET );
//  HAL_GPIO_WritePin( GPIOA, PUMP_C, GPIO_PIN_RESET );
//  HAL_GPIO_WritePin( GPIOB, PUMP_D, GPIO_PIN_RESET );

		/*�ر�4·����*/
		HAL_GPIO_WritePin( GPIOB, PUMP_A, GPIO_PIN_SET );
		HAL_GPIO_WritePin( GPIOB, PUMP_B, GPIO_PIN_SET );
		HAL_GPIO_WritePin( GPIOA, PUMP_C, GPIO_PIN_SET );
		HAL_GPIO_WritePin( GPIOB, PUMP_D, GPIO_PIN_SET );		
  /**************************************************************************************
    	4·����: PUMPA:PB1   PUMPB:PB2   PUMPC:PA4    PUMPD:PB10
  **************************************************************************************/
}






