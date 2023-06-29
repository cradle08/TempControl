/***********************************************************
Module Name: gpio.c
Description: gpio初始化 读/写
Module Date: 12-08-2016
Module Author: Firmware-Team
Others:
***********************************************************/
#ifndef  _GPIO_H_
#define  _GPIO_H_


#define   PUMP_A                      GPIO_PIN_1      //PB1
#define   PUMPA_F                     GPIO_PIN_9      //PB9
#define   PUMPA_S                     GPIO_PIN_4      //PB4    复用管脚

#define   PUMP_B                      GPIO_PIN_2      //PB2
#define   PUMPB_F                     GPIO_PIN_8      //PB8
#define   PUMPB_S                     GPIO_PIN_14     //PA14   复用管脚


#define   PUMP_C                      GPIO_PIN_4      //PA4
#define   PUMPC_F                     GPIO_PIN_8      //PA15   复用管脚
#define   PUMPC_S                     GPIO_PIN_13     //PA13   复用管脚


#define   PUMP_D                      GPIO_PIN_10     //PB10
#define   PUMPD_F                     GPIO_PIN_3      //PB3    复用管脚 
#define   PUMPD_S                     GPIO_PIN_0      //PA8    复用管脚

#define   TEC_IN1_A                   GPIO_PIN_2      //PC2
#define   TEC_IN2_A                   GPIO_PIN_1      //PC1
#define   TEC_IN1_B                   GPIO_PIN_2      //PB2
#define   TEC_IN2_B                   GPIO_PIN_1      //PB1
#define   TEC_IN1_C                   GPIO_PIN_8      //PC8
#define   TEC_IN2_C                   GPIO_PIN_7      //PC7
#define   TEC_IN1_D                   GPIO_PIN_11     //PC11
#define   TEC_IN2_D                   GPIO_PIN_12     //PC12

#define   AD_DOUT                     GPIO_PIN_15     //PB15
#define   AD_DIN                      GPIO_PIN_14     //PB14
#define   AD_SCLK                     GPIO_PIN_13     //PB13
#define   AD_CS                       GPIO_PIN_12     //PB12
#define   AD_SYNC                     GPIO_PIN_11     //PB11
extern void ConfigGpio( void );


#endif

