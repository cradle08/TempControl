/**
  ******************************************************************************
  * @file    Version.h
  * @author  yanxiangwen
  * @version V1.0.1
  * @date    09-02-2017
  * @brief
  ******************************************************************************
**/

#ifndef  __VERSION_INFO__
#define  __VERSION_INFO__

/*
*********************************************************************************************************
*                                              INCLUDE FILES
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*                                               INT DEFINES
*********************************************************************************************************
*/
/*****平台驱动版本号(内部使用)****************/

#define DRIVER_VERSION_MAIN					0x01				//一个字节		
#define DRIVER_VERSION_SECONDRY			0x00				//一个字节		
#define DRIVER_VERSION_REVISION			0x00				//一个字节		
#define DRIVER_VERSION_INTERIOR			0x00000006	//四个字节		

/*****fireware程序版本号****************/
#define FIRMWARE_VERSION_MAIN				0xFF				//一个字节			
#define FIRMWARE_VERSION_SECONDRY		0x00				//一个字节		
#define FIRMWARE_VERSION_REVISION		0x00				//一个字节		
#define FIRMWARE_VERSION_INTERIOR		0x00000004	//四个字节


/*****配置文件版本号****************/
#define CONFIG_VERSION_MAIN				  0xFF				//一个字节		
#define CONFIG_VERSION_SECONDRY			0xFF				//一个字节		


/*******通讯协议版本号**************/
#define PROTOCOL_VERSION_MAIN				0x02				//一个字节		
#define PROTOCOL_VERSION_SECONDRY		0x00				//一个字节		

__packed
typedef struct
{
  unsigned char main;
  unsigned char sub;
} MAIN_SUB_VER, *P_MAIN_SUB_VER;

__packed
typedef struct
{
  MAIN_SUB_VER  stMainVer;
  unsigned char revision;
  unsigned long interior;
} SOFTWARE_VER, *P_SOFTWARE_VER;

/*
*********************************************************************************************************
*                                           FUNCTION PROTOTYPES
*********************************************************************************************************
*/
unsigned long GetSoftwareVersion( unsigned char *pDataArr );
unsigned long GetBoardInformation( unsigned char *pDataArr );
unsigned long SetBoardInfo( unsigned char *pDataArr, unsigned char len );
/*
*********************************************************************************************************
*                                        LOCAL GLOBAL VARIABLES
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*                                             MODULE END
*********************************************************************************************************
*/




#endif /* End */
