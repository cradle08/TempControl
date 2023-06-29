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
/*****ƽ̨�����汾��(�ڲ�ʹ��)****************/

#define DRIVER_VERSION_MAIN					0x01				//һ���ֽ�		
#define DRIVER_VERSION_SECONDRY			0x00				//һ���ֽ�		
#define DRIVER_VERSION_REVISION			0x00				//һ���ֽ�		
#define DRIVER_VERSION_INTERIOR			0x00000006	//�ĸ��ֽ�		

/*****fireware����汾��****************/
#define FIRMWARE_VERSION_MAIN				0xFF				//һ���ֽ�			
#define FIRMWARE_VERSION_SECONDRY		0x00				//һ���ֽ�		
#define FIRMWARE_VERSION_REVISION		0x00				//һ���ֽ�		
#define FIRMWARE_VERSION_INTERIOR		0x00000004	//�ĸ��ֽ�


/*****�����ļ��汾��****************/
#define CONFIG_VERSION_MAIN				  0xFF				//һ���ֽ�		
#define CONFIG_VERSION_SECONDRY			0xFF				//һ���ֽ�		


/*******ͨѶЭ��汾��**************/
#define PROTOCOL_VERSION_MAIN				0x02				//һ���ֽ�		
#define PROTOCOL_VERSION_SECONDRY		0x00				//һ���ֽ�		

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
