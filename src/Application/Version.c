/**
  ******************************************************************************
  * @file    Version.c
  * @author  yanxiangwen
  * @version V1.0.1
  * @date    09-02-2017
  * @brief
  ******************************************************************************
**/
/*
*********************************************************************************************************
*                                             INCLUDE FILES
*********************************************************************************************************
*/
#include "Version.h"


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
static unsigned long GetDriverVersion( unsigned char *pDataArr );
static unsigned long GetFirmwareVersion( unsigned char *pDataArr );
static unsigned long GetConfigVersion( unsigned char *pDataArr );
static unsigned long GetProtocolVersion( unsigned char *pDataArr );


/*******************************************************************************************
* Func.  Name: GetDriverVersion
* Description: 获取平台软件的版本号
* Arguments  :
*      Inputs: 存储版本信息的数据缓存
*     Outputs: 数据长度
* Note(s)    :
*******************************************************************************************/
static unsigned long GetDriverVersion( unsigned char *pDataArr )
{
  P_SOFTWARE_VER pstDriverVer;

  if( ( void* )0!=pDataArr )
    pstDriverVer = ( P_SOFTWARE_VER )pDataArr;
  else
    return 0;

  pstDriverVer->stMainVer.main = DRIVER_VERSION_MAIN;
  pstDriverVer->stMainVer.sub  = DRIVER_VERSION_SECONDRY;
  pstDriverVer->revision = DRIVER_VERSION_REVISION;
  pstDriverVer->interior = DRIVER_VERSION_INTERIOR;

  return sizeof( SOFTWARE_VER );
}

/*******************************************************************************************
* Func.  Name: GetDriverVersion
* Description: 获取Firmware软件的版本号
* Arguments  :
*      Inputs: 存储版本信息的数据缓存
*     Outputs: 数据长度
* Note(s)    :
*******************************************************************************************/
static unsigned long GetFirmwareVersion( unsigned char *pDataArr )
{
  P_SOFTWARE_VER pstFirmwareVer;

  if( ( void* )0!=pDataArr )
    pstFirmwareVer = ( P_SOFTWARE_VER )pDataArr;
  else
    return 0;

  pstFirmwareVer->stMainVer.main = FIRMWARE_VERSION_MAIN;
  pstFirmwareVer->stMainVer.sub  = FIRMWARE_VERSION_SECONDRY;
  pstFirmwareVer->revision = FIRMWARE_VERSION_REVISION;
  pstFirmwareVer->interior = FIRMWARE_VERSION_INTERIOR;

  return sizeof( SOFTWARE_VER );
}

/*******************************************************************************************
* Func.  Name: GetConfigVersion
* Description: 获取配置软件的版本号
* Arguments  :
*      Inputs: 存储版本信息的数据缓存
*     Outputs: 数据长度
* Note(s)    :
*******************************************************************************************/
static unsigned long GetConfigVersion( unsigned char *pDataArr )
{
  P_MAIN_SUB_VER pstConfigVer;

  if( ( void* )0!=pDataArr )
    pstConfigVer = ( P_MAIN_SUB_VER )pDataArr;
  else
    return 0;

  pstConfigVer->main = CONFIG_VERSION_MAIN;
  pstConfigVer->sub  = CONFIG_VERSION_SECONDRY;

  return sizeof( MAIN_SUB_VER );
}

/*******************************************************************************************
* Func.  Name: GetProtocolVersion
* Description: 获取通讯协议的版本号
* Arguments  :
*      Inputs: 存储版本信息的数据缓存
*     Outputs: 数据长度
* Note(s)    :
*******************************************************************************************/
static unsigned long GetProtocolVersion( unsigned char *pDataArr )
{
  P_MAIN_SUB_VER pstProtocolVer;

  if( ( void* )0!=pDataArr )
    pstProtocolVer = ( P_MAIN_SUB_VER )pDataArr;
  else
    return 0;

  pstProtocolVer->main = PROTOCOL_VERSION_MAIN;
  pstProtocolVer->sub  = PROTOCOL_VERSION_SECONDRY;

  return sizeof( MAIN_SUB_VER );
}

/*******************************************************************************************
* Func.  Name: GetSoftwareVersion
* Description: 获取软件版本信息
* Arguments  :
*      Inputs: 存储版本信息的数据缓存,必须大于12字节
*     Outputs: 数据长度
* Note(s)    :
*******************************************************************************************/
unsigned long GetSoftwareVersion( unsigned char *pDataArr )
{
  unsigned char DataLen;

  DataLen = 0;

  /*	DataLen += GetDriverVersion(pDataArr+DataLen);

  	// 执行第一个版本信息查询后,防止未定义指针调用
  	if( 0==DataLen )
  		return DataLen;
  */
  DataLen += GetFirmwareVersion( pDataArr+DataLen );

  // 执行第一个版本信息查询后,防止未定义指针调用
  if( 0==DataLen )
    return DataLen;

  DataLen += GetProtocolVersion( pDataArr+DataLen );

  DataLen += GetConfigVersion( pDataArr+DataLen );

  return DataLen;
}

/*******************************************************************************************
* Func.  Name: SetBoardInfo
* Description: 设置硬件板卡信息
* Arguments  :
*      Inputs: 硬件板卡信息的数据缓存
*     Outputs: 数据长度
* Note(s)    :
*******************************************************************************************/
unsigned long SetBoardInfo( unsigned char *pDataArr, unsigned char len )
{
  unsigned long bStat = 1;

  //写入EEPROM

  return bStat;
}

/*******************************************************************************************
* Func.  Name: GetBoardInformation
* Description: 获取硬件板卡信息
* Arguments  :
*      Inputs: 存储硬件板卡信息的数据缓存
*     Outputs: 数据长度
* Note(s)    :
*******************************************************************************************/
unsigned long GetBoardInformation( unsigned char *pDataArr )
{
  unsigned char DataLen;

  DataLen = 0;

  DataLen += GetDriverVersion( pDataArr+DataLen );

  // 执行第一个版本信息查询后,防止未定义指针调用
  if( 0==DataLen )
    return DataLen;

  return DataLen;
}

