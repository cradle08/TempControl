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
* Description: ��ȡƽ̨�����İ汾��
* Arguments  :
*      Inputs: �洢�汾��Ϣ�����ݻ���
*     Outputs: ���ݳ���
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
* Description: ��ȡFirmware�����İ汾��
* Arguments  :
*      Inputs: �洢�汾��Ϣ�����ݻ���
*     Outputs: ���ݳ���
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
* Description: ��ȡ���������İ汾��
* Arguments  :
*      Inputs: �洢�汾��Ϣ�����ݻ���
*     Outputs: ���ݳ���
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
* Description: ��ȡͨѶЭ��İ汾��
* Arguments  :
*      Inputs: �洢�汾��Ϣ�����ݻ���
*     Outputs: ���ݳ���
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
* Description: ��ȡ�����汾��Ϣ
* Arguments  :
*      Inputs: �洢�汾��Ϣ�����ݻ���,�������12�ֽ�
*     Outputs: ���ݳ���
* Note(s)    :
*******************************************************************************************/
unsigned long GetSoftwareVersion( unsigned char *pDataArr )
{
  unsigned char DataLen;

  DataLen = 0;

  /*	DataLen += GetDriverVersion(pDataArr+DataLen);

  	// ִ�е�һ���汾��Ϣ��ѯ��,��ֹδ����ָ�����
  	if( 0==DataLen )
  		return DataLen;
  */
  DataLen += GetFirmwareVersion( pDataArr+DataLen );

  // ִ�е�һ���汾��Ϣ��ѯ��,��ֹδ����ָ�����
  if( 0==DataLen )
    return DataLen;

  DataLen += GetProtocolVersion( pDataArr+DataLen );

  DataLen += GetConfigVersion( pDataArr+DataLen );

  return DataLen;
}

/*******************************************************************************************
* Func.  Name: SetBoardInfo
* Description: ����Ӳ���忨��Ϣ
* Arguments  :
*      Inputs: Ӳ���忨��Ϣ�����ݻ���
*     Outputs: ���ݳ���
* Note(s)    :
*******************************************************************************************/
unsigned long SetBoardInfo( unsigned char *pDataArr, unsigned char len )
{
  unsigned long bStat = 1;

  //д��EEPROM

  return bStat;
}

/*******************************************************************************************
* Func.  Name: GetBoardInformation
* Description: ��ȡӲ���忨��Ϣ
* Arguments  :
*      Inputs: �洢Ӳ���忨��Ϣ�����ݻ���
*     Outputs: ���ݳ���
* Note(s)    :
*******************************************************************************************/
unsigned long GetBoardInformation( unsigned char *pDataArr )
{
  unsigned char DataLen;

  DataLen = 0;

  DataLen += GetDriverVersion( pDataArr+DataLen );

  // ִ�е�һ���汾��Ϣ��ѯ��,��ֹδ����ָ�����
  if( 0==DataLen )
    return DataLen;

  return DataLen;
}
