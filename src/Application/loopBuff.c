/**
  ******************************************************************************
  * @file    loopBuff.c
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
#include  <includes.h>


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

//��ʼ������
bool initQueue( Queue *q, void *pBuff, s32 size )
{
  if( q == NULL )
  {
    return false;
  }
  if( pBuff == NULL )
  {
    return false;
  }
  if( size <= 0 || size > QUEUE_MAX_SIZE )
  {
    return false;
  }
  q->queueSize = size;
  q->buff = pBuff;
  q->tail = 0;
  q->head = 0;

  return true;
}

//���
bool enQueue( Queue *q, s8 key )
{
  if( q == NULL )
  {
    return false;
  }
  int tail = ( q->tail+1 ) % q->queueSize;  //ȡ�ౣ֤����quil=queuesize-1ʱ����ת��0
  if( tail == q->head )                  		//��ʱ����û�пռ�
  {
#if DEBUG_EN
    //Debug_Printf("the queue has been filled full!");
#endif
    return false;
  }
  else
  {
    q->buff[q->tail] = key;
    q->tail = tail;
  }
  return true;
}

//����
s8 deQueue( Queue *q )
{
  int tmp = 0;
  if( q->tail == q->head )   //�ж϶��в�Ϊ��
  {
#if DEBUG_EN
    //Debug_Printf("the queue is NULL\n");
#endif
  }
  else
  {
    tmp = q->buff[q->head];
    q->head = ( q->head+1 ) % q->queueSize;
  }
  return tmp;
}

//�ж϶���Ϊ��
bool isQueueEmpty( Queue *q )
{
  if( q->head == q->tail )
  {
    return true;
  }
  else
  {
    return false;
  }
}

//�ж϶���Ϊ��
bool isQueueFull( Queue *q )
{
  if( ( q->tail + 1 ) % q->queueSize == q->head )
  {
#if DEBUG_EN
    //Debug_Printf("the queue has been filled full!\r\n");
#endif
    return true;
  }
  else
  {
    return false;
  }
}

//���пյĳ���
s32 queueEmptyN( Queue *q )
{
  u32 len;
  if( isQueueEmpty( q ) )
  {
    return q->queueSize - 1;
  }
  else
  {
    len = ( q->head + q->queueSize - q->tail - 1 ) % ( q->queueSize - 1 );
    return len;
  }
}

//�������ĳ���
s32 queueFullN( Queue *q )
{
  u32 len;
  if( isQueueFull( q ) )
  {
    return q->queueSize - 1;
  }
  else
  {
    len = ( q->tail + q->queueSize - q->head ) % q->queueSize;
    return len;
  }
}



