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

//初始化队列
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

//入队
bool enQueue( Queue *q, s8 key )
{
  if( q == NULL )
  {
    return false;
  }
  int tail = ( q->tail+1 ) % q->queueSize;  //取余保证，当quil=queuesize-1时，再转回0
  if( tail == q->head )                  		//此时队列没有空间
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

//出队
s8 deQueue( Queue *q )
{
  int tmp = 0;
  if( q->tail == q->head )   //判断队列不为空
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

//判断队列为空
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

//判断队列为满
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

//队列空的长度
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

//队列满的长度
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



