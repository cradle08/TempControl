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
#include  "loopbuff4b.h"

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


bool initQueue4b( Queue4b *q, void *pBuff, s32 size )
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


bool enQueue4b( Queue4b *q, s32 key )
{
  if( q == NULL )
  {
    return false;
  }
  int tail = ( q->tail+1 ) % q->queueSize;
  if( tail == q->head )
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


s32 deQueue4b( Queue4b *q )
{
  s32 tmp = 0;

  if( q->tail == q->head )
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


bool isQueueEmpty4b( Queue4b *q )
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


bool isQueueFull4b( Queue4b *q )
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


s32 queueEmptyN4b( Queue4b *q )
{
  u32 len;
  if( isQueueEmpty4b( q ) )
  {
    return q->queueSize - 1;
  }
  else
  {
    len = ( q->head + q->queueSize - q->tail - 1 ) % ( q->queueSize - 1 );
    return len;
  }
}


s32 queueFullN4b( Queue4b *q )
{
  u32 len;
  if( isQueueFull4b( q ) )
  {
    return q->queueSize - 1;
  }
  else
  {
    len = ( q->tail + q->queueSize - q->head ) % q->queueSize;
    return len;
  }
}



