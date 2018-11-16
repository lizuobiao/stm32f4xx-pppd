#ifndef __CYCLEQUEUE__
#define __CYCLEQUEUE__
#include <stdint.h>
#include <stdbool.h>

#define USART_TCPSENDING   1
#define USART_TCPSENDOVER  0

#define MaxQueueSize 10             //25
#define LEFTRAMSIZE  640            //640
#define RECEIVEBUFLEN      2048     //2048
typedef struct DataType_t{
    uint8_t* index;
    int16_t  size;
}DataType;

typedef struct seq{
		DataType    queue[MaxQueueSize];
    uint8_t*    heapcache;
    uint8_t*    currentCache;
		int8_t     rear;
		int8_t     front;
		int8_t     count;
    int16_t    leftram;
    bool       reConnectState;
}SeqCQueue;

typedef struct uUSART_RECEIVETYPE_t{  
		uint8_t Send_flag:1;
		uint8_t usartDMA_rxBuf[RECEIVEBUFLEN]; 
}USART_RECEIVETYPE;

int QueueNotEmpty( SeqCQueue *Q );
void QueueInitiate( SeqCQueue* Q );
int QueueAppend( SeqCQueue* Q, DataType x );
int QueueDelete( SeqCQueue *Q, DataType *d );
extern SeqCQueue seqCQueue;
void GetQueueFront( void );

extern USART_RECEIVETYPE UsartType;


extern SeqCQueue   seqCQueue;
#endif











