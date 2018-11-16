#include "stm32f4xx_hal.h"
#include "cycle_queue.h"
#include "cmsis_os.h"
#include "uart_driver.h"

extern UART_HandleTypeDef huart4;
extern DMA_HandleTypeDef hdma_uart4_rx;
extern DMA_HandleTypeDef hdma_uart4_tx;
SeqCQueue   seqCQueue;
#define DEBUG_INFO(fmt,args...)  printf(fmt, ##args)

/*-------------------private variable----------------------------------*/
DataType cache;
USART_RECEIVETYPE UsartType;

/*-------------------public function----------------------------------*/

void uart2_init()
{
	 HAL_UART_Receive_DMA( &huart3, seqCQueue.currentCache, LEFTRAMSIZE );   
}
void MX_UART_Config( UART_HandleTypeDef *huart, USART_TypeDef *UART, int baud ){
  
  DEBUG_INFO("Change the baud rate:%d\r\n", baud );
  
//  UART_REINIT:
//  
//  HAL_UART_DeInit( huart );

//  huart->Instance = UART;
//  huart->Init.BaudRate = baud;
//  huart->Init.WordLength = UART_WORDLENGTH_8B;
//  huart->Init.StopBits = UART_STOPBITS_1;
//  huart->Init.Parity = UART_PARITY_NONE;
//  huart->Init.Mode = UART_MODE_TX_RX;
//  huart->Init.HwFlowCtl = UART_HWCONTROL_NONE;
//  huart->Init.OverSampling = UART_OVERSAMPLING_16;

//  if( HAL_UART_Init(huart) != HAL_OK ){
//      DEBUG_INFO("initialization failed\r\n" );
//      osDelay( 1000 );
//      goto UART_REINIT;
//  }
//  
//  HAL_UART_Receive_DMA( huart, seqCQueue.currentCache, LEFTRAMSIZE );
//  
//  __HAL_UART_ENABLE_IT(huart,UART_IT_IDLE);
//  
//  if( UART == UART4 ){
//      
//      /* UART3_IRQn interrupt configuration */
//      HAL_NVIC_SetPriority(UART4_IRQn, 5, 0);           //add code
//      HAL_NVIC_EnableIRQ(UART4_IRQn);

//      UART4->SR;
//      
//      HAL_NVIC_SetPriority(DMA2_Channel3_IRQn, 5, 0); //add code
//      HAL_NVIC_EnableIRQ(DMA2_Channel3_IRQn);
//      
//      HAL_NVIC_SetPriority(DMA2_Channel4_5_IRQn, 5, 0);   //add code
//      HAL_NVIC_EnableIRQ(DMA2_Channel4_5_IRQn);
//      
//  }else{
//    
//  }

}

bool UART_GET_TX_STATE( void ){
	return ( UsartType.Send_flag == USART_TCPSENDOVER );
}

void UART_CLEAR_TX_STATE( void ){
	UsartType.Send_flag = USART_TCPSENDOVER;
}

void xHAL_UART_TxCpltCallback( void ){
    UsartType.Send_flag = USART_TCPSENDOVER;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
    UsartType.Send_flag = USART_TCPSENDOVER;
}

void InitPeripherals( void ){
      QueueInitiate( &seqCQueue );
      UsartType.Send_flag = USART_TCPSENDOVER;
}

void UartDmaSendData( UART_HandleTypeDef* huart, uint8_t *pdata, uint16_t Length ){

    if( Length >= 1024 && Length != 0 ){
        DEBUG_INFO("Uart4SendData:SEND MSG More than valid length: %d\r\n", Length );
        return ;
    }
    
    int16_t i = 0;
    while( UsartType.Send_flag == USART_TCPSENDING ){
        if( ++i > 500 ){
            HAL_UART_AbortTransmit( huart );
            DEBUG_INFO("\r\n$$$$$$$$$$$$$$$DMA TIMEOUT ERROR$$$$$$$$$$$$$$$\r\n");
            break;
        }else{
            osDelay(1);
        }
    };
    
    UsartType.Send_flag = USART_TCPSENDING;
    
    if( HAL_UART_Transmit_DMA( huart, pdata, Length) == HAL_BUSY ){
//        osDelay( 50 );
//        DEBUG_INFO("DMA Transmit BUSY\r\n");
//        HAL_UART_DMAResume( huart );
//        HAL_UART_Receive_DMA( huart, seqCQueue.currentCache, LEFTRAMSIZE	);
    }
}

void UartIdleReceiveData( UART_HandleTypeDef* huart ){
    
    if( ( __HAL_UART_GET_FLAG( huart,UART_FLAG_IDLE ) != RESET) ){
        uint32_t temp = 0;
        __HAL_UART_CLEAR_IDLEFLAG(huart);
        HAL_UART_DMAStop(huart);
        temp = huart->hdmarx->Instance->NDTR;
		if( ( LEFTRAMSIZE - temp ) != 0x00 && ( LEFTRAMSIZE - temp ) < LEFTRAMSIZE ){
            cache.size  = ( LEFTRAMSIZE - temp ) ;
            cache.index = seqCQueue.currentCache;
            if( QueueAppend( &seqCQueue, cache ) ){
                seqCQueue.leftram += cache.size;
                if( ( RECEIVEBUFLEN - seqCQueue.leftram ) >= LEFTRAMSIZE ){
                    seqCQueue.currentCache += cache.size;
                }else{
                    seqCQueue.leftram = 0;
                    seqCQueue.currentCache = seqCQueue.heapcache;
                }
            }
		}else{
            HAL_UART_DMAResume( huart );
        }
		
		HAL_UART_Receive_DMA( huart, seqCQueue.currentCache, LEFTRAMSIZE	);
    }
}

void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART3_IRQn 0 */

  /* USER CODE END USART3_IRQn 0 */
  HAL_UART_IRQHandler(&huart2);
  /* USER CODE BEGIN USART3_IRQn 1 */
    void UartIdleReceiveData( UART_HandleTypeDef* huart );
    UartIdleReceiveData( &huart2 );
  /* USER CODE END USART3_IRQn 1 */
}

void USART3_IRQHandler(void)
{
  /* USER CODE BEGIN USART3_IRQn 0 */

  /* USER CODE END USART3_IRQn 0 */
  HAL_UART_IRQHandler(&huart3);
  /* USER CODE BEGIN USART3_IRQn 1 */
    void UartIdleReceiveData( UART_HandleTypeDef* huart );
    UartIdleReceiveData( &huart3 );
  /* USER CODE END USART3_IRQn 1 */
}
/********************************************************************************/


