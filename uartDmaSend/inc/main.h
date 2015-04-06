/**
  ******************************************************************************
  * @file    USART/USART_TwoBoards/USART_DataExchangeDMA/main.h
  * @author  MCD Application Team
  * @version V1.4.0
  * @date    04-August-2014
  * @brief   Header for main.c module
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2013 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
 extern "C" {
#endif 

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"

#if defined (USE_STM324xG_EVAL)
  #include "stm324xg_eval.h"

#elif defined (USE_STM324x7I_EVAL) 
  #include "stm324x7i_eval.h"

#elif defined (USE_STM324x9I_EVAL) 
  #include "stm324x9i_eval.h"

#else
 #error "Please select first the Evaluation board used in your application (in Project Options)"
#endif

/* Exported typedef ----------------------------------------------------------*/
typedef enum {FAILED = 0, PASSED = !FAILED} TestStatus;

    
  // XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
  // MODIFIED FOR THE DISCOVERY BOARD
  // XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX

  //Button defs
#define USER_BUTTON_PIN                   GPIO_Pin_0
#define USER_BUTTON_GPIO_PORT             GPIOA
#define USER_BUTTON_GPIO_CLK              RCC_AHB1Periph_GPIOA


  /* Definition for USARTx resources ******************************************/
  #define USART1_CLK                       RCC_APB2Periph_USART1
  #define USART1_CLK_INIT                  RCC_AHB1PeriphClockCmd
  #define USART1_IRQn                      USART1_IRQn
  #define USART1_IRQHandler                USART1_IRQHandler

  #define USART1_TX_PIN                    GPIO_Pin_6                
  #define USART1_TX_GPIO_PORT              GPIOB                       
  #define USART1_TX_GPIO_CLK               RCC_AHB1Periph_GPIOB
  #define USART1_TX_SOURCE                 GPIO_PinSource6
  #define USART1_TX_AF                     GPIO_AF_USART1

  #define USART1_RX_PIN                    GPIO_Pin_7                
  #define USART1_RX_GPIO_PORT              GPIOB                    
  #define USART1_RX_GPIO_CLK               RCC_AHB1Periph_GPIOB
  #define USART1_RX_SOURCE                 GPIO_PinSource7
  #define USART1_RX_AF                     GPIO_AF_USART1

  /* Definition for DMAx resources ********************************************/
  #define USART1_DR_ADDRESS                ((uint32_t)USART1 + 0x04) 

  #define USART1_DMA                       DMA2
  #define USART1_DMAx_CLK                  RCC_AHB1Periph_DMA2
     
  #define USART1_TX_DMA_CHANNEL            DMA_Channel_4
  #define USART1_TX_DMA_STREAM             DMA2_Stream7
  #define USART1_TX_DMA_FLAG_FEIF          DMA_FLAG_FEIF7
  #define USART1_TX_DMA_FLAG_DMEIF         DMA_FLAG_DMEIF7
  #define USART1_TX_DMA_FLAG_TEIF          DMA_FLAG_TEIF7
  #define USART1_TX_DMA_FLAG_HTIF          DMA_FLAG_HTIF7
  #define USART1_TX_DMA_FLAG_TCIF          DMA_FLAG_TCIF7
              
  #define USART1_RX_DMA_CHANNEL            DMA_Channel_4
  #define USART1_RX_DMA_STREAM             DMA2_Stream2
  #define USART1_RX_DMA_FLAG_FEIF          DMA_FLAG_FEIF2
  #define USART1_RX_DMA_FLAG_DMEIF         DMA_FLAG_DMEIF2
  #define USART1_RX_DMA_FLAG_TEIF          DMA_FLAG_TEIF2
  #define USART1_RX_DMA_FLAG_HTIF          DMA_FLAG_HTIF
  #define USART1_RX_DMA_FLAG_TCIF          DMA_FLAG_TCIF2

  #define USART1_DMA_TX_IRQn               DMA2_Stream7_IRQn
  #define USART1_DMA_RX_IRQn               DMA2_Stream2_IRQn
  #define USART1_DMA_TX_IRQHandler         DMA2_Stream7_IRQHandler
  #define USART1_DMA_RX_IRQHandler         DMA2_Stream2_IRQHandler

/* Misc definition ************************************************************/
/* Transmit buffer size */
#define BUFFERSIZE                       100

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
