#include "interrupts.h"  

#include "DMA.h"

/* 
    This library contains all interrupts. If any interrupt is not alrady included in here, add it at an
    appropriate location and label it. The interrupts are not included in the .cpp files because the 
    interrupts seem to only work in .c files.  Add the code directly into the file for whatever purpose
    you need.
*/


/*DMA1 interrupts */

/* USART5 Rx */
void DMA1_Stream0_IRQHandler(void)  
{
	/* Test on DMA Stream Transfer Complete interrupt */
	if (DMA_GetITStatus(DMA1_Stream2, DMA_IT_TCIF2))
	{
		DMA_DeInit(DMA1_Stream1);
		
		DMA_Init(DMA1_Stream2, &RxStruct[0][0]);
		
		USART_DMACmd(UART5, USART_DMAReq_Rx, ENABLE);
		
		DMA_ITConfig(DMA1_Stream0, DMA_IT_TC, ENABLE);
		
		DMA_Cmd(DMA1_Stream0, ENABLE);

		DMA_ClearITPendingBit(DMA1_Stream0, DMA_IT_TCIF2);
	}
}

/* USART3 Rx */
void DMA1_Stream1_IRQHandler(void)
{
	/* Test on DMA Stream Transfer Complete interrupt */
	if (DMA_GetITStatus(DMA1_Stream2, DMA_IT_TCIF2))
	{
		DMA_DeInit(DMA1_Stream1);
		
		DMA_Init(DMA1_Stream2, &RxStruct[0][1]);
		
		USART_DMACmd(USART3, USART_DMAReq_Rx, ENABLE);
		
		DMA_ITConfig(DMA1_Stream1, DMA_IT_TC, ENABLE);
		
		DMA_Cmd(DMA1_Stream1, ENABLE);

		DMA_ClearITPendingBit(DMA1_Stream1, DMA_IT_TCIF2);
	}
}

/* UART4 Rx;  I2C_2 Rx;  I2C 3 Rx */
void DMA1_Stream2_IRQHandler(void)
{
	
	 /* Test on DMA Stream Transfer Complete interrupt */
	if (DMA_GetITStatus(DMA1_Stream2, DMA_IT_TCIF2))
	{
		DMA_DeInit(DMA1_Stream2);
		
		DMA_Init(DMA1_Stream2, &RxStruct[0][2]);
		
		USART_DMACmd(UART4, USART_DMAReq_Rx, ENABLE);
		
		DMA_ITConfig(DMA1_Stream2, DMA_IT_TC, ENABLE);
		
		DMA_Cmd(DMA1_Stream2, ENABLE);

		DMA_ClearITPendingBit(DMA1_Stream2, DMA_IT_TCIF2);
	}
}

/*USART3 Rx */
void DMA1_Stream3_IRQHandler(void)
{
	
}

/* UART4 Tx;  USART3 Tx */
void DMA1_Stream4_IRQHandler(void)
{
	
}

/* USART2 Rx */
void DMA1_Stream5_IRQHandler(void)
{
	/* Test on DMA Stream Transfer Complete interrupt */
	if (DMA_GetITStatus(DMA1_Stream2, DMA_IT_TCIF2))
	{
		DMA_DeInit(DMA1_Stream5);
		
		DMA_Init(DMA1_Stream2, &RxStruct[0][5]);
		
		USART_DMACmd(USART2, USART_DMAReq_Rx, ENABLE);
		
		DMA_ITConfig(DMA1_Stream5, DMA_IT_TC, ENABLE);
		
		DMA_Cmd(DMA1_Stream5, ENABLE);

		DMA_ClearITPendingBit(DMA1_Stream5, DMA_IT_TCIF2);
	}
}

/* USART2 Tx */
void DMA1_Stream6_IRQHandler(void)
{
	
}

/* UART5 Tx */
void DMA1_Stream7_IRQHandler(void)
{
	
}



/* DMA2 interrupts */


void DMA2_Stream0_IRQHandler(void)
{
	
}

void DMA2_Stream1_IRQHandler(void)
{
	/* Test on DMA Stream Transfer Complete interrupt */
	if (DMA_GetITStatus(DMA1_Stream2, DMA_IT_TCIF2))
	{
		DMA_DeInit(DMA2_Stream1);
		
		DMA_Init(DMA2_Stream2, &RxStruct[1][1]);
		
		USART_DMACmd(USART6, USART_DMAReq_Rx, ENABLE);
		
		DMA_ITConfig(DMA2_Stream1, DMA_IT_TC, ENABLE);
		
		DMA_Cmd(DMA2_Stream1, ENABLE);

		DMA_ClearITPendingBit(DMA2_Stream1, DMA_IT_TCIF2);
	}
}

void DMA2_Stream2_IRQHandler(void)
{
	
}

void DMA2_Stream3_IRQHandler(void)
{
	
}

void DMA2_Stream4_IRQHandler(void)
{
	
}
void DMA2_Stream5_IRQHandler(void)
{
	/* Test on DMA Stream Transfer Complete interrupt */
	if (DMA_GetITStatus(DMA1_Stream2, DMA_IT_TCIF2))
	{
		DMA_DeInit(DMA2_Stream5);
		
		DMA_Init(DMA2_Stream5, &RxStruct[1][5]);
		
		USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);
		
		DMA_ITConfig(DMA2_Stream5, DMA_IT_TC, ENABLE);
		
		DMA_Cmd(DMA2_Stream5, ENABLE);

		DMA_ClearITPendingBit(DMA2_Stream5, DMA_IT_TCIF2);
	}
}

void DMA2_Stream6_IRQHandler(void)
{
	
}

void DMA2_Stream7_IRQHandler(void)
{
	
}