/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    19-September-2011
  * @brief   Main Interrupt Service Routines.
  *          This file provides all exceptions handler and peripherals interrupt
  *          service routine.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_it.h"
#include "main.h"

extern __IO uint8_t speedDetectStart;
extern __IO uint32_t delay;

extern __IO const char * txPointer;
extern __IO const char * txBuffer;
extern __IO char * rxPointer;
extern __IO char * rxBuffer;


/******************************************************************************/
/*            Cortex-M4 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief   This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

__IO char cc = 0;

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
	if(speedDetectStart == 1) {
		if(delay > 0) {
			delay--;
		} else {
			//restart sending initialise AT command
			delay = 5000;
			cc++;
			
			if(rxBuffer != NULL) {
				delete rxBuffer;
			}
			rxBuffer = new char[50];
			
			switch(cc) {
				default:
					txBuffer = "AT\r";
			}
			
			rxPointer = rxBuffer;
			txPointer = txBuffer;
			
			//start transmission
			USART_ITConfig(USART, USART_IT_RXNE, ENABLE);
			
			USART_ITConfig(USART, USART_IT_TXE, ENABLE);			
		}
	}
}

/******************************************************************************/
/*                 STM32F4xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f4xx.s).                                               */
/******************************************************************************/

/**
  * @brief  This function handles EXTI0_IRQ Handler.
  * @param  None
  * @retval None
  */
void EXTI0_IRQHandler() {
	if(EXTI_GetITStatus(USER_BUTTON_EXTI_LINE) == SET) {

		USART3_DTR_GPIO_PORT->BSRRH = USART3_DTR_GPIO_PIN;
		USART_ITConfig(USART, USART_IT_ERR, ENABLE);
		
		USART_ITConfig(USART, USART_IT_RXNE, ENABLE);
		
		speedDetectStart = 1;
		disabelAllLeds();
		
		/* Clear the EXTI line pending bit */
		EXTI_ClearITPendingBit(USER_BUTTON_EXTI_LINE);
	}
}

/**
  * @brief  This function handles EXTI15_10_IRQ Handler.
  * @param  None
  * @retval None
  */
void EXTI15_10_IRQHandler() {
	if(EXTI_GetITStatus(USART3_DCD_EXTI_LINE) == SET) {
		/* Clear the EXTI line pending bit */
		EXTI_ClearITPendingBit(USART3_DCD_EXTI_LINE);
	}
	
	if(EXTI_GetITStatus(USART3_RI_EXTI_LINE) == SET) {
		/* Clear the EXTI line pending bit */
		EXTI_ClearITPendingBit(USART3_RI_EXTI_LINE);
	}
}

/**
* @brief  This function handles USRAT interrupt request.
* @param  None
* @retval None
*/
void USART3_IRQHandler(void)
{
		/* USART in Receiver mode */
		if (USART_GetITStatus(USART, USART_IT_RXNE) == SET)
		{
			if (rxPointer - rxBuffer < 9)
			{
				//indicate RX process
				enableLed(LED_RX_PROGRESS_GPIO_PORT, LED_RX_PROGRESS_GPIO_PIN);
								
				//receive Transaction data
				*rxPointer = USART_ReceiveData(USART);
				rxPointer++;
				
				USART_ClearITPendingBit(USART, USART_IT_RXNE);
			}
			
			if(rxPointer - rxBuffer == 9)
			{
				*rxPointer = '\0';
				
				disableLed(LED_RX_PROGRESS_GPIO_PORT, LED_RX_PROGRESS_GPIO_PIN);
				USART_ITConfig(USART, USART_IT_RXNE, DISABLE);
				
				//compare to correct responce
				char * crPointer = "AT\r\r\nOK\r\n";
				rxPointer = rxBuffer;
				while(*rxPointer == *crPointer && *crPointer != '\0'){
					crPointer++;
					rxPointer++;
				}
				
				if(*crPointer == '\0') {
					enableLed(LED_AT_PASS_GPIO_PORT, LED_AT_PASS_GPIO_PIN);
					speedDetectStart = 0;
				}
			}
			
			return;
		}

		/* USART in Tramitter mode */
		if (USART_GetITStatus(USART, USART_IT_TXE) == SET
			&& USART_GetITStatus(USART, USART_IT_TC) != SET)
		{
			if ((*txPointer) != '\0')
			{
				//indicate TX process
				enableLed(LED_TX_PROGRESS_GPIO_PORT, LED_TX_PROGRESS_GPIO_PIN);
				
				//send Transaction data
				USART_SendData(USART, *txPointer);
				txPointer++;
			}
			else
			{
				//wait until transfer completted, see USART_IT_TC
				USART_ITConfig(USART, USART_IT_TC, ENABLE);
			}
			
			return;
		}
		
		/* USART Tramition completed */
		if (USART_GetITStatus(USART, USART_IT_TC) == SET)
		{
			//check that was sent correct count of chars
			if((*txPointer) != '\0') {
				enableLed(LED_AT_FAIL_GPIO_PORT, LED_AT_FAIL_GPIO_PIN);
				speedDetectStart = 0;
			}
			
			//disable transmission mode
			disableLed(LED_TX_PROGRESS_GPIO_PORT, LED_TX_PROGRESS_GPIO_PIN);
			USART_ITConfig(USART, USART_IT_TXE, DISABLE);
			
			USART_ClearFlag(USART3, USART_IT_TC);
			USART_ITConfig(USART, USART_IT_TC, DISABLE);
				
			//start receive responce
			USART_ITConfig(USART, USART_IT_RXNE, ENABLE);
			
			return;
		}
		
		/* USART error detected */
		if(USART_GetITStatus(USART, USART_IT_ERR) == SET) {
			if(USART_GetFlagStatus(USART, USART_FLAG_ORE) == SET) {
				speedDetectStart = 0;
			} else if (USART_GetFlagStatus(USART, USART_FLAG_NE) == SET) {
				speedDetectStart = 0;
			} else if (USART_GetFlagStatus(USART, USART_FLAG_FE) == SET) {
				speedDetectStart = 0;
			} else if (USART_GetFlagStatus(USART, USART_FLAG_PE) == SET) {
				speedDetectStart = 0;
			}
				
			enableLed(LED_AT_FAIL_GPIO_PORT, LED_AT_FAIL_GPIO_PIN);
			
			USART_ClearFlag(USART3, USART_IT_ERR);
		}
}
