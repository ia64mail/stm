/**
  ******************************************************************************
  * @file    main.c 
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    19-September-2011
  * @brief   Main program body
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
#include "main.h"

__IO uint8_t speedDetectStart = 0;
__IO uint32_t delay = 0;

__IO const char * txBuffer;
__IO const char * txPointer;

__IO char * rxBuffer;
__IO char * rxPointer;

/**
  * @brief  Initialse USART3 perirherial and related IO pins
  * @param  None
  * @retval None
  */
void initUSART3(){
	GPIO_InitTypeDef GPIO_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	/* Enable SYSCFG bus */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
	
	/* RX input pin */
	RCC_APB1PeriphClockCmd(USART3_RX_GPIO_BUS, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin 	= USART3_RX_GPIO_PIN;
	GPIO_InitStructure.GPIO_Mode 	= USART3_RX_GPIO_MODE;
	GPIO_InitStructure.GPIO_Speed	= USART3_RX_GPIO_SPEED;
	GPIO_InitStructure.GPIO_OType = USART3_RX_GPIO_OTYPE;
	GPIO_InitStructure.GPIO_PuPd 	= USART3_RX_GPIO_PTYPE;
	GPIO_Init(USART3_RX_GPIO_PORT, &GPIO_InitStructure);
 
	GPIO_PinAFConfig(USART3_RX_GPIO_PORT, USART3_RX_GPIO_PIN_SOURCE, USART3_RX_GPIO_AF_MODE);	
	
	/* CTS input pin */
	RCC_APB1PeriphClockCmd(USART3_CTS_GPIO_BUS, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin 	= USART3_CTS_GPIO_PIN;
	GPIO_InitStructure.GPIO_Mode 	= USART3_CTS_GPIO_MODE;
	GPIO_InitStructure.GPIO_Speed	= USART3_CTS_GPIO_SPEED;
	GPIO_InitStructure.GPIO_OType = USART3_CTS_GPIO_OTYPE;
	GPIO_InitStructure.GPIO_PuPd 	= USART3_CTS_GPIO_PTYPE;
	GPIO_Init(USART3_CTS_GPIO_PORT, &GPIO_InitStructure);
 
	GPIO_PinAFConfig(USART3_CTS_GPIO_PORT, USART3_CTS_GPIO_PIN_SOURCE, USART3_CTS_GPIO_AF_MODE);	
	
	/* DCD input pin */
	RCC_AHB1PeriphClockCmd(USART3_DCD_GPIO_BUS, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin 	= USART3_DCD_GPIO_PIN;
	GPIO_InitStructure.GPIO_Mode 	= USART3_DCD_GPIO_MODE;
	GPIO_InitStructure.GPIO_Speed	= USART3_DCD_GPIO_SPEED;
	GPIO_InitStructure.GPIO_OType = USART3_DCD_GPIO_OTYPE;
	GPIO_InitStructure.GPIO_PuPd 	= USART3_DCD_GPIO_PTYPE;
	GPIO_Init(USART3_DCD_GPIO_PORT, &GPIO_InitStructure);

	//enable DCD interrupt
  SYSCFG_EXTILineConfig(USART3_DCD_EXTI_PORT_SOURCE, USART3_DCD_EXTI_PIN_SOURCE);

  EXTI_InitStructure.EXTI_Line = USART3_DCD_EXTI_LINE;
  EXTI_InitStructure.EXTI_Mode = USART3_DCD_EXTI_MODE;
  EXTI_InitStructure.EXTI_Trigger = USART3_DCD_EXTI_TRIGER;  
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);

  NVIC_InitStructure.NVIC_IRQChannel = USART3_DCD_NVIC_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = USART3_DCD_NVIC_PRIORITY;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = USART3_DCD_NVIC_SUBPRIORITY;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure); 

	/* RI input pin */
	RCC_AHB1PeriphClockCmd(USART3_RI_GPIO_BUS, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin 	= USART3_RI_GPIO_PIN;
	GPIO_InitStructure.GPIO_Mode 	= USART3_RI_GPIO_MODE;
	GPIO_InitStructure.GPIO_Speed	= USART3_RI_GPIO_SPEED;
	GPIO_InitStructure.GPIO_OType = USART3_RI_GPIO_OTYPE;
	GPIO_InitStructure.GPIO_PuPd 	= USART3_RI_GPIO_PTYPE;
	GPIO_Init(USART3_RI_GPIO_PORT, &GPIO_InitStructure);
 
	//enable RI interrupt
  SYSCFG_EXTILineConfig(USART3_RI_EXTI_PORT_SOURCE, USART3_RI_EXTI_PIN_SOURCE);

  EXTI_InitStructure.EXTI_Line = USART3_RI_EXTI_LINE;
  EXTI_InitStructure.EXTI_Mode = USART3_RI_EXTI_MODE;
  EXTI_InitStructure.EXTI_Trigger = USART3_RI_EXTI_TRIGER;  
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);

  NVIC_InitStructure.NVIC_IRQChannel = USART3_RI_NVIC_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = USART3_RI_NVIC_PRIORITY;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = USART3_RI_NVIC_SUBPRIORITY;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure); 
	
	/* TX output pin */
	RCC_APB1PeriphClockCmd(USART3_TX_GPIO_BUS, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin 	= USART3_TX_GPIO_PIN;
	GPIO_InitStructure.GPIO_Mode 	= USART3_TX_GPIO_MODE;
	GPIO_InitStructure.GPIO_Speed	= USART3_TX_GPIO_SPEED;
	GPIO_InitStructure.GPIO_OType = USART3_TX_GPIO_OTYPE;
	GPIO_InitStructure.GPIO_PuPd 	= USART3_TX_GPIO_PTYPE;
	GPIO_Init(USART3_TX_GPIO_PORT, &GPIO_InitStructure);
 
	GPIO_PinAFConfig(USART3_TX_GPIO_PORT, USART3_TX_GPIO_PIN_SOURCE, USART3_TX_GPIO_AF_MODE);	
	
	/* RTS output pin */
	RCC_APB1PeriphClockCmd(USART3_RTS_GPIO_BUS, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin 	= USART3_RTS_GPIO_PIN;
	GPIO_InitStructure.GPIO_Mode 	= USART3_RTS_GPIO_MODE;
	GPIO_InitStructure.GPIO_Speed	= USART3_RTS_GPIO_SPEED;
	GPIO_InitStructure.GPIO_OType = USART3_RTS_GPIO_OTYPE;
	GPIO_InitStructure.GPIO_PuPd 	= USART3_RTS_GPIO_PTYPE;
	GPIO_Init(USART3_RTS_GPIO_PORT, &GPIO_InitStructure);
 
	GPIO_PinAFConfig(USART3_RTS_GPIO_PORT, USART3_RTS_GPIO_PIN_SOURCE, USART3_RTS_GPIO_AF_MODE);	

	/* DTR output pin */
	RCC_AHB1PeriphClockCmd(USART3_DTR_GPIO_BUS, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin 	= USART3_DTR_GPIO_PIN;
	GPIO_InitStructure.GPIO_Mode 	= USART3_DTR_GPIO_MODE;
	GPIO_InitStructure.GPIO_Speed	= USART3_DTR_GPIO_SPEED;
	GPIO_InitStructure.GPIO_OType = USART3_DTR_GPIO_OTYPE;
	GPIO_InitStructure.GPIO_PuPd 	= USART3_DTR_GPIO_PTYPE;
	GPIO_Init(USART3_DTR_GPIO_PORT, &GPIO_InitStructure);
	
	/* USART3 device */
	USART_InitTypeDef USART_InitStructure;

	RCC_APB1PeriphClockCmd(USART_BUS, ENABLE);
	
	//USART_OverSampling8Cmd(USART, ENABLE);  
	
	USART_InitStructure.USART_BaudRate = USART_BAUND_RATE;
	USART_InitStructure.USART_WordLength = USART_WORD_LENGHT;
	USART_InitStructure.USART_StopBits = USART_STOP_BIT;
	USART_InitStructure.USART_Parity = USART_PARITY;
	USART_InitStructure.USART_HardwareFlowControl = USART_FLOW_CONTROL;
	USART_InitStructure.USART_Mode = USART_OPERATION_MODE;
	USART_Init(USART, &USART_InitStructure);
	
	//enable usart interrupt
	NVIC_InitStructure.NVIC_IRQChannel = USART_NVIC_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = USART_NVIC_PRIORITY;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = USART_NVIC_SUBPRIORITY;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	//enable devisce
	USART_Cmd(USART, ENABLE);
}

/**
  * @brief Initialise user LED and button 
  * @param  None
  * @retval None
  */
void initUserControl() {
	GPIO_InitTypeDef GPIO_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	/* Enable SYSCFG bus */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
	
	/* RX operation in progress */
  RCC_AHB1PeriphClockCmd(LED_RX_PROGRESS_GPIO_BUS, ENABLE);
  
  GPIO_InitStructure.GPIO_Pin = LED_RX_PROGRESS_GPIO_PIN;
  GPIO_InitStructure.GPIO_Mode = LED_RX_PROGRESS_GPIO_MODE;
  GPIO_InitStructure.GPIO_OType = LED_RX_PROGRESS_GPIO_OTYPE;
  GPIO_InitStructure.GPIO_PuPd = LED_RX_PROGRESS_GPIO_PTYPE;
  GPIO_InitStructure.GPIO_Speed = LED_RX_PROGRESS_GPIO_SPEED;
  GPIO_Init(LED_RX_PROGRESS_GPIO_PORT, &GPIO_InitStructure);

	/* TX operation in progress */
  RCC_AHB1PeriphClockCmd(LED_TX_PROGRESS_GPIO_BUS, ENABLE);
  
  GPIO_InitStructure.GPIO_Pin = LED_TX_PROGRESS_GPIO_PIN;
  GPIO_InitStructure.GPIO_Mode = LED_TX_PROGRESS_GPIO_MODE;
  GPIO_InitStructure.GPIO_OType = LED_TX_PROGRESS_GPIO_OTYPE;
  GPIO_InitStructure.GPIO_PuPd = LED_TX_PROGRESS_GPIO_PTYPE;
  GPIO_InitStructure.GPIO_Speed = LED_TX_PROGRESS_GPIO_SPEED;
  GPIO_Init(LED_TX_PROGRESS_GPIO_PORT, &GPIO_InitStructure);

	/* AT operation Pass */
  RCC_AHB1PeriphClockCmd(LED_AT_PASS_GPIO_BUS, ENABLE);
  
  GPIO_InitStructure.GPIO_Pin = LED_AT_PASS_GPIO_PIN;
  GPIO_InitStructure.GPIO_Mode = LED_AT_PASS_GPIO_MODE;
  GPIO_InitStructure.GPIO_OType = LED_AT_PASS_GPIO_OTYPE;
  GPIO_InitStructure.GPIO_PuPd = LED_AT_PASS_GPIO_PTYPE;
  GPIO_InitStructure.GPIO_Speed = LED_AT_PASS_GPIO_SPEED;
  GPIO_Init(LED_AT_PASS_GPIO_PORT, &GPIO_InitStructure);

	/* AT operation Fail */
  RCC_AHB1PeriphClockCmd(LED_AT_FAIL_GPIO_BUS, ENABLE);
  
  GPIO_InitStructure.GPIO_Pin = LED_AT_FAIL_GPIO_PIN;
  GPIO_InitStructure.GPIO_Mode = LED_AT_FAIL_GPIO_MODE;
  GPIO_InitStructure.GPIO_OType = LED_AT_FAIL_GPIO_OTYPE;
  GPIO_InitStructure.GPIO_PuPd = LED_AT_FAIL_GPIO_PTYPE;
  GPIO_InitStructure.GPIO_Speed = LED_AT_FAIL_GPIO_SPEED;
  GPIO_Init(LED_AT_FAIL_GPIO_PORT, &GPIO_InitStructure);
	
	/* User button */
	RCC_AHB1PeriphClockCmd(USER_BUTTON_GPIO_BUS, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin 	= USER_BUTTON_GPIO_PIN;
	GPIO_InitStructure.GPIO_Mode 	= USER_BUTTON_GPIO_MODE;
	GPIO_InitStructure.GPIO_Speed	= USER_BUTTON_GPIO_SPEED;
	GPIO_InitStructure.GPIO_OType = USER_BUTTON_GPIO_OTYPE;
	GPIO_InitStructure.GPIO_PuPd 	= USER_BUTTON_GPIO_PTYPE;
	GPIO_Init(USER_BUTTON_GPIO_PORT, &GPIO_InitStructure);
 
	//enable push interrupt
  SYSCFG_EXTILineConfig(USER_BUTTON_EXTI_PORT_SOURCE, USER_BUTTON_EXTI_PIN_SOURCE);

  EXTI_InitStructure.EXTI_Line = USER_BUTTON_EXTI_LINE;
  EXTI_InitStructure.EXTI_Mode = USER_BUTTON_EXTI_MODE;
  EXTI_InitStructure.EXTI_Trigger = USER_BUTTON_EXTI_TRIGER;  
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);

  NVIC_InitStructure.NVIC_IRQChannel = USER_BUTTON_NVIC_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = USER_BUTTON_NVIC_PRIORITY;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = USER_BUTTON_NVIC_SUBPRIORITY;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure); 
}

/**
  * @brief Initialise systick 
  * @param  None
  * @retval None
  */
void configSysTick(){
	if (SysTick_Config(SystemCoreClock / 1000))
	{ 
		/* Capture error */ 
		while (1);
	}
}

void enableLed(GPIO_TypeDef * port, uint16_t pin) {
	port->BSRRL = pin;
}

void disableLed(GPIO_TypeDef * port, uint16_t pin) {
	port->BSRRH = pin;
}

void disabelAllLeds() {
	//assume that all led connected to same port
	disableLed(LED_AT_PASS_GPIO_PORT, 
		LED_RX_PROGRESS_GPIO_PIN | LED_AT_PASS_GPIO_PIN | LED_AT_FAIL_GPIO_PIN | LED_TX_PROGRESS_GPIO_PIN);
}

/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
int main(void)
{
	configSysTick();
	initUserControl();
	initUSART3();
	
	while(1) {
	}
}
