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

/**
  * @brief  Inserts a delay time.
  * @param  nTime: specifies the delay time length, in 10 ms.
  * @retval None
  */
void Delay(__IO uint32_t nTime)
{
  /*
	TimingDelay = nTime;

  while(TimingDelay != 0);
	*/
}

void initRTC() {
	RTC_InitTypeDef rtc_InitStructure;
	
	/* Enable the PWR clock */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);
	
	/* Allow access to RTC */
	PWR_BackupAccessCmd(ENABLE);
	
  /* Reset RTC Domain */
  RCC_BackupResetCmd(ENABLE);
  RCC_BackupResetCmd(DISABLE);	
	
	/* Select the RTC Clock Source */
	RCC_RTCCLKConfig(RCC_RTCCLKSource_HSE_Div8);
	
	/* Enable the RTC Clock */
	RCC_RTCCLKCmd(ENABLE);

	/* Wait for RTC APB registers synchronisation */
	RTC_WaitForSynchro();
	
	/* Calendar Configuration with HSE supposed at 1MHz */
	rtc_InitStructure.RTC_AsynchPrediv = 0x7F; /* 1 000 000 / 128 */
	rtc_InitStructure.RTC_SynchPrediv  = 0x2000; /* (1 000 000 / 128) / 8 192 = 0.9536*/
	rtc_InitStructure.RTC_HourFormat = RTC_HourFormat_24;
	RTC_Init(&rtc_InitStructure);  
}

void initAlarm() {
	EXTI_InitTypeDef EXTI_InitStructure;
	RTC_AlarmTypeDef RTC_AlarmStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
  
	/* EXTI configuration */
	EXTI_ClearITPendingBit(EXTI_Line17);
	EXTI_InitStructure.EXTI_Line = EXTI_Line17;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	/* Enable the RTC Alarm Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = RTC_Alarm_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
  
	/* Set the alarm A Masks */
	RTC_AlarmStructInit(&RTC_AlarmStructure);
	RTC_AlarmStructure.RTC_AlarmMask = RTC_AlarmMask_All;
	RTC_SetAlarm(RTC_Format_BCD, RTC_Alarm_A, &RTC_AlarmStructure);
   
	/* Set alarm A sub seconds and enable SubSec Alarm : generate 8 interrupts per Second */
	//RTC_AlarmSubSecondConfig(RTC_Alarm_A, 0xFF, RTC_AlarmSubSecondMask_SS14_5);
	
	RTC_AlarmCmd(RTC_Alarm_A, ENABLE);
 
	/* Enable Alarm A interrupt */
	RTC_ITConfig(RTC_IT_ALRA, ENABLE);
}

void initTIM1() {
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	/* TIM1 clock enable */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
	
	/* Time base configuration */
	TIM_TimeBaseStructure.TIM_Prescaler = 65535;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Period = 1500;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);
	
	/* Enable TIM1 interrupt SOURCE */
	NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_TIM10_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);	
	
	/* TIM1 interrupt ACTION enable */
	TIM_ITConfig(TIM11, TIM_IT_Update, ENABLE);
}

/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
int main(void)
{
	/* Initialize LEDs and User_Button on STM32F4-Discovery --------------------*/
	GPIO_InitTypeDef  GPIO_InitStructure;
	
  /* Enable the GPIO_LED Clock */
  RCC_AHB1PeriphClockCmd(LED4_GPIO_CLK, ENABLE);
	RCC_AHB1PeriphClockCmd(LED5_GPIO_CLK, ENABLE);

  /* Configure the GPIO_LED pin */
  GPIO_InitStructure.GPIO_Pin = LED4_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(LED4_GPIO_PORT, &GPIO_InitStructure);
	
  GPIO_InitStructure.GPIO_Pin = LED5_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(LED5_GPIO_PORT, &GPIO_InitStructure);
	
	initRTC();
	initAlarm();	
	
	initTIM1();
	
	while(1) {
	}
}
