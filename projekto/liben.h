#ifndef LIBEN_H
#define LIBEN_H


#include "stm32f4xx_gpio.h"
#include "stm32f4xx_conf.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_exti.h"
#include "misc.h"
#include "stm32f4xx_syscfg.h"
#include "stm32f4xx_adc.h"
#include "stm32f4xx_dac.h"
#include "stm32f4xx_usart.h"
#include <stdio.h>
#include "stm32f4xx_spi.h"
#include "stm32f4xx_dma.h"
#include "stm32f4xx_i2c.h"



GPIO_InitTypeDef  GPIO_InitStructure;
TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure; //struktura konfiguracyjna timerow
TIM_OCInitTypeDef TIM_OCInitStructure; // do PWM
NVIC_InitTypeDef NVIC_InitStructure; // przerwania wewnetrzne (np. timer)
EXTI_InitTypeDef EXTI_InitStructure; // przerwania zewnetrzne (np. przycisk)
ADC_CommonInitTypeDef ADC_CommonInitStructure; // ADC (wspólna konfiguracja)
ADC_InitTypeDef ADC_InitStructure; // ADC (konfiguracja danego przetwornika)
DAC_InitTypeDef DAC_InitStructure; // DAC
USART_InitTypeDef USART_InitStructure; // USART

void ADC1_init();

void ADC2_init();




#endif
