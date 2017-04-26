#ifndef BIBLIOTEKA_H
#define BIBLIOTEKA_H


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

//do DMA (p2m)
#define ADC_1_ADDRESS_BASE 0x40012000
// ADC_DR = ADC regular Data Register
#define ADC_DR_ADDRESS_OFFSET 0x4C

//do DMA (m2p)
#define DAC_CHANNEL_1_ADDRESS_BASE 0x40007400
// DAC_DHR12R1 = DAC Data Holding Register 12 bits, Right aligned channel 1
#define DAC_DHR12R1_ADDRESS_OFFSET 0x08
#define DMA_DAC_SIGNAL_SIZE 32



GPIO_InitTypeDef  GPIO_InitStructure;
TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure; //struktura konfiguracyjna timerow
TIM_OCInitTypeDef TIM_OCInitStructure; // do PWM
NVIC_InitTypeDef NVIC_InitStructure; // przerwania wewnetrzne (np. timer)
EXTI_InitTypeDef EXTI_InitStructure; // przerwania zewnetrzne (np. przycisk)
ADC_CommonInitTypeDef ADC_CommonInitStructure; // ADC (wspólna konfiguracja)
ADC_InitTypeDef ADC_InitStructure; // ADC (konfiguracja danego przetwornika)
DAC_InitTypeDef DAC_InitStructure; // DAC
USART_InitTypeDef USART_InitStructure; // USART

void Delay(unsigned long int);
void LED_init();
void Button_init();
void TIM3_init(int, int);
void TIM4_init(int, int);
void PWM_init(int);
void TIM3_inter();
void Button_inter();
void ADC_init();
void DAC_init();
void USART_init(void);
void usartSendData(uint8_t);
uint8_t usartGetChar1(void);
uint8_t usartGetChar2(void);
void USART_inter(void);
void spi_init(int);//1-master, reszta slave
void MY_DMA_initP2M();
void MY_ADC_init(void);
void MY_DMA_initM2P(void);
void MY_DAC_init(void);
void MY_DAC_initTimerForUpdating(void);
void i2c_init();
void I2C_start(I2C_TypeDef* I2Cx, uint8_t address, uint8_t direction);
void I2C_stop(I2C_TypeDef* I2Cx);
void I2C_write(I2C_TypeDef* I2Cx, uint8_t data);
uint8_t I2C_read(I2C_TypeDef* I2Cx, uint ACK);

#endif
