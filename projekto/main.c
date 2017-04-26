#include "stm32f4xx_conf.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_adc.h"
#include "stm32f4xx_tim.h"
//#include "biblioteka.h"
//#include "tm_stm32f4_adc.h"
volatile double light;
volatile double calibration;
volatile double speed;
volatile double distance;
volatile double time;
volatile long long int ms=0;
volatile int first=0; //was this the first time the beam was interrupted?
volatile int grace=0; //used to prevent mistaking a long object for a really fast object, grace makes the app wait for the laser to shine again on the sensor

void TIM3_IRQHandler(void){
	if(TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)
	{

		ms++;								// a stupid hack, needs to be changed, there has to be a way to get the time elapsed from the registry ¯\_(ツ)_/¯
	TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
	}
}

int main(void)
{

	SystemInit();
	SystemCoreClockUpdate();
	//TM_DELAY_Init();
	TM_ADC_Init(ADC1, ADC_Channel_0);
	TM_ADC_Init(ADC1, ADC_Channel_11);
	Button_init();
	TIM3_init(1,39999);
	TIM3_inter();

	while(1){
		if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0)==1){ //if the reset button was pressed just reset everything
			first=0;
			speed=0;
			time=0;
			ms=0;
			grace=0;

		}
		light=TM_ADC_Read(ADC1, ADC_Channel_0); //PA0			//brightness
		calibration=TM_ADC_Read(ADC1, ADC_Channel_3); //PA3		//calibration connected to a potentiometer changes the threshold when the beam is considered as interrupted

		//$#$@$#%@
		//######$$$$$$$$$ SOME WAY TO MAKE IT MORE ROBOUST, PERHAPS SOME KIND DEFENSE MECHANISM TO COMBAT RANDOM BRIGHTNESS CHANGES/AUTOMATIC BRIGHTNESS ADJUSTMENT#$$$$$$$$$$$$#############
		//$#$##$#$

		if(light<calibration && grace==0){ //if the beam was intterupted and its not still being interrupted by the same object
			if(first==0){	//if the beam was interrupted first time
				//zacznij liczyc czas (zerujemy licznika i lecimy)
				ms=0;
				first=1;
			}
			else{			//if it was the second time the object appeared get time and calculate speed
				//skoncz liczyc czas
				time=ms/1000;
				speed=1/time;
				first=0;
			}
			grace=1;		//makes sure to start the "stopwatch once"
			// Waste some time //DONT
			//for (int i = 0; i < 50000; i++); //NO
		}
		if(light>calibration && grace==1){ //if the beam is not interrupted and it's after it was interrupted
			// Waste some time		//BAD PRACTICE
			//for (int i = 0; i < 50000; i++);	// DONT DO THIS AT HOME KIDS
			grace=0;					//goes back to normal
		}

	}
}
