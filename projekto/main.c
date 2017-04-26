#include "stm32f4xx_conf.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_adc.h"
#include "stm32f4xx_tim.h"
//#include "biblioteka.h"
//#include "tm_stm32f4_adc.h"
volatile double light;
volatile double calibration;
double speed=0;
double distance=1;
double time=0;
long int ms=0;
int first=0; //was this the first time the beam was interrupted?
int grace=0; //used to prevent mistaking a long object for a really fast object, grace makes the app wait for the laser to shine again on the sensor
long int wait=0; //waits a few cycles so the ifs don't get triggered like a million times a second on account of ADC being crazy
int ignore=0; //if an if was triggered ignore everything for about 500ms for now will be tweaked later
int ignoreduration=1000; //for how long ignore ifs after triggering

void TIM3_IRQHandler(void){
	if(TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)
	{

		ms++;								// a stupid hack, needs to be changed, there has to be a way to get the time elapsed from the registry ÂŻ\_(ă�„)_/ÂŻ
	TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
	}
}

int main(void)
{

	SystemInit();
	SystemCoreClockUpdate();
	//TM_DELAY_Init();
	TM_ADC_Init(ADC1, ADC_Channel_3);
	TM_ADC_Init(ADC2, ADC_Channel_11);
	Button_init();
	TIM3_init(1,39999);
	TIM3_inter();
	while(1){ //begins only when pressed
		if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0)==1){
			break;
		}
	}
	while(1){
		if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0)==1){ //if the reset button was pressed just reset everything
			first=0;
			speed=0.0;
			time=0.0;
			ms=0;
			grace=0;

		}
		light=TM_ADC_Read(ADC1, ADC_Channel_3); //PA3			//brightness
		calibration=TM_ADC_Read(ADC2, ADC_Channel_11); //PC1		//calibration connected to a potentiometer changes the threshold when the beam is considered as interrupted

		//$#$@$#%@
		//######$$$$$$$$$ SOME WAY TO MAKE IT MORE ROBOUST, PERHAPS SOME KIND OF DEFENSE MECHANISM TO COMBAT RANDOM BRIGHTNESS CHANGES/AUTOMATIC BRIGHTNESS ADJUSTMENT#$$$$$$$$$$$$#############
		//$#$##$#$
		if(!ignore){
			if(light<calibration && grace==0){ //if the beam was intterupted and its not still being interrupted by the same object
				if(first==0){	//if the beam was interrupted first time
					//zacznij liczyc czas (zerujemy licznika i lecimy)
					ms=0;
					first=1;
					wait=ms;
					ignore=1;
				}
				else{			//if it was the second time the object appeared get time and calculate speed
					//skoncz liczyc czas
					time=ms/1000.0;
					speed=distance/time;
					first=0;
					wait=ms;
					ignore=1;
				}
				grace=1;		//makes sure to start the "stopwatch once"
				// Waste some time //DONT
				//for (int i = 0; i < 50000; i++); //NO
			}
			if(light>calibration && grace==1){ //if the beam is not interrupted and it's after it was interrupted
				// Waste some time		//BAD PRACTICE
				//for (int i = 0; i < 50000; i++);	// DONT DO THIS AT HOME KIDS
				grace=0;	//goes back to normal
				wait=ms;
				ignore=1;
			}
		}
		if(ms-wait>ignoreduration){ignore=0;} //if the time from triggering an if is greater than 500ms check for ifs
	}
}
