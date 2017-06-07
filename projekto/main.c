#include "stdio.h"
#include "stm32f4xx_conf.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_adc.h"
#include "stm32f4xx_tim.h"
#include "defines.h"
<<<<<<< HEAD
#include "tm_stm32f4_delay.h"
#include "tm_stm32f4_hd44780.h"
int light;
int calibration;
=======
//#include "stm32f4xx.h"
#include "tm_stm32f4_delay.h"
#include "tm_stm32f4_hd44780.h"
//#include "biblioteka.h"
//#include "tm_stm32f4_adc.h"
volatile double light;
volatile double calibration;
>>>>>>> 79606a1e2c22e3f68659426a902938956c2c9036
double speed=0;//m/s
double distance=1;
double time=0;
long int ms=0;
int first=0; //was this the first time the beam was interrupted?
int grace=0; //used to prevent mistaking a long object for a really fast object, grace makes the app wait for the laser to shine again on the sensor
<<<<<<< HEAD
long int wait=0; //waits a few cycles so the ifs don't get triggered a million times a second on account of ADC being crazy
int ignore=0; //if an if was triggered ignore everything for about Xms (100ms is enough to measure speeds up to 5m/s)
int ignoreduration=100; //for how long to ignore ifs after triggering
char timems[6];
char speeds[4];
char lights[6];
char calibrations[6];

=======
long int wait=0; //waits a few cycles so the ifs don't get triggered like a million times a second on account of ADC being crazy
int ignore=0; //if an if was triggered ignore everything for about 500ms for now will be tweaked later
int ignoreduration=500; //for how long ignore ifs after triggering
char timems[6];
char speeds[6];
>>>>>>> 79606a1e2c22e3f68659426a902938956c2c9036

void TIM3_IRQHandler(void){
	if(TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)
	{

		ms++;
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
	}
}

int main(void)
{

	SystemInit();
	SystemCoreClockUpdate();
<<<<<<< HEAD
	TM_DELAY_Init();
	TM_HD44780_Init(16, 2);
=======
	//TM_DELAY_Init();
	 TM_HD44780_Init(16, 2);

	 TM_HD44780_Clear();

	    //Put string to LCD
	    TM_HD44780_Puts(0, 0, "raz");

	    //Wait a little

	    TM_HD44780_Puts(0, 0, "dwa");

	   	TM_HD44780_Puts(0, 1, "trzy");

>>>>>>> 79606a1e2c22e3f68659426a902938956c2c9036

	TM_ADC_Init(ADC1, ADC_Channel_3);
	TM_ADC_Init(ADC2, ADC_Channel_11);

	Button_init();

	TIM3_init(1,39999);
	TIM3_inter();
<<<<<<< HEAD

	TM_HD44780_Clear();
	TM_HD44780_Puts(0, 0, "Ready to start");
	TM_HD44780_Puts(0, 1, "BRI::");
	TM_HD44780_Puts(8, 1, "CAL:");
=======
	TM_HD44780_Clear();
	TM_HD44780_Puts(0, 0, "Ready to start");

>>>>>>> 79606a1e2c22e3f68659426a902938956c2c9036
	while(1){ //begins only when pressed
		if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0)==1){
			break;
		}
		light=TM_ADC_Read(ADC1, ADC_Channel_3); //PA3			//brightness
		calibration=TM_ADC_Read(ADC2, ADC_Channel_11)+50; //PC1		//calibration connected to a potentiometer changes the threshold when the beam is considered as interrupted

		sprintf(lights, "%d", light);
		TM_HD44780_Puts(4, 1, lights);
		sprintf(calibrations, "%d", calibration);
		TM_HD44780_Puts(12, 1, calibrations);
	}
	TM_HD44780_Clear();
	while(1){
		if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0)==1){ //if the reset button was pressed just reset everything
			first=0;
			speed=0.0;
			time=0.0;
			ms=0;
			grace=0;
			TM_HD44780_Clear();
		}
		light=TM_ADC_Read(ADC1, ADC_Channel_3); //PA3			//brightness
		calibration=TM_ADC_Read(ADC2, ADC_Channel_11)+50; //PC1		//calibration connected to a potentiometer changes the threshold when the beam is considered as interrupted

		if(!ignore){
			if(light>calibration && grace==0){ //if the beam was interrupted and it's not still being interrupted by the same object
				if(first==0){	//if the beam was interrupted first time
					ms=0;
					first=1;
					wait=ms;
					ignore=1;
					TM_HD44780_Clear();
					sprintf(speeds, "%f", speed);
				    TM_HD44780_Puts(3, 1, speeds);
				    TM_HD44780_Puts(13, 0, "raz");
				}
				else{			//if it was the second time the object appeared get time and calculate speed
					time=ms/1000.0;
					speed=distance/time;
					first=0;
					wait=ms;
					ignore=1;
				    TM_HD44780_Puts(13, 1, "dwa");
					sprintf(speeds, "%f", speed);
				    TM_HD44780_Puts(3, 1, speeds);
				}
				grace=1;		//makes sure to start the "stopwatch once"
			}
			if(light<calibration && grace==1){ //if the beam is not interrupted and it's after it was interrupted
				grace=0;	//goes back to normal
				wait=ms;
				ignore=1;
			}
		}
		if(ms-wait>ignoreduration){ignore=0;} //if the time from triggering an if is greater than ignoreduration check for ifs
<<<<<<< HEAD
	    TM_HD44780_Puts(0, 0, "T:");
	    TM_HD44780_Puts(0, 1, "S:");
		sprintf(timems, "%d", ms);
=======
		sprintf(timems, "%d", ms);

	    TM_HD44780_Puts(0, 0, "T:");
	    TM_HD44780_Puts(0, 1, "S:");
>>>>>>> 79606a1e2c22e3f68659426a902938956c2c9036
	    TM_HD44780_Puts(3, 0, timems);

	}
}
