#include "liben.h"


void ADC1_init() {
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	// zegar dla portu GPIO z którego wykorzystany zostanie pin jako wejście ADC (PA1)
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
	// zegar dla modułu ADC1

	GPIO_InitTypeDef GPIO_InitStructure;
	//inicjalizacja wejścia ADC
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* Wspólna konfiguracja dla wszystkich układów ADC */
	// niezależny tryb pracy przetworników
	ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
	// zegar główny podzielony przez 2
	ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;
	// opcja istotna tylko dla tryby multi ADC
	ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
	// czas przerwy pomiędzy kolejnymi konwersjami
	ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
	ADC_CommonInit(&ADC_CommonInitStructure);

	/* Konfiguracja danego przetwornika ADC */
	//ustawienie rozdzielczości przetwornika na maksymalną (12 bitów)
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
	//wyłączenie trybu skanowania (odczytywać będziemy jedno wejście ADC
	//w trybie skanowania automatycznie wykonywana jest konwersja na wielu
	//wejściach/kanałach)
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;
	//włączenie ciągłego trybu pracy
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
	//wyłączenie zewnętrznego wyzwalania
	//konwersja może być wyzwalana timerem, stanem wejścia itd. (szczegóły w
	//dokumentacji)
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
	//wartość binarna wyniku będzie podawana z wyrównaniem do prawej
	//funkcja do odczytu stanu przetwornika ADC zwraca wartość 16-bitową
	//dla przykładu, wartość 0xFF wyrównana w prawo to 0x00FF, w lewo 0x0FF0
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	//liczba konwersji równa 1, bo 1 kanał
	ADC_InitStructure.ADC_NbrOfConversion = 1;
	// zapisz wypełnioną strukturę do rejestrów przetwornika numer 1
	ADC_Init(ADC1, &ADC_InitStructure);

	/* konfiguracja wybranego kanału ADC */
	ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 1, ADC_SampleTime_84Cycles);

	/*W przykładzie powyżej konfigurujemy pierwszy kanał przetwornika ADC1
	 do samodzielnej pracy, ustawiając czas próbkowania na 84 cykle sygnału
	 zegarowego.*/

	//uruchomienie przetwornika ADC
	ADC_Cmd(ADC1, ENABLE);

	/*Odczyt wartości poprzez odpytywanie flagi zakończenia konwersji wykonujemy
	 następująco (zmienna ADC_Result trzeba najpierw zadeklarować jako volatile double):
	 ADC_SoftwareStartConv(ADC1);
	 while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);
	 ADC_Result = ADC_GetConversionValue(ADC1);
	 */
}


void ADC2_init() {
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	// zegar dla portu GPIO z którego wykorzystany zostanie pin jako wejście ADC (PA1)
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC2, ENABLE);
	// zegar dla modułu ADC1

	GPIO_InitTypeDef GPIO_InitStructure;
	//inicjalizacja wejścia ADC
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	/* Wspólna konfiguracja dla wszystkich układów ADC */
	// niezależny tryb pracy przetworników
	ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
	// zegar główny podzielony przez 2
	ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;
	// opcja istotna tylko dla tryby multi ADC
	ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
	// czas przerwy pomiędzy kolejnymi konwersjami
	ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
	ADC_CommonInit(&ADC_CommonInitStructure);

	/* Konfiguracja danego przetwornika ADC */
	//ustawienie rozdzielczości przetwornika na maksymalną (12 bitów)
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
	//wyłączenie trybu skanowania (odczytywać będziemy jedno wejście ADC
	//w trybie skanowania automatycznie wykonywana jest konwersja na wielu
	//wejściach/kanałach)
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;
	//włączenie ciągłego trybu pracy
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
	//wyłączenie zewnętrznego wyzwalania
	//konwersja może być wyzwalana timerem, stanem wejścia itd. (szczegóły w
	//dokumentacji)
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
	//wartość binarna wyniku będzie podawana z wyrównaniem do prawej
	//funkcja do odczytu stanu przetwornika ADC zwraca wartość 16-bitową
	//dla przykładu, wartość 0xFF wyrównana w prawo to 0x00FF, w lewo 0x0FF0
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	//liczba konwersji równa 1, bo 1 kanał
	ADC_InitStructure.ADC_NbrOfConversion = 1;
	// zapisz wypełnioną strukturę do rejestrów przetwornika numer 1
	ADC_Init(ADC1, &ADC_InitStructure);

	/* konfiguracja wybranego kanału ADC */
	ADC_RegularChannelConfig(ADC2, ADC_Channel_1, 1, ADC_SampleTime_84Cycles);

	/*W przykładzie powyżej konfigurujemy pierwszy kanał przetwornika ADC1
	 do samodzielnej pracy, ustawiając czas próbkowania na 84 cykle sygnału
	 zegarowego.*/

	//uruchomienie przetwornika ADC
	ADC_Cmd(ADC2, ENABLE);

	/*Odczyt wartości poprzez odpytywanie flagi zakończenia konwersji wykonujemy
	 następująco (zmienna ADC_Result trzeba najpierw zadeklarować jako volatile double):
	 ADC_SoftwareStartConv(ADC1);
	 while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);
	 ADC_Result = ADC_GetConversionValue(ADC1);
	 */
}
