//	Biblioteka PTM - Dawid Wisniewski
#include "biblioteka.h"

/* dodaj do main():
 *
 * SystemInit();
 * SystemCoreClockUpdate();
 */

//Funkcja wprowadza opoznienie
void Delay(unsigned long int count) {
	unsigned long int i = 0;
	for (i = 0; i < count; i++)
		;
}

//Inicjalizacja diod LED
void LED_init() {
	/* GPIOD Periph clock enable
	 * W³¹cznenie generatora sygnalu analogowego */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	/* Configure PD12, PD13, PD14 and PD15 in output pushpull mode */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14
			| GPIO_Pin_15; //inicjalizowane wyprowadzenia portu
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;/*tryb dzialania wyprowadzenia:
	 GPIO_Mode_IN - wejœcie binarne
	 GPIO_Mode_OUT - wyjœcie binarne
	 GPIO_Mode_AF - funkcja alternatywna*/
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //typ wyjscia: GPIO_OType_PP - wyjscie komplementarne, GPIO_OType_OD - wyjscie z otwartym drenem
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz; //maks dozwolona predkosc przelaczania wyprowadzen (2, 25, 50 lub 100)
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;/*rodzaj podciagania wyprowadzenia:
	 GPIO_PuPd_NOPULL - brak podci¹gania
	 GPIO_PuPd_UP - podci¹ganie do napiecia zasilania
	 GPIO)PuPd_DOWN - sciaganie do masy*/
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	/* Przyk³ad u¿ycia:

	 Wy³¹czanie / W³¹czanie diod
	 GPIO_ResetBits(GPIOD, GPIO_Pin_14);
	 GPIO_SetBits(GPIOD, GPIO_Pin_13);

	 Przelacznie stanu diody (zapalona/wylaczana)
	 GPIO_ToggleBits(GPIOD, GPIO_Pin_12);

	 Odczytanie stanu pojedynczego wyprowadzenia (funkcja)
	 GPIO_ReadInputDataBit
	 */
} //LED_init()

//inicjowanie przycisku
void Button_init() {
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/*Przyklad uzycia:
	 *
	 * Odczytywanie stanu przycisku
	 GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0);
	 */
} //Button_init()

/*inicjalizacja TIM3
 * parametry: preslacer, period
 */
void TIM3_init(int prescaler, int period) {
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); //podloczenie timer numer do do szyny APB

	/* Time base configuration */
	TIM_TimeBaseStructure.TIM_Period = period; //okres zliczania
	TIM_TimeBaseStructure.TIM_Prescaler = prescaler; //wartosc prescalera (dzielnika czestotliwosci z glownej szyny zegarowej)
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //dzielnik zegara
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //kierunek zliczania (gora/dol) - TIM_CounterMode_Up / TIM_CounterMode_Down
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

	TIM_Cmd(TIM3, ENABLE); //uruchomienie timera

	/* odczytywanie stanu rejestru w ktorym przechowywana jest aktualna wartosc licznika:
	 * unsigned int counter = TIMx->CNT;
	 *
	 * odczytanie i reset flagi przepelnienia timera
	 * if(TIM_GetFlagStatus(TIMx, TIM_FLAG_Update)) {
	 ...
	 ...
	 TIM_ClearFlag(TIMx, TIM_FLAG_Update);
	 }
	 */
} //TIM3_init()

/*inicjalizacja TIM4
 * parametry: preslacer, period
 */
void TIM4_init(int prescaler, int period) {
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE); //podloczenie timer numer do do szyny APB

	/* Time base configuration */
	TIM_TimeBaseStructure.TIM_Period = period; //okres zliczania
	TIM_TimeBaseStructure.TIM_Prescaler = prescaler; //wartosc prescalera (dzielnika czestotliwosci z glownej szyny zegarowej)
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //dzielnik zegara
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //kierunek zliczania (gora/dol) - TIM_CounterMode_Up / TIM_CounterMode_Down
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

	TIM_Cmd(TIM4, ENABLE); //uruchomienie timera

	/* odczytywanie stanu rejestru w ktorym przechowywana jest aktualna wartosc licznika:
	 * unsigned int counter = TIMx->CNT;
	 *
	 * odczytanie i reset flagi przepelnienia timera
	 * if(TIM_GetFlagStatus(TIMx, TIM_FLAG_Update)) {
	 ...
	 ...
	 TIM_ClearFlag(TIMx, TIM_FLAG_Update);
	 }
	 */
} //TIM4_init()

/*inicjalizacja PWM
 * parametry: Pulse */
void PWM_init(int PulseFromUser) {
	/* PWM1 Mode configuration: */
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = PulseFromUser;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

	/* Konfiguracjê kana³ów timera do pracy w trybie OutputCompare wykonuje siê poprzez
	 ka¿dorazowe wykonanie funkcji: */
	TIM_OC1Init(TIM4, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);

	/* przypisanie pinu GPIO jako kana³u wyjœciowego timera, na
	 którym generowany bêdzie sygna³ */
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource12, GPIO_AF_TIM4);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	/*  Wykaz mo¿liwych po³¹czeñ pinów portów GPIO z peryferiami (tryb Alternate Function)
	 znajduje siê w sekcji Pinouts and Pin Description, w nocie katalogowej w³aœciwej dla
	 stosowanego mikrokontrolera.

	 Ostatnim krokiem jest ustawienie odpowiedniego wspó³czynnika wype³nia*/
	//TIM4->CCR1 = 50;
} //PWM_init()

/* Przerwania timera 3 */
void TIM3_inter() {
	// ustawienie trybu pracy priorytetów przerwañ
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);

	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn; // numer przerwania
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00; // priorytet g³ówny
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00; // subpriorytet
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; // uruchom dany kana³
	// zapisz wype³nion¹ strukturê do rejestrów
	NVIC_Init(&NVIC_InitStructure);
	/* Numerów przerwañ nale¿y szukaæ w pliku stm32f4xx.h jako wartoœci typu
	 wyliczeniowego IRQn. */
	// wyczyszczenie przerwania od timera 3 (wyst¹pi³o przy konfiguracji timera)
	TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
	// zezwolenie na przerwania od przepe³nienia dla timera 3
	TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);

	/* Powy¿sze funkcje mo¿na znaleŸæ w pliku stm32f4xx_tim.h (deklaracje) oraz
	 * stm32f4xx_tim.c (definicje). Dodatkowo w pliku stm32f4xx_tim.h s¹ umieszczone
	 * wszystkie dostêpne Ÿród³a przerwañ dla timera (TIM_interrupt_sources). */

	/* Trzeba w main wywolac:
	 * TIM3_IRQHandler();
	 */
} //TIM3_inter()

/* obsluga przerwania timer 3
 * Trzeba w main wywolac:
 * TIM3_IRQHandler();
 *
 *Ta funkcje tez dam w main:
 void TIM3_IRQHandler(void) {
 if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET ) {
 // miejsce na kod wywo³ywany w momencie wyst¹pienia przerwania
 // wyzerowanie flagi wyzwolonego przerwania
 TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
 }
 }
 */

void Button_inter() {
	// ustawienie trybu pracy priorytetów przerwañ
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);

	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn; // numer przerwania
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00; // priorytet g³ówny
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00; // subpriorytet
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; // uruchom dany kana³
	// zapisz wype³nion¹ strukturê do rejestrów
	NVIC_Init(&NVIC_InitStructure);

	EXTI_InitStructure.EXTI_Line = GPIO_Pin_0; // wybór numeru aktualnie konfigurowanej linii przerwañ
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt; // wybór trybu - przerwanie b¹dŸ zdarzenie
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; // wybór zbocza, na które zareaguje przerwanie
	EXTI_InitStructure.EXTI_LineCmd = ENABLE; // uruchom dan¹ liniê przerwañ
	// zapisz strukturê konfiguracyjn¹ przerwañ zewnêtrznych do rejestrów
	EXTI_Init(&EXTI_InitStructure);
	// pod³¹czenie danego pinu portu do kontrolera przerwañ
	SYSCFG_EXTILineConfig(GPIOD, EXTI_PinSource0);
} //Button_inter()

/* Obs³uga przerwan zewnetrznych (trzeba ta funkcje zaimplementowac w pliku main.cpp
 * oraz wywolac w main()
 *
 void EXTI0_IRQHandler(void)
 {
 if(EXTI_GetITStatus(EXTI_Line0) != RESET)
 {
 // miejsce na kod wywo³ywany w momencie wyst¹pienia przerwania
 //  wyzerowanie  flagi  wyzwolonego  przerwania
 EXTI_ClearITPendingBit(EXTI_Line0);
 }
 }
 */

void ADC_init() {
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	// zegar dla portu GPIO z którego wykorzystany zostanie pin jako wejœcie ADC (PA1)
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
	// zegar dla modu³u ADC1

	GPIO_InitTypeDef GPIO_InitStructure;
	//inicjalizacja wejœcia ADC
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* Wspólna konfiguracja dla wszystkich uk³adów ADC */
	// niezale¿ny tryb pracy przetworników
	ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
	// zegar g³ówny podzielony przez 2
	ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;
	// opcja istotna tylko dla tryby multi ADC
	ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
	// czas przerwy pomiêdzy kolejnymi konwersjami
	ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
	ADC_CommonInit(&ADC_CommonInitStructure);

	/* Konfiguracja danego przetwornika ADC */
	//ustawienie rozdzielczoœci przetwornika na maksymaln¹ (12 bitów)
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
	//wy³¹czenie trybu skanowania (odczytywaæ bêdziemy jedno wejœcie ADC
	//w trybie skanowania automatycznie wykonywana jest konwersja na wielu
	//wejœciach/kana³ach)
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;
	//w³¹czenie ci¹g³ego trybu pracy
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
	//wy³¹czenie zewnêtrznego wyzwalania
	//konwersja mo¿e byæ wyzwalana timerem, stanem wejœcia itd. (szczegó³y w
	//dokumentacji)
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
	//wartoœæ binarna wyniku bêdzie podawana z wyrównaniem do prawej
	//funkcja do odczytu stanu przetwornika ADC zwraca wartoœæ 16-bitow¹
	//dla przyk³adu, wartoœæ 0xFF wyrównana w prawo to 0x00FF, w lewo 0x0FF0
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	//liczba konwersji równa 1, bo 1 kana³
	ADC_InitStructure.ADC_NbrOfConversion = 1;
	// zapisz wype³nion¹ strukturê do rejestrów przetwornika numer 1
	ADC_Init(ADC1, &ADC_InitStructure);

	/* konfiguracja wybranego kana³u ADC */
	ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 1, ADC_SampleTime_84Cycles);

	/*W przyk³adzie powy¿ej konfigurujemy pierwszy kana³ przetwornika ADC1
	 do samodzielnej pracy, ustawiaj¹c czas próbkowania na 84 cykle sygna³u
	 zegarowego.*/

	//uruchomienie przetwornika ADC
	ADC_Cmd(ADC1, ENABLE);

	/*Odczyt wartoœci poprzez odpytywanie flagi zakoñczenia konwersji wykonujemy
	 nastêpuj¹co (zmienna ADC_Result trzeba najpierw zadeklarowaæ jako volatile double):
	 ADC_SoftwareStartConv(ADC1);
	 while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);
	 ADC_Result = ADC_GetConversionValue(ADC1);
	 */
}

void DAC_init() {
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	// zegar dla portu GPIO z którego wykorzystany zostanie pin jako wyjœcie DAC (PA4)
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE);
	// zegar  dla modu³u DAC

	//inicjalizacja wyjœcia DAC
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	//wy³¹czenie zewnêtrznego wyzwalania
	//konwersja mo¿e byæ wyzwalana timerem, stanem wejœcia itd. (szczegó³y w dokumentacji)
	DAC_InitStructure.DAC_Trigger = DAC_Trigger_None;
	//nast. 2 linie - wy³¹czamy generator predefiniowanych przebiegów
	//wyjœciowych (wartoœci zadajemy sami, za pomoc¹ odpowiedniej funkcji)
	DAC_InitStructure.DAC_WaveGeneration = DAC_WaveGeneration_None;
	DAC_InitStructure.DAC_LFSRUnmask_TriangleAmplitude = DAC_LFSRUnmask_Bit0;
	//w³¹czamy buforowanie sygna³u wyjœciowego
	DAC_InitStructure.DAC_OutputBuffer = DAC_OutputBuffer_Enable;
	DAC_Init(DAC_Channel_1, &DAC_InitStructure);

	// uruchomienie przetwornika DAC
	DAC_Cmd(DAC_Channel_1, ENABLE);
	// ustawienie maksymalnej rozdzielczosci (12 bitów)
	// jako wartoœci podajemy liczbê 16-bitow¹ wyrównan¹ do prawej
	//DAC_SetChannel1Data(DAC_Align_12b_R, 0xFFF);
	// Korzystaj¹c z powy¿szej funkcji mo¿liwe jest zmienianie wartoœci generowanej
	// przez modu³ DAC
} //DAC_init()

// konfiguracja USART
void USART_init(void) {
	/* USARTx configured as follow:
	 - BaudRate = 115200 baud
	 - Word Length = 8 Bits
	 - One Stop Bit
	 - No parity
	 - Hardware flow control disabled (RTS and CTS signals)
	 - Receive and transmit enabled
	 */
	GPIO_InitTypeDef GPIO_InitStructure;

	// wlaczenie taktowania wybranego portu
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	// wlaczenie taktowania wybranego uk³adu USART
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

	/* Connect PXx to USARTx_Tx*/
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_USART3);

	/* Connect PXx to USARTx_Rx*/
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_USART3);

	/* w³¹czenie  funkcji  alternatywnych  pinów
	 mikroprocesora w wejœciach i wyjsciach uk³adu USART
	 konfiguracja lini Tx */
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	// konfiguracja linii Rx
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	// konfiguracja USART
	// predkosc transmisji (mozliwe standardowe opcje: 9600, 19200, 38400,
	// 57600, 115200, ...)
	USART_InitStructure.USART_BaudRate = 115200;
	// d³ugoœæ s³owa (USART_WordLength_8b lub USART_WordLength_9b)
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	//  liczba  bitów  stopu  (USART_StopBits_1,  USART_StopBits_0_5,
	// USART_StopBits_2, USART_StopBits_1_5)
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	//  sprawdzanie  parzystoœci  (USART_Parity_No,  USART_Parity_Even, USART_Parity_Odd)
	USART_InitStructure.USART_Parity = USART_Parity_No;
	//  sprzêtowa  kontrola  przep³ywu  (USART_HardwareFlowControl_None,
	//USART_HardwareFlowControl_RTS,  USART_HardwareFlowControl_CTS,
	//USART_HardwareFlowControl_RTS_CTS)
	USART_InitStructure.USART_HardwareFlowControl =
			USART_HardwareFlowControl_None;
	// tryb nadawania/odbierania (USART_Mode_Rx, USART_Mode_Rx )
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

	// konfiguracja
	USART_Init(USART3, &USART_InitStructure);

	// uruchomienie
	USART_Cmd(USART3, ENABLE);

	/* wyslanie danych
	 USART_SendData(USART3, 'A');
	 - tutaj wysy³am znak A, ale móg³bym wys³aæ dowolny znak typu uint8_t (wpisuje kod ASCII)

	 czekaj az dane zostana wyslane
	 while (USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET)  {}

	 */
} //USART_init()

//odbieranie danych USART
uint8_t usartGetChar1(void) {
	//czekaj na odebranie danych
	while (USART_GetFlagStatus(USART3, USART_FLAG_RXNE) == RESET) {
	}
	return USART_ReceiveData(USART3);
}
//odbieranie danych USART (troche inny sposob)
uint8_t usartGetChar2(void) {

	if (USART_GetFlagStatus(USART3, USART_FLAG_RXNE) == SET) {
		return USART_ReceiveData(USART3);
	}
	return 0xff;
}

//wysylanie danych USART
void usartSendData(uint8_t ch) {
	USART_SendData(USART3, ch);

	// czekanie na zakonczenie transmisji
	while (USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET) {
	}
}

//konfiguracja przerwañ USART
void USART_inter(void) {
	NVIC_InitTypeDef NVIC_InitStructure; //struktura do konfiguracji kontrolera NVIC (nested vector interrupt controller)

	// wlaczenie przerwania zwi¹zanego z odebraniem danych (pozostale zrodla
	// przerwan zdefiniowane sa w pliku stm32f4xx_usart.h)
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE); // enable the USART1 receive interrupt

	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn; // we want to configure the USART1 interrupts
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; // this sets the priority group of the USART1 interrupts
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; // this sets the subpriority inside the group
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; // the USART1 interrupts are globally enabled
	// konfiguracja kontrolera przerwan
	NVIC_Init(&NVIC_InitStructure);
	// wlaczenie przerwan od ukladu USART
	NVIC_EnableIRQ(USART3_IRQn);

	/* Funkcja obslugi przerwania
	 *
	 * void USART3_IRQHandler(void)
	 * {
	 * 		// sprawdzenie flagi zwiazanej z odebraniem danych przez USART
	 * 		if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
	 * 		{
	 * 			// odebrany bajt znajduje sie w rejestrze USART3->DR, odczytujemy go np.
	 * 			// char t = USART3->DR;
	 * 			// uint8_t znak = USART3->DR;
	 * 		}
	 * }
	 */
} //USART_inter()

void spi_init(int master) {
	//Pod³¹czenie zegara do modu³u SPI i portów wejœæ/wyjœæ:
	/* Enable the SPI periph */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
	/* Enable GPIO clocks */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	/* Enable SCK, MOSI and MISO GPIO clocks */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	/* Enable CS GPIO clock */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);

	//Wybór tak zwanych „alternative function” dla wyprowadzeñ GPIO:
	// SCK
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_SPI2);
	// MISO
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource14, GPIO_AF_SPI2);
	// MOSI
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource15, GPIO_AF_SPI2);

	//Konfiguracja wyjœæ GPIO odpowiedzialnych za komunikacjê SPI:
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

	/* SPI SCK pin configuration */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	/* SPI MOSI pin configuration */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	/* SPI MISO pin configuration */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	//Konfiguracja magistrali SPI dla trybu Master:
	/* SPI configuration ------------------------------------------------------
	 -*/
	if (master == 1) {
		//Konfiguracja magistrali SPI dla trybu Master:
		/* SPI configuration ------------------------------------------------------
		 -*/
		SPI_InitTypeDef SPI_InitStructure;
		SPI_I2S_DeInit(SPI2);
		SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
		SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
		SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
		SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
		SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
		SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;
		SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
		SPI_InitStructure.SPI_CRCPolynomial = 7;
		SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
		SPI_Init(SPI2, &SPI_InitStructure);
	} else {
		//Konfiguracja magistrali SPI dla trybu Slave:
		/* SPI configuration ------------------------------------------------------
		 -*/
		SPI_InitTypeDef SPI_InitStructure;
		SPI_I2S_DeInit(SPI2);
		SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
		SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
		SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
		SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
		SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
		SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;
		SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
		SPI_InitStructure.SPI_CRCPolynomial = 7;
		SPI_InitStructure.SPI_Mode = SPI_Mode_Slave;
		SPI_Init(SPI2, &SPI_InitStructure);
	} //else
	  //Uruchomienie SPI2:
	/* Enable SPI2 */
	SPI_Cmd(SPI2, ENABLE);

	//Konfiguracja wyjœcia Chip Select i wstêpne ustawienie tego wyjœcia w stan
	//wysoki:
	/* Configure GPIO PIN for Lis Chip select */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOE, &GPIO_InitStructure);
	/* Deselect : Chip Select high */
	GPIO_SetBits(GPIOE, GPIO_Pin_3);

	//Odbiór danych:
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET)
		;
	//Receive[0] = SPI_I2S_ReceiveData(SPI2);

	//Wysy³anie danych:
	SPI_I2S_SendData(SPI2, 0x3);
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET)
		;
} //spi_init()

//DMA
// transfer danych z uk³adu peryferyjnego do pamiêci na przyk³adzie ADC
// odczytana wartosc w zmiennej globalnej
// volatile uint16_t valueFromADC;
volatile uint16_t valueFromADC;

void MY_DMA_initP2M(void) {
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
	DMA_InitTypeDef strukturaDoInicjalizacjiDMA;
// wybór kana³u DMA
	strukturaDoInicjalizacjiDMA.DMA_Channel = DMA_Channel_0;

// ustalenie rodzaju transferu (memory2memory / peripheral2memory /memory2peripheral)
	strukturaDoInicjalizacjiDMA.DMA_DIR = DMA_DIR_PeripheralToMemory;
// tryb pracy - pojedynczy transfer b¹dŸ powtarzany
	strukturaDoInicjalizacjiDMA.DMA_Mode = DMA_Mode_Circular;
// ustalenie priorytetu danego kana³u DMA
	strukturaDoInicjalizacjiDMA.DMA_Priority = DMA_Priority_Medium;
// liczba danych do przes³ania
	strukturaDoInicjalizacjiDMA.DMA_BufferSize = (uint32_t) 1;
// adres Ÿród³owy
	strukturaDoInicjalizacjiDMA.DMA_PeripheralBaseAddr = (uint32_t)(
			ADC_1_ADDRESS_BASE + ADC_DR_ADDRESS_OFFSET);
// adres docelowy
	strukturaDoInicjalizacjiDMA.DMA_Memory0BaseAddr = (uint32_t) & valueFromADC;
// okreslenie, czy adresy maj¹ byæ inkrementowane po ka¿dej przes³anej paczce danych
	strukturaDoInicjalizacjiDMA.DMA_MemoryInc = DMA_MemoryInc_Disable;
	strukturaDoInicjalizacjiDMA.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
// ustalenie rozmiaru przesy³anych danych
	strukturaDoInicjalizacjiDMA.DMA_PeripheralDataSize =
			DMA_PeripheralDataSize_HalfWord;
	strukturaDoInicjalizacjiDMA.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
// ustalenie trybu pracy - jednorazwe przes³anie danych
	strukturaDoInicjalizacjiDMA.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	strukturaDoInicjalizacjiDMA.DMA_MemoryBurst = DMA_MemoryBurst_Single;
// wy³¹czenie kolejki FIFO (nie u¿ywana w tym przykadzie)
	strukturaDoInicjalizacjiDMA.DMA_FIFOMode = DMA_FIFOMode_Disable;
// wype³nianie wszystkich pól struktury jest niezbêdne w celu poprawnego dzia³ania, wpisanie
	//jednej z dozwolonych wartosci
	strukturaDoInicjalizacjiDMA.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
// zapisanie wype³nionej struktury do rejestrów wybranego po³¹czenia DMA
	DMA_Init(DMA2_Stream4, &strukturaDoInicjalizacjiDMA);
// uruchomienie odpowiedniego po³¹czenia DMA
	DMA_Cmd(DMA2_Stream4, ENABLE);
}

void MY_ADC_init(void) {
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); // wejscie ADC
//inicjalizacja wejœcia ADC
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	ADC_CommonInitTypeDef ADC_CommonInitStructure;
// niezale¿ny tryb pracy przetworników
	ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
// zegar g³ówny podzielony przez 2
	ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;
// opcja istotna tylko dla tryby multi ADC
	ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
// czas przerwy pomiêdzy kolejnymi konwersjami
	ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
	ADC_CommonInit(&ADC_CommonInitStructure);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE); //ADC
	ADC_InitTypeDef ADC_InitStructure;
//ustawienie rozdzielczoœci przetwornika na maksymaln¹ (12 bitów)
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
//wy³¹czenie trybu skanowania (odczytywaæ bêdziemy jedno wejœcie ADC
//w trybie skanowania automatycznie wykonywana jest konwersja na
	//wielu wejœciach/kana³ach)
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;
//w³¹czenie ci¹g³ego trybu pracy
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
//wy³¹czenie zewnêtrznego wyzwalania
//konwersja mo¿e byæ wyzwalana timerem, stanem wejœcia itd.
	//(szczegó³y w dokumentacji)
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
//wartoœæ binarna wyniku bêdzie podawana z wyrównaniem do prawej
//funkcja do odczytu stanu przetwornika ADC zwraca wartoœæ 16-bitow¹
//dla przyk³adu, wartoœæ 0xFF wyrównana w prawo to 0x00FF, w lewo 0x0FF0
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
//liczba konwersji równa 1, bo 1 kana³
	ADC_InitStructure.ADC_NbrOfConversion = 1;
// zapisz wype³nion¹ strukturê do rejestrów przetwornika numer 1
	ADC_Init(ADC1, &ADC_InitStructure);
// konfiguracja czasu próbkowania sygna³u
	ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 1, ADC_SampleTime_84Cycles);
// w³¹czenie wyzwalania DMA po ka¿dym zakoñczeniu konwersji
	ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE);
// w³¹czenie DMA dla ADC
	ADC_DMACmd(ADC1, ENABLE);
// uruchomienie modu³y ADC
	ADC_Cmd(ADC1, ENABLE);
} //funkcja_jakas

//DMA
//transfer danych z pamiêci do uk³adu peryferyjnego (tutaj DAC)
//potrzebna sta³a globalna
const uint16_t DAC_DMA_sine12bit[DMA_DAC_SIGNAL_SIZE] = { 2047, 2447, 2831,
		3185, 3498, 3750, 3939, 4056, 4095, 4056, 3939, 3750, 3495, 3185, 2831,
		2447, 2047, 1647, 1263, 909, 599, 344, 155, 38, 0, 38, 155, 344, 599,
		909, 1263, 1647 };
void MY_DMA_initM2P(void) {
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
	DMA_InitTypeDef strukturaDoInicjalizacjiDMA;
// wybór kana³u DMA
	strukturaDoInicjalizacjiDMA.DMA_Channel = DMA_Channel_7;
// ustalenie rodzaju transferu (memory2memory / peripheral2memory /memory2peripheral)
	strukturaDoInicjalizacjiDMA.DMA_DIR = DMA_DIR_MemoryToPeripheral;
// tryb pracy - pojedynczy transfer b¹dŸ powtarzany
	strukturaDoInicjalizacjiDMA.DMA_Mode = DMA_Mode_Circular;
// ustalenie priorytetu danego kana³u DMA
	strukturaDoInicjalizacjiDMA.DMA_Priority = DMA_Priority_Medium;
// liczba danych do przes³ania
	strukturaDoInicjalizacjiDMA.DMA_BufferSize = (uint32_t) DMA_DAC_SIGNAL_SIZE;
// adres Ÿród³owy
	strukturaDoInicjalizacjiDMA.DMA_Memory0BaseAddr =
			(uint32_t) DAC_DMA_sine12bit;
// adres docelowy
	strukturaDoInicjalizacjiDMA.DMA_PeripheralBaseAddr = (uint32_t)(
			DAC_CHANNEL_1_ADDRESS_BASE + DAC_DHR12R1_ADDRESS_OFFSET);
// zezwolenie na inkrementacje adresu po ka¿dej przes³anej paczce danych
	strukturaDoInicjalizacjiDMA.DMA_MemoryInc = DMA_MemoryInc_Enable;
	strukturaDoInicjalizacjiDMA.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
// ustalenie rozmiaru przesy³anych danych
	strukturaDoInicjalizacjiDMA.DMA_PeripheralDataSize =
			DMA_PeripheralDataSize_HalfWord;
	strukturaDoInicjalizacjiDMA.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
// ustalenie trybu pracy - jednorazwe przes³anie danych
	strukturaDoInicjalizacjiDMA.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	strukturaDoInicjalizacjiDMA.DMA_MemoryBurst = DMA_MemoryBurst_Single;
// wy³¹czenie kolejki FIFO (nie u¿ywana w tym przykadzie)
	strukturaDoInicjalizacjiDMA.DMA_FIFOMode = DMA_FIFOMode_Disable;
// wype³nianie wszystkich pól struktury jest niezbêdne w celu
//	poprawnego dzia³ania, wpisanie jednej z dozwolonych wartosci
	strukturaDoInicjalizacjiDMA.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
// zapisanie wype³nionej struktury do rejestrów wybranego po³¹czenia DMA
	DMA_Init(DMA1_Stream5, &strukturaDoInicjalizacjiDMA);
// uruchomienie odpowiedniego po³¹czenia DMA
	DMA_Cmd(DMA1_Stream5, ENABLE);
// uruchomienie DMA dla pierwszego kana³u DAC
	DAC_DMACmd(DAC_Channel_1, ENABLE);
}

void MY_DAC_init(void) {
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); // wyjscie DAC
//inicjalizacja wyjœcia DAC
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE); //DAC
	DAC_InitTypeDef DAC_InitStructure;
//wy³¹czenie zewnêtrznego wyzwalania
//konwersja mo¿e byæ wyzwalana timerem, stanem wejœcia itd.
//(szczegó³y w dokumentacji)
	DAC_InitStructure.DAC_Trigger = DAC_Trigger_T6_TRGO;
//wy³¹czamy generator predefiniowanych przebiegów wyjœciowych
//(wartoœci zadajemy sami, za pomoc¹ opowiedniej funkcji)
	DAC_InitStructure.DAC_WaveGeneration = DAC_WaveGeneration_None;
//w³¹czamy buforowanie sygna³u wyjœciowego
	DAC_InitStructure.DAC_OutputBuffer = DAC_OutputBuffer_Enable;
	DAC_InitStructure.DAC_LFSRUnmask_TriangleAmplitude = DAC_LFSRUnmask_Bit0;
	DAC_Init(DAC_Channel_1, &DAC_InitStructure);
	DAC_SetChannel1Data(DAC_Align_12b_R, 0x000);
	DAC_Cmd(DAC_Channel_1, ENABLE);
}

void MY_DAC_initTimerForUpdating(void) {
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_TimeBaseStructure.TIM_Prescaler = 42 - 1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Period = 20;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM6, &TIM_TimeBaseStructure);
	TIM_SelectOutputTrigger(TIM6, TIM_TRGOSource_Update);
	TIM_SetCounter(TIM6, 0);
	TIM_Cmd(TIM6, ENABLE);
}

//=============================================================

//I2C
int MCP9800_ADDRESS = 0x90;
int I2C1_SLAVE_ADDRESS7=0x15;
//inicjalizacja
void i2c_init() {
	//Ustawienie obs³ugi nale¿y zacz¹æ od za³¹czenia zegarów:
	// W³¹czenie zegara I2C
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
	// W³¹czenie zegara od GPIOB
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	//Nastêpnym krokiem jest skonfigurowanie pinów PB8 (SCL) oraz PB7 (SDA) jako
	//pinów magistrali I 2 C:

	// Ustawienie pinów SCL (PB8) i SDA (PB7)
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_8;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_OD; // Open-drain
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOB, &GPIO_InitStruct);
	// Po³¹czenie I2C z pinami
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_I2C1); // SCL
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_I2C1); // SDA
	//Ustawienie parametrów I 2 C odbywa siê za pomoc¹ struktury  I2C_InitTypeDef :
	// Ustawienie I2C
	I2C_InitTypeDef I2C_InitStruct;
	// Predkosc transmisji - 100kHz
	I2C_InitStruct.I2C_ClockSpeed = 100000;
	// Wybór I2C (inne mo¿liwosci dotycz¹ SMBUS)
	I2C_InitStruct.I2C_Mode = I2C_Mode_I2C;
	// Wspó³czynnik wype³nienia 50%
	I2C_InitStruct.I2C_DutyCycle = I2C_DutyCycle_2;
	// Adres w trybie Slave, pole nieistotne w trybie Master
	I2C_InitStruct.I2C_OwnAddress1 = I2C1_SLAVE_ADDRESS7;
	// ACK - potwierdzenie odbioru
	I2C_InitStruct.I2C_Ack = I2C_Ack_Enable;
	// D³ugosc adresacji urz¹dzeñ - 7bitów
	I2C_InitStruct.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	// init I2C1 - zapisanie ustawieñ na konkretny numer I2C
	I2C_Init(I2C1, &I2C_InitStruct);
	// W³¹czenie I2C1
	I2C_Cmd(I2C1, ENABLE);
} //i2c_init()

/* I2Cx --> w naszym pzypadku I2C1
 * address --> 7-bitowy adres urzadzenia
 * direction --> kierunek transmisji danych:
 * I2C_Direction_Tranmitter dla transmisji danych z urzadzenia Master
 * I2C_Direction_Receiver dla odbioru danych przez urz¹dzenie Master
 */
void I2C_start(I2C_TypeDef* I2Cx, uint8_t address, uint8_t direction) {
// Oczekiwanie na zwolnienie magistrali I2C, ¿eby nie powodowaæ kolizji
	while (I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY))
		;
// Wygenerowanie sygna³u START
	I2C_GenerateSTART(I2Cx, ENABLE);
// Oczekiwanie na potwierdzenie od Slave'a, ¿e odebra³ sygna³ START
	while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT))
		;
// Adresacja urz¹dzenia Slave i wybór kierunku transmisji danych
	I2C_Send7bitAddress(I2Cx, address, direction);
// Oczekiwanie na potwierdzenie kierunku transmisji
	if (direction == I2C_Direction_Transmitter) {
		while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
			;
	} else if (direction == I2C_Direction_Receiver) {
		while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED))
			;
	}
}

//Zakoñczenie transmisji realizowane jest poprzez wygenerowanie sygna³u STOP, które
//koñczy transmisjê i zwalnia magistralê:
/* Zakoñczenie transmisji i zwolnienie magistrali */
void I2C_stop(I2C_TypeDef* I2Cx) {
// Wygenerowanie sygna³u STOP
	I2C_GenerateSTOP(I2Cx, ENABLE);
}

/* Wysy³anie danych po I2C */
void I2C_write(I2C_TypeDef* I2Cx, uint8_t data) {
	I2C_SendData(I2Cx, data);
// Oczekiwanie na zakoñczenie transmisji
	while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
		;
}

/*Do odbioru danych przygotowano funkcjê I2C_read, której dodatkowy parametr
 ACK decyduje, czy mamy zamiar odczytywaæ wiêcej danych (ENABLE) czy odczytujemy
 ostatni bajt (DISABLE) :*/
/* Funkcja odbieraj¹ca 1 bajt danych oraz w zale¿nosci od parametru ACK:
 * - je¿eli ENABLE -> potwierdzaj¹ca odbiór (oznacza chêæ odbioru jeszcze 1 bajtu)
 * - je¿eli DISABLE -> NIE potwierdzaj¹ca odbioru (zakoñczenie odczytu)*/
uint8_t I2C_read(I2C_TypeDef* I2Cx, uint ACK) {
// W³¹czenie/Wy³¹czenie potwierdzenia odbioru
	I2C_AcknowledgeConfig(I2Cx, ACK);
// Oczekiwania a¿ odebranie bajtu danych zostanie zakoñczone
	while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED))
		;
// Odczyt odebranych danych z I2C
	uint8_t data = I2C_ReceiveData(I2Cx);
	return data;
}

/* odczyt temperatury z czujnika (tu bylu adresy 0x90 oraz 0x15)
Zmienne globalne:
int MCP9800_ADDRESS = 0x90;
int I2C1_SLAVE_ADDRESS7=0x15;
uint8_t zmienna1;
uint8_t zmienna2;

to w main:
// Rozpoczecie transmisji z uC do czujnika
I2C_start(I2C1, MCP9800_ADDRESS, I2C_Direction_Transmitter);
I2C_write(I2C1, 0x00);// Podanie rejestru, z ktorego nastapi odczyt
// Zakonczenie transmisji
I2C_stop(I2C1);
// Glowna petla odczytujaca dane
while (1) {
// Rozpoczecie transmisji odbierajaca dane z MPC9800
	I2C_start(I2C1, MCP9800_ADDRESS, I2C_Direction_Receiver);
// Odczyt 2 bajtów danych
	zmienna1 = I2C_read(I2C1, ENABLE);
	zmienna2 = I2C_read(I2C1, DISABLE);
// Zakonczenie transmisji
	I2C_stop(I2C1);
// Zebranie danych -> zmienna3 to temperatura z dok³adnosci¹ do 0.5 stopnia
	float zmienna3 = zmienna1 + zmienna2/2.0;
}*/
