/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body (Disco)
	* Author             : Gradowski Kamil
	* Project name       : Tulin 
  ******************************************************************************

  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim10;
DMA_HandleTypeDef hdma_tim2_ch1;
DMA_HandleTypeDef hdma_tim3_ch1_trig;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
uint32_t Duty = 0; //SKRET
uint32_t Duty2 = 83; // PREDKOSC
uint32_t serwo = 21; // SKRET
uint16_t czujniki[5];
uint16_t tablica[5];
uint32_t kalibracja[5] = {0,0,0,0,0};
uint16_t prog = 1900;
uint8_t status = 0 ;
uint16_t czas_kalibracji = 100;
volatile uint8_t petla = 0; 
int wagi[7] = {-20, -10, 0, 10, 20};
int blad=0, pop_blad =0 , Kp = 1 , Kd = 1; 
uint32_t ile = 0;
volatile int test;
//static uint16_t cnt = 0; // Licznik wyslanych wiadomosci
uint8_t Data[50]; // Tablica przechowujaca wysylana wiadomosc.
uint16_t size = 0; // Rozmiar wysylanej wiadomosci
uint8_t Received;
float kb =0.6;
uint8_t flaga_nucleo;
uint32_t cnt_f =0;
uint8_t poprawka = 0;
uint8_t wiad = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM1_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM10_Init(void);
static void MX_USART1_UART_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                
                                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
 if(GPIO_Pin == Button_Pin){
	 status =!status;               // ON/OFF
	 	if(status == 1) Duty2 = 25;	  // predkosc poczatkowa - ON
		else Duty2 = 0;               // OFF
 }
 }
//Przerwanie 100Hz, do glownej petli programu
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
 if(htim->Instance == TIM10){	 
	 petla = 1;
	 cnt_f++;
 }
 }
 
// Przerwanie wywolywane z czestotliwoscia 1KHz
void HAL_SYSTICK_Callback(void)
	
	{
		

		
  }
	
	//FUNKCJA DO USTALANIA PROGU - KALIBRACJI CZUJNIK�W
	void kalibracja_progu()
	{
		for(int j = 0 ; j < czas_kalibracji ; j++) 
		{
			for( int i = 0 ; i < 5 ; i++)
			{
			kalibracja[i] = kalibracja[i] + czujniki[i]; // Sumowanie wartosci z czujnik�w linii
			}
			
		}
		for( int k = 0 ; k < 5 ; k++)
		{
			kalibracja[k]= (kalibracja[k] / czas_kalibracji) ; //usrednienie wartosci z czujnik�w
		}
		
	}
	
	// FUNKCJA DETEKCJI LINII
	void czytaj_adc()
	{
		for( int i = 0 ; i < 5 ; i++)                // przeszukiwanie tablicy z wartosciami napiec
		{
			if(czujniki[i] > ( kalibracja[i] + prog) ) // zamiana wartosci napiec z transoptorow
			{                                          // na tablice zawierajaca informacje o wykryciu linii
				tablica[i] = 1;                          // wykrycie linii
			}
			else
			{
				tablica[i] = 0;                           // brak wykrycia linii
			}
				
		}
		
	}


	int licz_blad() 
{ 
    int err = 0;                         // wartosc bledu 
    int ilosc = 0;                        // ilosc czujnik�w z wykryta linia 
    
    for(int i=0; i<5; i++) 
    { 
        err += tablica[i]*wagi[i];        // wymnozenie czujnikow przez wagi 
        ilosc += tablica[i];              // tablica[i] ma wartosc 1/0 wiec zliczamy 
    }                                     // czujniki pod kt�rymi wykryto linie
    
    err = err / ilosc;                        // usrednienie wartosci bledu
    
    return err; 
}

int PD() 
{ 
    //wynik funkcji licz_blad() przechowywany jest w zmiennej blad
    int rozniczka = blad - pop_blad;   // obliczanie czlonu rozniczkujacego
    pop_blad = blad; 
    return Kp*blad + Kd*rozniczka;    // przypisanie wag do czlonow
}
/* Funkcja szukania linii po zgubieniu */
void szukanie_ln()
{
	uint8_t ilosc_cz = 0;
	uint8_t sz_blad = 0;
	uint8_t czas_zgubienia = 0;
	
	/* przeszukiwanie tablicy czujnikow */
	for(int l=0; l<5 ; l++)
	{
		ilosc_cz += tablica[l]; //zliczanie ilosci czujnikow pod ktorymi jest linia
	}
	/* jezeli wykryto linie i blad jest wiekszy od zera */
	if((ilosc_cz > 0) ^ (blad > 0)) 
	{
		sz_blad = blad;        // zachowanie wartosci bledu w zmiennej sz_blad
		czas_zgubienia = 0;
	}
	else                     // nie ma lini pod czujnikami
	{
		czas_zgubienia++;      // inkrementacja czasu braku lini pod czujnikami
		/* jezeli nie ma linii pod czujnikami i jezeli blad wynosi 0
		i jezeli trwa to od 0,5 s */
		if ((blad == 0) ^ (czas_zgubienia > 50))
		{
			blad = sz_blad;     // przypisanie do aktualnego bledu ostatnia porawna wartosc
		}
	}
}
/* Funkcja awaryjnego lub zdalnego zatrzymania robota */
void stop()
{
	if(flaga_nucleo == 6)     // jesli otrzymano polecenie STOP
	{
		status = 0;             // zatrzymaj silnik
	}
	else if( cnt_f == 2000)   //jesli przez 20s nie otrzymano zadnego polecenia
	{
		if(flaga_nucleo == 0)
		{
			status = 0;          // zatrzymaj silnik
		}
		flaga_nucleo = 0;      // wyzerowanie flagi wiadomosci
		cnt_f = 0;             // wyzerowanie licznika czasu
	}
	
}
//  UART

// UART!!!!!!!!!! - Bluetooth
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
 
 uint8_t Data[50]; // Tablica przechowujaca wysylana wiadomosc.
 uint16_t size = 0; // Rozmiar wysylanej wiadomosci
 
 // Odebrany znak zostaje przekonwertowany na liczbe calkowita i sprawdzony
 // instrukcja warunkowa 
	
 switch (atoi((char*)&Received)) {
 
 case 2: // Jezeli odebrany zostanie znak 2
 wiad = 1;
 HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);
 flaga_nucleo = 2;
 
 break;
 
 case 8: // Jezeli odebrany zostanie znak 8
wiad = 1;
 HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);
 flaga_nucleo = 8;
 
 break;
 
 case 6: // Jezeli odebrany zostanie znak 6
wiad = 1;
 flaga_nucleo = 6;
 
 
 break;
 
 case 3: // Jezeli odebrany zostanie znak 3
wiad = 1;
 flaga_nucleo = 3;
 
 
 //default: // Jezeli odebrano nieobslugiwany znak
 //size = sprintf(Data, "Odebrano nieznany znak: %c\n\r", Received[10]);
 
 }
 HAL_UART_Transmit_IT(&huart1, Data, size); // Rozpoczecie nadawania danych z wykorzystaniem przerwan
 HAL_UART_Receive_IT(&huart1, &Received, 1); // Ponowne wlaczenie nasluchiwania
 }

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM3_Init();
  MX_TIM1_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_TIM10_Init();
  MX_USART1_UART_Init();

  /* USER CODE BEGIN 2 */
HAL_TIM_PWM_Start_DMA(&htim3, TIM_CHANNEL_1, &Duty, 1);
HAL_TIM_PWM_Start_DMA(&htim2, TIM_CHANNEL_1, &Duty2, 1);
HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
HAL_ADC_Start_IT(&hadc1);
HAL_ADC_Start_DMA(&hadc1, (uint32_t*)czujniki, 5);
HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_RESET);
HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);
HAL_TIM_Base_Start_IT(&htim10);
HAL_UART_Receive_IT(&huart1, &Received, 1);

kalibracja_progu();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
szukanie_ln();
Duty = serwo;                                    		// sterowanie kolami
if( Duty >= 50) Duty = 50;													 // ograniczenie ruchow serwa
if (Duty <=0) Duty = 0;		
		
		// JAZDA. STATUS I PREDKOSC
	if(wiad == 1)
	{
	if(flaga_nucleo == 2 ) Duty2 = Duty2 + 5;         //przyspiesz
	else if(flaga_nucleo == 8) Duty2 = Duty2 - 5;	    //zwolnij
		wiad = 0;
	}
   	
		if(petla)
		{
			HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14);
			
		  czytaj_adc();
			
			blad = licz_blad();
			
			poprawka = PD();  // regulator PD
// nanoszenie poprawki - reakcja na blad
				serwo = 21 + (kb*poprawka);
				if( serwo >= 50) serwo = 50;
				if (serwo <=0) serwo = 0;	
			
			petla = 0;
		}
		
  /* USER CODE END 3 */

}
	}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 8;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC1 init function */
static void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV8;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 5;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = 4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = 5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM1 init function */
static void MX_TIM1_Init(void)
{

  TIM_Encoder_InitTypeDef sConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 403;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 15;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 15;
  if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 19;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 99;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim2);

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 4999;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 199;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim3);

}

/* TIM10 init function */
static void MX_TIM10_Init(void)
{

  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 99;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 9999;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
     PC3   ------> I2S2_SD
     PA4   ------> I2S3_WS
     PA5   ------> SPI1_SCK
     PA6   ------> SPI1_MISO
     PA7   ------> SPI1_MOSI
     PB10   ------> I2S2_CK
     PB12   ------> I2S2_WS
     PC7   ------> I2S3_MCK
     PA9   ------> USB_OTG_FS_VBUS
     PA10   ------> USB_OTG_FS_ID
     PA11   ------> USB_OTG_FS_DM
     PA12   ------> USB_OTG_FS_DP
     PC10   ------> I2S3_CK
     PC12   ------> I2S3_SD
     PB9   ------> I2C1_SDA
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15 
                          |IN1_Pin|IN2_Pin|GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin : PE2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : PE3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PE4 PE5 PE1 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : PC0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PC3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : Button_Pin */
  GPIO_InitStruct.Pin = Button_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Button_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA5 PA6 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB10 PB12 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PD12 PD13 PD14 PD15 
                           IN1_Pin IN2_Pin PD4 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15 
                          |IN1_Pin|IN2_Pin|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PC7 PC10 PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_10|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA10 PA11 PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PD5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
