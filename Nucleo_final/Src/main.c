/**
*******************************************************************************
 * File Name          : main.c
  * Description        : Main program body (Nucleo)
	* Author             : Gradowski Kamil
	* Project name       : Tulin 
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "SSD1331.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim10;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart1_rx;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
uint16_t Pulsometr;
int ile = 0;
uint32_t czas = 0;
uint8_t puls = 65;
uint8_t stary_puls = 65;
uint8_t zn_puls = 0;
volatile uint8_t k = 0, l = 0, flaga = 0, flaga2 = 0, tempo = 65;
//uint32_t cnt = 0; // Licznik 
uint8_t data[50]; // Tablica przechowujaca wysylana wiadomosc.
uint16_t size = 0; // Rozmiar wysylanej wiadomosci
uint8_t Received[1] = {0} ;
uint8_t sygnal = 0 , flaga_rs =0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM10_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART1_UART_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){  //wywolanie funkcji przerwan zewnetrznych
	/*       Funkcja zliczania impulsow pulsomierza
	zmienna ile przechowuje informacje o ilosci impulsow z pulsomierza*/
	if(GPIO_Pin == PULSE_Pin){              //jezeli przerwanie pochodzi od pinu pulsomierza
	ile = ile + 1;                          // inkrementacja zmiennej  
	if(ile>=30) ile = 0 ;
	}
	
	if(GPIO_Pin == Button1_Pin)
	{
		//if(HAL_GPIO_ReadPin(Button1_GPIO_Port, Button1_Pin) == GPIO_PIN_RESET)	
		//	k++;
		flaga = 1;
	
	}
	if(GPIO_Pin == Button2_Pin)
	{
		l=1;
	}
	
}
/*Przerwanie pochodzace od timera wywolywane po uplynieciu wczesniej okreslonego czasu */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){ 
 if(htim->Instance == TIM10){ // Jezeli przerwanie pochodzi od timera 10
	 czas = czas + 1; // inkrementacja zmiennej sluzacej do obliczania czasu 
	 
	 if(czas >= 10000)      // Wywolywane co 10s
	 {
		/* Zmienna puls przechowuje informacje o bierzacym pulsie
		 Zmienna zn_puls przechowuje wartosc pulsu bedacego srednia wazona
		 obecnego pulsu i wczesniejszego */
		 puls= ile*6;                            // zlicza przez 10s, wiec trzeba pomnozyc przez 6
		 zn_puls = (((stary_puls) + (2*puls) )/3); // usrednienei pulsu z wart poprzenia
		 stary_puls = puls;                      //przypisanie obecnego pulsu jako wczesniejszego
		 ile = 0;                                //wyzerowanie ilosci impulsow
		 czas=0;                                 //wyzerowanie czasu
		 
		 /* decydowanie o tempie */
			if(zn_puls < (tempo - 5)) sygnal = 2 ;         // dyczyja o wyslaniu polecenia przyspiesz
				else if (zn_puls > (tempo + 5)) sygnal = 8;  // dyczyja o wyslaniu polecenia zwolnij
				else sygnal = 3;                             // decyzja o nie zmienianiu tempa
		 
		 /* flaga nakazujaca nadanie wiadomosci */ 
		 flaga_rs = 1; 
		
	 } 
 }
}
 // UART //
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
 
 static uint8_t Data[1]; // Tablica przechowujaca wysylana wiadomosc.
 
 //sprintf(Data, "Odebrana wiadomosc: %s\n\r", Received);
Data[0] = sygnal;
if(flaga_rs == 1)
{
 HAL_UART_Transmit_DMA(&huart1, Data, 1); // Rozpoczecie nadawania danych z wykorzystaniem przerwan
	flaga_rs = 0;
}
 HAL_UART_Receive_DMA(&huart1, Received, 1); // Ponowne wlaczenie nasluchiwania
// HAL_GPIO_TogglePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin);
}

void liczenie_pulsu()
{
	
}

void menu1(uint8_t klik)
{

	//while(l==0)
	{
	switch(klik){
		case 0:
		ssd1331_display_string(0, 0, "    MENU  ", FONT_1608, GREEN);
		ssd1331_draw_line(2, 18, 92, 18, YELLOW);
		ssd1331_display_string(2, 20, " Wybierz puls:", FONT_1206, WHITE);
		ssd1331_display_string(2, 34, "- 100 [1/s]", FONT_1206, GREY);
		tempo = 100;
	
	break;
		
		case 1:
		ssd1331_display_string(0, 0, "    MENU  ", FONT_1608, GREEN);
		ssd1331_draw_line(2, 18, 92, 18, YELLOW);
		ssd1331_display_string(2, 20, " Wybierz puls:", FONT_1206, WHITE);
		ssd1331_display_string(2, 34, "- 110 [1/s]", FONT_1206, GREY);	
		tempo = 110;		
		break;
		
		case 2:
		ssd1331_display_string(0, 0, "    MENU  ", FONT_1608, GREEN);
		ssd1331_draw_line(2, 18, 92, 18, YELLOW);
		ssd1331_display_string(2, 20, " Wybierz puls:", FONT_1206, WHITE);
		ssd1331_display_string(2, 34, "- 120 [1/s]", FONT_1206, GREY);	
		tempo = 120;
		break;
		
		case 3:
		ssd1331_display_string(0, 0, "    MENU  ", FONT_1608, GREEN);
		ssd1331_draw_line(2, 18, 92, 18, YELLOW);
		ssd1331_display_string(2, 20, " Wybierz puls:", FONT_1206, WHITE);
		ssd1331_display_string(2, 34, "- 130 [1/s]", FONT_1206, GREY);	
		tempo = 130;
		break;
		
	}
	
}
}

void menu2(uint8_t klik)
{
	
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
  MX_ADC1_Init();
  MX_TIM10_Init();
  MX_SPI2_Init();
  MX_USART1_UART_Init();

  /* USER CODE BEGIN 2 */
HAL_ADC_Start_DMA(&hadc1, (uint32_t*)Pulsometr, 1);
HAL_TIM_Base_Start_IT(&htim10);

//HAL_UART_Transmit_IT(&huart1, data, size); // Rozpoczecie nadawania danych z wykorzystaniem przerwan

ssd1331_init();
ssd1331_clear_screen(BLACK);
ssd1331_display_string(0, 10, " Tulin_v.2 ", FONT_1608, RED);
ssd1331_display_string(0, 50, " GRADOWSKI Kamil", FONT_1206 , YELLOW);

HAL_Delay(1000);
ssd1331_clear_screen(BLACK);
ssd1331_display_string(0, 0, "    MENU  ", FONT_1608, GREEN);
ssd1331_draw_line(2, 18, 92, 18, YELLOW);
ssd1331_display_string(2, 20, " Wybierz puls:", FONT_1206, WHITE);



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE END WHILE */
		while(l==0)
		{
	while(flaga ==1)
	{
		while(HAL_GPIO_ReadPin(Button1_GPIO_Port, Button1_Pin) == GPIO_PIN_SET)	
		{
			ssd1331_clear_screen(BLACK);
			k++;
			if(k>=4) k=0;
			flaga =0;
			break;
			
		}
	}
menu1(k);
}
		if(flaga2 == 0)
	{	
	ssd1331_display_string(0, 0, "    MENU  ", FONT_1608, GREEN);
	ssd1331_draw_line(2, 18, 92, 18, YELLOW);
	ssd1331_display_string(2, 50, " WYBRALES PULS", FONT_1206, RED);
	HAL_Delay(1000);
	ssd1331_clear_screen(BLACK);
	ssd1331_display_string(0, 10, "  START za:  ", FONT_1608, GREEN);
	ssd1331_display_string(0, 30, "    3      ", FONT_1608, RED);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET);
	HAL_Delay(1000);
	ssd1331_clear_screen(BLACK);
	ssd1331_display_string(0, 10, "  START za:  ", FONT_1608, GREEN);
	ssd1331_display_string(0, 30, "    2      ", FONT_1608, RED);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_SET);
	HAL_Delay(1000);
	ssd1331_clear_screen(BLACK);
	ssd1331_display_string(0, 10, "  START za:  ", FONT_1608, GREEN);
	ssd1331_display_string(0, 30, "    1      ", FONT_1608, RED);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET);
	HAL_Delay(1000);
	ssd1331_clear_screen(BLACK);
	ssd1331_display_string(0, 10, "  START za:  ", FONT_1608, GREEN);
	ssd1331_display_string(0, 30, "    0      ", FONT_1608, RED);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_SET);
	flaga2=1;	
		
 }

	HAL_Delay(1000);
	ssd1331_clear_screen(BLACK);
	ssd1331_display_string(3, 30, " ! START !", FONT_1608, RED);
  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
  hadc1.Init.NbrOfConversion = 1;
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
  sConfig.SamplingTime = ADC_SAMPLETIME_144CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* SPI2 init function */
static void MX_SPI2_Init(void)
{

  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM10 init function */
static void MX_TIM10_Init(void)
{

  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 83;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 999;
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

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
  /* DMA2_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
     PA2   ------> USART2_TX
     PA3   ------> USART2_RX
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, CS_Pin|DC_Pin|RES_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Button2_Pin Button1_Pin Button3_Pin */
  GPIO_InitStruct.Pin = Button2_Pin|Button1_Pin|Button3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : USART_TX_Pin USART_RX_Pin */
  GPIO_InitStruct.Pin = USART_TX_Pin|USART_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PULSE_Pin */
  GPIO_InitStruct.Pin = PULSE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(PULSE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : CS_Pin DC_Pin RES_Pin */
  GPIO_InitStruct.Pin = CS_Pin|DC_Pin|RES_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PC10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

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
