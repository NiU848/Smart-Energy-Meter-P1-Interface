/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2020 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include "fatfs.h"

/* USER CODE BEGIN Includes */
/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include "metroTask.h"
#include "st_device.h"
#include "stdio.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
RTC_HandleTypeDef hrtc;

SD_HandleTypeDef hsd;
HAL_SD_CardInfoTypedef SDCardInfo;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim3;
/* Private variables ---------------------------------------------------------*/
uint32_t uwPrescalerValue = 0; 

char gauta_komanda=0;
int timerio_laikas;
char patikra=0;
int laikas1=0;
long long int galia_mW;
long long int itampa_mV;
long long int srove_mA;

char i=0;
	char myData[27] ;

RTC_TimeTypeDef currentTime;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void SystemClock_Config(void);
static void MX_SPI1_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_UART4_Init(void);
static void MX_RTC_Init(void);
static void MX_SDIO_SD_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
/* Private function prototypes -----------------------------------------------*/
/* Prescaler declaration */
static void Error_Handler(void);
static void Timer_Init(void);

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
/* Private function prototypes -----------------------------------------------*/
#ifdef __GNUC__
  /* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
PUTCHAR_PROTOTYPE  
{
    /* Place your implementation of fputc here */
    /* e.g. write a character to the USART */
    HAL_UART_Transmit(&huart4, (uint8_t *)&ch, 1, 100);


    return ch;
}
/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  SystemClock_Config();
  MX_SPI1_Init();
  MX_TIM3_Init();
  MX_USART3_UART_Init();
  MX_UART4_Init();
  MX_RTC_Init();
  MX_SDIO_SD_Init();
  MX_FATFS_Init();

  /* USER CODE BEGIN 2 */
  /**************** Code needed for STPM access *******************/ 
	    HAL_GPIO_WritePin(LED2_GPIO_type, LED2_GPIO_pin,GPIO_PIN_SET);	
			HAL_Delay(10000);
      HAL_GPIO_WritePin(LED2_GPIO_type, LED2_GPIO_pin,GPIO_PIN_RESET);	
			printf("STARTAS\r\n");  
  /* Init timer for application measure */
  Timer_Init();
	
	FATFS myFATAFS;
FIL myFILE;
//char abc;
unsigned int testByte;
//FRESULT fr; 
 /*   if(f_mount(&myFATAFS,SD_Path, 1)== FR_OK){
	 
	
	char myPath[] = "TESTAS2.TXT\n";
//			char pradziaaa[]="pradzia\n";
	f_open(&myFILE, myPath, FA_WRITE | FA_CREATE_ALWAYS);
			
			
			
f_close(&myFILE);
	 HAL_GPIO_TogglePin(LED2_GPIO_type, LED2_GPIO_pin);
} */
		int fileNumber =0;
	char name[16];

	//Give a work area to the default drive 
	f_mount(&myFATAFS,SD_Path, 1);

	//check if filename exist
	sprintf(name, "test%d.TXT", fileNumber);
	while(f_stat(name,NULL)==FR_OK)
	{
		fileNumber++;
		sprintf(name, "test%d.TXT", fileNumber);
	}
					
	//if filename is new create file
	f_open(&myFILE, name, FA_WRITE | FA_CREATE_ALWAYS);
	
  /* Init STPM with complete sequence and set the registers*/
  METRO_Init();/*this function initializes the STPM3x (POR seqiemce), 
	//configures the STPM3x by writing internal registers and the selects the latch type*/

  /* Start Timer Channel1 in interrupt mode*/
  if (HAL_TIM_Base_Start_IT(&htim3) != HAL_OK)
  {
    /* Starting Error */
    Error_Handler();
  }
   /***************************************************************/ 
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
    
  {
//	HAL_TIM_Base_Stop_IT(&htim3);    	
//timerio_laikas = __HAL_TIM_GetCounter(&htim3);    //read TIM2 counter value
		
		//paleidžiamas timeris 
//	if((HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_0)+((HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_1))<<1)==1)&&(patikra==0)){
	  /* Start Timer Channel1 in interrupt mode*/
 // if (HAL_TIM_Base_Start_IT(&htim3) != HAL_OK)
 // {
    /* Starting Error */
//    Error_Handler();
		
//  }
//	patikra=1;
	
//	METRO_Init();/*this function initializes the STPM3x (POR seqiemce), 
	//configures the STPM3x by writing internal registers and the selects the latch type*/
//	}	
	
	//stabdomas timeris /*
//		if((HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_0)+((HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_1))<<1)==0)&&(patikra==1)){
	  /* Start Timer Channel1 in interrupt mode*/
//		HAL_TIM_Base_Stop_IT(&htim3);
//		__HAL_TIM_SET_COUNTER(&htim3, 0);
//	patikra=0;
			//METRO_Init();//
//	}	

//		__HAL_TIM_SET_COUNTER(&htim3, 0);
//timerio_laikas = __HAL_TIM_GetCounter(&htim3);    //read TIM2 counter value
	
//H//AL_TIM_SET_COUNTER(&hTim3, 0);

		//   SLAVE DALIS
		
					
					
		/**************** Measure application ************************/
    if (metroData.metroTimerActive == 1)
    {
			//HAL_TIM_ReadCapturedValue
			//gal cia problema su Latchinimu?
			//HAL_Delay(20); //uzvelint 

			gauta_komanda=HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_0)+((HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_1))<<1);		
				if(gauta_komanda==1){
					HAL_GPIO_WritePin(GPIOC,GPIO_PIN_2,GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOC,GPIO_PIN_3,GPIO_PIN_SET);}
			else if(gauta_komanda==2){
					HAL_GPIO_WritePin(GPIOC,GPIO_PIN_2,GPIO_PIN_SET);
					HAL_GPIO_WritePin(GPIOC,GPIO_PIN_3,GPIO_PIN_SET);}
			else if(gauta_komanda==3){
					HAL_GPIO_WritePin(GPIOC,GPIO_PIN_2,GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOC,GPIO_PIN_3,GPIO_PIN_RESET);}				

	//		HAL_Delay(25); //uzvelint 
      METRO_Latch_Measures(); // METRO_latch_Measures: it handles the STPM3x latch
      METRO_Get_Measures();
      METRO_Update_Measures(); // METRO_Update_Measures: it reads the data from the STPM3x
					
					
if(f_mount(&myFATAFS,SD_Path, 1)== FR_OK){
	 
	
//	char myPath[] = "TESTAS2.TXT\n";
	
	f_open(&myFILE, name, FA_WRITE | FA_OPEN_ALWAYS);
	f_lseek(&myFILE, f_size(&myFILE));
//	i++;
//	printf("%i,\%i,\%i,\%d,\r\n",((metroData.powerActive)*144)/100,(metroData.rmsvoltage),(metroData.rmscurrent*144/100),gauta_komanda);
  //memset(myData, 0, sizeof(myData));
	sprintf(myData, "\n%9i,\%7i,\%5i,\%1d",((metroData.powerActive)*144)/100,(metroData.rmsvoltage),(metroData.rmscurrent*144/100),gauta_komanda);
	HAL_UART_Transmit(&huart4, (uint8_t *)&myData, sizeof(myData), 100);
//	f_puts("\n%i,\%i,\%i,\%d\0",((metroData.powerActive)*144)/100,(metroData.rmsvoltage),(metroData.rmscurrent*144/100),gauta_komanda);
	f_write(&myFILE, myData, sizeof(myData), &testByte);
  f_close(&myFILE);
//	HAL_Delay(10);
	 HAL_GPIO_TogglePin(LED2_GPIO_type, LED2_GPIO_pin);
      metroData.metroTimerActive = 0;
				

					
					
//			printf("%i,\%i,\%i,\%d,\r\n",((metroData.powerActive)*144)/100,(metroData.rmsvoltage),(metroData.rmscurrent*144/100),gauta_komanda);
//			HAL_GPIO_TogglePin(LED2_GPIO_type, LED2_GPIO_pin);

}
			
	//		laikas1++;
/*
			galia_mW=galia_mW+metroData.powerActive;
			itampa_mV=itampa_mV+metroData.rmsvoltage;
			srove_mA=srove_mA+metroData.rmscurrent;
	  	gauta_komanda=HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_0)+((HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_1))<<1);
			
//			if(laikas1==10){
//			HAL_Delay(25);	
			galia_mW=galia_mW/laikas1;
			itampa_mV=itampa_mV/laikas1;
			srove_mA=srove_mA/laikas1;
	*/		
//	gauta_komanda=HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_0)+((HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_1))<<1);

/*			
			galia_mW=0;
			itampa_mV=0;
			srove_mA=0;		
			laikas1=0; */
			}
		

    }
   /**************************************************************/ 
 

		
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

//  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* RTC init function */
void MX_RTC_Init(void)
{

  RTC_TimeTypeDef sTime;
  RTC_DateTypeDef DateToUpdate;

    /**Initialize RTC and set the Time and Date 
    */
  hrtc.Instance = RTC;
  hrtc.Init.AsynchPrediv = RTC_AUTO_1_SECOND;
  hrtc.Init.OutPut = RTC_OUTPUTSOURCE_NONE;
  HAL_RTC_Init(&hrtc);

  sTime.Hours = 0x1;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;

  HAL_RTC_SetTime(&hrtc, &sTime, FORMAT_BCD);

  DateToUpdate.WeekDay = RTC_WEEKDAY_MONDAY;
  DateToUpdate.Month = RTC_MONTH_JANUARY;
  DateToUpdate.Date = 0x1;
  DateToUpdate.Year = 0x0;

  HAL_RTC_SetDate(&hrtc, &DateToUpdate, FORMAT_BCD);

}

/* SDIO init function */
void MX_SDIO_SD_Init(void)
{

  hsd.Instance = SDIO;
  hsd.Init.ClockEdge = SDIO_CLOCK_EDGE_RISING;
  hsd.Init.ClockBypass = SDIO_CLOCK_BYPASS_DISABLE;
  hsd.Init.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_DISABLE;
  hsd.Init.BusWide = SDIO_BUS_WIDE_1B;
  hsd.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd.Init.ClockDiv = 0;

}

/* SPI1 init function */
void MX_SPI1_Init(void)
{

  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLED;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLED;
  hspi1.Init.CRCPolynomial = 10;
  HAL_SPI_Init(&hspi1);

}

/* TIM3 init function */
void MX_TIM3_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_IC_InitTypeDef sConfigIC;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 0;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_Base_Init(&htim3);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig);

  HAL_TIM_IC_Init(&htim3);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig);

  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_1);

}

/* UART4 init function */
void MX_UART4_Init(void)
{

  huart4.Instance = UART4;
  huart4.Init.BaudRate = 9600;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&huart4);

}

/* USART3 init function */
void MX_USART3_UART_Init(void)
{

  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&huart3);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __GPIOC_CLK_ENABLE();
  __GPIOD_CLK_ENABLE();
  __GPIOA_CLK_ENABLE();
  __GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2|GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_4 
                          |GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);

  /*Configure GPIO pins : PC0 PC1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PC2 PC3 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA1 PA2 PA3 
                           PA4 PA8 PA9 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3 
                          |GPIO_PIN_4|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
  /**************** Code needed for simple application *******************/ 

/**
  * @brief  Period elapsed callback in non blocking mode
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  metroData.metroTimerActive = 1;
}
/**
  * @brief  Configure timer with 1s period
  * @param  None
  * @retval None
  */

static void Timer_Init()
{
    /*##-1- Configure the TIM peripheral #######################################*/
  /* -----------------------------------------------------------------------
    In this example TIM3 input clock (TIM3CLK)  is set to APB1 clock (PCLK1) x2,
    since APB1 prescaler is set to 4 (0x100).
       TIM3CLK = PCLK1*2
       PCLK1   = HCLK/2
    => TIM3CLK = PCLK1*2 = (HCLK/2)*2 = HCLK = SystemCoreClock
    To get TIM3 counter clock at 10 KHz, the Prescaler is computed as following:
    Prescaler = (TIM3CLK / TIM3 counter clock) - 1
    Prescaler = (SystemCoreClock /10 KHz) - 1

    Note:
     SystemCoreClock variable holds HCLK frequency and is defined in system_stm32f1xx.c file.
     Each time the core clock (HCLK) changes, user had to update SystemCoreClock
     variable value. Otherwise, any configuration based on this variable will be incorrect.
     This variable is updated in three ways:
      1) by calling CMSIS function SystemCoreClockUpdate()
      2) by calling HAL API function HAL_RCC_GetSysClockFreq()
      3) each time HAL_RCC_ClockConfig() is called to configure the system clock frequency
  ----------------------------------------------------------------------- */

  /* Compute the prescaler value to have TIMx counter clock equal to 10000 Hz */
  uwPrescalerValue = (uint32_t)(SystemCoreClock / 10000) - 1;

  /* Set TIMx instance */
  htim3.Instance = TIM3;

  /* Initialize TIMx peripheral as follows:
       + Period = 10000 - 1
       + Prescaler = (SystemCoreClock/10000) - 1
       + ClockDivision = 0
       + Counter direction = Up
  */
  htim3.Init.Period            = 2000 - 1;
  htim3.Init.Prescaler         = uwPrescalerValue;
  htim3.Init.ClockDivision     = 0;
  htim3.Init.CounterMode       = TIM_COUNTERMODE_UP;
  htim3.Init.RepetitionCounter = 0;

  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }
  
}


/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
static void Error_Handler(void)
{
  while (1)
  {
  }
}


 /***************************************************************/ 

/* USER CODE END 4 */

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
