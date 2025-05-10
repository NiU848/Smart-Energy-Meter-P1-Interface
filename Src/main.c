/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2022 STMicroelectronics
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

/* USER CODE BEGIN Includes */
#include "metroTask.h"
#include "metrology_hal.h"
#include "st_device.h"
//epaper biblioteka
#include "bmp.h"
#include "epaper.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
//#include "epdfont.h"
//#include "epaper.c"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart3;

SPI_HandleTypeDef hspi2;

/* USER CODE BEGIN PV */
//CRC-16/IBM-3740
#define CRC_POLY 0x1021
#define CRC_INIT 0xFFFF  // CRC-16/IBM-3740

uint8_t regAddr = 0x034 | 0x80;
uint8_t regData[3];  
uint32_t oldIGAIN;    

uint8_t unlock_seq1 = 0xCA;
uint8_t unlock_seq2 = 0x35;
uint32_t new_gain = 0x00050000;
uint8_t reg_addr = 0x34; // Gain register for Channel 1

volatile uint16_t g_CalibratorValue = 0; // 'volatile' to prevent optimization


extern int counter;
extern int counter2;
uint8_t image_bw[EPD_W_BUFF_SIZE * EPD_H];


uint32_t voltageRMS = 0;
uint32_t currentRMS = 0;

int32_t activePWR = 0;
int32_t apparentPWR = 0;
int32_t reactivePWR = 0;

float powerFct = 0.00;

int32_t activeEnergy = 0;
int32_t apparentEnergy = 0.00;
int32_t reactiveEnergy = 0.00;

float activePower_W;

uint32_t screenStartTime = 0;
uint32_t sendP1Timer = 0;
uint8_t screenActive = 1;
uint8_t BTNcount = 0;


char  voltageStr[20], currentStr[20],
			factorStr[20],actpwrStr[20],
			reactPwrStr[20], appPwrStr[20],
			actEnergyStr[20], reactEnergyStr[20],
			appEnergyStr[20];

static const uint16_t crc16_table[] = {
	0x0000, 0x1021, 0x2042, 0x3063,   0x4084, 0x50A5, 0x60C6, 0x70E7,
0x8108, 0x9129, 0xA14A, 0xB16B,   0xC18C, 0xD1AD, 0xE1CE, 0xF1EF,
0x1231, 0x0210, 0x3273, 0x2252,   0x52B5, 0x4294, 0x72F7, 0x62D6,
0x9339, 0x8318, 0xB37B, 0xA35A,   0xD3BD, 0xC39C, 0xF3FF, 0xE3DE,
0x2462, 0x3443, 0x0420, 0x1401,   0x64E6, 0x74C7, 0x44A4, 0x5485,
0xA56A, 0xB54B, 0x8528, 0x9509,   0xE5EE, 0xF5CF, 0xC5AC, 0xD58D,
0x3653, 0x2672, 0x1611, 0x0630,   0x76D7, 0x66F6, 0x5695, 0x46B4,
0xB75B, 0xA77A, 0x9719, 0x8738,   0xF7DF, 0xE7FE, 0xD79D, 0xC7BC,
0x48C4, 0x58E5, 0x6886, 0x78A7,   0x0840, 0x1861, 0x2802, 0x3823,
0xC9CC, 0xD9ED, 0xE98E, 0xF9AF,   0x8948, 0x9969, 0xA90A, 0xB92B,
0x5AF5, 0x4AD4, 0x7AB7, 0x6A96,   0x1A71, 0x0A50, 0x3A33, 0x2A12,
0xDBFD, 0xCBDC, 0xFBBF, 0xEB9E,   0x9B79, 0x8B58, 0xBB3B, 0xAB1A,
0x6CA6, 0x7C87, 0x4CE4, 0x5CC5,   0x2C22, 0x3C03, 0x0C60, 0x1C41,
0xEDAE, 0xFD8F, 0xCDEC, 0xDDCD,   0xAD2A, 0xBD0B, 0x8D68, 0x9D49,
0x7E97, 0x6EB6, 0x5ED5, 0x4EF4,   0x3E13, 0x2E32, 0x1E51, 0x0E70,
0xFF9F, 0xEFBE, 0xDFDD, 0xCFFC,   0xBF1B, 0xAF3A, 0x9F59, 0x8F78,
0x9188, 0x81A9, 0xB1CA, 0xA1EB,   0xD10C, 0xC12D, 0xF14E, 0xE16F,
0x1080, 0x00A1, 0x30C2, 0x20E3,   0x5004, 0x4025, 0x7046, 0x6067,
0x83B9, 0x9398, 0xA3FB, 0xB3DA,   0xC33D, 0xD31C, 0xE37F, 0xF35E,
0x02B1, 0x1290, 0x22F3, 0x32D2,   0x4235, 0x5214, 0x6277, 0x7256,
0xB5EA, 0xA5CB, 0x95A8, 0x8589,   0xF56E, 0xE54F, 0xD52C, 0xC50D,
0x34E2, 0x24C3, 0x14A0, 0x0481,   0x7466, 0x6447, 0x5424, 0x4405,
0xA7DB, 0xB7FA, 0x8799, 0x97B8,   0xE75F, 0xF77E, 0xC71D, 0xD73C,
0x26D3, 0x36F2, 0x0691, 0x16B0,   0x6657, 0x7676, 0x4615, 0x5634,
0xD94C, 0xC96D, 0xF90E, 0xE92F,   0x99C8, 0x89E9, 0xB98A, 0xA9AB,
0x5844, 0x4865, 0x7806, 0x6827,   0x18C0, 0x08E1, 0x3882, 0x28A3,
0xCB7D, 0xDB5C, 0xEB3F, 0xFB1E,   0x8BF9, 0x9BD8, 0xABBB, 0xBB9A,
0x4A75, 0x5A54, 0x6A37, 0x7A16,   0x0AF1, 0x1AD0, 0x2AB3, 0x3A92,
0xFD2E, 0xED0F, 0xDD6C, 0xCD4D,   0xBDAA, 0xAD8B, 0x9DE8, 0x8DC9,
0x7C26, 0x6C07, 0x5C64, 0x4C45,   0x3CA2, 0x2C83, 0x1CE0, 0x0CC1,
0xEF1F, 0xFF3E, 0xCF5D, 0xDF7C,   0xAF9B, 0xBFBA, 0x8FD9, 0x9FF8,
0x6E17, 0x7E36, 0x4E55, 0x5E74,   0x2E93, 0x3EB2, 0x0ED1, 0x1EF0
};
/* Private variables ---------------------------------------------------------*/
uint32_t uwPrescalerValue = 0; 

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
//static void SystemClock_Config(void);
static void MX_SPI1_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_RTC_Init(void);
static void MX_SPI2_Init(void);
/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void display_info(){
		epd_paint_selectimage(image_bw);
    epd_paint_clear(EPD_COLOR_WHITE);

    // labeliai
    epd_paint_showString(10, 0, (uint8_t *)"V" , EPD_FONT_SIZE24x12, EPD_COLOR_BLACK);
		
		epd_paint_showString(20, 15, (uint8_t *)"rms" , EPD_FONT_SIZE8x6, EPD_COLOR_BLACK);
		
    epd_paint_showString(140, 0, (uint8_t *)"I" , EPD_FONT_SIZE24x12, EPD_COLOR_BLACK);
		
		epd_paint_showString(150, 15, (uint8_t *)"rms" , EPD_FONT_SIZE8x6, EPD_COLOR_BLACK);
		
		epd_paint_showString(10, 25, (uint8_t *)"P" , EPD_FONT_SIZE24x12, EPD_COLOR_BLACK);
		
		epd_paint_showString(20, 40, (uint8_t *)"Act." , EPD_FONT_SIZE8x6, EPD_COLOR_BLACK);
		
		epd_paint_showString(10, 50, (uint8_t *)"P", EPD_FONT_SIZE24x12, EPD_COLOR_BLACK);
		
		epd_paint_showString(20, 65, (uint8_t *)"Rea." , EPD_FONT_SIZE8x6, EPD_COLOR_BLACK);
	
		epd_paint_showString(10, 75, (uint8_t *)"P", EPD_FONT_SIZE24x12, EPD_COLOR_BLACK);
		
		epd_paint_showString(20, 90, (uint8_t *)"App." , EPD_FONT_SIZE8x6, EPD_COLOR_BLACK);
		
    epd_paint_showString(60, 100, (uint8_t *)"PowerFactor:" , EPD_FONT_SIZE16x8, EPD_COLOR_BLACK);
		
		
		epd_paint_showString(140, 25, (uint8_t *)"E" , EPD_FONT_SIZE24x12, EPD_COLOR_BLACK);
		
		epd_paint_showString(150, 40, (uint8_t *)"Act." , EPD_FONT_SIZE8x6, EPD_COLOR_BLACK);
		
		epd_paint_showString(140, 50, (uint8_t *)"E", EPD_FONT_SIZE24x12, EPD_COLOR_BLACK);
		
		epd_paint_showString(150, 65, (uint8_t *)"Rea." , EPD_FONT_SIZE8x6, EPD_COLOR_BLACK);
		
    epd_paint_showString(140, 75, (uint8_t *)"E" , EPD_FONT_SIZE24x12, EPD_COLOR_BLACK);
		
		epd_paint_showString(150, 90, (uint8_t *)"App." , EPD_FONT_SIZE8x6, EPD_COLOR_BLACK);

    // kintamieji
    epd_paint_showString(45, 0, (uint8_t *)voltageStr, EPD_FONT_SIZE24x12, EPD_COLOR_BLACK);
		
    epd_paint_showString(175, 0, (uint8_t *)currentStr, EPD_FONT_SIZE24x12, EPD_COLOR_BLACK);
		
		
		epd_paint_showString(45, 30, (uint8_t *)actpwrStr, EPD_FONT_SIZE16x8, EPD_COLOR_BLACK); 

		epd_paint_showString(45, 55, (uint8_t *)reactPwrStr, EPD_FONT_SIZE16x8, EPD_COLOR_BLACK);
		
		epd_paint_showString(45, 80, (uint8_t *)appPwrStr, EPD_FONT_SIZE16x8, EPD_COLOR_BLACK);
		
		
		epd_paint_showString(175, 30, (uint8_t *)actEnergyStr, EPD_FONT_SIZE16x8, EPD_COLOR_BLACK); 

		epd_paint_showString(175, 55, (uint8_t *)reactEnergyStr, EPD_FONT_SIZE16x8, EPD_COLOR_BLACK);
		
		epd_paint_showString(175, 80, (uint8_t *)appEnergyStr, EPD_FONT_SIZE16x8, EPD_COLOR_BLACK);
		
    epd_paint_showString(160, 100, (uint8_t *)factorStr, EPD_FONT_SIZE16x8, EPD_COLOR_BLACK);
		
		epd_displayBW_partial(image_bw);

		

}

uint16_t calculate_crc16(const char *data) {
    uint16_t crc = CRC_INIT;

    while (*data) {
        uint8_t index = ((crc >> 8) ^ *data++) & 0xFF;
        crc = (crc << 8) ^ crc16_table[index];
    }
    return crc;
}


void send_p1_data(uint32_t voltage, uint32_t current, int32_t active_power, uint32_t active_energy, 
                  uint32_t apparent_power, uint32_t reactive_power, uint32_t apparent_energy, uint32_t reactive_energy) {
    char message[512]; 
    uint16_t crc;

    snprintf(message, sizeof(message),
        "/EMETERSTPM33\n"
				"\n"
				"0-0:1.0.0(250405161658W)\n"
        "1-0:1.8.0(%09.3f*kWh)\n"
        "1-0:2.8.0(%09.3f*kWh)\n"
        "1-0:1.7.0(%09.3f*kW)\n"
        "1-0:2.7.0(%09.3f*kW)\n"
				"0-0:96.1.0(3034393839353535)\n"
        "1-0:3.7.0(%09.3f*kVAR)\n"
				"1-0:4.7.0(%09.3f*kVAR)\n"
        "1-0:12.7.0(%.2f*V)\n"
        "1-0:31.7.0(%.2f*A)\n"
        "!",
        active_energy / 1000000.0,
				active_energy / 1000000.0,
				active_power / 1000000.0,
		    active_power / 1000000.0,
				reactive_power / 1000000.0,
				reactive_power / 1000000.0,             
        voltage / 1000.0,              
        current / 1000.0               
    );

    //  CRC calc
    crc = calculate_crc16(message);

    // add crc to last row
    snprintf(message + strlen(message), sizeof(message) - strlen(message), "%04X\n", crc);

    // send
    HAL_UART_Transmit(&huart3, (uint8_t *)message, strlen(message), HAL_MAX_DELAY);
}

/* Prescaler declaration */
static void Error_Handler(void);
static void Timer_Init();

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

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  SystemClock_Config();
  MX_SPI1_Init();
  MX_TIM3_Init();
  MX_USART3_UART_Init();
  MX_RTC_Init();
	MX_SPI2_Init();
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET);
  /* USER CODE BEGIN 2 */
  /**************** Code needed for STPM access *******************/ 
		HAL_GPIO_WritePin(LED2_GPIO_type, LED2_GPIO_pin, GPIO_PIN_SET);
  /* Init timer for application measure */
		Timer_Init();
    epd_io_init();
    epd_init();
		
		epd_paint_newimage(image_bw, EPD_W, EPD_H, EPD_ROTATE_180, EPD_COLOR_WHITE);
    epd_paint_selectimage(image_bw);
    epd_paint_clear(EPD_COLOR_WHITE);
		epd_paint_showString(10, 0, (uint8_t *)&"Bakalauro Projektas", EPD_FONT_SIZE24x12, EPD_COLOR_BLACK);
    epd_paint_showString(10, 30, (uint8_t *)&"Normantas Nazarenko", EPD_FONT_SIZE16x8, EPD_COLOR_BLACK);
    epd_paint_showString(10, 50, (uint8_t *)&"EEI-1/1", EPD_FONT_SIZE16x8, EPD_COLOR_BLACK);
		epd_paint_showPicture((EPD_H+91) / 2, (EPD_W - 45) / 2, 80, 88, gImage_4, EPD_COLOR_WHITE);
    epd_displayBW(image_bw);
		
	  epd_enter_deepsleepmode(EPD_DEEPSLEEP_MODE1);

		
		epd_init_partial();
		
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET);
		HAL_Delay(100);
    METRO_Init();

  /* Start Timer Channel1 in interrupt mode*/
  if (HAL_TIM_Base_Start_IT(&htim3) != HAL_OK)
  {
    /* Starting Error */
    Error_Handler();
  }
   /***************************************************************/ 
	

	Metro_HAL_Set_C_Calibration(EXT1,INT_CHANNEL_2,0x02);
  g_CalibratorValue = Metro_HAL_Get_C_Calibration(EXT1, INT_CHANNEL_2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	/**************** Measure application ************************/

    // Nuskaitom STPM
		
    METRO_Latch_Measures();
    METRO_Get_Measures();
    Metro_Read_RMS(1, &voltageRMS, &currentRMS, 1);
    METRO_Update_Measures();
		
		
		activePWR = (abs(metroData.powerActive));
		apparentPWR = (abs(metroData.powerApparent));
		reactivePWR = (abs(metroData.powerReactive));
		
		activeEnergy = (abs(metroData.energyActive));
		apparentEnergy = (abs(metroData.energyApparent));
		reactiveEnergy = (abs(metroData.energyReactive));
		currentRMS = currentRMS;
		
		activePower_W = activePWR / 1000.0f;
		powerFct = (float)activePWR / (float)apparentPWR;
		
		  

		
		/*kintamus pakeiciam i string tipo, kad atvaizduot ekrane + scaling */
		snprintf(factorStr, sizeof(factorStr), "%.2f", powerFct);
				
		if (voltageRMS >= 1000) {
    snprintf(voltageStr, sizeof(voltageStr), "%u.%02uV", voltageRMS / 1000, (voltageRMS % 1000) / 10);
		} else {
				snprintf(voltageStr, sizeof(voltageStr), "%umV", voltageRMS);
		}

		if (currentRMS >= 1000) {
				snprintf(currentStr, sizeof(currentStr), "%u.%02uA", currentRMS / 1000, (currentRMS % 1000) / 10);
		} else {
				snprintf(currentStr, sizeof(currentStr), "%umA", currentRMS);
		}

		// mastelis (mW / W / kW)
		if (activePWR >= 1000000) {  
				snprintf(actpwrStr, sizeof(actpwrStr), "%u.%03ukW", activePWR / 1000000, (activePWR % 1000000) / 1000);
		} else if (activePWR >= 1000) {
				snprintf(actpwrStr, sizeof(actpwrStr), "%u.%02uW", activePWR / 1000, (activePWR % 1000) / 10);
		} else {
				snprintf(actpwrStr, sizeof(actpwrStr), "%umW", activePWR);
		}

		if (reactivePWR >= 1000000) {  
				snprintf(reactPwrStr, sizeof(reactPwrStr), "%u.%03ukVAR", reactivePWR / 1000000, (reactivePWR % 1000000) / 1000);
		} else if (reactivePWR >= 1000) {
				snprintf(reactPwrStr, sizeof(reactPwrStr), "%u.%02uVAR", reactivePWR / 1000, (reactivePWR % 1000) / 10);
		} else {
				snprintf(reactPwrStr, sizeof(reactPwrStr), "%umVAR", reactivePWR);
		}

		if (apparentPWR >= 1000000) {  
				snprintf(appPwrStr, sizeof(appPwrStr), "%u.%03ukVA", apparentPWR / 1000000, (apparentPWR % 1000000) / 1000);
		} else if (apparentPWR >= 1000) {
				snprintf(appPwrStr, sizeof(appPwrStr), "%u.%02uVA", apparentPWR / 1000, (apparentPWR % 1000) / 10);
		} else {
				snprintf(appPwrStr, sizeof(appPwrStr), "%umVA", apparentPWR);
		}

		
		if (activeEnergy >= 1000000) {
				snprintf(actEnergyStr, sizeof(actEnergyStr), "%u.%03ukWh", activeEnergy / 1000000, (activeEnergy % 1000000) / 1000);
		} else if (activeEnergy >= 1000) {
				snprintf(actEnergyStr, sizeof(actEnergyStr), "%u.%02uWh", activeEnergy / 1000, (activeEnergy % 1000) / 10);
		} else {
				snprintf(actEnergyStr, sizeof(actEnergyStr), "%umWh", activeEnergy);
		}

		if (reactiveEnergy >= 1000000) {
				snprintf(reactEnergyStr, sizeof(reactEnergyStr), "%u.%03ukVARh", reactiveEnergy / 1000000, (reactiveEnergy % 1000000) / 1000);
		} else if (reactiveEnergy >= 1000) {
				snprintf(reactEnergyStr, sizeof(reactEnergyStr), "%u.%02uVARh", reactiveEnergy / 1000, (reactiveEnergy % 1000) / 10);
		} else {
				snprintf(reactEnergyStr, sizeof(reactEnergyStr), "%umVARh", reactiveEnergy);
		}

		if (apparentEnergy >= 1000000) {
				snprintf(appEnergyStr, sizeof(appEnergyStr), "%u.%03ukVAh", apparentEnergy / 1000000, (apparentEnergy % 1000000) / 1000);
		} else if (apparentEnergy >= 1000) {
				snprintf(appEnergyStr, sizeof(appEnergyStr), "%u.%02uVAh", apparentEnergy / 1000, (apparentEnergy % 1000) / 10);
		} else {
				snprintf(appEnergyStr, sizeof(appEnergyStr), "%umVAh", apparentEnergy);
		}
		if (voltageRMS > 100000){ /*kadangi matavimu diapazonas yra nuo 100-260V, kai itampa <100V mum yra neaktualu*/
			if(counter2 >= 5){
					display_info();
					counter2 = 0;
			}
		}
		if(counter >= 1){
				if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_3) == GPIO_PIN_SET){
						send_p1_data(voltageRMS, currentRMS, activePWR, activeEnergy, apparentPWR, reactivePWR, apparentEnergy, reactiveEnergy);
				}
			counter = 0;
		}	
		
		
   /**************************************************************/ 
  /* USER CODE END WHILE */

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
void MX_SPI2_Init(void)
{

  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLED;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLED;
  hspi2.Init.CRCPolynomial = 10;
  HAL_SPI_Init(&hspi2);

}
/* TIM3 init function */
void MX_TIM3_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 1151;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 62499;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_Base_Init(&htim3);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig);

}

/* USART3 init function */
void MX_USART3_UART_Init(void)
{

  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
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
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_4
                          |GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);
	/*pc1 konfiguracija mygtukui*/
	
	GPIO_InitStruct.Pin = GPIO_PIN_2;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
	
	GPIO_InitStruct.Pin = GPIO_PIN_3;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
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
  htim3.Init.Period            = 10000 - 1;
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
