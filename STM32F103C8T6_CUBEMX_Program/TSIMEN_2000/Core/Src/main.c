/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "sht30_i2c_driver.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

//----- Miscellaneous variable definitions
uint8_t CRC_j = 0, CRC_i = 0, WHILEA = 0, memv_i = 0, A = 0;
uint32_t WAIT_TIME = 0x00000000;
uint16_t crc = 0xFFFF, CRC16 = 0;
uint8_t CRC_DATA[8], CHECKDATA[8], SPECTRAL_DATA[2063], DoubleBuf[8],sadasee;
// double Wave_Coefficient[6] = {0.11,-0.111,0.01,0.02,0.003,121.04},Wavelength[1024] = {0};
double Wave_Coefficient[6] = {0.11,-0.1232,0.332,0.1421,0.223,181.87652}, Wavelength;
float Energy[16388];
int PWM_DARK = 950, PWM_REFERENCE = 1130, PWM_SAMPLE = 700;
long long int Dou_i = 0;

//----- Master computer communication command (11 instructions)
uint8_t USART1_Tsimen_Reset[]     = {0x01,0x01,0x00,0x00,0x00,0x00,0x0A,0x3C};      // Global reset
// uint8_t USART1_Tsimen_Version[]   = {'T','S','-','2','0','0','0','-','0','0','0','0','0','1'};        // Tsimen Code Version
uint8_t USART1_Tsimen_Version[]   = "TS-2000-000001";                               // Tsimen Code Version
uint8_t USART1_Tsimen_Ver[]       = {0x01,0x02,0x00,0x00,0x00,0x00,0x0A,0x78};      // Read the sensor version
uint8_t USART1_Tsimen_Int[]       = {0x01,0x03,0x00,0x00,0x01,0xF4,0xDD,0x45};      // Set the spectrometer integration time (example)
uint8_t USART1_Tsimen_CheckInt[]  = {0x01,0x04,0x00,0x00,0x00,0x00,0x0A,0xF0};      // Check the current integration time of the spectrometer
uint8_t USART1_Tsimen_Ave[]       = {0x01,0x05,0x00,0x32,0x00,0x00,0x05,0x6C};      // Set the spectrometer average Times (example)
uint8_t USART1_Tsimen_CheckAve[]  = {0x01,0x06,0x00,0x00,0x00,0x00,0xCA,0x89};      // Check the current average times of the spectrometer
uint8_t USART1_Tsimen_DarSignal[] = {0x01,0x07,0x00,0x00,0x00,0x00,0x0A,0xB4};      // Read the spectrometer data under dark current conditions
uint8_t USART1_Tsimen_RefSignal[] = {0x01,0x08,0x00,0x00,0x00,0x00,0x0B,0xE0};      // Read the spectrometer data under reference signal conditions
uint8_t USART1_Tsimen_SamSignal[] = {0x01,0x09,0x00,0x00,0x00,0x00,0xCB,0xDD};      // Read the spectrometer data under sample signal conditions
uint8_t USART1_Tsimen_FullData[]  = {0x01,0x0A,0x00,0x00,0x00,0x00,0xCB,0x99};      // One-click acquisition of dark, parametric and sample spectrometer data
uint8_t USART1_Tsimen_TempData[]  = {0x01,0x0B,0x00,0x00,0x00,0x00,0x0B,0xA4};      // Read temperature and humidity data
uint8_t USART1_Tsimen_Jump[]      = {0x01,0x0C,0x00,0x00,0x00,0x00,0xCB,0x11};      // Send message to the STM32 for                                                              w'd jumping out of loop.
uint8_t USART1_Tsimen_OK[]        = {0x01,0x52,0x49};                               // OK code
uint8_t USART1_Tsimen_ERROR[]     = {0x01,0x46,0x41};                               // ERROR code
uint8_t USART1_Tsimen_SpecOK[]    = {0x01,0x53,0x50,0x45,0x43,0x4f,0x4b};           // Handle with the Spectrometer
uint8_t USART1_Tsimen_SpecERROR[] = {0x01,0x53,0x50,0x45,0x43,0x45,0x52};           // Spectrometer has some problems, maybe it is broken
uint8_t USART1_Tsimen_CRCERROR[]  = {0x01,0x43,0x52,0x43,0x45,0x52};                // CRC ERROR
uint8_t USART1_Tsimen_WL[]        = {0x01,0x0D,0x00,0x00,0x00,0x00,0x0B,0x2C};      // Read the calibrated wavelength
uint8_t USART1_Tsimen_SetWL[]     = {0x01,0x0E,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xC4,0xCE};
uint8_t USART1_Tsimen_WC[]        = {0x01,0x0F,0x00,0x00,0x00,0x00,0xCB,0X55};      // Read the wavelength cofficient
uint8_t USART1_Tsimen_SetEN[]     = {0x01,0x10};                                    // Set the energy cofficient


//----- Spectrometer communication command ()
uint8_t USART2_Spec_Reset[]       = {0x52,0xBD,0x3E};                               // Spec reset
uint8_t USART2_Spec_Int[]         = {0x69,0x00,0x00,0x01,0xF4,0x1E,0x78};           // Set spec int time
uint8_t USART2_Spec_Int_Default[] = {0x69,0x00,0x00,0x01,0xF4,0x1E,0x78};           // Default spec int time
uint8_t USART2_Spec_CheckInt[]    = {0x3F,0x69,0x6E,0xD0};                          // Check spec int time
uint8_t USART2_Spec_Ave[]         = {0x41,0x00,0x14,0xDB,0x21};                     // Set spec average
uint8_t USART2_Spec_Ave_Default[] = {0x41,0x00,0x32,0x01,0xA0};                     // Default spec average
uint8_t USART2_Spec_CheckAve[]    = {0x3F,0x41,0x70,0xD0};                          // Check spec average
uint8_t USART2_Spec_XenonOff[]    = {0x31,0x00,0x20,0x14};                          // Turn the xenon off
uint8_t USART2_Spec_XenonOnCon[]  = {0x31,0x01,0xE0,0xD5};                          // Turn the xenon on under continuous mode
uint8_t USART2_Spec_XenonOnOne[]  = {0x31,0x81,0x40,0xD4};                          // Turn the xenon on under signal mode
uint8_t USART2_Spec_Data[]        = {0x53,0x7D,0xFF};                               // Read spec data
uint8_t USART2_Spec_OK[]          = {0x06,0x42,0x3F};                               // OK code
uint8_t USART2_Spec_ERROR[]       = {0x15,0x8F,0x7E};                               // ERROR code
uint8_t USART2_Spec_FormData[]    = {0x06,0xAA,0x55,0xBB,0x44,0xCC,0x33,0xDD,0x22}; // Spectrometer Form Data


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  memset(I2CRXBuffer,0,sizeof(I2CRXBuffer));
  memset(CRC_DATA,0,sizeof(CRC_DATA));
  memset(DATA_CACHE1,0,RX1BUFFERSIZE);
  memset(DATA_CACHE2,0,RX2BUFFERSIZE);
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  STM32_Init();
  SPEC_Init();


  // __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,2500);
  // HAL_Delay(1500);

  // HAL_TIM_Base_Start_IT(&htim3);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    //----- Working LED
    HAL_GPIO_TogglePin(GPIOB,LED_Pin);
    HAL_Delay(50);


    /*--------------------------------------------------Main Part-------------------------------------------------------*/

    //----- command one: Global reset
    if (memcmp(DATA_CACHE1,USART1_Tsimen_Reset,sizeof(USART1_Tsimen_Reset)) == RESET)
    {
      CRC16 = ModBus_CRC16(DATA_CACHE1,6);
      if (DATA_CACHE1[7]==(uint8_t)CRC16&0xFF && DATA_CACHE1[6]==(uint8_t)(CRC16>>8)&0xFF)
      {
        CRC16 = 0;
        // __set_PRIMASK(1);
        // HAL_NVIC_SystemReset();
        PWM_PulseWidth(PWM_DARK);
        HAL_UART_Transmit(&huart2,USART2_Spec_Reset,sizeof(USART2_Spec_Reset),0xFFFF);
        HAL_Delay(1500);
        HAL_UART_Transmit(&huart2,USART2_Spec_Int_Default,sizeof(USART2_Spec_Int_Default),0xFFFF);
        HAL_Delay(30);
        HAL_UART_Transmit(&huart2,USART2_Spec_Ave_Default,sizeof(USART2_Spec_Ave_Default),0xFFFF);
        HAL_Delay(30);
        // Config_WaitTime();
        WaitandClear();
      }

      else
      {
        HAL_UART_Transmit(&huart1,USART1_Tsimen_CRCERROR,sizeof(USART1_Tsimen_CRCERROR),0xFFFF);
        memset(DATA_CACHE1,0,RX1BUFFERSIZE);
      }
    }

    //----- command two: Read the sensor Version
    else if (memcmp(DATA_CACHE1,USART1_Tsimen_Ver,6) == RESET)
    {
      CRC16 = ModBus_CRC16(DATA_CACHE1,6);
      if (DATA_CACHE1[7]==(uint8_t)CRC16&0xFF && DATA_CACHE1[6]==(uint8_t)(CRC16>>8)&0xFF)
      {
        CRC16 = 0; //Reset the CRC16 virable
        HAL_UART_Transmit(&huart1,USART1_Tsimen_Version,sizeof(USART1_Tsimen_Version),0xFFFF);
        memset(DATA_CACHE1,0,RX1BUFFERSIZE);
      }
      else 
      {
        HAL_UART_Transmit(&huart1,USART1_Tsimen_CRCERROR,sizeof(USART1_Tsimen_CRCERROR),0xFFFF);
        memset(DATA_CACHE1,0,RX1BUFFERSIZE);
      }
    }

    //----- command three: Set the spectrometer integration time
    else if (memcmp(DATA_CACHE1,USART1_Tsimen_Int,2) == RESET)
    {
      CRC16 = ModBus_CRC16(DATA_CACHE1,6);
      if (DATA_CACHE1[7]==(uint8_t)CRC16&0xFF && DATA_CACHE1[6]==(uint8_t)(CRC16>>8)&0xFF)
      {
        CRC16 = 0;
        memv(USART2_Spec_Int,DATA_CACHE1,2,3,4);
        CRC16 = ModBus_CRC16(USART2_Spec_Int,5);
        USART2_Spec_Int[6] = CRC16&0xFF;
        USART2_Spec_Int[5] = (CRC16>>8)&0xFF;
        CRC16 = 0;
        HAL_UART_Transmit(&huart2,USART2_Spec_Int,sizeof(USART2_Spec_Int),0xFFFF);
        HAL_Delay(30);
        WaitandClear();
        // Config_WaitTime();
      }

      else
      {
        HAL_UART_Transmit(&huart1,USART1_Tsimen_CRCERROR,sizeof(USART1_Tsimen_CRCERROR),0xFFFF);
        memset(DATA_CACHE1,0,RX1BUFFERSIZE);
      }
    }

    //----- command four: Check the spectrometer integration time
    else if (memcmp(DATA_CACHE1,USART1_Tsimen_CheckInt,sizeof(USART1_Tsimen_CheckInt)) == RESET)
    {
      CRC16 = ModBus_CRC16(DATA_CACHE1,6);
      if (DATA_CACHE1[7]==(uint8_t)CRC16&0xFF && DATA_CACHE1[6]==(uint8_t)(CRC16>>8)&0xFF)
      {
        CRC16 = 0;
        HAL_UART_Transmit(&huart2,USART2_Spec_CheckInt,sizeof(USART2_Spec_CheckInt),0xFFFF);
        HAL_Delay(30);
        memv(CHECKDATA,DATA_CACHE2,1,2,4);
        HAL_UART_Transmit(&huart1,CHECKDATA,4,0xFFFF);
        USART_RX2_LENDEMO = 0;
        memset(DATA_CACHE1,0,RX1BUFFERSIZE);
        memset(DATA_CACHE2,0,RX2BUFFERSIZE);
      }

      else
      {
        HAL_UART_Transmit(&huart1,USART1_Tsimen_CRCERROR,sizeof(USART1_Tsimen_CRCERROR),0xFFFF);
        memset(DATA_CACHE1,0,RX1BUFFERSIZE);
      }
    }

    //----- command five: Set the spectrometer average Times 
    else if (memcmp(DATA_CACHE1,USART1_Tsimen_Ave,2) == RESET)
    {
      CRC16 = ModBus_CRC16(DATA_CACHE1,6);
      if (DATA_CACHE1[7]==(uint8_t)CRC16&0xFF && DATA_CACHE1[6]==(uint8_t)(CRC16>>8)&0xFF)
      {
        CRC16 = 0;
        memv(USART2_Spec_Ave,DATA_CACHE1,2,3,2);
        CRC16 = ModBus_CRC16(USART2_Spec_Ave,3);
        USART2_Spec_Ave[4] = CRC16&0xFF;
        USART2_Spec_Ave[3] = (CRC16>>8)&0xFF;
        CRC16 = 0;
        HAL_UART_Transmit(&huart2,USART2_Spec_Ave,sizeof(USART2_Spec_Ave),0xFFFF);
        HAL_Delay(30);
        WaitandClear();
        // Config_WaitTime();
      }

      else
      {
        HAL_UART_Transmit(&huart1,USART1_Tsimen_CRCERROR,sizeof(USART1_Tsimen_CRCERROR),0xFFFF);
        memset(DATA_CACHE1,0,RX1BUFFERSIZE);
      }
    }

    //----- command six: Check the current average times of the spectrometer
    else if (memcmp(DATA_CACHE1,USART1_Tsimen_CheckAve,sizeof(USART2_Spec_CheckAve)) == RESET)
    {
      CRC16 = ModBus_CRC16(DATA_CACHE1,6);
      if (DATA_CACHE1[7]==(uint8_t)CRC16&0xFF && DATA_CACHE1[6]==(uint8_t)(CRC16>>8)&0xFF)
      {
        CRC16 = 0;
        HAL_UART_Transmit(&huart2,USART2_Spec_CheckAve,sizeof(USART2_Spec_CheckAve),0xFFFF);
        HAL_Delay(30);
        memv(CHECKDATA,DATA_CACHE2,1,2,2);
        HAL_UART_Transmit(&huart1,CHECKDATA,2,0xFFFF);
        USART_RX2_LENDEMO = 0;
        memset(DATA_CACHE1,0,RX1BUFFERSIZE);
        memset(DATA_CACHE2,0,RX2BUFFERSIZE);
      }

      else
      {
        HAL_UART_Transmit(&huart1,USART1_Tsimen_CRCERROR,sizeof(USART1_Tsimen_CRCERROR),0xFFFF);
        memset(DATA_CACHE1,0,RX1BUFFERSIZE);
      }
    }

    //----- command seven: Read the spectrometer data under dark current conditions 
    else if (memcmp(DATA_CACHE1,USART1_Tsimen_DarSignal,sizeof(USART1_Tsimen_DarSignal)) == RESET)
    {
      CRC16 = ModBus_CRC16(DATA_CACHE1,6);
      if (DATA_CACHE1[7]==(uint8_t)CRC16&0xFF && DATA_CACHE1[6]==(uint8_t)(CRC16>>8)&0xFF)
      {
        CRC16 = 0;
        HAL_UART_DMAStop(&huart1);
        __HAL_UART_DISABLE_IT(&huart1,UART_IT_IDLE);
        GetSpecData(USART2_Spec_XenonOff,PWM_DARK);
        __HAL_UART_ENABLE_IT(&huart1,UART_IT_IDLE);
        HAL_UART_Receive_DMA(&huart1,USART_RX1_BUFFER,RX1BUFFERSIZE);
      }
      
      else 
      {
        HAL_UART_Transmit(&huart1,USART1_Tsimen_CRCERROR,sizeof(USART1_Tsimen_CRCERROR),0xFFFF);
        memset(DATA_CACHE1,0,RX1BUFFERSIZE);
      }
    }
   
    //----- command eight: Read the spectrometer data under reference signal conditions
    else if (memcmp(DATA_CACHE1,USART1_Tsimen_RefSignal,sizeof(USART1_Tsimen_RefSignal)) == RESET)
    {
      CRC16 = ModBus_CRC16(DATA_CACHE1,6);
      if (DATA_CACHE1[7]==(uint8_t)CRC16&0xFF && DATA_CACHE1[6]==(uint8_t)(CRC16>>8)&0xFF)
      {
        CRC16 = 0;
        HAL_UART_DMAStop(&huart1);
        __HAL_UART_DISABLE_IT(&huart1,UART_IT_IDLE);
        GetSpecData(USART2_Spec_XenonOnOne,PWM_REFERENCE);
        __HAL_UART_ENABLE_IT(&huart1,UART_IT_IDLE);
        HAL_UART_Receive_DMA(&huart1,USART_RX1_BUFFER,RX1BUFFERSIZE);
      }
      
      else 
      {
        HAL_UART_Transmit(&huart1,USART1_Tsimen_CRCERROR,sizeof(USART1_Tsimen_CRCERROR),0xFFFF);
        memset(DATA_CACHE1,0,RX1BUFFERSIZE);
      }
    }

    //----- command nine: Read the spectrometer data under sample signal conditions
    else if (memcmp(DATA_CACHE1,USART1_Tsimen_SamSignal,sizeof(USART1_Tsimen_SamSignal)) == RESET)
    {
      CRC16 = ModBus_CRC16(DATA_CACHE1,6);
      if (DATA_CACHE1[7]==(uint8_t)CRC16&0xFF && DATA_CACHE1[6]==(uint8_t)(CRC16>>8)&0xFF)
      {
        CRC16 = 0;
        HAL_UART_DMAStop(&huart1);
        __HAL_UART_DISABLE_IT(&huart1,UART_IT_IDLE);
        GetSpecData(USART2_Spec_XenonOnOne,PWM_SAMPLE);
        __HAL_UART_ENABLE_IT(&huart1,UART_IT_IDLE);
        HAL_UART_Receive_DMA(&huart1,USART_RX1_BUFFER,RX1BUFFERSIZE);
      }
      
      else 
      {
        HAL_UART_Transmit(&huart1,USART1_Tsimen_CRCERROR,sizeof(USART1_Tsimen_CRCERROR),0xFFFF);
        memset(DATA_CACHE1,0,RX1BUFFERSIZE);
      }
    }

    //----- command ten: One-click acquisition of dark, parametric and sample spectrometer data
    else if (memcmp(DATA_CACHE1,USART1_Tsimen_FullData,sizeof(USART1_Tsimen_FullData)) == RESET)
    {
      CRC16 = ModBus_CRC16(DATA_CACHE1,6);
      if (DATA_CACHE1[7]==(uint8_t)CRC16&0xFF && DATA_CACHE1[6]==(uint8_t)(CRC16>>8)&0xFF)
      {
        CRC16 = 0;
        HAL_UART_DMAStop(&huart1);
        __HAL_UART_DISABLE_IT(&huart1,UART_IT_IDLE);
        GetSpecData(USART2_Spec_XenonOff,PWM_DARK);
        HAL_Delay(50);
        GetSpecData(USART2_Spec_XenonOnOne,PWM_REFERENCE);
        HAL_Delay(50);
        GetSpecData(USART2_Spec_XenonOnOne,PWM_SAMPLE);
        __HAL_UART_ENABLE_IT(&huart1,UART_IT_IDLE);
        HAL_UART_Receive_DMA(&huart1,USART_RX1_BUFFER,RX1BUFFERSIZE);
      }

      else
      {
        HAL_UART_Transmit(&huart1,USART1_Tsimen_CRCERROR,sizeof(USART1_Tsimen_CRCERROR),0xFFFF);
        memset(DATA_CACHE1,0,RX1BUFFERSIZE);
      }
    }

    //----- command eleven: Read temperature and humidity data
    else if (memcmp(DATA_CACHE1,USART1_Tsimen_TempData,sizeof(USART1_Tsimen_TempData)) == RESET)
    {
      CRC16 = ModBus_CRC16(DATA_CACHE1,6);
      if (DATA_CACHE1[7]==(uint8_t)CRC16&0xFF && DATA_CACHE1[6]==(uint8_t)(CRC16>>8)&0xFF)
      {
        CRC16 = 0;
        HAL_UART_DMAStop(&huart1);
        __HAL_UART_DISABLE_IT(&huart1,UART_IT_IDLE);
        HAL_UART_DMAStop(&huart2);
        __HAL_UART_DISABLE_IT(&huart2,UART_IT_IDLE);
        Measure_TR();
        InsideTemperature();
        __HAL_UART_ENABLE_IT(&huart1,UART_IT_IDLE);
        HAL_UART_Receive_DMA(&huart1,USART_RX1_BUFFER,RX1BUFFERSIZE);
        __HAL_UART_ENABLE_IT(&huart2,UART_IT_IDLE);
        HAL_UART_Receive_DMA(&huart2,USART_RX2_BUFFER,RX2BUFFERSIZE);
        memset(DATA_CACHE1,0,RX1BUFFERSIZE);
      }

      else
      {
        HAL_UART_Transmit(&huart1,USART1_Tsimen_CRCERROR,sizeof(USART1_Tsimen_CRCERROR),0xFFFF);
        memset(DATA_CACHE1,0,RX1BUFFERSIZE);
      }
    }
  __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,PWM_DARK);
  HAL_Delay(500);
  __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,PWM_REFERENCE);
  HAL_Delay(500);
  __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,PWM_SAMPLE);
  HAL_Delay(500);
    //----- command twelve: Read the wavelenth
    // else if (memcmp(DATA_CACHE1,USART1_Tsimen_WL,sizeof(USART1_Tsimen_WL)) == RESET)
    // {
    //   CRC16 = ModBus_CRC16(DATA_CACHE1,6);
    //   if (DATA_CACHE1[7]==(uint8_t)CRC16&0xFF && DATA_CACHE1[6]==(uint8_t)(CRC16>>8)&0xFF)
    //   {
    //     CRC16 = 0;

    //     //Counter the wavelenth and Send them to the master
    //     for (Dou_i=1;Dou_i<1025;Dou_i++)
    //     {
    //       Wavelength = 
    //       Wave_Coefficient[0]*(long long int)Dou_i*(long long int)Dou_i*(long long int)Dou_i*(long long int)Dou_i*(long long int)Dou_i+
    //       Wave_Coefficient[1]*(long long int)Dou_i*(long long int)Dou_i*(long long int)Dou_i*(long long int)Dou_i+
    //       Wave_Coefficient[2]*(long long int)Dou_i*(long long int)Dou_i*(long long int)Dou_i+
    //       Wave_Coefficient[3]*(long long int)Dou_i*(long long int)Dou_i+
    //       Wave_Coefficient[4]*(long long int)Dou_i+
    //       Wave_Coefficient[5];
    //       // // DoubleBuf[7] = *((uint8_t*)&Wavelength);
    //       // // DoubleBuf[6] = *((uint8_t*)&Wavelength+1);
    //       // // DoubleBuf[5] = *((uint8_t*)&Wavelength+2);
    //       // // DoubleBuf[4] = *((uint8_t*)&Wavelength+3);
    //       // // DoubleBuf[3] = *((uint8_t*)&Wavelength+4);
    //       // // DoubleBuf[2] = *((uint8_t*)&Wavelength+5);
    //       // // DoubleBuf[1] = *((uint8_t*)&Wavelength+6);
    //       // // DoubleBuf[0] = *((uint8_t*)&Wavelength+7);
    //       // // HAL_UART_Transmit(&huart1,DoubleBuf,sizeof(DoubleBuf),0xFFFF);
    //       printf("%lf\r\n",Wavelength);
    //     }

    //     memset(DATA_CACHE1,0,RX1BUFFERSIZE);
    //   }

    //   else
    //   {
    //     HAL_UART_Transmit(&huart1,USART1_Tsimen_CRCERROR,sizeof(USART1_Tsimen_CRCERROR),0xFFFF);
    //     memset(DATA_CACHE1,0,RX1BUFFERSIZE);
    //   }
    // }

    //----- command thirteen: Set the wavelength cofficient
    // else if (memcmp(DATA_CACHE1,USART1_Tsimen_SetWL,2) == RESET)
    // {
    //   CRC16 = ModBus_CRC16(DATA_CACHE1,50);
    //   if (DATA_CACHE1[51]==(uint8_t)CRC16&0xFF && DATA_CACHE1[50]==(uint8_t)(CRC16>>8)&0xFF)
    //   {
    //     CRC16 = 0;
    //     Receive_WavelengthCoefficient(DATA_CACHE1,50);
    //     HAL_UART_Transmit(&huart1,USART1_Tsimen_OK,sizeof(USART1_Tsimen_OK),0xFFFF);
    //     memset(DATA_CACHE1,0,RX1BUFFERSIZE);
    //   }

    //   else
    //   {
    //     HAL_UART_Transmit(&huart1,USART1_Tsimen_CRCERROR,sizeof(USART1_Tsimen_CRCERROR),0xFFFF);
    //     memset(DATA_CACHE1,0,RX1BUFFERSIZE);
    //   }
    // }

    //----- command fouteen: Read the wavelength cofficient
    // else if (memcmp(DATA_CACHE1,USART1_Tsimen_WC,sizeof(USART1_Tsimen_WC)) == RESET)
    // {
    //   CRC16 = ModBus_CRC16(DATA_CACHE1,6);
    //   if (DATA_CACHE1[7]==(uint8_t)CRC16&0xFF && DATA_CACHE1[6]==(uint8_t)(CRC16>>8)&0xFF)
    //   {
    //     CRC16 = 0;
    //     Send_WavelengthCoefficient(Wave_Coefficient,6);
    //     memset(DATA_CACHE1,0,RX1BUFFERSIZE);
    //   }

    //   else
    //   {
    //     HAL_UART_Transmit(&huart1,USART1_Tsimen_CRCERROR,sizeof(USART1_Tsimen_CRCERROR),0xFFFF);
    //     memset(DATA_CACHE1,0,RX1BUFFERSIZE);
    //   }
    // }

    // ----- command fifteen: Set the energy cofficient
    // else if (memcmp(DATA_CACHE1,USART1_Tsimen_SetWL,2) == RESET)
    // {
    //   CRC16 = ModBus_CRC16(DATA_CACHE1,26);
    //   if (DATA_CACHE1[7]==(uint8_t)CRC16&0xFF && DATA_CACHE1[6]==(uint8_t)(CRC16>>8)&0xFF)
    //   {
    //     CRC16 = 0;
        
    //     memset(DATA_CACHE1,0,RX1BUFFERSIZE);
    //   }

    //   else
    //   {
    //     HAL_UART_Transmit(&huart1,USART1_Tsimen_CRCERROR,sizeof(USART1_Tsimen_CRCERROR),0xFFFF);
    //     memset(DATA_CACHE1,0,RX1BUFFERSIZE);
    //   }
    // }



    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/**
 * @brief CRC16 check code. The function is used for data verification when communicating with PC.
 *  
 * @param pdata 
 * @param len 
 * @return uint16_t 
 */
uint16_t ModBus_CRC16( uint8_t *pdata, int len)     //polynomial: 8005
{
  crc = 0xFFFF;
  for ( CRC_j=0; CRC_j<len;CRC_j++)
  {
    crc=crc^pdata[CRC_j];
    for ( CRC_i=0; CRC_i<8; CRC_i++)
    {
      if( ( crc&0x0001) >0)
      {
        crc=crc>>1;
        crc=crc^ 0xA001;                            //We take 8005 in reverse.
      }
    else
    crc=crc>>1;
    }
  }
return crc;
}

/**
 * @brief Cut a specific length from the A array to the B array.
 * 
 * @param pdata target array
 * @param ppdata truncated array
 * @param st1 start bit of the destination array
 * @param st2 start bit of truncated array
 * @param length The length of the truncated array
 */
void memv(uint8_t *pdata,uint8_t *ppdata,int st1,int st2,int length)
{
  for (memv_i=0;memv_i<length;memv_i++)
  {
    pdata[memv_i+st1-1] = ppdata[memv_i+st2-1];
  }
}

/**
 * @brief Wait and clear the cache data
 * 
 */
void WaitandClear(void)
{
        while (WHILEA < 10)
        {
          if (memcmp(DATA_CACHE2,USART2_Spec_OK,3) ==  RESET)
          {
            HAL_UART_Transmit(&huart1,USART1_Tsimen_OK,sizeof(USART1_Tsimen_OK),0xFFFF);
            break;
          }
          else if (memcmp(DATA_CACHE2,USART2_Spec_ERROR,3) == RESET)
          {
            HAL_UART_Transmit(&huart1,USART1_Tsimen_ERROR,sizeof(USART1_Tsimen_ERROR),0xFFFF);
            break;
          }
          WHILEA++;
        }

        if (WHILEA == 10)
        {
          HAL_UART_Transmit(&huart1,USART1_Tsimen_SpecERROR,sizeof(USART1_Tsimen_SpecERROR),0xFFFF);
        }
        
        WHILEA = 0;
        USART_RX2_LENDEMO = 0;
        memset(DATA_CACHE1,0,RX1BUFFERSIZE);
        memset(DATA_CACHE2,0,RX2BUFFERSIZE);
}

/**
 * @brief Get the Spec Data object
 * 
 * @param specdata control the xenon's status
 * @param a set the pulse width of the PWM_timer2
 */
void GetSpecData(uint8_t *specdata,int a)
{
  HAL_UART_Transmit(&huart2,specdata,sizeof(specdata),0xFFFF);
  HAL_Delay(30);

  if (memcmp(DATA_CACHE2,USART2_Spec_OK,sizeof(USART2_Spec_OK)) == RESET)
  {
    PWM_PulseWidth(a);

    memset(DATA_CACHE2,0,RX2BUFFERSIZE);
    // HAL_UART_DMAStop(&huart2);
    // __HAL_UART_DISABLE_IT(&huart2,UART_IT_IDLE);
    HAL_Delay(500);

    HAL_UART_Transmit(&huart2,USART2_Spec_Data,sizeof(USART2_Spec_Data),0xFFFF);
    // HAL_UART_DMAStop(&huart1);
    while (memcmp(DATA_CACHE2,USART2_Spec_FormData,9) != RESET)
    {
      HAL_Delay(20);
      if (memcmp(DATA_CACHE1,USART1_Tsimen_Jump,sizeof(USART1_Tsimen_Jump)) == RESET)
      {
        CRC16 = ModBus_CRC16(DATA_CACHE1,6);
        if (DATA_CACHE1[7]==(uint8_t)CRC16&0xFF && DATA_CACHE1[6]==(uint8_t)(CRC16>>8)&0xFF)
        {
          CRC16 = 0;
          HAL_UART_Transmit(&huart1,USART1_Tsimen_OK,sizeof(USART1_Tsimen_OK),0xFFFF);
          break;
        }
      }
    }
    HAL_Delay(300);
    HAL_UART_Transmit(&huart1,DATA_CACHE2,2063,0xFFFF);

    //----- clear variables
    USART_RX2_LENDEMO = 0;
    memset(DATA_CACHE1,0,RX1BUFFERSIZE);
    memset(DATA_CACHE2,0,RX2BUFFERSIZE);
    // __HAL_UART_ENABLE_IT(&huart2,UART_IT_IDLE);
    // __HAL_UART_ENABLE_IT(&huart2,UART_IT_IDLE);
    // HAL_UART_Receive_DMA(&huart1,USART_RX1_BUFFER,RX1BUFFERSIZE);
    HAL_Delay(20);
    // HAL_UART_Receive_DMA(&huart2,DATA_CACHE2,RX2BUFFERSIZE);
  }
  
  else if (memcmp(DATA_CACHE2,USART2_Spec_ERROR,sizeof(USART2_Spec_ERROR)) == RESET)
  {
    HAL_UART_Transmit(&huart1,USART1_Tsimen_ERROR,sizeof(USART1_Tsimen_ERROR),0xFFFF);
    memset(DATA_CACHE1,0,RX1BUFFERSIZE);
    memset(DATA_CACHE2,0,RX2BUFFERSIZE);
  }
  else 
  {
    HAL_UART_Transmit(&huart1,USART1_Tsimen_SpecERROR,sizeof(USART1_Tsimen_SpecERROR),0xFFFF);
    memset(DATA_CACHE1,0,RX1BUFFERSIZE);
  }
}

/**
 * @brief Init the STM32 System
 * 
 */
void STM32_Init(void)
{
  /*--------------------------------------------------Init the usart DMA-------------------------------------------------------*/
  HAL_UART_Receive_DMA(&huart1,USART_RX1_BUFFER,RX1BUFFERSIZE);
  HAL_UART_Receive_DMA(&huart2,USART_RX2_BUFFER,RX2BUFFERSIZE);

  /*--------------------------------------------------Init the PWM-------------------------------------------------------*/
  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
  __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,PWM_DARK);
}

/**
 * @brief Init the Spectrometer
 * 
 */
void SPEC_Init(void)
{
  HAL_UART_Transmit(&huart2,USART2_Spec_Reset,sizeof(USART2_Spec_Reset),0xFFFF);
  HAL_Delay(1500);
  Judge_SpecStatus();
  HAL_UART_Transmit(&huart2,USART2_Spec_Int_Default,sizeof(USART2_Spec_Int_Default),0xFFFF);
  HAL_Delay(30);
  HAL_UART_Transmit(&huart2,USART2_Spec_Ave_Default,sizeof(USART2_Spec_Ave_Default),0xFFFF);
  HAL_Delay(30);
  // Config_WaitTime();
}

/**
 * @brief Config the wait time
 * 
 */
void Config_WaitTime(void)
{
  WAIT_TIME = 0;
  WAIT_TIME = (USART2_Spec_Int[1]<<24) | (USART2_Spec_Int[2]<<16) | (USART2_Spec_Int[3]<<8) | USART2_Spec_Int[4] ;
  WAIT_TIME = WAIT_TIME * (uint32_t)((USART2_Spec_Ave[1]<<8) | USART2_Spec_Ave[2]) ;
  WAIT_TIME = WAIT_TIME / 1000 ;
}

/**
 * @brief To judge the spec status, if the spec is incorrect, send message to the master PC
 * 
 */
void Judge_SpecStatus(void)
{
  A = 20 ; 
  while (A!=1)
  {
    if (memcmp(DATA_CACHE2,USART2_Spec_OK,sizeof(USART2_Spec_OK)) == RESET)
    {
      HAL_UART_Transmit(&huart1,USART1_Tsimen_SpecOK,sizeof(USART1_Tsimen_SpecOK),0xFFFF);
      USART_RX2_LENDEMO = 0;
      memset(DATA_CACHE2,0,10);
      break;
    }
    A-- ;
  }

  if (A == 1)
  {
    HAL_UART_Transmit(&huart1,USART1_Tsimen_SpecERROR,sizeof(USART1_Tsimen_SpecERROR),0xFFFF);
  }
  A = 0 ; 
}

/**
 * @brief Send the wavelength coefficient to the master
 * 
 * @param pr the coefficient array
 * @param a lenth
 */
void Send_WavelengthCoefficient(double *pr,uint8_t a)
{
  for (Dou_i=0;Dou_i<a;Dou_i++)
  {
  // DoubleBuf[7] = *((uint8_t*)&pr[Dou_i]);
  // DoubleBuf[6] = *((uint8_t*)&pr[Dou_i]+1);
  // DoubleBuf[5] = *((uint8_t*)&pr[Dou_i]+2);
  // DoubleBuf[4] = *((uint8_t*)&pr[Dou_i]+3);
  // DoubleBuf[3] = *((uint8_t*)&pr[Dou_i]+4);
  // DoubleBuf[2] = *((uint8_t*)&pr[Dou_i]+5);
  // DoubleBuf[1] = *((uint8_t*)&pr[Dou_i]+6);
  // DoubleBuf[0] = *((uint8_t*)&pr[Dou_i]+7);
  // HAL_UART_Transmit(&huart1,DoubleBuf,sizeof(DoubleBuf),0xFFFF);
  printf("%.15lf\r\n",Wave_Coefficient[Dou_i]);
  }
}

/**
 * @brief Receive the Coefficient
 * 
 * @param pr 
 */
void Receive_WavelengthCoefficient(uint8_t *pr,int a)
{
  for (Dou_i=0;Dou_i<a;Dou_i+=8)
  {
  DoubleBuf[0] = pr[9+Dou_i];
  DoubleBuf[1] = pr[8+Dou_i];
  DoubleBuf[2] = pr[7+Dou_i];
  DoubleBuf[3] = pr[6+Dou_i];
  DoubleBuf[4] = pr[5+Dou_i];
  DoubleBuf[5] = pr[4+Dou_i];
  DoubleBuf[6] = pr[3+Dou_i];
  DoubleBuf[7] = pr[2+Dou_i];
  Wave_Coefficient[Dou_i/8] = *((double*)&DoubleBuf[0]);
  }
}



/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

