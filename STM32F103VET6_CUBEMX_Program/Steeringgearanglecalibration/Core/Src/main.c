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
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
//#include <math.h>
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

int memv_i = 0;
int PWM_DARK = 970;
int PWM_REFERENCE = 1200;
int PWM_SAMPLE = 770;
int PWM_Any = 0;

uint8_t Dark_Angle[] = "dark";
uint8_t Ref_Angle[] = "ref";
uint8_t Sam_Angle[] = "sam";
uint8_t Any_Angle[] = "any";

uint8_t PWM_OK[] = "PWM IS OK";
uint8_t PWMBUFFER[4] ;

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
  memset(DATACACHE1,0,USART1BUFFERSIZE);
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  STM_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    HAL_GPIO_TogglePin(GPIOB,LED_Pin);
    HAL_Delay(40);

    if ( memcmp(DATACACHE1,Dark_Angle,3) == RESET )
    {
      PWM_PulseWidth(PWM_DARK);
      HAL_UART_Transmit(&huart1,PWM_OK,sizeof(PWM_OK),0xFFFF);

      memset(DATACACHE1,0,USART1BUFFERSIZE);
    }

    else if ( memcmp(DATACACHE1,Ref_Angle,3) == RESET )
    {
      PWM_PulseWidth(PWM_REFERENCE);
      HAL_UART_Transmit(&huart1,PWM_OK,sizeof(PWM_OK),0xFFFF);

      memset(DATACACHE1,0,USART1BUFFERSIZE);
    }

    else if ( memcmp(DATACACHE1,Sam_Angle,3) == RESET )
    {
      PWM_PulseWidth(PWM_SAMPLE);
      HAL_UART_Transmit(&huart1,PWM_OK,sizeof(PWM_OK),0xFFFF);

      memset(DATACACHE1,0,USART1BUFFERSIZE);
    }

    else if ( memcmp(DATACACHE1,Any_Angle,3) == RESET )
    {
      printf("Prepare to change the angle ...");
      memv(PWMBUFFER,DATACACHE1,1,4,4);
      PWM_Any = atoi(PWMBUFFER);
      HAL_Delay(500);
      printf("angle is : %d\r\n",PWM_Any);

      PWM_PulseWidth(PWM_Any);
      HAL_UART_Transmit(&huart1,PWM_OK,sizeof(PWM_OK),0xFFFF);

      memset(DATACACHE1,0,USART1BUFFERSIZE);
    }





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
}

/* USER CODE BEGIN 4 */

/**
 * @brief Init the STM32
 * 
 */
void STM_Init(void)
{
  /*--------------- Init USART DMA ---------------*/
  HAL_UART_Receive_DMA(&huart1,USART1BUFFER,USART1BUFFERSIZE);

  /*--------------- Init the PWM ---------------*/
  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
  PWM_PulseWidth(PWM_DARK);


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

