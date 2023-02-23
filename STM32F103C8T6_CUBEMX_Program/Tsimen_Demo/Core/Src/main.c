/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *作者：王福涛   2022-01-01
  ******************************************************************************
  */ 
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include "stm32f1xx_hal.h"
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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
char ID = 0x01;
char MAC[] =  {">MAC:[20-22-01-11-00-01]"};

char LS[8];

uint16_t CRC16; uint8_t CRCH; uint8_t CRCL;

uint8_t Uart1_RxBuff[8];		
uint8_t Uart1_Rx_Length = 8;		
uint8_t Uart1_Tx_Length ;		


uint8_t Uart1_Rx_RST[2] = {0x01,0x06};
uint8_t Uart1_Rx_V[2] =   {0x01,0x03};
uint8_t Uart1_Rx_ITC[2] = {0x01,0x64};
uint8_t Uart1_Rx_ITI[4] = {0x01,0x65,0x00,0x00};

uint8_t Uart1_Rx_X[4] =   {0x01,0x67};
uint8_t Uart1_Rx_Xe_H[2] ={0x01,0x68};
uint8_t Uart1_Rx_Xe_L[2] ={0x01,0x69};
uint8_t Uart1_Rx_Xe_I[2] ={0x01,0x6A};
uint8_t Uart1_Rx_PXC[2]=  {0x01,0x6B};
uint8_t Uart1_Rx_PXI[2]=  {0x01,0x6C};
uint8_t Uart1_Rx_WLI[2] = {0x01,0x6D};
uint8_t Uart1_Rx_BLS[2]=  {0x01,0x6E};
uint8_t Uart1_Rx_BLI[2]=  {0x01,0x6F};
uint8_t Uart1_Rx_AVC[2] = {0x01,0x70};
uint8_t Uart1_Rx_AVI[2] = {0x01,0x71};
uint8_t Uart1_Rx_S1[2] =  {0x01,0x72};

uint8_t Uart1_Rx_XeG[8] = {0x01,0x80,0x00,0x00,0x00,0x00,0xFF,0xFF};
uint8_t Uart1_Rx_XeK[4] = {0x01,0x81,0x01};
uint8_t Uart1_Rx_XeI[2] = {0x01,0x82};

uint8_t Uart1_Rx_TEM[2] = {0x01,0x85};

uint8_t Uart1_Rx_S0[2] =  {0x01,0x86};
uint8_t Uart1_Rx_SL[2] =  {0x01,0x87};
uint8_t Uart1_Rx_SR[2] =  {0x01,0x88};




uint8_t Rx1Flag = 0;
uint8_t Rx2Flag = 0;
uint8_t Rx3Flag = 0;
char  uart1_mode = 0;
char  uart2_mode = 0;
char  uart3_mode = 0;

char  Xe_status = 0;
char  Xe_status_C = 0x00;

char  Xe_0_S = 0;
char  Xe_1_S = 1;
char  Xe_2_S = 1;

uint8_t aRxBuffer;			//??????
uint8_t Uart2_RxBuff3[3];
uint8_t Uart2_RxBuff3A[3];
uint8_t Uart2_RxBuff3B[3];
uint8_t Uart2_RxBuff3_0[3];
uint8_t Uart2_RxBuff3_1[3];
uint8_t Uart2_RxBuff3_2[3];
uint8_t Uart2_RxBuff3_3[3];
uint8_t Uart2_RxBuff3_4[3];
uint8_t Uart2_RxBuff3_5[3];
uint8_t Uart2_RxBuff4[4];
uint8_t Uart2_RxBuff4A[4];
uint8_t Uart2_RxBuff4_0[4];
uint8_t Uart2_RxBuff4_1[4];
uint8_t Uart2_RxBuff4_2[4];
uint8_t Uart2_RxBuff5[5];
uint8_t Uart2_RxBuff7[7];
uint8_t Uart2_RxBuff7_1[7];
uint8_t Uart2_RxBuff8[8];
uint8_t Uart2_RxBuff11[11];
uint8_t Uart2_RxBuff23[23];
uint8_t Uart2_RxBuff243[243];
uint8_t Uart2_RxBuff2063[2063];		//????
uint8_t Uart2_RxBuff4099[4099];	
uint16_t Uart2_Rx_Length = 2063;		//??????
uint8_t Uart2_Tx_Length = 3;		

//uint8_t Uart3_RxBuff22[22] = {};


char  T_uart2_RST[3] = {0x52,0xBD,0x3E};
char  T_uart2_V[3] =   {0x56,0x7E,0x3F};
char  T_uart2_ITC[7] = {0x69,0x00,0x00,};  
char  T_uart2_ITI[4] = {0x3F,0x69,0x6E,0xD0};
char  T_uart2_S[3] =   {0x53,0x7D,0xFF};
char  T_uart2_X[3] =   {0x78,0x62,0xBF};
char  T_uart2_Xe_I[4] ={0x3F,0x30,0x54,0x10};
char  T_uart2_Xe[11] = {0x30};
char  T_uart2_PXC[11] ={0x50,0x00,0x03};
char  T_uart2_PXI[4] = {0x3F,0x50,0x7C,0x10};
char  T_uart2_WLI[4] = {0x3F,0x53,0x7D,0x50};
char  T_uart2_BLS[5] = {0x41,};
char  T_uart2_BLI[4] = {0x3F,0x41,0x70,0xD0};

char  T_uart2_XeK[4] = {0x31,0x01,0xE0,0xD5};

char  T_uart2_XeC0[4] ={0x31,0x00,0x20,0x14};//关氙灯
char  T_uart2_XeC1[4] ={0x31,0x01,0xE0,0xD5};//开氙灯
char  T_uart2_XeI[4] = {0x3F,0x31,0x94,0xD1};//查询氙灯

char  T_uart2_AVG[8] = {0x01,0x06,0x00,0x05,0x00,0x1E,0xFF,0xFF};


	


char  R_uart2_RST[3] = {0x06,0x42,0x3F};
char  R_uart2_V[3] =   {0x06,0x50,0x52};
char  R_uart2_V_T[50] ={0x01};
char  R_uart2_ITC[3] = {0x06,0x42,0x3F};
char  R_uart2_ITI[3] = {0x06,0x00,0x00};
char  R_uart2_S[4] =   {0x06,0xAA,0x55,0xBB};
char  R_uart2_X[1] =   {0x06};
char  R_uart2_Xe[3] =  {0x06,0x42,0x3F};
char  R_uart2_Xe_I[2] ={0x06,0x00};
char  R_uart2_PXC[3] = {0x06,0x42,0x3F};
char  R_uart2_PXI[1] = {0x06};
char  R_uart2_WLI[3] = {0x06,0xAA,0x55};
char  R_uart2_BLS[3] = {0x06,0x42,0x3F};
char  R_uart2_BLI[1] = {0x06};

char  R_uart2_XeG[3] = {0x06,0x42,0x3F};
char  R_uart2_XeK[3] = {0x06,0x42,0x3F};

char  R_uart2_XeI[4] = {0x06};

char  R_uart3_TEM[22] = {"Temp"};



char  R_uart2_AVG[8] = {0x01,0x06,0x00,0x05,0x00};
 

void PWM2200()
{
	__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,2200);  
}
void PWM1800()
{
__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,1800);  
} 

uint16_t crc16(uint8_t *addr,uint8_t num);

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

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_Delay(1000); 
  
  HAL_UART_Receive_IT(&huart1,(uint8_t *)Uart1_RxBuff, 8);

  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1); 

  HAL_UART_Receive_IT(&huart2,(uint8_t *)Uart2_RxBuff4, 4);
//   while(1)
//   {
//   HAL_UART_Transmit(&huart2, (uint8_t *)T_uart2_XeI, sizeof(T_uart2_XeI),0xFFFF); 
// 		HAL_Delay(500);
	  
//   }

//  while(Rx2Flag == 0)
//  {;}
//         Rx2Flag = 0;
// 	  if(Uart2_RxBuff4[0] == 0x06)
// 	  {
// 		  if(Uart2_RxBuff4[1] == 0)
// 			  Xe_status = 0;
// 		  else
// 			  Xe_status = 1;		  
// 	  }
	  memset(Uart2_RxBuff4,0,4);
	  
	  
	  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_1);//wft	 
		HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13);//wft		
		HAL_Delay(100);
	  
		if(Rx1Flag  == 1)
		{
			Rx1Flag  = 0;
			CRC16 = crc16((uint8_t *)Uart1_RxBuff, 6);
			CRCL = CRC16 & 0x00FF; 	CRCH = (CRC16 >> 8) & 0x00FF;

			if(ID == Uart1_RxBuff[0])
			{
							
				
				memcpy((char *)LS, (const char *)Uart1_RxBuff,sizeof(Uart1_RxBuff));
								
				Uart1_RxBuff[0] = 0x01; // 脱壳
								
				if(strncmp((char *)Uart1_RxBuff,(char *)Uart1_Rx_RST, 2) == 0)
				{
					uart1_mode = 1; 
					
					HAL_UART_Transmit(&huart1, (uint8_t *)LS, sizeof(LS),0xFFFF);
					
					HAL_UART_Receive_IT(&huart2,(uint8_t *)Uart2_RxBuff3, 3);
					HAL_UART_Transmit(&huart2, (uint8_t *)T_uart2_RST, sizeof(T_uart2_RST),0xFFFF);	//复位光谱仪
								
				}
				else if(strncmp((char *)Uart1_RxBuff,(char *)Uart1_Rx_V, 2) == 0)
				{
					uart1_mode = 2;
					
					HAL_UART_Receive_IT(&huart2,(uint8_t *)Uart2_RxBuff23, 23);
					HAL_UART_Transmit(&huart2, (uint8_t *)T_uart2_V, sizeof(T_uart2_V),0xFFFF);
								
				}
				else if(strncmp((char *)Uart1_RxBuff,(char *)Uart1_Rx_ITC, 2) == 0)//积分时间设置
				{
					uart1_mode = 3;
					
					T_uart2_ITC[3] = Uart1_RxBuff[4]; 
					T_uart2_ITC[4] = Uart1_RxBuff[5];					
					CRC16 = crc16((uint8_t *)T_uart2_ITC, 5); CRCL = CRC16 & 0x00FF; 	CRCH = (CRC16 >> 8) & 0x00FF;					
					T_uart2_ITC[5] = CRCH, T_uart2_ITC[6] = CRCL;					
							  
					HAL_UART_Receive_IT(&huart2,(uint8_t *)Uart2_RxBuff3_0, 3);
					HAL_UART_Transmit(&huart2, (uint8_t *)T_uart2_ITC, sizeof(T_uart2_ITC),0xFFFF);
				
				}
				else if(strncmp((char *)Uart1_RxBuff,(char *)Uart1_Rx_ITI,2) == 0)//积分时间查询
				{									
					uart1_mode = 4;
					
					HAL_UART_Receive_IT(&huart2,(uint8_t *)Uart2_RxBuff7, 7);
					HAL_UART_Transmit(&huart2, (uint8_t *)T_uart2_ITI, sizeof(T_uart2_ITI),0xFFFF);
					
				}								
/*				else if(strstr((char *)Uart1_RxBuff,(char *)T_uart2_AVG) != NULL)
				{
					uart1_mode = 5; 
					
					HAL_UART_Receive_IT(&huart2,(uint8_t *)Uart2_RxBuff8, 8);
					HAL_UART_Transmit(&huart2, (uint8_t *)T_uart2_AVG, sizeof(T_uart2_AVG),0xFFFF);
					
				}	*/											
				else if(strstr((char *)Uart1_RxBuff,(char *)Uart1_Rx_X) != NULL)
				{
					uart1_mode = 11;
					
					HAL_UART_Receive_IT(&huart2,(uint8_t *)Uart2_RxBuff243, 243);
					HAL_UART_Transmit(&huart2, (uint8_t *)T_uart2_X, sizeof(T_uart2_X),0xFFFF);
					
				}
				else if(strncmp((char *)Uart1_RxBuff,(char *)Uart1_Rx_Xe_H, 2) == 0)
				{
					uart1_mode = 12;
					
					T_uart2_Xe[1] = Uart1_RxBuff[2]; T_uart2_Xe[2] = Uart1_RxBuff[3]; T_uart2_Xe[3] = Uart1_RxBuff[4]; 
					T_uart2_Xe[4] = Uart1_RxBuff[5];
					
				}
				else if(strncmp((char *)Uart1_RxBuff,(char *)Uart1_Rx_Xe_L, 2) == 0)
				{
					uart1_mode = 12; 
					
					T_uart2_Xe[5] = Uart1_RxBuff[2]; T_uart2_Xe[6] = Uart1_RxBuff[3]; T_uart2_Xe[7] = Uart1_RxBuff[4]; 
					T_uart2_Xe[8] = Uart1_RxBuff[5]; 
					
					CRC16 = crc16((uint8_t *)T_uart2_Xe, 9);
					CRCL = CRC16 & 0x00FF; 	CRCH = (CRC16 >> 8) & 0x00FF;
					T_uart2_Xe[9] = CRCH; T_uart2_Xe[10] = CRCL;
					
					
					HAL_UART_Receive_IT(&huart2,(uint8_t *)Uart2_RxBuff3_1, 3);
					HAL_UART_Transmit(&huart2, (uint8_t *)T_uart2_Xe, sizeof(T_uart2_Xe),0xFFFF);
					
				}
				else if(strncmp((char *)Uart1_RxBuff,(char *)Uart1_Rx_Xe_I, 2) == 0)
				{
					uart1_mode = 13;  
					
					HAL_UART_Receive_IT(&huart2,(uint8_t *)Uart2_RxBuff11, 11);
					HAL_UART_Transmit(&huart2, (uint8_t *)T_uart2_Xe_I, sizeof(T_uart2_Xe_I),0xFFFF);
					
				}				
				else if(strncmp((char *)Uart1_RxBuff,(char *)Uart1_Rx_PXC, 2) == 0)
				{
					uart1_mode = 14;  
					
					T_uart2_PXC[3] = Uart1_RxBuff[2]; T_uart2_PXC[4] = Uart1_RxBuff[3]; T_uart2_PXC[5] = Uart1_RxBuff[4]; 
					T_uart2_PXC[6] = Uart1_RxBuff[5]; T_uart2_PXC[7] = 0x00; T_uart2_PXC[8] = 0x01;//T_uart2_PXC[9] = 0x47; T_uart2_PXC[10] = 0x34;
					CRC16 = crc16((uint8_t *)T_uart2_PXC, 9); CRCL = CRC16 & 0x00FF; 	CRCH = (CRC16 >> 8) & 0x00FF;
					T_uart2_PXC[9] = CRCH; T_uart2_PXC[10] = CRCL;
					HAL_UART_Receive_IT(&huart2,(uint8_t *)Uart2_RxBuff3_5, 3);
					HAL_UART_Transmit(&huart2, (uint8_t *)T_uart2_PXC, sizeof(T_uart2_PXC),0xFFFF);				
					
				}				
				else if(strncmp((char *)Uart1_RxBuff,(char *)Uart1_Rx_PXI, 2) == 0)
				{
					uart1_mode = 15;  
					
					HAL_UART_Receive_IT(&huart2,(uint8_t *)Uart2_RxBuff7_1, 7);
					HAL_UART_Transmit(&huart2, (uint8_t *)T_uart2_PXI, sizeof(T_uart2_PXI),0xFFFF);					
					
				}			
				else if(strncmp((char *)Uart1_RxBuff,(char *)Uart1_Rx_WLI, 2) == 0)
				{
					uart1_mode = 16; 
					
					HAL_UART_Receive_IT(&huart2,(uint8_t *)Uart2_RxBuff4099, 4099);
					HAL_UART_Transmit(&huart2, (uint8_t *)T_uart2_WLI, sizeof(T_uart2_WLI),0xFFFF);					
					
				}											
				else if(strncmp((char *)Uart1_RxBuff,(char *)Uart1_Rx_BLS, 2) == 0)
				{
					uart1_mode = 17;  
					
					T_uart2_BLS[1] = Uart1_RxBuff[2]; T_uart2_BLS[2] = Uart1_RxBuff[3];
					CRC16 = crc16((uint8_t *)T_uart2_BLS, 3);
					CRCL = CRC16 & 0x00FF; 	CRCH = (CRC16 >> 8) & 0x00FF;
					T_uart2_BLS[3] = CRCH; T_uart2_BLS[4] = CRCL;
					HAL_UART_Receive_IT(&huart2,(uint8_t *)Uart2_RxBuff3_2, 3);
					HAL_UART_Transmit(&huart2, (uint8_t *)T_uart2_BLS, sizeof(T_uart2_BLS),0xFFFF);					
				
				}
				else if(strncmp((char *)Uart1_RxBuff,(char *)Uart1_Rx_BLI, 2) == 0)
				{
					uart1_mode = 18;  
					
					HAL_UART_Receive_IT(&huart2,(uint8_t *)Uart2_RxBuff5, 5);
					HAL_UART_Transmit(&huart2, (uint8_t *)T_uart2_BLI, sizeof(T_uart2_BLI),0xFFFF);					
					
				}
				else if(strncmp((char *)Uart1_RxBuff,(char *)Uart1_Rx_XeG, 2) == 0)//*****************************************
				{
					uart1_mode = 20; 
										
					HAL_UART_Receive_IT(&huart2,(uint8_t *)Uart2_RxBuff3A, 3);
					HAL_UART_Transmit(&huart2, (uint8_t *)T_uart2_XeC0, sizeof(T_uart2_XeC0),0xFFFF);					
					
				}
				else if(strncmp((char *)Uart1_RxBuff,(char *)Uart1_Rx_XeK, 2) == 0)//*****************************************
				{
					uart1_mode = 21; 
			
					HAL_UART_Receive_IT(&huart2,(uint8_t *)Uart2_RxBuff3B, 3);
					HAL_UART_Transmit(&huart2, (uint8_t *)T_uart2_XeC1, sizeof(T_uart2_XeC1),0xFFFF);					
					
				}
				else if(strncmp((char *)Uart1_RxBuff,(char *)Uart1_Rx_XeI, 2) == 0)//*****************************************
				{
					uart1_mode = 22;  
					
					HAL_UART_Receive_IT(&huart2,(uint8_t *)Uart2_RxBuff4A, 4);
					HAL_UART_Transmit(&huart2, (uint8_t *)T_uart2_XeI, sizeof(T_uart2_XeI),0xFFFF);					
					
				}
				else if(strncmp((char *)Uart1_RxBuff,(char *)Uart1_Rx_S0, 2) == 0)
				{
					uart1_mode = 32; 
					Xe_0_S = 1;					
//	CRC16 = crc16((uint8_t *)XeC, 2);
//	CRCL = CRC16 & 0x00FF; 	CRCH = (CRC16 >> 8) & 0x00FF;
					if(Xe_status == 1)						
					{	


						
						HAL_UART_Receive_IT(&huart2,(uint8_t *)Uart2_RxBuff3A, 3);
					    HAL_UART_Transmit(&huart2, (uint8_t *)T_uart2_XeC0, sizeof(T_uart2_XeC0),0xFFFF);
						//HAL_UART_Receive_IT(&huart2,(uint8_t *)Uart2_RxBuff4_0, 4);
						//HAL_UART_Transmit(&huart2, (uint8_t *)T_uart2_XeC0, sizeof(T_uart2_XeC0),0xFFFF);//关闭氙灯
						
						
					}				
											
				}
				else if(strncmp((char *)Uart1_RxBuff,(char *)Uart1_Rx_SL, 2) == 0)
				{
					uart1_mode = 33;
					
					PWM2200();
					HAL_Delay(500);					
					
					Xe_1_S = 1;
					if(Xe_status == 0)
					{
						HAL_UART_Receive_IT(&huart2,(uint8_t *)Uart2_RxBuff3B, 3);
						HAL_UART_Transmit(&huart2, (uint8_t *)T_uart2_XeC1, sizeof(T_uart2_XeC1),0xFFFF);
					}																														
					
				}	
				else if(strncmp((char *)Uart1_RxBuff,(char *)Uart1_Rx_SR, 2) == 0)
				{
					uart1_mode = 34;
					
					PWM1800();
					HAL_Delay(500);									
//	CRC16 = crc16((uint8_t *)XeC, 2);
//	CRCL = CRC16 & 0x00FF; 	CRCH = (CRC16 >> 8) & 0x00FF;
					Xe_1_S = 1;
					if(Xe_status == 0)
					{
						HAL_UART_Receive_IT(&huart2,(uint8_t *)Uart2_RxBuff3B, 3);
						HAL_UART_Transmit(&huart2, (uint8_t *)T_uart2_XeC1, sizeof(T_uart2_XeC1),0xFFFF);
					}		
																											
				}

			
/*				else if(strncmp((char *)Uart1_RxBuff,(char *)Uart1_Rx_TEM, 2) == 0)//    *****************************************
				{
					uart1_mode = 25;  
					
					HAL_UART_Receive_IT(&huart3,(uint8_t *)Uart3_RxBuff22, 22);

				}*/				
			}
		
		}	  
		else  if(Rx2Flag  == 1)
			{
				Rx2Flag  = 0;
				if(strncmp((char *)Uart2_RxBuff3,(char *)R_uart2_RST, 3) == 0)
				{
					uart2_mode = 1;	
					
					LS[4] = 'O'; LS[5] = 'K'; 
					CRC16 = crc16((uint8_t *)LS, 2);
					CRCL = CRC16 & 0x00FF; 	CRCH = (CRC16 >> 8) & 0x00FF;
					LS[7] = CRCH; LS[6] = CRCL;	
					HAL_UART_Transmit(&huart1, (uint8_t *)LS, sizeof(LS),0xFFFF);
					HAL_Delay(2000);
				}								
				else	if(strncmp((char *)Uart2_RxBuff23,(char *)R_uart2_V, 3) == 0)
						{		
							uart2_mode = 2;
							
							R_uart2_V_T[0] = ID; strcat((char *)R_uart2_V_T, (char *)Uart2_RxBuff23); R_uart2_V_T[1] = 0x03;
							strcat((char *)R_uart2_V_T, (char *)MAC);
							HAL_UART_Transmit(&huart1, (uint8_t *)R_uart2_V_T, sizeof(R_uart2_V_T),0xFFFF);	
							//HAL_UART_Transmit(&huart1, (uint8_t *)MAC, sizeof(MAC),0xFFFF);	
						}
				else	if(strncmp((char *)Uart2_RxBuff3_0,(char *)R_uart2_ITC, 3) == 0)
						{		
							uart2_mode = 3;	
							
							HAL_UART_Transmit(&huart1, (uint8_t *)LS, sizeof(LS),0xFFFF);
						
						}
				else	if(strstr((char *)Uart2_RxBuff7,(char *)R_uart2_ITI) != NULL)
						{		
							uart2_mode = 4;
							
							LS[4] = Uart2_RxBuff7[3]; LS[5] = Uart2_RxBuff7[4];
							HAL_UART_Transmit(&huart1, (uint8_t *)LS, sizeof(LS),0xFFFF);

						}												
				else	if(strncmp((char *)Uart2_RxBuff243,(char *)R_uart2_X, 1) == 0)
										{		
											uart2_mode = 11;
											
											Uart2_RxBuff243[0] = ID;
											HAL_UART_Transmit(&huart1, (uint8_t *)Uart2_RxBuff243, sizeof(Uart2_RxBuff243),0xFFFF);

										}
				else	if(strncmp((char *)Uart2_RxBuff3_1,(char *)R_uart2_Xe, 3) == 0)
										{		
											uart2_mode = 12;
											
											T_uart2_Xe[0] = ID;										
											HAL_UART_Transmit(&huart1, (uint8_t *)T_uart2_Xe, sizeof(T_uart2_Xe),0xFFFF);
T_uart2_Xe[0] = ID;	
										}
				else	if(strncmp((char *)Uart2_RxBuff11,(char *)R_uart2_Xe_I, 2) == 0)
										{		
											uart2_mode = 13;
											
											Uart2_RxBuff11[0] = ID;
											HAL_UART_Transmit(&huart1, (uint8_t *)Uart2_RxBuff11, sizeof(Uart2_RxBuff11),0xFFFF);

										}
				else	if(strncmp((char *)Uart2_RxBuff3_5,(char *)R_uart2_PXC, 3) == 0)
										{		
											uart2_mode = 14;
											
											LS[0] = ID;
											HAL_UART_Transmit(&huart1, (uint8_t *)LS, sizeof(LS),0xFFFF);

										}
				else	if(strncmp((char *)Uart2_RxBuff7_1,(char *)R_uart2_PXI, 1) == 0)
										{		
											uart2_mode = 15;
											
											Uart2_RxBuff7_1[0] = ID;
											HAL_UART_Transmit(&huart1, (uint8_t *)Uart2_RxBuff7_1, sizeof(Uart2_RxBuff7_1),0xFFFF);

										}
				else	if(strncmp((char *)Uart2_RxBuff4099,(char *)R_uart2_WLI, 3) == 0)
										{		
											uart2_mode = 16;
											
											Uart2_RxBuff4099[0] = ID;
											HAL_UART_Transmit(&huart1, (uint8_t *)Uart2_RxBuff4099, sizeof(Uart2_RxBuff4099),0xFFFF);

										}
				else	if(strncmp((char *)Uart2_RxBuff3_2,(char *)R_uart2_BLS, 1) == 0)
										{		
											uart2_mode = 17;
											
											HAL_UART_Transmit(&huart1, (uint8_t *)LS, sizeof(LS),0xFFFF);
										}
				else	if(strncmp((char *)Uart2_RxBuff5,(char *)R_uart2_BLI, 1) == 0)
										{		
											uart2_mode = 18;
											LS[4] = Uart2_RxBuff5[1]; LS[5] = Uart2_RxBuff5[2];
											HAL_UART_Transmit(&huart1, (uint8_t *)LS, sizeof(LS),0xFFFF);
										}
				else	if(strncmp((char *)Uart2_RxBuff3A,(char *)R_uart2_XeG, 3) == 0)
										{		
											uart2_mode = 20;
											
											Xe_status = 0;
											//LS[4] = Uart2_RxBuff5[1]; LS[5] = Uart2_RxBuff5[2];
											HAL_UART_Transmit(&huart1, (uint8_t *)LS, sizeof(LS),0xFFFF);
										}				
				else	if(strncmp((char *)Uart2_RxBuff3B,(char *)R_uart2_XeK, 3) == 0)
										{		
											uart2_mode = 21;
											
											Xe_status = 1;
											//LS[4] = Uart2_RxBuff5[1]; LS[5] = Uart2_RxBuff5[2];
											HAL_UART_Transmit(&huart1, (uint8_t *)LS, sizeof(LS),0xFFFF);
										}										
				else	if(strncmp((char *)Uart2_RxBuff4A,(char *)R_uart2_XeI, 1) == 0)
										{		
											uart2_mode = 22;
											
											Uart2_RxBuff4A[0] = ID;
											HAL_UART_Transmit(&huart1, (uint8_t *)Uart2_RxBuff4A, sizeof(Uart2_RxBuff4A),0xFFFF);
										}	
				else	if(strncmp((char *)Uart2_RxBuff4_1,(char *)R_uart2_XeI, 1) == 0)
										{		
											uart2_mode = 33;
											
											if(Uart2_RxBuff4_1[1] != 0x00)
											Xe_status = 1;	
											else
											Xe_status = 0;	
											memset(Uart2_RxBuff4_1,0,4);
										}
				else	if(strncmp((char *)Uart2_RxBuff4_2,(char *)R_uart2_XeI, 1) == 0)
										{		
											if(Uart2_RxBuff4_2[1] != 0)
											Xe_status = 1;	
											else
											Xe_status = 0;	
											memset(Uart2_RxBuff4_2,0,4);
										}										
				else	if(strncmp((char *)Uart2_RxBuff2063,(char *)R_uart2_S, 4) == 0)
										{		
											uart2_mode = 32;
											
											Uart2_RxBuff2063[0] = ID;
											HAL_UART_Transmit(&huart1, (uint8_t *)Uart2_RxBuff2063, sizeof(Uart2_RxBuff2063),0xFFFF);

										}														
										
										
										
										
										
			}
/*		else  if(Rx3Flag  == 1)
			{
				Rx3Flag = 0;
				if(strncmp((char *)Uart3_RxBuff22,(char *)R_uart3_TEM, 1) == 0)
				{
					uart2_mode = 25;	
					
					LS[4] = 'O'; LS[5] = 'K'; 
					CRC16 = crc16((uint8_t *)LS, 2);
					CRCL = CRC16 & 0x00FF; 	CRCH = (CRC16 >> 8) & 0x00FF;
					LS[7] = CRCH; LS[6] = CRCL;	
					HAL_UART_Transmit(&huart1, (uint8_t *)LS, sizeof(LS),0xFFFF);					
				}												
				
			}*/

		else if(Xe_0_S == 1)//*****************************************
			{								
				Xe_0_S = 0;
				if(Xe_status == 0)	 
				{						
					HAL_UART_Receive_IT(&huart2,(uint8_t *)Uart2_RxBuff2063, 2063);
					HAL_UART_Transmit(&huart2, (uint8_t *)T_uart2_S, sizeof(T_uart2_S),0xFFFF);					
				}	
			}
		else if(Xe_1_S == 1)//*****************************************
			{								
				Xe_1_S = 0;
				if(Xe_status == 1)	 
				{						
					HAL_UART_Receive_IT(&huart2,(uint8_t *)Uart2_RxBuff2063, 2063);
					HAL_UART_Transmit(&huart2, (uint8_t *)T_uart2_S, sizeof(T_uart2_S),0xFFFF);					
				}	
			}

		HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_1);//wft				
		HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13);//wft
		
		HAL_Delay(100);
		
		switch(uart1_mode)
		{
			case 1:{  	HAL_UART_Receive_IT(&huart1,(uint8_t *)Uart1_RxBuff, 8); 
						uart1_mode = 0; memset(Uart1_RxBuff,0,8); break;}
			
			case 2:{	HAL_UART_Receive_IT(&huart1,(uint8_t *)Uart1_RxBuff, 8);
						uart1_mode = 0; memset(Uart1_RxBuff,0,8); break;}

			case 3:{	HAL_UART_Receive_IT(&huart1,(uint8_t *)Uart1_RxBuff, 8);
						uart1_mode = 0; memset(Uart1_RxBuff,0,8); 	break;}

			case 4:{	HAL_UART_Receive_IT(&huart1,(uint8_t *)Uart1_RxBuff, 8);
						uart1_mode = 0; memset(Uart1_RxBuff,0,8); 	break;}

			case 8:{	HAL_UART_Receive_IT(&huart1,(uint8_t *)Uart1_RxBuff, 8);
						uart1_mode = 0; memset(Uart1_RxBuff,0,8);   break;}	
						
			case 9:{	HAL_UART_Receive_IT(&huart1,(uint8_t *)Uart1_RxBuff, 8);
						uart1_mode = 0; memset(Uart1_RxBuff,0,8);   break;}		   
			case 10:{	HAL_UART_Receive_IT(&huart1,(uint8_t *)Uart1_RxBuff, 8);
						 memset(Uart1_RxBuff,0,8);   break;}	
			case 11:{	HAL_UART_Receive_IT(&huart1,(uint8_t *)Uart1_RxBuff, 8);
						uart1_mode = 0; memset(Uart1_RxBuff,0,8); 	break;}	
			case 12:{	HAL_UART_Receive_IT(&huart1,(uint8_t *)Uart1_RxBuff, 8);
						uart1_mode = 0; memset(Uart1_RxBuff,0,8); 	break;}	
			case 13:{	HAL_UART_Receive_IT(&huart1,(uint8_t *)Uart1_RxBuff, 8);
						uart1_mode = 0; memset(Uart1_RxBuff,0,8); 	break;}				
			case 14:{	HAL_UART_Receive_IT(&huart1,(uint8_t *)Uart1_RxBuff, 8);
						uart1_mode = 0; memset(Uart1_RxBuff,0,8); 	break;}						
			case 15:{	HAL_UART_Receive_IT(&huart1,(uint8_t *)Uart1_RxBuff, 8);
						uart1_mode = 0; memset(Uart1_RxBuff,0,8); 	break;}							
			case 16:{	HAL_UART_Receive_IT(&huart1,(uint8_t *)Uart1_RxBuff, 8);
						uart1_mode = 0; memset(Uart1_RxBuff,0,8); 	break;}	
			case 17:{	HAL_UART_Receive_IT(&huart1,(uint8_t *)Uart1_RxBuff, 8);
						uart1_mode = 0; memset(Uart1_RxBuff,0,8); 	break;}
			case 18:{	HAL_UART_Receive_IT(&huart1,(uint8_t *)Uart1_RxBuff, 8);
						uart1_mode = 0; memset(Uart1_RxBuff,0,8); 	break;}			
	
			
			case 20:{	HAL_UART_Receive_IT(&huart1,(uint8_t *)Uart1_RxBuff, 8); 
				        uart1_mode = 0; memset(Uart1_RxBuff,0,8); 	break;}
			case 21:{	HAL_UART_Receive_IT(&huart1,(uint8_t *)Uart1_RxBuff, 8);
						uart1_mode = 0; memset(Uart1_RxBuff,0,8); 	break;}
			case 22:{	HAL_UART_Receive_IT(&huart1,(uint8_t *)Uart1_RxBuff, 8);
						uart1_mode = 0; memset(Uart1_RxBuff,0,8); 	break;}
			case 25:{	HAL_UART_Receive_IT(&huart1,(uint8_t *)Uart1_RxBuff, 8);
						uart1_mode = 0; memset(Uart1_RxBuff,0,8); 	break;}
			case 32:{	HAL_UART_Receive_IT(&huart1,(uint8_t *)Uart1_RxBuff, 8);
						uart1_mode = 0; memset(Uart1_RxBuff,0,8); 	break;}
			case 33:{	HAL_UART_Receive_IT(&huart1,(uint8_t *)Uart1_RxBuff, 8);
						uart1_mode = 0; memset(Uart1_RxBuff,0,8); 	break;}
			case 34:{	HAL_UART_Receive_IT(&huart1,(uint8_t *)Uart1_RxBuff, 8);
						uart1_mode = 0; memset(Uart1_RxBuff,0,8); 	break;}

						default:  ;
		}
		
		
		
		
		switch(uart2_mode)
		{
			case 1:{	uart2_mode = 0; memset(Uart2_RxBuff3,0,3);break;}
			case 2:{	uart2_mode = 0; memset(Uart2_RxBuff23,0,23); memset(R_uart2_V_T,0,50);break;}
			case 3:{	uart2_mode = 0; memset(Uart2_RxBuff3_0,0,3);break;}
			case 4:{	uart2_mode = 0; memset(Uart2_RxBuff7,0,7);break;}											


			
			case 9:{	uart2_mode = 0; memset(Uart2_RxBuff2063,0,2063);break;}			
			case 10:{	uart2_mode = 0; memset(Uart2_RxBuff2063,0,2063);break;}
			case 11:{	uart2_mode = 0; memset(Uart2_RxBuff243,0,243);break;}			
			case 12:{	uart2_mode = 0; memset(Uart2_RxBuff3_1,0,3);break;}		
			case 13:{	uart2_mode = 0; memset(Uart2_RxBuff11,0,11);break;}				
			case 14:{	uart2_mode = 0; memset(Uart2_RxBuff3_5,0,3);break;}
			case 15:{	uart2_mode = 0; memset(Uart2_RxBuff3,0,3);break;}
			case 16:{	uart2_mode = 0; memset(Uart2_RxBuff4099,0,9);break;}
			case 17:{	uart2_mode = 0; memset(Uart2_RxBuff3_2,0,3);break;}			
			case 18:{	uart2_mode = 0; memset(Uart2_RxBuff5,0,5);break;}			

            case 20:{	uart2_mode = 0; memset(Uart2_RxBuff3A,0,3);break;}
			case 21:{	uart2_mode = 0; memset(Uart2_RxBuff3B,0,3);break;}	
			case 22:{	uart2_mode = 0; memset(Uart2_RxBuff4A,0,4);break;}	
	
			case 32:{	uart2_mode = 0; memset(Uart2_RxBuff2063,0,9);break;}

			default:;			
			
			
		}

	}
}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//CRC-16码（modbus）校验
//addr：需要校验的字节数组
//num：需要校验的字节数
//返回值：16位的CRC校验码
uint16_t crc16(uint8_t *addr,uint8_t num)
{
	uint16_t  i,j,temp;
	uint16_t  crc=0xFFFF;	
	for(i=0;i<num;i++)
	{
		crc=crc^(*addr);
		for(j=0;j<8;j++)
		{
			temp=crc&0x0001;
			crc=crc>>1;
			if(temp)
			{
				crc=crc^0xA001;
			}
		}
		addr++;
	}
	return crc;
	
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
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{			
		if(huart->Instance == USART1)
			{
				Rx1Flag = 1;
				HAL_UART_Receive_IT(&huart1,(uint8_t *)Uart1_RxBuff, 8);		
			}
			else	if(huart->Instance == USART2)
					{
						Rx2Flag  = 1;																		
					 }
			else	if(huart->Instance == USART3)
//					{
						Rx3Flag  = 1;																		
//					 }										 
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

