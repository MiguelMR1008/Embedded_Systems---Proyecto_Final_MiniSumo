/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2019 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */
volatile uint16_t timers[9],indice=0,cont=5000;
volatile uint8_t pwm1=50,pwm2=50,buffer[64];
volatile uint32_t adc_buf[6];
volatile unsigned char retorno='0',encontrado='0';

void control_motores();
void seguidor1_process();
void seguidor2_process();
void seguidor3_process();
void seguidor4_process();
void sharp1_process();
void sharp2_process();

typedef enum{
	  BUSCANDO=0,
	  ATACANDO_ATRAS,
	  ATACANDO_ADELANTE,
  }ST_MOTOR;

typedef enum{
	  S_ALERTA=0,
	  S_ESTABLE,
  }ST_SEGUIDOR;

typedef enum{
	  S_BUSCANDO=0,
	  S_FIJO,
  }ST_SHARP;

ST_MOTOR st_motores=BUSCANDO;
ST_SEGUIDOR st_seg1,st_seg2,st_seg3,st_seg4=S_ESTABLE;
ST_SHARP st_sharp1,st_sharp2=S_BUSCANDO;

  enum {
  	TMOTORES=0,
  	TSEG1,
  	TSEG2,
  	TSEG3,
  	TSEG4,
  	TSHARP1,
  	TSHARP2,
  	TADC,
  	TIMERS
  };
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_ADC1_Init(void);
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

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_USB_DEVICE_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
 HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
 HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_2);
 //HAL_TIM_Base_Start_IT(&htim2);
 HAL_ADC_Start_DMA(&hadc1,(uint32_t*)adc_buf,6);
 HAL_ADC_Start_IT(&hadc1);
 HAL_TIM_Base_Start_IT(&htim3);
 //HAL_TIM_Base_Start(&htim4);
 __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2,pwm1);
 __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2,pwm2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	if(cont==0){
		control_motores();
		if(timers[TADC]==0){
			HAL_ADC_Start_IT(&hadc1);
			seguidor1_process();
			seguidor2_process();
			seguidor3_process();
			if(retorno=='0'){
				//if(encontrado=='0'||encontrado=='1')
					sharp1_process();
				//if(encontrado=='0'||encontrado=='2')
					sharp2_process();
			}
			timers[TADC]=10;
		}
	}
    /*for (int i=0;i<100;i++){
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2,i);
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2,i);
		HAL_Delay(200);
	}for (int j=100;j>0;j--){
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2,j);
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2,j);
		HAL_Delay(200);
	}*/
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

  /**Initializes the CPU, AHB and APB busses clocks 
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
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC|RCC_PERIPHCLK_USB;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /**Common config 
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 6;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /**Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_71CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /**Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /**Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /**Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /**Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /**Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 71;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 99;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 36000;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 71;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 99;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){

	if(htim->Instance == TIM3){
		uint8_t i;
		/* Comprueba si la duración de cada timer se ha completado. El for comprueba el estado de
		 * cada uno de los 4 leds.
		 * Si el valor de cada timer es mayor a 0, quiere decir que el tiempo no
		 * se ha completado. Por lo tanto se le disminuye 1 al valor o tiempo restante
		*/
		/*cont++;
		if(cont==2000)
			HAL_GPIO_WritePin(GPIOA,Led_1_PA1_Pin,GPIO_PIN_SET);*/
		for(i=0;i<TIMERS;i++){
			if(timers[i]!=0){
				timers[i]--;
			}
		}if(cont>=1){
			cont--;
		}
	}
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
	uint32_t val[6];
	if(hadc->Instance==ADC1){
		val[0]=adc_buf[0];val[1]=adc_buf[1];
		val[2]=adc_buf[2];val[3]=adc_buf[3];
		val[4]=adc_buf[4];val[6]=adc_buf[5];
	}
}

void control_motores(){
	switch(st_motores){
		case BUSCANDO:
			if(timers[TMOTORES]==0){
				if(pwm1<=99||pwm2>=1){
					pwm1++;
					pwm2--;
					__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2,pwm1);
					__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2,pwm2);
					/*pwm1++;
					pwm2--;*/
				}
				timers[TMOTORES]=1;
			}
		break;
		case ATACANDO_ATRAS:
			if(timers[TMOTORES]==0){
				if(pwm1<=99||pwm2<=99){
					pwm1++;
					pwm2++;
					__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2,pwm1);
					__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2,pwm2);
				}
				timers[TMOTORES]=1;
			}
		break;
		case ATACANDO_ADELANTE:
			if(timers[TMOTORES]==0){
				if(pwm1>=1||pwm2>=1){
					pwm1--;
					pwm2--;
					__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2,pwm1);
					__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2,pwm2);
				}
				timers[TMOTORES]=1;
			}
		break;
	}
}

void seguidor1_process(){
		switch(st_seg1){
			case S_ESTABLE:
				//if(timers[TSEG1]==0){
					if(adc_buf[1]<2500){
						st_seg1=S_ALERTA;
						st_motores=ATACANDO_ADELANTE;
						timers[TSEG1]=500;
						retorno='1';
						pwm1=50;pwm2=50;
					}
				//}
			break;
			case S_ALERTA:
				if(timers[TSEG1]==0){
					if(adc_buf[1]<2500)
						timers[TSEG1]=500;
					else{
						st_seg1=S_ESTABLE;
						st_motores=BUSCANDO;
						timers[TSEG1]=10;
						retorno='0';
						pwm1=50;pwm2=50;
						//HAL_ADC_Stop(&hadc1);
					}
				}
			break;
		}
}

void seguidor2_process(){
		switch(st_seg2){
			case S_ESTABLE:
				//if(timers[TSEG2]==0){
					if(adc_buf[2]<2500){
						st_seg2=S_ALERTA;
						st_motores=ATACANDO_ADELANTE;
						timers[TSEG2]=500;
						retorno='1';
						pwm1=50;pwm2=50;
					}
				//}
			break;
			case S_ALERTA:
				if(timers[TSEG2]==0){
					if(adc_buf[2]<2500)
						timers[TSEG2]=500;
					else{
						st_seg2=S_ESTABLE;
						st_motores=BUSCANDO;
						timers[TSEG2]=10;
						retorno='0';
						pwm1=50;pwm2=50;
						//HAL_ADC_Stop(&hadc1);
					}
				}
			break;
		}
}

void seguidor3_process(){
		switch(st_seg3){
			case S_ESTABLE:
				if(timers[TSEG3]==0){
					if(adc_buf[3]<2500){
						st_seg3=S_ALERTA;
						st_motores=ATACANDO_ATRAS;
						timers[TSEG3]=500;
						retorno='1';
						pwm1=50;pwm2=50;
					}
				}
			break;
			case S_ALERTA:
				if(timers[TSEG3]==0){
					if(adc_buf[3]<2500)
						timers[TSEG3]=500;
					else{
						st_seg3=S_ESTABLE;
						st_motores=BUSCANDO;
						timers[TSEG3]=10;
						retorno='0';
						pwm1=50;pwm2=50;
						//HAL_ADC_Stop(&hadc1);
					}
				}
			break;
		}
}

void sharp1_process(){
		switch(st_sharp1){
			case S_BUSCANDO:
				if(timers[TSHARP1]==0){
					if(adc_buf[4]>2500){
						st_sharp1=S_FIJO;
						st_motores=ATACANDO_ATRAS;
						timers[TSHARP1]=10;
						encontrado='1';
						pwm1=50;pwm2=50;
					}
				}
			break;
			case S_FIJO:
				if(timers[TSHARP1]==0){
					if(adc_buf[4]>2500)
						timers[TSHARP1]=10;
					else{
						st_sharp1=S_BUSCANDO;
						st_motores=BUSCANDO;
						timers[TSHARP1]=10;
						encontrado='0';
						pwm1=50;pwm2=50;
						//HAL_ADC_Stop(&hadc1);
					}
				}
			break;
		}
}

void sharp2_process(){
		switch(st_sharp2){
			case S_BUSCANDO:
				if(timers[TSHARP2]==0){
					if(adc_buf[5]>2500){
						st_sharp2=S_FIJO;
						st_motores=ATACANDO_ADELANTE;
						timers[TSHARP2]=10;
						encontrado='2';
						pwm1=50;pwm2=50;
					}
				}
			break;
			case S_FIJO:
				if(timers[TSHARP2]==0){
					if(adc_buf[5]>2500)
						timers[TSHARP2]=10;
					else{
						st_sharp2=S_BUSCANDO;
						st_motores=BUSCANDO;
						timers[TSHARP2]=10;
						encontrado='0';
						pwm1=50;pwm2=50;
						//HAL_ADC_Stop(&hadc1);
					}
				}
			break;
		}
}
void CDC_Receive_Callback(uint8_t *buf,uint32_t len){
	memcpy(buffer+indice,buf,len);
	indice=indice+len;
	indice=0;
	if(buffer[0]=='1'){
		pwm1=49;
		pwm2=49;
		st_motores=ATACANDO_ATRAS;
	}else if(buffer[0]=='2'){
		pwm1=51;
		pwm2=51;
		st_motores=ATACANDO_ADELANTE;
	}else if(buffer[0]=='0'){
		pwm1=50;
		pwm2=50;
		st_motores=BUSCANDO;
	}

}


//HAL_GPIO_ReadPin(GPIOB,Pulsador2_PB13_Pin)==GPIO_PIN_RESET
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
