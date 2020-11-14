/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include "interruptConfig.h"
#include "ignition.h"
#include "crankshaft.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */


int triggerTooth;
float triggerAngle;
float delayAngle;
uint32_t timeToIgnition;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim21;
TIM_HandleTypeDef htim22;

/* USER CODE BEGIN PV */
uint32_t captures[2];
volatile uint8_t captureDone = 0;
//volatile float frequency = 0;
//volatile float period = 0;






uint16_t count = 0;

uint32_t diffCapture = 0;

__IO uint32_t IgniteDwellTime = 1000;
uint8_t Crank_HalfCycleFlag = 0;
int angleToIgnite = 120;
int currentToothAngle = 0;
// HAL GEBER DECLARATIONS
//uint32_t CrankShaftFreqHz = 0;
//uint32_t *pCrankShaftFreqHz = &crankShaftFreqHz;


//int adcVal;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM22_Init(void);
static void MX_TIM21_Init(void);
static void MX_NVIC_Init(void);
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
  MX_TIM2_Init();
  MX_TIM22_Init();
  MX_TIM21_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */

  // Start all TIM channels and Interrupts related to TIM
  HAL_TIM_OnePulse_Start_IT(&htim22, TIM_CHANNEL_1);
  //HAL_TIM_OnePulse_Start_IT(&htim22, TIM_CHANNEL_2);
  HAL_TIM_Base_Start_IT(&htim22);


  /*
  //HAL_TIM_IC_Start_DMA(&htim2,	 HAL_TIM_ACTIVE_CHANNEL_2,	 (uint32_t*)	 captures,	 2);
  //HAL_TIM_Base_Start_IT(&htim2);
  //HAL_TIM_Base_Start_IT(&htim21);
   */

  HAL_TIM_Base_Start_IT(&htim21);
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2);
  HAL_TIM_OC_Start_IT(&htim21, TIM_CHANNEL_2);

  HAL_Delay(500);

  /* All module Inits */
  Crank_Init();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  //Ignition_SetIgnitionTiming(10,20,30);

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_4;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* TIM21_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM21_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM21_IRQn);
  /* TIM22_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM22_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM22_IRQn);
  /* TIM2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM2_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(TIM2_IRQn);
  /* EXTI4_15_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 43;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM21 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM21_Init(void)
{

  /* USER CODE BEGIN TIM21_Init 0 */

  /* USER CODE END TIM21_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM21_Init 1 */

  /* USER CODE END TIM21_Init 1 */
  htim21.Instance = TIM21;
  htim21.Init.Prescaler = 31;
  htim21.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim21.Init.Period = 10000;
  htim21.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim21.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim21) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim21, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim21) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim21, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim21, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM21_Init 2 */

  /* USER CODE END TIM21_Init 2 */

}

/**
  * @brief TIM22 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM22_Init(void)
{

  /* USER CODE BEGIN TIM22_Init 0 */

  /* USER CODE END TIM22_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM22_Init 1 */

  /* USER CODE END TIM22_Init 1 */
  htim22.Instance = TIM22;
  htim22.Init.Prescaler = 31;
  htim22.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim22.Init.Period = 10;
  htim22.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim22.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim22) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim22, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim22) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OnePulse_Init(&htim22, TIM_OPMODE_SINGLE) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim22, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 100;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim22, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim22, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM22_Init 2 */
  //__HAL_TIM_ENABLE(&htim22);
  /* USER CODE END TIM22_Init 2 */
  HAL_TIM_MspPostInit(&htim22);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2|GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC2 PC3 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
float angleOfIgnition = 120;
float tempAngle = 0;

void Scheduler_10ms(){
	// Call all 10ms related functions
	//HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_3);
	IgniteDwellTime = 1000;
	//TIM22->CCR1 = dwellTime;
	//TIM22->ARR = 2500;
	// angle to spark has to be small enough to fit in the speed but large enough to allow dwell
	// dwell time around 2ms, means that one halfCycle is limited to min 2ms, which is 4ms for one crankshaft rotation which is 250hz with current method (works with 180hz and 130 degrees)
	tempAngle = tempAngle + 0.1;
	angleOfIgnition = tempAngle;

	angleOfIgnition = 120;

	if (tempAngle >= 90){
		tempAngle = 0;
	}

	Ignition_SetIgnitionTiming(1,20,Crank_ShaftFreqHz,Crank_TeethNmbr_P,Crank_MissingTeethNmbr_P);
	Ignition_SetIgnitionTiming(3,130,Crank_ShaftFreqHz,Crank_TeethNmbr_P,Crank_MissingTeethNmbr_P);


	//Ignition_SetIgnitionTiming(2000,angleToIgnite,Crank_ShaftFreqHz);
	//TIM22->CR1 |= TIM_CR1_CEN;
	if (IgniteDwellTime >= 2000)
	  IgniteDwellTime = 1000;
	else
	  IgniteDwellTime = IgniteDwellTime +1;

	//HAL_Delay(100);
	if (angleToIgnite >= 120){
	  angleToIgnite = 80;
	}
	else
	  angleToIgnite = angleToIgnite + 1;
	angleToIgnite = 120;
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim){
	if(htim == &htim22){
		// Ignition OPM TIM Channels
		if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1){

			// Cylinder 1
			if (Ignition_CoilStateCyl1 == DWELL){
				//GPIOC->BSRR = GPIO_PIN_3;
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_3,GPIO_PIN_SET);
			}
			if (Ignition_CoilStateCyl1 == FIRE){
				//GPIOC->BRR = GPIO_PIN_3;
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_3,GPIO_PIN_RESET);
			}


			// Cylinder 3
			if (Ignition_CoilStateCyl3 == DWELL){
				//GPIOC->BSRR = GPIO_PIN_3; // TODO use different pin in future . the one that corresponds cyl3
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_2,GPIO_PIN_SET);
			}
			if (Ignition_CoilStateCyl3 == FIRE){
				//GPIOC->BRR = GPIO_PIN_3;; // TODO use different pin in future . the one that corresponds cyl3
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_2,GPIO_PIN_RESET);
			}

		}
		if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2){
			//HAL_GPIO_WritePin(GPIOC,GPIO_PIN_3,GPIO_PIN_SET);
		}
		//HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_3);

	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	//HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_3);
	//HAL_GPIO_WritePin(GPIOC,GPIO_PIN_3,GPIO_PIN_RESET);
	if (GPIO_Pin == GPIO_PIN_8){
		if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8))
			Cam_CycleStart = 1;
		else
			Cam_CycleStart = 0;
	}else{
		__NOP();
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim == &htim21){
		// Call the 10ms task
		Scheduler_10ms();
	}else{
		__NOP();
	}

}

void HAL_TIM_IC_CaptureCallback( TIM_HandleTypeDef *htim ){


	if(htim == &htim2){
		if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2 ){
			//HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_3);
			// Ignite before halgeber driver, if the tooth is fine. There is huge delay (30-40us) in high speed9:
			Crank_AngleCalc(); // roughly 10us computation time


			//Cam_HalGeberDriver(####);

			Crank_CamPositionSync();

			Crank_HalfCycleFlag = 0; // every reset of the crankshaft (initial value) is start of a new cycle. halfCycle == 1 when we are in the 2nd part of camshaft rotation.
			//htim2->Instance->CR1 = 1000;
			//htim2.Instance->CR1 = 1000;

			if (Crank_TeethCount == 1){
				Ignition_StartTimerDwellCylinder(1,10);
				//Ignition_FireCylinder(1);
			}


			if (Crank_TeethCount == Ignition_FireToothCyl1){
				//HAL_GPIO_WritePin(GPIOC,GPIO_PIN_3,GPIO_PIN_SET);
				Ignition_StartTimerFireCylinder(1);
			}

			if (Crank_TeethCount == 6){
				Ignition_StartTimerDwellCylinder(3,10);
				//Ignition_FireCylinder(1);
			}


			if (Crank_TeethCount == Ignition_FireToothCyl3){
				//HAL_GPIO_WritePin(GPIOC,GPIO_PIN_3,GPIO_PIN_SET);
				Ignition_StartTimerFireCylinder(3);
			}


			/*
			if (Crank_TeethCount == 15){
				//TIM22->CR1 |= TIM_CR1_CEN;
				if (Crank_HalfCycleFlag == 0){
					Ignition_FireCylinder(3,timeToIgnition);
				}
				else{
					//Ignition_FireCylinderB(4);
				}
			}*/

			Crank_HalGeberDriver(&htim2);
		}
	}else{
		__NOP();
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
