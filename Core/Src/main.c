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
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "math.h"
#include "fonts.h"
#include "ssd1306.h"
#include "Animacion_Carga.h"

#include "app_common_typedef.h"
#include "MPU6050.h"

#define RESCALE_GYRO 90

//#define Pulgar
#define Indice
//#define Medio
//#define Anular
//#define Menique


#define Gyroscope
//#define Zigbee

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

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;

UART_HandleTypeDef huart1;

/* Definitions for Tarea1 */
osThreadId_t Tarea1Handle;
const osThreadAttr_t Tarea1_attributes = {
  .name = "Tarea1",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Tarea2 */
osThreadId_t Tarea2Handle;
const osThreadAttr_t Tarea2_attributes = {
  .name = "Tarea2",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Tarea3 */
osThreadId_t Tarea3Handle;
const osThreadAttr_t Tarea3_attributes = {
  .name = "Tarea3",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Tarea4 */
osThreadId_t Tarea4Handle;
const osThreadAttr_t Tarea4_attributes = {
  .name = "Tarea4",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Queue1 */
osMessageQueueId_t Queue1Handle;
const osMessageQueueAttr_t Queue1_attributes = {
  .name = "Queue1"
};
/* Definitions for Queue2 */
osMessageQueueId_t Queue2Handle;
const osMessageQueueAttr_t Queue2_attributes = {
  .name = "Queue2"
};
/* Definitions for Flag */
osEventFlagsId_t FlagHandle;
const osEventFlagsAttr_t Flag_attributes = {
  .name = "Flag"
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM6_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM7_Init(void);
void Inicializacion(void *argument);
void AnalogRead(void *argument);
void Gyro(void *argument);
void EnvioDatos(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//GLOBAL VARIABLES HERE
float count_gyro = 0;
float dt = 0;
extern uint32_t TickGet;

Init_State Flag_SSD_State = WAIT_TIMER_TO_BE_READY;
Gyro_State Flag_Gyro = SEARCHING_GYRO_I2C_DEVICE;
uint32_t analog_max[5]={4000,4000,4000,4000,4000};  // 0 es pulgar, 5 meñique
uint32_t analog_min[5]={0,0,0,0,0};  // 0 es pulgar, 5 meñique

//END GLOBAL VARIABLES


//DEBUG VARIABLES HERE
uint8_t BufferTx[9];

void MPU6050_Init (I2C_HandleTypeDef hi2c);
struct GyroFunction MPU6050_Read_Gyro (I2C_HandleTypeDef hi2c);
struct AccelFunction MPU6050_Read_Accel (I2C_HandleTypeDef hi2c);
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
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_TIM6_Init();
  MX_USART1_UART_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_Base_Start_IT(&htim6);
  HAL_TIM_Base_Start_IT(&htim7);
  __HAL_UART_ENABLE_IT(&huart1,UART_IT_TC);


  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of Queue1 */
  Queue1Handle = osMessageQueueNew (16, sizeof(struct Datos_Analogicos), &Queue1_attributes);

  /* creation of Queue2 */
  Queue2Handle = osMessageQueueNew (16, sizeof(struct Datos_Gyro), &Queue2_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of Tarea1 */
  Tarea1Handle = osThreadNew(Inicializacion, NULL, &Tarea1_attributes);

  /* creation of Tarea2 */
  Tarea2Handle = osThreadNew(AnalogRead, NULL, &Tarea2_attributes);

  /* creation of Tarea3 */
  Tarea3Handle = osThreadNew(Gyro, NULL, &Tarea3_attributes);

  /* creation of Tarea4 */
  Tarea4Handle = osThreadNew(EnvioDatos, NULL, &Tarea4_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Create the event(s) */
  /* creation of Flag */
  FlagHandle = osEventFlagsNew(&Flag_attributes);

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */

  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 16;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_I2C1
                              |RCC_PERIPHCLK_ADC;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_PLLSAI1;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_MSI;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 16;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV7;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_ADC1CLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
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
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 5;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_640CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00100413;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /** I2C Fast mode Plus enable
  */
  HAL_I2CEx_EnableFastModePlus(I2C_FASTMODEPLUS_I2C1);
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 320;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 65535;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 320;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 100;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1|LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB1 LD3_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_1|LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

}

/* USER CODE BEGIN 4 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(&htim6 == htim){
		count_gyro++;
	}
	else if(&htim7 == htim){
		TickGet++;
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == GPIO_PIN_0)
	{
		osEventFlagsSet(FlagHandle, 1U);
	}
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_Inicializacion */
/**
  * @brief  Function implementing the Tarea1 thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_Inicializacion */
void Inicializacion(void *argument)
{
  /* USER CODE BEGIN 5 */
	HAL_StatusTypeDef status;
	HAL_StatusTypeDef Flag_SSD1306;

	uint8_t Calibration_fail = 0;

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET); //PCB Led Green
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET); //PCB Led Red
  /* Infinite loop */
  for(;;)
  {

	  switch(Flag_SSD_State)
	  {
	  case WAIT_TIMER_TO_BE_READY:
		  if(1000 < TickGet) //before 30000
		  {
			  Flag_SSD_State = SEARCHING_OLED_I2C_DEVICE ; //waiting for timer to be established
		  }
		  break;
	  case SEARCHING_OLED_I2C_DEVICE:
		  Flag_Gyro = SEARCHING_GYRO_I2C_DEVICE;
		  Flag_SSD1306 = HAL_I2C_IsDeviceReady(&hi2c1, SSD1306_I2C_ADDR, 1, 1000); //before to the 20000 ms timeout
		  if(HAL_OK == Flag_SSD1306)
		  {
			  Flag_SSD_State = OLED_IS_AVAILABLE;	//i2c screen is available
		  }
		  else
		  {
			  Flag_SSD_State = OLED_IS_NOT_AVAILABLE; //i2c is not available
		  }

		  if(1U == Calibration_fail)
		  {
			  Calibration_fail = 0;
		  }

		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);

		  osEventFlagsClear(FlagHandle,1U); // Clear Flag 1 and 2
		  osEventFlagsClear(FlagHandle,2U); // Clear Flag 1 and 2
		  break;
	  case OLED_IS_AVAILABLE:					//init oled screen
		  SSD1306_Init();
		  SSD1306_Clear();
		  SSD1306_GotoXY(0, 0);
		  SSD1306_Puts("INICIANDO..", &Font_11x18, 1);
		  SSD1306_UpdateScreen();

		  for(int i = 0; i<4; i++)
		  {
			  for(int j = 0; j<38; j++)
			  {
				  SSD1306_DrawFilledRectangle(0, 20, 128, 45, 0x00);
				  SSD1306_DrawBitmap(0, 20, Animacion[j], 128, 45, 1);
				  SSD1306_UpdateScreen();
			  }
		  }
		  SSD1306_Clear();

		  SSD1306_GotoXY(0, 0);
		  SSD1306_Puts("INICIANDO..", &Font_11x18, 1);

		  SSD1306_GotoXY(0, 30);
		  SSD1306_Puts("Pulsa para", &Font_7x10, 1);
		  SSD1306_GotoXY(0, 50);
		  SSD1306_Puts("calibrar", &Font_7x10, 1);
		  SSD1306_UpdateScreen();

		  osEventFlagsWait(FlagHandle, 1U, osFlagsNoClear, osWaitForever); // wait Flag 1 from ISR
		  Flag_SSD_State = CALIBRATION_OLED;
		  break;
	  case OLED_IS_NOT_AVAILABLE:
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
		  osEventFlagsWait(FlagHandle, 1U, osFlagsNoClear, osWaitForever);
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
		  osEventFlagsClear(FlagHandle,2U);
		  //initialitation by a led is available
		  Flag_SSD_State = INIT_WITH_LED;
		  break;
	  case CALIBRATION_OLED:
		  osEventFlagsClear(FlagHandle,1U);
		  Flag_SSD_State = RUNNING_O;
		  break;
	  case INIT_WITH_LED:
		  osEventFlagsClear(FlagHandle,1U);
		  Flag_SSD_State = RUNNING_L;
		  break;
	  case RUNNING_O:
		  if(0U == Calibration_fail)
		  {
			  osEventFlagsSet(FlagHandle,2U);
		  }
		  osEventFlagsWait(FlagHandle, 1U, osFlagsNoClear, osWaitForever);
		  osEventFlagsClear(FlagHandle,2U);
		  osEventFlagsClear(FlagHandle,4U);
		  osEventFlagsClear(FlagHandle,8U);
		  osDelay(200);
		  Flag_SSD_State = SEARCHING_OLED_I2C_DEVICE;
		  break;
	  case RUNNING_L:
		  if(0U == Calibration_fail)
		  {
			  osEventFlagsSet(FlagHandle,2U);
		  }
		  osEventFlagsWait(FlagHandle, 1U, osFlagsNoClear, osWaitForever);
		  osEventFlagsClear(FlagHandle,2U);
		  osEventFlagsClear(FlagHandle,4U);
		  osEventFlagsClear(FlagHandle,8U);
		  osDelay(200);
		  Flag_SSD_State = SEARCHING_OLED_I2C_DEVICE;
		  break;
	  }



	  //only if init can be done by one of two's options, the FW can be run

	  if((CALIBRATION_OLED == Flag_SSD_State) || (INIT_WITH_LED == Flag_SSD_State))
	  {
		  if(CALIBRATION_OLED == Flag_SSD_State)
		  {
			  SSD1306_Clear();

			  SSD1306_GotoXY(0, 0);
			  SSD1306_Puts("CALIBRACION", &Font_11x18, 1);

			  SSD1306_GotoXY(0, 30);
			  SSD1306_Puts("Abre la", &Font_7x10, 1);
			  SSD1306_GotoXY(0, 50);
			  SSD1306_Puts("mano", &Font_7x10, 1);
			  SSD1306_UpdateScreen();

			  SSD1306_GotoXY(90, 35);
			  SSD1306_Puts("3", &Font_11x18, 1);
			  SSD1306_UpdateScreen();
			  osDelay(1000);

			  SSD1306_GotoXY(90, 35);
			  SSD1306_Puts("2", &Font_11x18, 1);
			  SSD1306_UpdateScreen();
			  osDelay(1000);

			  SSD1306_GotoXY(90, 35);
			  SSD1306_Puts("1", &Font_11x18, 1);
			  SSD1306_UpdateScreen();
			  osDelay(1000);

			  SSD1306_GotoXY(90, 35);
			  SSD1306_Puts("0", &Font_11x18, 1);
			  SSD1306_UpdateScreen();
			  osDelay(1000);
			  SSD1306_DrawFilledRectangle(0, 20, 128, 45, 0x00);
			  SSD1306_UpdateScreen();
		  }

		  if(INIT_WITH_LED == Flag_SSD_State)
		  {
			  // Open the hand 3 seconds

			  for(int i = 0; i < 6; i++)
			  {
				  HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_8); //Led Red
				  osDelay(500);
			  }

			 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
		  }

			HAL_ADC_Start(&hadc1);
			status = HAL_ADC_PollForConversion(&hadc1, 1);
			if(status == HAL_OK)
			{
#ifdef Medio
				analog_max[2] = HAL_ADC_GetValue(&hadc1);
#endif //Medio
			}

			status = HAL_ADC_PollForConversion(&hadc1, 1);
			if(status == HAL_OK)
			{
#ifdef Indice
				analog_max[1] = HAL_ADC_GetValue(&hadc1);
#endif //Indice
			}

			status = HAL_ADC_PollForConversion(&hadc1, 1);
			if(status == HAL_OK)
			{
#ifdef Menique
				analog_max[4] = HAL_ADC_GetValue(&hadc1);
#endif //Menique
			}

			status = HAL_ADC_PollForConversion(&hadc1, 1);
			if(status == HAL_OK)
			{
#ifdef Anular
				analog_max[3] = HAL_ADC_GetValue(&hadc1);
#endif //Anular
			}

			status = HAL_ADC_PollForConversion(&hadc1, 1);
			if(status == HAL_OK)
			{
#ifdef Pulgar
				analog_max[0] = HAL_ADC_GetValue(&hadc1);
#endif //Pulgar
			}

			HAL_ADC_Stop(&hadc1);



			if(CALIBRATION_OLED == Flag_SSD_State)
			{
				SSD1306_GotoXY(0, 30);
				SSD1306_Puts("Cierra la", &Font_7x10, 1);
				SSD1306_GotoXY(0, 50);
				SSD1306_Puts("mano", &Font_7x10, 1);
				SSD1306_UpdateScreen();

				SSD1306_GotoXY(90, 35);
				SSD1306_Puts("3", &Font_11x18, 1);
				SSD1306_UpdateScreen();
				osDelay(1000);

				SSD1306_GotoXY(90, 35);
				SSD1306_Puts("2", &Font_11x18, 1);
				SSD1306_UpdateScreen();
				osDelay(1000);

				SSD1306_GotoXY(90, 35);
				SSD1306_Puts("1", &Font_11x18, 1);
				SSD1306_UpdateScreen();
				osDelay(1000);

				SSD1306_GotoXY(90, 35);
				SSD1306_Puts("0", &Font_11x18, 1);
				SSD1306_UpdateScreen();
				osDelay(1000);
				SSD1306_DrawFilledRectangle(0, 20, 128, 45, 0x00);
				SSD1306_UpdateScreen();
			}

			if(INIT_WITH_LED == Flag_SSD_State)
			{
				//Close the hand 3 seconds

				for(int i = 0; i < 6; i++){
				  HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_1); //Led Green
				  osDelay(500);
				}

				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
			}

			HAL_ADC_Start(&hadc1);
			status = HAL_ADC_PollForConversion(&hadc1, 1);
			if(status == HAL_OK)
			{
#ifdef Medio
				analog_min[2] = HAL_ADC_GetValue(&hadc1);
#endif //Medio
			}

			status = HAL_ADC_PollForConversion(&hadc1, 1);
			if(status == HAL_OK)
			{
#ifdef Indice
				analog_min[1] = HAL_ADC_GetValue(&hadc1);
#endif //Indice
			}

			status = HAL_ADC_PollForConversion(&hadc1, 1);
			if(status == HAL_OK)
			{
#ifdef Menique
				analog_min[4] = HAL_ADC_GetValue(&hadc1);
#endif //Menique

			}

			status = HAL_ADC_PollForConversion(&hadc1, 1);
			if(status == HAL_OK)
			{
#ifdef Anular
				analog_min[3] = HAL_ADC_GetValue(&hadc1);
#endif //Anular
			}

			status = HAL_ADC_PollForConversion(&hadc1, 1);
			if(status == HAL_OK)
			{
#ifdef Pulgar
				analog_min[0] = HAL_ADC_GetValue(&hadc1);
#endif //Pulgar
			}

			HAL_ADC_Stop(&hadc1);
			if(
#ifdef Pulgar
			   analog_max[0] > analog_min[0]
#endif //Pulgar
#if defined(Pulgar) && (defined(Indice)||defined(Medio)||defined(Anular)||defined(Menique))
				&&
#endif
#ifdef Indice
			   analog_max[1] > analog_min[1]
#endif //Indice
#if defined(Indice) && (defined(Medio)||defined(Anular)||defined(Menique))
				&&
#endif
#ifdef Medio
			   analog_max[2] > analog_min[2]
#endif //Medio
#if defined(Medio) && (defined(Anular)||defined(Menique))
				&&
#endif
#ifdef Anular
			   analog_max[3] > analog_min[3]
#endif //Anular
#if defined(Anular) && defined(Menique)
				&&
#endif
#ifdef Menique
			   analog_max[4] > analog_min[4]
#endif //Menique
												)
			{

				osDelay(2000);

				if(CALIBRATION_OLED == Flag_SSD_State)
				{
					SSD1306_Clear();

					SSD1306_GotoXY(0, 0);
					SSD1306_Puts("CALIBRACION", &Font_11x18, 1);

					SSD1306_GotoXY(0, 30);
					SSD1306_Puts("Calibracion", &Font_7x10, 1);
					SSD1306_GotoXY(0, 50);
					SSD1306_Puts("Correcta", &Font_7x10, 1);
					SSD1306_UpdateScreen();
					osDelay(2000);
				}
				else if(INIT_WITH_LED == Flag_SSD_State)
				{
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
					osDelay(1000);
				}
				osDelay(1000);
				//xEventGroupClearBits(FlagHandle, 1);
				//xEventGroupSetBits(FlagHandle, 2); //envio de datos
			}
			else
			{
				if(CALIBRATION_OLED == Flag_SSD_State)
				{
					SSD1306_Clear();

					SSD1306_GotoXY(0, 0);
					SSD1306_Puts("CALIBRACION", &Font_11x18, 1);
					SSD1306_GotoXY(0, 30);
					SSD1306_Puts("Calibracion", &Font_7x10, 1);
					SSD1306_GotoXY(0, 50);
					SSD1306_Puts("Incorrecta", &Font_7x10, 1);
					SSD1306_UpdateScreen();

					osDelay(1000);
					SSD1306_Clear();

					SSD1306_GotoXY(0, 0);
					SSD1306_Puts("CALIBRACION", &Font_11x18, 1);
					SSD1306_GotoXY(0, 30);
					SSD1306_Puts("Pulsa para", &Font_7x10, 1);
					SSD1306_GotoXY(0, 50);
					SSD1306_Puts("Calibrar", &Font_7x10, 1);

					SSD1306_GotoXY(90, 35);
					SSD1306_Puts("!!", &Font_11x18, 1);

					SSD1306_UpdateScreen();
				}

				else if(INIT_WITH_LED == Flag_SSD_State)
				{
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
					osDelay(2000);
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
				}

				Calibration_fail = 1;
			}
		}

	 	osDelay(10);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_AnalogRead */
/**
* @brief Function implementing the Tarea2 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_AnalogRead */
void AnalogRead(void *argument)
{
  /* USER CODE BEGIN AnalogRead */
	HAL_StatusTypeDef status;
	struct Datos_Analogicos lectura;

	uint32_t datos[5];
	uint32_t datos_f[5] = {0,0,0,0,0};
	uint8_t grados_pack[5]={0,0,0,0,0};
  /* Infinite loop */
  for(;;)
  {
	  osEventFlagsWait(FlagHandle, 2U, osFlagsNoClear, osWaitForever);

	  if(RUNNING_L == Flag_SSD_State)
	  {
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET); //Led Green
	  }

	  HAL_ADC_Start(&hadc1);
		 status = HAL_ADC_PollForConversion(&hadc1, 5);
		 if(status == HAL_OK)
		 {
#ifdef Medio
			 datos[2] = HAL_ADC_GetValue(&hadc1);
#endif //Medio
		 }

		 status = HAL_ADC_PollForConversion(&hadc1, 5);
		 if(status == HAL_OK)
		 {
#ifdef Indice
			 datos[1] = HAL_ADC_GetValue(&hadc1);
#endif //Indice
		 }

		status = HAL_ADC_PollForConversion(&hadc1, 5);
		 if(status == HAL_OK)
		 {
#ifdef Menique
			 datos[4] = HAL_ADC_GetValue(&hadc1);
#endif //Menique
		 }

		status = HAL_ADC_PollForConversion(&hadc1, 5);
		 if(status == HAL_OK)
		 {
#ifdef Anular
			 datos[3] = HAL_ADC_GetValue(&hadc1);
#endif //Anular
		 }

		status = HAL_ADC_PollForConversion(&hadc1, 5);
		 if(status == HAL_OK)
		 {
#ifdef Pulgar
			 datos[0] = HAL_ADC_GetValue(&hadc1);
#endif //Pulgar
		 }

	 HAL_ADC_Stop(&hadc1);

	 //pulgar
	 //110 abierto, 50 cerrao
	 //tercer servo, 150 cerrao, 50 abierto

	 for(int k = 0; k<5; k++)
	 {
		 if(datos[k] > analog_max[k]){
			 datos[k] = analog_max[k];
		 }
		 else if(datos[k] < analog_min[k]){
			datos[k] = analog_min[k];
		 }

		 datos_f[k] = 0.3*datos[k] + 0.7*datos_f[k]; //filter

		 if(0 == k){
			 grados_pack[k] = 110-((datos_f[k]-analog_min[k])*(110-50))/(analog_max[k]-analog_min[k]+1); //el + 1 es para evitar dividir entre 0 en el caso de que algun dedo no funcione

			 if(grados_pack[k] > 110){
				 grados_pack[k] = 110;
			 }
			 else if(grados_pack[k] < 50){
				 grados_pack[k] = 50;
			 }
		 }
		 else{
		 	 grados_pack[k] = 180-((datos_f[k]-analog_min[k])*(180-25))/(analog_max[k]-analog_min[k]+1); //el + 1 es para evitar dividir entre 0 en el caso de que algun dedo no funcione


			 if(grados_pack[k] > 180){
				 grados_pack[k] = 180;
			 }
			 else if(grados_pack[k] < 25){
				 grados_pack[k] = 25;
			 }
		 }
	 }

#ifdef Pulgar
	 lectura.Analog_pulgar = grados_pack[0];
#endif //Pulgar
#ifdef Indice
	 lectura.Analog_indice = grados_pack[1];
#endif //Indice
#ifdef Medio
	 lectura.Analog_corazon = grados_pack[2];
#endif //Medio
#ifdef Anular
	 lectura.Analog_anular = grados_pack[3];
#endif //Anular
#ifdef Menique
	 lectura.Analog_menique = grados_pack[4];
#endif //Menique

#if defined(Pulgar)||defined(Indice)||defined(Medio)||defined(Anular)||defined(Menique)
	 osMessageQueuePut(Queue1Handle, &lectura, 0, 0);
#endif //defined(Pulgar)||defined(Indice)||defined(Medio)||defined(Anular)||defined(Menique)

	 osEventFlagsClear(FlagHandle,2U);
	 osEventFlagsSet(FlagHandle,4U);

	 osDelay(20);
  }
  /* USER CODE END AnalogRead */
}

/* USER CODE BEGIN Header_Gyro */
/**
* @brief Function implementing the Tarea3 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Gyro */
void Gyro(void *argument)
{
  /* USER CODE BEGIN Gyro */
#ifdef Gyroscope
	struct Datos_Gyro Inclinacion;
	struct AccelFunction datosA;
	struct GyroFunction datosG;
	float AnguloX, AnguloY;
	HAL_StatusTypeDef Flag_MPU6050;
#endif //Gyroscope

  /* Infinite loop */
  for(;;)
  {
	  osEventFlagsWait(FlagHandle, 4U, osFlagsNoClear, osWaitForever);
#ifdef Gyroscope
	  switch(Flag_Gyro)
	  {
	  case SEARCHING_GYRO_I2C_DEVICE:
		  Flag_MPU6050 = HAL_I2C_IsDeviceReady(&hi2c1, MPU6050_ADDR, 1, 1000);
		  if(HAL_OK == Flag_MPU6050)
		  {
			  Flag_Gyro = GYRO_IS_AVAILABLE;
		  }
		  else
		  {
			  Flag_Gyro = GYRO_IS_NOT_AVAILABLE;
		  }
		  break;

	  case GYRO_IS_AVAILABLE:
		  MPU6050_Init(hi2c1);
		  Flag_Gyro = RUNNING_GYRO;
		  break;

	  case GYRO_IS_NOT_AVAILABLE:
		  Flag_Gyro = RUNNING_WITHOUT_GYRO;
		  break;
	  case RUNNING_WITHOUT_GYRO:
	  case RUNNING_GYRO:
		  //nothing to do here
		  break;
	  }

	  if(RUNNING_GYRO == Flag_Gyro)
	  {

		datosG = MPU6050_Read_Gyro(hi2c1);
		datosA = MPU6050_Read_Accel(hi2c1);

		dt = count_gyro/1000;
		count_gyro = 0;

		//swaped scales from -90º/90º to 0º/180º
		AnguloX = 0.98*(AnguloX + datosG.Gx*dt) + 0.02*datosA.Ax;
		AnguloY = 0.98*(AnguloY + datosG.Gy*dt) + 0.02*datosA.Ay;

		Inclinacion.AnguloX = (uint8_t)(AnguloX + 90);
		Inclinacion.AnguloY = (uint8_t)(AnguloY + 90);

		osMessageQueuePut(Queue2Handle, &Inclinacion, 0, 0);
	  }
	  else if(RUNNING_WITHOUT_GYRO == Flag_Gyro)
	  {
		//if gyro doesn't work, angles set to 90º
		Inclinacion.AnguloX = 90;
		Inclinacion.AnguloY = 90;
	  }
		osMessageQueuePut(Queue2Handle, &Inclinacion, 0, 0);

#endif //Gyroscope
	osEventFlagsClear(FlagHandle,4U);
    osEventFlagsSet(FlagHandle,8U);

    osDelay(20);
  }
  /* USER CODE END Gyro */
}

/* USER CODE BEGIN Header_EnvioDatos */
/**
* @brief Function implementing the Tarea4 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_EnvioDatos */
void EnvioDatos(void *argument)
{
  /* USER CODE BEGIN EnvioDatos */
	int i = 0;

#if defined(Pulgar)||defined(Indice)||defined(Medio)||defined(Anular)||defined(Menique)
	osStatus_t val1;
	struct Datos_Analogicos buff1;
#endif //#if defined(Pulgar)||defined(Indice)||defined(Medio)||defined(Anular)||defined(Menique)

#ifdef Gyroscope
	osStatus_t val2;
	struct Datos_Gyro buff2;
#endif //Gyroscope

	//uint8_t BufferTx[9];
  /* Infinite loop */
  for(;;)
  {
    //xEventGroupWaitBits(FlagHandle,2,pdFALSE,pdFALSE,portMAX_DELAY);
    osEventFlagsWait(FlagHandle, 8U, osFlagsNoClear, osWaitForever);

    //DISPLAY
	  if(GYRO_IS_AVAILABLE == Flag_Gyro)
	  {
		  SSD1306_Clear();

		  SSD1306_GotoXY(0, 0);
		  SSD1306_Puts("CALIBRACION", &Font_11x18, 1);
		  SSD1306_GotoXY(0, 30);
		  SSD1306_Puts("GYRO", &Font_7x10, 1);
		  SSD1306_GotoXY(0, 50);
		  SSD1306_Puts("DISPONIBLE", &Font_7x10, 1);
		  SSD1306_UpdateScreen();
		  osDelay(3000);

		  SSD1306_Clear();
	  }
	  else if(GYRO_IS_NOT_AVAILABLE == Flag_Gyro)
	  {
		  SSD1306_Clear();

		  SSD1306_GotoXY(0, 0);
		  SSD1306_Puts("CALIBRACION", &Font_11x18, 1);
		  SSD1306_GotoXY(0, 30);
		  SSD1306_Puts("GYRO", &Font_7x10, 1);
		  SSD1306_GotoXY(0, 50);
		  SSD1306_Puts("NO DISPONIBLE", &Font_7x10, 1);
		  SSD1306_UpdateScreen();
		  osDelay(3000);

		  SSD1306_Clear();
	  }

	  if(RUNNING_O == Flag_SSD_State)
	  {
		  SSD1306_GotoXY(0, 0);
		  SSD1306_Puts("FUNCIONANDO", &Font_11x18, 1);

		  SSD1306_DrawFilledRectangle(0, 20, 128, 45, 0x00);
		  SSD1306_DrawBitmap(0, 20, Carga[i], 128, 45, 1);

		  SSD1306_UpdateScreen();
		  i++;

		  if(i == 12){
			  i = 0;
		 }
	  }

#if defined(Pulgar)||defined(Indice)||defined(Medio)||defined(Anular)||defined(Menique)
	val1 = osMessageQueueGet(Queue1Handle, &buff1, NULL, 0);
#endif // defined(Pulgar)||defined(Indice)||defined(Medio)||defined(Anular)||defined(Menique)
#ifdef Gyroscope
	val2 = osMessageQueueGet(Queue2Handle, &buff2, NULL, 0);
#endif //Gyroscope

	if(
#if defined(Pulgar)||defined(Indice)||defined(Medio)||defined(Anular)||defined(Menique)
	   (val1 == osOK)
#endif // defined(Pulgar)||defined(Indice)||defined(Medio)||defined(Anular)||defined(Menique)
#if (defined(Pulgar)||defined(Indice)||defined(Medio)||defined(Anular)||defined(Menique)) && defined(Gyroscope)
	   &&
#endif
#ifdef Gyroscope
	   (val2 == osOK)
#endif //Gyroscope
	   	   	   	     )
	{

		  BufferTx[0] = 200;
#ifdef Pulgar
		  BufferTx[1] = buff1.Analog_pulgar;
#endif //Puglar
#ifdef Indice
		  BufferTx[2] = buff1.Analog_indice;
#endif //Indice
#ifdef Medio
		  BufferTx[3] = buff1.Analog_corazon;
#endif //Medio
#ifdef Anular
		  BufferTx[4] = buff1.Analog_anular;
#endif //Anular
#ifdef Menique
		  BufferTx[5] = buff1.Analog_menique;
#endif //Menique

#ifdef Gyroscope
		  BufferTx[6] = buff2.AnguloX;
		  BufferTx[7] = buff2.AnguloY;
#endif //Gyroscope

		  BufferTx[8] = 0xA;

#ifdef Zigbee
		  HAL_UART_Transmit_IT(&huart1, BufferTx, 9);
#endif //Zigbee
	}
	osEventFlagsClear(FlagHandle,8U);
    osEventFlagsSet(FlagHandle,2U);

    osDelay(20);
  }
  /* USER CODE END EnvioDatos */
}

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
