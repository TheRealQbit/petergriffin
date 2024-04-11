/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define selection 4
#define MOTOR_A_IN1_PIN GPIO_PIN_6 // PC6
#define MOTOR_A_IN2_PIN GPIO_PIN_7 // PC7
#define MOTOR_B_IN3_PIN GPIO_PIN_8 // PC8
#define MOTOR_B_IN4_PIN GPIO_PIN_9 // PC9
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;

LCD_HandleTypeDef hlcd;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */
unsigned char SENSOR_1 = 0; // 0 for white 1 for black
unsigned char SENSOR_2 = 0;
unsigned char state;
unsigned int velocidades[selection + 1] = {0, 1023, 2047, 3071, 4095};
unsigned short valor = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC_Init(void);
static void MX_LCD_Init(void);
static void MX_TS_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void stopWheels(void) {
    // Stop both motors
    GPIOC->BSRR = (GPIO_PIN_6 | GPIO_PIN_8 ) << 16;
}
void moveForward(void) {
    // Move both motors forward
    GPIOC->BSRR = GPIO_PIN_6 | GPIO_PIN_8;
    GPIOC->BSRR = (GPIO_PIN_7 | GPIO_PIN_9) << 16; // Apagar el sentido contrario
}

void rightWheel(void) {
    // Turn right by moving right motor forward and left motor backward
    GPIOC->BSRR = GPIO_PIN_9;
    GPIOC->BSRR = GPIO_PIN_6;
    GPIOC->BSRR = (GPIO_PIN_7 | GPIO_PIN_8) << 16; // Apagar el sentido contrario
}

void leftWheel(void) {
    // Turn left by moving left motor forward and right motor backward
    GPIOC->BSRR = GPIO_PIN_7;
    GPIOC->BSRR = GPIO_PIN_8;
    GPIOC->BSRR = (GPIO_PIN_6 | GPIO_PIN_9) << 16; // Apagar el sentido contrario
}

void moveBackward(void) {
    // Move both motors backward
    GPIOC->BSRR = GPIO_PIN_7 | GPIO_PIN_9;
    GPIOC->BSRR = (GPIO_PIN_6 | GPIO_PIN_8) << 16;
}
void EXTI1_IRQHandler(void){  //for the right sensor
	if(EXTI -> PR == (1<<1)){
		SENSOR_1 = 1;
		EXTI -> PR |= (1<<1); // clean the flags with a 1
	}

}

void EXTI2_IRQHandler(void){  //for the left sensor
	if(EXTI -> PR == (1<<2)){
		SENSOR_2 = 1;
			EXTI -> PR |= (1<<2); //clean the flags with a 1
		}

}

void TIM4_IRQHandler(void){
	if((TIM4->SR & (1<<1))!=0){
	if(state == 0){
		state = 1;
	}
	else{
		state = 0;
	}

	TIM4->CCR1 += 250;

	TIM4->SR &= ~(1<<1);
}
}
void ADC1_IRQHandler(void) {
    if ((ADC1->SR & (1<<1)) != 0) {
        valor = ADC1->DR; // Lee el valor del ADC

    }
}


void ajustarVelocidad(unsigned short valor) {
	unsigned int velocidad = 0;
	for(unsigned int i = 0; i<selection; i++){
	    if(velocidades[i]<= valor && valor <=velocidades[i+1]){
	    	if (valor <= velocidades[1]) {
	    		        velocidad = 50;
	    		        TIM3->CCR2 = velocidad;
	    		        TIM3->CCR4 = velocidad;

	    		    } else if (valor <= velocidades[2]) {
	    		        velocidad = 70;
	    		        TIM3->CCR2 = velocidad;
	    		        TIM3->CCR4 = velocidad;

	    		    } else if (valor <= velocidades[3]) {
	    		        velocidad = 100;
	    		        TIM3->CCR2 = velocidad;
	    		        TIM3->CCR4 = velocidad;

	    		    }
	    }
	}
}
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
  MX_ADC_Init();
  MX_LCD_Init();
  MX_TS_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  GPIOC->MODER &= ~(1 << (6*2+1));
  GPIOC->MODER |= (1 << (6*2));

  GPIOC->MODER |= (1 << (7*2+1));
  GPIOC->MODER &= ~(1 << (7*2));
  GPIOC->AFR[0]=0x20000000;

  GPIOC->MODER &= ~(1<<(8*2+1));
  	GPIOC->MODER |= (1<<(8*2));


  GPIOC->MODER 	|=  (1<<(9*2+1));
  	GPIOC->MODER 	&= ~(1<<(9*2));
  	GPIOC->AFR[1] = 0x00000020; //TIM3 para el PC9 (ver apuntes GPIO)

    // PC1 & PC2 as digital input (00)
    GPIOC->MODER &= ~(1 << (1*2+1));
    GPIOC->MODER &= ~(1 << (1*2));

    GPIOC->MODER &= ~(1 << (2*2+1));
    GPIOC->MODER &= ~(1 << (2*2));

    GPIOA->MODER &= ~(1 << (0*2+1));
    GPIOA->MODER &= ~(1 << (0*2));
    //Configure the EXTI1
    //Hacemos la interrupcion del primer infrarrojo
    SYSCFG -> EXTICR[0] = 0;
    EXTI -> IMR |= (1<<1);
    EXTI -> RTSR |= (1<<1);
    EXTI -> FTSR |= (1<<1);
    NVIC->ISER[0] |= (1 << 7);  //EXTI1 posicion 7


     //Configure the EXTI2
    SYSCFG -> EXTICR[0] = 0;
    EXTI -> IMR |= (1<<2);
    EXTI -> RTSR |= (1<<2);
    EXTI -> FTSR |= (1<<2);
    NVIC->ISER[0] |= (1 << 8); //EXTI2 posicion 8


     //Configure buzzer
     GPIOB->MODER &= ~(1 << (8 * 2));
     GPIOB->MODER |= (1 << (8 * 2+1));
     //Configuration TIM4
     TIM4->CR1 = 0;
     TIM4->CR2 = 0;
     TIM4->SMCR = 0;

     TIM4->CNT = 0;
     TIM4->PSC = 31999;
     TIM4->ARR = 0xFFFF;
     TIM4->CCR1 = 250;

     TIM4->CCMR1 = 0;
     TIM4->CCMR2 = 0;
     TIM4->CCER = 0;

     TIM4->DIER = (1<<1);

     TIM4->EGR |= 0x0001;
     TIM4->SR = 0;
     TIM4->CR1 |= 0x0001;

     NVIC->ISER[0] |= (1 << 30);

     //PA5 as an input(00)
           GPIOA->MODER |= 0x00000C00;


       	ADC1 -> CR2 &= ~(0x00000001);//MAKE SURE THE POWER IS OFF
       	ADC1 -> CR1 = 0x00000020;
       	ADC1 -> CR2 = 0x00000412;
       	ADC1 -> SQR1 = 0x00000000;//I JUST WANT ONE CONVERSION
       	ADC1 -> SQR5 = 0x00000005;
       	ADC1 -> CR2 |= 0x00000001;//POWER ON
       	 while ((ADC1->SR&0x0040)==0); // If ADCONS = 0, I wait till converter is ready
       	 ADC1->CR2 |= 0x40000000;
       	 NVIC->ISER[0] |= (1<<18);
       	// PWM Configuration for pin PC6 and channel 1 of TIM3
       	TIM3->CR1 = 0x0000; // Disable TIM3
       	TIM3->CR2 = 0x0000; // Trigger mode configuration
       	TIM3->SMCR = 0; // Synchronization control configuration

       	TIM3->PSC = 319; // Prescaler configuration
       	TIM3->CNT = 0; // Initialize counter
       	TIM3->ARR = 99; // Auto-reload value configuration
       	TIM3->CCR2 = 1; // Duty cycle configuration (DC debe estar definido previamente)
       	TIM3->CCR4 = 1;

       	TIM3->DIER = 0x0000; // Disable interrupts
       	TIM3->DCR = 0;
       	//PC7 ch2

       	TIM3->CCMR1 |= (1<<(5*2+1)); // Clear channel 1 configuration bits
     	TIM3->CCMR1 |= (1<<(7*2));	//1
     	TIM3->CCMR1 |= (1<<(6*2+1));	//1
     	TIM3->CCMR1 &= ~(1<<(6*2));	//0
     	//PC9 ch4
     	TIM3->CCMR2 |= (1<<(5*2+1));	//PE
     	//Los 3 siguientes para PWM (ver manual)
     	TIM3->CCMR2 |= (1<<(7*2));	//1
     	TIM3->CCMR2 |= (1<<(6*2+1));	//1
     	TIM3->CCMR2 &= ~(1<<(6*2));	//0



     	TIM3->CCER |= (1<<(2*2));		//CC1E
     	TIM3->CCER |= (1<<(6*2));		//CC1E
     	TIM3->CR1 |= (1<<(3*2+1));		//HW (bit ARPE)
     	TIM3->EGR |= (1<<0);		//UG
     	TIM3->CR1 |= (1<<0);		//ON
     	TIM3->SR = 0;				//FLAG


     	TIM3->CCR2 = 20;
    	TIM3->CCR4 = 20;


         NVIC->ISER[0] |= (1 << 29);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	  if(((GPIOC->IDR&(1<<1))!=0) && ((GPIOC->IDR&(1<<2))!=0)){
	 	  	stopWheels();
	 	  	GPIOB->MODER = (1<<(8*2));

	 	  	  }
	 	  	  else if(((GPIOC->IDR&(1<<1))!=0) && ((GPIOC->IDR&(1<<2))==0)){
	 	  		  rightWheel();
	 	  		 if(state == 1){
	 	  			GPIOB->MODER = (1<<(8*2));
	 	  					 	  }
	 	  					 	  else{
	 	  					 		GPIOB->MODER = ~(1<<(8*2));
	 	  					 	  }

	 	  	  }
	 	  	  else if(((GPIOC->IDR&(1<<1))==0) && ((GPIOC->IDR&(1<<2))!=0)){

	 	  		  leftWheel();
	 	  		 if(state == 1){
	 	  			GPIOB->MODER = (1<<(8*2));
	 	  					 	  }
	 	  					 	  else{
	 	  					 		GPIOB->MODER = ~(1<<(8*2));
	 	  					 	  }
	 	  	  }
	 	  	  else if(((GPIOC->IDR&(1<<1))==0) && ((GPIOC->IDR&(1<<2))==0)){
	 	  		  moveForward();
	 	  		GPIOB->MODER = ~(1<<(8*2));
	 	  	  }
	 	  SENSOR_1=0;
	 	  SENSOR_2=0;
    /* USER CODE BEGIN 3 */
  }
}
  /* USER CODE END 3 */


/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLL_DIV3;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_LCD;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  PeriphClkInit.LCDClockSelection = RCC_RTCCLKSOURCE_LSE;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc.Init.LowPowerAutoWait = ADC_AUTOWAIT_DISABLE;
  hadc.Init.LowPowerAutoPowerOff = ADC_AUTOPOWEROFF_DISABLE;
  hadc.Init.ChannelsBank = ADC_CHANNELS_BANK_A;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.NbrOfConversion = 1;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T2_CC3;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc.Init.DMAContinuousRequests = DISABLE;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_4CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

}

/**
  * @brief LCD Initialization Function
  * @param None
  * @retval None
  */
static void MX_LCD_Init(void)
{

  /* USER CODE BEGIN LCD_Init 0 */

  /* USER CODE END LCD_Init 0 */

  /* USER CODE BEGIN LCD_Init 1 */

  /* USER CODE END LCD_Init 1 */
  hlcd.Instance = LCD;
  hlcd.Init.Prescaler = LCD_PRESCALER_1;
  hlcd.Init.Divider = LCD_DIVIDER_16;
  hlcd.Init.Duty = LCD_DUTY_1_4;
  hlcd.Init.Bias = LCD_BIAS_1_4;
  hlcd.Init.VoltageSource = LCD_VOLTAGESOURCE_INTERNAL;
  hlcd.Init.Contrast = LCD_CONTRASTLEVEL_0;
  hlcd.Init.DeadTime = LCD_DEADTIME_0;
  hlcd.Init.PulseOnDuration = LCD_PULSEONDURATION_0;
  hlcd.Init.MuxSegment = LCD_MUXSEGMENT_DISABLE;
  hlcd.Init.BlinkMode = LCD_BLINKMODE_OFF;
  hlcd.Init.BlinkFrequency = LCD_BLINKFREQUENCY_DIV8;
  if (HAL_LCD_Init(&hlcd) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LCD_Init 2 */

  /* USER CODE END LCD_Init 2 */

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

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
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

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TS Initialization Function
  * @param None
  * @retval None
  */
static void MX_TS_Init(void)
{

  /* USER CODE BEGIN TS_Init 0 */

  /* USER CODE END TS_Init 0 */

  /* USER CODE BEGIN TS_Init 1 */

  /* USER CODE END TS_Init 1 */
  /* USER CODE BEGIN TS_Init 2 */

  /* USER CODE END TS_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD4_Pin|LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
