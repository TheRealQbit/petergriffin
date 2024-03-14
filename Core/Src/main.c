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
#include "stm32l152c_discovery.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MOTOR_A_IN1_PIN GPIO_PIN_6 // PC6
#define MOTOR_A_IN2_PIN GPIO_PIN_7 // PC7
#define MOTOR_B_IN3_PIN GPIO_PIN_8 // PC8
#define MOTOR_B_IN4_PIN GPIO_PIN_9 // PC9
#define BUZZER_PIN GPIO_PIN_8 // PB8

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
unsigned char IR_SENSOR_1_STATE = 0; // 0 for white 1 for black
unsigned char IR_SENSOR_2_STATE = 0;
unsigned char state = 0; //0 for stop, 1 for forward, 2 for right, 3 for left, 4 for backward

ADC_HandleTypeDef hadc;

LCD_HandleTypeDef hlcd;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC_Init(void);
static void MX_LCD_Init(void);
static void MX_TS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void stopWheels(void) {
    // Stop both motors
    GPIOC->BSRR = (GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9) << 16;
}
void moveForward(void) {
    // Move both motors forward
    GPIOC->BSRR = GPIO_PIN_6 | GPIO_PIN_8;
    GPIOC->BSRR = (GPIO_PIN_7 | GPIO_PIN_9) << 16;
}

void turnRight(void) {
    // Turn right by moving right motor forward and left motor backward
    GPIOC->BSRR = GPIO_PIN_6 | GPIO_PIN_9;
    GPIOC->BSRR = (GPIO_PIN_7) << 16;
    GPIOC->BSRR = (GPIO_PIN_8) << 16;
}

void turnLeft(void) {
    // Turn left by moving left motor forward and right motor backward
    GPIOC->BSRR = GPIO_PIN_7 | GPIO_PIN_8;
    GPIOC->BSRR = (GPIO_PIN_6) << 16;
    GPIOC->BSRR = (GPIO_PIN_9) << 16;
}
void moveBackward(void) {
    // Move both motors backward
    GPIOC->BSRR = GPIO_PIN_7 | GPIO_PIN_9;
    GPIOC->BSRR = (GPIO_PIN_6 | GPIO_PIN_8) << 16;
}
void EXTI1_IRQHandler(void){
	if(EXTI->PR!=0){
		state++;
		if(state>3)state=0;
		EXTI->PR=0x01;
	}
}
void EXTI2_IRQHandler(void){
	if(EXTI->PR!=0){
		state++;
		if(state>3)state=0;
		EXTI->PR=0x01;
	}
}
void activateBuzzer(void) {
    //Activate the buzzer
    GPIOB->BSRR = BUZZER_PIN;
}
void deactivateBuzzer(void) {
    // Deactivate the buzzer
    GPIOB->BSRR = BUZZER_PIN << 16;
}
void TIM4_IRQHandler(void) {
if ((TIM4->SR & 0x0002)!=0)
{
numero++; // Increase in 1 the number to be shown in the LCD
TIM4->CCR1 += tiempo; // Update the comparison value, adding 1000 steps = 1 second
TIM4->SR = 0x0000; // Clear all flags
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
  /* USER CODE BEGIN 2 */
  // PC6, PC7, PC8, and PC9 as digital outputs (01)
  GPIOC->MODER &= ~(1 << (6*2+1));
  GPIOC->MODER |= (1 << (6*2));
  GPIOC->MODER &= ~(1 << (7*2+1));
  GPIOC->MODER |= (1 << (7*2));
  GPIOC->MODER &= ~(1 << (8*2+1));
  GPIOC->MODER |= (1 << (8*2));
  GPIOC->MODER &= ~(1 << (9*2+1));
  GPIOC->MODER |= (1 << (9*2));

  // PC1 & PC2 as digital input (00)
  GPIOC->MODER &= ~(1 << (1*2+1));
  GPIOC->MODER &= ~(1 << (1*2));
  GPIOC->MODER &= ~(1 << (2*2+1));
  GPIOC->MODER &= ~(1 << (2*2));

  //Configure the EXTI1
  EXTI->FTSR |= 0x01;                      //Set off the falling edge trigger
  EXTI->RTSR &= ~(0x01);                   //Set on the rising edge trigger
  SYSCFG->EXTICR[0] = 0;                   //Set the EXTI1 to PC1
  EXTI->IMR |= 0x01;                       //Unmask the EXTI1
  NVIC->ISER[0] |= (1 << 7);

  //Configure the EXTI2
  EXTI->FTSR |= 0x0004;                      //Set off the falling edge trigger
  EXTI->RTSR |= 0x0004;                      //Set on the rising edge trigger
  SYSCFG->EXTICR[0] &= 0xF0FF;               //Set the EXTI2 to PC2
  SYSCFG->EXTICR[0] |= 0x0200;               // Set PC2 as EXTI2
  EXTI->IMR |= 0x0004;                       //Unmask the EXTI2
  NVIC->ISER[0] |= (1 << 8);

  //PC8 as digital output(01)
  GPIOB->MODER &= ~(1 << (8*2 +1));
  GPIOB->MODER |= (1 << (8*2));

  //Config buzzer
  GPIOB->MODER &= ~(0x03 << (8 * 2)); // Limpiar los bits
  GPIOB->MODER |= (0x01 << (8 * 2)); // Configurar como salida

  TIM4->PSC = 31999;
  TIM4->ARR = 0xffff;
  TIM4->CCMR1 = 0x0000;
  TIM4->CNT = 0;
  TIM4->CCR1 = 1000;
  TIM4->DIER = 0x0000;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	if (IR_SENSOR_1_STATE == 1 || IR_SENSOR_2_STATE == 1) {
	        activateBuzzer();
	    } else {
	        deactivateBuzzer();
	    }
    if (IR_SENSOR_1_STATE == 1 && IR_SENSOR_2_STATE == 1) {
        state = 0;
    } else if (IR_SENSOR_1_STATE == 0 && IR_SENSOR_2_STATE == 1) {
        state = 3;
    } else if (IR_SENSOR_1_STATE == 1 && IR_SENSOR_2_STATE == 0) {
        state = 2;
    } else if (IR_SENSOR_1_STATE == 0 && IR_SENSOR_2_STATE == 0) {
        state = 1;
    } else {
        state = 1;
    }
      switch (state) {
          case 0:
              moveForward();
              break;
          case 1:
              stopWheels();
              break;
          case 2:
              turnLeft();
              break;
          case 3:
              turnRight();
              break;
          default:
              stopWheels();
              break;
      }
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
  sConfig.Channel = ADC_CHANNEL_4;
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
