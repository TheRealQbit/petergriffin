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
#define MOTOR_A_IN1_PIN GPIO_PIN_6 // PC6
#define MOTOR_A_IN2_PIN GPIO_PIN_7 // PC7
#define MOTOR_B_IN3_PIN GPIO_PIN_8 // PC8
#define MOTOR_B_IN4_PIN GPIO_PIN_9 // PC9
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
ADC_HandleTypeDef hadc;
LCD_HandleTypeDef hlcd;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC_Init(void);
static void MX_LCD_Init(void);
static void MX_TS_Init(void);
/* USER CODE BEGIN PFP */
void stopWheels(void);
void moveForward(void);
void turnRight(void);
void turnLeft(void);
void moveBackward(void);
void espera(int tiempo);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void moveForward(void) {
    // Move both motors forward
    GPIOC->BSRR = GPIO_PIN_6 | GPIO_PIN_8;
    GPIOC->BSRR = (GPIO_PIN_7 | GPIO_PIN_9) << 16;
}
void turnRight(void) {
    // Turn right by moving right motor forward and left motor backward
    GPIOC->BSRR = GPIO_PIN_6 | GPIO_PIN_9;
    GPIOC->BSRR = (GPIO_PIN_7 | GPIO_PIN_8) << 16;
}
void turnLeft(void) {
    // Turn left by moving left motor forward and right motor backward
    GPIOC->BSRR = GPIO_PIN_7 | GPIO_PIN_8;
    GPIOC->BSRR = (GPIO_PIN_6 | GPIO_PIN_9) << 16;
}
void moveBackward(void) {
    // Move both motors backward
    GPIOC->BSRR = GPIO_PIN_7 | GPIO_PIN_9;
    GPIOC->BSRR = (GPIO_PIN_6 | GPIO_PIN_8) << 16;
}
void espera(int tiempo) {
    for (int i = 0; i < tiempo * 1000; i++) {
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
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
      // Stop both wheels for 1 second
      stopWheels();
      espera(1000);
      // Move both wheels forward for 2 seconds
      moveForward();
      espera(2000);
      // Stop both wheels for 1 second
      stopWheels();
      espera(1000);
      // Turn right for 2 seconds
      turnRight();
      espera(2000);
      // Stop both wheels for 1 second
      stopWheels();
      espera(1000);
      // Turn left for 2 seconds
      turnLeft();
      espera(2000);
      // Stop both wheels for 1 second
      stopWheels();
      espera(1000);
      // Move both wheels backward for 2 seconds
      moveBackward();
      espera(2000);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
