/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include "LIB_MPU6500_SPI.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
char bufferTxt[30];
MPU6500_Init_Values_t 	MPU6500_Datos; //Iniciamos donde se guardaran todos los datos a leer
MPU6500_status_e	MPU6500_Status;
MPU6500_Init_float_t	MPU6500_Conv;
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
  MX_SPI3_Init();
  MX_USART3_UART_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

  MPU6500_Status=MPU6500_Init(&MPU6500_Datos,10,DPS250,G4);
  if (MPU6500_Status==MPU6500_fail) {
  	for (;;) {
  		 sprintf(bufferTxt,"Fallo al iniciar MPU\r\n");
  				  HAL_UART_Transmit(&huart3, (uint8_t *)bufferTxt, strlen(bufferTxt), HAL_MAX_DELAY);

  		}
  }
  sprintf(bufferTxt," Exito al iniciar MPU\r\n");
 		  HAL_UART_Transmit(&huart3, (uint8_t *)bufferTxt, strlen(bufferTxt), HAL_MAX_DELAY);
 HAL_ADC_Start(&hadc1);

  HAL_Delay(2000);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  uint16_t adc_val=0;
	  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_4);
	  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_5);
	  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);
	  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_1);
	  	if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK)
	  	    {
	  		 adc_val=HAL_ADC_GetValue(&hadc1);
	  		sprintf(bufferTxt," adc= %d ",adc_val);
	  		HAL_UART_Transmit(&huart3, (uint8_t *)bufferTxt, strlen(bufferTxt), HAL_MAX_DELAY);
	  		HAL_ADC_Start(&hadc1);
	  	    }
	  MPU6500_Read(&MPU6500_Datos);
	  MPU6500_Conv=MPU6500_Converter(&MPU6500_Datos, DPS250_CONV, G4_CONV);
	sprintf(bufferTxt," Gx= %.2f ",MPU6500_Conv.MPU6500_floatGX);
	HAL_UART_Transmit(&huart3, (uint8_t *)bufferTxt, strlen(bufferTxt), HAL_MAX_DELAY);

	sprintf(bufferTxt," Gy= %.2f ",MPU6500_Conv.MPU6500_floatGY);
	HAL_UART_Transmit(&huart3, (uint8_t *)bufferTxt, strlen(bufferTxt), HAL_MAX_DELAY);

	sprintf(bufferTxt," Gz= %.2f ",MPU6500_Conv.MPU6500_floatGZ);
	HAL_UART_Transmit(&huart3, (uint8_t *)bufferTxt, strlen(bufferTxt), HAL_MAX_DELAY);

	sprintf(bufferTxt," Ax= %.2f ",MPU6500_Conv.MPU6500_floatAX);
	HAL_UART_Transmit(&huart3, (uint8_t *)bufferTxt, strlen(bufferTxt), HAL_MAX_DELAY);

	sprintf(bufferTxt," Ay= %.2f ",MPU6500_Conv.MPU6500_floatAY);
	HAL_UART_Transmit(&huart3, (uint8_t *)bufferTxt, strlen(bufferTxt), HAL_MAX_DELAY);

	sprintf(bufferTxt," Az= %.2f \r\n",MPU6500_Conv.MPU6500_floatAZ);
	HAL_UART_Transmit(&huart3, (uint8_t *)bufferTxt, strlen(bufferTxt), HAL_MAX_DELAY);

	HAL_Delay(100);
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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
#ifdef USE_FULL_ASSERT
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
