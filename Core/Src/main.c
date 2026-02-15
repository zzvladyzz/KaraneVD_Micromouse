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
#include "tim.h"
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

/* Variables para los encoders*/
const int8_t estadoTabla[16]={0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0}; //valor encoders de tabla de verdad
uint8_t estadoAnterior_L=0;
uint8_t estadoAnterior_R=0;
int32_t ticksD=0;
int32_t ticksI=0;
int32_t contD=0;
int32_t contI=0;


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

uint32_t ADC_Read_Manual(ADC_HandleTypeDef *hadc, uint32_t channel) {
    ADC_ChannelConfTypeDef sConfig = {0};
    uint32_t result = 0;

    // 1. Configurar el canal específico en la posición 1
    sConfig.Channel = channel;
    sConfig.Rank = 1;
    // Aumenta el tiempo de muestreo si lees 0 o valores inestables
    sConfig.SamplingTime = ADC_SAMPLETIME_56CYCLES;

    // 2. Aplicar configuración
    if (HAL_ADC_ConfigChannel(hadc, &sConfig) != HAL_OK) {
        return 0; // Error de configuración
    }

    // 3. Ciclo de conversión
    HAL_ADC_Start(hadc);

    // Esperar con un timeout prudente (10ms es suficiente)
    if (HAL_ADC_PollForConversion(hadc, 10) == HAL_OK) {
        result = HAL_ADC_GetValue(hadc);
    }

    // 4. Parar el ADC para liberar el secuenciador
    HAL_ADC_Stop(hadc);

    return result;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{

	if(GPIO_Pin==Enc_D_A_Pin||GPIO_Pin==Enc_D_B_Pin)
		{
		uint8_t bitStatusL=((HAL_GPIO_ReadPin(Enc_D_A_GPIO_Port, Enc_D_A_Pin))?2:0) | ((HAL_GPIO_ReadPin(Enc_D_B_GPIO_Port, Enc_D_B_Pin))?1:0);
		ticksD+=(-estadoTabla[((estadoAnterior_L<<2)|bitStatusL)]);
		estadoAnterior_L=bitStatusL;
		}
	if(GPIO_Pin==Enc_I_A_Pin||GPIO_Pin==Enc_I_B_Pin)
		{
		uint8_t bitStatusR=((HAL_GPIO_ReadPin(Enc_I_A_GPIO_Port, Enc_I_A_Pin))?2:0) | ((HAL_GPIO_ReadPin(Enc_I_B_GPIO_Port, Enc_I_B_Pin))?1:0);
		ticksI+=(estadoTabla[((estadoAnterior_R<<2)|bitStatusR)]);
		estadoAnterior_R=bitStatusR;
		}
	if(GPIO_Pin==Enc_D_C_Pin)
	{
		contD++;
	}
	if(GPIO_Pin==Enc_I_C_Pin)
		{
			contI++;
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
  MX_SPI3_Init();
  MX_USART3_UART_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

  __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,00);
  __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,00);
  __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,00);
  __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,00);

  __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_4,00);

  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5,GPIO_PIN_SET);
  HAL_Delay(1000);

  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5,GPIO_PIN_RESET);

  MPU6500_Status=MPU6500_Init(&MPU6500_Datos,10,DPS250,G2);
  /*if (MPU6500_Status==MPU6500_fail) {
  	for (;;) {
  		 sprintf(bufferTxt,"Fallo al iniciar MPU\r\n");
  				  HAL_UART_Transmit(&huart3, (uint8_t *)bufferTxt, strlen(bufferTxt), HAL_MAX_DELAY);
  				HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_5);
  				HAL_Delay(500);
  		}
  }*/
  sprintf(bufferTxt," Exito al iniciar MPU\r\n");
 		  HAL_UART_Transmit(&huart3, (uint8_t *)bufferTxt, strlen(bufferTxt), HAL_MAX_DELAY);
 HAL_ADC_Start(&hadc1);

  HAL_Delay(1000);
uint16_t xxx=0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_4,xxx);
	  xxx=xxx+8;
	  if(xxx>200)
	  {
		  xxx=0;
		  //HAL_Delay(4000);
	  }
	  HAL_Delay(500);
	  HAL_GPIO_WritePin(MOTOR_EN_GPIO_Port, MOTOR_EN_Pin, GPIO_PIN_RESET);
	  //HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_4);
	  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_5);
	  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);
	  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_1);
	  __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,50);
	    __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,100);
	    __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,50);
	    __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,100);

	  MPU6500_Read(&MPU6500_Datos);
	  MPU6500_Conv=MPU6500_Converter(&MPU6500_Datos, DPS250_CONV, G2_CONV);

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



	uint16_t adc2_val=00;
/*

	HAL_ADC_Start(&hadc1);

	    // Esperar con un timeout prudente (10ms es suficiente)
	    if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK) {
	        adc2_val = HAL_ADC_GetValue(&hadc1);

		    // 4. Parar el ADC para liberar el secuenciador
		    HAL_ADC_Stop(&hadc1);
	    }*/
	adc2_val=(uint16_t)ADC_Read_Manual(&hadc1, 7);
	if(adc2_val<4010 && adc2_val>4000)
	{
		ticksD=0;
		ticksI=0;
		contD=0;
		contI=0;
	}
	sprintf(bufferTxt," Ad0 boton= %d ",adc2_val);
	HAL_UART_Transmit(&huart3, (uint8_t *)bufferTxt, strlen(bufferTxt), HAL_MAX_DELAY);

	HAL_GPIO_WritePin(IR1_TX_GPIO_Port, IR1_TX_Pin, GPIO_PIN_SET);
	HAL_Delay(100);
	adc2_val=(uint16_t)ADC_Read_Manual(&hadc2, 1);
	HAL_Delay(5);

	sprintf(bufferTxt," ir1= %d ",adc2_val);
	HAL_UART_Transmit(&huart3, (uint8_t *)bufferTxt, strlen(bufferTxt), HAL_MAX_DELAY);
	HAL_GPIO_WritePin(IR1_TX_GPIO_Port, IR1_TX_Pin, GPIO_PIN_RESET);

	adc2_val=(uint16_t)ADC_Read_Manual(&hadc2, 2);
	sprintf(bufferTxt," Adc2= %d ",adc2_val);
	HAL_UART_Transmit(&huart3, (uint8_t *)bufferTxt, strlen(bufferTxt), HAL_MAX_DELAY);


	adc2_val=(uint16_t)ADC_Read_Manual(&hadc2, 3);
	sprintf(bufferTxt," Adc3= %d ",adc2_val);
	HAL_UART_Transmit(&huart3, (uint8_t *)bufferTxt, strlen(bufferTxt), HAL_MAX_DELAY);


	adc2_val=(uint16_t)ADC_Read_Manual(&hadc2, 4);
	sprintf(bufferTxt," Adc= %d \r\n",adc2_val);


	HAL_UART_Transmit(&huart3, (uint8_t *)bufferTxt, strlen(bufferTxt), HAL_MAX_DELAY);

	sprintf(bufferTxt," I= %ld ",ticksI);
	HAL_UART_Transmit(&huart3, (uint8_t *)bufferTxt, strlen(bufferTxt), HAL_MAX_DELAY);

	sprintf(bufferTxt," contI= %ld ",contI);
	HAL_UART_Transmit(&huart3, (uint8_t *)bufferTxt, strlen(bufferTxt), HAL_MAX_DELAY);
		sprintf(bufferTxt," D= %ld ",ticksD);
		HAL_UART_Transmit(&huart3, (uint8_t *)bufferTxt, strlen(bufferTxt), HAL_MAX_DELAY);

		sprintf(bufferTxt," contD= %ld \r\n",contD);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
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
