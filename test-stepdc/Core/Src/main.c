/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "i2c-lcd.h"
#include "delay.h"
#include "stdio.h"
#include "math.h"
#include "string.h"
#include "stdlib.h"
#include "stdbool.h"
#include "ESPDataLogger.h"
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

I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_I2C2_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define IN1Port GPIOA             // khai báo chân điều khiển động cơ
#define IN1Pin  GPIO_PIN_0
#define IN2Port GPIOA
#define IN2Pin  GPIO_PIN_1
#define IN3Port GPIOA
#define IN3Pin  GPIO_PIN_2
#define IN4Port GPIOA
#define IN4Pin  GPIO_PIN_3

#define IN5Port GPIOB
#define IN5Pin  GPIO_PIN_12
#define IN6Port GPIOB
#define IN6Pin  GPIO_PIN_13
#define IN7Port GPIOB
#define IN7Pin  GPIO_PIN_14
#define IN8Port GPIOB
#define IN8Pin  GPIO_PIN_15


#define stepsperrev 2048    // số bước quay 1 vòng

void delay(uint16_t us){
	__HAL_TIM_SET_COUNTER(&htim1,0);
	while(__HAL_TIM_GET_COUNTER(&htim1)<us);
}

void SetPin (GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_SET);
}

void ResetPin (GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET);
}

void stepper_wave_drive (int step)  // hàm điều khiển động cơ quay theo bước
{
	switch (step){
		case 0:
			  SetPin(IN1Port, IN1Pin);   // IN1 SET
			  ResetPin(IN2Port, IN2Pin);   // IN2 RESET
			  ResetPin(IN3Port, IN3Pin);   // IN3 RESET
			  ResetPin(IN4Port, IN4Pin);   // IN4 RESET
			  break;

		case 1:
			  ResetPin(IN1Port, IN1Pin);   // IN1 RESET
			  SetPin(IN2Port, IN2Pin);   // IN2 SET
			  ResetPin(IN3Port, IN3Pin);   // IN3 RESET
			  ResetPin(IN4Port, IN4Pin);   // IN4 RESET
			  break;

		case 2:
			  ResetPin(IN1Port, IN1Pin);   // IN1 RESET
			  ResetPin(IN2Port, IN2Pin);   // IN2 RESET
			  SetPin(IN3Port, IN3Pin);   // IN3 SET
			  ResetPin(IN4Port, IN4Pin);   // IN4 RESET
			  break;

		case 3:
			  ResetPin(IN1Port, IN1Pin);   // IN1 RESET
			  ResetPin(IN2Port, IN2Pin);   // IN2 RESET
			  ResetPin(IN3Port, IN3Pin);   // IN3 RESET
			  SetPin(IN4Port, IN4Pin);   // IN4 SET

		}
}
void stepper_wave_drive1 (int step)   //hàm điều khiển động cơ quay theo bước
{
	switch (step){
		case 0:
			  SetPin  (IN5Port, IN5Pin);
			  ResetPin(IN6Port, IN6Pin);
			  ResetPin(IN7Port, IN7Pin);
			  ResetPin(IN8Port, IN8Pin);
			  break;

		case 1:
			  ResetPin(IN5Port, IN5Pin);
			  SetPin  (IN6Port, IN6Pin);
			  ResetPin(IN7Port, IN7Pin);
			  ResetPin(IN8Port, IN8Pin);
			  break;

		case 2:
			  ResetPin(IN5Port, IN5Pin);
			  ResetPin(IN6Port, IN6Pin);
			  SetPin  (IN7Port, IN7Pin);
			  ResetPin(IN8Port, IN8Pin);
			  break;

		case 3:
			  ResetPin(IN5Port, IN5Pin);
			  ResetPin(IN6Port, IN6Pin);
			  ResetPin(IN7Port, IN7Pin);
			  SetPin  (IN8Port, IN8Pin);

		}
}
void stepper_set_rpm (int rpm)         // set độ trễ của mỗi bước( hay tốc độ quay của động cơ)
{
	delay(60000000/stepsperrev/rpm);
}

void stepper_step_angle (int angle, int direction, int rpm) // hàm điều khiển động cơ quay theo góc
{
	float anglepersequence = 0.703125;  // 360 = 512 sequences
	int numberofsequences = (int) (angle/anglepersequence);

	for (int seq=0; seq<numberofsequences; seq++)
	{
		if (direction == 0)  // for clockwise
		{
			for (int step=3; step>=0; step--)
			{
				stepper_wave_drive(step);
				stepper_set_rpm(rpm);
			}

		}

		else if (direction == 1)  // for anti-clockwise
		{
			for (int step=0; step<4; step++)
			{
				stepper_wave_drive(step);
				stepper_set_rpm(rpm);
			}
		}
	}
}
void stepper_step_angle1 (int angle, int direction, int rpm) 		// hàm điều khiển động cơ quay theo góc
{
	float anglepersequence = 0.703125;  // 360 = 512 sequences
	int numberofsequences = (int) (angle/anglepersequence);

	for (int seq=0; seq<numberofsequences; seq++)
	{
		if (direction == 0)  // for clockwise
		{
			for (int step=3; step>=0; step--)
			{
				stepper_wave_drive1(step);
				stepper_set_rpm(rpm);
			}

		}

		else if (direction == 1)  // for anti-clockwise
		{
			for (int step=0; step<4; step++)
			{
				stepper_wave_drive1(step);
				stepper_set_rpm(rpm);
			}
		}
	}
}
float currentAngle =0;
float angle;

void Stepper_rotate (int angle, int rpm)          // tính góc động cơ đã quay
{
	int changeinangle = 0;
	changeinangle = angle-currentAngle;
	if (changeinangle > 1)  // clockwise
	{
		stepper_step_angle (changeinangle,0,rpm);
		currentAngle = angle;
	}
	else if (changeinangle <-1)//
	{
		changeinangle = -(changeinangle);
		stepper_step_angle (changeinangle,1,rpm);
		currentAngle = angle;
	}

}


uint16_t adc_ldr7 =0;
float V_ldr7=0;
uint16_t adc_ldr8 =0;
float V_ldr8=0;
uint16_t adc_ldr9 =0;
float V_ldr9=0;
char buff7[16];
char buff8[16];
char buff9[16];
uint8_t rx[15]={0};
uint8_t data[15]="\nhello kien";
uint8_t flag=0;


//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//{
//  UNUSED(huart);
//  if(huart->Instance== USART1){
//
//	  HAL_UART_Receive_IT(huart, rx, sizeof(rx));
//  }
//}

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
  MX_TIM1_Init();
  MX_I2C2_Init();
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  lcd_init();
 // HAL_UART_Receive_IT(&huart1, rx, sizeof(rx));
  HAL_TIM_Base_Start(&htim1);  // khởi tạo hàm-=timer

  ESP_Init("iPhone","12345678");
  uint32_t time=HAL_GetTick();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  //HAL_UART_Transmit(&huart1, data, sizeof(data), 100);

	  HAL_ADC_Start(&hadc1);                   // đọc giá trị ldr
	  HAL_ADC_PollForConversion(&hadc1, 300);
	  adc_ldr7 =HAL_ADC_GetValue(&hadc1);
	  V_ldr7=(float)((adc_ldr7/4095.00)*5);

	   if (HAL_GetTick() - time >= 20000){     // gửi dữ liệu lên cloud sau mỗi 20s

		   ESP_Send_Data("ETVL3RB9PHFN259M", 1, adc_ldr7);
		   time = HAL_GetTick();
	   }
	  HAL_ADC_PollForConversion(&hadc1, 300);
	  adc_ldr8 =HAL_ADC_GetValue(&hadc1);
	  V_ldr8=(float)((adc_ldr8/4095.00)*5);
	  HAL_ADC_PollForConversion(&hadc1, 300);
	  adc_ldr9 =HAL_ADC_GetValue(&hadc1);
	  V_ldr9=(float)((adc_ldr9/4095.00)*5);
	  sprintf(buff7,"adc7_value: %d",adc_ldr7);
   	  sprintf(buff8,"adc8_value: %d",adc_ldr8);
	  sprintf(buff9,"adc9_value: %d",adc_ldr9);
      lcd_put_cur(0, 0);						// hiển thi lcd
	  lcd_send_string(buff7);
	  lcd_put_cur(1, 0);
	  lcd_send_string(buff8);
	  HAL_ADC_Stop(&hadc1);


// điều khiển động cơ
		  if(adc_ldr7 > adc_ldr8 && (adc_ldr7- adc_ldr8)>100){
			  stepper_step_angle(18, 0,13);
		  }
		  else if(adc_ldr8 > adc_ldr7 && (adc_ldr8- adc_ldr7)>100){
			  stepper_step_angle(18, 1, 13);
		  }


		  if(adc_ldr7 > adc_ldr9 && (adc_ldr7- adc_ldr9)>100){
			  stepper_step_angle1(18, 0,13);
		  }
		  else if(adc_ldr9 > adc_ldr7 && (adc_ldr9- adc_ldr7)>100){
		      stepper_step_angle1(18, 1, 13);
		  }

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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
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

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 3;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

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

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 72-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 0xffff-1;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA0 PA1 PA2 PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB12 PB13 PB14 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
