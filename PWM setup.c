/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : PWM setup.c
  * @brief          : Initial setup and testing of PWM functionality using TIM2 and TIM3
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

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
  /* Configure the system clock */
  SystemClock_Config();
	
	// --------------- LED SETUP -------------------
	__HAL_RCC_GPIOC_CLK_ENABLE(); // Enable the GPIOC clock in the RCC
	GPIO_InitTypeDef initStr = {GPIO_PIN_8 | GPIO_PIN_9,
															GPIO_MODE_OUTPUT_PP,
															GPIO_SPEED_FREQ_LOW,
															GPIO_NOPULL};
	//GPIO_InitTypeDef initStr2 = {GPIO_PIN_6 | GPIO_PIN_7, GPIO_MODE_AF_PP,GPIO_SPEED_FREQ_LOW,GPIO_NOPULL};
	
	HAL_GPIO_Init(GPIOC, &initStr); // Initialize pins PC8, PC9 
	//HAL_GPIO_Init(GPIOC, &initStr2);
															
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET); // Start PC9 high
													

  RCC->APB1ENR |= RCC_APB1ENR_TIM2EN; // Enable TIM2 and TIM3
  RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
  
  // Exersize 3.1
	//TIM2 -> PSC = (0x1F3F); // 7999 (8000)
	//TIM2 -> PSC = (0x173F);														
	//TIM2 -> ARR = (0x1C2); // 250 -> 8000*250 = 2,000,000 desired f = 4Hz
	TIM2 -> PSC  = (0x173F);
	TIM2 -> ARR  = (0x1C2);
	TIM2 -> DIER |= (1);
	TIM2 -> EGR |= (1);
									
	NVIC_EnableIRQ(TIM2_IRQn);
	NVIC_SetPriority(TIM2_IRQn, 2);
															
	TIM2 -> CR1 |= (1); // timer enable
	
	
	
	// Exersize 3.2
	TIM3 -> PSC = (20); 														
	TIM3 -> ARR = (20); //100*100 = 10,000 desired f = 800Hz		
  
		
	
	TIM3 -> CCMR1 |= (0b00 << 8);
	TIM3 -> CCMR1 |= (0b00);	     // set to output mode 
	TIM3 -> CCMR1 |= (7 << 4);   // set OC1M to PWM mode 2	
  TIM3 -> CCMR1 |= (6 << 12);   // set OC2M to PWM mode 1	
	TIM3 -> CCMR1 |= (1 << 3) | (1 << 11); // enable channel 1 & 2 preload
	TIM3 -> CCER |= (1); // channel 1 output enable 
	TIM3 -> CCER |= (1 << 4); // channel 2 output enable 
	TIM3 -> CCR1 = (20); // set CCR channels to 20% of 100 = 20
	TIM3 -> CCR2 = (20);
	
	//PC6 AF0 = TIM3_CH1 
	//PC7 AF0 = TIM3_CH2
	GPIOC -> MODER |= (0b10 << 12); // set PC6 to AF mode
	GPIOC -> MODER |= (0b10 << 14); // set PC7 to AF mode
	GPIOC -> MODER |= (0x2 << 12); // set PC6 to AF mode
	GPIOC -> MODER |= (0x2 << 14); // set PC7 to AF mode
	
	GPIOC -> AFR[0] &= ~(15 << 24);  
	GPIOC -> AFR[0] |= (0b0000 << 24); // set PC6 to AF0
	GPIOC -> AFR[0] &= ~(15 << 28);
	GPIOC -> AFR[0] |= (0b0000 << 28); // set PC7 to AF0
	
	TIM3 -> CR1 |= (1); // timer enable
	
	
	
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		
  }
  /* USER CODE END 3 */
}


void TIM2_IRQHandler(void){
	TIM2->SR = 0;
	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_8);
	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_9);
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
