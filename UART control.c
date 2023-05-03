/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : UART_control.c
  * @brief          : UART bluetooth input to robot, able to toggle Forward, Left, Right, Back, and Stop mode with w,a,d,s,x laptop keystrokes
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



void SystemClock_Config(void);

void usart_char_transmit(char c);
void usart_str_transmit(char *c);
void USART3_4_IRQHandler(void);
volatile char _RX_dir, _RX_flag, _RX_idle = 0;
volatile char start = 1;
volatile char curr_dir = 0;
char *err = "error\n";


/**
  * @brief  The application entry point.
  * @retval int
	
  */
int main(void)
{
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
	__HAL_RCC_USART3_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE(); 
	
	//USART PINS: GPIOB 10 & 11
	/*
	GPIO_InitTypeDef initStr = {GPIO_PIN_10 | GPIO_PIN_11,
															GPIO_MODE_AF_PP,
															GPIO_SPEED_FREQ_LOW,
															GPIO_NOPULL};
	HAL_GPIO_Init(GPIOB, &initStr);														
	GPIOB->AFR[1] |= ((4 << 8) | (4 << 12));
	*/
	
	//USART PINS: GPIOC 4 & 5
	//PC4 = TX, PC5 = RX
	GPIO_InitTypeDef initStr = {GPIO_PIN_4 | GPIO_PIN_5,
                              GPIO_MODE_AF_PP,
                              GPIO_SPEED_FREQ_LOW,
                              GPIO_NOPULL};
  HAL_GPIO_Init(GPIOC, &initStr);
  GPIOC->AFR[0] |= ((1 << 16) | (1 << 20));

															
	GPIO_InitTypeDef initLEDs = {GPIO_PIN_8 | GPIO_PIN_9 |
															 GPIO_PIN_6 | GPIO_PIN_7,
															 GPIO_MODE_OUTPUT_PP,
															 GPIO_SPEED_FREQ_LOW,
															 GPIO_NOPULL};
	HAL_GPIO_Init(GPIOC, &initLEDs); // Initialize LED pins
 
  
	USART3->BRR &= ~(127 << 0);
	USART3->BRR |= (70 << 0);
															
	USART3->CR1 |= ((1 << 0) | (1 << 2) | (1 << 3) | (1 << 5)); // ENABLE: USART, RX, TX, RXNEIE
	NVIC_EnableIRQ(USART3_4_IRQn);
	NVIC_SetPriority(USART3_4_IRQn, 1);
  

	char rec_char = 0;
  while (1)
  {
		if(start) {
			usart_str_transmit("CMD?\r");
			start = 0;
		}
   
  }
}

void Stop(){
	//usart_str_transmit("STOPPED\n");
	curr_dir = 'x';
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
	// all off 
}
void Right(){
	//HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_8);
	curr_dir = 'd';
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
	// front-left forward 
	// front-right back
	// back-left back
	// back-right forward
}

void Left(){
	//HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_9);
	curr_dir = 'a';
	
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
	// front-left back 
	// front-right forward
	// back-left forward
	// back-right back
}

void Forward(){
	//HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_7);
	curr_dir = 'w';
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
	// all forward
	
	// front sensor on 
	
	// polling
	// while(1){
	//  if(obstacle){
	//			Stop(); 
	//			usart_str_transmit("OBSTACLE IN FRONT\n");
	//   }
	//  }
						
}

void Back(){
	//HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6);
	curr_dir = 's';
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
	// all backward
}


void usart_char_transmit(char c) {
	while((USART3->ISR & 128) == 0) {}
	USART3->TDR = c;
	return;
}

void usart_str_transmit(char *c) {
	char *p = c;
	while(*p != '\0') {
		usart_char_transmit(*p);
		p++;
	}
	return;
}

void USART3_4_IRQHandler(void) {
	if(_RX_flag == 0) {
		_RX_dir = 0;
		_RX_flag = 1;
	}
	else {
		_RX_dir |= USART3->RDR;
		if(_RX_dir == curr_dir){
			Stop();
			usart_str_transmit("STOPPED");
		} 
		else {
			
			if(_RX_dir == 'w') {
				Stop();
				Forward();
				usart_str_transmit("MOVING FORWARD");
			}
			else if(_RX_dir == 'a') {
				Stop();
				Left();
				usart_str_transmit("MOVING LEFT");
			}
			else if(_RX_dir == 'd') {
				Stop();
				Right();
				usart_str_transmit("MOVING RIGHT");
			}
			else if(_RX_dir == 's') {
				Stop();
				Back();
				usart_str_transmit("MOVING BACK");
			}
			else if(_RX_dir == 'x'){
				Stop();
				usart_str_transmit("STOPPED");
			}
			
		}
		
		usart_str_transmit("\n\r");
		_RX_flag = 0;
	}
}


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
