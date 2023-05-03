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
#include <stdio.h>
#include <string.h>

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

void TransmitChar(char c) {
	while(1) {
		if(USART3->ISR & (1<<7)) {
			break;
		}
	}
	USART3->TDR = c; 
}

void TransmitString(char* string) {
	for (int i = 0; string[i] != '\0'; i++) {
		TransmitChar(string[i]);
	}
}

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MotorDir_Clockwise(uint32_t, uint32_t);
void MotorDir_Counter_Clockwise(uint32_t, uint32_t);
void RoverMovement_Forward(void);
void RoverMovement_Reverse(void);
void RoverMovement_Left(void);
void RoverMovement_Right(void);
void RoverMovement_Stop(void);
void Test_Forward(void);
void Test_Reverse(void);
void Test_Left(void);
void Test_Right(void);
void Test_Stop(void);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//***********SETUP PCB variable names ******************//
	const uint32_t PCB1_IN1 = GPIO_ODR_0;
	const uint32_t PCB1_IN2 = GPIO_ODR_1;
	const uint32_t PCB2_IN1 = GPIO_ODR_2;
	const uint32_t PCB2_IN2 = GPIO_ODR_3;
	const uint32_t PCB3_IN1 = GPIO_ODR_4;
	const uint32_t PCB3_IN2 = GPIO_ODR_5;
	const uint32_t PCB4_IN1 = GPIO_ODR_10;
	const uint32_t PCB4_IN2 = GPIO_ODR_11;
	const uint32_t Mask_For_All_PCB_Address = PCB1_IN1 | PCB1_IN2 | PCB2_IN1 | PCB2_IN2 | PCB3_IN1 | PCB3_IN2 | PCB4_IN1 | PCB4_IN2;
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
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN; //enable B pins
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN; //enable C pins
	
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  /* USER CODE BEGIN 2 */
//***************** SETUP LED *************************//
	//PIN C9 green led
	GPIOC->MODER |= (0x1 << 18);
	GPIOC->OTYPER &= ~(0x1<<9);
	GPIOC->OSPEEDR &= ~(0x1<<18);
	GPIOC->PUPDR &= ~(0x1<<18);
	
	//PIN C8 orange led
	GPIOC->MODER |= (0x1 << 16);
	GPIOC->OTYPER &= ~(0x1<<8);
	GPIOC->OSPEEDR &= ~(0x1<<16);
	GPIOC->PUPDR &= ~(0x1<<16);
	
	//PIN C6 and C7 red and blue led
	GPIOC->MODER |= (0x5<<12);
	GPIOC->OTYPER &= ~(0x3<<6);
	GPIOC->OSPEEDR &= ~(0x5<<12);
	GPIOC->PUPDR &= ~(0x5<<12);
	
//**********SETUP B pins 0-5 and 10,11 ****************//
	GPIOB->MODER |= GPIO_MODER_MODER0_0 | GPIO_MODER_MODER1_0 | GPIO_MODER_MODER2_0 | GPIO_MODER_MODER3_0 | GPIO_MODER_MODER4_0 | GPIO_MODER_MODER5_0 | GPIO_MODER_MODER10_0 | GPIO_MODER_MODER11_0 ; // Set to General pupose output mode (bits: 01)
	//GPIOB->OTYPER |= ; //should be in Output push-pull state
	//GPIOB->OSPEEDR |= ; //kept at default speed
  //GPIOB->PUPDR |= ; // should be in No pull-up, pull-down state
	
	
	
	//USART stuff begin	
	// PC4 TX
	// Set mode to alternate function
		GPIOC->MODER |= (1<<9);
		GPIOC->MODER &= ~(1<<8);
		
  // PC5 RX
	// Set mode to alternate function
		GPIOC->MODER |= (1<<11);
		GPIOC->MODER &= ~(1<<10);
		
		//alterante fucntion to AF1 0001
		GPIOC->AFR[0] &= ~(1U << 23 | 1U << 22 | 1U << 21);
		GPIOC->AFR[0] |= (1U << 20);
		GPIOC->AFR[0] &= ~(1U << 19 | 1U << 18 | 1U << 17);
		GPIOC->AFR[0] |= (1U << 16);
	
		// initialize USART
		RCC->APB1ENR |= RCC_APB1ENR_USART3EN;
		USART3->BRR = HAL_RCC_GetHCLKFreq() / 9600; 
		USART3->CR1 |= (1<<2 | 1<<3);
		USART3->CR1 |= (1<<0);
		
//USART stuff end
	
  /* USER CODE END 2 */
	
	//GPIOC->ODR |= GPIO_ODR_7;
	//GPIOC->ODR |= GPIO_ODR_8;
	//HAL_Delay(5000);
	//Test_Stop();
//	HAL_Delay(5000);
	
	//GPIOC->ODR |= GPIO_ODR_8;
	//GPIOC->ODR |= GPIO_ODR_9;
	
	
	HAL_Delay(500);
	Test_Forward();
	HAL_Delay(5000);
	Test_Reverse();
	HAL_Delay(5000);
	Test_Left();
	HAL_Delay(5000);
	Test_Right();
	HAL_Delay(5000);
	//Test_Stop();

	
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
//***********************Helper Methods************************//
void MotorDir_Clockwise(uint32_t pinIN1, uint32_t pinIN2){
	GPIOB->ODR |= pinIN2;
	GPIOB->ODR &= ~(pinIN1);
}

void MotorDir_Counter_Clockwise(uint32_t pinIN1, uint32_t pinIN2){
	GPIOB->ODR |= pinIN1;
	GPIOB->ODR &= ~(pinIN2);
}

void RoverMovement_Forward(){
	MotorDir_Counter_Clockwise(PCB1_IN1, PCB1_IN2);
	MotorDir_Counter_Clockwise(PCB2_IN1, PCB2_IN2);
	MotorDir_Clockwise(PCB3_IN1, PCB3_IN2);
	MotorDir_Clockwise(PCB4_IN1, PCB4_IN2);
}

void RoverMovement_Reverse(){
	MotorDir_Clockwise(PCB1_IN1, PCB1_IN2);
	MotorDir_Clockwise(PCB2_IN1, PCB2_IN2);
	MotorDir_Counter_Clockwise(PCB3_IN1, PCB3_IN2);
	MotorDir_Counter_Clockwise(PCB4_IN1, PCB4_IN2);
}

void RoverMovement_Left(){
	MotorDir_Clockwise(PCB1_IN1, PCB1_IN2);
	MotorDir_Counter_Clockwise(PCB2_IN1, PCB2_IN2);
	MotorDir_Clockwise(PCB3_IN1, PCB3_IN2);
	MotorDir_Counter_Clockwise(PCB4_IN1, PCB4_IN2);
}

void RoverMovement_Right(){
	MotorDir_Counter_Clockwise(PCB1_IN1, PCB1_IN2);
	MotorDir_Clockwise(PCB2_IN1, PCB2_IN2);
	MotorDir_Counter_Clockwise(PCB3_IN1, PCB3_IN2);
	MotorDir_Clockwise(PCB4_IN1, PCB4_IN2);
}
	
void RoverMovement_Stop(){
	GPIOB->ODR  |=  Mask_For_All_PCB_Address;
}
/*
Testing if all the pins were set correctly for moving the rover in the forward direction.
If true red light will turn on, else no light will turn on.
*/
void Test_Forward(){
	RoverMovement_Forward();
	if((GPIOB->IDR & (Mask_For_All_PCB_Address)) == (PCB1_IN1 | PCB2_IN1 | PCB3_IN2 | PCB4_IN2)){
		GPIOC->ODR |= GPIO_ODR_6;
	}
	
}
/*
Testing if all the pins were set correctly for moving the rover in the Reverse direction.
If true blue light will turn on, else no light will turn on.
*/
void Test_Reverse(){
	RoverMovement_Reverse();
	if((GPIOB->IDR & (Mask_For_All_PCB_Address)) == (PCB1_IN2 | PCB2_IN2 | PCB3_IN1 | PCB4_IN1)){
		GPIOC->ODR |= GPIO_ODR_7;
	}
}
/*
Testing if all the pins were set correctly for moving the rover in the Left direction.
If true green light will turn on, else no light will turn on.
*/
void Test_Left(){
	RoverMovement_Left();
	if((GPIOB->IDR & (Mask_For_All_PCB_Address)) == (PCB1_IN2 | PCB2_IN1 | PCB3_IN2 | PCB4_IN1)){
		GPIOC->ODR |= GPIO_ODR_8;
	}
}
/*
Testing if all the pins were set correctly for moving the rover in the Right direction.
If true orange light will turn on, else no light will turn on.
*/
void Test_Right(){
	RoverMovement_Right();
	if((GPIOB->IDR & (Mask_For_All_PCB_Address)) == (PCB1_IN1 | PCB2_IN2 | PCB3_IN1 | PCB4_IN2)){
		GPIOC->ODR |= GPIO_ODR_9;
	}
}
/*
Testing if all the pins were set correctly for stopping the rover from moving in any direction.
If true all lights will turn off, else all lights will remain on.
*/
void Test_Stop(){
	RoverMovement_Stop();
	//GPIOB->ODR  |=  Mask_For_All_PCB_Address;
	
//TransmitString(" just started ");
//	char str[100]; 
//  sprintf(str, " data reg is: %d ", GPIOB->IDR);
//	TransmitString(str);
	
	HAL_Delay(1);

	if((GPIOB->IDR & (1<<0)) != 0 &  (GPIOB->IDR & (1<<1)) != 0 &  (GPIOB->IDR & (1<<2)) != 0 &  (GPIOB->IDR & (1<<3)) != 0 & 
		  (GPIOB->IDR & (1<<4)) != 0 &  (GPIOB->IDR & (1<<5)) != 0 &  (GPIOB->IDR & (1<<10)) != 0 &  (GPIOB->IDR & (1<<11)) != 0){
		  GPIOC->ODR &= ~(1<<6);
			GPIOC->ODR &= ~(1<<7);
			GPIOC->ODR &= ~(1<<8);
			GPIOC->ODR &= ~(1<<9);	
}
//	
//	if((GPIOB->IDR & (Mask_For_All_PCB_Address)) == Mask_For_All_PCB_Address){
//		GPIOC->ODR |= GPIO_ODR_9;
//	}
			
			
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
