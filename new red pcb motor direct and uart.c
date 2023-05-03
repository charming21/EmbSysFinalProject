/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : new red pcb motor direct and uart.c
  * @brief          : (Testing) Combines motor direction functions and UART control to achieve initial motor control for testing on new dual motor driver PCBs
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
void MotorDir_Clockwise(uint32_t, uint32_t);
void MotorDir_Counter_Clockwise(uint32_t, uint32_t);
void RoverMovement_Forward();
void RoverMovement_Reverse();
void RoverMovement_Left();
void RoverMovement_Right();
void RoverMovement_Stop();
void Test_Forward();
void Test_Reverse();
void Test_Left();
void Test_Right();
void Test_Stop();

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//***********SETUP PCB variable names ******************//
	const uint32_t PCB1_IN1 = GPIO_ODR_12;
	const uint32_t PCB1_IN2 = GPIO_ODR_13;
	const uint32_t PCB2_IN1 = GPIO_ODR_8;
	const uint32_t PCB2_IN2 = GPIO_ODR_9;
	const uint32_t PCB3_IN1 = GPIO_ODR_14;
	const uint32_t PCB3_IN2 = GPIO_ODR_15;
	const uint32_t PCB4_IN1 = GPIO_ODR_6;
	const uint32_t PCB4_IN2 = GPIO_ODR_7;
	
	const uint32_t Standby = GPIO_ODR_2;
	
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
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN; //enable A pins
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN; //enable B pins
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN; //enable C pins
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN; //enable TIM2
	
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  /* USER CODE BEGIN 2 */
//***************** SETUP LED *************************//
	{
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
	}
	

//**********SETUP A pins 0,1 for PWM ***************************//
	GPIOA->MODER |= GPIO_MODER_MODER0_1 | GPIO_MODER_MODER1_1 ;//| GPIO_MODER_MODER2_1 | GPIO_MODER_MODER3_1 ; // Set to Alternate function output mode (bits: 10)
	GPIOA->AFR[0] |= 0x2 | 0x2<<4 ; //| 0x2 << 8 | 0x2 << 12; //PA0,1,2,3 alternate function AF2

//**********END of SETUP A pins 0,1 ********************//
	
//**********SETUP B pins 2,6-9,12-15 for logic high/low ****************//
	//remove gpio_moder_moder3_0
	GPIOB->MODER |= GPIO_MODER_MODER2_0 | GPIO_MODER_MODER3_0 | GPIO_MODER_MODER6_0|GPIO_MODER_MODER7_0|GPIO_MODER_MODER8_0|GPIO_MODER_MODER9_0 | GPIO_MODER_MODER12_0| GPIO_MODER_MODER13_0| GPIO_MODER_MODER14_0| GPIO_MODER_MODER15_0; // Set to General pupose output mode (bits: 01)
	//GPIOB->OTYPER |= ; //should be in Output push-pull state
	//GPIOB->OSPEEDR |= ; //kept at default speed
  //GPIOB->PUPDR |= ; // should be in No pull-up, pull-down state
//**********SETUP B pins 2,6-9,12-15  ****************//

//**********SETUP B pins 10,11 for PWM****************//
	GPIOB->MODER |= GPIO_MODER_MODER10_1 | GPIO_MODER_MODER11_1 ; //Set to Alternate function output mode (bits: 10)
	GPIOB->AFR[1] |= 0x2<<8 | 0x2<<12 ;//PB10,11 alternate function AF2
//**********END of SETUP B pins 4,5,10,11 ****************//

/******************SETUP PWM TIMER 2 (PA0,1,PB10,11 channel 1,2,3,4) ***********************/
	
	TIM2 -> PSC = (999); 														
	TIM2 -> ARR = (10); //100*100 = 10,000 desired f = 800Hz		
	TIM2 -> DIER = 0; //RESET inturrupts to disable on all cases

	TIM2 -> CCMR1 |= (0b00 << 8);		// set channel 2 to output mode
	TIM2 -> CCMR1 |= (0b00);	     // set channel 1 to output mode
	TIM2 -> CCMR2 |= (0b00 << 8);		// set channel 4 to output mode
	TIM2 -> CCMR2 |= (0b00);	     // set channel 3 to output mode 
	TIM2 -> CCMR1 |= (6 << 4);   // set OC1M to PWM mode 1	
  TIM2 -> CCMR1 |= (6 << 12);   // set OC2M to PWM mode 1	
	TIM2 -> CCMR2 |= (6 << 4);   // set OC3M to PWM mode 1	
	TIM2 -> CCMR2 |= (6 << 12);   // set OC4M to PWM mode 1	
	TIM2 -> CCMR1 |= (1 << 3) | (1 << 11); // enable channel 1 & 2 preload
	TIM2 -> CCMR2 |= (1 << 3) | (1 << 11); // enable channel 3 & 4 preload
	TIM2 -> CCER |= (1); // channel 1 output enable 
	TIM2 -> CCER |= (1 << 4); // channel 2 output enable 
	TIM2 -> CCER |= (1 << 8); // channel 3 output enable 
	TIM2 -> CCER |= (1 << 12); // channel 4 output enable 
	TIM2 -> CCR1 = (10); // Heavy Duty for channel 1
	TIM2 -> CCR2 = (10); // Heavy Duty for Channel 2
	TIM2 -> CCR3 = (10); // Heavy Duty for channel 3
	TIM2 -> CCR4 = (10); // Heavy Duty for Channel 4
	
	TIM2 -> CR1 |= (1); // timer enable
	
/******************END OF SETUP PWM TIMER 2 (PA0,1,PB10,11 channel 1,2,3,4) ***********************/

	
	//setting standby high PB2
	GPIOB->ODR  |= 0x1<<2;
	HAL_Delay(1000);
  /* USER CODE END 2 */

	Test_Forward();
  HAL_Delay(5000);
	Test_Reverse();
  HAL_Delay(5000);
	Test_Left();
  HAL_Delay(5000);
  Test_Right();
	HAL_Delay(5000);
	Test_Stop();
  
	//GPIOB->ODR  =  (Mask_For_All_PCB_Address);
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
void MotorDir_Counter_Clockwise(uint32_t pinIN1, uint32_t pinIN2){
	GPIOB->ODR |= pinIN2;
	GPIOB->ODR &= ~(pinIN1);
}

void MotorDir_Clockwise(uint32_t pinIN1, uint32_t pinIN2){
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
	GPIOB->ODR  &=  ~(Mask_For_All_PCB_Address); //turns off signals to logical lows
	HAL_Delay(1); //comment this out later TODO
}
/*
Testing if all the pins were set correctly for moving the rover in the forward direction.
If true red light will turn on, else no light will turn on.
*/
void Test_Forward(){
	RoverMovement_Forward();
	if((GPIOB->IDR & (Mask_For_All_PCB_Address)) == (PCB1_IN2 | PCB2_IN2 | PCB3_IN1 | PCB4_IN1)){
		GPIOC->ODR |= GPIO_ODR_6;
	}
	
}
/*
Testing if all the pins were set correctly for moving the rover in the Reverse direction.
If true blue light will turn on, else no light will turn on.
*/
void Test_Reverse(){
	RoverMovement_Reverse();
	if((GPIOB->IDR & (Mask_For_All_PCB_Address)) == (PCB1_IN1 | PCB2_IN1 | PCB3_IN2 | PCB4_IN2)){
		GPIOC->ODR |= GPIO_ODR_7;
	}
}
/*
Testing if all the pins were set correctly for moving the rover in the Left direction.
If true green light will turn on, else no light will turn on.
*/
void Test_Left(){
	RoverMovement_Left();
	if((GPIOB->IDR & (Mask_For_All_PCB_Address)) == (PCB1_IN1 | PCB2_IN2 | PCB3_IN1 | PCB4_IN2)){
		GPIOC->ODR |= GPIO_ODR_8;
	}
}
/*
Testing if all the pins were set correctly for moving the rover in the Right direction.
If true orange light will turn on, else no light will turn on.
*/
void Test_Right(){
	RoverMovement_Right();
	if((GPIOB->IDR & (Mask_For_All_PCB_Address)) == (PCB1_IN2 | PCB2_IN1 | PCB3_IN2 | PCB4_IN1)){
		GPIOC->ODR |= GPIO_ODR_9;
	}
}
/*
Testing if all the pins were set correctly for stopping the rover from moving in any direction.
If true all lights will turn off, else all lights will remain on.
*/
void Test_Stop(){
	RoverMovement_Stop();
	if((GPIOB->IDR & (Mask_For_All_PCB_Address | Standby)) == (Standby)){
		GPIOC->ODR &= ~(GPIO_ODR_6| GPIO_ODR_7 | GPIO_ODR_8 | GPIO_ODR_9);
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
