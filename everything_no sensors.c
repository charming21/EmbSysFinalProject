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



void SystemClock_Config(void);

void Stop(void);
void Right(void);
void Left(void);
void Forward(void);
void Back(void);

void usart_char_transmit(char c);
void usart_str_transmit(char *c);
void USART3_4_IRQHandler(void);
volatile char _RX_dir, _RX_flag, _RX_idle = 0;
volatile char start = 1;
volatile char curr_dir = 0;
char *err = "error\n";

void MotorDir_Clockwise(uint32_t, uint32_t, char);
void MotorDir_Counter_Clockwise(uint32_t, uint32_t, char);
void RoverMovement_Forward(void);
void RoverMovement_Reverse(void);
void RoverMovement_Left(void);
void RoverMovement_Right(void);
void RoverMovement_Stop(void);
void Setup_Motor(void);

const uint32_t PCB1_IN1 = GPIO_MODER_MODER8_1;// Timer 3 channel 3 - PC8
const uint32_t PCB1_IN2 = GPIO_MODER_MODER9_1;// Timer 3 channel 4 - PC9
const uint32_t PCB2_IN1 = GPIO_MODER_MODER0_1;// Timer 2 channel 1 - PA0
const uint32_t PCB2_IN2 = GPIO_MODER_MODER1_1;// Timer 2 channel 2 - PA1
const uint32_t PCB3_IN1 = GPIO_MODER_MODER4_1;// Timer 3 channel 1 - PB4
const uint32_t PCB3_IN2 = GPIO_MODER_MODER5_1;// Timer 3 channel 2 - PB5
const uint32_t PCB4_IN1 = GPIO_MODER_MODER10_1;// Timer 2 channel 3 - PB10
const uint32_t PCB4_IN2 = GPIO_MODER_MODER11_1;// Timer 2 channel 4 - PB11

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

															
	GPIO_InitTypeDef initLEDs = {//GPIO_PIN_8 | GPIO_PIN_9 |
															 GPIO_PIN_6 | GPIO_PIN_7,
															 GPIO_MODE_OUTPUT_PP,
															 GPIO_SPEED_FREQ_LOW,
															 GPIO_NOPULL};
	HAL_GPIO_Init(GPIOC, &initLEDs); // Initialize LED pins
 
  
	USART3->BRR &= ~(127 << 0);
	USART3->BRR |= HAL_RCC_GetHCLKFreq()/9600;
															
	USART3->CR1 |= ((1 << 0) | (1 << 2) | (1 << 3) | (1 << 5)); // ENABLE: USART, RX, TX, RXNEIE
	NVIC_EnableIRQ(USART3_4_IRQn);
	NVIC_SetPriority(USART3_4_IRQn, 1);
  
	Setup_Motor(); //Setting up the motor functions
															 
	char rec_char = 0;
  while (1)
  {
		if(start) {
			usart_str_transmit("CMD?\r");
			start = 0;
		}
		
		/*HAL_Delay(2000);
		//char str[100];
		//sprintf(str, "currnet direction : %c ", curr_dir);
		usart_char_transmit(curr_dir);
		*/
		
   
  }
}

void Stop(){
	//usart_str_transmit("STOPPED\n");
	curr_dir = 'X';
	//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);
	//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);
	//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
	//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
	// all off 
	RoverMovement_Stop();
}
void Right(){
	//HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_8);
	curr_dir = 'd';
	//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
	// front-left forward 
	// front-right back
	// back-left back
	// back-right forward
	RoverMovement_Right();
}

void Left(){
	//HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_9);
	curr_dir = 'a';
	
	//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
	// front-left back 
	// front-right forward
	// back-left forward
	// back-right back
	RoverMovement_Left();
}

void Forward(){
	//HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_7);
	curr_dir = 'w';
	//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
	// all forward
	RoverMovement_Forward();
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
	//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
	// all backward
	RoverMovement_Reverse();
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
			curr_dir = 'X';
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



//***********************Helper Methods************************//
void Setup_Motor(){
	
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN; //enable A pins
	//RCC->AHBENR |= RCC_AHBENR_GPIOBEN; //enable B pins
	//RCC->AHBENR |= RCC_AHBENR_GPIOCEN; //enable C pins
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN; //enable TIM3
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN; //enable TIM2
	
	//**********SETUP B pins 4,5,10,11 for PWM****************//
	//GPIOB->MODER |= GPIO_MODER_MODER0_1 | GPIO_MODER_MODER1_1  | GPIO_MODER_MODER4_1 | GPIO_MODER_MODER5_1 ;  // Set to Alternate function output mode (bits: 10)
	GPIOB->MODER |= GPIO_MODER_MODER4_1 | GPIO_MODER_MODER5_1 ;  // Set to Alternate function output mode (bits: 10)
	GPIOB->MODER |= GPIO_MODER_MODER10_1 | GPIO_MODER_MODER11_1 ; //Set to Alternate function output mode (bits: 10)
	//GPIOB->OTYPER |= ; //should be in Output push-pull state
	//GPIOB->OSPEEDR |= ; //kept at default speed
  //GPIOB->PUPDR |= ; // should be in No pull-up, pull-down state
	//GPIOB->AFR[0] |= 0x1 | 0x1<<4 | 0x1 << 16 | 0x1 << 20; //PB0,1,4,5 alternate function AF1
	GPIOB->AFR[0] |=  0x1 << 16 | 0x1 << 20; //PB4,5 alternate function AF1
	GPIOB->AFR[1] |= 0x2<<8 | 0x2<<12 ;//PB10,11 alternate function AF2
//**********END of SETUP B pins 4,5,10,11 ****************//

//**********SETUP A pins 0,1 for PWM ***************************//
	GPIOA->MODER |= GPIO_MODER_MODER0_1 | GPIO_MODER_MODER1_1 | GPIO_MODER_MODER2_1 | GPIO_MODER_MODER3_1 ; // Set to Alternate function output mode (bits: 10)
	//GPIOB->OTYPER |= ; //should be in Output push-pull state
	//GPIOB->OSPEEDR |= ; //kept at default speed
  //GPIOB->PUPDR |= ; // should be in No pull-up, pull-down state
	GPIOA->AFR[0] |= 0x2 | 0x2<<4 | 0x2 << 8 | 0x2 << 12; //PA0,1,2,3 alternate function AF2

//**********END of SETUP A pins 0,1 ********************//

//**********SETUP C pins 0,1 for PWM ********************//
	GPIOC->MODER |= (0x1 << 19); //PC9 alternate function AF0
	GPIOC->MODER |= (0x1 << 17); //PC8 alternate function AF0
//**********END of SETUP C pins 0,1 ********************//

	/******************SETUP PWM TIMER 3 (PB0,1,4,5 channel 1,2,3,4) ***********************/
	
	TIM3 -> PSC = (999); 														
	TIM3 -> ARR = (10); //100*100 = 10,000 desired f = 800Hz		
	TIM3 -> DIER = 0; //RESET inturrupts to disable on all cases

	TIM3 -> CCMR1 &= ~(0b11 << 8);		// set channel 2 to output mode
	TIM3 -> CCMR1 &= ~(0b11);	     	// set channel 1 to output mode
	TIM3 -> CCMR2 &= ~(0b11 << 8);		// set channel 4 to output mode
	TIM3 -> CCMR2 &= ~(0b11);	     	// set channel 3 to output mode 
//	if(TIM3->CCMR2 == 0){
//		GPIOC->ODR |= GPIO_ODR_6; // turn in on red LED
//	}
	
	TIM3 -> CCMR1 |= (6 << 4);   		// set OC1M to PWM mode 1	
  TIM3 -> CCMR1 |= (6 << 12);   	// set OC2M to PWM mode 1	
	TIM3 -> CCMR2 |= (6 << 4);   		// set OC3M to PWM mode 1	
	TIM3 -> CCMR2 |= (6 << 12);   	// set OC4M to PWM mode 1	
	TIM3 -> CCMR1 |= (1 << 3) | (1 << 11); // enable channel 1 & 2 preload
	TIM3 -> CCMR2 |= (1 << 3) | (1 << 11); // enable channel 3 & 4 preload
	TIM3 -> CCER |= (1); 						// channel 1 output enable 
	TIM3 -> CCER |= (1 << 4); 			// channel 2 output enable 
	TIM3 -> CCER |= (1 << 8); 			// channel 3 output enable 
	TIM3 -> CCER |= (1 << 12); 			// channel 4 output enable 
//	if(TIM3->CCER == 0x1111){
//		GPIOC->ODR |= GPIO_ODR_6; // turn in on red LED
//	}
	TIM3 -> CCR1 = (10); // Heavy Duty for channel 1
	TIM3 -> CCR2 = (10); // Heavy Duty for Channel 2
	TIM3 -> CCR3 = (10); // Heavy Duty for channel 3
	TIM3 -> CCR4 = (10); // Heavy Duty for Channel 4
	if(TIM3->CCR1 == 10){
		GPIOC->ODR |= GPIO_ODR_6; // turn in on red LED
	}
	if(TIM3->CCR2 == 10){
		GPIOC->ODR |= GPIO_ODR_7; // turn in on red LED
	}
	if(TIM3->CCR3 == 10){
		//GPIOC->ODR |= GPIO_ODR_8; // turn in on red LED
	}
	if(TIM3->CCR4 == 10){
		GPIOC->ODR |= GPIO_ODR_9; // turn in on red LED
	}
	
	TIM3 -> CR1 |= (1);	 // timer enable

	/******************END OF SETUP PWM TIMER 3 (PB0,1,4,5 channel 1,2,3,4) ***********************/
	
	/******************SETUP PWM TIMER 2 (PA0,1,2,3 channel 1,2,3,4) ***********************/
	
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
	
	/******************END OF SETUP PWM TIMER 2 (PA0,1,2,3 channel 1,2,3,4) ***********************/
	
}
void MotorDir_Clockwise(uint32_t pinIN1, uint32_t pinIN2, char portAorB){
	if(portAorB == 'A')
	{
		TIM2 -> CCR1 = (0);
		TIM2 -> CCR2 = (10);
	}
	else if(portAorB == 'B')
	{
		if(pinIN1 == PCB3_IN1 & pinIN2 == PCB3_IN2){
			TIM3 -> CCR1 = (0);
			TIM3 -> CCR2 = (10);
		}
		else if(pinIN1 == PCB4_IN1 & pinIN2 == PCB4_IN2){
			TIM2 -> CCR3 = (0);
			TIM2 -> CCR4 = (10);
		}
	}
	else if(portAorB == 'C')
	{
		TIM3 -> CCR3 = (0);
		TIM3 -> CCR4 = (10);
	}
}

void MotorDir_Counter_Clockwise(uint32_t pinIN1, uint32_t pinIN2, char portAorB){
	if(portAorB == 'A')
	{
		TIM2 -> CCR1 = (10);
		TIM2 -> CCR2 = (0);
	}
	else if(portAorB == 'B')
	{
		if(pinIN1 == PCB3_IN1 & pinIN2 == PCB3_IN2){
			TIM3 -> CCR1 = (10);
			TIM3 -> CCR2 = (0);
		}
		else if(pinIN1 == PCB4_IN1 & pinIN2 == PCB4_IN2){
			TIM2 -> CCR3 = (10);
			TIM2 -> CCR4 = (0);
		}
	}
	else if(portAorB == 'C')
	{
		TIM3 -> CCR3 = (10);
		TIM3 -> CCR4 = (0);
	}
}

void RoverMovement_Forward(){
	MotorDir_Counter_Clockwise(PCB1_IN1, PCB1_IN2, 'C');
	MotorDir_Counter_Clockwise(PCB2_IN1, PCB2_IN2, 'A');
	MotorDir_Clockwise(PCB3_IN1, PCB3_IN2, 'B');
	MotorDir_Clockwise(PCB4_IN1, PCB4_IN2, 'B');
}

void RoverMovement_Reverse(){
	MotorDir_Clockwise(PCB1_IN1, PCB1_IN2, 'C');
	MotorDir_Clockwise(PCB2_IN1, PCB2_IN2, 'A');
	MotorDir_Counter_Clockwise(PCB3_IN1, PCB3_IN2, 'B');
	MotorDir_Counter_Clockwise(PCB4_IN1, PCB4_IN2, 'B');
}

void RoverMovement_Left(){
	MotorDir_Clockwise(PCB1_IN1, PCB1_IN2, 'C');
	MotorDir_Counter_Clockwise(PCB2_IN1, PCB2_IN2, 'A');
	MotorDir_Clockwise(PCB3_IN1, PCB3_IN2, 'B');
	MotorDir_Counter_Clockwise(PCB4_IN1, PCB4_IN2, 'B');
}

void RoverMovement_Right(){
	MotorDir_Counter_Clockwise(PCB1_IN1, PCB1_IN2, 'C');
	MotorDir_Clockwise(PCB2_IN1, PCB2_IN2, 'A');
	MotorDir_Counter_Clockwise(PCB3_IN1, PCB3_IN2, 'B');
	MotorDir_Clockwise(PCB4_IN1, PCB4_IN2, 'B');
}
	
void RoverMovement_Stop(){
	//GPIOB->ODR  |=  Mask_For_All_PCB_Address;
	//GPIOC->ODR &= ~(GPIO_ODR_6 | GPIO_ODR_7 | GPIO_ODR_8 | GPIO_ODR_9); // Turn off all LEDS
	
	TIM3 -> CCR1 = (10); // set heavy duty to 100%
	TIM3 -> CCR2 = (10); // set heavy duty to 100%
	TIM3 -> CCR3 = (10); // set heavy duty to 100%
	TIM3 -> CCR4 = (10); // set heavy duty to 100%
	
	TIM2 -> CCR1 = (10); // set heavy duty to 100%
	TIM2 -> CCR2 = (10); // set heavy duty to 100%
	TIM2 -> CCR3 = (10); // set heavy duty to 100%
	TIM2 -> CCR4 = (10); // set heavy duty to 100%
	
	//HAL_Delay(1);
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