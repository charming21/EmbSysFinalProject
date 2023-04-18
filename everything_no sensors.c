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

void MotorDir_Clockwise(uint32_t, uint32_t);
void MotorDir_Counter_Clockwise(uint32_t, uint32_t);
void RoverMovement_Forward(void);
void RoverMovement_Reverse(void);
void RoverMovement_Left(void);
void RoverMovement_Right(void);
void RoverMovement_Stop(void);
void Motor_Setup(void);

//port B pins
const uint32_t PCB1_IN1 = GPIO_ODR_12;
const uint32_t PCB1_IN2 = GPIO_ODR_13;
const uint32_t PCB2_IN1 = GPIO_ODR_8;
const uint32_t PCB2_IN2 = GPIO_ODR_9;
const uint32_t PCB3_IN1 = GPIO_ODR_14;
const uint32_t PCB3_IN2 = GPIO_ODR_15;
const uint32_t PCB4_IN1 = GPIO_ODR_6;
const uint32_t PCB4_IN2 = GPIO_ODR_7;
//port B pin
const uint32_t Standby = GPIO_ODR_2;
const uint32_t Mask_For_All_PCB_Address = PCB1_IN1 | PCB1_IN2 | PCB2_IN1 | PCB2_IN2 | PCB3_IN1 | PCB3_IN2 | PCB4_IN1 | PCB4_IN2;

//uint16_t counter = 0;

void BusyLoop(int);
void BusyLoop(int t){
	while(t>0){
		t--;
	}
}
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
	
	Motor_Setup();
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
 
  
	USART3->BRR = HAL_RCC_GetHCLKFreq() / 9600;
															
	USART3->CR1 |= ((1 << 0) | (1 << 2) | (1 << 3) | (1 << 5)); // ENABLE: USART, RX, TX, RXNEIE
	NVIC_EnableIRQ(USART3_4_IRQn);
	NVIC_SetPriority(USART3_4_IRQn, 1);
  

	char rec_char = 0;
  while (1)
  {
		if(start) {
			usart_str_transmit("CMD?\n\r");
			start = 0;
		}
   
															 
  }
	/*														 
	RoverMovement_Forward();
  HAL_Delay(5000);
RoverMovement_Reverse();
HAL_Delay(5000);
RoverMovement_Left();
HAL_Delay(5000);
RoverMovement_Right();
HAL_Delay(5000);
	RoverMovement_Stop();*/
}

void Stop(){
	//usart_str_transmit("STOPPED\n");
	curr_dir = 'x';
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
	RoverMovement_Stop();
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
	RoverMovement_Right();
}

void Left(){
	//HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_9);
	curr_dir = 'a';
	
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
	// front-left back 
	// front-right forward
	// back-left forward
	// back-right back
	RoverMovement_Left();
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
	RoverMovement_Forward();	
}

void Back(){
	//HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6);
	curr_dir = 's';
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
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
			//curr_dir = "x";
			Stop();
			//BusyLoop(1000);
			usart_str_transmit("STOPPED");
		} 
		else {
			
			if(_RX_dir == 'w') {
				Stop();
				//BusyLoop(1000);

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



void Motor_Setup(){
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN; //enable A pins
	//RCC->AHBENR |= RCC_AHBENR_GPIOBEN; //enable B pins
	//RCC->AHBENR |= RCC_AHBENR_GPIOCEN; //enable C pins
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN; //enable TIM2
	
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
}

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
	//HAL_Delay(1); //comment this out later TODO
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
