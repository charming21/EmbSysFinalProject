
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
void setup_LEDs(void);
void TransmitChar(char c);
void TransmitString(char* string);
void delay_microsecond(uint32_t us);
void send_pulse(char pin, int trig, int echo);
void USART3_4_IRQHandler(void);

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
void Forward();
void Reverse();
void Left();
void Right();
void Stop();
void Back();


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
	
	volatile char _RX_flag = 0;
	
	volatile char _RX_dir = 'x';
	volatile char curr_dir = 1;
	
	
/* USER CODE END 0 */

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
  HAL_Init();
  SystemClock_Config();

  // RCC
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN; //enable A pins
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN; //enable B pins
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN; //enable C pins
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN; //enable TIM2
	
	setup_LEDs();
	
// setup USART 
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
		USART3->CR1 |= (1<<2 | 1<<3 | 1<<5 );
		NVIC_EnableIRQ(USART3_4_IRQn);
		NVIC_SetPriority(USART3_4_IRQn, 1);
		USART3->CR1 |= (1<<0);
			

  // sensor 1
	// Configure PA4 (trigger)
		GPIOA->MODER |= (1<<8);
//		GPIOA->MODER &= ~(1<<9);

//	// PA5 (echo)
//		GPIOA->MODER &= ~(1<<10 | 1<<11);
//		GPIOA->PUPDR |= (1<<10);
//		GPIOA->PUPDR &= ~(1<<11);
		
// sensor 2
	// Configure PA8 (trigger)
		GPIOA->MODER |= (1<<16);
//		GPIOA->MODER &= ~(1<<17);

//	// Set trigger to low
//		GPIOA->ODR &= ~(1<<8);

//	// Configure PA9 (echo)
//		GPIOA->MODER &= ~(1<<18 | 1<<19);
//		GPIOA->PUPDR |= (1<<18);
//		GPIOA->PUPDR &= ~(1<<19);
		
// sensor 3 
	// Configure PA14 (trigger)
		GPIOA->MODER |= (1<<28);
//		GPIOA->MODER &= ~(1<<29);

//	// Set trigger to low
//		GPIOA->ODR &= ~(1<<14);

//	// Configure PA15 (echo)
//		GPIOA->MODER &= ~(1<<30 | 1<<31);
//		GPIOA->PUPDR |= (1<<30);
//		GPIOA->PUPDR &= ~(1<<31);

// sensor 4 
	// Configure PA10 (trigger)
		GPIOA->MODER |= (1<<20);
//		GPIOA->MODER &= ~(1<<21);

//	// Set trigger to low
//		GPIOA->ODR &= ~(1<<10);

//	// Configure PA13 (echo)
//		GPIOA->MODER &= ~(1<<26 | 1<<27);
//		GPIOA->PUPDR |= (1<<26);
//		GPIOA->PUPDR &= ~(1<<27);


////**********SETUP A pins 0,1 for PWM ***************************//
	GPIOA->MODER |= GPIO_MODER_MODER0_1 | GPIO_MODER_MODER1_1 ;//| GPIO_MODER_MODER2_1 | GPIO_MODER_MODER3_1 ; // Set to Alternate function output mode (bits: 10)
	GPIOA->AFR[0] |= 0x2 | 0x2<<4 ; //| 0x2 << 8 | 0x2 << 12; //PA0,1,2,3 alternate function AF2

//**********END of SETUP A pins 0,1 ********************//
	
//**********SETUP B pins 2,6-9,12-15 for logic high/low ****************//
	//remove gpio_moder_moder3_0
	GPIOB->MODER |= GPIO_MODER_MODER2_0 | GPIO_MODER_MODER3_0 | GPIO_MODER_MODER6_0|GPIO_MODER_MODER7_0|GPIO_MODER_MODER8_0|GPIO_MODER_MODER9_0 | GPIO_MODER_MODER12_0| GPIO_MODER_MODER13_0| GPIO_MODER_MODER14_0| GPIO_MODER_MODER15_0; // Set to General pupose output mode (bits: 01)
	GPIOA->MODER |= 0x1<<4;; // testing
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
	TIM2 -> CCR1 = (5); // Heavy Duty for channel 1
	TIM2 -> CCR2 = (5); // Heavy Duty for Channel 2
	TIM2 -> CCR3 = (5); // Heavy Duty for channel 3
	TIM2 -> CCR4 = (5); // Heavy Duty for Channel 4
	
	TIM2 -> CR1 |= (1); // timer enable
	
/******************END OF SETUP PWM TIMER 2 (PA0,1,PB10,11 channel 1,2,3,4) ***********************/

	//setting standby high PB2
	GPIOB->ODR  |= 0x1<<2;
	HAL_Delay(100);
	
	
	TransmitString("CMD?");
  /* USER CODE END 2 */

//	Test_Forward();
//  HAL_Delay(5000);
//	Test_Left();
//  HAL_Delay(5000);
//	Test_Reverse();
//  HAL_Delay(5000);
//  Test_Right();
//	HAL_Delay(5000);
//	Test_Stop();
	
  while (1)
  {
		// senssors stuff don't use if sensors are not connected
		//send_pulse('A', 4, 5);
		//send_pulse('A', 8, 9);
		//send_pulse('A', 14, 15);
		//send_pulse('A', 10, 13);
		

		}
		
}


void Stop(){
	//usart_str_transmit("STOPPED\n");
	curr_dir = 'x';
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
	RoverMovement_Right();
	//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
	// front-left forward 
	// front-right back
	// back-left back
	// back-right forward
}

void Left(){
	//HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_9);
	curr_dir = 'a';
	
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
	
	RoverMovement_Left();
	// front-left back 
	// front-right forward
	// back-left forward
	// back-right back
}

void Forward(){
	//HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_7);
	curr_dir = 'w';
	//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
	RoverMovement_Forward();
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
	RoverMovement_Reverse();
	//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
	// all backward
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
			TransmitString("STOPPED");
		} 
		else {
			
			if(_RX_dir == 'w') {
				Stop();
				Forward();
				TransmitString("MOVING FORWARD");
			}
			else if(_RX_dir == 'a') {
				Stop();
				Left();
				TransmitString("MOVING LEFT");
			}
			else if(_RX_dir == 'd') {
				Stop();
				Right();
				TransmitString("MOVING RIGHT");
			}
			else if(_RX_dir == 's') {
				Stop();
				Back();
				TransmitString("MOVING BACK");
			}
			else if(_RX_dir == 'x'){
				Stop();
				TransmitString("STOPPED");
			}
			else {
				//Stop();
				TransmitString("else");
			}
	}
		
		//TransmitString("\n\r");
		_RX_flag = 0;
	}
}

void setup_LEDs(void) {
	
	// PC6 RED LED
	// Set mode to general-purpose 
		GPIOC->MODER |= (1<<12);
		GPIOC->MODER &= ~(1<<13);
	// Set type to push-pull
		GPIOC->OTYPER &= ~(1<<6);
	// Set speed to low
		GPIOC->OSPEEDR &= ~(1<<12);
	// Set PUPDR to no pull-up/down
		GPIOC->PUPDR &= ~(1<<12 | 1<<13);
		
	// PC7 BLUE LED
	// Set mode to general-purpose 
		GPIOC->MODER |= (1<<14);
		GPIOC->MODER &= ~(1<<15);
	// Set type to push-pull
		GPIOC->OTYPER &= ~(1<<7);
	// Set speed to low
		GPIOC->OSPEEDR &= ~(1<<14);
	// Set PUPDR to no pull-up/down
		GPIOC->PUPDR &= ~(1<<14 | 1<<15);
			
	// PC8 ORANGE LED
	// Set mode to general-purpose 
		GPIOC->MODER |= (1<<16);
		GPIOC->MODER &= ~(1<<17);
	// Set type to push-pull
		GPIOC->OTYPER &= ~(1<<8);
	// Set speed to low
		GPIOC->OSPEEDR &= ~(1<<16);
	// Set PUPDR to no pull-up/down
		GPIOC->PUPDR &= ~(1<<16 | 1<<17);
		
	// PC9 GREEN LED
	// Set mode to general-purpose 
		GPIOC->MODER |= (1<<18);
		GPIOC->MODER &= ~(1<<19);
	// Set type to push-pull
		GPIOC->OTYPER &= ~(1<<9);
	// Set speed to low
		GPIOC->OSPEEDR &= ~(1<<18);
	// Set PUPDR to no pull-up/down
		GPIOC->PUPDR &= ~(1<<18 | 1<<19);

}

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

/**
* Provides minimum delay (in microseconds) based on varible us
* @param us delay amount in ms
*/
void delay_microsecond(uint32_t us)
{
// initialize TIM2
  	RCC->APB1ENR |= RCC_APB1ENR_TIM14EN;
  	TIM14->PSC = 79;
  	TIM14->ARR = us - 1;
  	TIM14->DIER |= TIM_DIER_UIE; //enable interrupt
  	TIM14->SR &= ~TIM_SR_UIF;
  	TIM14->CR1 |= TIM_CR1_CEN; //enable timer
	
  // reset timer and wait for it to reach ARR value
    TIM14->CNT = 0;
  	while ((TIM14->SR & TIM_SR_UIF) == 0); 
  	TIM14->SR &= ~TIM_SR_UIF; 
  	TIM14->CR1 &= ~TIM_CR1_CEN; //disble timer
}

/**
* Sends an ultrasonic pulse and measures how far an object is from the ultrasonic sensor. 
* Provides visual feedback using LEDs, and UART messages for more specific details.
* @param pin GPIO pin A-E
* @param trig trigger pin number
* @param echo echo pin number
*/
void send_pulse(char pin, int trig, int echo) {
	
	uint32_t pulse_length = 0;
    float distance = 0.0;
	GPIO_TypeDef * GPIO_pin;
	
	if (pin == 'A') {
		GPIO_pin = GPIOA;
	} else if (pin == 'B') {
		GPIO_pin = GPIOB;
	} else if (pin == 'C') {
		GPIO_pin = GPIOB;
	}
	
	  GPIO_pin->ODR &= ~(1<<trig);
	  delay_microsecond(2);
    GPIO_pin->ODR |= (1<<trig);
    delay_microsecond(10);
    GPIO_pin->ODR &= ~(1<<trig);
		
	
  // wait for echo to be high
	while(1) {
		if(GPIO_pin->IDR & (1<<echo)) {
			break;
	}
	}
				
  // while echo is high
    while (GPIO_pin->IDR & (1<<echo))
    {
    	pulse_length++;
    	delay_microsecond(10);
    }
		
		// GPIOC->ODR |= (1<<9); to check if we got stuck 
				
  // distance in cm (not really)
    distance = pulse_length * 0.034 / 2;
			
  
	/*
		if (distance < 0.1) {
			TransmitString(" something in the way mmmm ");
			GPIOC->ODR |= (1<<6);
		} else {
			//TransmitString(" keep goin ");
			GPIOC->ODR &= ~(1<<6);
			}
	*/
	// check which sensor we're measuring
	if(trig == 4) {
		if (distance < 0.1) {
			TransmitString(" something in the way mmmm ");
			Stop();
			GPIOC->ODR |= (1<<9);
		} else {
			//TransmitString(" keep goin ");
			GPIOC->ODR &= ~(1<<9);
			}
	}
	else if(trig == 8) {
		if (distance < 0.1) {
			TransmitString(" something in the way mmmm ");
			GPIOC->ODR |= (1<<7);
		} else {
			//TransmitString(" keep goin ");
			GPIOC->ODR &= ~(1<<7);
			}
		}
	else if(trig == 14) {
		if (distance < 0.1) {
			TransmitString(" something in the way mmmm ");
			GPIOC->ODR |= (1<<8);
		} else {
			//TransmitString(" keep goin ");
			GPIOC->ODR &= ~(1<<8);
			}
		}
	else if(trig == 10) {
		if (distance < 0.1) {
			TransmitString(" something in the way mmmm ");
			GPIOC->ODR |= (1<<9);
		} else {
			//TransmitString(" keep goin ");
			GPIOC->ODR &= ~(1<<9);
			}
		}
	
	// to "print" the distance
	//char str[100]; 
    //sprintf(str, " distance is: %f ", distance);
	//TransmitString(str);
    pulse_length = 0;	
			
	HAL_Delay(50); // probably change this later.
	
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
	//MotorDir_Counter_Clockwise(PCB2_IN1, PCB2_IN2);
	//MotorDir_Clockwise(PCB3_IN1, PCB3_IN2);
	//MotorDir_Clockwise(PCB4_IN1, PCB4_IN2);
}

void RoverMovement_Reverse(){
	//MotorDir_Clockwise(PCB1_IN1, PCB1_IN2);
	MotorDir_Clockwise(PCB2_IN1, PCB2_IN2);
//	MotorDir_Counter_Clockwise(PCB3_IN1, PCB3_IN2);
//	MotorDir_Counter_Clockwise(PCB4_IN1, PCB4_IN2);
}

void RoverMovement_Left(){
	//MotorDir_Clockwise(PCB1_IN1, PCB1_IN2);
//	MotorDir_Counter_Clockwise(PCB2_IN1, PCB2_IN2);
   MotorDir_Clockwise(PCB3_IN1, PCB3_IN2);
//	MotorDir_Counter_Clockwise(PCB4_IN1, PCB4_IN2);
}

void RoverMovement_Right(){
	//MotorDir_Counter_Clockwise(PCB1_IN1, PCB1_IN2);
//	MotorDir_Clockwise(PCB2_IN1, PCB2_IN2);
//	MotorDir_Counter_Clockwise(PCB3_IN1, PCB3_IN2);
	MotorDir_Clockwise(PCB4_IN1, PCB4_IN2);
}
	
void RoverMovement_Stop(){
	GPIOB->ODR  &=  ~(Mask_For_All_PCB_Address); //turns off signals to logical lows
	//HAL_Delay(1); //comment this out later TODO
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
