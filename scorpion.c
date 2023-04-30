
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
 /**
  * Updated April 25,2023
  * For Embedded System Final 
  * Our Rover Name is Scorpion
  * Our Team members : Brittney Morales, Saoud Aldowaish, Zander Bagley
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdio.h>
#include <string.h>

/* Private function for USART ---------------------------------------------------------*/
void setup_LEDs(void);
void TransmitChar(char c);
void TransmitString(char* string);
void USART3_4_IRQHandler(void);

/* Private function for Ultrasonic Sensors ---------------------------------------------------------*/
void delay_microsecond(uint32_t us);
void send_pulse(char pin, int trig, int echo);

/* Private function For Motor Direction Logic -----------------------------------------------*/
void MotorDir_Clockwise(uint32_t, uint32_t);
void MotorDir_Counter_Clockwise(uint32_t, uint32_t);
void RoverMovement_Forward();
void RoverMovement_Reverse();
void RoverMovement_Left();
void RoverMovement_Right();
void RoverMovement_Stop();
void Test_Forward(); 	//Used for testing direction of motors
void Test_Reverse(); 	//Used for testing direction of motors
void Test_Left(); 		//Used for testing direction of motors
void Test_Right();		//Used for testing direction of motors
void Test_Stop(); 		//Used for testing direction of motors

/* Private function for Driving Rover ---------------------------------------------------------*/
void Forward();
void Left();
void Right();
void Stop();
void Back();


/* Private Variables ---------------------------------------------------------*/
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
	
//***********Global Variable for Rover ******************//
volatile char _RX_flag = 0;
volatile char _RX_dir = 'x';
volatile char curr_dir = 1;

int autonomous_mode = 0;
int auto_mode_front_left_sensor_object = 0;
int auto_mode_front_sensor_object = 0;
int auto_mode_front_right_sensor_object = 0;
int auto_mode_right_sensor_object = 0;
int auto_mode_left_sensor_object = 0;
int rover_was_moving_right = 0;
int rover_was_moving_left = 0;
int distance_counter = 0;
	
/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  HAL_Init();
  SystemClock_Config();

  // RCC Enable use of periphals
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN; //enable A pins
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN; //enable B pins
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN; //enable C pins
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN; //enable TIM2
	
////**********SETUP for USART ***************************//
// setup USART 
	// PC4 TX blue wire
	// Set mode to alternate function
	GPIOC->MODER |= (1<<9);
	GPIOC->MODER &= ~(1<<8);
		
  // PC5 RX green wire
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
////**********END SETUP for USART ***************************//

////**********SETUP for Ultrasonic Sensor 1-5 ***************************//
  // sensor 1
	// Configure PA4 (trigger)
		GPIOA->MODER |= (1<<8);
	// sensor 2
	// Configure PA8 (trigger)
		GPIOA->MODER |= (1<<16);
	// sensor 3
	// Configure PA10 (trigger)
		GPIOA->MODER |= (1<<20);
	// sensor 4
	// Configure PC8 (trigger)
		GPIOC->MODER |= (1<<16); 
	// sensor 5
	// Configure PC6 (trigger)
		GPIOC->MODER |= (1<<12); 
////**********END of SETUP for Ultrasonic Sensor 1-5 ***************************//

////**********SETUP A pins 0,1 for PWM ***************************//
	GPIOA->MODER |= GPIO_MODER_MODER0_1 | GPIO_MODER_MODER1_1 ;//| GPIO_MODER_MODER2_1 | GPIO_MODER_MODER3_1 ; // Set to Alternate function output mode (bits: 10)
	GPIOA->AFR[0] |= 0x2 | 0x2<<4 ; //| 0x2 << 8 | 0x2 << 12; //PA0,1,2,3 alternate function AF2

//**********END of SETUP A pins 0,1 ********************//
	
//**********SETUP B pins 2,6-9,12-15 for logic high/low ****************//
	//remove gpio_moder_moder3_0
	GPIOB->MODER |= GPIO_MODER_MODER2_0 | GPIO_MODER_MODER3_0 | GPIO_MODER_MODER6_0|GPIO_MODER_MODER7_0|GPIO_MODER_MODER8_0|GPIO_MODER_MODER9_0 | GPIO_MODER_MODER12_0| GPIO_MODER_MODER13_0| GPIO_MODER_MODER14_0| GPIO_MODER_MODER15_0; // Set to General pupose output mode (bits: 01)
	GPIOA->MODER |= 0x1<<4;; // testing
//**********SETUP B pins 2,6-9,12-15  ****************//

//**********SETUP B pins 10,11 for PWM****************//
	GPIOB->MODER |= GPIO_MODER_MODER10_1 | GPIO_MODER_MODER11_1 ; //Set to Alternate function output mode (bits: 10)
	GPIOB->AFR[1] |= 0x2<<8 | 0x2<<12 ;//PB10,11 alternate function AF2
//**********END of SETUP B pins 4,5,10,11 ****************//

/******************SETUP PWM TIMER 2 (PA0,1,PB10,11 channel 1,2,3,4) ***********************/
	TIM2 -> PSC = (99); 														
	TIM2 -> ARR = (100); //100*100 = 10,000 desired f = 800Hz		
	TIM2 -> DIER = 0; //RESET inturrupts to disable on all cases

	TIM2 -> CCMR1 |= (0b00 << 8);	// set channel 2 to output mode
	TIM2 -> CCMR1 |= (0b00);	    // set channel 1 to output mode
	TIM2 -> CCMR2 |= (0b00 << 8);	// set channel 4 to output mode
	TIM2 -> CCMR2 |= (0b00);	    // set channel 3 to output mode 
	TIM2 -> CCMR1 |= (6 << 4);   	// set OC1M to PWM mode 1	
  	TIM2 -> CCMR1 |= (6 << 12);   	// set OC2M to PWM mode 1	
	TIM2 -> CCMR2 |= (6 << 4);   	// set OC3M to PWM mode 1	
	TIM2 -> CCMR2 |= (6 << 12);   	// set OC4M to PWM mode 1	
	TIM2 -> CCMR1 |= (1 << 3) | (1 << 11); // enable channel 1 & 2 preload
	TIM2 -> CCMR2 |= (1 << 3) | (1 << 11); // enable channel 3 & 4 preload
	TIM2 -> CCER |= (1); 		// channel 1 output enable 
	TIM2 -> CCER |= (1 << 4); 	// channel 2 output enable 
	TIM2 -> CCER |= (1 << 8); 	// channel 3 output enable 
	TIM2 -> CCER |= (1 << 12); 	// channel 4 output enable 
	TIM2 -> CCR1 = (35); // Heavy Duty for channel 1
	TIM2 -> CCR2 = (36); // Heavy Duty for Channel 2
	TIM2 -> CCR3 = (40); // Heavy Duty for channel 3
	TIM2 -> CCR4 = (40); // Heavy Duty for Channel 4
	
	TIM2 -> CR1 |= (1); // timer enable
	
/******************END OF SETUP PWM TIMER 2 (PA0,1,PB10,11 channel 1,2,3,4) ***********************/

	//setting standby high PB2
	GPIOB->ODR  |= 0x1<<2;
	HAL_Delay(100);
	
	TransmitString("CMD?");
	
  while (1)
  {
		// senssors stuff don't use if sensors are not connected
   		send_pulse('A', 4, 5);
   		send_pulse('A', 8, 9);
  		send_pulse('A', 10, 13);
		send_pulse('C', 8, 9);
		send_pulse('C', 6, 7);
		
		if(autonomous_mode == 1) {
			if (auto_mode_front_left_sensor_object == 0 && auto_mode_front_sensor_object == 0 && auto_mode_front_right_sensor_object == 0) { // no object infront of rover
				if (rover_was_moving_left == 1) {
					HAL_Delay(500);
					rover_was_moving_left = 0;
				}
				if (rover_was_moving_right == 1) {
					HAL_Delay(500);
					rover_was_moving_right = 0;
				}
				Forward();
			} 
			
		   if (auto_mode_front_left_sensor_object == 1 || auto_mode_front_sensor_object == 1) { // if there is an object infornt or front left of the rover
				if (auto_mode_right_sensor_object == 0 && rover_was_moving_left == 0 ) { // if there is no object to the right and the rover wasn't moving left
					Right();
					rover_was_moving_right = 1;
				}
				else if (auto_mode_left_sensor_object == 0) { // if there is no object to the left of the rover 
					Left();
					rover_was_moving_left = 1;
					rover_was_moving_right = 0; 
				} 
			}
		    else if (auto_mode_front_right_sensor_object == 1) { // if there is an object front right of the rover
				if (auto_mode_left_sensor_object == 0 && rover_was_moving_right == 0 ) {  // if there is no object to the left and the rover wasn't moving right
					Left();
					rover_was_moving_left = 1;
				}
				else if (auto_mode_right_sensor_object == 0) {  // if there is no object to the right of the rover 
					Right();
					rover_was_moving_right = 1;
					rover_was_moving_left = 0; 
				} 
			}		
		}
	}
		
}

/**
 * @brief Stops the Rover's movements and sets the current direction to 'x'
 * 
 */
void Stop(){
	if(autonomous_mode == 0) {
	curr_dir = 'x';
	// all off 
	RoverMovement_Stop();
	}
}
/**
 * @brief Moves the rover to the Right and sets the current direction to 'd'
 * 
 */
void Right(){
	curr_dir = 'd';
	RoverMovement_Right();
}
/**
 * @brief Moves the rover to the Left and sets the current direction to 'a'
 * 
 */
void Left(){
	curr_dir = 'a';
	RoverMovement_Left();
}
/**
 * @brief Moves the rover Forward and sets the current direction to 'w'
 * 
 */
void Forward(){
	curr_dir = 'w';
	RoverMovement_Forward();						
}
/**
 * @brief Moves the rover in Reverse and sets the current direction to 's'
 * 
 */
void Back(){
	curr_dir = 's';
	RoverMovement_Reverse();
}

/**
 * @brief Interrupt handler when recieving a string of characters via usart connection. Grabs input and set the direction of the rover. 
 * And transmit a string via string which direction the rover is going.
 * 
 */
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
			
			if(_RX_dir == 'w' && autonomous_mode == 0) {
				Stop();
				Forward();
				TransmitString("MOVING FORWARD");
			}
			else if(_RX_dir == 'a' && autonomous_mode == 0) {
				Stop();
				Left();
				TransmitString("MOVING LEFT");
			}
			else if(_RX_dir == 'd' && autonomous_mode == 0) {
				Stop();
				Right();
				TransmitString("MOVING RIGHT");
			}
			else if(_RX_dir == 's' && autonomous_mode == 0) {
				Stop();
				Back();
				TransmitString("MOVING BACK");
			}
			else if(_RX_dir == 'x' && autonomous_mode == 0){
				Stop();
				TransmitString("STOPPED");
			}
			else if(_RX_dir == 't' && autonomous_mode == 0){
				autonomous_mode = 1;
				distance_counter = 0;
				TransmitString(" Entered Autonomous Mode");
			}
			if(_RX_dir == 'e' && autonomous_mode == 1){
				autonomous_mode = 0;
				Stop();
				TransmitString(" Exit Autonomous Mode");
			}
	}
		//TransmitString("\n\r");
		_RX_flag = 0;
	}
}

/*
code for setting up the LEDs not needed because we are using the pins for the sensors
*/
void setup_LEDs(void) {
	
	// PC6 RED LED
	// Set mode to general-purpose 
	//	GPIOC->MODER |= (1<<12);
	//	GPIOC->MODER &= ~(1<<13);
	// Set type to push-pull
		GPIOC->OTYPER &= ~(1<<6);
	// Set speed to low
		GPIOC->OSPEEDR &= ~(1<<12);
	// Set PUPDR to no pull-up/down
		GPIOC->PUPDR &= ~(1<<12 | 1<<13);
		
	// PC7 BLUE LED
	// Set mode to general-purpose 
	//  GPIOC->MODER |= (1<<14);
	//	GPIOC->MODER &= ~(1<<15);
	// Set type to push-pull
		GPIOC->OTYPER &= ~(1<<7);
	// Set speed to low
		GPIOC->OSPEEDR &= ~(1<<14);
	// Set PUPDR to no pull-up/down
		GPIOC->PUPDR &= ~(1<<14 | 1<<15);
			
	// PC8 ORANGE LED
	// Set mode to general-purpose 
	//	GPIOC->MODER |= (1<<16);
	//	GPIOC->MODER &= ~(1<<17);
	// Set type to push-pull
		GPIOC->OTYPER &= ~(1<<8);
	// Set speed to low
		GPIOC->OSPEEDR &= ~(1<<16);
	// Set PUPDR to no pull-up/down
		GPIOC->PUPDR &= ~(1<<16 | 1<<17);
		
	// PC9 GREEN LED
	// Set mode to general-purpose 
	//	GPIOC->MODER |= (1<<18);
	//	GPIOC->MODER &= ~(1<<19);
	// Set type to push-pull
		GPIOC->OTYPER &= ~(1<<9);
	// Set speed to low
		GPIOC->OSPEEDR &= ~(1<<18);
	// Set PUPDR to no pull-up/down
		GPIOC->PUPDR &= ~(1<<18 | 1<<19);

}
/**
 * @brief Transmitting a character at a time via usart connection
 * 
 * @param c 
 */
void TransmitChar(char c) {
	while(1) {
		if(USART3->ISR & (1<<7)) {
			break;
		}
	}
	USART3->TDR = c; 
}
/**
 * @brief Transmitting a string of characters via usart connection
 * 
 * @param string 
 */
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
		GPIO_pin = GPIOC;
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
				
  // distance in cm (not accurate)
    distance = pulse_length * 0.034 / 2;
	
	// check which sensor we're measuring
 	if(trig == 8 && pin == 'A') { //front left sensor 
		if (distance < 0.1) { 
			TransmitString(" object in range of sensor 1 front left PA8");
			auto_mode_front_left_sensor_object = 1;
			if (curr_dir == 'w') {
			Stop();				
			}
		} else {
			auto_mode_front_left_sensor_object = 0;
			}
	}
	if(trig == 8 && pin == 'C') { //front sensor 
		if (distance < 0.1) { 
			TransmitString(" object in range of sensor 5 front PC8 ");
			auto_mode_front_sensor_object = 1;
			if (curr_dir == 'w') {
			Stop();				
			}
		} else {
			auto_mode_front_sensor_object = 0;
			}
	}
	if(trig == 6) { //front right sensor 
		if (distance < 0.1) { 
			TransmitString(" object in range of sensor 4 front right PC6 ");
			auto_mode_front_right_sensor_object = 1;
			if (curr_dir == 'w') {
			Stop();				
			}
		} else {
			auto_mode_front_right_sensor_object = 0;
			}
	}
	else if(trig == 10) { //right sensor
		if (distance < 0.1) {
			TransmitString(" object in range of sensor 2 (right) PA10 ");
			auto_mode_right_sensor_object = 1;
			if (curr_dir == 'd') { 
			Stop();
			}
		} else {
			auto_mode_right_sensor_object = 0;
			}
		}
	else if(trig == 4) { //left sensor
		if (distance < 0.1) {
			TransmitString(" object in range of sensor 3 (left) PA4 ");
			auto_mode_left_sensor_object = 1;
		} else {
			auto_mode_left_sensor_object = 0;
			}
		}
		
	// to "print" the distance for testing no longer needed
//	 char str[100]; 
//   sprintf(str, " distance is: %f ", distance);
//	 TransmitString(str);
		
    pulse_length = 0;			
	//HAL_Delay(50); // Used for testing  
	
}

/**
 * @brief Sets GPIO pinIN2 high and sets GIO pinIN1 to low 
 * and changes the direction of the wheel to turn counter-clockwise
 * 
 * @param pinIN1 represents PCB IN1
 * @param pinIN2 represents PCB IN2
 */
void MotorDir_Counter_Clockwise(uint32_t pinIN1, uint32_t pinIN2){
	GPIOB->ODR |= pinIN2;
	GPIOB->ODR &= ~(pinIN1);
}

/**
 * @brief Sets GPIO pinIN2 Low and sets GIO pinIN1 to High 
 * and changes the direction of the wheel to turn clockwise
 * 
 * @param pinIN1 represents PCB IN1
 * @param pinIN2 represents PCB IN2
 */
void MotorDir_Clockwise(uint32_t pinIN1, uint32_t pinIN2){
	GPIOB->ODR |= pinIN1;
	GPIOB->ODR &= ~(pinIN2);
}
/**
 * @brief Sets PCB 1 and 2 to turn clockwise and PCB 3 and 4 to turn counter-clockwise.
 * Getting the rover to move in reverse.
 * 
 */
void RoverMovement_Reverse(){
	MotorDir_Counter_Clockwise(PCB1_IN1, PCB1_IN2);
	MotorDir_Counter_Clockwise(PCB2_IN1, PCB2_IN2);
	MotorDir_Clockwise(PCB3_IN1, PCB3_IN2);
	MotorDir_Clockwise(PCB4_IN1, PCB4_IN2);
}
/**
 * @brief Sets PCB 1 and 2 to turn counter-clockwise and PCB 3 and 4 to turn clockwise.
 * Getting the rover to move forward.
 * 
 */
void RoverMovement_Forward(){
	MotorDir_Clockwise(PCB1_IN1, PCB1_IN2);
	MotorDir_Clockwise(PCB2_IN1, PCB2_IN2);
	MotorDir_Counter_Clockwise(PCB3_IN1, PCB3_IN2);
	MotorDir_Counter_Clockwise(PCB4_IN1, PCB4_IN2);
}

/**
 * @brief Sets PCB 1 and 3 to turn clockwise and PCB 2 and 4 to turn counter-clockwise.
 * Getting the rover to move Left.
 * 
 */
void RoverMovement_Left(){
	MotorDir_Clockwise(PCB1_IN1, PCB1_IN2);
	MotorDir_Counter_Clockwise(PCB2_IN1, PCB2_IN2);
  MotorDir_Clockwise(PCB3_IN1, PCB3_IN2);
	MotorDir_Counter_Clockwise(PCB4_IN1, PCB4_IN2);
}

/**
 * @brief Sets PCB 1 and 3 to turn counter-clockwise and PCB 2 and 4 to turn clockwise.
 * Getting the rover to move Right.
 * 
 */
void RoverMovement_Right(){
	MotorDir_Counter_Clockwise(PCB1_IN1, PCB1_IN2);
	MotorDir_Clockwise(PCB2_IN1, PCB2_IN2);
	MotorDir_Counter_Clockwise(PCB3_IN1, PCB3_IN2);
	MotorDir_Clockwise(PCB4_IN1, PCB4_IN2);
}

/**
 * @brief Sets all the pins related to the PCB1, 2, 3,and 4 to LOW
 * 
 */
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
