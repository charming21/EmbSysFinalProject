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
uint16_t received_data_LED_color;
uint16_t new_data_flag = 0;
void setup_LEDs(void);
void TransmitChar(char c);
void TransmitString(char* string);
void delay_microsecond(uint32_t us);
void send_pulse(char pin, int trig, int echo);

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
	  RCC->AHBENR |= RCC_AHBENR_GPIOCEN_Msk; 
	  RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	
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
		USART3->BRR = HAL_RCC_GetHCLKFreq() / 115200; 
		USART3->CR1 |= (1<<2 | 1<<3);
		USART3->CR1 |= (1<<0);
			
		
  // sensor 1
	// Configure PA0 (trigger)
		GPIOA->MODER |= (1<<0);
		GPIOA->MODER &= ~(1<<1);

	// Set trigger to low
		GPIOA->ODR &= ~(1<<0);

	// Configure PA1 (echo)
		GPIOA->MODER &= ~(1<<2 | 1<<3);
		GPIOA->PUPDR |= (1<<2);
		GPIOA->PUPDR &= ~(1<<3);
		
  // sensor 2
	// Configure PA4 (trigger)
		GPIOA->MODER |= (1<<8);
		GPIOA->MODER &= ~(1<<9);

	// Set trigger to low
		GPIOA->ODR &= ~(1<<4);

	// Configure PA5 (echo)
		GPIOA->MODER &= ~(1<<10 | 1<<11);
		GPIOA->PUPDR |= (1<<10);
		GPIOA->PUPDR &= ~(1<<11);
	
  while (1)
  {
		send_pulse('A', 0, 1);
		//send_pulse('A', 4, 5);
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
  	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
  	TIM2->PSC = 79;
  	TIM2->ARR = us - 1;
  	TIM2->DIER |= TIM_DIER_UIE; //enable interrupt
  	TIM2->SR &= ~TIM_SR_UIF;
  	TIM2->CR1 |= TIM_CR1_CEN; //enable timer
	
  // reset timer and wait for it to reach ARR value
    TIM2->CNT = 0;
  	while ((TIM2->SR & TIM_SR_UIF) == 0); 
  	TIM2->SR &= ~TIM_SR_UIF; 
  	TIM2->CR1 &= ~TIM_CR1_CEN; //disble timer
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
				
  // distance in cm
    distance = pulse_length * 0.034 / 2;
			
  // check which sensor we're measuring
	if(trig == 0) {
		if (distance < 0.1) {
			TransmitString(" something in the way mmmm ");
			GPIOC->ODR |= (1<<6);
		} else {
			TransmitString(" keep goin ");
			GPIOC->ODR &= ~(1<<6);
			}
	}
	else {
		if (distance < 0.1) {
			TransmitString(" something in the way mmmm ");
			GPIOC->ODR |= (1<<7);
		} else {
			TransmitString(" keep goin ");
			GPIOC->ODR &= ~(1<<7);
			}
			}
	// to "print" the distance
	//char str[100]; 
    //sprintf(str, " distance is: %f ", distance);
	//TransmitString(str);
    pulse_length = 0;	
	HAL_Delay(100); // probably change this later.
	
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
