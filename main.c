/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "button.h"
#include "led.h"
#include "timer.h"
 
 
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
LED_TypeDef led;
LED_TypeDef led2;
BUTTON_TypeDef button;
BUTTON_TypeDef button2;
uint8_t last_button_state = 0;
uint8_t led2_state;
/* USER CODE END PV */
 
/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void Button_led_polling(uint8_t* last_button_state, LED_TypeDef* led, BUTTON_TypeDef* button);
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
 
  /* USER CODE BEGIN 1 */
 
  /* USER CODE END 1 */
 
  /* MCU Configuration--------------------------------------------------------*/
 
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
   LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);
 
  /* SysTick_IRQn interrupt configuration */
  NVIC_SetPriority(SysTick_IRQn, 3);
 
  /* USER CODE BEGIN Init */
 
  /* USER CODE END Init */
 
  /* Configure the system clock */
  SystemClock_Config();
 
  /* USER CODE BEGIN SysInit */
 
  /* USER CODE END SysInit */
 
  /* Initialize all configured peripherals */
  /* USER CODE BEGIN 2 */
 
  Led_init(&led, GPIOA, 5);
  Button_init(&button, GPIOC, 13, LL_GPIO_PULL_NO);
  Button_enableIRQ(&button, LL_EXTI_TRIGGER_FALLING);
 
  // Partie 1.8 : second bouton
  Button_init(&button2, GPIOB, 3, LL_GPIO_PULL_DOWN);
  Button_enableIRQ(&button2, LL_EXTI_TRIGGER_RISING);
 
  // Partie 2.3 : seconde led
  Led_init(&led2, GPIOC, 7);
  TIM_config(TIM21, 16000000, 3);
  TIM_config(TIM2, 16000000, 2);
  /* USER CODE END 2 */
 
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
 
	  /* Partie 1.4 Clignotement LED */
//	  if(Led_isOn(&led)){
//		  Led_turnOff(&led);
//		  LL_mDelay(500);
//	  }
//	  else if(Led_isOff(&led)){
//		  Led_turnOn(&led);
//		  LL_mDelay(500);
//	  }
 
	  /* Partie 1.5 Utilisation Button par Polling */
//	  last_button_state = Button_state(&button);
//	  Button_led_polling(&last_button_state, &led, &button);
 
	  /* Partie 1.8 Second Bouton */
	  // Simuler un bouton appuyé à travers un pull-up
//	  GPIOB->PUPDR &= ~(0b11<<(2*3));
//	  GPIOB->PUPDR |= (0b01<<(2*3));
//	  LL_mDelay(200);
////	   Simuler un bouton relaché à travers un pull-down
//	  GPIOB->PUPDR &= ~(0b11<<(2*3));
//	  GPIOB->PUPDR |= (0b10<<(2*3));
//	  LL_mDelay(200);
 
	  /* Partie 2.3 Timer led status */
	  led2_state = Led_isOn(&led2);
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}
 
/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_0);
  while(LL_FLASH_GetLatency()!= LL_FLASH_LATENCY_0)
  {
  }
  LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);
  while (LL_PWR_IsActiveFlag_VOS() != 0)
  {
  }
  LL_RCC_HSI_Enable();
 
   /* Wait till HSI is ready */
  while(LL_RCC_HSI_IsReady() != 1)
  {
 
  }
  LL_RCC_HSI_SetCalibTrimming(16);
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSI);
 
   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_HSI)
  {
 
  }
 
  LL_Init1msTick(16000000);
 
  LL_SetSystemCoreClock(16000000);
}
 
/* USER CODE BEGIN 4 */
 
/* Partie 1.6 Polling */
void Button_led_polling(uint8_t* last_button_state, LED_TypeDef* led, BUTTON_TypeDef* button){
	uint8_t current_button_state;
	current_button_state = Button_state(button);
    if (current_button_state && current_button_state != *last_button_state){
        Led_toggle(led);
    }
    *last_button_state = Button_state(button);
    LL_mDelay(100); // Anti Rebond
}
 
/* Partie 1.7 Interrupt */
/* Gestionnaire d’interruption EXTI pour PC13 */
void EXTI4_15_IRQHandler(void)
{
    if (EXTI->PR & (1 << button.pin)){ // Si interruption port 13
        Led_toggle(&led);
        EXTI->PR |= (1 << button.pin); // baisser le drapeau EXTI sur le port
    }
}
 
/* Partie 1.8 Second Bouton */
void EXTI2_3_IRQHandler(void){
    if( EXTI->PR & (1 << button2.pin)){ // Si interruption port 3
    	Led_toggle(&led);
    	EXTI->PR |= (1 << button2.pin); // baisser le drapeau EXTI sur le port
    }
}
 
/* Partie 2.2 Timer interrupt */
void TIM21_IRQHandler(void) {
	Led_toggle(&led);
	// Baisser drapeau interruption
	TIM21->SR = TIM21->SR & ~TIM_SR_UIF_Msk;
}
 
void TIM2_IRQHandler(void) {
	Led_toggle(&led2);
	// Baisser drapeau interruption
	TIM2->SR = TIM2->SR & ~TIM_SR_UIF_Msk;
}
 
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
#ifdef USE_FULL_ASSERT
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