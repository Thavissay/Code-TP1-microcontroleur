#ifndef BUTTON_H
#define BUTTON_H
#include <stm32l0xx_ll_exti.h>
 
#include "stm32l053xx.h"
#include <stdint.h>
 
/* ===== Structure BUTTON ===== */
typedef struct{
    GPIO_TypeDef* gpioPort;   // Port GPIO (ex : GPIOC)
    uint8_t pin;              // Numéro de pin (ex : 13)
} BUTTON_TypeDef;
 
/* ===== Prototypes ===== */
void Button_init(BUTTON_TypeDef* button, GPIO_TypeDef* port, uint8_t pin, uint8_t pull);
uint8_t Button_state(BUTTON_TypeDef* button);
void Button_enableIRQ(BUTTON_TypeDef* button, uint8_t trigger);
 
#endif