#include "button.h"
 
void Button_init(BUTTON_TypeDef* button, GPIO_TypeDef* port, uint8_t pin, uint8_t pull){
 
    button->gpioPort = port;
    button->pin = pin;
 
 
    uint8_t nb_port;
    nb_port = ((uint32_t)port - IOPPERIPH_BASE) / 0x400;
    RCC->IOPENR |= (1 << nb_port);
 
 
    port->MODER &= ~(0b11 << (2 * pin));   // 00 = input
 
    port->PUPDR &= ~(0b11 << (2 * pin));
    port->PUPDR |=  (pull << (2 * pin));
}
 
 
uint8_t Button_state(BUTTON_TypeDef* button){
 
    if ( (button->gpioPort->IDR & (1 << button->pin)) != 0 ){
        return 1;
    }
    else{
        return 0;
    }
}
 
void Button_enableIRQ(BUTTON_TypeDef* button, uint8_t trigger)
{
    /* 1. Activer l’horloge du SYSCFG */
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
 
    /* 2. Déterminer le registre EXTICR à utiliser */
    uint8_t nb_EXTI = button->pin / 4;
 
    /* 3. Déterminer le numéro du port GPIO */
    uint8_t nb_port;
    nb_port = ((uint32_t)button->gpioPort - IOPPERIPH_BASE) / 0x400;
 
    /* 4. Configuration du registre EXTICR */
    SYSCFG->EXTICR[nb_EXTI] &= ~(0b1111 << (4 * (button->pin % 4)));
    SYSCFG->EXTICR[nb_EXTI] |=  (nb_port << (4 * (button->pin % 4)));
 
    /* 5. Activation de l’EXTI dans l’IMR */
    EXTI->IMR |= (1 << button->pin);
 
    /* 6. Configuration du trigger */
    switch (trigger)
    {
        case LL_EXTI_TRIGGER_RISING:
            EXTI->RTSR |=  (1 << button->pin);
            EXTI->FTSR &= ~(1 << button->pin);
            break;
 
        case LL_EXTI_TRIGGER_FALLING:
            EXTI->FTSR |=  (1 << button->pin);
            EXTI->RTSR &= ~(1 << button->pin);
            break;
 
        case LL_EXTI_TRIGGER_RISING_FALLING:
            EXTI->RTSR |= (1 << button->pin);
            EXTI->FTSR |= (1 << button->pin);
            break;
 
        default:
            break;
    }
 
    /* 7. Activation du vecteur NVIC */
    if (button->pin < 2)
    {
        NVIC_EnableIRQ(EXTI0_1_IRQn);
        NVIC_SetPriority(EXTI0_1_IRQn, 0);
    }
    else if (button->pin < 4)
    {
        NVIC_EnableIRQ(EXTI2_3_IRQn);
        NVIC_SetPriority(EXTI2_3_IRQn, 0);
    }
    else
    {
        NVIC_EnableIRQ(EXTI4_15_IRQn);
        NVIC_SetPriority(EXTI4_15_IRQn, 0);
    }
}