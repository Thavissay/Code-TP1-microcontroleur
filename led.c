#include "led.h"
 
void Led_init(LED_TypeDef* led, GPIO_TypeDef* port, uint8_t pin){
 
    led->gpioPort = port;
    led->pin = pin;
 
    uint8_t nb_port;
    nb_port = ((uint32_t)port - IOPPERIPH_BASE) / 0x400;
 
    RCC->IOPENR |= (1 << nb_port);
 
    led->gpioPort->MODER &= ~(0b11 << (2 * pin));
    led->gpioPort->MODER |=  (0b01 << (2 * pin));
}
 
void Led_turnOn(LED_TypeDef *led){
    led->gpioPort->ODR |= (1 << led->pin);
}
 
void Led_turnOff(LED_TypeDef *led){
    led->gpioPort->ODR &= ~(1 << led->pin);
}
 
void Led_toggle(LED_TypeDef *led){
    led->gpioPort->ODR ^= (1 << led->pin);
}
 
uint8_t Led_isOn(LED_TypeDef *led){
 
    if ( (led->gpioPort->ODR & (1 << led->pin)) != 0 ){
        return 1;
    }
    else{
        return 0;
    }
}
 
uint8_t Led_isOff(LED_TypeDef *led){
 
    if ( (led->gpioPort->ODR & (1 << led->pin)) != 0 ){
        return 0;
    }
    else{
        return 1;
    }
}