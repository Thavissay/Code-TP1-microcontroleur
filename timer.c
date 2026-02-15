#include "timer.h"
 
/**
* @brief  Configure un timer générique (TIM2, TIM21...)
* @note   Basé sur les consignes du TP1 Partie 2.1 [cite: 175-193]
*/
void TIM_config(TIM_TypeDef *timer, uint32_t HCLKFrequency, uint8_t nb_s) {
 
    /* 1. Activer l'horloge du timer [cite: 182-184] */
    // TIM2 et TIM6 sont sur APB1
    if (timer == TIM2) {
        LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM2);
    }
    // TIM21 et TIM22 sont sur APB2
    else if (timer == TIM21) {
        LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM21);
    }
 
    /* 2. Configurer le timer en décompteur  */
    LL_TIM_SetCounterMode(timer, LL_TIM_COUNTERMODE_DOWN);
 
    /* 3. Affecter la valeur 999 au Prescaler (PSC) [cite: 188-189]
     * Cela divise la fréquence HCLK par 1000 (PSC + 1).
     * Si HCLK = 16 MHz, le Timer tourne à 16 kHz.
     */
    LL_TIM_SetPrescaler(timer, 999);
 
    /* 4. Calcul de l'Auto-Reload (ARR) [cite: 190]
     * Formule du TP : ARR = (Freq * nb_s) / (PSC + 1) - 1
     * Ici : (16000000 * nb_s) / 1000 - 1
     */
    uint32_t arr_value = (HCLKFrequency * nb_s) / 1000 - 1;
 
    /* PROTECTION : Les timers TIM2/TIM21 sont sur 16 bits (Max 65535).
     * Avec PSC=999, 1 seconde = 16000 ticks.
     * Max possible = 65535 / 16000 = 4 secondes.
     */
    if (arr_value > 65535) {
        arr_value = 65535; // Plafonne à ~4s pour éviter le bug
    }
 
    timer->ARR = arr_value;
 
    /* 5. Activation de l'interruption */
    timer->DIER |= TIM_DIER_UIE;
 
    /* 6. Activation dans le NVIC [cite: 192] */
    if (timer == TIM2) {
        NVIC_SetPriority(TIM2_IRQn, 0);
        NVIC_EnableIRQ(TIM2_IRQn);
    }
    else if (timer == TIM21) {
        NVIC_SetPriority(TIM21_IRQn, 0);
        NVIC_EnableIRQ(TIM21_IRQn);
    }
 
    /* 7. Lancement du timer [cite: 193] */
    timer->CR1 |= TIM_CR1_CEN;
}