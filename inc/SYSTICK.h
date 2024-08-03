/*
 * SYSTICK.h
 *
 *  Created on: Aug 2, 2024
 *      Author: muska
 */


#ifndef SYSTICK_H_
#define SYSTICK_H_

#include "stm32f407xx.h"
#include <stdint.h>

// Function prototypes
void SysTick_Init(uint32_t ticks);
void SysTick_Handler(void);
uint32_t SysTick_GetTicks(void);
void SysTick_Delay(uint32_t delay);

#endif /* SYSTICK_H_ */
