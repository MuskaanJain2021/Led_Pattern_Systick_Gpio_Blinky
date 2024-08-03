/*
 * SYSTICK.c
 *
 *  Created on: Aug 2, 2024
 *      Author: muska
 */
#include "systick.h"
#include "core_cm4.h" // CMSIS header for ARM Cortex-M4

// SysTick global counter for tracking elapsed time
volatile uint32_t sysTickCounter = 0;

/**
 * @brief  Initializes the SysTick timer with a specified reload value.
 * @param  ticks: Reload value for the SysTick timer. The timer will overflow
 *                 and generate an interrupt when it counts down to zero.
 * @retval None
 * @note   If SysTick_Config fails (e.g., if ticks is too large), the function
 *         enters an infinite loop to indicate an error.
 */
void SysTick_Init(uint32_t ticks) {
    // Configure the SysTick timer with the specified reload value
    if (SysTick_Config(ticks)) {
        // Handle error if SysTick_Config fails by entering an infinite loop
        while (1);
    }

    // Set the SysTick interrupt priority to the lowest possible value
    NVIC_SetPriority(SysTick_IRQn, (1UL << __NVIC_PRIO_BITS) - 1UL);
}

/**
 * @brief  SysTick interrupt service routine (ISR).
 * @retval None
 * @note   This function is called by the Cortex-M4 core whenever the SysTick
 *         timer generates an interrupt. It increments the global sysTickCounter.
 */
void SysTick_Handler(void) {
    // Increment the global tick counter each time SysTick generates an interrupt
    sysTickCounter++;
}

/**
 * @brief  Gets the current value of the SysTick counter.
 * @retval Current value of the sysTickCounter.
 */
uint32_t SysTick_GetTicks(void) {
    // Return the current tick count
    return sysTickCounter;
}

/**
 * @brief  Busy-wait delay function based on the SysTick timer.
 * @param  delay: Number of ticks to wait. The function will wait until the
 *                 SysTick counter increments by the specified delay value.
 * @retval None
 * @note   This function uses a busy-wait loop to create a delay. It is blocking
 *         and may not be suitable for time-critical applications.
 */
void SysTick_Delay(uint32_t delay) {
    // Record the start time using the current value of the SysTick counter
    uint32_t startTick = SysTick_GetTicks();

    // Wait until the specified delay has elapsed
    while ((SysTick_GetTicks() - startTick) < delay);
}
