/*
 * gpio.c
 *
 *  Created on: Aug 1, 2024
 *      Author: muska
 */
#include "stm32f4xx.h"  // Or the appropriate header file
#include "GPIO.h"       // Ensure GPIO.h is included for the GPIO functions and constants


/**
 * @brief Initializes the GPIO pin with the specified configuration.
 *
 * @param Port Pointer to the GPIO port base address.
 * @param pin GPIO pin number to configure.
 * @param mode GPIO mode (input, output, alternate function, analog).
 * @param output_type GPIO output type (push-pull or open-drain).
 * @param speed GPIO speed (low, medium, high, very high).
 * @param pull GPIO pull-up/pull-down configuration.
 * @param alternate_function GPIO alternate function.
 */
void GPIO_Pin_Init(GPIO_TypeDef *Port, uint8_t pin, uint8_t mode, uint8_t output_type, uint8_t speed, uint8_t pull, uint8_t alternate_function)
{
    GPIO_ClockControl(Port, ENABLE);
    Port->MODER |= mode << (pin * 2);
    Port->OTYPER |= output_type << pin;
    Port->OSPEEDR |= speed << (pin * 2);
    Port->PUPDR |= pull << (pin * 2);

    if (pin < 8) {
        Port->AFR[0] |= alternate_function << (pin * 4); // Set alternate function bits
    } else {
        Port->AFR[1] |= alternate_function << ((pin - 8) * 4); // Set alternate function bits
    }
}

/**
 * @brief Reads data from the specified input pin.
 *
 * @param pGPIOx Pointer to the GPIO peripheral register base address.
 * @param PinNumber GPIO pin number to read from.
 * @return bool Status of the input pin (true = high, false = low).
 */
bool GPIO_ReadFromInputPin(GPIO_TypeDef *pGPIOx, uint8_t PinNumber)
{
    // Read the pin state
    uint8_t value = (uint8_t)((pGPIOx->IDR >> PinNumber) & 0x01);

    // Return true if the pin is high, false if it is low
    return (value == 0x01);
}

/**
 * @brief Writes data to the specified output pin.
 *
 * @param pGPIOx Pointer to the GPIO peripheral register base address.
 * @param PinNumber GPIO pin number to write to.
 * @param Val Value to write (0 or 1).
 */
void GPIO_WriteToOutputPin(GPIO_TypeDef *pGPIOx, uint8_t PinNumber, uint8_t Val)
{
    if (Val == GPIO_PIN_SET)
    {
        // Write 1 to the output data register at the bit position corresponding to the PinNumber
        pGPIOx->ODR |= (1 << PinNumber);
    }
    else
    {
        // Write 0 to the output data register at the bit position corresponding to the PinNumber
        pGPIOx->ODR &= ~(1 << PinNumber);
    }
}

/**
 * @brief Writes data to the specified output port.
 *
 * @param pGPIOx Pointer to the GPIO peripheral register base address.
 * @param Val Value to write to the GPIO port.
 */
void GPIO_WriteToOutputPort(GPIO_TypeDef *pGPIOx, uint16_t Val)
{
    pGPIOx->ODR = Val;
}

/**
 * @brief Configures the interrupt for the specified GPIO pin.
 *
 * @param pin GPIO pin number to configure for the interrupt.
 * @param edge_select Selection of edge (0 = rising, 1 = falling, 2 = both).
 * @param priority Interrupt priority.
 */
void GPIO_Interrupt_Setup(int pin, int edge_select, uint32_t priority)
{
    // Enable interrupt mask for the specified pin
    EXTI->IMR |= (1 << pin);

    // Configure edge selection
    switch (edge_select)
    {
        case 0:
            EXTI->RTSR |= (1 << pin);  // Rising edge
            EXTI->FTSR &= ~(1 << pin); // Ensure falling edge is disabled
            break;
        case 1:
            EXTI->FTSR |= (1 << pin);  // Falling edge
            EXTI->RTSR &= ~(1 << pin); // Ensure rising edge is disabled
            break;
        case 2:
            EXTI->RTSR |= (1 << pin);  // Both edges
            EXTI->FTSR |= (1 << pin);
            break;
        default:
            // Invalid edge selection
            return;
    }

    // Configure NVIC for the specified pin
    IRQn_Type irqNumber;
    if (pin <= 4)
    {
        irqNumber = (IRQn_Type)(EXTI0_IRQn + pin);
    }
    else if (pin <= 9)
    {
        irqNumber = EXTI9_5_IRQn;
    }
    else if (pin <= 15)
    {
        irqNumber = EXTI15_10_IRQn;
    }
    else
    {
        // Invalid pin number
        return;
    }

    NVIC_SetPriority(irqNumber, priority);
    NVIC_EnableIRQ(irqNumber);
}

/**
 * @brief Enables or disables the clock for the specified GPIO port.
 *
 * @param PORT Pointer to the GPIO peripheral register base address.
 * @param EN_DI Enable or disable clock (ENABLE or DISABLE).
 * @return int 1 if successful, -1 if invalid port.
 */
int GPIO_ClockControl(GPIO_TypeDef *PORT, uint8_t EN_DI)
{
    if (EN_DI == ENABLE)
    {
        switch ((uintptr_t)PORT)
        {
            case (uintptr_t)GPIOA:
                RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
                break;
            case (uintptr_t)GPIOB:
                RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
                break;
            case (uintptr_t)GPIOC:
                RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
                break;
            case (uintptr_t)GPIOD:
                RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
                break;
            case (uintptr_t)GPIOE:
                RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;
                break;
            case (uintptr_t)GPIOH:
                RCC->AHB1ENR |= RCC_AHB1ENR_GPIOHEN;
                break;
            default:
                return -1; // Invalid port
        }
    }
    else if (EN_DI == DISABLE)
    {
        switch ((uintptr_t)PORT)
        {
            case (uintptr_t)GPIOA:
                RCC->AHB1ENR &= ~RCC_AHB1ENR_GPIOAEN;
                break;
            case (uintptr_t)GPIOB:
                RCC->AHB1ENR &= ~RCC_AHB1ENR_GPIOBEN;
                break;
            case (uintptr_t)GPIOC:
                RCC->AHB1ENR &= ~RCC_AHB1ENR_GPIOCEN;
                break;
            case (uintptr_t)GPIOD:
                RCC->AHB1ENR &= ~RCC_AHB1ENR_GPIODEN;
                break;
            case (uintptr_t)GPIOE:
                RCC->AHB1ENR &= ~RCC_AHB1ENR_GPIOEEN;
                break;
            case (uintptr_t)GPIOH:
                RCC->AHB1ENR &= ~RCC_AHB1ENR_GPIOHEN;
                break;
            default:
                return -1; // Invalid port
        }
    }
    else
    {
        return -1; // Invalid enable/disable parameter
    }

    return 1;
}
