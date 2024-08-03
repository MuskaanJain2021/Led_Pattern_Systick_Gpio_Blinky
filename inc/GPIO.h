/*
 * GPIO.h
 *
 *  Created on: Aug 1, 2024
 *      Author: muska
 */

/*
 * GPIO.h
 *
 *  Created on: Aug 1, 2024
 *      Author: muska
 */

#ifndef GPIO_H_
#define GPIO_H_

#include "stm32f407xx.h"   // MCU specific header file
#include <stdbool.h>
#include <stdint.h>
#define GPIO_PIN_SET 1
#define GPIO_PIN_RESET 0

/* Enumeration for GPIO modes */
typedef enum {
    GPIO_MODE_INPUT = 0,
    GPIO_MODE_GENERAL_PURPOSE_OUTPUT,
    GPIO_MODE_ALTERNATE_FUNCTION,
    GPIO_MODE_ANALOG
} GPIO_Mode_t;

/* Enumeration for GPIO output types */
typedef enum {
    GPIO_OUTPUT_TYPE_PUSH_PULL = 0,
    GPIO_OUTPUT_TYPE_OPEN_DRAIN
} GPIO_OutputType_t;

/* Enumeration for GPIO speeds */
typedef enum {
    GPIO_SPEED_LOW = 0,
    GPIO_SPEED_MEDIUM,
    GPIO_SPEED_HIGH,
    GPIO_SPEED_VERY_HIGH
} GPIO_Speed_t;

/* Enumeration for GPIO pull-up/pull-down configurations */
typedef enum {
    GPIO_PULL_NO = 0,
    GPIO_PULL_UP,
    GPIO_PULL_DOWN
} GPIO_Pull_t;

/* Enumeration for GPIO interrupt edges */
typedef enum {
    GPIO_INTERRUPT_EDGE_RISING = 0,
    GPIO_INTERRUPT_EDGE_FALLING,
    GPIO_INTERRUPT_EDGE_RISING_FALLING
} GPIO_InterruptEdge_t;

/* Enumeration for alternate functions */
typedef enum {
    GPIO_AF_NONE = 0,
    GPIO_AF_ANALOG,
    GPIO_AF_SYS,
    GPIO_AF_MCO1,
    GPIO_AF_MCO2,
    GPIO_AF_RTC_REFIN,
    GPIO_AF_TIM1,
    GPIO_AF_TIM2,
    GPIO_AF_TIM3,
    GPIO_AF_TIM4,
    GPIO_AF_TIM5,
    GPIO_AF_TIM8,
    GPIO_AF_TIM9,
    GPIO_AF_TIM10,
    GPIO_AF_TIM11,
    GPIO_AF_I2C1,
    GPIO_AF_I2C2,
    GPIO_AF_I2C3,
    GPIO_AF_SPI1,
    GPIO_AF_SPI2,
    GPIO_AF_I2S2,
    GPIO_AF_I2S2EXT,
    GPIO_AF_SPI3,
    GPIO_AF_I2S_EXT,
    GPIO_AF_I2S3,
    GPIO_AF_USART1,
    GPIO_AF_USART2,
    GPIO_AF_USART3,
    GPIO_AF_I2S3EXT,
    GPIO_AF_USART4,
    GPIO_AF_UART5,
    GPIO_AF_USART6,
    GPIO_AF_CAN1,
    GPIO_AF_CAN2,
    GPIO_AF_TIM12,
    GPIO_AF_TIM13,
    GPIO_AF_TIM14,
    GPIO_AF_OTG_FS1,
    GPIO_AF_OTG_HS1,
    GPIO_AF_ETH1,
    GPIO_AF_FSMC1,
    GPIO_AF_SDIO1,
    GPIO_AF_OTG_FS2,
    GPIO_AF_DCMI1,
    GPIO_AF_EVENTOUT
} GPIO_AlternateFunction_t;

/* GPIO Pin Configuration Structure */
typedef struct {
    uint8_t GPIO_PinNumber;          // GPIO pin number
    GPIO_Mode_t GPIO_PinMode;        // GPIO mode
    GPIO_OutputType_t GPIO_PinOType; // GPIO output type
    GPIO_Speed_t GPIO_PinSpeed;      // GPIO speed
    GPIO_Pull_t GPIO_PinPuPdControl; // GPIO pull-up/pull-down
    GPIO_AlternateFunction_t GPIO_PinAltFunMode; // GPIO alternate function mode
} GPIO_PinConfig_t;

/* GPIO Handle Structure */
typedef struct {
    GPIO_TypeDef *pGPIOx;            // Pointer to GPIO port base address
    GPIO_PinConfig_t GPIO_PinConfig; // GPIO pin configuration settings
} GPIO_Handle_t;

/* API Functions Supported by GPIO Driver */
int GPIO_ClockControl(GPIO_TypeDef *PORT, uint8_t EN_DI);
void GPIO_Pin_Init(GPIO_TypeDef *Port, uint8_t pin, uint8_t mode, uint8_t output_type, uint8_t speed, uint8_t pull, uint8_t alternate_function);
bool GPIO_ReadFromInputPin(GPIO_TypeDef *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_TypeDef *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_TypeDef *pGPIOx, uint8_t PinNumber, uint8_t Val);
void GPIO_WriteToOutputPort(GPIO_TypeDef *pGPIOx, uint16_t Val);
void GPIO_ToggleOutputPin(GPIO_TypeDef *pGPIOx, uint8_t PinNumber);
void GPIO_Interrupt_Setup(int pin, int edge_select, uint32_t priority);

#endif /* GPIO_H_ */
