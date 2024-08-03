# Project Title: Traffic Light Simulation using STM32F407
## Project Overview:


 This project demonstrates a traffic light simulation using the STM32F407 microcontroller and its onboard LEDs. The simulation controls the green, red, and orange LEDs to emulate different traffic light states and patterns, providing a practical exercise in GPIO configuration, SysTick timer usage, and interrupt handling in embedded systems. The project includes individual LED blinking, combinations of LEDs, multi-LED patterns, and sequential LED patterns, all with precise timing managed by the SysTick timer. This project aims to enhance understanding of embedded programming fundamentals and real-time system operations.

## Project Goals:

>> Individual LED Blinking:

Green LED: Blink on and off every 1 second.
Red LED: Blink on and off every 1 second.
Orange LED: Blink on and off every 1 second.
 Combinations of LEDs:

Green + Orange: Both LEDs turn on together for 1 second, then turn off together for 1 second.
Green + Red: Both LEDs turn on together for 1 second, then turn off together for 1 second.
Red + Orange: Both LEDs turn on together for 1 second, then turn off together for 1 second.
Multi-LED Pattern:

Green + Orange + Red: All three LEDs turn on together for 2 seconds, then turn off together for 2 seconds.
>> Sequential LED Pattern:

Green → Red → Orange: Turn on each LED sequentially with 2-second intervals, and turn them off in the same sequence.
## Components:

-STM32F407VGT6 microcontroller
-Onboard LEDs (Green, Red, Orange)
# Project Structure:

## GPIO Configuration:

-Initialize the GPIO pins for the onboard LEDs.
-Configure the GPIO pins as output using the GPIO_Pin_Init function.
## SysTick Timer:

-Initialize the SysTick timer to generate interrupts every 1 ms.
-Implement a delay function using the SysTick timer.
-Create an interrupt handler for SysTick to keep track of time.
## Main Application:

-Implement the traffic light sequences and patterns using the delay function.
-Control the LEDs based on the specified sequences.
#Expected Outcomes:

=>By completing this project, you will gain hands-on experience with:

-Configuring and controlling GPIO pins.

-Using the SysTick timer for generating precise delays.

-Writing modular and efficient embedded software.

-Understanding and handling interrupts in a real-time system.
