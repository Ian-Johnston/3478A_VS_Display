/**
  ******************************************************************************
  * @file    timer.h
  * @brief   This file contains all the function prototypes for
  *          the timer.c file
  ******************************************************************************
*/

#ifndef TIMER_H
#define TIMER_H

#include "stm32f1xx.h" // Include the STM32 HAL/LL header file
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_tim.h" // For TIM APIs and TIM_HandleTypeDef

//***********************************************************************************
// Timer 2

// Externally accessible variables for TIM2
extern volatile uint8_t timer_flag;
extern volatile uint8_t task_ready;

extern volatile uint8_t isaBuffer[256];
extern volatile uint8_t inaBuffer[256];

extern volatile uint8_t logReady;

// Function prototypes
void TIM2_Init(void);
void TIM2_IRQHandler(void);
void SetTimerDuration(uint16_t ms);
void DMM_HandleSyncState(void);

extern volatile char displayStr[13];
extern volatile char punctStr[13];
extern volatile char displayWithPunct[32];
extern volatile uint8_t Annunc[13];

//***********************************************************************************
// Timer 3

// External TIM3 handle
extern TIM_HandleTypeDef htim3;

// External SYNC state variable
extern volatile uint8_t syncState;

void DMM_HandleO2Clock(void);
void MX_TIM3_Init(void);

// Externally accessible variables for TIM3
extern volatile uint16_t captured_value;
extern volatile uint8_t tim3_capture_flag;

// Function prototypes for TIM3
void TIM3_Init_InputCapture(void);
void TIM3_IRQHandler(void);
uint16_t TIM3_GetCapturedValue(void);


#endif // TIMER_H


