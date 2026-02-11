/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
  ******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "gpio.h"
#include "spi.h"
#include "lt7680.h"
#include "main.h"

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/

void MX_GPIO_Init(void) {

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    __HAL_RCC_TIM3_CLK_ENABLE();  // Enable clock for TIM3
    __HAL_RCC_AFIO_CLK_ENABLE();  // Enable clock for Alternate Function IO
	

    // LED - Test output pin
    /* Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(TEST_OUT_GPIO_Port, TEST_OUT_Pin, GPIO_PIN_RESET);
    /* Configure GPIO pin : TEST_OUT_Pin */
    GPIO_InitStruct.Pin = TEST_OUT_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(TEST_OUT_GPIO_Port, &GPIO_InitStruct);


    // LT7680A-R Reset
    /* Configure GPIO pin for LT7680 RESET */
    GPIO_InitStruct.Pin = RESET_PIN;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(RESET_PORT, &GPIO_InitStruct);


    // HP3457A I/O pins (serial interface decode)
    // These are 5Vdc tolerant pins on the Blue Pill so can interface directly with the 3457A 5V logic levels
    /* Configure GPIO pin : DMM_SYNC_Pin */
    GPIO_InitStruct.Pin = DMM_SYNC_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;   // rising edge = start of command
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(DMM_SYNC_GPIO_Port, &GPIO_InitStruct);

    /* Configure GPIO pin : DMM_O2_Pin */
    GPIO_InitStruct.Pin = DMM_O2_Pin;
    //GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(DMM_O2_GPIO_Port, &GPIO_InitStruct);

    /* Configure GPIO pin : DMM_INA_Pin */
    GPIO_InitStruct.Pin = DMM_INA_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(DMM_INA_GPIO_Port, &GPIO_InitStruct);

    /* Configure GPIO pin : DMM_PWO_Pin */
    GPIO_InitStruct.Pin = DMM_PWO_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(DMM_PWO_GPIO_Port, &GPIO_InitStruct);

    /* Configure GPIO pin : DMM_ISA_Pin */
    GPIO_InitStruct.Pin = DMM_ISA_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(DMM_ISA_GPIO_Port, &GPIO_InitStruct);
	
	
	// ST7701S LCD direct SPI
	// Configure bit bang SPI pins for direct LCD SPI
	GPIO_InitStruct.Pin = LCD_CS_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(LCD_CS_Port, &GPIO_InitStruct);
	
	GPIO_InitStruct.Pin = LCD_SCK_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(LCD_SCK_Port, &GPIO_InitStruct);
	
	GPIO_InitStruct.Pin = LCD_SDI_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(LCD_SDI_Port, &GPIO_InitStruct);


    // Selection header LK1
    // Configure GPIO pin B0
    GPIO_InitStruct.Pin = GPIO_PIN_0;       // Select pin B0
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT; // Set as input
    GPIO_InitStruct.Pull = GPIO_PULLUP;   // Enable pull-up resistor
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW; // Low speed is sufficient for input
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);


    // Selection header LK2
    // Configure GPIO pin B1
    //GPIO_InitStruct.Pin = GPIO_PIN_1;       // Select pin B1
    //GPIO_InitStruct.Mode = GPIO_MODE_INPUT; // Set as input
    //GPIO_InitStruct.Pull = GPIO_PULLUP;   // Enable pull-up resistor
    //GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW; // Low speed is sufficient for input
    //HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);


    /* EXTI interrupt init */
    HAL_NVIC_SetPriority(EXTI15_10_IRQn, 1, 0);  // High priority for SYNC
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

    HAL_NVIC_SetPriority(TIM3_IRQn, 2, 0);       // slightly lower than SYNC
    HAL_NVIC_EnableIRQ(TIM3_IRQn);

}

