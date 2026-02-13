/*
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  *
  * By Ian Johnston (IanSJohnston on YouTube),
  * for 3.71" 960x240 TFT LCD (ST7701S) and using LT7680 controller adaptor
  * Visual Studio 2022 with VisualGDB plugin:
  * To upload HEX from VS2022 = BUILD then PROGRAM AND START WITHOUT DEBUGGING
  * Use LIVE WATCH to view variables live debug
  *
  * For use with LT7680A-R & 240x960 TFT LCD
  *
  ******************************************************************************

  * Inteface with 3478A:
  * ====================
  * U611 pin    BluePill          Function
  * 9           B15               INA
  * 15          B14               ISA
  * 12          B11               SYNC
  * 6           B1                02
  * 2           B12               PW0
  * 5           not connected
  * 20                            +5V
  * 10                            0V
  * 
  * 
  * 
 */


/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "spi.h"
#include "gpio.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "lt7680.h"
#include "timer.h"
#include <stdbool.h>    // bool support, otherwise use _Bool
#include <stdlib.h>
#include "display.h"
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_tim.h"
#include <stddef.h>

TIM_HandleTypeDef htim3; // Definition of htim3

//******************************************************************************
// Variables

volatile uint8_t TEST1 = 0;
volatile uint8_t TEST2 = 0;
volatile uint8_t TEST3 = 0;

#define MAX_BUFFER_SIZE 256  // Define the size of the buffer

volatile uint32_t debugO2Callback = 0;

#define DISPLAY_LENGTH 12  // Number of characters in the main display

char displayString[DISPLAY_LENGTH + 1];  // +1 for null-terminator

uint8_t isaState = 0;
uint8_t inaState = 0;

extern volatile uint8_t logReady;



//******************************************************************************
// HP Symbols

// Placeholder definitions for missing symbols
#define Da 0x01
#define Db 0x02
#define Dc 0x04
#define Dd 0x08
#define De 0x10
#define Df 0x20
#define Dg1 0x40
#define Dg2 0x80
#define Dm 0x100
#define Ds 0x200
#define Dk 0x400
#define Dt 0x800
#define Dn 0x1000
#define Dr 0x2000


//******************************************************************************

// Flag indicating finish of SPI transmission to OLED
volatile uint8_t SPI1_TX_completed_flag = 1;

// Flag indicating finish of start-up initialization
volatile uint8_t Init_Completed_flag = 0;

// Private function prototypes
void SystemClock_Config(void);



//******************************************************************************

// LT7680A-R - SPI transmission finished interrupt callback
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef* hspi) {
	if (hspi->Instance == SPI1)
	{
		SPI1_TX_completed_flag = 1;
	}
}


//************************************************************************************************************************************************************
//************************************************************************************************************************************************************

// Main
int main(void) {

	// Reset of all peripherals, Initializes the Flash interface and the Systick.
	HAL_Init();

	// Configure the system clock
	SystemClock_Config();

	// Initialize all configured peripherals
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_SPI1_Init();		// LT7680A-R

	MX_TIM3_Init();

	// Start TIM3 input-capture on CH4 (PB1 = TIM3_CH4)
	HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_4);

	// Pull CS high and SCLK low immediately after reset
	HAL_GPIO_WritePin(LCD_CS_Port, LCD_CS_Pin, GPIO_PIN_SET);			// Pull CS high
	HAL_GPIO_WritePin(LCD_SCK_Port, LCD_SCK_Pin, GPIO_PIN_RESET);		// CLK pin low

	// Pull LT7680 RESET pin high immediately after reset
	HAL_GPIO_WritePin(RESET_PORT, RESET_PIN, GPIO_PIN_SET);   // Release reset high

	HardwareReset();				// Reset LT7680 - Pull LCM_RESET low for 100ms and wait

	HAL_Delay(1000);

	BuyDisplay_Init();				// Initialize ST7701S BuyDisplay 3.71" driver IC

	HAL_Delay(100);

	SendAllToLT7680_LT();			// run subs to setup LT7680 based on Levetop info

	HAL_Delay(10);

	// Main loop timer
	//SetTimerDuration(35);			// 35 ms timed action set.......not used
	//HAL_Delay(5);
	//SetBacklightFull();
	ConfigurePWMAndSetBrightness(BACKLIGHTFULL);  // Configure Timer-1 and PWM-1 for backlighting. Settable 0-100%

	ClearScreen();					// Again.....

	//TFT_WipeTest();
	//ClearScreen();					// Again.....

	// Right wipe to clear random pixels down the far right hand side	- Only needed if back porch adjustments can't get rid of edge pixels
	DrawLine(0, 959, 239, 959, 0x00, 0x00, 0x00);	// far right hand vertical line, black, 1 pixel line. (this line hidden!)
	DrawLine(0, 958, 239, 958, 0x00, 0x00, 0x00);	// (this line hidden!)
	DrawLine(0, 957, 239, 957, 0x00, 0x00, 0x00);
	DrawLine(0, 956, 239, 956, 0x00, 0x00, 0x00);
	DrawLine(0, 955, 239, 955, 0x00, 0x00, 0x00);
	DrawLine(0, 954, 239, 954, 0x00, 0x00, 0x00);
	DrawLine(0, 953, 239, 953, 0x00, 0x00, 0x00);
	DrawLine(0, 952, 239, 952, 0x00, 0x00, 0x00);
	DrawLine(0, 951, 239, 951, 0x00, 0x00, 0x00);

	DisplaySplash();

	HAL_Delay(2000);

	ClearScreen();					// Again.....

	// Test only - 400pixel based test lines for viewing the centre line and the left, middle and far right positions.
	// The internal memory is set up as 400x960 but the leftmost 80 pixels are considered overscan and don't show up, thus 320
	// startX, startY, endX, endY, colorRED, colorGREEN, colorBLUE
	//DrawLine(0, 0, 239, 0, 0x00, 0xFF, 0xFF);		// far left hand vertical line, black, 1 pixel line. 938 not 960 seems to be far right edge!
	//DrawLine(0, 480, 239, 480, 0xFF, 0x00, 0xFF);	// mid-way
	//DrawLine(0, 959, 239, 959, 0xFF, 0xFF, 0x00);	// far right
	//DrawLine(119, 0, 119, 959, 0xFF, 0xFF, 0xFF);	// centred on R6243 horizontally

	__HAL_GPIO_EXTI_CLEAR_IT(DMM_SYNC_Pin);			// Clear any pending interrupt flag for SYNC (PB11)
	//HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);			// Ready to accept 3457A inputs


	//**************************************************************************************************
	// Main loop initialize

	while (1) {			// While loop running continious, full speed

		// TEST
		//isaState = HAL_GPIO_ReadPin(DMM_ISA_GPIO_Port, DMM_ISA_Pin);
		//inaState = HAL_GPIO_ReadPin(DMM_INA_GPIO_Port, DMM_INA_Pin);
		//volatile uint8_t debugIsaState = isaState; // Monitor these in Live Watch
		//volatile uint8_t debugInaState = inaState;

		HAL_GPIO_TogglePin(GPIOC, TEST_OUT_Pin); // Test LED toggle

		//DisplayMain();

		//HAL_Delay(10);

		//DisplayAnnunciators();

		//HAL_Delay(10);

		// 3478A capture
		Decode3478_LatchedFrame();

	}

}


// System Clock Configuration
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	//Initializes the RCC Oscillators according to the specified parameters in the RCC_OscInitTypeDef structure.
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	// Initializes the CPU, AHB and APB buses clocks
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
		| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
	{
		Error_Handler();
	}
}


// This function is executed in case of error occurrence.
void Error_Handler(void) {
	// User can add his own implementation to report the HAL error return state
	__disable_irq();
	while (1)
	{
	}
	/* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
	   ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	   /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
