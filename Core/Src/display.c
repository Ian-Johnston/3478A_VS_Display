/**
  ******************************************************************************
  * @file    display.c
  * @brief   This file provides code for the
  *          display MAIN, AUX, ANNUNCIATORS & SPLASH.
  ******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "spi.h"
#include "main.h"
#include "timer.h"
#include "lcd.h"
#include "lt7680.h"
#include "display.h"
#include <string.h>  // For strchr, strncpy
#include <stdio.h>   // For debugging (optional)

//#define DURATION_MS 5000     // 5 seconds in milliseconds
#define TIMER_INTERVAL_MS 35 // The interval of your timed sub in milliseconds

// Display colours default
uint32_t MainColourFore = 0xFFFFFF;			// White FFFFFF
uint32_t AnnunColourFore = 0x00FF00;		// Green 00FF00
uint32_t BackgroundColour = 0x000000;		// Black 000000
uint32_t SplashIanJColourFore = 0xFFFF00;	// Yellow FFFF00
uint32_t TriggerColourFore = 0x00FF00;	    // Green 00FF00
extern volatile uint8_t dpFarRight;



//************************************************************************************************************************************************************

void DisplayMain(void)
{
	SetTextColors(MainColourFore, BackgroundColour); // Foreground, Background
	ConfigureFontAndPosition(
		0b00,    // Internal CGROM
		0b10,    // Font size
		0b00,    // ISO 8859-1
		0,       // Full alignment enabled
		0,       // Chroma keying disabled
		1,       // Rotate 90 degrees counterclockwise
		0b11,    // Width multiplier
		0b11,    // Height multiplier
		1,       // Line spacing
		4,       // Character spacing
		Xpos_MAIN,     // Cursor X
		Ypos_MAIN      // Cursor Y
	);

	// Always draw exactly 14 characters
	char text1[15];   // 14 chars + terminator
	int i;

	// Copy up to 14 source characters (some frames really have 14)
	for (i = 0; i < 14; i++) {
		char c = displayWithPunct[i];
		text1[i] = (c == '\0') ? ' ' : c;
	}

	text1[14] = '\0';

	ShiftUnitsRight(text1);		// Shift last 4 chars to the right if match criteria

	FixUnitText(text1);			// Fix units

	FixRandomStrings(text1);

	DrawText(text1);
}


// Fix random strings and text anomolies
void FixRandomStrings(char* text1)
{
	int i;
	static char prevText1[15] = "              ";  // 14 spaces + '\0'
	int reject = 0;

	// Reject any string containing '?'
	if (strchr(text1, '?') != NULL) {
		strcpy(text1, prevText1);
		reject = 1;
	}

	if (!reject) {

		// Only perform S->5 replacement if this is NOT the SELF TEST message
		if (strstr(text1, "5ELF TE5T") == NULL) {
			for (i = 0; i < 9; i++) {
				if (text1[i] == 'S') {
					text1[i] = '5';
				}
			}
		}

		// Fix random text anomalies
		if (strstr(text1, ":,:,:,:,:,:,:") != NULL) {
			strcpy(text1, "##############");
		}
		else if (strstr(text1, "5ELF TE5T") != NULL) {
			strcpy(text1, "SELF TEST OK  ");
		}

		// Fix ADR5 -> ADRS
		{
			char* p = strstr(text1, "ADR5");
			if (p != NULL) {
				p[3] = 'S';
			}
		}

		// Fix "C:" -> "C" (terminate string at colon)
		//{
		//	char* p = strstr(text1, "C:");
		//	if (p != NULL) {
		//		p[1] = '\0';   // end string after 'C'
		//	}
		//}

		// Save as last good string
		strcpy(prevText1, text1);
	}

	return;
}


// Shift chars right - We have more space on the TFT so can afford to do this
// Enter the original four chars to be shifted
void ShiftUnitsRight(char* text1)
{
	static const char* unit4[] = {
		" VDC", "MVDC",
		"KOHM", " OHM", "MOHM", "GOHM",
		"MAAC", "UAAC", "UADC", "MADC",
		" ADC", " AAC", "MVAC", " VAC",
		"MSEC", " SEC",
		"  HZ", " MHZ",
		"  DB"
	};

	for (int u = 0; u < (int)(sizeof(unit4) / sizeof(unit4[0])); u++) {
		const char* p = unit4[u];

		if (text1[9] == p[0] &&
			text1[10] == p[1] &&
			text1[11] == p[2] &&
			text1[12] == p[3]) {

			// shift ONLY the 4-char unit suffix right by one
			text1[13] = text1[12];
			text1[12] = text1[11];
			text1[11] = text1[10];
			text1[10] = text1[9];
			text1[9] = ' ';
			return;
		}
	}
}



// Replace characters
// Enter the before & after of the 4 chat text to be replaced
void FixUnitText(char* text1)
{
	static const struct {
		const char from[5];   // 4 chars + '\0'
		const char to[5];     // 4 chars + '\0'
	} rules[] = {
		{ "MSEC", "  ms" },
		{ " SEC", "   s" },
		{ "  HZ", "  Hz" },
		{ " MHZ", " MHz" },
		{ "  DB", "  dB" },
		{ "MVAC", "mVAC" },
		{ "MVDC", "mVDC" },
		{ "KOHM", "kohm" },
		{ " OHM", " ohm" },
		{ "GOHM", "Gohm" },
		{ "MOHM", "Mohm" },
		{ "MAAC", "mAAC" },
		{ "MADC", "mADC" },
		{ "ADR5", "ADRS" },
		{ "MADC", "mADC" },
		{ "UADC", "\xB5""ADC" }   // µADC (0xB5 in ISO-8859-1)
	};

	for (int i = 0; text1[i + 3] != '\0'; i++) {
		for (int r = 0; r < (int)(sizeof(rules) / sizeof(rules[0])); r++) {
			if (text1[i] == rules[r].from[0] &&
				text1[i + 1] == rules[r].from[1] &&
				text1[i + 2] == rules[r].from[2] &&
				text1[i + 3] == rules[r].from[3]) {

				text1[i] = rules[r].to[0];
				text1[i + 1] = rules[r].to[1];
				text1[i + 2] = rules[r].to[2];
				text1[i + 3] = rules[r].to[3];
				return; // only one unit expected
			}
		}
	}
}


void DisplayTrigger(void)
{
	SetTextColors(TriggerColourFore, BackgroundColour); // Foreground, Background
	ConfigureFontAndPosition(
		0b00,    // Internal CGROM
		0b10,    // Font size
		0b00,    // ISO 8859-1
		0,       // Full alignment enabled
		0,       // Chroma keying disabled
		1,       // Rotate 90 degrees counterclockwise
		0b01,    // Width multiplier
		0b01,    // Height multiplier
		1,       // Line spacing
		4,       // Character spacing
		Xpos_TRIGGER,     // Cursor X
		Ypos_TRIGGER      // Cursor Y
	);

	if (dpFarRight == 1) {
		char sym[2];
		sym[0] = 0x04;   // H0,L4 - diamond
		sym[1] = '\0';
		DrawText(sym);
	}
	else {
		DrawText(" ");
	}
}


void DisplayAnnunciators() {

	// ANNUNCIATORS - Print or clear text on the LCD
	const char* AnnuncNames[12] = {
		"SRQ", "LSTN", "TLK", "RMT", "MATH", "AZOFF", "2ohm", "4ohm", "MRNG", "STRG", "CAL", "SHIFT"
	};


	// Set Y-position of the annunciators
	int AnnuncYCoords[12] = {
		10,   // SRQ
		77,   // LSTN
		157,  // TLK
		220,  // RMT
		285,  // MATH
		365,  // AZOFF
		454,  // 2ohm
		532,  // 4ohm
		616,  // MRNG
		693,  // STRG
		770,  // CAL
		830   // SHIFT
	};


	for (int i = 0; i < 12; i++) {
		if (Annunc[12 - i] == 1) {  // Turn the annunciator ON
			SetTextColors(AnnunColourFore, BackgroundColour); // Foreground: Green, Background: Black
			ConfigureFontAndPosition(
				0b00,    // Internal CGROM
				0b00,    // 16-dot font size
				0b00,    // ISO 8859-1
				0,       // Full alignment enabled
				0,       // Chroma keying disabled
				1,       // Rotate 90 degrees counterclockwise
				0b01,    // Width X0
				0b01,    // Height X0
				5,       // Line spacing
				0,       // Character spacing
				Xpos_ANNUNC,  // Cursor X (fixed)
				AnnuncYCoords[i] // Cursor Y (from array)
			);

			// TEMP TEST: force all annunciators ON (flash)
			//for (int k = 1; k <= 12; k++) {
			//	Annunc[k] = 1;
			//}

			DrawText(AnnuncNames[i]); // Print the corresponding name
		}
		else {  // Turn the annunciator OFF
			SetTextColors(BackgroundColour, BackgroundColour); // Foreground: Black, Background: Black
			ConfigureFontAndPosition(
				0b00,    // Internal CGROM
				0b00,    // 16-dot font size
				0b00,    // ISO 8859-1
				0,       // Full alignment enabled
				0,       // Chroma keying disabled
				1,       // Rotate 90 degrees counterclockwise
				0b01,    // Width X0
				0b01,    // Height X0
				5,       // Line spacing
				0,       // Character spacing
				Xpos_ANNUNC,  // Cursor X (fixed)
				AnnuncYCoords[i] // Cursor Y (from array)
			);
			DrawText(AnnuncNames[i]); // Clear the text by drawing in black
		}
	}

}


void DisplaySplash() {

	uint32_t originalColour = MainColourFore;

	// Colour sequence (no black or white)
	uint32_t splashCols[10] = {
		0xFF0000, // red
		0x00FF00, // green
		0x0000FF, // blue
		0xFFFF00, // yellow
		0xFF00FF, // magenta
		0x00FFFF, // cyan
		0xFF8000, // orange
		0x8000FF, // purple
		0x00FF80, // spring green
		0xFFFFFF  // white
	};

	for (int pass = 0; pass < 10; pass++) {

		MainColourFore = splashCols[pass];
		AnnunColourFore = splashCols[pass];

		// Main
		SetTextColors(MainColourFore, BackgroundColour); // Foreground, Background
		ConfigureFontAndPosition(
			0b00,    // Internal CGROM
			0b10,    // Font size
			0b00,    // ISO 8859-1
			0,       // Full alignment enabled
			0,       // Chroma keying disabled
			1,       // Rotate 90 degrees counterclockwise
			0b11,    // Width multiplier
			0b11,    // Height multiplier
			1,       // Line spacing
			4,       // Character spacing
			Xpos_MAIN,     // Cursor X
			Ypos_MAIN      // Cursor Y
		);

		// Always draw exactly 14 characters (13 source + 1 added)
		char text1[15];   // 14 chars + terminator
		int i;

		// Copy the 13 source characters (displayWithPunct is always 13 chars)
		for (i = 0; i < 13; i++) {
			char c = displayWithPunct[i];
			text1[i] = (c == '\0') ? ' ' : c;
		}

		// Default: add trailing space as the 14th character
		text1[13] = ' ';
		text1[14] = '\0';

		memcpy(text1, "##############", 14);
		text1[14] = '\0';

		DrawText(text1);

		HAL_Delay(10);

		// Annunciators
		const char* AnnuncNames[12] = {
		   "SRQ", "LSTN", "TLK", "RMT", "MATH", "AZOFF", "2ohm", "4ohm", "MRNG", "STRG", "CAL", "SHIFT"
		};

		// Set Y-position of the annunciators
		int AnnuncYCoords[12] = {
			10,   // SRQ
			77,   // LSTN
			157,  // TLK
			220,  // RMT
			285,  // MATH
			365,  // AZOFF
			454,  // 2ohm
			532,  // 4ohm
			616,  // MRNG
			693,  // STRG
			770,  // CAL
			830   // SHIFT
		};

		for (int i = 0; i < 12; i++) {

			SetTextColors(AnnunColourFore, BackgroundColour); // ON
			ConfigureFontAndPosition(
				0b00,    // Internal CGROM
				0b00,    // 16-dot font size
				0b00,    // ISO 8859-1
				0,       // Full alignment enabled
				0,       // Chroma keying disabled
				1,       // Rotate 90 degrees counterclockwise
				0b01,    // Width X0
				0b01,    // Height X0
				5,       // Line spacing
				0,       // Character spacing
				Xpos_ANNUNC,
				AnnuncYCoords[i]
			);

			DrawText(AnnuncNames[i]);
			HAL_Delay(5);
		}

		// Diamond
		SetTextColors(TriggerColourFore, BackgroundColour); // Foreground, Background
		ConfigureFontAndPosition(
			0b00,    // Internal CGROM
			0b10,    // Font size
			0b00,    // ISO 8859-1
			0,       // Full alignment enabled
			0,       // Chroma keying disabled
			1,       // Rotate 90 degrees counterclockwise
			0b01,    // Width multiplier
			0b01,    // Height multiplier
			1,       // Line spacing
			4,       // Character spacing
			Xpos_TRIGGER,     // Cursor X
			Ypos_TRIGGER      // Cursor Y
		);

		char sym[2];
		sym[0] = 0x04;   // H0,L4 - diamond
		sym[1] = '\0';
		DrawText(sym);

		HAL_Delay(1);  // Required after each full run
	}

	// Restore white at the end
	AnnunColourFore = 0x00FF00;
	MainColourFore = 0xFFFFFF;

	// --- Final redraw of annunciators in green so they actually end up green ---

	// Annunciators
	const char* AnnuncNames[12] = {
	   "SRQ", "LSTN", "TLK", "RMT", "MATH", "AZOFF", "2ohm", "4ohm", "MRNG", "STRG", "CAL", "SHIFT"
	};

	// Set Y-position of the annunciators
	int AnnuncYCoords[12] = {
		10,   // SRQ
		77,   // LSTN
		157,  // TLK
		220,  // RMT
		285,  // MATH
		365,  // AZOFF
		454,  // 2ohm
		532,  // 4ohm
		616,  // MRNG
		693,  // STRG
		770,  // CAL
		830   // SHIFT
	};

	for (int i = 0; i < 12; i++) {

		SetTextColors(AnnunColourFore, BackgroundColour); // ON
		ConfigureFontAndPosition(
			0b00,    // Internal CGROM
			0b00,    // 16-dot font size
			0b00,    // ISO 8859-1
			0,       // Full alignment enabled
			0,       // Chroma keying disabled
			1,       // Rotate 90 degrees counterclockwise
			0b01,    // Width X0
			0b01,    // Height X0
			5,       // Line spacing
			0,       // Character spacing
			Xpos_ANNUNC,
			AnnuncYCoords[i]
		);

		DrawText(AnnuncNames[i]);
		HAL_Delay(5);
	}
}


// Original splash not used.
void DisplaySplash2() {

	// Main
	SetTextColors(MainColourFore, BackgroundColour); // Foreground, Background
	ConfigureFontAndPosition(
		0b00,    // Internal CGROM
		0b10,    // Font size
		0b00,    // ISO 8859-1
		0,       // Full alignment enabled
		0,       // Chroma keying disabled
		1,       // Rotate 90 degrees counterclockwise
		0b11,    // Width multiplier
		0b11,    // Height multiplier
		1,       // Line spacing
		4,       // Character spacing
		Xpos_MAIN,     // Cursor X
		Ypos_MAIN      // Cursor Y
	);

	// Always draw exactly 14 characters (13 source + 1 added)
	char text1[15];   // 14 chars + terminator
	int i;

	// Copy the 13 source characters (displayWithPunct is always 13 chars)
	for (i = 0; i < 13; i++) {
		char c = displayWithPunct[i];
		text1[i] = (c == '\0') ? ' ' : c;
	}

	// Default: add trailing space as the 14th character
	text1[13] = ' ';
	text1[14] = '\0';

	memcpy(text1, "##############", 14);
	text1[14] = '\0';

	DrawText(text1);

	HAL_Delay(10);

	// Annunciators
	const char* AnnuncNames[12] = {
	   "SRQ", "LSTN", "TLK", "RMT", "MATH", "AZOFF", "2ohm", "4ohm", "MRNG", "STRG", "CAL", "SHIFT"
	};

	// Set Y-position of the annunciators
	int AnnuncYCoords[12] = {
		10,   // SRQ
		77,   // LSTN
		157,  // TLK
		220,  // RMT
		285,  // MATH
		365,  // AZOFF
		454,  // 2ohm
		532,  // 4ohm
		616,  // MRNG
		693,  // STRG
		770,  // CAL
		830   // SHIFT
	};

	for (int i = 0; i < 12; i++) {

		SetTextColors(AnnunColourFore, BackgroundColour); // ON
		ConfigureFontAndPosition(
			0b00,    // Internal CGROM
			0b00,    // 16-dot font size
			0b00,    // ISO 8859-1
			0,       // Full alignment enabled
			0,       // Chroma keying disabled
			1,       // Rotate 90 degrees counterclockwise
			0b01,    // Width X0
			0b01,    // Height X0
			5,       // Line spacing
			0,       // Character spacing
			Xpos_ANNUNC,
			AnnuncYCoords[i]
		);

		DrawText(AnnuncNames[i]);
		HAL_Delay(10);
	}

	// Diamond
	SetTextColors(TriggerColourFore, BackgroundColour); // Foreground, Background
	ConfigureFontAndPosition(
		0b00,    // Internal CGROM
		0b10,    // Font size
		0b00,    // ISO 8859-1
		0,       // Full alignment enabled
		0,       // Chroma keying disabled
		1,       // Rotate 90 degrees counterclockwise
		0b01,    // Width multiplier
		0b01,    // Height multiplier
		1,       // Line spacing
		4,       // Character spacing
		Xpos_TRIGGER,     // Cursor X
		Ypos_TRIGGER      // Cursor Y
	);

	char sym[2];
	sym[0] = 0x04;   // H0,L4 - diamond
	sym[1] = '\0';
	DrawText(sym);

}


