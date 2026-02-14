/**
  ******************************************************************************
  * @file    timer.c
  * @brief   This file provides code for the configuration
  *          of the timer, and used in main.c
  ******************************************************************************
*/

#include "timer.h"
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_tim.h"
#include "main.h"  // Include main.h to access DMM pin definitions

//***********************************************************************************
// Timer 2 - Timed Action loop
// Timer interrupt sets a flag at regular intervals.
// Use your main loop to monitor this flag and ensures other conditions (like task completion)
// are met before running the dependent subroutine.  

// Timer flag variables
//volatile uint8_t timer_flag = 0;
//volatile uint8_t task_ready = 0;

uint8_t data = 0;  // Global variable for Live Watch visibility

/*
// Timer 2 - initialization function
void TIM2_Init(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN; // Enable TIM2 clock

    TIM2->PSC = 7200 - 1;               // Prescaler: 7200 (72 MHz / 7200 = 10 kHz)
    TIM2->ARR = 5000 - 1;               // Default Auto-reload: 500 ms
    TIM2->DIER |= TIM_DIER_UIE;         // Enable update interrupt
    TIM2->CR1 |= TIM_CR1_CEN;           // Enable the timer

    NVIC_EnableIRQ(TIM2_IRQn);          // Enable TIM2 interrupt in NVIC

    HAL_NVIC_SetPriority(TIM2_IRQn, 3, 0);  // Set lower priority for TIM2
    HAL_NVIC_EnableIRQ(TIM2_IRQn);


}
*/

/*
// Timer 2 - interrupt handler
void TIM2_IRQHandler(void) {

    if (TIM2->SR & TIM_SR_UIF) { // Check for update interrupt
        TIM2->SR &= ~TIM_SR_UIF; // Clear the interrupt flag
        timer_flag = 1;          // Set the timer flag
    }
}
*/

// Timer 2 - Dynamic timer duration setting function
void SetTimerDuration(uint16_t ms) {
    // Calculate ARR based on the desired duration in milliseconds
    TIM2->ARR = (10 * ms) - 1; // For a 10 kHz timer clock (1 tick = 0.1 ms)
    TIM2->EGR = TIM_EGR_UG;    // Force update to apply changes immediately
}


//***********************************************************************************
// Timer 3 - 3478A Input Capture Functionality

/* Buffers for Data Capture */
#define MAX_BUFFER_SIZE 256

volatile uint8_t isaBuffer[MAX_BUFFER_SIZE];
volatile uint8_t inaBuffer[MAX_BUFFER_SIZE];
static uint16_t isaIndex = 0;
static uint16_t inaIndex = 0;
#define LOG_TRIGGER_COUNT 256 // Log after 256 samples

/* Private Function Prototypes */
volatile uint8_t bufferFull = 0; // Indicates if the buffer is full
volatile uint8_t syncState = 0;
volatile uint32_t O2Count;
volatile uint32_t ISAcount = 0;
volatile uint32_t INAcount = 0;
volatile uint8_t  lastBit = 0;
volatile uint8_t  lastSync = 0;
volatile uint32_t lastBitOnes = 0;
volatile uint32_t syncHighCount = 0;
volatile uint32_t syncLowCount = 0;

// *********************************************

// ---- Decode state (Live Watch friendly) ----
volatile uint16_t lastCmd = 0;
volatile uint8_t  lastDataByte = 0;
volatile uint8_t  payloadBytesExpected = 0;
volatile uint8_t  payloadBytesGot = 0;
volatile uint8_t  frameReady = 0;

volatile uint8_t  regA[6], regB[6], regC[6];
volatile uint8_t  ann[2];

// Internal decode state
static uint8_t  prevSync = 0;

static uint8_t currentTarget = 0;  // 1=A,2=B,3=C,4=ANN,5=X(2E0)

// Decoded output for Live Watch / display
volatile uint8_t digitCode[12];   // 7-bit HP char code per digit (1..12 -> index 0..11)
volatile uint8_t punctCode[12];   // 0..3 (none, '.', ':', ',')

static void DecodeAllDigits(void);

volatile uint8_t lastExpected = 0;
volatile uint32_t cmd028Count = 0;
volatile uint32_t cmd068Count = 0;
volatile uint32_t cmd0A8Count = 0;
volatile uint32_t cmd2F0Count = 0;
volatile uint16_t lastCmd2 = 0;
volatile uint16_t cmdSeen[8] = { 0 };
volatile uint32_t cmdSeenCount[8] = { 0 };
static void TrackCmd(uint16_t cmd);
volatile uint8_t digitVal[12];   // 0–9, or 0xF for blank
static void DecodeDigitsFromRegA(void);
volatile int8_t digitDec[12];
volatile uint8_t  regX[6];          // payload after 0x2E0 (candidate “Reg B”)
volatile uint32_t cmd2E0Count = 0;  // Live Watch
volatile uint32_t cmdIgnoredCount = 0;
volatile uint8_t numStart = 0;
volatile uint8_t numLen = 0;
volatile uint8_t numDigits[12];
static void ExtractBestRun(void);
volatile uint8_t regANib[12];      // 12 nibbles extracted from regA
volatile uint8_t regANibPrev[12];
volatile uint8_t regANibChanged[12]; // 1 if that nibble changed this frame
static void ExtractRegANibbles(void);
volatile uint8_t lsdNibIdx = 4;       // you’ve found this
volatile uint16_t changedMask;
volatile uint8_t digitNibIdx[6] = { 4, 5, 6, 7, 8, 9 };  // LSD..MSD
static uint8_t tenCount = 0;       // 0..9 within current 10-bit chunk
static uint8_t byteShift = 0;      // assembled 8-bit value, LSB-first
static uint8_t byteBitCount = 0;   // 0..7
volatile uint32_t isaBranchHits = 0;
volatile uint8_t lastCmdRawLSB = 0;  // debug only (same as lastCmdRaw now)
volatile uint8_t lastCmdRawMSB = 0;  // retained for Live Watch compatibility
volatile uint8_t lastCmdRaw = 0;
static uint16_t isa10 = 0;
static uint8_t  isaBitCount = 0;
static uint8_t  inaGap2 = 0;      // counts down 2->0 after SYNC falls
volatile uint16_t lastCmd10 = 0;
volatile uint32_t cmdOtherCount = 0;
volatile uint32_t cmd068NearCount = 0;   // counts commands in 0x060..0x06F
volatile uint32_t cmd0A8NearCount = 0;   // counts commands in 0x0A0..0x0AF
volatile uint16_t cmdRing[16];
volatile uint8_t  cmdRingIdx = 0;
volatile uint32_t regCWrites = 0;
volatile uint8_t  lastRegC[6];
volatile uint16_t lastCmdAtRegC = 0;
volatile uint32_t regCNonZeroWrites = 0;
volatile uint8_t dbgCode[12];
static uint8_t HP3478_GetCharCode(uint8_t d);
static uint8_t HP3478_GetPunct(uint8_t d);
volatile uint8_t dbgCode_alt[12];
static uint8_t HP3478_GetCharCode_Alt(uint8_t d);
volatile char displayStr[13];       // 12 chars + \0
volatile char punctStr[13];         // punctuation per digit + \0
volatile char displayWithPunct[32]; // debug/combined string + \0
volatile uint8_t Annunc[13];   // use indices 1..12, ignore 0
volatile uint8_t dbg_char8 = 0;
volatile uint8_t dbg_char8_mapped = 0;
volatile uint8_t dbg_after_build = 0;
volatile uint8_t dpFarRight = 0;     // 1 when the far-right DP is ON this frame


//*************************
//volatile uint8_t dbgCode[12];



//***********************************************************************************

void DMM_HandleO2Clock(void)
{
    GPIO_PinState syncLevel = HAL_GPIO_ReadPin(DMM_SYNC_GPIO_Port, DMM_SYNC_Pin);

    lastSync = (syncLevel == GPIO_PIN_SET) ? 1 : 0;
    syncState = lastSync;

    if (syncLevel == GPIO_PIN_SET) syncHighCount++;
    else                           syncLowCount++;

    O2Count++;

    if (HAL_GPIO_ReadPin(DMM_PWO_GPIO_Port, DMM_PWO_Pin) != GPIO_PIN_SET) {
        return;
    }

    /* -------- SYNC edge detection & re-alignment -------- */
    {
        uint8_t syncNow = (syncLevel == GPIO_PIN_SET) ? 1u : 0u;

        if (syncNow != prevSync) {

            /* reset common byte assembly */
            tenCount = 0;
            byteShift = 0;
            byteBitCount = 0;

            if (syncNow) {
                /* entering ISA phase */
                isa10 = 0;
                isaBitCount = 0;
            }
            else {
                /* entering INA phase → 2 dummy clocks */
                inaGap2 = 2;
            }
        }
        prevSync = syncNow;
    }

    /* ================= ISA ================= */
    if (syncLevel == GPIO_PIN_SET) {

        isaBranchHits++;

        uint8_t bit = (uint8_t)HAL_GPIO_ReadPin(DMM_ISA_GPIO_Port, DMM_ISA_Pin) & 1u;
        lastBit = bit;
        if (bit) lastBitOnes++;

        isaBuffer[isaIndex++] = bit;
        isaIndex %= MAX_BUFFER_SIZE;
        ISAcount++;

        /* ---- build 10-bit ISA command (LSB first) ---- */
        isa10 |= (uint16_t)(bit << isaBitCount);
        isaBitCount++;

        if (isaBitCount == 10) {

            lastCmd = isa10;        // FULL 10-bit command

            lastCmd10 = lastCmd;

            // keep a short ring buffer of commands (Live Watch)
            cmdRing[cmdRingIdx++ & 0x0F] = lastCmd;

            // simple “near” detectors to prove we’re in the right neighborhood
            if ((lastCmd & 0xFF0) == 0x060) cmd068NearCount++;
            if ((lastCmd & 0xFF0) == 0x0A0) cmd0A8NearCount++;

            // if not one of the expected ones, count it
            if (lastCmd != 0x028 && lastCmd != 0x068 && lastCmd != 0x0A8 &&
                lastCmd != 0x2F0 && lastCmd != 0x2E0 && lastCmd != 0x3F0 &&
                lastCmd != 0x320) {
                cmdOtherCount++;
            }

            lastCmd2 = isa10;

            isa10 = 0;
            isaBitCount = 0;

            /* drop stale payload if any */
            if (payloadBytesExpected && currentTarget) {
                cmdIgnoredCount++;
                payloadBytesExpected = 0;
                currentTarget = 0;
                payloadBytesGot = 0;
            }

            /* ---- command counters ---- */
            if (lastCmd == 0x028) cmd028Count++;
            else if ((lastCmd & 0xFF8) == 0x068) cmd068Count++;  // accept 0x068..0x06F
            else if ((lastCmd & 0xFF8) == 0x0A8) cmd0A8Count++;  // accept 0x0A8..0x0AF
            else if (lastCmd == 0x2F0) cmd2F0Count++;
            else if (lastCmd == 0x2E0) cmd2E0Count++;


            payloadBytesGot = 0;
            frameReady = 0;

            /* ---- arm payload capture ---- */
            if (lastCmd == 0x028) { payloadBytesExpected = 6; currentTarget = 1; }
            else if (lastCmd == 0x068) { payloadBytesExpected = 6; currentTarget = 2; }
            else if (lastCmd == 0x0A8) { payloadBytesExpected = 6; currentTarget = 3; }
            else if (lastCmd == 0x2F0) { payloadBytesExpected = 2; currentTarget = 4; }
            else { payloadBytesExpected = 0; currentTarget = 0; }

            lastExpected = payloadBytesExpected;
        }
    }

    /* ================= INA ================= */
    else {

        // 2 clock gap between ISA and first INA payload bit
        if (inaGap2) {
            inaGap2--;
            return;
        }

        uint8_t bit = (uint8_t)HAL_GPIO_ReadPin(DMM_INA_GPIO_Port, DMM_INA_Pin) & 1u;
        lastBit = bit;
        if (bit) lastBitOnes++;

        inaBuffer[inaIndex++] = bit;
        inaIndex %= MAX_BUFFER_SIZE;
        INAcount++;

        // INA payload is pure 8-bit bytes (LSB-first) once the 2-gap clocks are skipped
        byteShift |= (uint8_t)(bit << byteBitCount);
        byteBitCount++;

        if (byteBitCount == 8) {

            lastDataByte = byteShift;

            // reset for next byte
            byteShift = 0;
            byteBitCount = 0;

            if (payloadBytesExpected && currentTarget) {

                if (currentTarget == 1 && payloadBytesGot < 6) regA[payloadBytesGot] = lastDataByte;
                else if (currentTarget == 2 && payloadBytesGot < 6) regB[payloadBytesGot] = lastDataByte;
                else if (currentTarget == 3 && payloadBytesGot < 6) regC[payloadBytesGot] = lastDataByte;
                else if (currentTarget == 4 && payloadBytesGot < 2) ann[payloadBytesGot] = lastDataByte;

                payloadBytesGot++;

                if (payloadBytesGot >= payloadBytesExpected) {

                    uint8_t finishedTarget = currentTarget;

                    payloadBytesExpected = 0;
                    currentTarget = 0;

                    if (finishedTarget == 3) {
                        // RegC just completed
                        regCWrites++;
                        lastCmdAtRegC = lastCmd;   // should be 0x0A8

                        uint8_t any = 0;
                        for (int i = 0; i < 6; i++) {
                            lastRegC[i] = regC[i];
                            any |= regC[i];
                        }
                        if (any) regCNonZeroWrites++;
                    }

                    if (finishedTarget == 1 ||
                        finishedTarget == 2 ||
                        finishedTarget == 3) {
                        //DecodeAllDigits();
                        HP3478_BuildDisplayString(displayStr, punctStr);

                        // Capture far-right decimal point (digit 12, index 11).
                        // This is the one that flashes on trigger.
                        dpFarRight = (punctStr[11] == '.') ? 1u : 0u;


                        dbg_after_build = 1;                          // marker: we reached here
                        dbg_char8 = (uint8_t)displayStr[8];           // what actually ended up in the string
                        dbg_char8_mapped = (uint8_t)displayStr[8];    // should be '=' (0x3D) if your mapping is in use



                        int k = 0;
                        for (int i = 0; i < 12; i++) {
                            displayWithPunct[k++] = displayStr[i];
                            if (punctStr[i] != ' ') displayWithPunct[k++] = punctStr[i]; // '.',':',','
                        }
                        displayWithPunct[k] = '\0';


                        for (int i = 0; i < 12; i++) {
                            uint8_t d = (uint8_t)(i + 1);   // because we now build display left-to-right with d=1..12
                            dbgCode[i] = HP3478_GetCharCode(d);
                        }


                        for (int i = 0; i < 12; i++) {
                            uint8_t d = (uint8_t)(i + 1);
                            dbgCode_alt[i] = HP3478_GetCharCode_Alt(d);
                        }

                        frameReady = 1;

                    }

                    if (finishedTarget == 4) {
                        // ann[0] and ann[1] contain the two bytes received after 0x2F0
                        // Bits are transmitted LSB-first overall, and the forum mapping says
                        // bit position 12..1 = SMPL..SHIFT (reverse order on the wire).
                        uint16_t bits = (uint16_t)ann[0] | ((uint16_t)ann[1] << 8);

                        // Map to Annunc[1..12] where:
                        // 1=SHIFT ... 12=SMPL (matches the forum table)
                        for (int pos = 1; pos <= 12; pos++) {
                            Annunc[pos] = (bits >> (pos - 1)) & 1u;
                        }
                    }


                }
            }
        }
    }

}


void MX_TIM3_Init(void) {
    TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
    TIM_MasterConfigTypeDef sMasterConfig = { 0 };
    TIM_IC_InitTypeDef sConfigIC = { 0 };

    htim3.Instance = TIM3;
    htim3.Init.Prescaler = 72 - 1;              // 1 MHz counter clock (timing not critical for capture)
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.Period = 0xFFFF;                 // Free-running counter
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    if (HAL_TIM_IC_Init(&htim3) != HAL_OK) {
        Error_Handler();
    }

    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK) {
        Error_Handler();
    }

    // Input capture on TIM3_CH4 (PB1) rising edge
    sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
    sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
    sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
    sConfigIC.ICFilter = 0;

    if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_4) != HAL_OK) {
        Error_Handler();
    }

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK) {
        Error_Handler();
    }
}


void TIM3_IRQHandler(void) {
    HAL_TIM_IRQHandler(&htim3);
}


void DMM_HandleSyncState(void) {
    // Update syncState based on SYNC pin level
    syncState = HAL_GPIO_ReadPin(DMM_SYNC_GPIO_Port, DMM_SYNC_Pin);

    // Optional: Toggle LED to indicate SYNC change
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == DMM_SYNC_Pin) {
        // Update syncState or perform your custom logic
        syncState = HAL_GPIO_ReadPin(DMM_SYNC_GPIO_Port, DMM_SYNC_Pin);

        // Optional: Toggle LED for debugging
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    }
}


void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef* htim) {
    if (htim->Instance == TIM3) {
        if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4) {
            // This fires on every real O2 rising edge (PB1 / TIM3_CH4)
            DMM_HandleO2Clock();
        }
    }
}


static uint8_t DecodeHpDigitCode(uint8_t digit1to12, uint8_t* punctOut)
{
    // digit1to12: 1..12
    // regA/regB/regC are 6 bytes each, where regA[0] is byte#1 in xi's table

    uint8_t d = digit1to12;
    uint8_t pair = (uint8_t)((d - 1) >> 1);     // 0 for digits 1-2, 1 for 3-4, ... 5 for 11-12
    uint8_t idx = (uint8_t)(5 - pair);         // maps to byte index: digits 1-2 -> idx5, 11-12 -> idx0

    uint8_t a = regA[idx];
    uint8_t b = regB[idx];
    uint8_t c = regC[idx];

    uint8_t isEven = (uint8_t)((d & 1u) == 0u);

    uint8_t nibble;     // 4 LSBs from regA
    uint8_t b2;         // 2 bits from regB
    uint8_t ext;        // 1 bit from regC
    uint8_t p;          // punctuation 2-bit

    if (isEven) {
        // even digits: RegA bits0-3, RegB bits0-1, RegC bit0
        nibble = (uint8_t)(a & 0x0F);
        b2 = (uint8_t)(b & 0x03);
        ext = (uint8_t)(c & 0x01);

        // punctuation for even digits: RegB bits2-3
        p = (uint8_t)((b >> 2) & 0x03);
    }
    else {
        // odd digits: RegA bits4-7, RegB bits4-5, RegC bit4
        nibble = (uint8_t)((a >> 4) & 0x0F);
        b2 = (uint8_t)((b >> 4) & 0x03);
        ext = (uint8_t)((c >> 4) & 0x01);

        // punctuation for odd digits: RegB bits6-7
        p = (uint8_t)((b >> 6) & 0x03);
    }

    // Build 7-bit HP character code:
    // bits 0..3 = nibble, bits 4..5 = b2, bit6 = ext
    uint8_t code = (uint8_t)(nibble | (uint8_t)(b2 << 4) | (uint8_t)(ext << 6));

    if (punctOut) *punctOut = p;
    return code;
}


static void DecodeAllDigits(void)
{
    // digit 1..12 -> array 0..11
    for (uint8_t d = 1; d <= 12; d++) {
        uint8_t p;
        uint8_t code = DecodeHpDigitCode(d, &p);
        digitCode[d - 1] = code;
        punctCode[d - 1] = p;
    }
}


static void TrackCmd(uint16_t cmd)
{
    // simple “top-N unique” tracker
    for (int i = 0; i < 8; i++) {
        if (cmdSeenCount[i] && cmdSeen[i] == cmd) {
            cmdSeenCount[i]++;
            return;
        }
    }
    for (int i = 0; i < 8; i++) {
        if (cmdSeenCount[i] == 0) {
            cmdSeen[i] = cmd;
            cmdSeenCount[i] = 1;
            return;
        }
    }
    // if full, ignore (good enough)
}


static int8_t HpNibbleToDigit(uint8_t v)
{
    switch (v) {
    case 3:  return 1;
    case 4:  return 2;
    case 1:  return 3;
    case 0:  return 4;
    case 7:  return 5;
    case 5:  return 7;
    default: return -1; // unknown/blank for now
    }
}


static void DecodeDigitsFromRegA(void)
{
    uint8_t tmp[12];
    uint8_t di = 0;

    for (int i = 5; i >= 0; i--) {
        uint8_t v = regA[i];
        tmp[di++] = (v >> 4) & 0x0F;   // high nibble first
        tmp[di++] = v & 0x0F;   // low nibble
    }

    for (int k = 0; k < 12; k++) {
        digitVal[k] = tmp[11 - k];                 // raw HP code
        digitDec[k] = HpNibbleToDigit(digitVal[k]); // usable digit
    }

    ExtractBestRun();

}


static void ExtractBestRun(void)
{
    uint8_t bestStart = 0, bestLen = 0;
    uint8_t curStart = 0, curLen = 0;

    for (uint8_t i = 0; i < 12; i++) {
        if (digitDec[i] >= 0 && digitDec[i] <= 9) {
            if (curLen == 0) curStart = i;
            curLen++;
            if (curLen > bestLen) { bestLen = curLen; bestStart = curStart; }
        }
        else {
            curLen = 0;
        }
    }

    numStart = bestStart;
    numLen = bestLen;

    for (uint8_t j = 0; j < 12; j++) numDigits[j] = 0xFF;
    for (uint8_t j = 0; j < bestLen && j < 12; j++) numDigits[j] = (uint8_t)digitDec[bestStart + j];
}


static void ExtractRegANibbles(void)
{
    // Extract 12 nibbles from regA[0..5]
    // low nibble first, then high nibble
    int j = 0;
    for (int i = 0; i < 6; i++) {
        uint8_t v = regA[i];
        regANib[j++] = v & 0x0F;
        regANib[j++] = (v >> 4) & 0x0F;
    }

    // Track which nibbles changed this frame
    changedMask = 0;

    for (int k = 0; k < 12; k++) {
        if (regANib[k] != regANibPrev[k]) {
            regANibChanged[k] = 1;
            changedMask |= (1u << k);
        }
        else {
            regANibChanged[k] = 0;
        }
        regANibPrev[k] = regANib[k];
    }
}


// Returns 7-bit HP char code (0..127) for digit number d = 1..12
static uint8_t HP3478_GetCharCode(uint8_t d)
{
    // d: 1..12
    uint8_t bi = (12 - d) / 2;          // 0..5
    uint8_t even = ((d & 1u) == 0u);    // digit number even?

    uint8_t a = regA[bi];
    uint8_t b = regB[bi];
    uint8_t c = regC[bi];

    uint8_t code = 0;

    if (even) {
        // even digits: use low nibble/bits
        // bit6 from RegC bit0
        // bit5 from RegB bit1
        // bit4 from RegB bit0
        // bit3..0 from RegA bits3..0
        code |= ((c >> 0) & 1u) << 6;
        code |= ((b >> 1) & 1u) << 5;
        code |= ((b >> 0) & 1u) << 4;
        code |= ((a >> 3) & 1u) << 3;
        code |= ((a >> 2) & 1u) << 2;
        code |= ((a >> 1) & 1u) << 1;
        code |= ((a >> 0) & 1u) << 0;
    }
    else {
        // odd digits: use high nibble/bits
        // bit6 from RegC bit4
        // bit5 from RegB bit5
        // bit4 from RegB bit4
        // bit3..0 from RegA bits7..4
        code |= ((c >> 4) & 1u) << 6;
        code |= ((b >> 5) & 1u) << 5;
        code |= ((b >> 4) & 1u) << 4;
        code |= ((a >> 7) & 1u) << 3;
        code |= ((a >> 6) & 1u) << 2;
        code |= ((a >> 5) & 1u) << 1;
        code |= ((a >> 4) & 1u) << 0;
    }

    return code;
}


// Returns punctuation code 0..3 for digit number d = 1..12
// 0 none, 1 '.', 2 ':', 3 ','
static uint8_t HP3478_GetPunct(uint8_t d)
{
    uint8_t bi = (12 - d) / 2;          // 0..5
    uint8_t even = ((d & 1u) == 0u);

    uint8_t b = regB[bi];

    if (even) {
        // even digits: RegB bit3..2
        return (uint8_t)((b >> 2) & 0x03u);
    }
    else {
        // odd digits: RegB bit7..6
        return (uint8_t)((b >> 6) & 0x03u);
    }
}


static char HP3478_CodeToAscii(uint8_t code)
{

    // Replace chars
    if (code == 0x3F) return '=';
    if (code == 0x1F) return '_';

    // Unit letters sometimes arrive without bit6; map 0x00..0x1F to 'A'..'Z' if it fits
    if (code < 0x20) {
        uint8_t c2 = (uint8_t)(code | 0x40);
        if (c2 >= 'A' && c2 <= 'Z') return (char)c2;
    }

    // If it’s normal printable ASCII, just return it (this includes '=' at 0x3D)
    if (code >= 0x20 && code <= 0x7E) return (char)code;

    return '?';
}


void HP3478_BuildDisplayString(char out12[13], char punct12[13])
{
    for (int i = 0; i < 12; i++) {
        uint8_t d = (uint8_t)(i + 1);   // d = 1..12  (was 12-i)

        uint8_t code = HP3478_GetCharCode(d);

        if (code == 0x3F) code = 0x3D;   // force '='

        out12[i] = HP3478_CodeToAscii(code);

        uint8_t p = HP3478_GetPunct(d);
        punct12[i] = (p == 1) ? '.' : (p == 2) ? ':' : (p == 3) ? ',' : ' ';
    }

    // Use this to live watch in order to get code for ? chars - live watch ---> dbgCode[12]
    //for (int i = 0; i < 12; i++) {
    //    uint8_t d = (uint8_t)(i + 1);
    //    dbgCode[i] = HP3478_GetCharCode(d);
    //}

    out12[12] = '\0';
    punct12[12] = '\0';
}


static uint8_t HP3478_GetCharCode_Alt(uint8_t d)
{
    uint8_t bi = (12 - d) / 2;
    uint8_t even = ((d & 1u) == 0u);

    uint8_t a = regA[bi];
    uint8_t b = regB[bi];
    uint8_t c = regC[bi];

    uint8_t code = 0;

    if (even) {
        code |= ((c >> 0) & 1u) << 6;
        // SWAPPED vs normal:
        code |= ((b >> 0) & 1u) << 5;
        code |= ((b >> 1) & 1u) << 4;

        code |= ((a >> 3) & 1u) << 3;
        code |= ((a >> 2) & 1u) << 2;
        code |= ((a >> 1) & 1u) << 1;
        code |= ((a >> 0) & 1u) << 0;
    }
    else {
        code |= ((c >> 4) & 1u) << 6;
        // SWAPPED vs normal:
        code |= ((b >> 4) & 1u) << 5;
        code |= ((b >> 5) & 1u) << 4;

        code |= ((a >> 7) & 1u) << 3;
        code |= ((a >> 6) & 1u) << 2;
        code |= ((a >> 5) & 1u) << 1;
        code |= ((a >> 4) & 1u) << 0;
    }

    return code;
}