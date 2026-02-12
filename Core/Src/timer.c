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
// Timer 3 - 3457A Input Capture Functionality

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
//static uint8_t  prevSync = 0;

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
//static uint8_t  inaGap2 = 0;      // counts down 2->0 after SYNC falls
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
static uint8_t HP3457_GetCharCode(uint8_t d);
static uint8_t HP3457_GetPunct(uint8_t d);
volatile uint8_t dbgCode_alt[12];
static uint8_t HP3457_GetCharCode_Alt(uint8_t d);
volatile char displayStr[13];       // 12 chars + \0
volatile char punctStr[13];         // punctuation per digit + \0
volatile char displayWithPunct[32]; // debug/combined string + \0
volatile uint8_t Annunc[13];   // use indices 1..12, ignore 0
volatile uint8_t dbg_char8 = 0;
volatile uint8_t dbg_char8_mapped = 0;
volatile uint8_t dbg_after_build = 0;

// 3478A testing
volatile uint32_t isaCmdCompleteCount = 0;
volatile uint32_t isaCmdLowNibbleZeroCount = 0;
volatile uint32_t isaCmdLowNibbleNonZeroCount = 0;
//volatile uint16_t lastCmd10 = 0;
//volatile uint8_t isaGap2 = 0;
//volatile uint8_t inaGap2 = 0;
static uint8_t skipThisClock = 0;
volatile uint32_t cmd_000_1FF = 0;
volatile uint32_t cmd_200_2FF = 0;
volatile uint32_t cmd_300_3FF = 0;
volatile uint16_t lastCmd10_hex = 0;
volatile uint32_t cmd260Count = 0;
//volatile uint8_t  inaBytesSinceLastCmd = 0;
volatile uint16_t lastCmdPrev = 0;
volatile uint8_t  lastInaLenPrev = 0;

volatile uint16_t cmdSeq[16];
volatile uint8_t  lenSeq[16];
volatile uint8_t  seqIdx = 0;

volatile uint8_t  inaBytesSinceLastCmd = 0;

//volatile uint8_t  lastPayload[64];
//volatile uint8_t  lastPayloadLen = 0;

//static uint8_t payloadBuf[64];
//static uint8_t payloadBufLen = 0;
static uint16_t currentCmd10 = 0;

volatile uint8_t  lastLen260 = 0, lastLen380 = 0;
volatile uint32_t cnt260 = 0, cnt380 = 0;

volatile uint8_t  pay260[64], len260 = 0;
volatile uint8_t  pay380[64], len380 = 0;

volatile uint8_t pay260_prev[64];
volatile uint8_t len260_prev = 0;

volatile uint8_t pay260_diffMask[64];   // 1 if changed
volatile uint8_t diffCount = 0;

volatile uint8_t pay260_prev20[64], pay260_prev31[64];
volatile uint8_t havePrev20 = 0, havePrev31 = 0;

volatile uint8_t diffCount20 = 0, diffCount31 = 0;
volatile uint8_t diffMask20[64], diffMask31[64];

volatile uint32_t chg31[64];
volatile uint32_t frames31;

volatile uint8_t stableDump[16];

volatile uint8_t dbg_len260;
volatile uint8_t dbg_p260_0, dbg_p260_9, dbg_p260_10, dbg_p260_30;

volatile uint32_t dbg_inISA = 0;
volatile uint32_t dbg_inINA = 0;

volatile uint8_t  dbg_byteBitCount = 0;
volatile uint8_t  dbg_lastDataByte2 = 0;
volatile uint32_t dbg_bytesMade = 0;
volatile uint16_t dbg_payloadBufLen = 0;

volatile uint16_t dbg_lastCmdPrev = 0;

volatile uint32_t hit260 = 0;
volatile uint32_t hit380 = 0;
volatile uint16_t dbg_cmdPrev_atTest = 0;
volatile uint16_t dbg_lenPrev_atTest = 0;

volatile uint32_t chg380[64];
//volatile uint8_t  pay380_prev[64];
volatile uint8_t  havePrev380 = 0;
volatile uint8_t  diffCount380 = 0;

volatile uint32_t chg260_31[64];
volatile uint32_t chg260_20[64];

//volatile uint8_t  pay260_prev31[64];
//volatile uint8_t  pay260_prev20[64];

//volatile uint8_t  havePrev31 = 0;
//volatile uint8_t  havePrev20 = 0;

volatile uint8_t  diffCount260_31 = 0;
volatile uint8_t  diffCount260_20 = 0;

volatile uint8_t dbg260_b0 = 0;
volatile uint8_t dbg260_b1 = 0;
volatile uint8_t dbg260_b2 = 0;
volatile uint8_t dbg260_b3 = 0;
volatile uint8_t dbg260_b4 = 0;
volatile uint8_t dbg260_b5 = 0;
volatile uint8_t dbg260_b6 = 0;
volatile uint8_t dbg260_b7 = 0;
volatile uint8_t dbg260_b8 = 0;
volatile uint8_t dbg260_b9 = 0;
volatile uint8_t dbg260_b10 = 0;
volatile uint8_t dbg260_b11 = 0;
volatile uint8_t dbg260_b12 = 0;
volatile uint8_t dbg260_b13 = 0;
volatile uint8_t dbg260_b14 = 0;
volatile uint8_t dbg260_b15 = 0;
volatile uint8_t dbg260_b16 = 0;
volatile uint8_t dbg260_b17 = 0;
volatile uint8_t dbg260_b18 = 0;
volatile uint8_t dbg260_b19 = 0;
volatile uint8_t dbg260_b20 = 0;
volatile uint8_t dbg260_b21 = 0;
volatile uint8_t dbg260_b22 = 0;

volatile uint8_t xor260_31[31];
volatile uint8_t dbg_xor260_b0, dbg_xor260_b1, dbg_xor260_b2;

volatile uint8_t xor380[31], pay380_prev[31];
//volatile uint8_t havePrev380;
volatile uint8_t dbg_xor380_b0, dbg_xor380_b1;

/* 0x260 extra (len 20) */
static uint8_t  xor260_20[20];
static uint8_t  dbg_xor260_20_b0, dbg_xor260_20_b1;

/* 0x380 prev-length tracking */
static uint8_t  len380_prev;

static uint8_t cand260[64];
static uint8_t candLen260 = 0;
static uint8_t candValid260 = 0;
static uint8_t candRepeats260 = 0;

//static uint8_t stable260[64];
static uint8_t stableLen260 = 0;
static uint32_t stable260Hits = 0;

// Debug taps
static uint8_t dbg_stableAccept = 0;
static uint8_t dbg_candRepeats = 0;
static uint8_t dbg_diffFirstIdx = 0xFF;

// --- NEW: 0x260 stability latch ---
static uint8_t  last260_raw[64];
static uint8_t  last260_raw_len = 0;

static uint8_t  stable260[64];
//static uint8_t  stable260_len = 0;

static uint32_t stable260_acceptCount = 0;
static uint32_t stable260_rejectCount = 0;

// --- 3478A: stability filter for 0x260 payload ---
uint8_t  stable260_sameCount = 0;
uint8_t  stable260_candLen = 0;
uint8_t  stable260_cand[64];

//uint8_t  stable260_len = 0;
uint8_t  stable260_pay[64];
//uint32_t stable260_acceptCount = 0;
//uint32_t stable260_rejectCount = 0;

static uint8_t havePrev260_20 = 0;
static uint8_t prev260_20[20];

static uint8_t havePrev260_31 = 0;
static uint8_t prev260_31[31];

volatile uint8_t  pay260_raw[64];
volatile uint8_t  len260_raw = 0;

volatile uint8_t payloadIdx = 0;
volatile uint8_t payload[64];

volatile uint8_t pay260_diff[64];
volatile uint8_t len260_diff = 0;

volatile uint8_t pay260_diff_latched[64];
volatile uint8_t len260_diff_latched = 0;
volatile uint8_t pay260_diff_latched_valid = 0;

volatile uint8_t pay260_prev_valid = 0;

volatile uint8_t pay260_latch_armed = 0;

volatile uint8_t pay260_latch_enable = 0;

volatile uint8_t last260_ff_count = 0;
volatile uint8_t last260_00_count = 0;

volatile uint8_t prevSyncLevel = 0;

volatile uint8_t ina_ff_run = 0;

volatile uint8_t pay260_raw_latched[64];
volatile uint8_t len260_raw_latched = 0;

volatile uint8_t pay260_latch_timeout = 0;

volatile uint8_t pay260_raw_latched2[64];
volatile uint8_t len260_raw_latched2 = 0;

volatile uint8_t pay260_latch_hits = 0;   /* how many update packets captured this arm (0,1,2) */

volatile uint8_t pay260_raw_next[64];
volatile uint8_t len260_raw_next = 0;
volatile uint8_t pay260_wait_next = 0;

volatile uint8_t pay260_seq[6][32];
volatile uint8_t pay260_seq_len[6];
volatile uint8_t pay260_seq_count = 0;
volatile uint8_t pay260_seq_active = 0;

volatile uint8_t pay260_upd[4][32];
volatile uint8_t pay260_upd_len[4];
volatile uint8_t pay260_upd_count = 0;
volatile uint8_t pay260_upd_active = 0;

volatile uint16_t pay260_upd_timeout = 0;

uint32_t pay260_frames_per_sec = 0;
uint32_t pay260_upd_per_sec = 0;
uint32_t pay260_frames_in_window = 0;
uint32_t pay260_upd_in_window = 0;
uint32_t pay260_rate_last_tick = 0;

/* ---- 0x260 frame-rate meters (units: frames/sec) ---- */
volatile uint32_t fps260 = 0;                 /* measured 0x260 frames/sec */
volatile uint32_t fps260_win_count = 0;
volatile uint32_t fps260_last_ms = 0;

volatile uint32_t fps260_upd = 0;             /* measured update/write pkts/sec (30,24,7) */
volatile uint32_t fps260_upd_win_count = 0;
volatile uint32_t fps260_upd_last_ms = 0;

/* Optional: total counters (handy sanity checks) */
volatile uint32_t cnt260_total = 0;
volatile uint32_t cnt260_upd_total = 0;

#define MS_PER_SEC 1000u

uint32_t pay260_upd_deadline_ms = 0;     /* absolute tick when update-capture stops */
uint32_t pay260_upd_window_ms = 2500;  /* capture window length in ms (tweakable) */

//volatile uint8_t inaGap2 = 0; // counts down 2 dummy clocks after SYNC falling edge

uint8_t isaTenCount;

uint8_t isaByteShift;

uint8_t isaByteBitCount;

uint8_t isaCmdBytes[4];

uint8_t isaCmdByteCount;

//uint8_t currentCmdBytes[8];

//uint8_t currentCmdLen;

//uint8_t currentCmdId;

//uint8_t lastCmdBytes[8];

//uint8_t lastCmdLen;

//uint8_t lastCmdId;

uint8_t payloadBuf[128];

uint16_t payloadBufLen;

//uint8_t lastPayload[128];

//uint16_t lastPayloadLen;
//static uint8_t  prevSync = 0;

//static uint8_t  isaGap2 = 0;
//static uint8_t  inaGap2 = 0;

//static uint8_t  isaShift = 0;
//static uint8_t  isaBitCount8 = 0;
//static uint8_t  isaBytes[8];
//static uint8_t  isaLen = 0;

//static uint8_t  inaShift = 0;
//static uint8_t  inaBitCount8 = 0;

/* 3478A ISA command capture (8-bit, LSB first) */
static uint8_t  isaCmdByte = 0;
static uint8_t  isaCmdBitCount = 0;
static uint8_t  isaCmdCaptured = 0;   /* 1 once we’ve latched the command for this SYNC-high */
static uint8_t  isaSyncHighClkCount = 0; /* debug: clocks seen while SYNC high */

/* ================= 3478A ISA/INA GLOBALS ================= */

volatile uint8_t  prevSync = 0;

volatile uint8_t  isaGap2 = 0;
volatile uint8_t  inaGap2 = 0;

volatile uint8_t  isaShift = 0;
volatile uint8_t  isaBitCount8 = 0;
volatile uint8_t  isaBytes[8];
volatile uint8_t  isaLen = 0;

volatile uint8_t  inaShift = 0;
volatile uint8_t  inaBitCount8 = 0;

volatile uint8_t  currentCmdBytes[8];
volatile uint8_t  currentCmdLen = 0;
volatile uint8_t  currentCmdId = 0;

volatile uint8_t  lastCmdBytes[8];
volatile uint8_t  lastCmdLen = 0;
volatile uint8_t  lastCmdId = 0;

volatile uint16_t lastPayloadLen = 0;
volatile uint8_t  lastPayload[128];   /* make big enough */

//uint8_t prevPwo = 0;
uint8_t pwoNow = 0;

volatile uint32_t dbg_sync_rise = 0;
volatile uint32_t dbg_sync_fall = 0;
volatile uint32_t dbg_isa_bits = 0;
volatile uint32_t dbg_isa_bytes = 0;
volatile uint32_t dbg_ina_bytes = 0;

uint8_t prevPwo = 0;

//volatile uint8_t  prevSync = 0;

//volatile uint8_t  isaShift = 0;
//volatile uint8_t  isaBitCount8 = 0;
volatile uint8_t  isaBytes[8];
//volatile uint8_t  isaLen = 0;

//volatile uint8_t  inaShift = 0;
//volatile uint8_t  inaBitCount8 = 0;




//*************************
//volatile uint8_t dbgCode[12];


static uint8_t Same260_AllowBlink(const uint8_t* a, const uint8_t* b, uint8_t len, uint8_t* firstDiffIdx)
{
    // Allow byte[0] to toggle (you observed 0 <-> 96 with the flashing DP)
    // Everything else must match to be considered “same”.
    for (uint8_t i = 0; i < len; i++) {
        if (i == 0) continue;                 // ignore blink byte
        if (a[i] != b[i]) {
            if (firstDiffIdx) *firstDiffIdx = i;
            return 0;
        }
    }
    if (firstDiffIdx) *firstDiffIdx = 0xFF;
    return 1;
}

static void LatchStable260(const uint8_t* frame, uint8_t len)
{
    if (len > sizeof(stable260)) len = sizeof(stable260);
    for (uint8_t i = 0; i < len; i++) stable260[i] = frame[i];
    stableLen260 = len;
    stable260Hits++;
}




void DMM_HandleO2Clock(void)
{
    GPIO_PinState syncLevel = HAL_GPIO_ReadPin(DMM_SYNC_GPIO_Port, DMM_SYNC_Pin);

    lastSync = (syncLevel == GPIO_PIN_SET) ? 1 : 0;
    syncState = lastSync;

    if (syncLevel == GPIO_PIN_SET) syncHighCount++;
    else                           syncLowCount++;

    O2Count++;

    /* Only decode while PWO is active */
    if (HAL_GPIO_ReadPin(DMM_PWO_GPIO_Port, DMM_PWO_Pin) != GPIO_PIN_SET) {
        return;
    }

    /* ================= ISA / INA (3478A variable ISA length) ================= */

    /* SYNC level (sampled each O2 clock) */
    uint8_t syncNow = (syncLevel == GPIO_PIN_SET) ? 1u : 0u;

    /* --------- EDGE DETECT (MUST RUN EVERY O2 CLOCK) --------- */

    /* SYNC rising: INA -> ISA */
    if ((prevSync == 0u) && (syncNow == 1u)) {

        /* CLOSE OUT PREVIOUS INA payload (belongs to previous command) */
        lastCmdId = currentCmdId;
        lastCmdLen = currentCmdLen;

        for (uint8_t i = 0; i < sizeof(lastCmdBytes); i++) {
            if (i < lastCmdLen && i < sizeof(currentCmdBytes)) lastCmdBytes[i] = currentCmdBytes[i];
            else                                               lastCmdBytes[i] = 0;
        }

        lastPayloadLen = payloadBufLen;
        if (lastPayloadLen > sizeof(lastPayload)) lastPayloadLen = sizeof(lastPayload);

        for (uint16_t i = 0; i < lastPayloadLen; i++) {
            lastPayload[i] = payloadBuf[i];
        }

        /* mirrors (optional) */
        lastCmdPrev = lastCmdId;
        lastInaLenPrev = (uint16_t)payloadBufLen;

        /* START NEW ISA burst: skip 2 dummy clocks */
        isaGap2 = 2;
        isaShift = 0;
        isaBitCount8 = 0;
        isaLen = 0;

        /* Reset INA accumulator ready for next INA phase */
        payloadBufLen = 0;
        inaGap2 = 0;
        inaShift = 0;
        inaBitCount8 = 0;

        dbg_sync_rise++;
    }
    /* SYNC falling: ISA -> INA */
    else if ((prevSync == 1u) && (syncNow == 0u)) {

        /* FINALIZE ISA burst into currentCmdBytes */
        currentCmdLen = isaLen;

        for (uint8_t i = 0; i < sizeof(currentCmdBytes); i++) {
            if (i < currentCmdLen && i < sizeof(isaBytes)) currentCmdBytes[i] = isaBytes[i];
            else                                           currentCmdBytes[i] = 0;
        }

        currentCmdId = (currentCmdLen > 0u) ? currentCmdBytes[0] : 0u;

        /* mirrors (optional) */
        lastCmd10 = currentCmdId;
        lastCmd = currentCmdId;
        currentCmd10 = currentCmdId;

        /* START INA burst: skip 2 dummy clocks */
        inaGap2 = 2;
        inaShift = 0;
        inaBitCount8 = 0;

        payloadBufLen = 0;

        dbg_sync_fall++;
    }

    /* --------- DATA SAMPLING --------- */

    if (syncNow == 1u) {

        /* ================= ISA bytes ================= */
        uint8_t bit = (uint8_t)HAL_GPIO_ReadPin(DMM_ISA_GPIO_Port, DMM_ISA_Pin) & 1u;
        dbg_isa_bits++;

        if (isaGap2) {
            isaGap2--;
        }
        else {
            isaShift |= (uint8_t)(bit << isaBitCount8);
            isaBitCount8++;

            if (isaBitCount8 == 8u) {
                if (isaLen < sizeof(isaBytes)) {
                    isaBytes[isaLen++] = isaShift;
                }
                isaShift = 0;
                isaBitCount8 = 0;
                dbg_isa_bytes++;
            }
        }
    }
    else {

        /* ================= INA bytes ================= */
        uint8_t bit = (uint8_t)HAL_GPIO_ReadPin(DMM_INA_GPIO_Port, DMM_INA_Pin) & 1u;

        if (inaGap2) {
            inaGap2--;
            /* do not return; just skip dummy clocks */
        }
        else {
            inaShift |= (uint8_t)(bit << inaBitCount8);
            inaBitCount8++;

            if (inaBitCount8 == 8u) {
                uint8_t b = inaShift;
                inaShift = 0;
                inaBitCount8 = 0;

                if (payloadBufLen < sizeof(payloadBuf)) {
                    payloadBuf[payloadBufLen++] = b;
                }

                dbg_ina_bytes++;
            }
        }
    }

    /* Update prevSync ONCE, at the very end */
    prevSync = syncNow;
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
static uint8_t HP3457_GetCharCode(uint8_t d)
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
static uint8_t HP3457_GetPunct(uint8_t d)
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


static char HP3457_CodeToAscii(uint8_t code)
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


void HP3457_BuildDisplayString(char out12[13], char punct12[13])
{
    for (int i = 0; i < 12; i++) {
        uint8_t d = (uint8_t)(i + 1);   // d = 1..12  (was 12-i)

        uint8_t code = HP3457_GetCharCode(d);

        if (code == 0x3F) code = 0x3D;   // force '='

        out12[i] = HP3457_CodeToAscii(code);

        uint8_t p = HP3457_GetPunct(d);
        punct12[i] = (p == 1) ? '.' : (p == 2) ? ':' : (p == 3) ? ',' : ' ';
    }

    // Use this to live watch in order to get code for ? chars - live watch ---> dbgCode[12]
    //for (int i = 0; i < 12; i++) {
    //    uint8_t d = (uint8_t)(i + 1);
    //    dbgCode[i] = HP3457_GetCharCode(d);
    //}

    out12[12] = '\0';
    punct12[12] = '\0';
}


static uint8_t HP3457_GetCharCode_Alt(uint8_t d)
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