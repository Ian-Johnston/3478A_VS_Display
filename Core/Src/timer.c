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
//volatile uint8_t  frameReady = 0;

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
//uint8_t pwoNow = 0;

//volatile uint32_t dbg_sync_rise = 0;
//volatile uint32_t dbg_sync_fall = 0;
volatile uint32_t dbg_isa_bits = 0;
volatile uint32_t dbg_isa_bytes = 0;
volatile uint32_t dbg_ina_bytes = 0;

//uint8_t prevPwo = 0;

//volatile uint8_t  prevSync = 0;

//volatile uint8_t  isaShift = 0;
//volatile uint8_t  isaBitCount8 = 0;
volatile uint8_t  isaBytes[8];
//volatile uint8_t  isaLen = 0;

//volatile uint8_t  inaShift = 0;
//volatile uint8_t  inaBitCount8 = 0;


/* NEW globals for raw capture + alignment */
uint8_t  pwoPrev = 0;
//uint32_t dbg_pwo_rise = 0;
//uint32_t dbg_pwo_fall = 0;

//uint16_t isaRawLen = 0;
//uint8_t  isaRawBits[128];   /* enough for ISA bursts */

//uint16_t inaRawLen = 0;
//uint8_t  inaRawBits[512];   /* enough for INA bursts */

uint8_t  isaBitAlign = 0xFF; /* 0..7 when known, 0xFF unknown */
uint8_t  isaAlignLocked = 0; /* 0/1: once we find a good align in this PWO frame */

volatile uint8_t  isaRawBits[64];
volatile uint16_t isaRawLen = 0;

volatile uint8_t  inaRawBits[512];
volatile uint16_t inaRawLen = 0;

volatile uint8_t  isaCmdCandidates[8];

volatile uint32_t dbg_sync_rise = 0;
volatile uint32_t dbg_sync_fall = 0;

uint8_t inaBitAlign = 0;


/* Raw per-PWO-frame capture (bits, LSB-first in time order as sampled) */
volatile uint8_t  frameIsa[64];
volatile uint16_t frameIsaLen = 0;      /* number of ISA bits captured this PWO frame */

volatile uint8_t  frameInaN[512];
volatile uint16_t frameInaLen = 0;      /* number of INA bits captured this PWO frame */

/* Optional: a stable snapshot you can inspect after PWO falls */
volatile uint8_t  frameIsaLatched[64];
volatile uint16_t frameIsaLatchedLen = 0;

volatile uint8_t  frameInaLatched[512];
volatile uint16_t frameInaLatchedLen = 0;

volatile uint8_t  frameReady = 0;       /* set to 1 on PWO falling edge */

volatile uint8_t  prevPwo = 0;          /* for PWO edge detect */

volatile uint32_t dbg_pwo_rise = 0;
volatile uint32_t dbg_pwo_fall = 0;
volatile uint32_t dbg_frame_isa_ovf = 0;
volatile uint32_t dbg_frame_ina_ovf = 0;

volatile uint8_t  frameLatchEnable = 0;   /* set to 1 in Live Watch to freeze NEXT complete PWO frame */
volatile uint8_t  frameHold = 0;          /* internal: 1 = latched and holding, 0 = free-running */


/* ===== Frame capture (PWO window) ===== */
volatile uint8_t  frameIsa[64];
volatile uint16_t frameIsaLen;

volatile uint8_t  frameInaN[512];
volatile uint16_t frameInaLen;

/* ===== Latched frame ===== */
volatile uint8_t  frameIsaLatched[64];
volatile uint16_t frameIsaLatchedLen;

volatile uint8_t  frameInaLatched[512];
volatile uint16_t frameInaLatchedLen;

volatile uint8_t  frameLatchEnable;   /* set this to 1 in Live Watch to latch next full frame */
volatile uint8_t  frameHold;          /* goes 1 when latch happened */
volatile uint8_t  frameReady;         /* pulses 1 when a frame is latched (then cleared next call) */

volatile uint8_t  frameBaselineValid = 0;
volatile uint16_t frameIsaBaseLen = 0;
volatile uint16_t frameInaBaseLen = 0;
volatile uint8_t  frameIsaBase[64];
volatile uint8_t  frameInaBase[512];

volatile uint8_t  frameIsa[64];      /* bits: 0/1 */
volatile uint8_t  frameIna[512];     /* bits: 0/1 */
volatile uint32_t dbg_frame_diff = 0;

volatile uint8_t  inFrame = 0;
volatile uint8_t  framePrevValid = 0;
volatile uint16_t frameIsaPrevLen = 0;
volatile uint16_t frameInaPrevLen = 0;
volatile uint8_t  frameIsaPrev[64];
volatile uint8_t  frameInaPrev[512];
volatile uint32_t dbg_frame_latched = 0;
volatile uint32_t dbg_frame_end = 0;
volatile uint8_t  frameActive = 0;
volatile uint8_t  inPwoWindow;
volatile uint8_t  syncRiseCountInPwo;
volatile uint8_t  dbg_sync_rise_in_pwo;
volatile uint8_t  frameReady;
volatile uint8_t  frameKind;

volatile uint8_t  _pwoPrev;
volatile uint8_t  _syncPrevInPwo;
volatile uint8_t  _syncRiseCountInPwo;
uint8_t  frameSyncRiseCount = 0;
uint32_t dbg_ina_bits = 0;
volatile uint8_t  syncRiseInPwo = 0;

volatile uint8_t frameKindLatched = 0;       /* 0=none, 1=single-sync frame, 2=double-sync frame */
volatile uint8_t frameLatchEnablePrev = 0;  /* internal: detect re-arm */

volatile uint16_t frameO2Len = 0;          /* live timeline length (O2 clocks inside PWO) */
volatile uint16_t frameO2LatchedLen = 0;   /* latched timeline length */

//volatile uint8_t  frameKindLatched = 0;    /* 0 none, 1 single-sync frame, 2 dual-sync frame */

/* ---- Decoded display RAM from last latched frame ---- */
volatile uint8_t disp3478_ram[12];
volatile uint8_t disp3478_hi[12];
volatile uint8_t disp3478_lo[12];
volatile uint8_t disp3478_valid;

/* Optional debug: what ISA bytes were seen in the latched frame */
volatile uint8_t disp3478_isa_list[16];
volatile uint8_t disp3478_isa_list_len;

volatile uint8_t  frameO2Latched[512];

volatile uint8_t frameO2[512];

volatile uint8_t frameReadySticky = 0;

//volatile uint32_t dbg_decode_hits = 0;
volatile uint8_t  disp3478_bestOff = 0;
//volatile uint8_t  disp3478_bestMSB = 0;
//volatile uint8_t  disp3478_bestScore = 0;


volatile uint8_t  disp3478_isa8_lsb[16];
volatile uint8_t  disp3478_isa8_msb[16];
volatile uint8_t  disp3478_isa10_lsb[16];
volatile uint8_t  disp3478_isa10_msb[16];
volatile uint8_t  disp3478_isa8_len;
volatile uint8_t  disp3478_isa10_len;

volatile uint8_t  disp3478_ina8_lsb[64];
volatile uint8_t  disp3478_ina8_len;

//volatile uint16_t dbg_decode_hits;

volatile uint8_t disp3478_isa_bits_len;

volatile uint8_t  disp3478_seen_1A = 0;
volatile uint8_t  disp3478_seen_0A = 0;
volatile uint8_t  disp3478_opcode_score = 0;
volatile uint32_t dbg_decode_hits = 0;


volatile uint8_t  disp3478_bestMSB = 0;
volatile uint8_t  disp3478_bestScore = 0;
//volatile uint8_t  disp3478_opcode_hits = 0;
//volatile uint8_t  disp3478_saw_1A = 0;
//volatile uint8_t  disp3478_saw_0A = 0;


volatile uint8_t  disp3478_bestStep = 0;   /* bytes are spaced by this many clocks */
volatile uint8_t  disp3478_bestPre = 0;   /* dummy clocks before the 8 bits */
volatile uint8_t  disp3478_bestPost = 0;   /* dummy clocks after the 8 bits */

volatile uint8_t  disp3478_opcode_hits = 0;
volatile uint8_t  disp3478_saw_1A = 0;
volatile uint8_t  disp3478_saw_0A = 0;

volatile uint8_t  disp3478_ina_bits[512];
volatile uint16_t disp3478_ina_bitcount;

volatile uint8_t disp3478_bestInv = 0;

volatile uint64_t disp3478_isaBitsPacked = 0;   // LSB = first ISA bit captured
volatile uint8_t  disp3478_isaBitsPackedLen = 0;

volatile uint8_t  disp3478_tryScore[2][3];      // [inv][pre]
volatile uint8_t  disp3478_tryBytes[2][3][8];   // [inv][pre][word], up to 8 words
volatile uint8_t  disp3478_tryWords = 0;        // number of 10-clock words decoded





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
    uint8_t syncNow = (syncLevel == GPIO_PIN_SET) ? 1u : 0u;

    lastSync = syncNow;
    syncState = syncNow;

    if (syncNow) syncHighCount++;
    else         syncLowCount++;

    O2Count++;

    /* Sample PWO every O2 clock so we can detect end-of-frame */
    uint8_t pwoNow = (HAL_GPIO_ReadPin(DMM_PWO_GPIO_Port, DMM_PWO_Pin) == GPIO_PIN_SET) ? 1u : 0u;

    /* ---------------- PWO FALLING EDGE = END OF FRAME ---------------- */
    if ((prevPwo == 1u) && (pwoNow == 0u)) {

        /* Decide frame kind ONLY here (no flicker) */
        uint8_t frameKindNow = (syncRiseInPwo >= 2u) ? 2u : 1u;

        /* Latch only if armed, not already holding, and it's a kind-2 frame */
        if ((frameLatchEnable == 1u) && (frameHold == 0u) && (frameKindNow == 2u)) {

            /* Copy lengths (clamp) */
            frameIsaLatchedLen = frameIsaLen;
            if (frameIsaLatchedLen > (uint16_t)sizeof(frameIsaLatched)) frameIsaLatchedLen = sizeof(frameIsaLatched);

            frameInaLatchedLen = frameInaLen;
            if (frameInaLatchedLen > (uint16_t)sizeof(frameInaLatched)) frameInaLatchedLen = sizeof(frameInaLatched);

            frameO2LatchedLen = frameO2Len;
            if (frameO2LatchedLen > (uint16_t)sizeof(frameO2Latched))  frameO2LatchedLen = sizeof(frameO2Latched);

            /* Copy ISA bits */
            for (uint16_t i = 0; i < frameIsaLatchedLen; i++) frameIsaLatched[i] = frameIsa[i];
            for (uint16_t i = frameIsaLatchedLen; i < sizeof(frameIsaLatched); i++) frameIsaLatched[i] = 0;

            /* Copy INA bits */
            for (uint16_t i = 0; i < frameInaLatchedLen; i++) frameInaLatched[i] = frameIna[i];
            for (uint16_t i = frameInaLatchedLen; i < sizeof(frameInaLatched); i++) frameInaLatched[i] = 0;

            /* Copy tagged O2 stream */
            for (uint16_t i = 0; i < frameO2LatchedLen; i++) frameO2Latched[i] = frameO2[i];
            for (uint16_t i = frameO2LatchedLen; i < sizeof(frameO2Latched); i++) frameO2Latched[i] = 0;

            frameKindLatched = frameKindNow;
            frameHold = 1u;
            frameReady = 1u;          /* one-shot pulse (decoder consumes it) */
            frameReadySticky = 1u;    /* sticky: stays 1 so you can SEE a capture happened */
            frameLatchEnable = 0u;
        }

        /* Reset per-frame counters/buffers for next frame */
        syncRiseInPwo = 0;

        frameIsaLen = 0;
        frameInaLen = 0;
        frameO2Len = 0;

        /* Re-seed edge detect for next frame */
        prevSync = syncNow;
        prevPwo = pwoNow;
        return;
    }

    /* Track PWO state for next call */
    prevPwo = pwoNow;

    /* If PWO not active, do not collect anything */
    if (pwoNow == 0u) {
        prevSync = syncNow;
        return;
    }

    /* ---------------- Inside PWO window: count SYNC edges ---------------- */
    if ((prevSync == 0u) && (syncNow == 1u)) {
        syncRiseInPwo++;
        dbg_sync_rise++;
    }
    else if ((prevSync == 1u) && (syncNow == 0u)) {
        dbg_sync_fall++;
    }
    prevSync = syncNow;

    /* ---------------- Collect RAW bits ---------------- */
    if (syncNow == 1u) {
        /* ISA bit */
        uint8_t bit = (uint8_t)(HAL_GPIO_ReadPin(DMM_ISA_GPIO_Port, DMM_ISA_Pin) & 1u);
        dbg_isa_bits++;

        if (frameIsaLen < sizeof(frameIsa)) frameIsa[frameIsaLen++] = bit;

        /* tagged stream: (pwo<<2)|(sync<<1)|bit */
        if (frameO2Len < sizeof(frameO2)) frameO2[frameO2Len++] = (uint8_t)((1u << 2) | (1u << 1) | (bit & 1u));
    }
    else {
        /* INA bit */
        uint8_t bit = (uint8_t)(HAL_GPIO_ReadPin(DMM_INA_GPIO_Port, DMM_INA_Pin) & 1u);
        dbg_ina_bits++;

        if (frameInaLen < sizeof(frameIna)) frameIna[frameInaLen++] = bit;

        /* tagged stream */
        if (frameO2Len < sizeof(frameO2)) frameO2[frameO2Len++] = (uint8_t)((1u << 2) | (0u << 1) | (bit & 1u));
    }
}




static uint8_t is_known_opcode(uint8_t b)
{
    /* 3478A ROM disassembly shows these instruction bytes */
    switch (b)
    {
    case 0xFC:
    case 0xFD:
    case 0x24:
    case 0xB8:
    case 0xC8:
    case 0x2A:
    case 0xBC:
    case 0x53:
    case 0x1A:
    case 0x4F:
    case 0xD0:
        return 1u;
    default:
        return 0u;
    }
}



void Decode3478_LatchedFrame(void)
{
    if (frameReady == 0u)
        return;

    frameReady = 0u;

    disp3478_isa_list_len = 0u;
    disp3478_opcode_hits = 0u;
    disp3478_saw_1A = 0u;
    disp3478_saw_0A = 0u;

    disp3478_bestScore = 0u;
    disp3478_bestStep = 10u;   /* proven */
    disp3478_bestPre = 0u;
    disp3478_bestPost = 0u;
    disp3478_bestMSB = 0u;    /* assume LSB-first */
    disp3478_bestOff = 0u;
    disp3478_bestInv = 1u;    /* proven by your tryScore matrix */

    const uint8_t* s = frameO2Latched;
    uint16_t N = frameO2LatchedLen;
    if (N > 512u) N = 512u;

    /* Collect ISA bits where SYNC==1 */
    uint8_t isaBits[128];
    uint16_t isaBitsLen = 0;

    for (uint16_t i = 0; i < N; i++)
    {
        uint8_t sync = (uint8_t)((s[i] >> 1) & 1u);
        if (sync)
        {
            uint8_t bit = (uint8_t)(s[i] & 1u);
            if (isaBitsLen < (uint16_t)sizeof(isaBits))
                isaBits[isaBitsLen++] = bit;
        }
    }

    disp3478_isa_bits_len = (uint8_t)((isaBitsLen > 255u) ? 255u : isaBitsLen);

    if (isaBitsLen < 10u)
        return;

    /* Search offset 0..9 and pre 0..2, with inversion forced, LSB-first */
    uint8_t bestScore = 0u;
    uint8_t bestOff = 0u;
    uint8_t bestPre = 0u;

    for (uint8_t off = 0u; off < 10u; off++)
    {
        for (uint8_t pre = 0u; pre < 3u; pre++)
        {
            uint8_t score = 0u;
            uint16_t idx = off;

            while ((uint16_t)(idx + pre + 7u) < isaBitsLen)
            {
                uint8_t b = 0u;
                for (uint8_t k = 0u; k < 8u; k++)
                {
                    uint8_t bit = isaBits[idx + pre + k];
                    /* inversion forced */
                    bit = (uint8_t)(1u - bit);
                    b |= (uint8_t)((bit & 1u) << k); /* LSB-first */
                }

                if (is_known_opcode(b))
                    score++;

                idx = (uint16_t)(idx + 10u);
            }

            /* choose best; tie-break prefer pre=1, then smaller off */
            uint8_t take = 0u;
            if (score > bestScore) take = 1u;
            else if (score == bestScore && score != 0u)
            {
                if (bestPre != 1u && pre == 1u) take = 1u;
                else if (bestPre == pre && off < bestOff) take = 1u;
            }

            if (take)
            {
                bestScore = score;
                bestOff = off;
                bestPre = pre;
            }
        }
    }

    disp3478_bestScore = bestScore;
    disp3478_bestOff = bestOff;
    disp3478_bestPre = bestPre;
    disp3478_bestPost = (uint8_t)(10u - (bestPre + 8u));
    disp3478_bestMSB = 0u;
    disp3478_bestInv = 1u;

    /* Decode ISA bytes using the chosen off+pre */
    uint16_t idx = bestOff;

    while ((uint16_t)(idx + bestPre + 7u) < isaBitsLen)
    {
        uint8_t b = 0u;
        for (uint8_t k = 0u; k < 8u; k++)
        {
            uint8_t bit = isaBits[idx + bestPre + k];
            bit = (uint8_t)(1u - bit); /* invert */
            b |= (uint8_t)((bit & 1u) << k);
        }

        if (disp3478_isa_list_len < (uint8_t)sizeof(disp3478_isa_list))
            disp3478_isa_list[disp3478_isa_list_len++] = b;

        if (is_known_opcode(b))
        {
            disp3478_opcode_hits++;
            if (b == 0x1A) disp3478_saw_1A = 1u;
            if (b == 0x0A) disp3478_saw_0A = 1u;
        }

        idx = (uint16_t)(idx + 10u);
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