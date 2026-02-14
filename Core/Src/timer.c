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

#define MAX_BUFFER_SIZE 256

volatile uint8_t syncState = 0;
volatile uint32_t O2Count;
volatile uint8_t  lastSync = 0;
volatile uint32_t syncHighCount = 0;
volatile uint32_t syncLowCount = 0;

volatile uint16_t lastCmd = 0;
volatile uint8_t  lastDataByte = 0;

volatile uint8_t  regA[6], regB[6], regC[6];
volatile uint8_t  ann[2];

volatile uint8_t digitCode[12];   // 7-bit HP char code per digit (1..12 -> index 0..11)
volatile uint8_t punctCode[12];   // 0..3 (none, '.', ':', ',')

volatile uint16_t cmdSeen[8] = { 0 };
volatile uint32_t cmdSeenCount[8] = { 0 };
volatile uint8_t digitVal[12];   // 0–9, or 0xF for blank
volatile int8_t digitDec[12];

volatile uint8_t numStart = 0;
volatile uint8_t numLen = 0;
volatile uint8_t numDigits[12];
static void ExtractBestRun(void);
volatile uint8_t regANib[12];      // 12 nibbles extracted from regA
volatile uint8_t regANibPrev[12];

volatile uint8_t regANibChanged[12]; // 1 if that nibble changed this frame
volatile uint16_t changedMask;
volatile char displayWithPunct[32]; // debug/combined string + \0

static uint8_t stableLen260 = 0;
static uint32_t stable260Hits = 0;

static uint8_t  stable260[64];

volatile uint8_t  prevSync = 0;

volatile uint32_t dbg_isa_bits = 0;
volatile uint32_t dbg_isa_bytes = 0;
volatile uint32_t dbg_ina_bytes = 0;

volatile uint32_t dbg_sync_rise = 0;
volatile uint32_t dbg_sync_fall = 0;

volatile uint8_t  prevPwo = 0;          /* for PWO edge detect */

volatile uint8_t  frameIsa[64];
volatile uint16_t frameIsaLen;
volatile uint16_t frameInaLen;
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

uint32_t dbg_ina_bits = 0;
volatile uint8_t  syncRiseInPwo = 0;

volatile uint8_t frameKindLatched = 0;       /* 0=none, 1=single-sync frame, 2=double-sync frame */
volatile uint8_t frameLatchEnablePrev = 0;  /* internal: detect re-arm */

volatile uint16_t frameO2Len = 0;          /* live timeline length (O2 clocks inside PWO) */
volatile uint16_t frameO2LatchedLen = 0;   /* latched timeline length */

volatile uint8_t disp3478_ram[12];
volatile uint8_t disp3478_hi[12];
volatile uint8_t disp3478_lo[12];
volatile uint8_t disp3478_valid;
volatile uint8_t disp3478_isa_list[16];
volatile uint8_t disp3478_isa_list_len;

volatile uint8_t  frameO2Latched[512];

#define DISP3478_CAP_DATA_MAX   (512u)
#define DISP3478_CAP_HDR        (12u)
#define DISP3478_CAP_MAX        (DISP3478_CAP_HDR + DISP3478_CAP_DATA_MAX)
volatile uint8_t  disp3478_cap[520];     /* 8-byte header + 512 payload */
volatile uint16_t disp3478_capLen = 0u;  /* valid bytes in disp3478_cap */
volatile uint8_t  disp3478_capSeq = 0u;  /* increments each capture */


volatile uint8_t frameO2[512];
volatile uint8_t frameReadySticky = 0;

volatile uint8_t  disp3478_bestOff = 0;
volatile uint8_t  disp3478_isa8_lsb[16];
volatile uint8_t  disp3478_isa8_msb[16];
volatile uint8_t  disp3478_isa10_lsb[16];
volatile uint8_t  disp3478_isa10_msb[16];
volatile uint8_t  disp3478_isa8_len;
volatile uint8_t  disp3478_isa10_len;
volatile uint8_t  disp3478_ina8_lsb[64];
volatile uint8_t  disp3478_ina8_len;
volatile uint8_t  disp3478_isa_bits_len;
volatile uint8_t  disp3478_seen_1A = 0;
volatile uint8_t  disp3478_seen_0A = 0;
volatile uint8_t  disp3478_opcode_score = 0;
volatile uint32_t dbg_decode_hits = 0;
volatile uint8_t  disp3478_bestMSB = 0;
volatile uint8_t  disp3478_bestScore = 0;
volatile uint8_t  disp3478_bestStep = 0;   /* bytes are spaced by this many clocks */
volatile uint8_t  disp3478_bestPre = 0;   /* dummy clocks before the 8 bits */
volatile uint8_t  disp3478_bestPost = 0;   /* dummy clocks after the 8 bits */
volatile uint8_t  disp3478_opcode_hits = 0;
volatile uint8_t  disp3478_saw_1A = 0;
volatile uint8_t  disp3478_saw_0A = 0;
volatile uint8_t  disp3478_ina_bits[512];
volatile uint16_t disp3478_ina_bitcount;
volatile uint8_t  disp3478_bestInv = 0;
volatile uint64_t disp3478_isaBitsPacked = 0;   // LSB = first ISA bit captured
volatile uint8_t  disp3478_isaBitsPackedLen = 0;
volatile uint8_t  disp3478_tryScore[2][3];      // [inv][pre]
volatile uint8_t  disp3478_tryBytes[2][3][8];   // [inv][pre][word], up to 8 words
volatile uint8_t  disp3478_tryWords = 0;        // number of 10-clock words decoded

// Live Watch: choose whether O2 sampling is on rising or falling edge
volatile uint8_t sampleOnFallingEdge = 0;  // 0=rising (default), 1=falling

// Internal: remembers what polarity is currently applied to TIM3_CH4
static uint8_t appliedSampleOnFallingEdge = 0;

volatile uint8_t  disp3478_isa10_len = 0;
volatile uint16_t disp3478_isa10_list[16];   // 10-bit words stored in uint16_t

/* SYNC region breakdown inside latched frame (for debugging) */
volatile uint8_t  disp3478_segCount = 0;
volatile uint16_t disp3478_segLen[8];      // length in O2 samples
volatile uint8_t  disp3478_segSync[8];     // 0=INA region, 1=ISA region

/* INA nibble decode (first few nibbles per INA region) */
volatile uint8_t  disp3478_inaNibCount[8];

/* Raw tagged preview + SYNC count sanity */
volatile uint16_t disp3478_frameN = 0;
volatile uint16_t disp3478_sync1_count = 0;
volatile uint8_t  disp3478_tagPreview[64];

volatile uint16_t disp3478_segSum = 0;              // sum of segLen[] the decoder produced

volatile uint8_t  disp3478_segSyncFlat0 = 0, disp3478_segSyncFlat1 = 0, disp3478_segSyncFlat2 = 0, disp3478_segSyncFlat3 = 0, disp3478_segSyncFlat4 = 0;
volatile uint16_t disp3478_segLenFlat0 = 0, disp3478_segLenFlat1 = 0, disp3478_segLenFlat2 = 0, disp3478_segLenFlat3 = 0, disp3478_segLenFlat4 = 0;

volatile uint8_t disp3478_inaBestInv[8];
volatile uint8_t disp3478_inaBestOff[8];
volatile uint8_t disp3478_inaBestScore[8];

volatile uint8_t disp3478_inaNontrivCount[8];
volatile uint8_t disp3478_inaCountF[8];
volatile uint8_t disp3478_inaCount0[8];
volatile uint8_t disp3478_inaCountOther[8];

volatile uint8_t  disp3478_inaNib[8][64];
volatile uint8_t  disp3478_inaNontriv[8][64];

volatile uint8_t  disp3478_inaHist[8][16];      // nibble frequency table
volatile uint8_t  disp3478_inaModeNib[8];       // most frequent nibble value (0..15)
volatile uint8_t  disp3478_inaStripCount[8];
volatile uint8_t  disp3478_inaStripped[8][32];  // payload after stripping 0/F/mode

volatile uint8_t disp3478_inaBestStripNib[8];   // which nibble (besides 0/F) we stripped
volatile uint8_t disp3478_inaStripScore[8];     // how good the strip choice was
volatile uint8_t disp3478_inaStripped[8][32];   // already have, keep 32 for now

/* Store last STATIC decoded stripped lists */
volatile uint8_t disp3478_staticStripCount[8];
volatile uint8_t disp3478_staticStripped[8][32];

/* Store last CHANGE decoded stripped lists */
volatile uint8_t disp3478_changeStripCount[8];
volatile uint8_t disp3478_changeStripped[8][32];

/* Simple diff results for seg2 and seg4 */
volatile uint8_t disp3478_diffSeg2_first;
volatile uint8_t disp3478_diffSeg2_count;
volatile uint8_t disp3478_diffSeg2_xor[32];

volatile uint8_t disp3478_diffSeg4_first;
volatile uint8_t disp3478_diffSeg4_count;
volatile uint8_t disp3478_diffSeg4_xor[32];

volatile uint8_t  frameLatchModeStatic = 0;   /* set to 1 in Live Watch to latch the next PWO (static or change) */

/* Previous CHANGE decoded stripped lists (for change-vs-change diff) */
volatile uint8_t disp3478_havePrevChange = 0;
volatile uint8_t disp3478_prevChangeStripCount[8];
volatile uint8_t disp3478_prevChangeStripped[8][32];

/* Diff between consecutive CHANGE frames (seg2 and seg4) */
volatile uint8_t disp3478_diffChgSeg2_first;
volatile uint8_t disp3478_diffChgSeg2_count;
volatile uint8_t disp3478_diffChgSeg2_xor[32];

volatile uint8_t disp3478_diffChgSeg4_first;
volatile uint8_t disp3478_diffChgSeg4_count;
volatile uint8_t disp3478_diffChgSeg4_xor[32];
volatile uint8_t disp3478_haveStatic = 0;
volatile uint8_t disp3478_haveChange = 0;

volatile uint8_t disp3478_prevChangeStripCount4 = 0;
volatile uint8_t disp3478_prevChangeStripped4[32];

volatile uint8_t disp3478_diffChgSeg4_first = 0xFF;
volatile uint8_t disp3478_diffChgSeg4_count = 0;
volatile uint8_t disp3478_diffChgSeg4_xor[32];

volatile uint8_t disp3478_dbgDiffPrev4[32];
volatile uint8_t disp3478_dbgDiffCurr4[32];

volatile uint8_t disp3478_prevStrip4Count = 0;
volatile uint8_t disp3478_prevStrip4[32];
volatile uint8_t disp3478_havePrevStrip4 = 0;

volatile uint8_t disp3478_bestRot4 = 0;      // debug: chosen rotation
volatile uint8_t disp3478_bestRot4Score = 0; // debug: match score

volatile uint8_t disp3478_winStart = 0;
volatile uint8_t disp3478_winLen = 0;
volatile uint8_t disp3478_winPrev[16];
volatile uint8_t disp3478_winCurr[16];
volatile uint8_t disp3478_winXor[16];

volatile uint8_t disp3478_textLen = 0;
volatile char    disp3478_text[200];








//*****************************************************************************************



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
    /* ---------------- O2 edge select (rising vs falling) ----------------
       TIM3_CH4 is configured for input capture. We switch CC4 polarity live:
         CC4P=0 => rising
         CC4P=1 => falling
       You can change sampleOnFallingEdge in Live Watch before arming capture.
    */
    if (sampleOnFallingEdge != appliedSampleOnFallingEdge)
    {
        __disable_irq();
        if (sampleOnFallingEdge)
            TIM3->CCER |= TIM_CCER_CC4P;     // capture on falling
        else
            TIM3->CCER &= (uint16_t)~TIM_CCER_CC4P; // capture on rising
        __enable_irq();

        appliedSampleOnFallingEdge = sampleOnFallingEdge;
    }

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
    if ((prevPwo == 1u) && (pwoNow == 0u))
    {
        /* Decide frame kind ONLY here */
        uint8_t frameKindNow = (syncRiseInPwo >= 2u) ? 2u : 1u;

        /* Latch rules:
           - frameLatchEnable: latch ONLY digit-change frames (kind==2) (existing behaviour)
           - frameLatchModeStatic: latch NEXT PWO regardless of kind (new behaviour)
        */
        if ((frameHold == 0u) && ((frameLatchEnable == 1u && frameKindNow == 2u) || (frameLatchModeStatic == 1u)))
        {
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

            /* Disarm whichever mode was used */
            if (frameLatchEnable == 1u && frameKindNow == 2u)
                frameLatchEnable = 0u;

            if (frameLatchModeStatic == 1u)
                frameLatchModeStatic = 0u;
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



static void CaptureSeg4Window(const uint8_t* prev4,
    const uint8_t* curr4,
    const uint8_t* xor4,
    uint8_t first,
    uint8_t count)
{
    /* pick a 16-nibble window centered on the change */
    if (first == 0xFFu || count == 0u)
    {
        disp3478_winStart = 0u;
        disp3478_winLen = 0u;
        for (uint8_t i = 0u; i < 16u; i++) {
            disp3478_winPrev[i] = 0u;
            disp3478_winCurr[i] = 0u;
            disp3478_winXor[i] = 0u;
        }
        return;
    }

    uint8_t mid = (uint8_t)(first + (count / 2u));
    uint8_t start = (mid > 7u) ? (uint8_t)(mid - 7u) : 0u;

    /* clamp to 0..(32-16) since your dbg arrays are 32 max */
    if (start > 16u) start = 16u;

    disp3478_winStart = start;
    disp3478_winLen = 16u;

    for (uint8_t i = 0u; i < 16u; i++)
    {
        uint8_t idx = (uint8_t)(start + i);
        disp3478_winPrev[i] = prev4[idx];
        disp3478_winCurr[i] = curr4[idx];
        disp3478_winXor[i] = xor4[idx];
    }
}




static void Normalize3478_Seg4Strip(uint8_t* a, uint8_t n)
{
    if (n < 4u || n > 32u) return;

    /* No previous reference yet (or length changed): seed reference */
    if (disp3478_havePrevStrip4 == 0u || disp3478_prevStrip4Count != n)
    {
        disp3478_prevStrip4Count = n;
        for (uint8_t i = 0u; i < n; i++) disp3478_prevStrip4[i] = a[i];
        disp3478_havePrevStrip4 = 1u;

        disp3478_bestRot4 = 0u;
        disp3478_bestRot4Score = 0u;
        return;
    }

    /* Try all rotations; pick the one that maximises equality vs previous */
    uint8_t bestR = 0u;
    uint8_t bestScore = 0u;

    for (uint8_t r = 0u; r < n; r++)
    {
        uint8_t score = 0u;
        for (uint8_t i = 0u; i < n; i++)
        {
            uint8_t v = a[(uint8_t)((i + r) % n)];
            if (v == disp3478_prevStrip4[i]) score++;
        }

        if (score > bestScore)
        {
            bestScore = score;
            bestR = r;
        }
    }

    /* Apply chosen rotation */
    uint8_t tmp[32];
    for (uint8_t i = 0u; i < n; i++)
        tmp[i] = a[(uint8_t)((i + bestR) % n)];
    for (uint8_t i = 0u; i < n; i++)
        a[i] = tmp[i];

    /* Update reference */
    disp3478_prevStrip4Count = n;
    for (uint8_t i = 0u; i < n; i++) disp3478_prevStrip4[i] = a[i];

    disp3478_bestRot4 = bestR;
    disp3478_bestRot4Score = bestScore;
}






static void Process3478Run(const uint8_t* s,
    uint16_t runStart,
    uint16_t runEnd,
    uint8_t curSync,
    uint8_t seg)
{
    uint16_t runLen = (uint16_t)(runEnd - runStart);

    /* store segment info */
    disp3478_segSync[seg] = curSync;
    disp3478_segLen[seg] = runLen;
    disp3478_segSum = (uint16_t)(disp3478_segSum + runLen);

    if (curSync != 0u)
        return; /* only decode INA when SYNC==0 */

    /* ---- Find best inv/off for nibble decode (cap 32 nibbles) ---- */
    uint8_t bestInv = 0u, bestOff = 0u, bestScore = 0u;

    for (uint8_t inv = 0u; inv < 2u; inv++)
    {
        for (uint8_t off = 0u; off < 4u; off++)
        {
            uint16_t usable = (runLen > off) ? (uint16_t)(runLen - off) : 0u;
            uint16_t nibs = (uint16_t)(usable / 4u);
            if (nibs > 32u) nibs = 32u;

            uint8_t score = 0u;

            for (uint16_t n = 0; n < nibs; n++)
            {
                uint8_t v = 0u;
                for (uint8_t k = 0u; k < 4u; k++)
                {
                    uint16_t bi = (uint16_t)(runStart + off + (n * 4u) + k);
                    uint8_t bit = (uint8_t)(s[bi] & 1u);
                    if (inv) bit = (uint8_t)(1u - bit);
                    v |= (uint8_t)((bit & 1u) << k);
                }
                v &= 0xFu;
                if ((v != 0x0u) && (v != 0xFu)) score++;
            }

            if (score > bestScore)
            {
                bestScore = score;
                bestInv = inv;
                bestOff = off;
            }
        }
    }

    disp3478_inaBestInv[seg] = bestInv;
    disp3478_inaBestOff[seg] = bestOff;
    disp3478_inaBestScore[seg] = bestScore;

    /* ---- Decode nibbles using best settings ---- */
    uint16_t usable = (runLen > bestOff) ? (uint16_t)(runLen - bestOff) : 0u;
    uint16_t nibs = (uint16_t)(usable / 4u);
    if (nibs > 32u) nibs = 32u;

    disp3478_inaNibCount[seg] = (uint8_t)nibs;

    for (uint16_t n = 0; n < nibs; n++)
    {
        uint8_t v = 0u;
        for (uint8_t k = 0u; k < 4u; k++)
        {
            uint16_t bi = (uint16_t)(runStart + bestOff + (n * 4u) + k);
            uint8_t bit = (uint8_t)(s[bi] & 1u);
            if (bestInv) bit = (uint8_t)(1u - bit);
            v |= (uint8_t)((bit & 1u) << k);
        }
        disp3478_inaNib[seg][n] = (uint8_t)(v & 0xFu);
    }

    /* ---- Histogram (kept for debug) ---- */
    for (uint8_t b = 0u; b < 16u; b++) disp3478_inaHist[seg][b] = 0u;

    for (uint16_t n = 0; n < nibs; n++)
    {
        uint8_t v = (uint8_t)(disp3478_inaNib[seg][n] & 0xFu);
        disp3478_inaHist[seg][v]++;
    }

    /* mode nibble (debug only; we no longer strip by mode) */
    {
        uint8_t mode = 0u, modeCnt = 0u;
        for (uint8_t v = 0u; v < 16u; v++)
        {
            uint8_t c = disp3478_inaHist[seg][v];
            if (c > modeCnt) { modeCnt = c; mode = v; }
        }
        disp3478_inaModeNib[seg] = mode;
    }

    /* ---- Counts + nontrivial list ---- */
    disp3478_inaNontrivCount[seg] = 0u;
    disp3478_inaCountF[seg] = 0u;
    disp3478_inaCount0[seg] = 0u;
    disp3478_inaCountOther[seg] = 0u;

    for (uint16_t n = 0; n < nibs; n++)
    {
        uint8_t v = (uint8_t)(disp3478_inaNib[seg][n] & 0xFu);

        if (v == 0xFu) disp3478_inaCountF[seg]++;
        else if (v == 0x0u) disp3478_inaCount0[seg]++;
        else disp3478_inaCountOther[seg]++;

        if ((v != 0x0u) && (v != 0xFu))
        {
            uint8_t k = disp3478_inaNontrivCount[seg];
            if (k < 32u)
            {
                disp3478_inaNontriv[seg][k] = v;
                disp3478_inaNontrivCount[seg] = (uint8_t)(k + 1u);
            }
        }
    }

    for (uint8_t k = disp3478_inaNontrivCount[seg]; k < 32u; k++)
        disp3478_inaNontriv[seg][k] = 0u;

    /* ---- Strip candidate separator nibble X (instead of “mode”) ----
       Always strip 0 and F, optionally strip one extra nibble from candidates. */
    {
        static const uint8_t cand[6] = { 1u, 8u, 14u, 7u, 6u, 9u };

        uint8_t bestX = 0u;
        uint8_t bestStripScore = 0u;
        uint8_t bestLen = 0u;
        uint8_t bestTmp[32];

        for (uint8_t ci = 0u; ci < 6u; ci++)
        {
            uint8_t X = cand[ci];
            uint8_t tmp[32];
            uint8_t tmpLen = 0u;

            for (uint16_t n = 0; n < nibs; n++)
            {
                uint8_t v = (uint8_t)(disp3478_inaNib[seg][n] & 0xFu);

                if (v == 0x0u) continue;
                if (v == 0xFu) continue;
                if (v == X)    continue;

                if (tmpLen < 32u) tmp[tmpLen++] = v;
            }

            uint8_t score = 0u;

            /* length preference */
            if (tmpLen >= 22u && tmpLen <= 26u) score += 5u;      /* ~24 */
            else if (tmpLen >= 10u && tmpLen <= 14u) score += 3u; /* ~12 */
            else if (tmpLen >= 14u && tmpLen <= 18u) score += 1u; /* ~16 */

            /* diversity penalty */
            if (tmpLen > 0u)
            {
                uint8_t hist[16];
                for (uint8_t i = 0u; i < 16u; i++) hist[i] = 0u;
                for (uint8_t i = 0u; i < tmpLen; i++) hist[tmp[i] & 0xFu]++;

                uint8_t maxBin = 0u;
                for (uint8_t i = 0u; i < 16u; i++) if (hist[i] > maxBin) maxBin = hist[i];

                if ((uint16_t)maxBin * 10u > (uint16_t)tmpLen * 7u)
                {
                    if (score > 0u) score--;
                }
            }

            if (score > bestStripScore)
            {
                bestStripScore = score;
                bestX = X;
                bestLen = tmpLen;
                for (uint8_t i = 0u; i < 32u; i++) bestTmp[i] = (i < tmpLen) ? tmp[i] : 0u;
            }
        }

        disp3478_inaBestStripNib[seg] = bestX;
        disp3478_inaStripScore[seg] = bestStripScore;
        disp3478_inaStripCount[seg] = bestLen;
        for (uint8_t i = 0u; i < 32u; i++) disp3478_inaStripped[seg][i] = bestTmp[i];

        /* ---- NEW: phase-normalize seg4 stripped list ---- */
        if (seg == 4u)
        {
            Normalize3478_Seg4Strip(disp3478_inaStripped[4], disp3478_inaStripCount[4]);
        }
    }
}


static void Disp3478_BuildCap(const uint8_t* s, uint16_t N, uint8_t kind)
{
    if (N > 512u) N = 512u;

    /* header: "3478C" + seq + kind + lenLSB */
    disp3478_cap[0] = '3';
    disp3478_cap[1] = '4';
    disp3478_cap[2] = '7';
    disp3478_cap[3] = '8';
    disp3478_cap[4] = 'C';
    disp3478_cap[5] = disp3478_capSeq++;     /* increments every capture */
    disp3478_cap[6] = kind;
    disp3478_cap[7] = (uint8_t)(N & 0xFFu);  /* (optional; header is mainly for sanity) */

    /* payload: always overwrite 512 so no stale tail */
    for (uint16_t i = 0u; i < 512u; i++)
        disp3478_cap[8u + i] = (i < N) ? s[i] : 0u;

    /* report the full blob size (header + payload) */
    disp3478_capLen = (uint16_t)(8u + 512u);
}




void Decode3478_LatchedFrame(void)
{
    if (frameReady == 0u)
        return;

    frameReady = 0u;

    const uint8_t* s = frameO2Latched;
    uint16_t N = frameO2LatchedLen;
    if (N > 512u) N = 512u;

    /* Always rebuild capture blob on every latched frame */
#ifdef frameKindLatched
    Disp3478_BuildCap(s, N, (uint8_t)frameKindLatched);
#else
    Disp3478_BuildCap(s, N, 0u);
#endif


    disp3478_frameN = N;

    /* ---------------- Clear outputs ---------------- */
    disp3478_isa10_len = 0;
    for (uint8_t i = 0u; i < 16u; i++) disp3478_isa10_list[i] = 0;

    disp3478_segCount = 0;
    disp3478_segSum = 0;

    for (uint8_t i = 0u; i < 8u; i++)
    {
        disp3478_segLen[i] = 0;
        disp3478_segSync[i] = 0;

        disp3478_inaNibCount[i] = 0;
        for (uint8_t j = 0u; j < 32u; j++) disp3478_inaNib[i][j] = 0;

        disp3478_inaBestInv[i] = 0;
        disp3478_inaBestOff[i] = 0;
        disp3478_inaBestScore[i] = 0;

        disp3478_inaNontrivCount[i] = 0;
        disp3478_inaCountF[i] = 0;
        disp3478_inaCount0[i] = 0;
        disp3478_inaCountOther[i] = 0;
        for (uint8_t j = 0u; j < 32u; j++) disp3478_inaNontriv[i][j] = 0;

        for (uint8_t b = 0u; b < 16u; b++) disp3478_inaHist[i][b] = 0;
        disp3478_inaModeNib[i] = 0;

        disp3478_inaStripCount[i] = 0;
        for (uint8_t j = 0u; j < 32u; j++) disp3478_inaStripped[i][j] = 0;

        disp3478_inaBestStripNib[i] = 0;
        disp3478_inaStripScore[i] = 0;
    }

    /* static-vs-change diff outputs (existing) */
    disp3478_diffSeg2_first = 0xFFu;
    disp3478_diffSeg2_count = 0u;
    for (uint8_t i = 0u; i < 32u; i++) disp3478_diffSeg2_xor[i] = 0u;

    disp3478_diffSeg4_first = 0xFFu;
    disp3478_diffSeg4_count = 0u;
    for (uint8_t i = 0u; i < 32u; i++) disp3478_diffSeg4_xor[i] = 0u;

    /* change-vs-change diff outputs for seg4 */
    disp3478_diffChgSeg4_first = 0xFFu;
    disp3478_diffChgSeg4_count = 0u;
    for (uint8_t i = 0u; i < 32u; i++) disp3478_diffChgSeg4_xor[i] = 0u;

    /* snapshot of XOR inputs (seg4) */
    for (uint8_t i = 0u; i < 32u; i++) {
        disp3478_dbgDiffPrev4[i] = 0u;
        disp3478_dbgDiffCurr4[i] = 0u;
    }

    disp3478_sync1_count = 0;
    for (uint8_t i = 0u; i < 64u; i++) disp3478_tagPreview[i] = 0;

    uint16_t pv = (N > 64u) ? 64u : N;
    for (uint16_t i = 0u; i < pv; i++)
        disp3478_tagPreview[i] = s[i];

    /* ---------------- ISA bits (SYNC==1), ISA active-low ---------------- */
    uint8_t isaBits[128];
    uint16_t isaBitsLen = 0;

    for (uint16_t i = 0u; i < N; i++)
    {
        uint8_t sync = (uint8_t)((s[i] >> 1) & 1u);
        if (sync) disp3478_sync1_count++;

        if (sync)
        {
            uint8_t bit = (uint8_t)(s[i] & 1u);
            bit = (uint8_t)(1u - (bit & 1u)); /* invert ISA */
            if (isaBitsLen < (uint16_t)sizeof(isaBits))
                isaBits[isaBitsLen++] = bit;
        }
    }

    disp3478_isa_bits_len = (uint8_t)((isaBitsLen > 255u) ? 255u : isaBitsLen);

    /* ---------------- ISA10 decode (LSB-first, stride 10) ---------------- */
    uint16_t idx = 0u;
    while ((uint16_t)(idx + 9u) < isaBitsLen)
    {
        uint16_t w = 0u;
        for (uint8_t k = 0u; k < 10u; k++)
            w |= (uint16_t)((isaBits[idx + k] & 1u) << k);

        if (disp3478_isa10_len < 16u)
            disp3478_isa10_list[disp3478_isa10_len++] = w;

        idx = (uint16_t)(idx + 10u);
    }

    /* ---------------- Segment by SYNC level, decode INA on SYNC==0 ---------------- */
    if (N == 0u)
        return;

    uint8_t curSync = (uint8_t)((s[0] >> 1) & 1u);
    uint16_t runStart = 0u;
    uint8_t seg = 0u;

    for (uint16_t i = 1u; i < N; i++)
    {
        uint8_t syncNow = (uint8_t)((s[i] >> 1) & 1u);
        if (syncNow != curSync)
        {
            if (seg < 8u)
            {
                Process3478Run(s, runStart, i, curSync, seg);
                seg++;
            }
            curSync = syncNow;
            runStart = i;
        }
    }

    /* final run */
    if (seg < 8u)
    {
        Process3478Run(s, runStart, N, curSync, seg);
        seg++;
    }

    /* ---------------- Save STATIC or CHANGE copies ---------------- */
    if (frameKindLatched == 2u)
    {
        disp3478_haveChange = 1u;

        for (uint8_t sg = 0u; sg < 8u; sg++)
        {
            disp3478_changeStripCount[sg] = disp3478_inaStripCount[sg];
            for (uint8_t k = 0u; k < 32u; k++)
                disp3478_changeStripped[sg][k] = disp3478_inaStripped[sg][k];
        }

        /* ---- CHANGE-vs-CHANGE diff for seg4 (with input snapshots) ---- */
        {
            uint8_t n = disp3478_changeStripCount[4];

            if (disp3478_havePrevChange == 1u)
            {
                uint8_t m = disp3478_prevChangeStripCount4;
                if (m < n) n = m;

                if (n > 0u)
                {
                    uint8_t first = 0xFFu;
                    uint8_t count = 0u;

                    for (uint8_t i = 0u; i < n; i++)
                    {
                        uint8_t a = disp3478_prevChangeStripped4[i];
                        uint8_t b = disp3478_changeStripped[4][i];

                        disp3478_dbgDiffPrev4[i] = a;
                        disp3478_dbgDiffCurr4[i] = b;

                        uint8_t x = (uint8_t)(a ^ b);
                        disp3478_diffChgSeg4_xor[i] = x;

                        if (x != 0u)
                        {
                            if (first == 0xFFu) first = i;
                            count++;
                        }
                    }

                    for (uint8_t i = n; i < 32u; i++) {
                        disp3478_dbgDiffPrev4[i] = 0u;
                        disp3478_dbgDiffCurr4[i] = 0u;
                        disp3478_diffChgSeg4_xor[i] = 0u;
                    }

                    disp3478_diffChgSeg4_first = first;
                    disp3478_diffChgSeg4_count = count;
                }
            }

            disp3478_havePrevChange = 1u;
            disp3478_prevChangeStripCount4 = disp3478_changeStripCount[4];
            for (uint8_t i = 0u; i < 32u; i++)
                disp3478_prevChangeStripped4[i] = disp3478_changeStripped[4][i];
        }
    }
    else
    {
        disp3478_haveStatic = 1u;

        for (uint8_t sg = 0u; sg < 8u; sg++)
        {
            disp3478_staticStripCount[sg] = disp3478_inaStripCount[sg];
            for (uint8_t k = 0u; k < 32u; k++)
                disp3478_staticStripped[sg][k] = disp3478_inaStripped[sg][k];
        }
    }

    /* hard clear unused entries */
    for (uint8_t i = seg; i < 8u; i++)
    {
        disp3478_segSync[i] = 0;
        disp3478_segLen[i] = 0;

        disp3478_inaNibCount[i] = 0;
        disp3478_inaBestInv[i] = 0;
        disp3478_inaBestOff[i] = 0;
        disp3478_inaBestScore[i] = 0;

        disp3478_inaNontrivCount[i] = 0;
        disp3478_inaCountF[i] = 0;
        disp3478_inaCount0[i] = 0;
        disp3478_inaCountOther[i] = 0;

        disp3478_inaModeNib[i] = 0;

        disp3478_inaStripCount[i] = 0;
        disp3478_inaBestStripNib[i] = 0;
        disp3478_inaStripScore[i] = 0;

        for (uint8_t b = 0u; b < 16u; b++) disp3478_inaHist[i][b] = 0;

        for (uint8_t j = 0u; j < 32u; j++) {
            disp3478_inaNib[i][j] = 0;
            disp3478_inaNontriv[i][j] = 0;
            disp3478_inaStripped[i][j] = 0;
        }
    }

    disp3478_segCount = seg;

    /* Flatten first 5 segments for Live Watch sanity */
    disp3478_segSyncFlat0 = (disp3478_segCount > 0u) ? disp3478_segSync[0] : 0;
    disp3478_segSyncFlat1 = (disp3478_segCount > 1u) ? disp3478_segSync[1] : 0;
    disp3478_segSyncFlat2 = (disp3478_segCount > 2u) ? disp3478_segSync[2] : 0;
    disp3478_segSyncFlat3 = (disp3478_segCount > 3u) ? disp3478_segSync[3] : 0;
    disp3478_segSyncFlat4 = (disp3478_segCount > 4u) ? disp3478_segSync[4] : 0;

    disp3478_segLenFlat0 = (disp3478_segCount > 0u) ? disp3478_segLen[0] : 0;
    disp3478_segLenFlat1 = (disp3478_segCount > 1u) ? disp3478_segLen[1] : 0;
    disp3478_segLenFlat2 = (disp3478_segCount > 2u) ? disp3478_segLen[2] : 0;
    disp3478_segLenFlat3 = (disp3478_segCount > 3u) ? disp3478_segLen[3] : 0;
    disp3478_segLenFlat4 = (disp3478_segCount > 4u) ? disp3478_segLen[4] : 0;

    /* NEW: build LiveWatch text string */
    Build3478Text();

    Build3478Cap(s, N);
}



static uint16_t CapPut8(uint8_t* dst, uint16_t pos, uint16_t max, uint8_t v)
{
    if (pos < max) dst[pos] = v;
    return (uint16_t)(pos + 1u);
}

static uint16_t CapPut16LE(uint8_t* dst, uint16_t pos, uint16_t max, uint16_t v)
{
    pos = CapPut8(dst, pos, max, (uint8_t)(v & 0xFFu));
    pos = CapPut8(dst, pos, max, (uint8_t)((v >> 8) & 0xFFu));
    return pos;
}

/* Build single-variable capture blob for Live Watch */
void Build3478Cap(const uint8_t* s, uint16_t N)
{
    disp3478_capLen = 0u;

    /* header (12 bytes) */
    disp3478_cap[0] = '3';
    disp3478_cap[1] = '4';
    disp3478_cap[2] = '7';
    disp3478_cap[3] = '8';
    disp3478_cap[4] = 'C';
    disp3478_cap[5] = disp3478_capSeq++;      /* SEQ (increments every capture) */
    disp3478_cap[6] = frameKindLatched;       /* kind */
    disp3478_cap[7] = 0u;                     /* reserved */
    disp3478_cap[8] = (uint8_t)(N & 0xFFu);   /* N LSB */
    disp3478_cap[9] = (uint8_t)(N >> 8);      /* N MSB */
    disp3478_cap[10] = 0u;                     /* reserved */
    disp3478_cap[11] = 0u;                     /* reserved */

    uint16_t take = N;
    if (take > DISP3478_CAP_DATA_MAX) take = DISP3478_CAP_DATA_MAX;

    for (uint16_t i = 0u; i < take; i++)
        disp3478_cap[DISP3478_CAP_HDR + i] = s[i];

    for (uint16_t i = take; i < DISP3478_CAP_DATA_MAX; i++)
        disp3478_cap[DISP3478_CAP_HDR + i] = 0u;

    disp3478_capLen = (uint16_t)(DISP3478_CAP_HDR + take);
}







static uint16_t AppendHexNib(char* dst, uint16_t pos, uint16_t max, uint8_t v)
{
    static const char hx[16] = "0123456789abcdef";
    if (pos + 2u >= max) return pos;
    dst[pos++] = hx[(v >> 4) & 0x0Fu];
    dst[pos++] = hx[v & 0x0Fu];
    return pos;
}

static uint16_t AppendStr(char* dst, uint16_t pos, uint16_t max, const char* s)
{
    while (*s != 0)
    {
        if (pos + 1u >= max) break;
        dst[pos++] = *s++;
    }
    return pos;
}

static uint16_t AppendNibList(char* dst, uint16_t pos, uint16_t max,
    const uint8_t* arr, uint8_t count)
{
    for (uint8_t i = 0u; i < count; i++)
    {
        if (pos + 2u >= max) break;
        /* nibble as single hex char */
        static const char hx[16] = "0123456789abcdef";
        dst[pos++] = hx[arr[i] & 0x0Fu];
        if (pos + 1u < max) dst[pos++] = ' ';
    }
    return pos;
}

void Build3478Text(void)
{
    /* Build a readable debug string from already-decoded results */
    uint16_t pos = 0u;
    uint16_t max = (uint16_t)sizeof(disp3478_text);

    /* prefix: frame kind */
    pos = AppendStr(disp3478_text, pos, max, "K=");
    pos = AppendHexNib(disp3478_text, pos, max, (uint8_t)frameKindLatched);

    pos = AppendStr(disp3478_text, pos, max, " N=");
    pos = AppendHexNib(disp3478_text, pos, max, (uint8_t)((disp3478_frameN >> 8) & 0xFFu));
    pos = AppendHexNib(disp3478_text, pos, max, (uint8_t)((disp3478_frameN >> 0) & 0xFFu));

    pos = AppendStr(disp3478_text, pos, max, "  ");

    /* ISA10 words (first few) */
    pos = AppendStr(disp3478_text, pos, max, "ISA10:");
    for (uint8_t i = 0u; i < disp3478_isa10_len; i++)
    {
        uint16_t w = disp3478_isa10_list[i];
        if (pos + 6u >= max) break;
        disp3478_text[pos++] = ' ';
        disp3478_text[pos++] = "0123456789abcdef"[(w >> 12) & 0x0Fu];
        disp3478_text[pos++] = "0123456789abcdef"[(w >> 8) & 0x0Fu];
        disp3478_text[pos++] = "0123456789abcdef"[(w >> 4) & 0x0Fu];
        disp3478_text[pos++] = "0123456789abcdef"[(w >> 0) & 0x0Fu];
    }
    pos = AppendStr(disp3478_text, pos, max, "  ");

    /* Dump stripped nibble streams for seg2 and seg4 (these are your most useful right now) */
    pos = AppendStr(disp3478_text, pos, max, "S2(");
    pos = AppendHexNib(disp3478_text, pos, max, disp3478_inaStripCount[2]);
    pos = AppendStr(disp3478_text, pos, max, "): ");
    pos = AppendNibList(disp3478_text, pos, max, disp3478_inaStripped[2], disp3478_inaStripCount[2]);
    pos = AppendStr(disp3478_text, pos, max, " | ");

    pos = AppendStr(disp3478_text, pos, max, "S4(");
    pos = AppendHexNib(disp3478_text, pos, max, disp3478_inaStripCount[4]);
    pos = AppendStr(disp3478_text, pos, max, "): ");
    pos = AppendNibList(disp3478_text, pos, max, disp3478_inaStripped[4], disp3478_inaStripCount[4]);

    /* terminate */
    if (pos >= max) pos = (uint16_t)(max - 1u);
    disp3478_text[pos] = 0;
    disp3478_textLen = (uint8_t)((pos > 255u) ? 255u : pos);
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