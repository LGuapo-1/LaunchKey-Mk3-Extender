#include <SPI.h>
#include <EEPROM.h>
#include <U8x8lib.h>


//**************************************************************************************************
// Macros
//**************************************************************************************************
#define USE_SERIAL_DEBUG 0
#define USE_TIMING_DEBUG 0
#define USE_64_BIT_TIME  0

#define CPU_MHZ 16

#define PIN_DISPLAY_CLOCK 13
#define PIN_DISPLAY_MOSI  11
#define PIN_DISPLAY_CS    10
#define PIN_DISPLAY_DC    8
#define PIN_DISPLAY_RESET 7
#define PIN_ENCODER_CLOCK 6
#define PIN_ENCODER_DATA  5
#define PIN_BUTTON        4
#define PIN_ADC           0
#define PIN_INT0          2
#define PIN_INT1          3

#define ADC_CHANNEL_MW   0
#define ADC_CHANNEL_PED  1
#define ADC_REF_5V       (1 << REFS0)
#define ADC_REF_EXTERNAL 0
#define ADC_START        (1 << ADSC)
#define ADC_NO_VALUE     0xFFFF

// The compiler for AVR architecture defaults to 16-bit size for integer literals, so force
// up-cast to 32-bit (or 64-bit) for Millis

#define ENCODER_PROCESS_WAIT_MILLIS ((Millis)2)

#define BUTTON_PROCESS_MILLIS      ((Millis)2)
#define BUTTON_STABLE_LEVEL_MILLIS ((Millis)10)
#define BUTTON_HOLD_MILLIS         ((Millis)2000)
#define BUTTON_DBL_CLICK_MILLIS    ((Millis)150)

// #define PRIORITY_CONTEXT0_PERIOD_USEC 250
#define PRIORITY_CONTEXT0_PERIOD_USEC 100
#define PRIORITY_CONTEXT0_INT_VECT    TIMER1_OVF_vect

// The value (PRIORITY_CONTEXT0_PERIOD_USEC * CPU_MHZ / 2) must fit in a 16-bit register
static_assert(((uint64_t)(PRIORITY_CONTEXT0_PERIOD_USEC * CPU_MHZ) >> 1) <= 0xFFFFULL, "");

#define SCALE_N_256THS_WITH_ROUNDING(n,v) ((((uint16_t)(v) * (uint16_t)(n)) + (1 << 7)) >> 8)

#define MIDI_STATUS_IS_CC(b)      (((b) & 0xF0) == 0xB0)
#define MIDI_STATUS_IS_NOTE_ON(b) (((b) & 0xF0) == 0x90)
#define MIDI_IS_STATUS(b, p)      ((((b) & 0x80) && ((p) != 0xF0)) || ((b) == 0xF7))
#define MIDI_STATUS_CHANNEL(b)    ((b) & 0x0F)

#define UI_LINE_OFFSET    1
#define UI_TIMEOUT_SECS   10
#define UI_TIMEOUT_MILLIS ((Millis)1000 * 60 * UI_TIMEOUT_SECS)

#define UI_LOOP_SLEEP_MILLIS ((Millis)30)

#define PRESET_SLOT_COUNT             16
#define PRESET_BLOCK_BYTE_COUNT       32
#define PRESET_INDEX_WEAR_LEVEL_COUNT 10


//**************************************************************************************************
// Types
//**************************************************************************************************
#if USE_64_BIT_TIME
typedef uint64_t Millis;
#else
typedef uint32_t Millis;
#endif

typedef uint32_t Micros;

struct UIParamPage
{
    volatile uint8_t* valuePtr;
    uint8_t           valueMin;
    uint8_t           valueMax;
    uint8_t           valueDisplayOffs;
    bool              isReadWrite;
    bool              isPressAndTurn;
    bool              isCalibration;
    const char* const label;
};


//**************************************************************************************************
// Globals
//**************************************************************************************************
#if USE_64_BIT_TIME
volatile uint32_t gTimeRolloverCount = 0;
volatile uint32_t gTimePrevNow       = 0;
#endif

uint8_t gADCCounter = 0;

volatile uint8_t gADCMWValue            = 0;
uint8_t          gADCMWPrevValue        = 255;
uint16_t         gADCMWFastSum[2]       = { 0, 0 };
uint8_t          gADCMWFastSumIndex     = 0;
uint8_t          gADCMWFastSumCount     = 0;
uint8_t          gADCMWFastBuffer[3]    = { 0, 0, 0 };
const uint8_t*   gADCMWFastBufferPtrMin = &(gADCMWFastBuffer[0]);
const uint8_t*   gADCMWFastBufferPtrMax = &(gADCMWFastBuffer[2]);
uint8_t*         gADCMWFastBufferPtr    = gADCMWFastBufferPtrMin;
uint16_t         gADCMWSlowSum[2]       = { 0, 0 };
uint8_t          gADCMWSlowSumIndex     = 0;
uint8_t          gADCMWSlowSumCount     = 0;

volatile uint8_t gADCPedValue     = 0;
uint8_t          gADCPedPrevValue = 255;
uint16_t         gADCPedSum[2]    = { 0, 0 };
uint8_t          gADCPedSumIndex  = 0;
uint8_t          gADCPedSumCount  = 0;

volatile Millis gEncPrevClockChangeMillis  = 0;
volatile int8_t gEncValue                  = 0;
uint8_t         gEncPrevClockLevel         = HIGH;
uint8_t         gEncClockLevelBuffer[2]    = { HIGH, HIGH };
uint8_t         gEncClockLevelBufferIndex  = 0;
uint8_t         gEncDataLevelBuffer[2]     = { HIGH, HIGH };
uint8_t         gEncDataLevelBufferIndex   = 0;

Millis           gButtonPrevProcessMillis = 0;
Millis           gButtonPrevChangeMillis  = 0;
Millis           gButtonStableLowMillis   = 0;
Millis           gButtonStableHighMillis  = 0;
bool             gButtonDidHandleHold     = false;
bool             gButtonIsClickPending    = false;
uint8_t          gButtonPrevLevel         = HIGH;
uint8_t          gButtonPrevStableLevel   = HIGH;
volatile bool    gButtonIsDown            = false;
volatile uint8_t gButtonClickCount        = 0;
volatile uint8_t gButtonDblClickCount     = 0;
volatile bool    gButtonDidHold           = false;
volatile bool    gButtonIsForceHold       = false;

volatile uint8_t gMIDIShouldReplaceModWheel = 1;
volatile uint8_t gMIDIShouldProcessVelocity = 1;
volatile uint8_t gMIDIShouldSendPedal       = 1;

uint8_t gMIDISendPrevStatus = 0x00;

uint8_t gMIDIInputPrevStatus        = 0x00;
uint8_t gMIDIInputNoteValue         = 0;
uint8_t gMIDIInputDataByteCount     = 0;
uint8_t gMIDIInputCurrDataByteCount = 0;
bool    gMIDIInputIsCCFiltering     = false;
bool    gMIDIInputDidDropStatus     = false;
bool    gMIDIInputIsRunningStatus   = false;
bool    gMIDIInputIsMessageComplete = true;

volatile uint8_t gMIDIVelocityProcessChan          = 0;
volatile uint8_t gMIDIVelocityLowThresh            = 0;
volatile uint8_t gMIDIVelocityShouldBlackKeyAdjust = 1;

// Velocity compression ratio can be greater than or equal to 1:1, less than or equal to 10:1, and
// increments by 0.25. In practice, the ratio reciprocal is what is actually used (the multiplier).
// Specifically, Vout = MIN(Vin, threshold) + [MAX(Vin - threshold, 0) * multiplier]. The multiplier
// operation is calculated using integer radix-2 division, so the multiplier is represented as a
// fraction where the denominator is always 512. We use 512 since it allows for the maximum
// precision with 16-bit operations: 127 * 512 = 65,024, 2^16 = 65,536 (even adding 256 for rounding
// fits). So a ratio of 1:1 would have a multiplier numerator of 512. A ratio of 3.25:1 would have a
// multipler numerator of 158.
//
// The ratio index is a value from 0 to 36 (inclusive) that represents the ratio itself for UI
// purposes only. Specifically, index = (4 * ratio) - 4. So the index of a ratio of 1:1 is 0. The
// index of a ratio of 3.25:1 is 9.
//
// The ratio multiplier numerator can be calculated from the index by multN = 2048 / (index + 4)
//
// The compression makeup is the value that must be added to a compressed velocity with input L to
// reach L. Specifically, makeup = L - [threshold + ([(L - threshold) * multN] / 512)]
// 
volatile uint8_t  gMIDIVelocityCompThresh     = 0;
volatile uint16_t gMIDIVelocityCompRatioMultN = 512;
volatile uint8_t  gMIDIVelocityCompMakeup     = 0;
uint8_t           gMIDIVelocityCompRaioIndex  = 0;
uint8_t           gMIDIVelocityCompLimit      = 0;
volatile uint8_t  gMIDIVelocityMonitorIn      = 0;
volatile uint8_t  gMIDIVelocityMonitorOut     = 0;

bool             gMIDIOutputWasLastSend  = false;
volatile uint8_t gMIDIOutputModWheelChan = 0x00;
volatile uint8_t gMIDIOutputModWheelCC   = 0x01; 
volatile uint8_t gMIDIOutputPedalChan    = 0x00;
volatile uint8_t gMIDIOutputPedalCC      = 0x0B; 

bool    gIsMIDINoteBlackKeyLUT[128];
uint8_t gMIDIVelocityBlackKeyAdjustLUT[128];

bool             gIsCalibrationActive  = false;
volatile uint8_t gCalibrationScaleMW   = 61;
volatile uint8_t gCalibrationScalePed  = 36;
volatile uint8_t gCalibrationOffsetMW  = 54;
volatile uint8_t gCalibrationOffsetPed = 16;

uint8_t gStoreState      = 0;
uint8_t gLoadState       = 1;
uint8_t gPrevPresetIndex = 0;
uint8_t gPresetIndex     = 0;

U8X8_SH1106_128X64_NONAME_4W_HW_SPI gDisplay(PIN_DISPLAY_CS,
                                             PIN_DISPLAY_DC,
                                             PIN_DISPLAY_RESET);

uint8_t           gUIParamPageIndex = 0;
constexpr uint8_t gUIParamPageCount = 22;

UIParamPage gUIParamPages[gUIParamPageCount] = {
  { &gLoadState,                        0, 1,   0, true,  false, false, "Hold to load"    },
  { &gMIDIShouldReplaceModWheel,        0, 1,   0, true,  false, false, "Mod Wheel"       },
  { &gMIDIOutputModWheelCC,             0, 127, 0, true,  true,  false, "Mod Wheel CC"    },
  { &gMIDIOutputModWheelChan,           0, 15,  1, true,  false, false, "Mod Wheel Chan"  },
  { &gADCMWValue,                       0, 127, 0, false, false, false, "Mod Wheel Mon"   },
  { &gCalibrationScaleMW,               0, 200, 0, true,  true,  true,  "Cal MW Scale"    },
  { &gCalibrationOffsetMW,              0, 200, 0, true,  true,  true,  "Cal MW Offset"   },
  { &gMIDIShouldSendPedal,              0, 1,   0, true,  false, false, "Exp Pedal"       },
  { &gMIDIOutputPedalCC,                0, 127, 0, true,  true,  false, "Exp Pedal CC"    },
  { &gMIDIOutputPedalChan,              0, 15,  1, true,  false, false, "Exp Pedal Chan"  },
  { &gADCPedValue,                      0, 127, 0, false, false, false, "Exp Pedal Mon"   },
  { &gCalibrationScalePed,              0, 200, 0, true,  true,  true,  "Cal Ped Scale"   },
  { &gCalibrationOffsetPed,             0, 200, 0, true,  true,  true,  "Cal Ped Offset"  },
  { &gMIDIShouldProcessVelocity,        0, 1,   0, true,  false, false, "Velocity"        },
  { &gMIDIVelocityProcessChan,          0, 15,  1, true,  false, false, "Velocity Chan"   },
  { &gMIDIVelocityLowThresh,            1, 127, 0, true,  true,  false, "Velocity Min"    },
  { &gMIDIVelocityCompThresh,           1, 127, 0, true,  true,  false, "Vel Comp Thrsh"  },
  { &gMIDIVelocityCompRaioIndex,        0, 36,  0, true,  false, false, "Vel Comp Ratio"  },
  { &gMIDIVelocityCompLimit,            1, 127, 0, true,  true,  false, "Vel Comp Limit"  },
  { &gMIDIVelocityShouldBlackKeyAdjust, 0, 1,   0, true,  false, false, "Vel Blk Key Adj" },
  { &gMIDIVelocityMonitorOut,           0, 127, 0, false, false, false, "Velocity Mon"    },
  { &gStoreState,                       0, 1,   0, true,  false, false, "Hold to save"    },
};

uint8_t         gUIParamPrevDisplayValues[2] = { 0, 0 };
volatile Millis gUITimeoutTimestamp          = 0;
bool            gUIIsTimedOut                = false;

char gStrBuffer[32];

#if USE_TIMING_DEBUG
volatile uint16_t gDebugMicros1 = 0;
volatile uint16_t gDebugMicros2 = 0;
#endif


//**************************************************************************************************
// Functions
//**************************************************************************************************
//**************************************************************************************************
// Get the current time in milliseconds
//**************************************************************************************************
static inline Millis NowMillis(void)
{
#if USE_64_BIT_TIME
    const uint32_t now = millis();

    if (now < gTimePrevNow) {
        gTimeRolloverCount++;
    }

    gTimePrevNow = now;

    return (((uint64_t)gTimeRolloverCount) << 32) | gTimePrevNow;
#else
    return millis();
#endif
}

//**************************************************************************************************
// Return an elapsed time in milliseconds
//**************************************************************************************************
static inline Millis ElapsedMillis(const Millis startMillis, const Millis endMillis)
{
#if USE_64_BIT_TIME
    return endMillis - startMillis;
#else
    if (endMillis >= startMillis) {
        return endMillis - startMillis;
    } else {
        return ((uint32_t)0xFFFFFFFF - startMillis) + endMillis + 1;
    }
#endif
}

//**************************************************************************************************
// Return an elapsed time in microseconds
//**************************************************************************************************
static inline uint32_t ElapsedMicros(const Micros startMicros, const Micros endMicros)
{
    if (endMicros >= startMicros) {
        return endMicros - startMicros;
    } else {
        return ((uint32_t)0xFFFFFFFF - startMicros) + endMicros + 1;
    }
}

//**************************************************************************************************
// Create the table that maps MIDI note number to is-black-key boolean
//**************************************************************************************************
static void CreateIsMIDINoteBlackKeyLUT(void)
{
    // Modulo arithmetic is expensive on 8-bit AVR, but loads only take 2 cycles

    for (uint8_t i = 0; i < 120; i += 12) {
        gIsMIDINoteBlackKeyLUT[i + 0]  = false;
        gIsMIDINoteBlackKeyLUT[i + 1]  = true;
        gIsMIDINoteBlackKeyLUT[i + 2]  = false;
        gIsMIDINoteBlackKeyLUT[i + 3]  = true;
        gIsMIDINoteBlackKeyLUT[i + 4]  = false;
        gIsMIDINoteBlackKeyLUT[i + 5]  = false;
        gIsMIDINoteBlackKeyLUT[i + 6]  = true;
        gIsMIDINoteBlackKeyLUT[i + 7]  = false;
        gIsMIDINoteBlackKeyLUT[i + 8]  = true;
        gIsMIDINoteBlackKeyLUT[i + 9]  = false;
        gIsMIDINoteBlackKeyLUT[i + 10] = true;
        gIsMIDINoteBlackKeyLUT[i + 11] = false;
    }
    gIsMIDINoteBlackKeyLUT[120]  = false;
    gIsMIDINoteBlackKeyLUT[121]  = true;
    gIsMIDINoteBlackKeyLUT[122]  = false;
    gIsMIDINoteBlackKeyLUT[123]  = true;
    gIsMIDINoteBlackKeyLUT[124]  = false;
    gIsMIDINoteBlackKeyLUT[125]  = false;
    gIsMIDINoteBlackKeyLUT[126]  = true;
    gIsMIDINoteBlackKeyLUT[127]  = false;
}

//**************************************************************************************************
// Create the table that maps black key velocity to corrected velocity
//**************************************************************************************************
static void CreateMIDIVelocityBlackKeyAdjustLUT(void)
{
    // This table is definitely much bigger than it needs to be, but we have the memory so might as
    // well use it
    for (uint8_t i = 0; i < 8; i++) {
        gMIDIVelocityBlackKeyAdjustLUT[i] = i;
    }

    gMIDIVelocityBlackKeyAdjustLUT[8]  = 7;
    gMIDIVelocityBlackKeyAdjustLUT[9]  = 8;
    gMIDIVelocityBlackKeyAdjustLUT[10] = 8;
    gMIDIVelocityBlackKeyAdjustLUT[11] = 9;
    gMIDIVelocityBlackKeyAdjustLUT[12] = 9;
    gMIDIVelocityBlackKeyAdjustLUT[13] = 10;
    gMIDIVelocityBlackKeyAdjustLUT[14] = 10;
    gMIDIVelocityBlackKeyAdjustLUT[15] = 11;
    gMIDIVelocityBlackKeyAdjustLUT[16] = 11;
    gMIDIVelocityBlackKeyAdjustLUT[17] = 11;
    gMIDIVelocityBlackKeyAdjustLUT[18] = 12;
    gMIDIVelocityBlackKeyAdjustLUT[19] = 12;
    gMIDIVelocityBlackKeyAdjustLUT[20] = 13;
    gMIDIVelocityBlackKeyAdjustLUT[21] = 13;
    gMIDIVelocityBlackKeyAdjustLUT[22] = 14;
    gMIDIVelocityBlackKeyAdjustLUT[23] = 14;
    gMIDIVelocityBlackKeyAdjustLUT[24] = 15;

    for (uint8_t i = 25; i < 80; i++) {
        gMIDIVelocityBlackKeyAdjustLUT[i] = i - 10;
    }

    gMIDIVelocityBlackKeyAdjustLUT[80] = 71;
    gMIDIVelocityBlackKeyAdjustLUT[81] = 72;
    gMIDIVelocityBlackKeyAdjustLUT[82] = 74;
    gMIDIVelocityBlackKeyAdjustLUT[83] = 75;
    gMIDIVelocityBlackKeyAdjustLUT[84] = 76;
    gMIDIVelocityBlackKeyAdjustLUT[85] = 78;
    gMIDIVelocityBlackKeyAdjustLUT[86] = 79;
    gMIDIVelocityBlackKeyAdjustLUT[87] = 81;
    gMIDIVelocityBlackKeyAdjustLUT[88] = 83;
    gMIDIVelocityBlackKeyAdjustLUT[89] = 85;
    gMIDIVelocityBlackKeyAdjustLUT[90] = 87;
    gMIDIVelocityBlackKeyAdjustLUT[91] = 88;
    gMIDIVelocityBlackKeyAdjustLUT[92] = 90;
    gMIDIVelocityBlackKeyAdjustLUT[93] = 92;

    for (uint8_t i = 94; i < 128; i++) {
        gMIDIVelocityBlackKeyAdjustLUT[i] = i;
    }
}

//**************************************************************************************************
// Get the number of data bytes for a MIDI message, or return 255 to indicate no fixed length
//**************************************************************************************************
static inline uint8_t MIDIGetDataByteCount(const uint8_t status)
{
    const uint8_t nibbleHigh = (status & 0xF0) >> 4;

    switch (nibbleHigh) {
        case 0x8:
        case 0x9:
        case 0xA:
        case 0xB:
        case 0xE:
            return 2;
            break;

        case 0xC:
        case 0xD:
            return 1;
            break;

        default:
            break;
    }

    if (status == 0xF2) {
        return 2;
    } else if (status == 0xF3) {
        return 1;
    } else if (status >= 0xF7) {
        return 0;
    }

    return 255;
}

//**************************************************************************************************
// Send MIDI (and handle running status)
//**************************************************************************************************
static inline void MIDISend(const uint8_t midiByte)
{
    const bool isStatus = MIDI_IS_STATUS(midiByte, gMIDISendPrevStatus);

    // No need to send repeated status 0x8n, 0x9n, 0xAn, 0xBn, 0xCn, 0xDn, or 0xEn
    if (isStatus) {
        if ((midiByte >= 0xF0) || (midiByte != gMIDISendPrevStatus)) {
            Serial.write(midiByte);
            gMIDISendPrevStatus = midiByte;
        }
    } else {
        Serial.write(midiByte);
    }
}

//**************************************************************************************************
// Send MIDI (and handle running status)
//**************************************************************************************************
static inline void MIDISend(const uint8_t* const midiBytes, const uint8_t count)
{
    for (uint8_t i = 0; i < count; i++) {
        MIDISend(midiBytes[i]);
    }
}

//**************************************************************************************************
// Start a new ADC
//**************************************************************************************************
static inline void ADCStart(const uint8_t channel)
{
    analogReference(EXTERNAL);
    noInterrupts();
    ADMUX = ADC_REF_5V | (channel & 0x07);
    ADCSRA |= ADC_START;
    interrupts();
}

//**************************************************************************************************
// Return the current ADC channel
//**************************************************************************************************
static inline uint8_t ADCGetChannel(void)
{
    return ADMUX & 0x07;
}

//**************************************************************************************************
// Read and return the latest ADC value or return 0xFFFF if no value is available
//**************************************************************************************************
static inline uint16_t ADCGetValue(void)
{
    uint16_t ret = ADC_NO_VALUE;

    noInterrupts();
    if (!(ADCSRA & ADC_START)) {
        const uint16_t lsb = ADCL;
        const uint16_t msb = ADCH;

        ret = (msb << 8) | lsb;
    }
    interrupts();

    return ret;
}

//**************************************************************************************************
// Scale the 10-bit ADC value from the mod wheel to a 7-bit MIDI value
//**************************************************************************************************
static inline uint8_t ADCMWScaleToMIDI(const uint16_t value)
{
    return SCALE_N_256THS_WITH_ROUNDING(gCalibrationScaleMW, value);
}

//**************************************************************************************************
// Scale the 10-bit ADC value from the expression pedal to a 7-bit MIDI value
//**************************************************************************************************
static inline uint8_t ADCPedScaleToMIDI(const uint16_t value)
{
    return SCALE_N_256THS_WITH_ROUNDING(gCalibrationScalePed, value);
}

//**************************************************************************************************
// Setup priority context 0 (uses Timer1 overflow interrupt)
//**************************************************************************************************
static inline void PriorityContext0Init(void)
{
    noInterrupts();
    
    // Mask off the Timer1 overflow interrupt
    TIMSK1 = 0x00;

    // Set timer 1 for "PWM, Phase and Frequency Correct" mode (WGM[3:0] = 0b1000) with all output
    // compare disabled. In this mode, the timer will count up from zero until it reaches ICR1. It
    // then counts back down to zero, and the overflow interrupt triggers when the count reaches
    // zero. Also, disable the input capture noise canceler and don't care about the input capture
    // edge trigger (so just set to falling.) Lastly, we disable prescaling.
    TCCR1A = 0;
    TCCR1B = ((0x2) << 3) | 0x01;

    // ICR1 is the top value (the midpoint of the period) since the counter increments and then
    // decrements before triggering the interrupt on returning to zero.
    ICR1 = ((uint16_t)(PRIORITY_CONTEXT0_PERIOD_USEC * CPU_MHZ)) >> 1;

    // Set the counter to zero
    TCNT1 = 0;

    // Set output compare to maximum
    OCR1A = 0xFFFF;
    OCR1B = 0xFFFF;

    interrupts();
}

//**************************************************************************************************
// Enable the interrupt context 0
//**************************************************************************************************
static inline void PriorityContext0Run(void)
{
    TIMSK1 = 0x01;
}

//**************************************************************************************************
// Disable the interrupt for context 0
//**************************************************************************************************
static inline void PriorityContext0Stop(void)
{
    TIMSK1 = 0x00;
}

//**************************************************************************************************
// Process button
//**************************************************************************************************
static inline void ButtonProcess(const Millis nowMillis)
{
    gButtonPrevProcessMillis = nowMillis;

    const uint8_t buttonLevel = digitalRead(PIN_BUTTON);

    // If the level just changed, then note the new level and the time and return
    if (buttonLevel != gButtonPrevLevel) {
        gButtonPrevLevel        = buttonLevel;
        gButtonPrevChangeMillis = nowMillis;

        return;
    }

    // If the level has not changed but is not yet stable, then return
    const Millis levelElapsedMillis = ElapsedMillis(gButtonPrevChangeMillis, nowMillis);
    if (levelElapsedMillis < BUTTON_STABLE_LEVEL_MILLIS) {
        return;
    }

    // Assume the level is stable from here on

    if (buttonLevel == LOW) {
        if (gButtonPrevStableLevel == HIGH) {
            // On transition from stable high to stable low, clear the hold flag and note the time
            gButtonDidHandleHold   = false;
            gButtonStableLowMillis = nowMillis;
        } else if (!gButtonDidHandleHold) {
            // If the level has been stable low for long enough and the UI context is not forcing a
            // hold, then we have a hold so note the hold for the UI context and note that we don't
            // have a click
            const Millis holdElapsedMillis = ElapsedMillis(gButtonStableLowMillis, nowMillis);
            if ((holdElapsedMillis > BUTTON_HOLD_MILLIS) && !gButtonIsForceHold) {
                gButtonDidHold       = true;
                gButtonDidHandleHold = true;
            }
        }
    } else {
        if (gButtonPrevStableLevel == LOW) {
            if (gButtonDidHandleHold || gButtonIsForceHold) {
                // On transition from stable low to stable high, if the stable low resulted in a
                // hold or the UI context forced a hold, then just drop the pending click (i.e. 
                // a click-hold is not a click or a double click, it's just a hold)
                gButtonIsClickPending = false;
                // Since we just transitioned from stable low to stable high, we can clear the
                // forced hold flag set by the UI context
                gButtonIsForceHold = false;
            } else {
                // On transition from stable low to stable high, if we have a click pending, then
                // we have a double click so note the double click for the UI context and clear
                // the click pending flag
                if (gButtonIsClickPending) {
                    gButtonDblClickCount++;
                    gButtonIsClickPending = false;
                } else {
                    // Otherwise we have a possible click pending so note the time and set the
                    // click pending flag
                    gButtonStableHighMillis = nowMillis;
                    gButtonIsClickPending   = true;
                }
            }
        } else if (gButtonIsClickPending) {
            // If there is a click pending and the level has been stable high for long enough,
            // then we have a click so note the click for the UI context and clear the click 
            // pending flag 
            const Millis releaseElapsedMillis = ElapsedMillis(gButtonStableHighMillis, nowMillis);
            if (releaseElapsedMillis > BUTTON_DBL_CLICK_MILLIS) {
                gButtonClickCount++;
                gButtonIsClickPending = false;
            }
        }
    }

    // Note the stable level and the current state for the UI context
    gButtonPrevStableLevel = buttonLevel;
    gButtonIsDown          = buttonLevel == LOW;
}

//**************************************************************************************************
// Process ADC (mod wheel)
//**************************************************************************************************
static void ADCMWProcess(uint16_t adcValue)
{
    // Clamp input to 10 bits
    if (adcValue > 1023) {
        adcValue = 1023;
    }

    // Mod-wheel doesn't mechanically go to zero, so we shift values to account for this non-zero
    // minimum
    if (adcValue <= gCalibrationOffsetMW) {
        adcValue = 0;
    } else {
        adcValue -= gCalibrationOffsetMW;
    }

    gADCMWFastSum[gADCMWFastSumIndex] += adcValue;
    gADCMWFastSumCount++;

    if (gADCMWFastSumCount == 16) {
        // Take the sums and divide by 32 (rounding up) to find the average 
        const uint16_t valueAvg = (gADCMWFastSum[0] + gADCMWFastSum[1] + (1 << 4)) >> 5;
        const uint8_t  value    = ADCMWScaleToMIDI(valueAvg);

        gADCMWFastSumIndex                = gADCMWFastSumIndex ? 0 : 1;
        gADCMWFastSum[gADCMWFastSumIndex] = 0;
        gADCMWFastSumCount                = 0;

        // Feed the ring buffer
        *gADCMWFastBufferPtr = value;
        gADCMWFastBufferPtr  =
          gADCMWFastBufferPtr < gADCMWFastBufferPtrMax ? gADCMWFastBufferPtr + 1
                                                       : gADCMWFastBufferPtrMin;

        // If we have a new value and all three values in the ring buffer are different, then update
        if ((value != gADCMWValue)                       &&
            (gADCMWFastBuffer[0] != gADCMWFastBuffer[1]) &&
            (gADCMWFastBuffer[1] != gADCMWFastBuffer[2]) &&
            (gADCMWFastBuffer[2] != gADCMWFastBuffer[0])) {
            gADCMWValue = value;

            const uint8_t inactiveSlowIndex = gADCMWSlowSumIndex ? 0 : 1;

            gADCMWSlowSum[gADCMWSlowSumIndex] = 0;
            gADCMWSlowSum[inactiveSlowIndex]  = valueAvg * 32;
            gADCMWSlowSumCount                = 0;
        }
    }

    gADCMWSlowSum[gADCMWSlowSumIndex] += adcValue;
    gADCMWSlowSumCount++;

    if (gADCMWSlowSumCount == 32) {
        // Take the sum and divide by 64 (rounding up) to find the average
        const uint16_t valueAvg = (gADCMWSlowSum[0] + gADCMWSlowSum[1] + (1 << 5)) >> 6;
        uint8_t        value    = ADCMWScaleToMIDI(valueAvg);

        gADCMWSlowSumIndex                = gADCMWSlowSumIndex ? 0 : 1;
        gADCMWSlowSum[gADCMWSlowSumIndex] = 0;
        gADCMWSlowSumCount                = 0;

        const uint8_t inactiveFastIndex   = gADCMWFastSumIndex ? 0 : 1;
        gADCMWFastSum[gADCMWFastSumIndex] = 0;
        gADCMWFastSum[inactiveFastIndex]  = valueAvg * 16;
        gADCMWFastSumCount                = 0;

        gADCMWFastBuffer[0] = value;
        gADCMWFastBuffer[1] = value;
        gADCMWFastBuffer[2] = value;

        // If the value differs from the previous value, then lowpass filter by averaging the new
        // value and the previous value (rounding down)
        if (value != gADCMWValue) {
            value = (value + gADCMWValue) >> 1;

            // If we still have a new value after the average, then update
            if (value != gADCMWValue) {
                gADCMWValue = value;
            }
        }
    }

    if (gADCMWValue > 127) {
        gADCMWValue = 127;
    }
}

//**************************************************************************************************
// Process ADC (expression pedal)
//**************************************************************************************************
static void ADCPedProcess(uint16_t adcValue)
{
    if (adcValue >= gCalibrationOffsetPed) {
        adcValue -= gCalibrationOffsetPed;
     } else {
        adcValue = 0;
     }

    // Clamp input to 10 bits
    if (adcValue > 1023) {
        adcValue = 1023;
    }

    gADCPedSum[gADCPedSumIndex] += adcValue;
    gADCPedSumCount++;

    if (gADCPedSumCount == 2) {
        const uint16_t adcAvg = (gADCPedSum[0] + gADCPedSum[1] + (1 << 1)) >> 2;
        uint8_t        value  = ADCPedScaleToMIDI(adcAvg);

        if (gADCPedValue != value) {
            gADCPedValue = (gADCPedValue + value) >> 1;
            if (gADCPedValue > 127) {
                gADCPedValue = 127;
            }
        }

        gADCPedSumIndex             = gADCPedSumIndex == 0 ? 1 : 0;
        gADCPedSum[gADCPedSumIndex] = 0;
        gADCPedSumCount             = 0;
    }
}

//**************************************************************************************************
// Process encoder
//**************************************************************************************************
static inline void EncoderProcess(const Millis nowMillis)
{
    const uint8_t clockLevel = digitalRead(PIN_ENCODER_CLOCK);
    const uint8_t dataLevel  = digitalRead(PIN_ENCODER_DATA);

    // The encoder levels are noisy, so wait for both clock and data levels to be stable by checking
    // that two readings in a row are the same for both clock and data
    gEncClockLevelBuffer[gEncClockLevelBufferIndex] = clockLevel;
    gEncDataLevelBuffer[gEncDataLevelBufferIndex]   = dataLevel;

    gEncClockLevelBufferIndex = gEncClockLevelBufferIndex ? 0 : 1;
    gEncDataLevelBufferIndex  = gEncDataLevelBufferIndex  ? 0 : 1;

    if (gEncClockLevelBuffer[0] != gEncClockLevelBuffer[1]) {
        return;
    }

    if (gEncDataLevelBuffer[0] != gEncDataLevelBuffer[1]) {
        return;
    }

    // We read consider the data level only once per clock cycle (on clock level low)

    if (clockLevel == gEncPrevClockLevel) {
        return;
    }

    // Once we have a stable clock level change, note the current level and time so that we can
    // throttle calls to this function
    gEncPrevClockLevel        = clockLevel;
    gEncPrevClockChangeMillis = nowMillis;

    // Only consider data level when the clock is in the low phase
    if (clockLevel == HIGH) {
        return;
    }

    if (dataLevel == LOW) {
        if (gEncValue > -128) {
            gEncValue--;
        }
    } else {
        if (gEncValue < 127) {
            gEncValue++;
        }
    }
}

//**************************************************************************************************
// Process MIDI input velocity (data byte)
//**************************************************************************************************
static inline void MIDIProcessInputVelocity(uint8_t* const inoutMIDIBytePtr)
{
    uint8_t velocity = *inoutMIDIBytePtr;

    if (gMIDIVelocityShouldBlackKeyAdjust && gIsMIDINoteBlackKeyLUT[gMIDIInputNoteValue]) {
        velocity = gMIDIVelocityBlackKeyAdjustLUT[velocity];
    }

    if (velocity < gMIDIVelocityCompLimit) {
        if (velocity > gMIDIVelocityCompThresh) {
            const uint8_t  threshedV = velocity - gMIDIVelocityCompThresh;
            const uint16_t multedV   = (uint16_t)threshedV * gMIDIVelocityCompRatioMultN;
            const uint8_t  compedV   = (uint8_t)((multedV + (uint16_t)256) >> (uint16_t)9);

            velocity = gMIDIVelocityCompThresh + compedV;
        }

        velocity += gMIDIVelocityCompMakeup;
    }

    if (velocity < gMIDIVelocityLowThresh) {
        velocity = gMIDIVelocityLowThresh;
    } else if (velocity > 127) {
        velocity = 127;
    }

    *inoutMIDIBytePtr = velocity;
}

//**************************************************************************************************
// Process MIDI input status byte
//**************************************************************************************************
static inline void MIDIProcessInputStatusByte(const uint8_t midiByte)
{
    gMIDIInputPrevStatus        = midiByte;
    gMIDIInputDataByteCount     = MIDIGetDataByteCount(gMIDIInputPrevStatus);
    gMIDIInputCurrDataByteCount = gMIDIInputDataByteCount;
    gMIDIInputIsRunningStatus   = false;

    // If we have a CC status byte, assume for now that we'll filter the entire message
    gMIDIInputIsCCFiltering = MIDI_STATUS_IS_CC(gMIDIInputPrevStatus) && gMIDIShouldReplaceModWheel;

    // And note that we are dropping the status byte in case it turns out that we shouldn't
    // be filtering and thus need to send this status byte later
    gMIDIInputDidDropStatus = gMIDIInputIsCCFiltering;
}

//**************************************************************************************************
// Process MIDI input data byte
//**************************************************************************************************
static inline void MIDIProcessInputDataByte(uint8_t* const inoutMIDIBytePtr)
{
    const uint8_t midiByte = *inoutMIDIBytePtr;

    // If input is running status, but that running status was interrupted by a local
    // output, then we can send the input status now
    if (gMIDIOutputWasLastSend && gMIDIInputIsRunningStatus) {
        // If we're filtering the message, then note that we're effectively dropping the
        // status byte since we may need to send it later if it turns out that we don't
        // want to filter
        if (gMIDIInputIsCCFiltering) {
            gMIDIInputDidDropStatus = true;
        } else {
            MIDISend(gMIDIInputPrevStatus);
            gMIDIInputIsRunningStatus = false;
        }
    }

    // Check for velocity adjust
    if (MIDI_STATUS_IS_NOTE_ON(gMIDIInputPrevStatus)) {
        if (gMIDIInputCurrDataByteCount == 2) {
            gMIDIInputNoteValue = midiByte;
        } else if ((gMIDIInputCurrDataByteCount == 1) &&
                   (midiByte > 0)                     &&
                   (MIDI_STATUS_CHANNEL(gMIDIInputPrevStatus) == gMIDIVelocityProcessChan)) {
            gMIDIVelocityMonitorIn = midiByte;

            if (gMIDIShouldProcessVelocity) {
                MIDIProcessInputVelocity(inoutMIDIBytePtr);
            }

            gMIDIVelocityMonitorOut = *inoutMIDIBytePtr;
        }
    } else if (gMIDIInputIsCCFiltering            &&
               (gMIDIInputCurrDataByteCount == 2) &&
               (midiByte != 0x01)) {     
        // else if we're currently filtering a CC message but the CC number (1st data
        // byte) is not a match for filtering, then stop the filtering...

        gMIDIInputIsCCFiltering = false;

        // If we dropped the status byte, then send it now
        if (gMIDIInputDidDropStatus) {
            MIDISend(gMIDIInputPrevStatus);
            gMIDIInputDidDropStatus   = false;
            gMIDIInputIsRunningStatus = false;
        }
    }

    // Decrement the current data byte count (Note: 255 means no set limit (e.g. SysEx))
    if (gMIDIInputCurrDataByteCount != 255) {
        gMIDIInputCurrDataByteCount--;
    }
}

//**************************************************************************************************
// Process MIDI input
//**************************************************************************************************
static inline bool MIDIProcessInput(const Millis nowMillis)
{
    bool didProcessMIDIInput = false;

    do {
        const int ret = Serial.read();
        if (ret < 0) {
            break;
        }

        gUITimeoutTimestamp = nowMillis;

        const uint8_t midiByte = (uint8_t)ret;
        const bool    isStatus = MIDI_IS_STATUS(midiByte, gMIDIInputPrevStatus);

        // Check for new status byte
        if (isStatus) {
            MIDIProcessInputStatusByte(midiByte);
        } else if (gMIDIInputCurrDataByteCount > 0) {
            MIDIProcessInputDataByte(&midiByte);
        }

        // If we're not filtering, then send
        if (!gMIDIInputIsCCFiltering) {
            MIDISend(midiByte);
        }

        // The input message is complete iff we're no longer waiting for data bytes
        gMIDIInputIsMessageComplete = gMIDIInputCurrDataByteCount == 0;

        // Reset to allow for running status
        if (gMIDIInputCurrDataByteCount == 0) {
            gMIDIInputIsRunningStatus   = true;
            gMIDIInputDidDropStatus     = false;
            gMIDIInputCurrDataByteCount = gMIDIInputDataByteCount;
            gMIDIInputIsCCFiltering     =
              MIDI_STATUS_IS_CC(gMIDIInputPrevStatus) && gMIDIShouldReplaceModWheel;
        }

        // Update state to reflect that MIDI input was sent
        gMIDIOutputWasLastSend = false;
        didProcessMIDIInput    = true;
    } while (true);

    return didProcessMIDIInput;
}

//**************************************************************************************************
// Process MIDI input and output
//**************************************************************************************************
static inline void MIDIProcess(const Millis nowMillis)
{
    // Handle MIDI input
    const bool didProcessMIDIInput = MIDIProcessInput(nowMillis);

    // If we're not sending local mod wheel values, then update stored value so that when local
    // mod wheel is enabled only a new value is sent
    if (!gMIDIShouldReplaceModWheel) {
        gADCMWPrevValue = gADCMWValue;
    }

    // Same for expression pedal
    if (!gMIDIShouldSendPedal) {
        gADCPedPrevValue = gADCPedValue;
    }

    if (!gMIDIShouldReplaceModWheel && !gMIDIShouldSendPedal) {
        return;
    }

    // Only send local data if we didn't have MIDI input, and we aren't midway through sending an
    // input message
    if (didProcessMIDIInput || !gMIDIInputIsMessageComplete) {
        return;
    }

    // Only send changes
    const bool isModWheelValueNew = gADCMWValue != gADCMWPrevValue;
    const bool isPedalValueNew    = gADCPedValue != gADCPedPrevValue;

    if (!isModWheelValueNew && !isPedalValueNew) {
        return;
    }

    if (isModWheelValueNew) {
        const uint8_t status = 0xB0 | (gMIDIOutputModWheelChan & 0x0F);
        const uint8_t msg[3] = { status, gMIDIOutputModWheelCC, gADCMWValue };
        MIDISend(msg, 3);

        gADCMWPrevValue = gADCMWValue;
    }

    if (isPedalValueNew) {
        const uint8_t status = 0xB0 | (gMIDIOutputPedalChan & 0x0F);
        const uint8_t msg[3] = { status, gMIDIOutputPedalCC, gADCPedValue };
        MIDISend(msg, 3);

        gADCPedPrevValue = gADCPedValue;
    }

    gMIDIOutputWasLastSend = true;

    gUITimeoutTimestamp = nowMillis;  
}

//**************************************************************************************************
// Process ADC
//**************************************************************************************************
static inline void ADCProcess(void)
{
    const uint16_t adcValue = ADCGetValue();

    if (adcValue == ADC_NO_VALUE) {
        return;
    }

    const uint8_t adcChan = ADCGetChannel();

    if (adcChan == ADC_CHANNEL_PED) {
        ADCPedProcess(adcValue);
        ADCStart(ADC_CHANNEL_MW);

        return;
    }

    ADCMWProcess(adcValue);

    gADCCounter++;

    if (gADCCounter < 16) {
        ADCStart(ADC_CHANNEL_MW);
    } else {
        ADCStart(ADC_CHANNEL_PED);
        gADCCounter = 0;
    }
}

//**************************************************************************************************
// ISR for priority context 0
//**************************************************************************************************
ISR(PRIORITY_CONTEXT0_INT_VECT)
{
#if USE_TIMING_DEBUG
    static Micros     lastStartMicros = micros();
    volatile Micros   startMicros     = micros();
    volatile Micros   elapsedMicros   = ElapsedMicros(lastStartMicros, startMicros);
    if ((uint16_t)elapsedMicros > gDebugMicros1) {
        gDebugMicros1 = elapsedMicros;
    }
    startMicros     = micros();
    lastStartMicros = startMicros;
#endif

    // Prevent this function from being reentrant
    PriorityContext0Stop();

    // And now re-enable interrupts
    interrupts();

    // Do the work for this context

    const Millis nowMillis = NowMillis();

#if !USE_SERIAL_DEBUG
    MIDIProcess(nowMillis);
#endif

    ADCProcess();

    if (ElapsedMillis(gButtonPrevProcessMillis, nowMillis) >= BUTTON_PROCESS_MILLIS) {
        ButtonProcess(nowMillis);
    }

    if (ElapsedMillis(gEncPrevClockChangeMillis, nowMillis) >= ENCODER_PROCESS_WAIT_MILLIS) {
        EncoderProcess(nowMillis);
    }

#if USE_TIMING_DEBUG
    elapsedMicros = ElapsedMicros(startMicros, micros());
    if ((uint16_t)elapsedMicros > gDebugMicros2) {
        gDebugMicros2 = elapsedMicros;
    }
#endif

    // Re-enable this context
    PriorityContext0Run();
}

//**************************************************************************************************
// Find the wear-level location of the current preset index (and the index itself)
//**************************************************************************************************
static uint8_t FindPresetIndexWearLevelLocation(uint8_t* const retIndex)
{
    for (uint8_t i = 0; i < PRESET_INDEX_WEAR_LEVEL_COUNT; i++) {
        const uint8_t index = EEPROM.read(i);

        if ((index >= 0) && (index < PRESET_SLOT_COUNT)) {
            if (retIndex != NULL) {
                *retIndex = index;
            }
            return i;
        }
    }

    if (retIndex != NULL) {
        *retIndex = 0;
    }
    return 0;
}

//**************************************************************************************************
// Store the new current preset index in EEPROM
//**************************************************************************************************
static void StorePresetIndex(const uint8_t index)
{
    uint8_t location = FindPresetIndexWearLevelLocation(NULL);

    if (EEPROM.read(location) != index) {
        // Invalidate the current location
        EEPROM.update(location, 255);
        // Store to the new location
        location = location < PRESET_SLOT_COUNT - 1 ? location + 1 : 0;
        EEPROM.update(location, index);
    }

    return location;
}

//**************************************************************************************************
// Store all parameters
//**************************************************************************************************
static void StoreAll(const uint8_t presetIndex)
{
    // The first preset block holds non-preset data
    const uint16_t byteOffset = (presetIndex + 1) * PRESET_BLOCK_BYTE_COUNT;

    EEPROM.update(byteOffset + 0,  gMIDIShouldReplaceModWheel);
    EEPROM.update(byteOffset + 1,  gMIDIOutputModWheelCC);
    EEPROM.update(byteOffset + 2,  gMIDIOutputModWheelChan);
    EEPROM.update(byteOffset + 3,  gMIDIShouldSendPedal);
    EEPROM.update(byteOffset + 4,  gMIDIOutputPedalCC);
    EEPROM.update(byteOffset + 5,  gMIDIOutputPedalChan);
    EEPROM.update(byteOffset + 6,  gMIDIShouldProcessVelocity);
    EEPROM.update(byteOffset + 7,  gMIDIVelocityLowThresh);
    EEPROM.update(byteOffset + 8,  gMIDIVelocityCompThresh);
    EEPROM.update(byteOffset + 9,  gMIDIVelocityCompRaioIndex);
    EEPROM.update(byteOffset + 10, gMIDIVelocityCompLimit);
    EEPROM.update(byteOffset + 11, gMIDIVelocityProcessChan);
    EEPROM.update(byteOffset + 12, gMIDIVelocityShouldBlackKeyAdjust);
    EEPROM.update(byteOffset + 13, gCalibrationScaleMW);
    EEPROM.update(byteOffset + 14, gCalibrationOffsetMW);
    EEPROM.update(byteOffset + 15, gCalibrationScalePed);
    EEPROM.update(byteOffset + 16, gCalibrationOffsetPed);

    gPrevPresetIndex = presetIndex;
    StorePresetIndex(presetIndex);
}

//**************************************************************************************************
// Load all parameters
//
// Note that under normal circumstances, interrupts should probably be disabled before calling this
//**************************************************************************************************
static void LoadAll(const uint8_t presetIndex)
{
    // The first preset block holds non-preset data
    const uint16_t byteOffset = (presetIndex + 1) * PRESET_BLOCK_BYTE_COUNT;

    gMIDIShouldReplaceModWheel        = EEPROM.read(byteOffset + 0);
    gMIDIOutputModWheelCC             = EEPROM.read(byteOffset + 1);
    gMIDIOutputModWheelChan           = EEPROM.read(byteOffset + 2);
    gMIDIShouldSendPedal              = EEPROM.read(byteOffset + 3);
    gMIDIOutputPedalCC                = EEPROM.read(byteOffset + 4);
    gMIDIOutputPedalChan              = EEPROM.read(byteOffset + 5);
    gMIDIShouldProcessVelocity        = EEPROM.read(byteOffset + 6);
    gMIDIVelocityLowThresh            = EEPROM.read(byteOffset + 7);
    gMIDIVelocityCompThresh           = EEPROM.read(byteOffset + 8);
    gMIDIVelocityCompRaioIndex        = EEPROM.read(byteOffset + 9);
    gMIDIVelocityCompLimit            = EEPROM.read(byteOffset + 10);
    gMIDIVelocityProcessChan          = EEPROM.read(byteOffset + 11);
    gMIDIVelocityShouldBlackKeyAdjust = EEPROM.read(byteOffset + 12);
    gCalibrationScaleMW               = EEPROM.read(byteOffset + 13);
    gCalibrationOffsetMW              = EEPROM.read(byteOffset + 14);
    gCalibrationScalePed              = EEPROM.read(byteOffset + 15);
    gCalibrationOffsetPed             = EEPROM.read(byteOffset + 16);

    gPrevPresetIndex = presetIndex;
    StorePresetIndex(presetIndex);

    for (uint8_t i = 0; i < gUIParamPageCount; i++) {
        UIParamPage* const paramPage = &(gUIParamPages[i]);

        if (!paramPage->isReadWrite) {
            continue;
        }

        if (*(paramPage->valuePtr) < paramPage->valueMin) {
            *(paramPage->valuePtr) = paramPage->valueMin;
        } else if (*(paramPage->valuePtr) > paramPage->valueMax) {
            *(paramPage->valuePtr) = paramPage->valueMax;
        }
    }

    uint16_t multN  = 0;
    uint8_t  makeup = 0;
    VelocityCompGetRatioAndMakeup(gMIDIVelocityCompThresh, gMIDIVelocityCompLimit, &multN, &makeup);
    gMIDIVelocityCompRatioMultN = multN;
    gMIDIVelocityCompMakeup     = makeup;
}

//**************************************************************************************************
// Get UI encoder events
//**************************************************************************************************
static void UIGetEncoderEvents(int8_t* const retEncValue)
{
    noInterrupts();
    const int8_t encValue = gEncValue;
    gEncValue             = 0;
    interrupts();

    *retEncValue = encValue;
}

//**************************************************************************************************
// Get UI button events
//**************************************************************************************************
static void UIGetButtonEvents(const bool     didEncoderChange,
                              uint8_t* const retButtonClickCount,
                              uint8_t* const retButtonDblClickCount,
                              bool* const    retIsButtonDown,
                              bool* const    retDidButtonHold)
{
    noInterrupts();
    const uint8_t buttonClickCount = gButtonClickCount;
    gButtonClickCount              = 0;

    const uint8_t buttonDblClickCount = gButtonDblClickCount;
    gButtonDblClickCount              = 0;

    const bool isButtonDown = gButtonIsDown;

    const bool didButtonHold = gButtonDidHold;
    gButtonDidHold           = false;

    if (isButtonDown && didEncoderChange) {
        gButtonIsForceHold = true;
    }
    interrupts();

    *retButtonClickCount    = buttonClickCount;
    *retButtonDblClickCount = buttonDblClickCount;
    *retIsButtonDown        = isButtonDown;
    *retDidButtonHold       = didButtonHold;
}

//**************************************************************************************************
// Get UI events
//**************************************************************************************************
static void UIGetEvents(int8_t* const  retEncValue,
                               uint8_t* const retButtonClickCount,
                               uint8_t* const retButtonDblClickCount,
                               bool* const    retIsButtonDown,
                               bool* const    retDidButtonHold)
{
    UIGetEncoderEvents(retEncValue);

    const bool didEncoderChange = *retEncValue != 0;

    UIGetButtonEvents(didEncoderChange,
                      retButtonClickCount,
                      retButtonDblClickCount,
                      retIsButtonDown,
                      retDidButtonHold);
}

//**************************************************************************************************
// Update parameter page navigation
//**************************************************************************************************
static bool UIParamPageNav(uint8_t buttonClickCount, uint8_t buttonDblClickCount)
{
    constexpr uint8_t maxParamPageIndex = gUIParamPageCount - 1;
    const uint8_t     prevPageIndex     = gUIParamPageIndex;               

    while (buttonClickCount > 0) {
        gUIParamPageIndex = gUIParamPageIndex >= maxParamPageIndex ? 0 : gUIParamPageIndex + 1;

        // Skip calibration parameters if calibration is not active
        if (!gIsCalibrationActive && gUIParamPages[gUIParamPageIndex].isCalibration) {
            continue;
        }

        buttonClickCount--;
    }

    while (buttonDblClickCount) {
        gUIParamPageIndex = gUIParamPageIndex == 0 ? maxParamPageIndex : gUIParamPageIndex - 1;

        // Skip calibration parameters if calibration is not active
        if (!gIsCalibrationActive && gUIParamPages[gUIParamPageIndex].isCalibration) {
            continue;
        }

        buttonDblClickCount--;
    }

    if (gUIParamPageIndex == prevPageIndex) {
        return false;
    }

    // For preset load and store pages, set the preset index to the most recently loaded or stored
    const uint8_t* const valuePtr = gUIParamPages[gUIParamPageIndex].valuePtr;
    if ((valuePtr == &gStoreState) || (valuePtr == &gLoadState)) {
        gPresetIndex = gPrevPresetIndex;
        gLoadState   = 1;
    }

    return true;
}

//**************************************************************************************************
// Calculate values for velocity compression given threshold and using global ratio index
//**************************************************************************************************
static void VelocityCompGetRatioAndMakeup(uint8_t         thresh,
                                          const uint8_t   limit,
                                          uint16_t* const retMultN,
                                          uint8_t* const  retMakeup)
{
    // gMIDIVelocityCompRaioIndex is UI only, so we don't need to worry about atomic access

    const uint8_t  index = gMIDIVelocityCompRaioIndex + 4;
    const uint16_t multN = (((uint16_t)20480 / (uint16_t)index) + (uint16_t)5) / (uint16_t)10;

    *retMultN = multN;

    if (thresh > limit) {
        thresh = limit;
    }

    const uint16_t threshU16       = (uint16_t)thresh;
    const uint16_t limitU16        = (uint16_t)limit;
    const uint16_t threshOffsetMax = (((limitU16 - threshU16) * multN) >> (uint16_t)9);

    if (threshOffsetMax >= (limitU16 - threshU16)) {
        *retMakeup = 0;
        return;
    }

    const uint16_t makeupU16 = limitU16 - (threshU16 + threshOffsetMax);

    *retMakeup = (uint8_t)makeupU16;
}

//**************************************************************************************************
// Timestamp the UI timeout
//**************************************************************************************************
static void UITimeoutTimestamp(void)
{
    const Millis nowMillis = NowMillis();

    noInterrupts();
    gUITimeoutTimestamp = nowMillis;
    interrupts();
}

//**************************************************************************************************
// Return true iff the UI has timeout
//**************************************************************************************************
static bool UIDidTimeout()
{
    noInterrupts();
    const Millis uiTimeoutTimestamp = gUITimeoutTimestamp;
    interrupts();

    if (ElapsedMillis(uiTimeoutTimestamp, NowMillis()) >= (uint32_t)UI_TIMEOUT_MILLIS) {
        return true;
    } else {
        return false;
    }
}

//**************************************************************************************************
// Update parameter value for velocity compression threshold
//**************************************************************************************************
static void UIUpdateParamVelocityCompThresh(const uint8_t newParamValue)
{
    uint16_t multN  = 0;
    uint8_t  makeup = 0;
    VelocityCompGetRatioAndMakeup(newParamValue, gMIDIVelocityCompLimit, &multN, &makeup);

    noInterrupts();
    gMIDIVelocityCompThresh     = newParamValue;
    gMIDIVelocityCompRatioMultN = multN;
    gMIDIVelocityCompMakeup     = makeup;
    interrupts();
}

//**************************************************************************************************
// Update parameter value for velocity compression limit
//**************************************************************************************************
static void UIUpdateParamVelocityCompLimit(const uint8_t newParamValue)
{
    uint16_t multN  = 0;
    uint8_t  makeup = 0;
    VelocityCompGetRatioAndMakeup(gMIDIVelocityCompThresh, newParamValue, &multN, &makeup);

    noInterrupts();
    gMIDIVelocityCompLimit      = newParamValue;
    gMIDIVelocityCompRatioMultN = multN;
    gMIDIVelocityCompMakeup     = makeup;
    interrupts();
}

//**************************************************************************************************
// Update parameter value for velocity compression ratio
//**************************************************************************************************
static void UIUpdateParamVelocityCompRatioIndex(uint8_t newParamValue, const bool isPressAndTurn)
{
    // gMIDIVelocityCompRaioIndex is UI only, so we don't need to worry about atomic access

    if (isPressAndTurn) {
        const bool shouldRound = (newParamValue & 0x03) != 0x00;

        if (shouldRound) {
            if (newParamValue > gMIDIVelocityCompRaioIndex) {
                newParamValue += 4;
            }
            newParamValue = newParamValue & ((uint8_t)(~(0x03)));
        }
    }

    gMIDIVelocityCompRaioIndex = newParamValue;

    noInterrupts();
    const uint8_t thresh = gMIDIVelocityCompThresh;
    const uint8_t limit  = gMIDIVelocityCompLimit;
    interrupts();

    uint16_t multN  = 0;
    uint8_t  makeup = 0;
    VelocityCompGetRatioAndMakeup(thresh, limit, &multN, &makeup);

    noInterrupts();
    gMIDIVelocityCompRatioMultN = multN;
    gMIDIVelocityCompMakeup     = makeup;
    interrupts();
}

//**************************************************************************************************
// Update preset index
//**************************************************************************************************
static bool UIUpdateParamPresetIndex(const int8_t encValue)
{
    if (encValue == 0) {
        return false;
    }

    int16_t newParamValueI16 = (int16_t)gPresetIndex + (int16_t)encValue;

    if (newParamValueI16 < 0) {
        newParamValueI16 = 0;
    } else if (newParamValueI16 >= PRESET_SLOT_COUNT) {
        newParamValueI16 = PRESET_SLOT_COUNT - 1;
    }
    const uint8_t newParamValue = (uint8_t)newParamValueI16;

    if (newParamValue != gPresetIndex) {
        gPresetIndex = newParamValue;
        return true;
    } else {
        return false;
    }
}

//**************************************************************************************************
// Update parameter value for preset store
//**************************************************************************************************
static uint8_t UIUpdateParamPresetStore(const int8_t encValue, const bool didButtonHold)
{
    if (didButtonHold && gStoreState) {
        noInterrupts();
        StoreAll(gPresetIndex);
        interrupts();

        gStoreState = 0;

        // Returning a value that is necessarily different from the value of the parameter page
        // variable will force a UI display update 
        return 255;
    }

    if (UIUpdateParamPresetIndex(encValue)) {
        gStoreState = 1;
    }

    return gPresetIndex;
}

//**************************************************************************************************
// Update parameter value for preset store
//**************************************************************************************************
static uint8_t UIUpdateParamPresetLoad(const int8_t encValue, const bool didButtonHold)
{
    if (didButtonHold) {
        noInterrupts();
        LoadAll(gPresetIndex);
        interrupts();

        gStoreState = 0;
        gLoadState  = 0;

        // Returning a value that is necessarily different from the value of the parameter page
        // variable will force a UI display update 
        return 255;
    }

    if (UIUpdateParamPresetIndex(encValue)) {
        gLoadState = 1;
    }

    return gPresetIndex;
}

//**************************************************************************************************
// Update parameter value
//**************************************************************************************************
static uint8_t UIUpdateParam(const int8_t encValue,
                             const bool   isButtonDown,
                             const bool   didButtonHold)
{
    UIParamPage* const      paramPagePtr  = &(gUIParamPages[gUIParamPageIndex]);
    volatile uint8_t* const uiValuePtr    = paramPagePtr->valuePtr;

    if (uiValuePtr == &gStoreState) {
        return UIUpdateParamPresetStore(encValue, didButtonHold);
    } else if (uiValuePtr == &gLoadState) {
        return UIUpdateParamPresetLoad(encValue, didButtonHold);
    }

    const bool isReadWrite = paramPagePtr->isReadWrite;

    noInterrupts();
    const uint8_t oldParamValue = *uiValuePtr;
    interrupts();
    uint8_t newParamValue = oldParamValue;

    if (!isReadWrite) {
        return newParamValue;
    }

    const uint8_t maxParamValue    = paramPagePtr->valueMax;
    const uint8_t minParamValue    = paramPagePtr->valueMin;
    int16_t       newParamValueI16 = (int16_t)newParamValue;

    if (didButtonHold) {
        newParamValueI16 = minParamValue;
    }

    if (encValue) {
        int16_t delta = (int16_t)encValue;
        if (isButtonDown) {
            const bool isPressAndTurn = paramPagePtr->isPressAndTurn;
            if (isPressAndTurn) {
                delta *= 10;
            }
        }
        newParamValueI16 += delta;
    }

    if (newParamValueI16 < (int16_t)minParamValue) {
        newParamValueI16 = (int16_t)minParamValue;
    } else if (newParamValueI16 > (int16_t)maxParamValue) {
        newParamValueI16 = (int16_t)maxParamValue;
    }

    newParamValue = (uint8_t)newParamValueI16;

    if (newParamValue == oldParamValue) {
        return oldParamValue;
    }

    gStoreState = 1;

    if (uiValuePtr == &gMIDIVelocityCompRaioIndex) {
        UIUpdateParamVelocityCompRatioIndex(newParamValue, isButtonDown && encValue);
    } else if (uiValuePtr == &gMIDIVelocityCompThresh) {
        UIUpdateParamVelocityCompThresh(newParamValue);
    } else if (uiValuePtr == &gMIDIVelocityCompLimit) {
        UIUpdateParamVelocityCompLimit(newParamValue);
    } else {
        noInterrupts();
        *uiValuePtr = newParamValue;
        interrupts();
    }

    return newParamValue;
}

//**************************************************************************************************
// Display parameter label
//**************************************************************************************************
static void UIDisplayParamLabel(const bool shouldForceUpdate)
{
    if (!shouldForceUpdate) {
        return;
    }

    // Since the displayed parameter has changed, reset the display timeout
    UITimeoutTimestamp();

    gDisplay.setFont(u8x8_font_chroma48medium8_r);
    gDisplay.drawString(0, UI_LINE_OFFSET, "               ");
    gDisplay.drawString(0, UI_LINE_OFFSET, gUIParamPages[gUIParamPageIndex].label);
}

//**************************************************************************************************
// Display parameter value for velocity compression ratio
//**************************************************************************************************
static void UIDisplayParamValueVelocityCompRatioIndex()
{
    const uint16_t indexU16            = (uint16_t)gMIDIVelocityCompRaioIndex;
    const uint16_t ratioNumTimes100U16 = ((gMIDIVelocityCompRaioIndex + 4) * 100) >> 2;

    memset(gStrBuffer, 0, sizeof(gStrBuffer));
    utoa(ratioNumTimes100U16, gStrBuffer, 10);

    uint8_t len = 0;
    for (uint8_t i = 0; i < sizeof(gStrBuffer); i++) {
        if (gStrBuffer[i] == 0) {
            break;
        }
        len++;
    }

    if (len < 3) {
        return;
    }

    const uint8_t wholeDigitCount = len - 2;

    uint8_t fracDigitCount = 0;
    for (uint8_t i = wholeDigitCount; i < len; i++) {
        if (gStrBuffer[i] != '0') {
            fracDigitCount++;
        }
    }

    if (fracDigitCount == 0) {
        gStrBuffer[wholeDigitCount]     = ':';
        gStrBuffer[wholeDigitCount + 1] = '1';
        len                             = wholeDigitCount + 2;
    } else {
        uint8_t offs = 0;

        if (fracDigitCount == 2) {
            gStrBuffer[wholeDigitCount + 2] = gStrBuffer[wholeDigitCount + 1];
            offs = 3;
        } else {
            offs = 2;
        }

        gStrBuffer[wholeDigitCount + 1]        = gStrBuffer[wholeDigitCount];
        gStrBuffer[wholeDigitCount]            = '.';
        gStrBuffer[wholeDigitCount + offs + 0] = ':';
        gStrBuffer[wholeDigitCount + offs + 1] = '1';
        len                                    = wholeDigitCount + offs + 2;
    }

    gDisplay.drawString(0, UI_LINE_OFFSET + 3, gStrBuffer);

    memcpy(gStrBuffer, "        ", 9);
    gDisplay.drawString(len * 2, UI_LINE_OFFSET + 3, gStrBuffer + len);
}

//**************************************************************************************************
// Display velocity monitor
//**************************************************************************************************
static void UIDisplayParamValueVelocityMonitor(const bool shouldForceUpdate)
{
    noInterrupts();
    const uint8_t values[2] = { gMIDIVelocityMonitorIn, gMIDIVelocityMonitorOut };
    interrupts();

    const bool didChange =
      (values[0] != gUIParamPrevDisplayValues[0]) || (values[1] != gUIParamPrevDisplayValues[1]);

    if (!didChange && !shouldForceUpdate) {
        return;
    }

    gUIParamPrevDisplayValues[0] = values[0];
    gUIParamPrevDisplayValues[1] = values[1];

    // Since the value for the display has changed, reset the display timeout
    UITimeoutTimestamp();

    const uint8_t xOffsets[2] = { 0, 10 };

    for (uint8_t i = 0; i < 2; i++) {
        const uint8_t value = values[i];
        uint8_t       x     = xOffsets[i];

        memset(gStrBuffer, 0, sizeof(gStrBuffer));
        utoa(value, gStrBuffer, 10);

        if (value < 10) {
            gStrBuffer[2] = ' ';
            gStrBuffer[1] = gStrBuffer[0];
            gStrBuffer[0] = ' ';
        } else if (value < 100) {
            if (i == 0) {
                gStrBuffer[2] = ' ';

                gDisplay.drawString(0, UI_LINE_OFFSET + 3, " ");
                x++;
            } else {
                gStrBuffer[2] = gStrBuffer[1];
                gStrBuffer[1] = gStrBuffer[0];
                gStrBuffer[0] = ' ';

                gDisplay.drawString(14, UI_LINE_OFFSET + 3, " ");
                x--;
            }
        }

        gDisplay.drawString(x, UI_LINE_OFFSET + 3, gStrBuffer);
    }

    // Draw the arrow
    gStrBuffer[0] = 66;
    gStrBuffer[1] = 0;
    gDisplay.setFont(u8x8_font_open_iconic_arrow_2x2);
    gDisplay.drawString(7, UI_LINE_OFFSET + 3, gStrBuffer);
}

//**************************************************************************************************
// Display parameter value for preset
//**************************************************************************************************
static void UIDisplayParamValuePreset(void)
{
    memset(gStrBuffer, 0, sizeof(gStrBuffer));
    utoa(gPresetIndex + 1, gStrBuffer, 10);

    gDisplay.drawString(14, UI_LINE_OFFSET + 3, " ");
    gDisplay.drawString(12, UI_LINE_OFFSET + 3, gStrBuffer);
}

//**************************************************************************************************
// Display parameter value for store preset
//**************************************************************************************************
static void UIDisplayParamValueStoreState(void)
{
    if (gStoreState) {
        gDisplay.drawString(0, UI_LINE_OFFSET + 3, "Save? ");
        UIDisplayParamValuePreset();
    } else {
        gDisplay.drawString(0, UI_LINE_OFFSET + 3, "Saved   ");
    }
}

//**************************************************************************************************
// Display parameter value for load preset
//**************************************************************************************************
static void UIDisplayParamValueLoadState(void)
{
    if (gLoadState) {
        gDisplay.drawString(0, UI_LINE_OFFSET + 3, "Load? ");
        UIDisplayParamValuePreset();
    } else {
        gDisplay.drawString(0, UI_LINE_OFFSET + 3, "Loaded  ");
    }
}

//**************************************************************************************************
// Display parameter value
//**************************************************************************************************
static void UIDisplayParamValue(const uint8_t newParamValue, const bool shouldForceUpdate)
{
    const UIParamPage* const uiParamPage = &(gUIParamPages[gUIParamPageIndex]);

    gDisplay.setFont(u8x8_font_px437wyse700a_2x2_r);

    if (uiParamPage->valuePtr == &gMIDIVelocityMonitorOut) {
        UIDisplayParamValueVelocityMonitor(shouldForceUpdate);
        return;
    }

    const uint8_t newDisplayValue = newParamValue + uiParamPage->valueDisplayOffs;

    if ((newDisplayValue == gUIParamPrevDisplayValues[0]) && !shouldForceUpdate) {
        return;
    }
    gUIParamPrevDisplayValues[0] = newDisplayValue;

    // Since the displayed parameter value has changed, reset the display timeout
    UITimeoutTimestamp();

    if (uiParamPage->valuePtr == &gMIDIVelocityCompRaioIndex) {
        UIDisplayParamValueVelocityCompRatioIndex();
        return;
    } else if (uiParamPage->valuePtr == &gStoreState) {
        UIDisplayParamValueStoreState();
        return;
    } else if (uiParamPage->valuePtr == &gLoadState) {
        UIDisplayParamValueLoadState();
        return;
    }

    if ((uiParamPage->valueMin == 0) && (uiParamPage->valueMax == 1))
    {
        if (newDisplayValue) {
            gDisplay.drawString(0, UI_LINE_OFFSET + 3, "On ");
        } else {
            gDisplay.drawString(0, UI_LINE_OFFSET + 3, "Off");
        }

        return;
    }

    memset(gStrBuffer, 0, sizeof(gStrBuffer));
    utoa(newDisplayValue, gStrBuffer, 10);

    if (newDisplayValue < 100) {
        gStrBuffer[2] = ' ';
    }
    if (newDisplayValue < 10) {
        gStrBuffer[1] = ' ';
    }

    gDisplay.drawString(0, UI_LINE_OFFSET + 3, gStrBuffer);
}

#if USE_TIMING_DEBUG
//**************************************************************************************************
// Display debug timing info
//**************************************************************************************************
static void UIDebugDisplayTimingInfo(const bool shouldForceUpdate)
{
    static uint16_t debugMicros1, debugMicros2;

    noInterrupts();
    uint16_t newDebugMicros = gDebugMicros1;
    interrupts();
    if ((debugMicros1 != newDebugMicros) || shouldForceUpdate) {
        debugMicros1 = newDebugMicros;
        memset(gStrBuffer, 0, sizeof(gStrBuffer));
        utoa(newDebugMicros, gStrBuffer, 10);
        gDisplay.setFont(u8x8_font_chroma48medium8_r);
        gDisplay.drawString(11, 7, "     ");
        gDisplay.drawString(11, 7, gStrBuffer);
    }

    noInterrupts();
    newDebugMicros = gDebugMicros2;
    interrupts();
    if ((debugMicros2 != newDebugMicros) || shouldForceUpdate) {
        debugMicros2 = newDebugMicros;
        memset(gStrBuffer, 0, sizeof(gStrBuffer));
        utoa(newDebugMicros, gStrBuffer, 10);
        gDisplay.setFont(u8x8_font_chroma48medium8_r);
        gDisplay.drawString(0, 7, "     ");
        gDisplay.drawString(0, 7, gStrBuffer);
    }
}
#endif

//**************************************************************************************************
// Setup
//**************************************************************************************************
static bool SetupIsCalibrationActive(void)
{
    uint8_t highCount = 0;

    for (uint8_t i = 0; i < 200; i++) {
        if (digitalRead(PIN_BUTTON) == LOW) {
            highCount++;

            if (highCount > 100) {
                return true;
            }
        }
    }

    return false;
}

//**************************************************************************************************
// Setup
//**************************************************************************************************
void setup(void)
{
    CreateIsMIDINoteBlackKeyLUT();
    CreateMIDIVelocityBlackKeyAdjustLUT();

    noInterrupts();

#if USE_SERIAL_DEBUG
    Serial.begin(31250);
#else
    // MIDI
    Serial.begin(31250, SERIAL_8N1);
#endif

    gDisplay.begin();
    gDisplay.setFlipMode(1);
    gDisplay.setFont(u8x8_font_chroma48medium8_r);
    gDisplay.drawString(0, UI_LINE_OFFSET, "Starting...");

    pinMode(PIN_INT0,          INPUT_PULLUP);
    pinMode(PIN_INT1,          INPUT_PULLUP);
    pinMode(PIN_ENCODER_DATA,  INPUT_PULLUP);
    pinMode(PIN_ENCODER_CLOCK, INPUT_PULLUP);
    pinMode(PIN_BUTTON,        INPUT_PULLUP);
    pinMode(PIN_ADC,           INPUT);
    // Set INT0 and INT1 to falling edge
    EICRA = 0x0A;

    gIsCalibrationActive = SetupIsCalibrationActive();

    gEncPrevClockChangeMillis = NowMillis();

    const uint8_t location = FindPresetIndexWearLevelLocation(&gPresetIndex);
    LoadAll(gPresetIndex);

    ADCStart(ADC_CHANNEL_MW);
    for (uint16_t i = 0; i < 2000; i++) {
        delayMicroseconds(PRIORITY_CONTEXT0_PERIOD_USEC);
        ADCProcess();
    }
    gADCMWPrevValue  = gADCMWValue;
    gADCPedPrevValue = gADCPedValue;

    PriorityContext0Init();
    PriorityContext0Run();

    delay(2000);

    gDisplay.clearDisplay();

    interrupts();
}

//**************************************************************************************************
// Main loop (lowest priority context)
//**************************************************************************************************
void loop(void)
{
    static bool oncePred = true;

    bool didWake = false;
    if (UIDidTimeout()) {
        if (!gUIIsTimedOut) {
            gUIIsTimedOut = true;
            gDisplay.clearDisplay();
        }
    } else if (gUIIsTimedOut) {
        gUIIsTimedOut = false;
        didWake       = true;
    }

    int8_t  encValue;
    uint8_t buttonClickCount;
    uint8_t buttonDblClickCount;
    bool    isButtonDown;
    bool    didButtonHold;

    UIGetEvents(&encValue,
                &buttonClickCount,
                &buttonDblClickCount,
                &isButtonDown,
                &didButtonHold);

    const bool didParamPageChange = UIParamPageNav(buttonClickCount, buttonDblClickCount);
    const bool shouldForceUpdate  = didParamPageChange || didWake || oncePred;

    if (shouldForceUpdate) {
        gDisplay.clearDisplay();
    }

    UIDisplayParamLabel(shouldForceUpdate);

    const uint8_t newParamValue = UIUpdateParam(encValue, isButtonDown, didButtonHold);

    UIDisplayParamValue(newParamValue, shouldForceUpdate);

    oncePred = false;

#if USE_TIMING_DEBUG
    UIDebugDisplayTimingInfo(shouldForceUpdate);
#endif

    delay(UI_LOOP_SLEEP_MILLIS);
}
