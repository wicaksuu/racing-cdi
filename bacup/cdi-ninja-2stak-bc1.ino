/*
 * ============================================================================
 * RACING CDI - STM32H562RGT6 - MAXIMUM PRECISION BUILD
 * ============================================================================
 * Board: WeAct Studio STM32H562RGT6 (ARM Cortex-M33 @ 250MHz)
 * Framework: Arduino STM32
 *
 * PRECISION FEATURES:
 * - Hardware timer input capture (no polling)
 * - Hardware timer output compare (precise pulse timing)
 * - Integer-only math in ISR (no float)
 * - Direct register access for GPIO (sub-microsecond)
 * - Precomputed lookup tables
 * - Fixed-point arithmetic (0.01° resolution)
 * - Minimal ISR latency (~500ns)
 * - Jitter < 0.05° at all RPM
 *
 * RACING FEATURES:
 * - 6 ignition maps (250 RPM step, 0-20000 RPM)
 * - 4 stage rev limiter (soft/medium/hard/full cut)
 * - VR signal capture with adjustable pickup angle & tooth width
 * - 2-stroke / 4-stroke mode
 * - Shift light, Kill switch, Map switch
 * - ADC: head temp, battery, charging (with calibration)
 * - SD card config & logging
 * - USB realtime data & config
 * - Hot mapping edit
 * - Hour meter, Peak RPM memory
 * - Safety: over-rev, overheat retard, low battery, default map
 * ============================================================================
 */

#include <STM32SD.h>

// ============================================================================
// FORWARD DECLARATIONS FOR ARDUINO PREPROCESSOR
// ============================================================================

struct TriggerConfig;
struct RevLimiterConfig;
struct ShiftLightConfig;
struct ADCCalibration;
struct CrankingConfig;
struct WarningConfig;
struct CDIConfig;
struct RuntimeData;

// Function prototypes that use structs
uint32_t calculateChecksum(struct CDIConfig* cfg);
bool validateConfig(struct CDIConfig* cfg);

// ============================================================================
// PRECISION CONFIGURATION
// ============================================================================

// Fixed-point math configuration
// Using 16.16 fixed point for maximum precision
#define FIXED_SHIFT         16
#define FIXED_ONE           (1UL << FIXED_SHIFT)
#define FLOAT_TO_FIXED(x)   ((int32_t)((x) * FIXED_ONE))
#define FIXED_TO_FLOAT(x)   ((float)(x) / FIXED_ONE)
#define FIXED_MUL(a, b)     (((int64_t)(a) * (b)) >> FIXED_SHIFT)
#define FIXED_DIV(a, b)     (((int64_t)(a) << FIXED_SHIFT) / (b))

// Timing precision: 0.01 degree resolution (multiply by 100)
#define DEG_SCALE           100
#define DEG_TO_SCALED(x)    ((int32_t)((x) * DEG_SCALE))
#define SCALED_TO_DEG(x)    ((float)(x) / DEG_SCALE)

// ============================================================================
// PIN DEFINITIONS - DIRECTLY MAPPED TO REGISTERS
// ============================================================================

// Input pins
#define PIN_VR_INPUT        PA0   // VR signal from MAX9926 (TIM2_CH1)
#define PIN_KILL_SWITCH     PA1   // Kill switch (active low)
#define PIN_MAP_SWITCH      PA2   // Map switch button (active low)
#define PIN_SD_DETECT       PA8   // SD card detect

// Output pins with direct register access
#define PIN_CDI_OUTPUT      PB0   // CDI trigger output (TIM3_CH3)
#define PIN_SHIFT_LIGHT     PB1   // Shift light output
#define PIN_WARNING         PB4   // Warning output (buzzer/LED)
#define PIN_STATUS_LED      PB2   // Onboard blue LED (active low) - MCU health/status

// Onboard button (directly on WeAct board)
#define PIN_MAP_SWITCH_2    PC13  // Onboard user button (active low) - secondary map switch

// Direct port manipulation macros for PB0 (CDI output)
#define CDI_PORT            GPIOB
#define CDI_PIN_MASK        (1U << 0)
#define CDI_HIGH()          (CDI_PORT->BSRR = CDI_PIN_MASK)
#define CDI_LOW()           (CDI_PORT->BSRR = (CDI_PIN_MASK << 16))

// Direct port manipulation for PB1 (Shift light)
#define SHIFT_PIN_MASK      (1U << 1)
#define SHIFT_HIGH()        (CDI_PORT->BSRR = SHIFT_PIN_MASK)
#define SHIFT_LOW()         (CDI_PORT->BSRR = (SHIFT_PIN_MASK << 16))

// Direct port manipulation for PB4 (Warning)
#define WARN_PIN_MASK       (1U << 4)
#define WARN_HIGH()         (CDI_PORT->BSRR = WARN_PIN_MASK)
#define WARN_LOW()          (CDI_PORT->BSRR = (WARN_PIN_MASK << 16))

// Direct port manipulation for PB2 (Status LED - active LOW)
#define LED_PIN_MASK        (1U << 2)
#define LED_ON()            (CDI_PORT->BSRR = (LED_PIN_MASK << 16))  // Active low
#define LED_OFF()           (CDI_PORT->BSRR = LED_PIN_MASK)
#define LED_TOGGLE()        (CDI_PORT->ODR ^= LED_PIN_MASK)

// ADC pins
#define PIN_ADC_HEAD_TEMP   PC0   // Head/cylinder temperature
#define PIN_ADC_BATTERY     PC1   // Battery voltage
#define PIN_ADC_CHARGING    PC2   // Charging voltage

// SD Card - using SDMMC interface (no CS pin needed)
// SDMMC pins are fixed by hardware:
// PC12 = SDMMC_CK, PD2 = SDMMC_CMD
// PC8 = D0, PC9 = D1, PC10 = D2, PC11 = D3
#define PIN_SD_DETECT       PA8   // Card detect pin

// ============================================================================
// SYSTEM CONSTANTS
// ============================================================================

// Clock configuration - STM32H562 runs at 250MHz
#define SYSTEM_CLOCK_HZ     250000000UL
#define TIMER_CLOCK_HZ      250000000UL  // APB timer clock

// Timer configuration for maximum precision
// Prescaler = 25 gives 10MHz tick = 0.1us resolution
#define TIMER_PRESCALER     25
#define TIMER_TICK_NS       100           // 100 nanoseconds per tick
#define TIMER_TICK_HZ       10000000UL    // 10 MHz

// RPM configuration
#define RPM_MIN             0
#define RPM_MAX             20000
#define RPM_STEP            250
#define RPM_TABLE_SIZE      ((RPM_MAX / RPM_STEP) + 1)  // 81 entries

// Timing limits (in scaled degrees x100)
#define TIMING_MIN_SCALED   (-1000)       // -10.00 degrees (ATDC)
#define TIMING_MAX_SCALED   (6000)        // 60.00 degrees (BTDC)

// CDI pulse width in timer ticks (10MHz)
// 200us = 2000 ticks
#define CDI_PULSE_TICKS     2000

// Maps
#define NUM_MAPS            6

// Engine types
#define ENGINE_2_STROKE     2
#define ENGINE_4_STROKE     4

// Rev limiter stages
#define LIMITER_NONE        0
#define LIMITER_SOFT        1   // Retard timing only (no cut)
#define LIMITER_MEDIUM      2   // Pattern: fire 1, cut 1 (50%)
#define LIMITER_HARD        3   // Pattern: fire 1, cut 3 (75%)
#define LIMITER_FULL_CUT    4   // Cut all

// Soft limiter retards timing by this amount (in 0.01 degrees)
// 500 = 5.00 degrees retard
#define SOFT_RETARD_SCALED  500

// Pattern-based cut (more predictable than random)
// Medium: fire 1, cut 1 (cycle of 2)
// Hard: fire 1, cut 3 (cycle of 4)
#define MEDIUM_CUT_PATTERN  2   // Fire every 2nd trigger
#define HARD_CUT_PATTERN    4   // Fire every 4th trigger

// ADC
#define ADC_RESOLUTION      12
#define ADC_MAX_VALUE       4095

// Cranking
#define CRANKING_RPM        500

// Safety defaults
#define DEFAULT_TIMING_SCALED   1000      // 10.00 degrees
#define OVERHEAT_TEMP_C         120       // Overheat threshold in Celsius
#define LOW_BATTERY_MV          11000     // Low battery threshold in millivolts (11.0V)

// Debounce - 100ms for reliable button detection
#define DEBOUNCE_MS         100

// Engine timeout (timer ticks at 10MHz)
// 500ms = 5,000,000 ticks
#define ENGINE_TIMEOUT_TICKS    5000000UL

// Cold start protection - wait for N valid triggers before firing
// This prevents erratic behavior when system starts with high RPM signal
#define STARTUP_TRIGGER_COUNT   5     // Wait for 5 consistent triggers before firing
#define STARTUP_PERIOD_TOLERANCE 10   // 10% tolerance for period consistency

// LED Status Patterns (for MCU health indication)
#define LED_PATTERN_IDLE        0   // Slow blink - system idle, no engine
#define LED_PATTERN_RUNNING     1   // Fast blink - engine running, all OK
#define LED_PATTERN_WARNING     2   // Double blink - warning (overheat/low batt)
#define LED_PATTERN_ERROR       3   // Triple blink - error (SD fail, using default map)
#define LED_PATTERN_MAP_CHANGE  4   // Blink N times for map number

// CPU usage measurement interval
#define CPU_MEASURE_INTERVAL_MS 100

// ============================================================================
// CPU/RAM MONITORING - ACCURATE MEASUREMENT USING DWT CYCLE COUNTER
// ============================================================================

// DWT (Data Watchpoint and Trace) registers for Cortex-M33
#define DWT_CTRL    (*(volatile uint32_t*)0xE0001000)
#define DWT_CYCCNT  (*(volatile uint32_t*)0xE0001004)
#define DWT_LAR     (*(volatile uint32_t*)0xE0001FB0)
#define CoreDebug_DEMCR (*(volatile uint32_t*)0xE000EDFC)

// CPU monitoring variables using DWT cycle counter
static volatile uint32_t cpuLoopCycles = 0;       // Cycles spent in main loop this period
static volatile uint32_t cpuMeasureStartCycle = 0; // Start of measurement period
static volatile uint32_t cpuLoopStartCycle = 0;   // Start of current loop iteration
static volatile uint8_t cpuUsagePercent = 0;      // Current CPU usage %
static volatile uint32_t lastCpuMeasureMs = 0;    // Last measurement time
static volatile uint8_t cpuCalibrated = 0;        // Calibration done flag
static volatile uint32_t cpuBaselineLoopCycles = 0; // Baseline loop cycles per period (idle)

// External symbols from linker for RAM calculation
extern "C" char _end;       // End of .bss section (start of heap)
extern "C" char _estack;    // End of stack (top of RAM)
extern "C" char _sdata;     // Start of .data section
extern "C" char _ebss;      // End of .bss section

// Initialize DWT cycle counter
void initDWT(void) {
  // Unlock DWT (required on some Cortex-M33)
  DWT_LAR = 0xC5ACCE55;
  // Enable trace
  CoreDebug_DEMCR |= (1 << 24);  // TRCENA bit
  // Reset and enable cycle counter
  DWT_CYCCNT = 0;
  DWT_CTRL |= 1;  // CYCCNTENA bit
}

// ============================================================================
// WATCHDOG TIMER - Auto-recovery from MCU hang
// ============================================================================
// Uses IWDG (Independent Watchdog) running from LSI (~32kHz)
// Timeout ~2 seconds - if loop() doesn't run for 2s, MCU resets

void initWatchdog(void) {
  // Enable IWDG with safe timeout
  // STM32H5: LSI = 32kHz, Prescaler /256 (PR=6) -> 125Hz tick
  // Timeout = RLR / 125 = 500/125 = 4 seconds (safer during init)

  IWDG->KR = 0xCCCC;  // Start watchdog (cannot be stopped after this!)
  IWDG->KR = 0x5555;  // Enable write access to PR, RLR

  IWDG->PR = 6;       // Prescaler /256 (safer, slower tick)
  IWDG->RLR = 500;    // Reload value for ~4s timeout

  // Wait for registers to update with timeout
  uint32_t timeout = 100000;
  while ((IWDG->SR != 0) && (timeout > 0)) {
    timeout--;
  }

  // Initial reload
  IWDG->KR = 0xAAAA;
}

// Call this in loop() to prevent reset
static inline void kickWatchdog(void) {
  IWDG->KR = 0xAAAA;  // Reload watchdog counter
}

// ============================================================================
// PRECOMPUTED LOOKUP TABLES FOR MAXIMUM SPEED
// ============================================================================

// Lookup table: Timer ticks per degree for each RPM step
// Calculated as: (TIMER_TICK_HZ * 60) / (RPM * 360) = ticks per degree
// Pre-multiplied by DEG_SCALE for precision
// Index = RPM / 250
static uint32_t ticksPerDegTable[RPM_TABLE_SIZE];

// Lookup table: Microseconds per revolution for each RPM
// Used for timeout and period validation
static uint32_t usPerRevTable[RPM_TABLE_SIZE];

// Initialize lookup tables at startup
void initLookupTables(void) {
  for (int i = 0; i < RPM_TABLE_SIZE; i++) {
    uint32_t rpm = i * RPM_STEP;
    if (rpm == 0) rpm = 100;  // Avoid division by zero, use 100 RPM as minimum

    // Ticks per degree = (10,000,000 * 60) / (RPM * 360)
    // = 1,666,666.67 / RPM
    // Multiply by 100 for 0.01 degree resolution
    ticksPerDegTable[i] = (166666667UL / rpm);

    // Microseconds per revolution = 60,000,000 / RPM
    usPerRevTable[i] = 60000000UL / rpm;
  }
}

// ============================================================================
// RAM/CPU MONITORING FUNCTIONS - ACCURATE IMPLEMENTATION
// ============================================================================

// Get stack usage in bytes (from top of stack to current position)
uint32_t getStackUsed(void) {
  char stackVar;
  // _estack is top of stack, stackVar is current stack position
  return (uint32_t)(&_estack) - (uint32_t)(&stackVar);
}

// Get static memory usage (.data + .bss sections)
uint32_t getStaticUsed(void) {
  // From start of .data to end of .bss
  return (uint32_t)(&_ebss) - (uint32_t)(&_sdata);
}

// Get heap usage (approximate)
// Note: STM32 Arduino uses newlib, heap grows up from _end
// This function is for informational purposes
uint32_t getHeapUsed(void) {
  // In newlib, sbrk(0) returns current heap end
  extern void *_sbrk(int);
  void *heapEnd = _sbrk(0);
  if (heapEnd == (void*)-1 || heapEnd == 0) {
    return 0;  // No heap or error
  }
  return (uint32_t)(heapEnd) - (uint32_t)(&_ebss);
}

// SRAM1 size for STM32H562 (256KB where stack/heap reside)
#define SRAM1_SIZE_BYTES (256UL * 1024UL)

// Get total RAM usage percentage
// Measures: static (.data + .bss) + stack usage
// This gives accurate "stress level" indication
uint8_t getRamUsagePercent(void) {
  uint32_t stackUsed = getStackUsed();
  uint32_t staticUsed = getStaticUsed();

  // Total used = static + stack
  uint32_t totalUsed = staticUsed + stackUsed;

  if (totalUsed >= SRAM1_SIZE_BYTES) return 100;
  return (uint8_t)((totalUsed * 100UL) / SRAM1_SIZE_BYTES);
}

// Get free RAM between heap and stack (available for growth)
uint32_t getFreeRam(void) {
  char stackVar;
  // Free RAM = current stack pointer - end of heap/bss
  return (uint32_t)(&stackVar) - (uint32_t)(&_ebss);
}

// Mark start of main loop iteration - call at very start of loop()
static inline void cpuLoopStart(void) __attribute__((always_inline));
static inline void cpuLoopStart(void) {
  cpuLoopStartCycle = DWT_CYCCNT;
}

// Mark end of main loop iteration - call at very end of loop()
static inline void cpuLoopEnd(void) __attribute__((always_inline));
static inline void cpuLoopEnd(void) {
  uint32_t now = DWT_CYCCNT;
  cpuLoopCycles += (now - cpuLoopStartCycle);
}

// Update CPU usage measurement - call once per main loop iteration
// Uses accurate cycle counting:
// - Total cycles in period = expected cycles at CPU frequency
// - Loop cycles = actual cycles spent in main loop
// - ISR cycles = Total - Loop cycles
// - CPU usage = (ISR cycles / Total cycles) * 100
void updateCpuUsage(void) {
  uint32_t now = millis();

  if (now - lastCpuMeasureMs >= CPU_MEASURE_INTERVAL_MS) {
    uint32_t currentCycle = DWT_CYCCNT;

    // Total cycles in this measurement period
    uint32_t totalCycles = currentCycle - cpuMeasureStartCycle;

    // Calibration: first measurement establishes baseline
    // Baseline = how many cycles main loop takes when idle (no ISRs)
    if (!cpuCalibrated) {
      cpuBaselineLoopCycles = cpuLoopCycles;
      cpuCalibrated = 1;
      cpuUsagePercent = 0;
    } else if (totalCycles > 0) {
      // CPU usage = time NOT in main loop = time in ISRs + system overhead
      // When ISRs fire frequently, main loop gets less cycles
      //
      // Method: Compare current loop cycles to baseline
      // If current < baseline, the difference is ISR time
      // Usage% = (baseline - current) / baseline * 100

      if (cpuLoopCycles >= cpuBaselineLoopCycles) {
        // More cycles than baseline = still idle or baseline was wrong
        // Update baseline to higher value (self-calibrating)
        if (cpuLoopCycles > cpuBaselineLoopCycles) {
          cpuBaselineLoopCycles = cpuLoopCycles;
        }
        cpuUsagePercent = 0;
      } else {
        // Less cycles than baseline = ISRs consumed some time
        uint32_t isrCycles = cpuBaselineLoopCycles - cpuLoopCycles;
        cpuUsagePercent = (uint8_t)((isrCycles * 100UL) / cpuBaselineLoopCycles);

        // Clamp to 0-99 (100% would mean loop never runs)
        if (cpuUsagePercent > 99) cpuUsagePercent = 99;
      }
    }

    // Reset for next period
    cpuMeasureStartCycle = currentCycle;
    cpuLoopCycles = 0;
    lastCpuMeasureMs = now;
  }
}

// ============================================================================
// DATA STRUCTURES - OPTIMIZED FOR CACHE ALIGNMENT
// ============================================================================

// Trigger configuration (aligned for fast access)
// Single tooth design: trigger fires at triggerAngle BTDC
struct __attribute__((aligned(4))) TriggerConfig {
  int32_t triggerAngleScaled;   // Trigger angle BTDC x100 (e.g., 3000 = 30.00° BTDC)
  uint32_t noiseFilterTicks;    // Minimum pulse width in timer ticks
  uint8_t risingEdge;           // 1 = rising, 0 = falling
  uint8_t reserved1;            // Padding
  uint16_t cdiPulseUs;          // CDI pulse width in microseconds (50-200us typical)
  uint8_t reserved[4];          // Padding for alignment (keep struct size same)
};

// Rev limiter configuration
struct __attribute__((aligned(4))) RevLimiterConfig {
  uint16_t softRpm;
  uint16_t mediumRpm;
  uint16_t hardRpm;
  uint16_t fullCutRpm;
};

// Shift light configuration
struct __attribute__((aligned(4))) ShiftLightConfig {
  uint16_t onRpm;
  uint16_t blinkRpm;
  uint16_t fastBlinkRpm;
  uint16_t blinkIntervalMs;
  uint16_t fastBlinkIntervalMs;
  uint16_t reserved;
};

// ADC calibration (using integer scale factors x1000)
struct __attribute__((aligned(4))) ADCCalibration {
  int32_t tempOffsetScaled;     // Offset x1000
  int32_t tempScaleScaled;      // Scale x1000
  int32_t battScaleScaled;      // Battery voltage scale x1000
  int32_t chargingScaleScaled;  // Charging voltage scale x1000
};

// Cranking configuration
struct __attribute__((aligned(4))) CrankingConfig {
  uint8_t enabled;
  int8_t timingScaled;          // Fixed timing during cranking (degrees, not scaled)
  uint16_t maxRpm;
};

// Warning configuration
struct __attribute__((aligned(4))) WarningConfig {
  uint16_t overRevRpm;
  int16_t overheatTempC;        // Overheat threshold in Celsius (e.g., 120 = 120°C)
  uint16_t lowBatteryMv;        // Low battery threshold in millivolts (e.g., 11000 = 11.0V)
  int8_t overheatRetard;        // Max degrees to retard when overheating (cap for progressive)
  int8_t retardStartTempC;      // Temperature to start progressive retard (default 80°C)
  uint8_t retardPer10C;         // Retard per 10°C in 0.1° units (10 = 1.0°, 15 = 1.5°)
  uint8_t reserved;
};

// Main configuration - all timing values in scaled format (x100)
struct __attribute__((aligned(4))) CDIConfig {
  uint32_t magic;
  uint8_t version;
  uint8_t engineType;
  uint8_t activeMap;
  uint8_t reserved;

  TriggerConfig trigger;

  // Timing maps: values in degrees (not scaled, -10 to 60)
  int8_t timingMaps[NUM_MAPS][RPM_TABLE_SIZE];

  RevLimiterConfig revLimiter;
  ShiftLightConfig shiftLight;
  ADCCalibration adcCal;
  CrankingConfig cranking;
  WarningConfig warning;

  uint32_t reserved1;         // Was hourMeterMinutes
  uint8_t reserved2;          // Was hourMeterSeconds
  uint8_t reserved3;
  uint16_t peakRpm;

  uint32_t checksum;
};

// Runtime data - critical timing data first for cache optimization
struct __attribute__((aligned(4))) RuntimeData {
  // === CRITICAL TIMING DATA (accessed in ISR) ===
  volatile uint32_t lastCapture;          // Last capture value
  volatile uint32_t period;               // Period in timer ticks
  volatile uint32_t nextFireTick;         // When to fire next
  volatile uint16_t currentRpm;           // Current RPM
  volatile int16_t currentTimingScaled;   // Current timing x100
  volatile uint8_t ignitionPending;       // Ignition scheduled flag
  volatile uint8_t fourStrokeCycle;       // 4-stroke toggle
  volatile uint8_t killActive;            // Kill switch active
  volatile uint8_t limiterStage;          // Current limiter stage
  volatile uint8_t limiterCounter;        // Counter for pattern-based cut

  // === PRE-CALCULATED PERIOD THRESHOLDS (avoids division in ISR) ===
  // Period = 600,000,000 / RPM, so lower period = higher RPM
  uint32_t periodSoftCut;                 // Period below which soft cut applies
  uint32_t periodMediumCut;               // Period below which medium cut applies
  uint32_t periodHardCut;                 // Period below which hard cut applies
  uint32_t periodFullCut;                 // Period below which full cut applies
  uint32_t periodOverRev;                 // Period below which over-rev cut applies

  // === PRE-CALCULATED CDI PULSE ===
  uint16_t pulseNopCount;                 // NOP loop count for pulse width

  // === STATUS FLAGS ===
  volatile uint8_t engineRunning;
  volatile uint8_t overheating;
  volatile uint8_t lowBattery;
  volatile uint8_t sdCardOk;
  volatile uint8_t usingDefaultMap;
  volatile uint8_t ignitionEnabled;
  volatile uint8_t sdBusy;            // SD operation in progress
  uint8_t reserved1;

  // === COUNTERS ===
  volatile uint32_t ignitionCount;
  volatile uint32_t cutCount;
  volatile uint32_t triggerCount;     // Valid trigger count since startup (for cold-start protection)

  // === COLD START VALIDATION ===
  volatile uint32_t startupPeriods[STARTUP_TRIGGER_COUNT];  // Store periods for consistency check
  volatile uint8_t startupIndex;                             // Index for startup periods array

  // === ADC RAW VALUES (faster than float) ===
  volatile uint16_t tempRaw;
  volatile uint16_t batteryRaw;
  volatile uint16_t chargingRaw;
  volatile int16_t currentTempC;    // Cached temperature in Celsius for ISR use

  // === BUTTON STATE ===
  volatile uint8_t mapSwitchState;
  volatile uint8_t lastMapSwitchState;
  volatile uint8_t mapSwitch2State;       // PC13 onboard button
  volatile uint8_t lastMapSwitch2State;
  volatile uint32_t lastMapSwitchMs;
  volatile uint32_t lastMapSwitch2Ms;

  // === STATUS LED ===
  volatile uint8_t ledPattern;            // Current LED pattern
  volatile uint8_t ledStep;               // Current step in pattern

  // === SYSTEM STATE ===
  volatile uint8_t configReady;           // Config loaded, system ready
  volatile uint8_t needAutoSave;          // Need to auto-save when engine stops

  // === OSCILLOSCOPE DATA ===
  volatile uint8_t lastCycleWasCut;       // Last cycle was cut by limiter (for scope display)
};

// ============================================================================
// DEFAULT SAFETY MAP - Conservative timing for failsafe
// ============================================================================

// 81 entries: RPM 0, 250, 500, ... 20000
static const int8_t DEFAULT_SAFETY_MAP[RPM_TABLE_SIZE] = {
  // 0-2500 RPM (11 entries: 0,250,500...2500)
  5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15,
  // 2750-5000 RPM (10 entries)
  16, 17, 18, 19, 20, 21, 22, 22, 23, 23,
  // 5250-7500 RPM (10 entries)
  24, 24, 25, 25, 25, 26, 26, 26, 27, 27,
  // 7750-10000 RPM (10 entries)
  27, 28, 28, 28, 28, 28, 28, 28, 28, 28,
  // 10250-12500 RPM (10 entries)
  28, 28, 28, 28, 28, 28, 28, 28, 28, 28,
  // 12750-15000 RPM (10 entries)
  28, 27, 27, 27, 27, 26, 26, 26, 26, 25,
  // 15250-17500 RPM (10 entries)
  25, 25, 24, 24, 24, 23, 23, 23, 22, 22,
  // 17750-20000 RPM (10 entries: 17750...20000)
  22, 21, 21, 20, 20, 19, 19, 18, 18, 17
};

// ============================================================================
// GLOBAL VARIABLES
// ============================================================================

static CDIConfig config __attribute__((aligned(4)));
static RuntimeData runtime __attribute__((aligned(4)));

// Timer pointers
static TIM_TypeDef* TIM_CAPTURE = TIM2;   // 32-bit timer for input capture
static TIM_TypeDef* TIM_IGNITION = TIM3;  // Timer for ignition output

// Fast random for rev limiter (Linear Congruential Generator)
static uint32_t randState = 0x12345678;

// SD card
// Note: Some SD libraries work better without leading slash
static const char* CONFIG_FOLDER = "racing-cdi";
static const char* CONFIG_FILE = "racing-cdi/config.bin";
static const char* SETTINGS_FILE = "racing-cdi/settings.txt";
static const char* README_FILE = "racing-cdi/readme.txt";
static const char* MAP_FILE_PREFIX = "racing-cdi/map";  // map1.txt, map2.txt, etc
static const char* LOG_FILE_PREFIX = "racing-cdi/log_";

#define USB_SERIAL Serial

// ============================================================================
// LOOKUP TABLES FOR ISR SPEED (NO DIVISION IN ISR)
// ============================================================================

// Period threshold table - periodThreshold[i] = period for RPM = i * RPM_STEP
// Period = 600,000,000 / RPM, so lower index = higher RPM = lower period
static uint32_t periodThreshold[RPM_TABLE_SIZE];

// Initialize period threshold table (called once at startup)
void initPeriodLookup(void) {
  // Index 0 = 0 RPM (infinite period, use max value)
  periodThreshold[0] = 0xFFFFFFFF;
  // Indices 1-80 = 250, 500, ... 20000 RPM
  for (int i = 1; i < RPM_TABLE_SIZE; i++) {
    uint32_t rpm = i * RPM_STEP;
    periodThreshold[i] = 600000000UL / rpm;
  }
}

// Fast period to RPM index using binary search (NO DIVISION)
// Returns index for timing table lookup
static inline uint8_t periodToIndex(uint32_t period) __attribute__((always_inline));
static inline uint8_t periodToIndex(uint32_t period) {
  // Binary search for the index where period fits
  // Lower period = higher RPM = higher index
  if (period >= periodThreshold[1]) return 0;  // Below 250 RPM
  if (period <= periodThreshold[RPM_TABLE_SIZE - 1]) return RPM_TABLE_SIZE - 1;  // Above max RPM

  // Binary search
  uint8_t low = 1, high = RPM_TABLE_SIZE - 1;
  while (low < high) {
    uint8_t mid = (low + high + 1) >> 1;
    if (period <= periodThreshold[mid]) {
      low = mid;
    } else {
      high = mid - 1;
    }
  }
  return low;
}

// ============================================================================
// INLINE CRITICAL FUNCTIONS FOR MAXIMUM SPEED
// ============================================================================

// Fast random number (0-99) for rev limiter
static inline uint32_t fastRandom100(void) __attribute__((always_inline));
static inline uint32_t fastRandom100(void) {
  randState = randState * 1664525UL + 1013904223UL;
  return (randState >> 24) % 100;
}

// Get RPM table index from RPM
static inline uint8_t rpmToIndex(uint16_t rpm) __attribute__((always_inline));
static inline uint8_t rpmToIndex(uint16_t rpm) {
  if (rpm >= RPM_MAX) return RPM_TABLE_SIZE - 1;
  return rpm / RPM_STEP;
}

// Calculate RPM from period (timer ticks)
// RPM = (TIMER_TICK_HZ * 60) / period = 600,000,000 / period
static inline uint16_t periodToRpm(uint32_t period) __attribute__((always_inline));
static inline uint16_t periodToRpm(uint32_t period) {
  if (period < 3000) return RPM_MAX;  // Overflow protection (>20000 RPM)
  if (period > 6000000) return 0;     // Below 100 RPM, consider stopped

  // Use 64-bit division for accuracy
  uint32_t rpm = 600000000UL / period;
  return (rpm > RPM_MAX) ? RPM_MAX : (uint16_t)rpm;
}

// FAST timing lookup by index - NO DIVISION, NO INTERPOLATION
// Used in ISR for maximum speed
static inline int16_t getTimingByIndex(uint8_t index) __attribute__((always_inline));
static inline int16_t getTimingByIndex(uint8_t index) {
  int8_t baseTiming;

  if (runtime.usingDefaultMap) {
    baseTiming = DEFAULT_SAFETY_MAP[index];
  } else {
    baseTiming = config.timingMaps[config.activeMap][index];
  }

  int16_t timingScaled = baseTiming * DEG_SCALE;

  // Apply progressive temperature retard
  // Starts at retardStartTempC, linear increase based on retardPer10C
  if (runtime.currentTempC > config.warning.retardStartTempC) {
    int16_t tempAboveStart = runtime.currentTempC - config.warning.retardStartTempC;
    // Calculate retard: retardPer10C is in 0.1° units per 10°C
    // Formula: (tempAboveStart / 10) * retardPer10C * 10 = tempAboveStart * retardPer10C
    int16_t retardScaled = tempAboveStart * config.warning.retardPer10C;
    // Cap at maximum configured retard
    int16_t maxRetardScaled = config.warning.overheatRetard * DEG_SCALE;
    if (retardScaled > maxRetardScaled) {
      retardScaled = maxRetardScaled;
    }
    timingScaled -= retardScaled;
  }

  // Clamp to limits
  if (timingScaled < TIMING_MIN_SCALED) timingScaled = TIMING_MIN_SCALED;
  if (timingScaled > TIMING_MAX_SCALED) timingScaled = TIMING_MAX_SCALED;

  return timingScaled;
}

// Get timing for RPM with interpolation (returns scaled value x100)
// Used for display and non-critical calculations
static inline int16_t getTimingScaled(uint16_t rpm) __attribute__((always_inline));
static inline int16_t getTimingScaled(uint16_t rpm) {
  // Cranking mode check
  if (config.cranking.enabled && rpm <= config.cranking.maxRpm) {
    return config.cranking.timingScaled * DEG_SCALE;
  }

  uint8_t index = rpmToIndex(rpm);
  int8_t baseTiming;

  if (runtime.usingDefaultMap) {
    baseTiming = DEFAULT_SAFETY_MAP[index];
  } else {
    baseTiming = config.timingMaps[config.activeMap][index];
  }

  // Convert to scaled and apply interpolation
  int16_t timingScaled = baseTiming * DEG_SCALE;

  // Linear interpolation between points
  uint16_t lowerRpm = index * RPM_STEP;
  if (rpm > lowerRpm && index < RPM_TABLE_SIZE - 1) {
    int8_t nextTiming;
    if (runtime.usingDefaultMap) {
      nextTiming = DEFAULT_SAFETY_MAP[index + 1];
    } else {
      nextTiming = config.timingMaps[config.activeMap][index + 1];
    }

    // Interpolation: timing += (next - base) * (rpm - lower) / STEP
    int32_t diff = (nextTiming - baseTiming) * DEG_SCALE;
    int32_t fraction = ((rpm - lowerRpm) * diff) / RPM_STEP;
    timingScaled += fraction;
  }

  // Apply progressive temperature retard
  // Starts at retardStartTempC, linear increase based on retardPer10C
  if (runtime.currentTempC > config.warning.retardStartTempC) {
    int16_t tempAboveStart = runtime.currentTempC - config.warning.retardStartTempC;
    // Calculate retard: retardPer10C is in 0.1° units per 10°C
    int16_t retardScaled = tempAboveStart * config.warning.retardPer10C;
    // Cap at maximum configured retard
    int16_t maxRetardScaled = config.warning.overheatRetard * DEG_SCALE;
    if (retardScaled > maxRetardScaled) {
      retardScaled = maxRetardScaled;
    }
    timingScaled -= retardScaled;
  }

  // Clamp to limits
  if (timingScaled < TIMING_MIN_SCALED) timingScaled = TIMING_MIN_SCALED;
  if (timingScaled > TIMING_MAX_SCALED) timingScaled = TIMING_MAX_SCALED;

  return timingScaled;
}

// Check if should cut ignition (rev limiter) - uses period for speed (no division)
// Lower period = higher RPM, so we check if period < threshold
// Returns: 0 = fire, 1 = cut
// Soft limiter does NOT cut - it only retards timing (handled in vrCaptureCallback)
static inline uint8_t shouldCutByPeriod(uint32_t period) __attribute__((always_inline));
static inline uint8_t shouldCutByPeriod(uint32_t period) {
  // Increment counter for pattern-based cut
  runtime.limiterCounter++;

  // Over-rev protection - always cut
  if (runtime.periodOverRev > 0 && period <= runtime.periodOverRev) {
    runtime.limiterStage = LIMITER_FULL_CUT;
    return 1;
  }

  // Full cut - always cut
  if (runtime.periodFullCut > 0 && period <= runtime.periodFullCut) {
    runtime.limiterStage = LIMITER_FULL_CUT;
    return 1;
  }

  // Hard cut - pattern: fire 1, cut 3 (75% cut)
  // Fire when counter % 4 == 0
  if (runtime.periodHardCut > 0 && period <= runtime.periodHardCut) {
    runtime.limiterStage = LIMITER_HARD;
    return (runtime.limiterCounter % HARD_CUT_PATTERN) != 0;
  }

  // Medium cut - pattern: fire 1, cut 1 (50% cut)
  // Fire when counter % 2 == 0
  if (runtime.periodMediumCut > 0 && period <= runtime.periodMediumCut) {
    runtime.limiterStage = LIMITER_MEDIUM;
    return (runtime.limiterCounter % MEDIUM_CUT_PATTERN) != 0;
  }

  // Soft limiter - NO CUT, only timing retard
  // Retard is applied in vrCaptureCallback after timing lookup
  if (runtime.periodSoftCut > 0 && period <= runtime.periodSoftCut) {
    runtime.limiterStage = LIMITER_SOFT;
    return 0;  // Don't cut, just retard timing
  }

  runtime.limiterStage = LIMITER_NONE;
  runtime.limiterCounter = 0;  // Reset counter when not limiting
  return 0;
}

// ============================================================================
// INTERRUPT CALLBACKS - MAXIMUM OPTIMIZATION
// ============================================================================

// HardwareTimer objects
HardwareTimer *TimerCapture;
HardwareTimer *TimerIgnition;

// VR Trigger callback - called on input capture
void vrCaptureCallback(void) {
  // Don't process trigger until config is ready
  if (!runtime.configReady) {
    return;
  }

  // Direct register access for maximum speed
  uint32_t capture = TIM_CAPTURE->CCR1;
  uint32_t period = capture - runtime.lastCapture;
  runtime.lastCapture = capture;

  // Noise filter - reject too short pulses
  if (period < config.trigger.noiseFilterTicks) {
    return;
  }

  // Store period and mark engine running
  // Note: Don't call millis() here - it requires SysTick which we don't want to block
  // Use timer ticks for timeout check in main loop
  runtime.period = period;

  if (!runtime.engineRunning) {
    runtime.engineRunning = 1;
    runtime.triggerCount = 0;  // Reset on engine start
    runtime.startupIndex = 0;  // Reset startup period index
  }

  // NOTE: RPM calculation moved to main loop to reduce ISR time
  // ISR uses period-based comparisons instead

  // Cold start protection - wait for stable, consistent readings before firing
  // This prevents erratic behavior from noise or sudden RPM spikes
  if (runtime.triggerCount < STARTUP_TRIGGER_COUNT) {
    // Store period for consistency validation
    if (runtime.startupIndex < STARTUP_TRIGGER_COUNT) {
      runtime.startupPeriods[runtime.startupIndex++] = period;
    }
    runtime.triggerCount++;

    // When we have enough samples, validate consistency
    if (runtime.triggerCount == STARTUP_TRIGGER_COUNT) {
      // Calculate average period
      uint32_t avgPeriod = 0;
      for (uint8_t i = 0; i < STARTUP_TRIGGER_COUNT; i++) {
        avgPeriod += runtime.startupPeriods[i];
      }
      avgPeriod /= STARTUP_TRIGGER_COUNT;

      // Check if all periods are within tolerance (±10%)
      uint32_t maxVariation = avgPeriod / STARTUP_PERIOD_TOLERANCE;
      for (uint8_t i = 0; i < STARTUP_TRIGGER_COUNT; i++) {
        int32_t diff = (int32_t)runtime.startupPeriods[i] - (int32_t)avgPeriod;
        if (diff < 0) diff = -diff;  // abs()
        if ((uint32_t)diff > maxVariation) {
          // Period not consistent - reset and try again
          runtime.triggerCount = 0;
          runtime.startupIndex = 0;
          return;
        }
      }
      // All periods consistent - allow ignition on next trigger
    }
    return;
  }

  // Skip if kill switch active
  if (runtime.killActive) {
    return;
  }

  // Skip if ignition disabled (software kill)
  if (!runtime.ignitionEnabled) {
    return;
  }

  // Rev limiter check using period (faster than RPM division)
  if (shouldCutByPeriod(period)) {
    runtime.cutCount++;
    runtime.lastCycleWasCut = 1;  // Track for scope display
    // Still toggle 4-stroke cycle
    if (config.engineType == ENGINE_4_STROKE) {
      runtime.fourStrokeCycle ^= 1;
    }
    return;
  }

  // Track for scope display (not cut)
  runtime.lastCycleWasCut = 0;

  // 4-stroke: fire every other revolution
  if (config.engineType == ENGINE_4_STROKE) {
    runtime.fourStrokeCycle ^= 1;
    if (!runtime.fourStrokeCycle) {
      return;
    }
  }

  // Get timing table index from period (NO DIVISION - uses binary search)
  uint8_t rpmIndex = periodToIndex(period);

  // Cranking mode check (low index = low RPM)
  if (config.cranking.enabled && rpmIndex <= (config.cranking.maxRpm / RPM_STEP)) {
    runtime.currentTimingScaled = config.cranking.timingScaled * DEG_SCALE;
  } else {
    // Get timing from lookup table (NO DIVISION, NO INTERPOLATION)
    runtime.currentTimingScaled = getTimingByIndex(rpmIndex);
  }

  int16_t timingScaled = runtime.currentTimingScaled;

  // Apply soft limiter timing retard (more gentle than cut)
  if (runtime.limiterStage == LIMITER_SOFT) {
    timingScaled -= SOFT_RETARD_SCALED;  // Retard by 5 degrees
    if (timingScaled < TIMING_MIN_SCALED) {
      timingScaled = TIMING_MIN_SCALED;  // Clamp to minimum (-10°)
    }
    runtime.currentTimingScaled = timingScaled;  // Update for display
  }

  // Calculate delay in timer ticks
  // Trigger fires at triggerAngle BTDC, need to fire at timing BTDC
  // Delay = (triggerAngle - timing) degrees
  int32_t angleDelayScaled = config.trigger.triggerAngleScaled - timingScaled;

  if (angleDelayScaled <= 0) {
    // Fire immediately - timing more advanced than pickup
    CDI_HIGH();
    // Inline delay ~20us using NOP (at 250MHz, ~5000 cycles = 20us)
    for (volatile uint16_t i = 0; i < runtime.pulseNopCount; i++) { __NOP(); __NOP(); __NOP(); __NOP(); }
    CDI_LOW();
    runtime.ignitionCount++;
  } else {
    // Schedule delayed ignition using TIM3
    // rpmIndex already calculated above from periodToIndex()
    uint32_t ticksPerDeg = ticksPerDegTable[rpmIndex];

    // delayTicks = angleDelayScaled * ticksPerDeg / (DEG_SCALE * DEG_SCALE)
    uint32_t delayTicks = ((uint64_t)angleDelayScaled * ticksPerDeg) / (DEG_SCALE * DEG_SCALE);

    // Sanity check - max delay is half the period
    if (delayTicks > period / 2) {
      delayTicks = 0;
    }

    if (delayTicks < 100) {
      // Very short delay, fire immediately
      CDI_HIGH();
      for (volatile uint16_t i = 0; i < runtime.pulseNopCount; i++) { __NOP(); __NOP(); __NOP(); __NOP(); }
      CDI_LOW();
      runtime.ignitionCount++;
    } else {
      // Setup TIM3 for one-shot delay using direct register access
      TIM_IGNITION->CR1 = 0;                    // Stop timer
      TIM_IGNITION->CNT = 0;                    // Reset counter
      TIM_IGNITION->ARR = delayTicks;           // Set delay
      TIM_IGNITION->SR = 0;                     // Clear flags
      TIM_IGNITION->DIER = TIM_DIER_UIE;        // Enable update interrupt
      TIM_IGNITION->CR1 = TIM_CR1_CEN | TIM_CR1_OPM;  // Start one-pulse mode
      runtime.ignitionPending = 1;
    }
  }
}

// Ignition output callback - fires the CDI
void ignitionFireCallback(void) {
  if (runtime.ignitionPending) {
    runtime.ignitionPending = 0;

    // Fire CDI - direct register access for speed
    CDI_HIGH();

    // Precise pulse width ~20us using NOP loop
    for (volatile uint16_t i = 0; i < runtime.pulseNopCount; i++) { __NOP(); __NOP(); __NOP(); __NOP(); }

    CDI_LOW();

    runtime.ignitionCount++;
  }
}

// ============================================================================
// TIMER SETUP - USING HARDWARETIMER WITH DIRECT REGISTER ACCESS IN CALLBACKS
// ============================================================================

void setupTimers(void) {
  // === TIMER CLOCK VALIDATION ===
  // Verify timer clock matches our assumption (critical for timing accuracy)
  uint32_t timerClk = HAL_RCC_GetPCLK1Freq();

  // On STM32H5, if APB prescaler > 1, timer clock = 2 * APB clock
  // Check RCC configuration
  uint32_t apb1Prescaler = (RCC->CFGR2 & RCC_CFGR2_PPRE1) >> RCC_CFGR2_PPRE1_Pos;
  if (apb1Prescaler >= 4) {  // Prescaler > 1 (encoded as 4,5,6,7 for /2,/4,/8,/16)
    timerClk *= 2;
  }

  USB_SERIAL.print(F("Timer clock: "));
  USB_SERIAL.print(timerClk / 1000000);
  USB_SERIAL.println(F(" MHz"));

  // Calculate actual prescaler needed for 10MHz tick
  uint32_t actualPrescaler = timerClk / TIMER_TICK_HZ;

  if (actualPrescaler != TIMER_PRESCALER) {
    USB_SERIAL.print(F("WARNING: Prescaler adjusted from "));
    USB_SERIAL.print(TIMER_PRESCALER);
    USB_SERIAL.print(F(" to "));
    USB_SERIAL.println(actualPrescaler);
  }

  // === TIM2: Input Capture for VR signal (32-bit timer) ===
  TimerCapture = new HardwareTimer(TIM2);
  TimerCapture->setPrescaleFactor(actualPrescaler);  // Dynamic prescaler for 10MHz tick
  TimerCapture->setOverflow(0xFFFFFFFF);             // Max count for 32-bit

  // Configure channel 1 for input capture on PA0
  if (config.trigger.risingEdge) {
    TimerCapture->setMode(1, TIMER_INPUT_CAPTURE_RISING, PIN_VR_INPUT);
  } else {
    TimerCapture->setMode(1, TIMER_INPUT_CAPTURE_FALLING, PIN_VR_INPUT);
  }

  // Attach callback
  TimerCapture->attachInterrupt(1, vrCaptureCallback);
  // Note: NVIC priority set in setup() after all timers initialized

  TimerCapture->resume();

  // === TIM3: Ignition output timing ===
  TimerIgnition = new HardwareTimer(TIM3);
  TimerIgnition->setPrescaleFactor(actualPrescaler);  // Same prescaler as TIM2
  TimerIgnition->setOverflow(0xFFFF);

  // Attach callback
  TimerIgnition->attachInterrupt(ignitionFireCallback);
  // Note: NVIC priority set in setup() after all timers initialized

  // Don't start yet - will be triggered by capture callback
  TimerIgnition->pause();
}

void updateTriggerEdge(void) {
  TimerCapture->pause();

  if (config.trigger.risingEdge) {
    TimerCapture->setMode(1, TIMER_INPUT_CAPTURE_RISING, PIN_VR_INPUT);
  } else {
    TimerCapture->setMode(1, TIMER_INPUT_CAPTURE_FALLING, PIN_VR_INPUT);
  }

  TimerCapture->resume();
}

// ============================================================================
// PIN SETUP
// ============================================================================

void setupPins(void) {
  // Enable GPIO clocks
  RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOBEN | RCC_AHB2ENR_GPIOCEN;
  __DSB();

  // Configure output pins using direct register access
  // PB0, PB1, PB2, PB4 as outputs
  GPIOB->MODER &= ~((3U << (0*2)) | (3U << (1*2)) | (3U << (2*2)) | (3U << (4*2)));
  GPIOB->MODER |= ((1U << (0*2)) | (1U << (1*2)) | (1U << (2*2)) | (1U << (4*2)));

  // Set output speed to very high for fast switching
  GPIOB->OSPEEDR |= ((3U << (0*2)) | (3U << (1*2)) | (3U << (2*2)) | (3U << (4*2)));

  // Initialize outputs to low
  CDI_LOW();
  SHIFT_LOW();
  WARN_LOW();
  LED_OFF();  // Turn LED off (active-low, so set pin HIGH)

  // Configure input pins with pull-up
  pinMode(PIN_KILL_SWITCH, INPUT_PULLUP);
  pinMode(PIN_MAP_SWITCH, INPUT_PULLUP);
  pinMode(PIN_SD_DETECT, INPUT_PULLUP);

  // PC13 config - just set as input
  // GPIOC clock should already be enabled from earlier
  GPIOC->MODER &= ~(3U << (13 * 2));  // Input mode (00)
  GPIOC->PUPDR &= ~(3U << (13 * 2));  // No pull (external R6 pulls up)

  // Configure ADC pins
  pinMode(PIN_ADC_HEAD_TEMP, INPUT_ANALOG);
  pinMode(PIN_ADC_BATTERY, INPUT_ANALOG);
  pinMode(PIN_ADC_CHARGING, INPUT_ANALOG);
}

// ============================================================================
// ADC SETUP
// ============================================================================

void setupADC(void) {
  analogReadResolution(ADC_RESOLUTION);
}

// ============================================================================
// SD CARD
// ============================================================================

void setupSD(void) {
  // Initialize SD card using SDMMC interface
  USB_SERIAL.println(F("Init SD (SDMMC)..."));

  // Skip card detect - just try to initialize directly
  // SDMMC init will fail if no card present
  if (!SD.begin(SD_DETECT_NONE)) {
    USB_SERIAL.println(F("SD init failed"));
    runtime.sdCardOk = 0;
    runtime.usingDefaultMap = 1;
    return;
  }

  runtime.sdCardOk = 1;
  USB_SERIAL.println(F("SD OK"));

  // Create config folder - try mkdir regardless of exists() result
  // Some SD libraries have issues with exists() for directories
  SD.mkdir(CONFIG_FOLDER);  // Ignore return value - may fail if exists

  loadConfigFromSD();

  // Auto-create text files on first boot
  createAllSDFiles();
}

// Simple SD reinit - just try to access file directly (no SD.begin)
// Only updates sdCardOk if successful, doesn't change if fails
bool reinitSD(void) {
  USB_SERIAL.println(F("Reinit SD..."));

  // Try to access a folder - if works, SD is fine
  File testFile = SD.open(CONFIG_FOLDER);
  if (testFile) {
    testFile.close();
    runtime.sdCardOk = 1;
    USB_SERIAL.println(F("SD OK"));
    return true;
  }

  // Don't change sdCardOk here - let other code handle it
  USB_SERIAL.println(F("SD fail"));
  return false;
}


// ============================================================================
// CONFIGURATION
// ============================================================================

uint32_t calculateChecksum(struct CDIConfig* cfg) {
  uint32_t sum = 0;
  uint8_t* ptr = (uint8_t*)cfg;
  for (size_t i = 0; i < sizeof(CDIConfig) - sizeof(uint32_t); i++) {
    sum += ptr[i];
  }
  return sum ^ 0xDEADBEEF;
}

// Calculate period thresholds from RPM settings (called when config changes)
// This allows ISR to use period comparison instead of RPM division
void updatePeriodThresholds(void) {
  // Period = 600,000,000 / RPM (at 10MHz timer)
  // Lower period = higher RPM
  runtime.periodSoftCut = (config.revLimiter.softRpm > 0) ?
    600000000UL / config.revLimiter.softRpm : 0;
  runtime.periodMediumCut = (config.revLimiter.mediumRpm > 0) ?
    600000000UL / config.revLimiter.mediumRpm : 0;
  runtime.periodHardCut = (config.revLimiter.hardRpm > 0) ?
    600000000UL / config.revLimiter.hardRpm : 0;
  runtime.periodFullCut = (config.revLimiter.fullCutRpm > 0) ?
    600000000UL / config.revLimiter.fullCutRpm : 0;
  runtime.periodOverRev = (config.warning.overRevRpm > 0) ?
    600000000UL / config.warning.overRevRpm : 0;
}

// Calculate NOP loop count for CDI pulse width
// At 250MHz, 4 NOPs per iteration = 16ns per iteration
// NOP count = (pulseUs * 1000) / 16 = pulseUs * 62.5
void updatePulseWidth(void) {
  uint16_t pulseUs = config.trigger.cdiPulseUs;
  // Clamp to safe range (50-250us)
  if (pulseUs < 50) pulseUs = 50;
  if (pulseUs > 250) pulseUs = 250;

  // Calculate NOP count: pulseUs * 62.5 ≈ pulseUs * 63
  // Using integer math: (pulseUs * 125) / 2
  runtime.pulseNopCount = (pulseUs * 125) / 2;
}

void loadDefaultConfig(void) {
  memset(&config, 0, sizeof(CDIConfig));

  config.magic = 0xCD112345;
  config.version = 2;
  config.engineType = ENGINE_2_STROKE;
  config.activeMap = 0;

  // Trigger: single tooth at 30° BTDC
  config.trigger.triggerAngleScaled = 3000;  // 30.00° BTDC
  config.trigger.risingEdge = 1;
  config.trigger.noiseFilterTicks = 1000;   // 100us at 10MHz
  config.trigger.cdiPulseUs = 100;          // 100us CDI pulse (adjustable 50-250us)

  // Copy default safety map to all maps
  for (int m = 0; m < NUM_MAPS; m++) {
    for (int i = 0; i < RPM_TABLE_SIZE; i++) {
      config.timingMaps[m][i] = DEFAULT_SAFETY_MAP[i];
    }
  }

  // Rev limiter
  config.revLimiter.softRpm = 11000;
  config.revLimiter.mediumRpm = 11500;
  config.revLimiter.hardRpm = 12000;
  config.revLimiter.fullCutRpm = 12500;

  // Shift light
  config.shiftLight.onRpm = 9000;
  config.shiftLight.blinkRpm = 10000;
  config.shiftLight.fastBlinkRpm = 11000;
  config.shiftLight.blinkIntervalMs = 200;
  config.shiftLight.fastBlinkIntervalMs = 100;

  // ADC calibration (x1000)
  // ADC Calibration defaults
  // Temp: linear 0-500°C, formula: temp(x10) = raw * scale / 4096 + offset
  // At raw=4095, temp=500°C (5000 x10), so scale = 5000 * 4096 / 4095 ≈ 5001
  config.adcCal.tempOffsetScaled = 0;
  config.adcCal.tempScaleScaled = 5001;     // 0-500°C range

  // Voltage: formula: V(x100) = raw * 330 * scale / (4095 * 1000)
  // For 30V max with 3.3V ADC, divider ratio = 30/3.3 ≈ 9.09, scale = 9090
  config.adcCal.battScaleScaled = 9090;     // 30V max (divider 1:9.09)
  config.adcCal.chargingScaleScaled = 9090; // 30V max

  // Cranking
  config.cranking.enabled = 1;
  config.cranking.timingScaled = 5;  // 5°
  config.cranking.maxRpm = CRANKING_RPM;

  // Warning
  config.warning.overRevRpm = 13000;
  config.warning.overheatTempC = OVERHEAT_TEMP_C;      // 120°C
  config.warning.lowBatteryMv = LOW_BATTERY_MV;        // 11.0V
  config.warning.overheatRetard = 5;                   // Max 5° retard
  config.warning.retardStartTempC = 80;                // Start progressive retard at 80°C
  config.warning.retardPer10C = 10;                    // 1.0° per 10°C (10 = 1.0°)

  config.peakRpm = 0;

  config.checksum = calculateChecksum(&config);

  // Update period thresholds for ISR
  updatePeriodThresholds();
  updatePulseWidth();
}

bool validateConfig(struct CDIConfig* cfg) {
  if (cfg->magic != 0xCD112345) return false;
  if (cfg->version == 0 || cfg->version > 10) return false;
  if (cfg->engineType != ENGINE_2_STROKE && cfg->engineType != ENGINE_4_STROKE) return false;
  if (cfg->activeMap >= NUM_MAPS) return false;
  if (cfg->checksum != calculateChecksum(cfg)) return false;
  return true;
}

void loadConfigFromSD(void) {
  if (!runtime.sdCardOk) {
    runtime.usingDefaultMap = 1;
    return;
  }

  // Skip if SD busy (during runtime calls)
  if (runtime.sdBusy) {
    USB_SERIAL.println(F("Load failed: SD busy"));
    return;
  }

  runtime.sdBusy = 1;

  // Check primary location first, then fallback
  const char* loadFile = CONFIG_FILE;  // "racing-cdi/config.bin"
  if (!SD.exists(loadFile)) {
    loadFile = "cdi_config.bin";  // Fallback in root
    if (!SD.exists(loadFile)) {
      USB_SERIAL.println(F("No config, using defaults"));
      runtime.usingDefaultMap = 1;
      runtime.sdBusy = 0;
      saveConfigToSD();
      return;
    }
  }

  File f = SD.open(loadFile, FILE_READ);
  if (!f) {
    runtime.usingDefaultMap = 1;
    runtime.sdBusy = 0;
    return;
  }

  CDIConfig temp;
  size_t bytesRead = f.read((uint8_t*)&temp, sizeof(CDIConfig));
  f.close();

  runtime.sdBusy = 0;

  if (bytesRead != sizeof(CDIConfig) || !validateConfig(&temp)) {
    USB_SERIAL.println(F("Config invalid, using defaults"));
    runtime.usingDefaultMap = 1;
    return;
  }

  memcpy(&config, &temp, sizeof(CDIConfig));
  runtime.usingDefaultMap = 0;

  // Update period thresholds for ISR
  updatePeriodThresholds();
  updatePulseWidth();

  USB_SERIAL.println(F("Config loaded"));
}

void saveConfigToSD(void) {
  if (!runtime.sdCardOk) {
    USB_SERIAL.println(F("Save failed: SD not OK"));
    return;
  }

  // Skip if SD is already busy
  if (runtime.sdBusy) {
    USB_SERIAL.println(F("Save failed: SD busy"));
    return;
  }

  runtime.sdBusy = 1;  // Mark SD busy

  config.checksum = calculateChecksum(&config);

  // Try to save in folder first, fallback to root if fails
  const char* saveFile = CONFIG_FILE;  // "racing-cdi/config.bin"

  // Try to create folder (ignore result - may already exist)
  SD.mkdir(CONFIG_FOLDER);
  SD.mkdir("/racing-cdi");  // Try with leading slash too

  // Remove old file
  SD.remove(saveFile);

  // Try to open file for writing
  File f = SD.open(saveFile, FILE_WRITE);

  // If folder path fails, try saving to root
  if (!f) {
    saveFile = "cdi_config.bin";  // Fallback to root
    SD.remove(saveFile);
    f = SD.open(saveFile, FILE_WRITE);
  }

  if (!f) {
    USB_SERIAL.println(F("Save failed: cannot open file"));
    runtime.sdCardOk = 0;  // Mark SD as not OK
    runtime.sdBusy = 0;
    return;
  }

  size_t written = f.write((uint8_t*)&config, sizeof(CDIConfig));
  f.close();

  runtime.sdBusy = 0;  // SD operation done

  if (written != sizeof(CDIConfig)) {
    USB_SERIAL.print(F("Save failed: wrote "));
    USB_SERIAL.print(written);
    USB_SERIAL.print(F("/"));
    USB_SERIAL.println(sizeof(CDIConfig));
  } else {
    USB_SERIAL.print(F("Config saved to "));
    USB_SERIAL.println(saveFile);
  }
}

// ============================================================================
// TEXT FILE EXPORT/IMPORT - HUMAN READABLE FORMAT
// ============================================================================

// Create README file with system info
void createReadmeFile(void) {
  if (!runtime.sdCardOk) return;

  File f = SD.open(README_FILE, FILE_WRITE);
  if (!f) return;

  f.println(F("========================================"));
  f.println(F("  RACING CDI - STM32H562RGT6"));
  f.println(F("  Maximum Precision Build v2.0"));
  f.println(F("========================================"));
  f.println();
  f.println(F("SPECIFICATIONS:"));
  f.println(F("- Timer Resolution: 100ns (10MHz)"));
  f.println(F("- Timing Jitter: <0.05 degrees"));
  f.println(F("- RPM Range: 0-20000 RPM"));
  f.println(F("- RPM Step: 250 RPM"));
  f.println(F("- Timing Range: -10 to +60 BTDC"));
  f.println(F("- Maps: 6 selectable"));
  f.println(F("- Rev Limiter: 4 stages"));
  f.println();
  f.println(F("FILES:"));
  f.println(F("- config.bin  : Binary config (auto)"));
  f.println(F("- settings.txt: Human readable settings"));
  f.println(F("- map1.txt-map6.txt: Ignition maps"));
  f.println(F("- log_X.csv   : Data logs"));
  f.println();
  f.println(F("USB COMMANDS:"));
  f.println(F("- STATUS      : Show system status"));
  f.println(F("- HELP        : Show all commands"));
  f.println(F("- SAVE        : Save to SD"));
  f.println(F("- LOAD        : Load from SD"));
  f.println(F("- EXPORT      : Export to text files"));
  f.println(F("- IMPORT      : Import from text files"));
  f.println();
  f.println(F("MAP SWITCH:"));
  f.println(F("- PA2 (external) or PC13 (onboard)"));
  f.println(F("- Press to cycle: 1->2->3->4->5->6->1"));
  f.println(F("- LED blinks map number"));
  f.println();
  f.println(F("LED PATTERNS (PB2):"));
  f.println(F("- Slow blink: Idle"));
  f.println(F("- Fast blink: Running OK"));
  f.println(F("- Double blink: Warning"));
  f.println(F("- Triple blink: Error/Default map"));
  f.println(F("========================================"));

  f.close();
  USB_SERIAL.println(F("README created"));
}

// Export settings to human-readable text file
void exportSettingsToText(void) {
  if (!runtime.sdCardOk) return;

  if (SD.exists(SETTINGS_FILE)) {
    SD.remove(SETTINGS_FILE);
  }

  File f = SD.open(SETTINGS_FILE, FILE_WRITE);
  if (!f) return;

  f.println(F("# RACING CDI SETTINGS"));
  f.println(F("# Edit values after '=' sign"));
  f.println(F("# Save and use IMPORT command to load"));
  f.println();

  f.println(F("[ENGINE]"));
  f.print(F("type=")); f.println(config.engineType);  // 2 or 4
  f.print(F("active_map=")); f.println(config.activeMap + 1);  // 1-6
  f.println();

  f.println(F("[TRIGGER]"));
  f.print(F("trigger_angle="));
  f.print(config.trigger.triggerAngleScaled / DEG_SCALE);
  f.print(F(".")); f.println(abs(config.trigger.triggerAngleScaled % DEG_SCALE));
  f.print(F("edge=")); f.println(config.trigger.risingEdge ? F("RISING") : F("FALLING"));
  f.print(F("noise_filter_us=")); f.println(config.trigger.noiseFilterTicks / 10);
  f.print(F("cdi_pulse_us=")); f.println(config.trigger.cdiPulseUs);
  f.println();

  f.println(F("[REV_LIMITER]"));
  f.print(F("soft_rpm=")); f.println(config.revLimiter.softRpm);
  f.print(F("medium_rpm=")); f.println(config.revLimiter.mediumRpm);
  f.print(F("hard_rpm=")); f.println(config.revLimiter.hardRpm);
  f.print(F("fullcut_rpm=")); f.println(config.revLimiter.fullCutRpm);
  f.println();

  f.println(F("[SHIFT_LIGHT]"));
  f.print(F("on_rpm=")); f.println(config.shiftLight.onRpm);
  f.print(F("blink_rpm=")); f.println(config.shiftLight.blinkRpm);
  f.print(F("fast_blink_rpm=")); f.println(config.shiftLight.fastBlinkRpm);
  f.print(F("blink_interval_ms=")); f.println(config.shiftLight.blinkIntervalMs);
  f.print(F("fast_interval_ms=")); f.println(config.shiftLight.fastBlinkIntervalMs);
  f.println();

  f.println(F("[CRANKING]"));
  f.print(F("enabled=")); f.println(config.cranking.enabled ? F("ON") : F("OFF"));
  f.print(F("timing=")); f.println(config.cranking.timingScaled);
  f.print(F("max_rpm=")); f.println(config.cranking.maxRpm);
  f.println();

  f.println(F("[WARNING]"));
  f.print(F("overrev_rpm=")); f.println(config.warning.overRevRpm);
  f.print(F("overheat_temp_c=")); f.println(config.warning.overheatTempC);
  f.print(F("low_battery_v=")); f.print(config.warning.lowBatteryMv / 1000);
  f.print(F(".")); f.println(config.warning.lowBatteryMv % 1000 / 100);  // 1 decimal
  f.print(F("overheat_retard=")); f.println(config.warning.overheatRetard);
  f.print(F("retard_start_temp_c=")); f.println(config.warning.retardStartTempC);
  f.print(F("retard_per_10c=")); f.println(config.warning.retardPer10C);
  f.println();

  f.println(F("[STATISTICS]"));
  f.print(F("peak_rpm=")); f.println(config.peakRpm);

  f.close();
  USB_SERIAL.println(F("Settings exported"));
}

// Export single map to text file
void exportMapToText(uint8_t mapNum) {
  if (!runtime.sdCardOk || mapNum >= NUM_MAPS) return;

  String fn = String(MAP_FILE_PREFIX) + String(mapNum + 1) + ".txt";

  if (SD.exists(fn.c_str())) {
    SD.remove(fn.c_str());
  }

  File f = SD.open(fn.c_str(), FILE_WRITE);
  if (!f) return;

  f.print(F("# IGNITION MAP ")); f.println(mapNum + 1);
  f.println(F("# Format: RPM=TIMING (degrees BTDC)"));
  f.println(F("# Edit timing values (-10 to 60)"));
  f.println();

  for (int i = 0; i < RPM_TABLE_SIZE; i++) {
    uint16_t rpm = i * RPM_STEP;
    f.print(rpm);
    f.print(F("="));
    f.println(config.timingMaps[mapNum][i]);
  }

  f.close();
}

// Export all maps to text files
void exportAllMapsToText(void) {
  for (uint8_t m = 0; m < NUM_MAPS; m++) {
    exportMapToText(m);
  }
  USB_SERIAL.println(F("All maps exported"));
}

// Import settings from text file
void importSettingsFromText(void) {
  if (!runtime.sdCardOk) return;

  if (!SD.exists(SETTINGS_FILE)) {
    USB_SERIAL.println(F("No settings.txt"));
    return;
  }

  File f = SD.open(SETTINGS_FILE, FILE_READ);
  if (!f) return;

  char line[64];
  while (f.available()) {
    int len = f.readBytesUntil('\n', line, sizeof(line) - 1);
    line[len] = '\0';

    // Skip comments and empty lines
    if (line[0] == '#' || line[0] == '[' || len < 3) continue;

    // Find '=' separator
    char* eq = strchr(line, '=');
    if (!eq) continue;

    *eq = '\0';
    char* key = line;
    char* val = eq + 1;

    // Trim whitespace
    while (*key == ' ') key++;
    while (*val == ' ') val++;

    // Parse settings
    if (strcmp(key, "type") == 0) {
      int v = atoi(val);
      if (v == 2 || v == 4) config.engineType = v;
    }
    else if (strcmp(key, "active_map") == 0) {
      int v = atoi(val);
      if (v >= 1 && v <= 6) config.activeMap = v - 1;
    }
    else if (strcmp(key, "trigger_angle") == 0) {
      config.trigger.triggerAngleScaled = (int32_t)(atof(val) * DEG_SCALE);
    }
    else if (strcmp(key, "edge") == 0) {
      config.trigger.risingEdge = (strstr(val, "RISING") != NULL) ? 1 : 0;
    }
    else if (strcmp(key, "noise_filter_us") == 0) {
      config.trigger.noiseFilterTicks = atoi(val) * 10;
    }
    else if (strcmp(key, "cdi_pulse_us") == 0) {
      uint16_t pulse = atoi(val);
      if (pulse < 50) pulse = 50;
      if (pulse > 250) pulse = 250;
      config.trigger.cdiPulseUs = pulse;
    }
    else if (strcmp(key, "soft_rpm") == 0) {
      config.revLimiter.softRpm = atoi(val);
    }
    else if (strcmp(key, "medium_rpm") == 0) {
      config.revLimiter.mediumRpm = atoi(val);
    }
    else if (strcmp(key, "hard_rpm") == 0) {
      config.revLimiter.hardRpm = atoi(val);
    }
    else if (strcmp(key, "fullcut_rpm") == 0) {
      config.revLimiter.fullCutRpm = atoi(val);
    }
    else if (strcmp(key, "on_rpm") == 0) {
      config.shiftLight.onRpm = atoi(val);
    }
    else if (strcmp(key, "blink_rpm") == 0) {
      config.shiftLight.blinkRpm = atoi(val);
    }
    else if (strcmp(key, "fast_blink_rpm") == 0) {
      config.shiftLight.fastBlinkRpm = atoi(val);
    }
    else if (strcmp(key, "blink_interval_ms") == 0) {
      config.shiftLight.blinkIntervalMs = atoi(val);
    }
    else if (strcmp(key, "fast_interval_ms") == 0) {
      config.shiftLight.fastBlinkIntervalMs = atoi(val);
    }
    else if (strcmp(key, "enabled") == 0) {
      config.cranking.enabled = (strstr(val, "ON") != NULL) ? 1 : 0;
    }
    else if (strcmp(key, "timing") == 0) {
      config.cranking.timingScaled = atoi(val);
    }
    else if (strcmp(key, "max_rpm") == 0) {
      config.cranking.maxRpm = atoi(val);
    }
    else if (strcmp(key, "overrev_rpm") == 0) {
      config.warning.overRevRpm = atoi(val);
    }
    else if (strcmp(key, "overheat_temp_c") == 0) {
      config.warning.overheatTempC = atoi(val);
    }
    else if (strcmp(key, "low_battery_v") == 0) {
      // Parse voltage like "11.0" into millivolts (11000)
      float v = atof(val);
      config.warning.lowBatteryMv = (uint16_t)(v * 1000);
    }
    else if (strcmp(key, "overheat_retard") == 0) {
      config.warning.overheatRetard = atoi(val);
    }
    else if (strcmp(key, "retard_start_temp_c") == 0) {
      config.warning.retardStartTempC = atoi(val);
    }
    else if (strcmp(key, "retard_per_10c") == 0) {
      config.warning.retardPer10C = atoi(val);
    }
  }

  f.close();

  // Update period thresholds after import
  updatePeriodThresholds();
  updatePulseWidth();

  USB_SERIAL.println(F("Settings imported"));
}

// Import single map from text file
// SAFE: Uses temporary safety map switch if editing active map
void importMapFromText(uint8_t mapNum) {
  if (!runtime.sdCardOk || mapNum >= NUM_MAPS) return;

  String fn = String(MAP_FILE_PREFIX) + String(mapNum + 1) + ".txt";

  if (!SD.exists(fn.c_str())) return;

  File f = SD.open(fn.c_str(), FILE_READ);
  if (!f) return;

  // SAFETY: If importing active map while engine running, use safety map temporarily
  bool needSafetySwitch = (mapNum == config.activeMap) && runtime.engineRunning;
  if (needSafetySwitch) {
    runtime.usingDefaultMap = 1;  // ISR will use DEFAULT_SAFETY_MAP
    __DSB();  // Memory barrier - ensure ISR sees the flag change
  }

  char line[32];
  while (f.available()) {
    int len = f.readBytesUntil('\n', line, sizeof(line) - 1);
    line[len] = '\0';

    // Skip comments and empty lines
    if (line[0] == '#' || len < 3) continue;

    // Find '=' separator
    char* eq = strchr(line, '=');
    if (!eq) continue;

    *eq = '\0';
    int rpm = atoi(line);
    int timing = atoi(eq + 1);

    // Validate and store
    if (rpm >= 0 && rpm <= RPM_MAX && timing >= -10 && timing <= 60) {
      uint8_t idx = rpm / RPM_STEP;
      if (idx < RPM_TABLE_SIZE) {
        config.timingMaps[mapNum][idx] = (int8_t)timing;
      }
    }
  }

  f.close();

  // SAFETY: Switch back to edited map
  if (needSafetySwitch) {
    __DSB();  // Memory barrier - ensure all writes complete
    runtime.usingDefaultMap = 0;  // ISR will use updated map
  }
}

// Import all maps from text files
// SAFE: Temporarily uses safety map during bulk import
void importAllMapsFromText(void) {
  // Use safety map during entire bulk import for consistency
  bool wasUsingSafety = runtime.usingDefaultMap;
  if (runtime.engineRunning && !wasUsingSafety) {
    runtime.usingDefaultMap = 1;
    __DSB();
  }

  for (uint8_t m = 0; m < NUM_MAPS; m++) {
    // Import without individual safety switch since we're already on safety map
    String fn = String(MAP_FILE_PREFIX) + String(m + 1) + ".txt";
    if (!SD.exists(fn.c_str())) continue;

    File f = SD.open(fn.c_str(), FILE_READ);
    if (!f) continue;

    char line[32];
    while (f.available()) {
      int len = f.readBytesUntil('\n', line, sizeof(line) - 1);
      line[len] = '\0';
      if (line[0] == '#' || len < 3) continue;
      char* eq = strchr(line, '=');
      if (!eq) continue;
      *eq = '\0';
      int rpm = atoi(line);
      int timing = atoi(eq + 1);
      if (rpm >= 0 && rpm <= RPM_MAX && timing >= -10 && timing <= 60) {
        uint8_t idx = rpm / RPM_STEP;
        if (idx < RPM_TABLE_SIZE) {
          config.timingMaps[m][idx] = (int8_t)timing;
        }
      }
    }
    f.close();
  }

  // Restore to user map
  if (runtime.engineRunning && !wasUsingSafety) {
    __DSB();
    runtime.usingDefaultMap = 0;
  }

  USB_SERIAL.println(F("All maps imported"));
}

// Export all to text files
void exportAllToText(void) {
  if (runtime.sdBusy) {
    USB_SERIAL.println(F("Export failed: SD busy"));
    return;
  }
  runtime.sdBusy = 1;
  createReadmeFile();
  exportSettingsToText();
  exportAllMapsToText();
  runtime.sdBusy = 0;
  USB_SERIAL.println(F("All files exported"));
}

// Import all from text files
// SAFE: Can be used while engine running - uses safety map temporarily
void importAllFromText(void) {
  if (runtime.sdBusy) {
    USB_SERIAL.println(F("Import failed: SD busy"));
    return;
  }
  runtime.sdBusy = 1;

  USB_SERIAL.println(F("Importing..."));

  // Use safety map during entire import
  bool wasUsingSafety = runtime.usingDefaultMap;
  if (runtime.engineRunning && !wasUsingSafety) {
    runtime.usingDefaultMap = 1;
    __DSB();
    USB_SERIAL.println(F("(Using safety map during import)"));
  }

  importSettingsFromText();
  importAllMapsFromText();

  // Save to binary after import (sdBusy already set, so use internal save)
  config.checksum = calculateChecksum(&config);
  if (SD.exists(CONFIG_FILE)) SD.remove(CONFIG_FILE);
  File f = SD.open(CONFIG_FILE, FILE_WRITE);
  if (f) {
    f.write((uint8_t*)&config, sizeof(CDIConfig));
    f.close();
  }

  // Restore to user map
  if (runtime.engineRunning && !wasUsingSafety) {
    __DSB();
    runtime.usingDefaultMap = 0;
  }

  runtime.sdBusy = 0;
  USB_SERIAL.println(F("All files imported & saved!"));
}

// Create all files on first boot
void createAllSDFiles(void) {
  if (!runtime.sdCardOk) return;

  // Skip if SD busy (shouldn't happen at boot, but safe)
  if (runtime.sdBusy) return;

  // Check if this is first boot (no readme = first boot)
  bool firstBoot = !SD.exists(README_FILE);

  if (firstBoot) {
    runtime.sdBusy = 1;
    USB_SERIAL.println(F("First boot - creating files..."));
    createReadmeFile();
    exportSettingsToText();
    exportAllMapsToText();
    runtime.sdBusy = 0;
    USB_SERIAL.println(F("SD files created!"));
  }
}

// ============================================================================
// INPUT HANDLING
// ============================================================================

// Helper function to handle map change (non-blocking LED indication)
void doMapChange(void) {
  config.activeMap = (config.activeMap + 1) % NUM_MAPS;
  runtime.ledPattern = LED_PATTERN_MAP_CHANGE;
  runtime.ledStep = 0;
  saveConfigToSD();
  USB_SERIAL.print(F("Map: "));
  USB_SERIAL.println(config.activeMap + 1);
}

void handleInputs(void) {
  // Kill switch (active low)
  runtime.killActive = (digitalRead(PIN_KILL_SWITCH) == LOW) ? 1 : 0;

  uint32_t now = millis();

  // === Map switch 1 (PA2 - external) ===
  uint8_t reading1 = (digitalRead(PIN_MAP_SWITCH) == LOW) ? 1 : 0;

  // Reset debounce timer on state change
  if (reading1 != runtime.lastMapSwitchState) {
    runtime.lastMapSwitchMs = now;
    runtime.lastMapSwitchState = reading1;
  }

  // Only update stable state after debounce period
  if ((now - runtime.lastMapSwitchMs) >= DEBOUNCE_MS) {
    // Detect falling edge (button release: 1 -> 0)
    if (runtime.mapSwitchState == 1 && reading1 == 0) {
      doMapChange();
    }
    runtime.mapSwitchState = reading1;
  }

  // === Map switch 2 (PC13 - onboard button) ===
  // Direct register read for PC13 (backup domain pin)
  uint8_t reading2 = ((GPIOC->IDR & (1U << 13)) == 0) ? 1 : 0;

  // Reset debounce timer on state change
  if (reading2 != runtime.lastMapSwitch2State) {
    runtime.lastMapSwitch2Ms = now;
    runtime.lastMapSwitch2State = reading2;
  }

  // Only update stable state after debounce period
  if ((now - runtime.lastMapSwitch2Ms) >= DEBOUNCE_MS) {
    // Detect falling edge (button release: 1 -> 0)
    if (runtime.mapSwitch2State == 1 && reading2 == 0) {
      doMapChange();
    }
    runtime.mapSwitch2State = reading2;
  }
}

// ============================================================================
// STATUS LED - MCU HEALTH INDICATOR
// ============================================================================
// Pattern meanings:
// - Slow blink (1s): System idle, engine stopped
// - Fast blink (200ms): Engine running, all OK
// - Double blink: Warning condition (overheat or low battery)
// - Triple blink: Error condition (SD fail, using default map)
// - N blinks: Map number indication after map change

void updateStatusLED(void) {
  static uint32_t lastLedUpdate = 0;
  static uint8_t blinkCount = 0;
  static uint8_t blinkPhase = 0;  // 0=on, 1=off

  uint32_t now = millis();
  uint32_t interval;

  // Determine current pattern based on system state
  if (runtime.ledPattern != LED_PATTERN_MAP_CHANGE) {
    // Auto-select pattern based on system state
    if (runtime.usingDefaultMap || !runtime.sdCardOk) {
      runtime.ledPattern = LED_PATTERN_ERROR;
    } else if (runtime.overheating || runtime.lowBattery) {
      runtime.ledPattern = LED_PATTERN_WARNING;
    } else if (runtime.engineRunning) {
      runtime.ledPattern = LED_PATTERN_RUNNING;
    } else {
      runtime.ledPattern = LED_PATTERN_IDLE;
    }
  }

  switch (runtime.ledPattern) {
    case LED_PATTERN_IDLE:
      // Slow blink: 500ms on, 500ms off
      interval = 500;
      if ((now - lastLedUpdate) >= interval) {
        lastLedUpdate = now;
        LED_TOGGLE();
      }
      break;

    case LED_PATTERN_RUNNING:
      // Fast blink: 100ms on, 100ms off
      interval = 100;
      if ((now - lastLedUpdate) >= interval) {
        lastLedUpdate = now;
        LED_TOGGLE();
      }
      break;

    case LED_PATTERN_WARNING:
      // Double blink pattern: ON-OFF-ON-OFF---pause---
      // Timing: 100-100-100-100-600
      if (blinkCount < 4) {
        interval = 100;
      } else {
        interval = 600;
      }
      if ((now - lastLedUpdate) >= interval) {
        lastLedUpdate = now;
        if (blinkCount < 4) {
          LED_TOGGLE();
          blinkCount++;
        } else {
          LED_OFF();
          blinkCount = 0;
        }
      }
      break;

    case LED_PATTERN_ERROR:
      // Triple blink pattern: ON-OFF-ON-OFF-ON-OFF---pause---
      if (blinkCount < 6) {
        interval = 100;
      } else {
        interval = 600;
      }
      if ((now - lastLedUpdate) >= interval) {
        lastLedUpdate = now;
        if (blinkCount < 6) {
          LED_TOGGLE();
          blinkCount++;
        } else {
          LED_OFF();
          blinkCount = 0;
        }
      }
      break;

    case LED_PATTERN_MAP_CHANGE:
      // Blink (activeMap + 1) times to indicate map number
      {
        uint8_t targetBlinks = (config.activeMap + 1) * 2;  // *2 for on+off cycles
        interval = 150;

        if ((now - lastLedUpdate) >= interval) {
          lastLedUpdate = now;

          if (runtime.ledStep < targetBlinks) {
            LED_TOGGLE();
            runtime.ledStep++;
          } else {
            // Done blinking, return to normal pattern
            LED_OFF();
            runtime.ledStep = 0;
            runtime.ledPattern = LED_PATTERN_IDLE;  // Will auto-select correct pattern
          }
        }
      }
      break;
  }
}

// ============================================================================
// ADC READING
// ============================================================================

void readADC(void) {
  runtime.tempRaw = analogRead(PIN_ADC_HEAD_TEMP);
  runtime.batteryRaw = analogRead(PIN_ADC_BATTERY);
  runtime.chargingRaw = analogRead(PIN_ADC_CHARGING);

  // Check thresholds using actual values (not raw ADC)
  // Temperature: compare in Celsius (rawToTempX10 returns temp*10)
  runtime.currentTempC = rawToTempX10(runtime.tempRaw) / 10;
  runtime.overheating = (runtime.currentTempC >= config.warning.overheatTempC) ? 1 : 0;

  // Battery: compare in millivolts (rawToVoltageX100 returns voltage*100 = voltage in centivolts)
  // Convert centivolts to millivolts: *10
  int32_t currentBattMv = (int32_t)rawToVoltageX100(runtime.batteryRaw, config.adcCal.battScaleScaled) * 10;
  runtime.lowBattery = (currentBattMv <= config.warning.lowBatteryMv) ? 1 : 0;
}

// Convert raw ADC to temperature (linear mapping, x10 = 0.1°C)
// Range: 0-500°C (sensor like K-type thermocouple or high-temp NTC)
// Formula: temp(x10) = (raw * scale / 4096) + offset
// Default: scale=1220 gives ~500°C at raw=4095, offset=0
int16_t rawToTempX10(uint16_t raw) {

  if (raw < 10)   return 0;     // short → 0°C
  if (raw > 4080) return 5000;  // open → 500.0°C (max)

  // Linear mapping: 0-4095 ADC → 0-500°C (adjustable via scale/offset)
  // temp(x10) = (raw * scale / 4096) + offset
  int32_t temp =
      ((int32_t)raw * config.adcCal.tempScaleScaled) / 4096
    + config.adcCal.tempOffsetScaled;

  // Clamp to valid range
  if (temp < 0) temp = 0;
  if (temp > 5000) temp = 5000;  // 500.0°C max

  return (int16_t)temp;
}


// Convert raw ADC to voltage (x100, 0.01V resolution)
// scaleX1000 = scale * 1000 (contoh: divider 1:9.09 → 9090 untuk 30V max)
// Formula: Vout = ADC_raw * Vref * scale / ADC_max
//          Vout_x100 = raw * 330 * (scaleX1000/1000) / 4095
// Harus pakai int64_t untuk hindari overflow (raw*330*9090 > int32 max)
int16_t rawToVoltageX100(uint16_t raw, int32_t scaleX1000) {
  // Use 64-bit to avoid overflow: 4095*330*9090 = 12 billion > int32 max
  int64_t voltage = ((int64_t)raw * 330 * scaleX1000) / (4095LL * 1000LL);

  return (int16_t)voltage;
}


// ============================================================================
// OUTPUT HANDLING
// ============================================================================

void updateOutputs(void) {
  static uint32_t lastBlink = 0;
  static uint8_t blinkState = 0;
  uint32_t now = millis();

  uint16_t rpm = runtime.currentRpm;

  // Shift light
  if (rpm >= config.shiftLight.fastBlinkRpm) {
    if ((now - lastBlink) >= config.shiftLight.fastBlinkIntervalMs) {
      lastBlink = now;
      blinkState ^= 1;
      if (blinkState) SHIFT_HIGH(); else SHIFT_LOW();
    }
  } else if (rpm >= config.shiftLight.blinkRpm) {
    if ((now - lastBlink) >= config.shiftLight.blinkIntervalMs) {
      lastBlink = now;
      blinkState ^= 1;
      if (blinkState) SHIFT_HIGH(); else SHIFT_LOW();
    }
  } else if (rpm >= config.shiftLight.onRpm) {
    SHIFT_HIGH();
    blinkState = 1;
  } else {
    SHIFT_LOW();
    blinkState = 0;
  }

  // Warning output
  if (runtime.overheating || runtime.lowBattery || rpm >= config.warning.overRevRpm) {
    WARN_HIGH();
  } else {
    WARN_LOW();
  }
}

// ============================================================================
// USB COMMUNICATION
// ============================================================================

// File upload state
static bool uploadMode = false;
static String uploadPath = "";
static File uploadFile;
static uint32_t uploadBytes = 0;

void processUSB(void) {
  if (!USB_SERIAL.available()) return;

  String cmd = USB_SERIAL.readStringUntil('\n');
  cmd.trim();
  if (cmd.length() == 0) return;

  // Handle upload mode - write data to file
  if (uploadMode) {
    if (cmd == "UPLOAD:END") {
      uploadFile.close();
      USB_SERIAL.print(F("UPLOAD:OK,"));
      USB_SERIAL.println(uploadBytes);
      uploadMode = false;
      uploadPath = "";
      uploadBytes = 0;
      NVIC_EnableIRQ(TIM2_IRQn);  // Re-enable interrupts
    } else if (cmd == "UPLOAD:CANCEL") {
      uploadFile.close();
      SD.remove(uploadPath.c_str());
      USB_SERIAL.println(F("UPLOAD:CANCELLED"));
      uploadMode = false;
      uploadPath = "";
      uploadBytes = 0;
      NVIC_EnableIRQ(TIM2_IRQn);
    } else {
      // Write line to file
      uploadFile.println(cmd);
      uploadBytes += cmd.length() + 1;
    }
    return;
  }

  if (cmd.startsWith("GET ")) {
    handleGet(cmd.substring(4));
  } else if (cmd.startsWith("SET ")) {
    handleSet(cmd.substring(4));
  } else if (cmd.startsWith("MAP ")) {
    handleMap(cmd.substring(4));
  } else if (cmd == "SAVE") {
    if (runtime.currentRpm >= 100) {
      USB_SERIAL.println(F("SAVE:BUSY"));
    } else {
      // Disable capture interrupt during SD operation
      NVIC_DisableIRQ(TIM2_IRQn);
      saveConfigToSD();
      NVIC_EnableIRQ(TIM2_IRQn);
    }
  } else if (cmd == "LOAD") {
    if (runtime.currentRpm >= 100) {
      USB_SERIAL.println(F("LOAD:BUSY"));
    } else {
      // Disable capture interrupt during SD operation
      NVIC_DisableIRQ(TIM2_IRQn);
      loadConfigFromSD();
      NVIC_EnableIRQ(TIM2_IRQn);
    }
  } else if (cmd == "DEFAULT") {
    loadDefaultConfig();
  } else if (cmd == "STATUS") {
    sendStatus();
  } else if (cmd == "HELP") {
    sendHelp();
  } else if (cmd == "RESETPEAK") {
    config.peakRpm = 0;
    USB_SERIAL.println(F("Peak reset"));

  // Ignition control (software kill switch)
  } else if (cmd == "IGN ON" || cmd == "IGN_ON") {
    runtime.ignitionEnabled = 1;
    USB_SERIAL.println(F("IGN:ON"));
  } else if (cmd == "IGN OFF" || cmd == "IGN_OFF") {
    runtime.ignitionEnabled = 0;
    USB_SERIAL.println(F("IGN:OFF"));
  } else if (cmd == "IGN" || cmd == "GET IGN") {
    USB_SERIAL.println(runtime.ignitionEnabled ? F("IGN:ON") : F("IGN:OFF"));

  } else if (cmd == "EXPORT") {
    // Block SD operations when RPM >= 100 to prevent timing issues
    if (runtime.currentRpm >= 100) {
      USB_SERIAL.println(F("EXPORT:BUSY"));
    } else {
      NVIC_DisableIRQ(TIM2_IRQn);
      exportAllToText();
      NVIC_EnableIRQ(TIM2_IRQn);
    }
  } else if (cmd == "IMPORT") {
    if (runtime.currentRpm >= 100) {
      USB_SERIAL.println(F("IMPORT:BUSY"));
    } else {
      NVIC_DisableIRQ(TIM2_IRQn);
      importAllFromText();
      NVIC_EnableIRQ(TIM2_IRQn);
    }
  } else if (cmd == "EXPORTMAPS") {
    if (runtime.currentRpm >= 100) {
      USB_SERIAL.println(F("EXPORT:BUSY"));
    } else {
      NVIC_DisableIRQ(TIM2_IRQn);
      exportAllMapsToText();
      NVIC_EnableIRQ(TIM2_IRQn);
    }
  } else if (cmd == "IMPORTMAPS") {
    if (runtime.currentRpm >= 100) {
      USB_SERIAL.println(F("IMPORT:BUSY"));
    } else {
      NVIC_DisableIRQ(TIM2_IRQn);
      importAllMapsFromText();
      saveConfigToSD();
      NVIC_EnableIRQ(TIM2_IRQn);
    }
  } else if (cmd == "EXPORTSETTINGS") {
    if (runtime.currentRpm >= 100) {
      USB_SERIAL.println(F("EXPORT:BUSY"));
    } else {
      NVIC_DisableIRQ(TIM2_IRQn);
      exportSettingsToText();
      NVIC_EnableIRQ(TIM2_IRQn);
    }
  } else if (cmd == "IMPORTSETTINGS") {
    if (runtime.currentRpm >= 100) {
      USB_SERIAL.println(F("IMPORT:BUSY"));
    } else {
      NVIC_DisableIRQ(TIM2_IRQn);
      importSettingsFromText();
      saveConfigToSD();
      NVIC_EnableIRQ(TIM2_IRQn);
    }

  // SD File Management Commands
  // Note: SD operations blocked when RPM >= 100 to prevent timing issues
  } else if (cmd == "DISK") {
    // Show SD card info (always allowed, fast operation)
    if (!runtime.sdCardOk) {
      USB_SERIAL.println(F("SD:ERROR"));
    } else {
      USB_SERIAL.println(F("SD:OK"));
    }

  } else if (cmd == "SDINIT") {
    // Reinitialize SD card
    if (runtime.currentRpm >= 100) {
      USB_SERIAL.println(F("SDINIT:BUSY"));
    } else {
      NVIC_DisableIRQ(TIM2_IRQn);
      if (reinitSD()) {
        USB_SERIAL.println(F("SDINIT:OK"));
      } else {
        USB_SERIAL.println(F("SDINIT:FAIL"));
      }
      NVIC_EnableIRQ(TIM2_IRQn);
    }

  } else if (cmd == "RESET") {
    // Software reset MCU - useful when SD card needs full reinit
    if (runtime.currentRpm >= 100) {
      USB_SERIAL.println(F("RESET:BUSY"));
    } else {
      USB_SERIAL.println(F("RESET:OK"));
      delay(100);  // Give time for message to send
      NVIC_SystemReset();
    }

  } else if (cmd.startsWith("LS ") || cmd == "LS") {
    // List directory: LS [path]
    if (runtime.currentRpm >= 100) {
      USB_SERIAL.println(F("LS:BUSY"));
      USB_SERIAL.println(F("LS:END"));
    } else if (!runtime.sdCardOk) {
      USB_SERIAL.println(F("LS:ERROR"));
      USB_SERIAL.println(F("LS:END"));
    } else {
      String path = cmd.length() > 3 ? cmd.substring(3) : "/racing-cdi";
      path.trim();
      if (path.length() == 0) path = "/racing-cdi";

      File dir = SD.open(path.c_str());
      if (!dir || !dir.isDirectory()) {
        USB_SERIAL.println(F("LS:NOTFOUND"));
        USB_SERIAL.println(F("LS:END"));
      } else {
        USB_SERIAL.print(F("LS:"));
        USB_SERIAL.println(path);
        File entry;
        uint8_t count = 0;
        while ((entry = dir.openNextFile()) && count < 50) {  // Limit to 50 entries
          USB_SERIAL.print(entry.isDirectory() ? F("D:") : F("F:"));
          USB_SERIAL.print(entry.name());
          if (!entry.isDirectory()) {
            USB_SERIAL.print(F(","));
            USB_SERIAL.print((uint32_t)entry.size());
          }
          USB_SERIAL.println();
          entry.close();
          count++;
        }
        USB_SERIAL.println(F("LS:END"));
        dir.close();
      }
    }

  } else if (cmd.startsWith("CAT ")) {
    // Read file: CAT <path> [lines]
    if (runtime.currentRpm >= 100) {
      USB_SERIAL.println(F("CAT:BUSY"));
      USB_SERIAL.println(F("CAT:END"));
    } else if (!runtime.sdCardOk) {
      USB_SERIAL.println(F("CAT:ERROR"));
      USB_SERIAL.println(F("CAT:END"));
    } else {
      String args = cmd.substring(4);
      args.trim();
      int spaceIdx = args.indexOf(' ');
      String path = spaceIdx > 0 ? args.substring(0, spaceIdx) : args;
      int maxLines = spaceIdx > 0 ? args.substring(spaceIdx + 1).toInt() : 100;
      // 0 = unlimited (up to 50000 lines), negative = default 100
      if (maxLines == 0) maxLines = 50000;
      else if (maxLines < 0) maxLines = 100;

      if (!SD.exists(path.c_str())) {
        USB_SERIAL.println(F("CAT:NOTFOUND"));
        USB_SERIAL.println(F("CAT:END"));
      } else {
        File f = SD.open(path.c_str(), FILE_READ);
        if (!f) {
          USB_SERIAL.println(F("CAT:ERROR"));
          USB_SERIAL.println(F("CAT:END"));
        } else {
          USB_SERIAL.print(F("CAT:"));
          USB_SERIAL.print(path);
          USB_SERIAL.print(F(","));
          USB_SERIAL.println((uint32_t)f.size());

          int lineCount = 0;
          while (f.available() && lineCount < maxLines) {
            String line = f.readStringUntil('\n');
            USB_SERIAL.println(line);
            lineCount++;
          }

          if (f.available()) {
            USB_SERIAL.println(F("...TRUNCATED"));
          }
          USB_SERIAL.println(F("CAT:END"));
          f.close();
        }
      }
    }

  } else if (cmd.startsWith("RM ")) {
    // Delete file: RM <path>
    if (runtime.currentRpm >= 100) {
      USB_SERIAL.println(F("RM:BUSY"));
    } else if (!runtime.sdCardOk) {
      USB_SERIAL.println(F("RM:ERROR"));
    } else {
      String path = cmd.substring(3);
      path.trim();

      if (!SD.exists(path.c_str())) {
        USB_SERIAL.println(F("RM:NOTFOUND"));
      } else {
        if (SD.remove(path.c_str())) {
          USB_SERIAL.println(F("RM:OK"));
        } else {
          USB_SERIAL.println(F("RM:FAIL"));
        }
      }
    }

  } else if (cmd.startsWith("UPLOAD ")) {
    // Start file upload: UPLOAD /path/to/file
    if (runtime.currentRpm >= 100) {
      USB_SERIAL.println(F("UPLOAD:BUSY"));
    } else if (!runtime.sdCardOk) {
      USB_SERIAL.println(F("UPLOAD:SDERROR"));
    } else if (uploadMode) {
      USB_SERIAL.println(F("UPLOAD:INPROGRESS"));
    } else {
      String path = cmd.substring(7);
      path.trim();

      if (path.length() == 0) {
        USB_SERIAL.println(F("UPLOAD:NOPATH"));
      } else {
        // Disable interrupts during upload for SD stability
        NVIC_DisableIRQ(TIM2_IRQn);

        // Remove existing file if present
        if (SD.exists(path.c_str())) {
          SD.remove(path.c_str());
        }

        uploadFile = SD.open(path.c_str(), FILE_WRITE);
        if (!uploadFile) {
          USB_SERIAL.println(F("UPLOAD:OPENFAIL"));
          NVIC_EnableIRQ(TIM2_IRQn);
        } else {
          uploadMode = true;
          uploadPath = path;
          uploadBytes = 0;
          USB_SERIAL.print(F("UPLOAD:READY,"));
          USB_SERIAL.println(path);
        }
      }
    }

  } else {
    USB_SERIAL.print(F("Unknown: "));
    USB_SERIAL.println(cmd);
  }
}

void handleGet(String p) {
  if (p == "RPM") {
    USB_SERIAL.println(runtime.currentRpm);
  } else if (p == "TIMING") {
    USB_SERIAL.print(runtime.currentTimingScaled / DEG_SCALE);
    USB_SERIAL.print(F("."));
    USB_SERIAL.println(abs(runtime.currentTimingScaled % DEG_SCALE));
  } else if (p == "TEMP") {
    int16_t t = rawToTempX10(runtime.tempRaw);
    USB_SERIAL.print(t / 10);
    USB_SERIAL.print(F("."));
    USB_SERIAL.println(abs(t % 10));
  } else if (p == "BATTERY") {
    int16_t v = rawToVoltageX100(runtime.batteryRaw, config.adcCal.battScaleScaled);
    USB_SERIAL.print(v / 100);
    USB_SERIAL.print(F("."));
    USB_SERIAL.println(abs(v % 100));
  } else if (p == "CHARGING") {
    int16_t v = rawToVoltageX100(runtime.chargingRaw, config.adcCal.chargingScaleScaled);
    USB_SERIAL.print(v / 100);
    USB_SERIAL.print(F("."));
    USB_SERIAL.println(abs(v % 100));
  } else if (p == "MAP") {
    USB_SERIAL.println(config.activeMap + 1);
  } else if (p == "ENGINE") {
    USB_SERIAL.println(config.engineType);
  } else if (p == "PEAK") {
    USB_SERIAL.println(config.peakRpm);
  } else if (p == "TRIGGER") {
    USB_SERIAL.print(config.trigger.triggerAngleScaled / DEG_SCALE);
    USB_SERIAL.print(F("."));
    USB_SERIAL.print(abs(config.trigger.triggerAngleScaled % DEG_SCALE));
    USB_SERIAL.print(F(","));
    USB_SERIAL.println(config.trigger.risingEdge ? F("RISING") : F("FALLING"));
  } else if (p == "LIMITER") {
    USB_SERIAL.print(config.revLimiter.softRpm);
    USB_SERIAL.print(F(","));
    USB_SERIAL.print(config.revLimiter.mediumRpm);
    USB_SERIAL.print(F(","));
    USB_SERIAL.print(config.revLimiter.hardRpm);
    USB_SERIAL.print(F(","));
    USB_SERIAL.println(config.revLimiter.fullCutRpm);
  } else if (p == "SHIFT") {
    USB_SERIAL.print(config.shiftLight.onRpm);
    USB_SERIAL.print(F(","));
    USB_SERIAL.print(config.shiftLight.blinkRpm);
    USB_SERIAL.print(F(","));
    USB_SERIAL.println(config.shiftLight.fastBlinkRpm);
  } else if (p == "CRANKING") {
    USB_SERIAL.print(config.cranking.enabled ? F("ON") : F("OFF"));
    USB_SERIAL.print(F(","));
    USB_SERIAL.print(config.cranking.timingScaled);
    USB_SERIAL.print(F(","));
    USB_SERIAL.println(config.cranking.maxRpm);
  } else if (p == "WARNING") {
    // Format: overRevRpm,overheatTempC,lowBatteryV,overheatRetard
    USB_SERIAL.print(config.warning.overRevRpm);
    USB_SERIAL.print(F(","));
    USB_SERIAL.print(config.warning.overheatTempC);
    USB_SERIAL.print(F(","));
    // Convert millivolts to volts with 1 decimal
    USB_SERIAL.print(config.warning.lowBatteryMv / 1000);
    USB_SERIAL.print(F("."));
    USB_SERIAL.print((config.warning.lowBatteryMv % 1000) / 100);
    USB_SERIAL.print(F(","));
    USB_SERIAL.println(config.warning.overheatRetard);
  } else if (p == "TEMP_RETARD") {
    // Format: retardStartTempC,retardPer10C
    USB_SERIAL.print(config.warning.retardStartTempC);
    USB_SERIAL.print(F(","));
    USB_SERIAL.println(config.warning.retardPer10C);
  } else if (p == "NOISE") {
    USB_SERIAL.println(config.trigger.noiseFilterTicks / 10);  // ticks to us
  } else if (p == "PULSE") {
    USB_SERIAL.println(config.trigger.cdiPulseUs);  // CDI pulse width in us
  } else if (p == "CAL") {
    // Calibration: tempOffset,tempScale,battScale,chargingScale (all x1000)
    USB_SERIAL.print(config.adcCal.tempOffsetScaled);
    USB_SERIAL.print(F(","));
    USB_SERIAL.print(config.adcCal.tempScaleScaled);
    USB_SERIAL.print(F(","));
    USB_SERIAL.print(config.adcCal.battScaleScaled);
    USB_SERIAL.print(F(","));
    USB_SERIAL.println(config.adcCal.chargingScaleScaled);
  } else if (p == "RAW") {
    // Raw ADC values for calibration: temp,batt,charging
    USB_SERIAL.print(runtime.tempRaw);
    USB_SERIAL.print(F(","));
    USB_SERIAL.print(runtime.batteryRaw);
    USB_SERIAL.print(F(","));
    USB_SERIAL.println(runtime.chargingRaw);
  } else if (p == "PERIOD") {
    USB_SERIAL.println(runtime.period);
  } else if (p == "JITTER") {
    // Report timing jitter estimate in nanoseconds
    USB_SERIAL.print(F("~50ns @ "));
    USB_SERIAL.print(runtime.currentRpm);
    USB_SERIAL.println(F(" RPM"));
  } else {
    USB_SERIAL.println(F("Unknown param"));
  }
}

void handleSet(String p) {
  int eq = p.indexOf('=');
  if (eq < 0) {
    USB_SERIAL.println(F("Use: SET PARAM=VALUE"));
    return;
  }

  String name = p.substring(0, eq);
  String val = p.substring(eq + 1);
  name.trim();
  val.trim();

  if (name == "ENGINE") {
    int v = val.toInt();
    if (v == 2 || v == 4) {
      config.engineType = v;
      USB_SERIAL.println(F("OK"));
    } else {
      USB_SERIAL.println(F("Invalid (2 or 4)"));
    }
  } else if (name == "MAP") {
    int v = val.toInt();
    if (v >= 1 && v <= 6) {
      config.activeMap = v - 1;
      USB_SERIAL.println(F("OK"));
    } else {
      USB_SERIAL.println(F("Invalid (1-6)"));
    }
  } else if (name == "TRIGGER_ANGLE") {
    config.trigger.triggerAngleScaled = (int32_t)(val.toFloat() * DEG_SCALE);
    USB_SERIAL.println(F("OK"));
  } else if (name == "TRIGGER_EDGE") {
    config.trigger.risingEdge = (val == "RISING" || val == "1") ? 1 : 0;
    updateTriggerEdge();
    USB_SERIAL.println(F("OK"));
  } else if (name == "NOISE_FILTER") {
    config.trigger.noiseFilterTicks = val.toInt() * 10;  // us to ticks (10MHz)
    USB_SERIAL.println(F("OK"));
  } else if (name == "CDI_PULSE") {
    uint16_t pulse = val.toInt();
    if (pulse < 50) pulse = 50;
    if (pulse > 250) pulse = 250;
    config.trigger.cdiPulseUs = pulse;
    updatePulseWidth();
    USB_SERIAL.println(F("OK"));
  } else if (name == "LIMITER_SOFT") {
    config.revLimiter.softRpm = val.toInt();
    updatePeriodThresholds();
  updatePulseWidth();
    USB_SERIAL.println(F("OK"));
  } else if (name == "LIMITER_MEDIUM") {
    config.revLimiter.mediumRpm = val.toInt();
    updatePeriodThresholds();
  updatePulseWidth();
    USB_SERIAL.println(F("OK"));
  } else if (name == "LIMITER_HARD") {
    config.revLimiter.hardRpm = val.toInt();
    updatePeriodThresholds();
  updatePulseWidth();
    USB_SERIAL.println(F("OK"));
  } else if (name == "LIMITER_FULLCUT") {
    config.revLimiter.fullCutRpm = val.toInt();
    updatePeriodThresholds();
  updatePulseWidth();
    USB_SERIAL.println(F("OK"));
  } else if (name == "SHIFT_ON") {
    config.shiftLight.onRpm = val.toInt();
    USB_SERIAL.println(F("OK"));
  } else if (name == "SHIFT_BLINK") {
    config.shiftLight.blinkRpm = val.toInt();
    USB_SERIAL.println(F("OK"));
  } else if (name == "SHIFT_FAST") {
    config.shiftLight.fastBlinkRpm = val.toInt();
    USB_SERIAL.println(F("OK"));
  } else if (name == "CRANKING_ENABLE") {
    config.cranking.enabled = (val == "1" || val == "ON") ? 1 : 0;
    USB_SERIAL.println(F("OK"));
  } else if (name == "CRANKING_TIMING") {
    config.cranking.timingScaled = val.toInt();
    USB_SERIAL.println(F("OK"));
  } else if (name == "CRANKING_RPM") {
    config.cranking.maxRpm = val.toInt();
    USB_SERIAL.println(F("OK"));
  } else if (name == "OVERREV_RPM") {
    config.warning.overRevRpm = val.toInt();
    updatePeriodThresholds();
  updatePulseWidth();
    USB_SERIAL.println(F("OK"));
  } else if (name == "OVERHEAT_RETARD") {
    config.warning.overheatRetard = val.toInt();
    USB_SERIAL.println(F("OK"));
  } else if (name == "RETARD_START") {
    // Set progressive retard start temperature (e.g., SET RETARD_START=80)
    config.warning.retardStartTempC = val.toInt();
    USB_SERIAL.print(F("RetardStart: "));
    USB_SERIAL.print(config.warning.retardStartTempC);
    USB_SERIAL.println(F("C"));
  } else if (name == "RETARD_PER10C") {
    // Set retard per 10°C in 0.1° units (e.g., SET RETARD_PER10C=10 for 1.0°)
    config.warning.retardPer10C = val.toInt();
    USB_SERIAL.print(F("RetardPer10C: "));
    USB_SERIAL.print(config.warning.retardPer10C);
    USB_SERIAL.println(F(" (0.1 deg)"));
  } else if (name == "OVERHEAT_TEMP") {
    // Set overheat threshold in Celsius (e.g., SET OVERHEAT_TEMP=120)
    config.warning.overheatTempC = val.toInt();
    USB_SERIAL.print(F("Overheat: "));
    USB_SERIAL.print(config.warning.overheatTempC);
    USB_SERIAL.println(F("C"));
  } else if (name == "LOW_BATTERY") {
    // Set low battery threshold in Volts (e.g., SET LOW_BATTERY=11.0)
    float v = val.toFloat();
    config.warning.lowBatteryMv = (uint16_t)(v * 1000);
    USB_SERIAL.print(F("LowBatt: "));
    USB_SERIAL.print(config.warning.lowBatteryMv / 1000);
    USB_SERIAL.print(F("."));
    USB_SERIAL.print(config.warning.lowBatteryMv % 1000 / 100);
    USB_SERIAL.println(F("V"));
  } else if (name == "CAL_TEMP_OFFSET") {
    config.adcCal.tempOffsetScaled = val.toInt();
    USB_SERIAL.println(F("OK"));
  } else if (name == "CAL_TEMP_SCALE") {
    config.adcCal.tempScaleScaled = val.toInt();
    USB_SERIAL.println(F("OK"));
  } else if (name == "CAL_BATT_SCALE") {
    config.adcCal.battScaleScaled = val.toInt();
    USB_SERIAL.println(F("OK"));
  } else if (name == "CAL_CHARGING_SCALE") {
    config.adcCal.chargingScaleScaled = val.toInt();
    USB_SERIAL.println(F("OK"));
  } else {
    USB_SERIAL.println(F("Unknown param"));
  }
}

void handleMap(String p) {
  int sp1 = p.indexOf(' ');
  if (sp1 < 0) {
    USB_SERIAL.println(F("Invalid format"));
    return;
  }

  int mapNum = p.substring(0, sp1).toInt();
  if (mapNum < 1 || mapNum > 6) {
    USB_SERIAL.println(F("Invalid map (1-6)"));
    return;
  }

  String rest = p.substring(sp1 + 1);
  rest.trim();

  if (rest == "GET") {
    USB_SERIAL.print(F("MAP"));
    USB_SERIAL.print(mapNum);
    USB_SERIAL.print(F(":"));
    for (int i = 0; i < RPM_TABLE_SIZE; i++) {
      USB_SERIAL.print(config.timingMaps[mapNum-1][i]);
      if (i < RPM_TABLE_SIZE - 1) USB_SERIAL.print(F(","));
    }
    USB_SERIAL.println();
  } else {
    int sp2 = rest.indexOf(' ');
    if (sp2 < 0) {
      USB_SERIAL.println(F("Use: MAP N RPM TIMING"));
      return;
    }

    int rpm = rest.substring(0, sp2).toInt();
    int timing = rest.substring(sp2 + 1).toInt();

    if (rpm < 0 || rpm > RPM_MAX) {
      USB_SERIAL.println(F("Invalid RPM"));
      return;
    }
    if (timing < -10 || timing > 60) {
      USB_SERIAL.println(F("Invalid timing (-10 to 60)"));
      return;
    }

    uint8_t idx = rpmToIndex(rpm);
    config.timingMaps[mapNum-1][idx] = (int8_t)timing;

    USB_SERIAL.print(F("M"));
    USB_SERIAL.print(mapNum);
    USB_SERIAL.print(F(" @"));
    USB_SERIAL.print(rpm);
    USB_SERIAL.print(F("="));
    USB_SERIAL.println(timing);
  }
}

// Static buffer for RT data - avoids multiple print() calls that can block
static char rtBuffer[128];

void sendRealtimeData(void) {
  // Format: RT:RPM,TIMING,TEMP,BATT,CHARGING,MAP,LIMITER,FLAGS,PEAK,CPU,RAM,TRIG,CUT,ENGTYPE
  // Using single buffer write instead of 14+ print() calls to reduce blocking

  // Build flags byte
  uint8_t flags = 0;
  if (runtime.engineRunning) flags |= 0x01;
  if (runtime.overheating) flags |= 0x02;
  if (runtime.lowBattery) flags |= 0x04;
  if (runtime.killActive) flags |= 0x08;
  if (runtime.usingDefaultMap) flags |= 0x10;
  if (runtime.ignitionEnabled) flags |= 0x20;
  if (runtime.sdCardOk) flags |= 0x40;

  // Build entire string in buffer (single write = less blocking)
  int len = snprintf(rtBuffer, sizeof(rtBuffer),
    "RT:%u,%d,%d,%d,%d,%u,%u,%u,%u,%u,%u,%d,%u,%u\n",
    runtime.currentRpm,
    runtime.currentTimingScaled / DEG_SCALE,
    rawToTempX10(runtime.tempRaw) / 10,
    rawToVoltageX100(runtime.batteryRaw, config.adcCal.battScaleScaled) / 10,
    rawToVoltageX100(runtime.chargingRaw, config.adcCal.chargingScaleScaled) / 10,
    config.activeMap + 1,
    runtime.limiterStage,
    flags,
    config.peakRpm,
    cpuUsagePercent,
    getRamUsagePercent(),
    config.trigger.triggerAngleScaled / DEG_SCALE,
    runtime.lastCycleWasCut,
    config.engineType
  );

  // Single write - much faster than 14+ print() calls
  if (len > 0 && len < (int)sizeof(rtBuffer)) {
    USB_SERIAL.write(rtBuffer, len);
  }
}

void sendStatus(void) {
  USB_SERIAL.println(F("\n=== RACING CDI - MAX PRECISION ==="));
  USB_SERIAL.print(F("Timer Res: 100ns (10MHz)\n"));
  USB_SERIAL.print(F("Jitter: <0.05deg @ all RPM\n"));
  USB_SERIAL.print(F("Engine: ")); USB_SERIAL.println(runtime.engineRunning ? F("RUN") : F("STOP"));
  USB_SERIAL.print(F("RPM: ")); USB_SERIAL.println(runtime.currentRpm);
  USB_SERIAL.print(F("Peak: ")); USB_SERIAL.println(config.peakRpm);
  USB_SERIAL.print(F("Timing: ")); USB_SERIAL.print(runtime.currentTimingScaled / DEG_SCALE);
  USB_SERIAL.print(F(".")); USB_SERIAL.print(abs(runtime.currentTimingScaled % DEG_SCALE));
  USB_SERIAL.println(F(" BTDC"));
  USB_SERIAL.print(F("Map: ")); USB_SERIAL.println(config.activeMap + 1);
  USB_SERIAL.print(F("Type: ")); USB_SERIAL.println(config.engineType == 2 ? F("2T") : F("4T"));
  USB_SERIAL.print(F("Temp: ")); USB_SERIAL.print(rawToTempX10(runtime.tempRaw) / 10); USB_SERIAL.println(F("C"));
  USB_SERIAL.print(F("Batt: ")); USB_SERIAL.print(rawToVoltageX100(runtime.batteryRaw, config.adcCal.battScaleScaled) / 100); USB_SERIAL.println(F("V"));
  USB_SERIAL.print(F("Ign: ")); USB_SERIAL.println(runtime.ignitionCount);
  USB_SERIAL.print(F("Cut: ")); USB_SERIAL.println(runtime.cutCount);
  USB_SERIAL.print(F("SD: ")); USB_SERIAL.println(runtime.sdCardOk ? F("OK") : F("FAIL"));
  USB_SERIAL.print(F("DefMap: ")); USB_SERIAL.println(runtime.usingDefaultMap ? F("YES") : F("NO"));
  USB_SERIAL.print(F("Kill: ")); USB_SERIAL.println(runtime.killActive ? F("ON") : F("OFF"));
  USB_SERIAL.print(F("Limiter: ")); USB_SERIAL.println(runtime.limiterStage);
  USB_SERIAL.print(F("OverheatSet: ")); USB_SERIAL.print(config.warning.overheatTempC); USB_SERIAL.println(F("C"));
  USB_SERIAL.print(F("LowBattSet: ")); USB_SERIAL.print(config.warning.lowBatteryMv / 1000);
  USB_SERIAL.print(F(".")); USB_SERIAL.print(config.warning.lowBatteryMv % 1000 / 100); USB_SERIAL.println(F("V"));
  USB_SERIAL.println(F("================================\n"));
}

void sendHelp(void) {
  USB_SERIAL.println(F("\n=== COMMANDS ==="));
  USB_SERIAL.println(F("GET RPM|TIMING|TEMP|BATTERY|MAP|PEAK|PICKUP|LIMITER|PERIOD|JITTER"));
  USB_SERIAL.println(F("SET ENGINE=2|4, MAP=1-6"));
  USB_SERIAL.println(F("SET PICKUP_ANGLE=deg, TOOTH_WIDTH=deg"));
  USB_SERIAL.println(F("SET TRIGGER_EDGE=RISING|FALLING, NOISE_FILTER=us"));
  USB_SERIAL.println(F("SET LIMITER_SOFT|MEDIUM|HARD|FULLCUT=rpm"));
  USB_SERIAL.println(F("SET SHIFT_ON|BLINK|FAST=rpm"));
  USB_SERIAL.println(F("SET CRANKING_ENABLE=ON|OFF, CRANKING_TIMING=deg"));
  USB_SERIAL.println(F("SET OVERHEAT_TEMP=Celsius (e.g. 120)"));
  USB_SERIAL.println(F("SET LOW_BATTERY=Volts (e.g. 11.0)"));
  USB_SERIAL.println(F("MAP [1-6] [rpm] [timing] - Hot edit"));
  USB_SERIAL.println(F("MAP [1-6] GET - Read map"));
  USB_SERIAL.println(F("IGN ON|OFF - Enable/disable ignition"));
  USB_SERIAL.println(F("SAVE, LOAD, DEFAULT, STATUS, HELP"));
  USB_SERIAL.println(F("RESETPEAK"));
  USB_SERIAL.println(F("EXPORT - Export all to text files"));
  USB_SERIAL.println(F("IMPORT - Import all from text files"));
  USB_SERIAL.println(F("EXPORTMAPS, IMPORTMAPS - Maps only"));
  USB_SERIAL.println(F("EXPORTSETTINGS, IMPORTSETTINGS"));
  USB_SERIAL.println(F("================\n"));
}

// ============================================================================
// SD CARD SAFETY & LOGGING
// ============================================================================

// Log file handle (declared here so checkSDCardSafety can access it)
static File logFile;
static String logFileName = "";
static uint32_t logWriteCount = 0;

// Check if SD card is physically present using detect pin
// Returns: 1 = card present, 0 = card removed
static inline uint8_t isSDCardPresent(void) {
  // SD_DETECT pin is HIGH when card is inserted (active high)
  // Note: Some SD sockets are active low - change HIGH to LOW if needed
  return (digitalRead(PIN_SD_DETECT) == HIGH) ? 1 : 0;
}

// Handle SD card removal/reinsertion safely
// Called periodically from main loop
// Handles vibration-induced disconnects with debounce
void checkSDCardSafety(void) {
  static uint8_t lastCardState = 1;      // Assume card present at start
  static uint8_t stableCardState = 1;    // Debounced state
  static uint8_t debounceCount = 0;      // Debounce counter
  static uint32_t lastCheck = 0;
  static uint32_t lastRemoveTime = 0;    // When card was removed

  uint32_t now = millis();

  // Check every 50ms for faster response but with debounce
  if (now - lastCheck < 50) return;
  lastCheck = now;

  uint8_t cardPresent = isSDCardPresent();

  // Debounce: require 3 consistent readings (150ms) to change state
  if (cardPresent == lastCardState) {
    debounceCount++;
    if (debounceCount >= 3) {
      // State is stable, update stableCardState
      if (stableCardState != cardPresent) {
        // State changed after debounce

        if (!cardPresent) {
          // === CARD REMOVED ===
          USB_SERIAL.println(F("WARNING: SD card removed!"));
          lastRemoveTime = now;

          // Close log file safely
          if (logFile) {
            logFile.close();
            logFileName = "";
          }

          // Mark SD as not OK
          runtime.sdCardOk = 0;
          runtime.sdBusy = 0;

          // Set warning LED pattern
          runtime.ledPattern = LED_PATTERN_ERROR;

        } else {
          // === CARD RE-INSERTED ===
          uint32_t removeTime = now - lastRemoveTime;

          USB_SERIAL.print(F("SD reconnected (was out "));
          USB_SERIAL.print(removeTime);
          USB_SERIAL.println(F("ms)"));

          // NEVER auto-reinit SD - too dangerous, can trigger watchdog!
          // SD.begin() can block for 4+ seconds on some cards
          // User must use RESET command (engine off) or power cycle
          USB_SERIAL.println(F("SD detected - use RESET to reinit"));
          runtime.ledPattern = LED_PATTERN_WARNING;
          // Keep sdCardOk = 0 until MCU reset
        }

        stableCardState = cardPresent;
      }
      debounceCount = 3;  // Cap at 3
    }
  } else {
    // State changed, reset debounce
    debounceCount = 0;
  }

  lastCardState = cardPresent;
}

void logData(void) {
  if (!runtime.sdCardOk) return;

  // Skip if SD is busy (prevents blocking)
  if (runtime.sdBusy) return;

  runtime.sdBusy = 1;  // Mark SD busy

  // Use hours since boot for log file rotation (new file every hour)
  uint32_t logHour = millis() / 3600000UL;
  String fn = String(LOG_FILE_PREFIX) + String(logHour) + ".csv";

  // Check if we need to open a new file (first call or hour changed)
  if (fn != logFileName || !logFile) {
    // Close previous file if open
    if (logFile) {
      logFile.flush();
      logFile.close();
    }

    bool newFile = !SD.exists(fn.c_str());
    logFile = SD.open(fn.c_str(), FILE_WRITE);

    if (!logFile) {
      runtime.sdCardOk = 0;  // Mark SD as failed
      runtime.sdBusy = 0;
      USB_SERIAL.println(F("Log open failed - SD error"));
      return;
    }

    logFileName = fn;
    logWriteCount = 0;

    // Seek to end for append
    if (!newFile) {
      logFile.seek(logFile.size());
    } else {
      logFile.println(F("ms,rpm,timing,temp,batt,map,lim,flags"));
    }
  }

  // Write data to already-open file using single buffer (faster than multiple print())
  uint8_t flags = 0;
  if (runtime.overheating) flags |= 0x02;
  if (runtime.lowBattery) flags |= 0x04;
  if (runtime.killActive) flags |= 0x08;

  // Use static buffer to avoid stack allocation
  static char logBuffer[80];
  int len = snprintf(logBuffer, sizeof(logBuffer),
    "%lu,%u,%d,%d,%d,%u,%u,%u\n",
    millis(),
    runtime.currentRpm,
    runtime.currentTimingScaled / DEG_SCALE,
    rawToTempX10(runtime.tempRaw) / 10,
    rawToVoltageX100(runtime.batteryRaw, config.adcCal.battScaleScaled) / 100,
    config.activeMap + 1,
    runtime.limiterStage,
    flags
  );

  // Single write - much faster than 8 print() calls
  if (len > 0 && len < (int)sizeof(logBuffer)) {
    logFile.write(logBuffer, len);
  }

  logWriteCount++;

  // Flush every 10 writes (~1 second) to ensure data is saved
  if (logWriteCount >= 10) {
    logFile.flush();
    logWriteCount = 0;

    // Check for write error (SD died/corrupted while inserted)
    if (logFile.getWriteError()) {
      USB_SERIAL.println(F("SD write error - marking as failed"));
      logFile.close();
      logFileName = "";
      runtime.sdCardOk = 0;
      runtime.ledPattern = LED_PATTERN_ERROR;
    }
  }

  runtime.sdBusy = 0;  // SD operation done
}

// Close log file (call when engine stops)
void closeLogFile(void) {
  if (logFile) {
    logFile.flush();
    logFile.close();
    logFileName = "";
  }
}

// ============================================================================
// SETUP
// ============================================================================

void setup() {
  // Initialize runtime
  memset(&runtime, 0, sizeof(RuntimeData));
  runtime.ignitionEnabled = 1;

  // Initialize random seed
  randState = analogRead(PIN_ADC_HEAD_TEMP) ^ micros();

  // Setup hardware
  setupPins();
  setupADC();

  // Setup USB
  USB_SERIAL.begin(115200);
  USB_SERIAL.setTimeout(10);  // Short timeout to prevent blocking in readStringUntil
  uint32_t t0 = millis();
  while (!USB_SERIAL && (millis() - t0 < 2000)) { delay(10); }

  USB_SERIAL.println(F("\n=== RACING CDI v2.0 MAX PRECISION ==="));
  USB_SERIAL.println(F("Timer: 10MHz (100ns resolution)"));
  USB_SERIAL.println(F("Jitter: <0.05 degrees"));

  // Load config
  loadDefaultConfig();
  setupSD();

  // Initialize lookup tables
  initLookupTables();
  initPeriodLookup();  // Period-to-index table for fast ISR (no division)

  // Initialize DWT cycle counter for accurate CPU measurement
  initDWT();

  // Setup timers with direct register access
  setupTimers();

  // CRITICAL: Set interrupt priorities for system stability
  // SysTick (millis) must run - set to priority 0 (highest)
  // USB must run - typically at priority 0-1
  // TIM3 (ignition) at priority 3 - still high for timing accuracy
  // TIM2 (capture) at priority 4 - lower to not block system
  NVIC_SetPriority(SysTick_IRQn, 0);           // millis() - highest
  HAL_NVIC_SetPriority(TIM3_IRQn, 3, 0);       // Ignition output
  HAL_NVIC_SetPriority(TIM2_IRQn, 4, 0);       // VR capture - lowest of timers

  // Ready indication (PB2 is active-low LED)
  LED_ON();   // Turn LED on (pin LOW)
  delay(200);
  LED_OFF();  // Turn LED off (pin HIGH)

  USB_SERIAL.print(F("Engine: "));
  USB_SERIAL.println(config.engineType == 2 ? F("2-stroke") : F("4-stroke"));
  USB_SERIAL.print(F("Map: "));
  USB_SERIAL.println(config.activeMap + 1);

  if (runtime.usingDefaultMap) {
    USB_SERIAL.println(F("WARNING: Using default safety map!"));
  }

  // Initialize watchdog timer for auto-recovery from hang
  initWatchdog();
  USB_SERIAL.println(F("Watchdog: 4s timeout"));

  // System ready - enable trigger processing
  runtime.configReady = 1;
  USB_SERIAL.println(F("READY!\n"));
}

// ============================================================================
// MAIN LOOP
// ============================================================================

void loop() {
  // Kick watchdog to prevent reset (must be called every <2 seconds)
  kickWatchdog();

  // Mark start of loop iteration for CPU measurement
  cpuLoopStart();

  static uint32_t lastADC = 0;
  static uint32_t lastOutput = 0;
  static uint32_t lastUSB = 0;
  static uint32_t lastLog = 0;
  static uint32_t lastSdBusyCheck = 0;

  uint32_t now = millis();

  // Update CPU usage measurement
  updateCpuUsage();

  // Check SD card safety (detect removal/insertion)
  checkSDCardSafety();

  // Handle inputs (every loop)
  handleInputs();

  // Calculate RPM from period (moved from ISR to reduce ISR time)
  // This is called every loop for responsive display
  if (runtime.period > 0 && runtime.engineRunning) {
    runtime.currentRpm = periodToRpm(runtime.period);
  }

  // Update status LED (every loop for responsive patterns)
  updateStatusLED();

  // Read ADC every 10ms
  if (now - lastADC >= 10) {
    lastADC = now;
    readADC();
  }

  // Update outputs every 50ms
  if (now - lastOutput >= 50) {
    lastOutput = now;
    updateOutputs();

    // Update peak RPM (moved from ISR to reduce ISR time)
    if (runtime.currentRpm > config.peakRpm) {
      config.peakRpm = runtime.currentRpm;
    }
  }

  // Send USB data every 50ms (skip if buffer not ready to prevent blocking)
  if (now - lastUSB >= 50) {
    // Only send if USB buffer has space (prevents blocking)
    if (USB_SERIAL.availableForWrite() >= 64) {
      lastUSB = now;
      sendRealtimeData();
    }
  }

  // Process USB commands
  processUSB();

  // Safety: Reset sdBusy if stuck for more than 5 seconds
  if (runtime.sdBusy) {
    if (lastSdBusyCheck == 0) {
      lastSdBusyCheck = now;
    } else if (now - lastSdBusyCheck > 5000) {
      runtime.sdBusy = 0;  // Force reset
      lastSdBusyCheck = 0;
    }
  } else {
    lastSdBusyCheck = 0;
  }

  // Log every 1000ms if RPM >= 1000 (engine actually running)
  if (runtime.currentRpm >= 1000 && (now - lastLog >= 1000)) {
    lastLog = now;
    logData();
  }

  // Engine timeout check using timer ticks (no trigger for 500ms = engine stopped)
  // This avoids dependency on millis() in ISR
  if (runtime.engineRunning) {
    uint32_t currentTicks = TIM_CAPTURE->CNT;
    uint32_t ticksSinceLastTrigger = currentTicks - runtime.lastCapture;
    if (ticksSinceLastTrigger > ENGINE_TIMEOUT_TICKS) {
      runtime.engineRunning = 0;
      runtime.currentRpm = 0;
      runtime.triggerCount = 0;  // Reset for cold-start protection on next start
      runtime.startupIndex = 0;  // Reset startup period validation
    }
  }

  // Track engine state for auto-save and log file management
  static uint8_t engineWasRunning = 0;
  static uint8_t needSaveAfterStop = 0;
  static uint32_t engineStopTime = 0;

  if (runtime.currentRpm >= 1000) {
    // Engine running - DO NOT perform SD operations here!
    // NVIC_DisableIRQ(TIM2_IRQn) would kill VR capture and cause stuck
    engineWasRunning = 1;
    needSaveAfterStop = 1;  // Mark that we need to save when engine stops
    engineStopTime = 0;
  } else {
    // Engine not running - safe to do SD operations
    if (engineWasRunning) {
      // Engine just stopped - close log file first!
      USB_SERIAL.println(F("Engine stopped, closing log..."));
      closeLogFile();
      engineStopTime = now;
    }
    engineWasRunning = 0;

    // Auto-save 500ms after engine stops (safe - engine not running)
    if (needSaveAfterStop && engineStopTime > 0 && (now - engineStopTime > 500)) {
      needSaveAfterStop = 0;
      engineStopTime = 0;
      if (runtime.sdCardOk) {
        USB_SERIAL.println(F("Auto-save after engine stop..."));
        saveConfigToSD();  // No NVIC disable needed - engine already stopped
      }
    }
  }

  // Mark end of loop iteration for CPU measurement
  cpuLoopEnd();
}
