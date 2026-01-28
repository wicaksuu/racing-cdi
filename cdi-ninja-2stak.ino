/*
 * ============================================================================
 * RACING CDI - STM32H562RGT6 - MAXIMUM PRECISION BUILD
 * ============================================================================
 * Board: WeAct Studio STM32H562RGT6 (ARM Cortex-M33 @ 250MHz)
 * Framework: Arduino STM32
 *
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
struct QuickShifterConfig;
struct CalibrationConfig;
struct CalibrationData;
struct CDIConfig;
struct RuntimeData;
struct CalibrationState;
struct FlashDefaults;

// Function prototypes that use structs
uint32_t calculateChecksum(struct CDIConfig* cfg);
bool validateConfig(struct CDIConfig* cfg);
uint32_t calculateFlashChecksum(struct FlashDefaults* fd);
uint32_t calculateCalChecksum(struct CalibrationData* cd);
bool validateCalData(struct CalibrationData* cd);

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
#define PIN_QUICK_SHIFT     PA3   // Quick shifter sensor input
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

// Calibration pins (self-test mode) - SEPARATE from CDI pins!
#define PIN_CAL_RPM_OUT     PB_8   // RPM generator output (TIM16_CH1) - jumper to PA0
#define PIN_CAL_IGN_CAP     PB_3   // Ignition capture input (polling) - jumper from PB0
// Note: PB3 is on P2 Left header, dedicated for calibration only

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

// Quick Shifter map: 21 points (0, 1000, 2000, ... 20000 RPM)
#define QS_RPM_STEP         1000
#define QS_TABLE_SIZE       21

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

// Max valid period (timer ticks at 10MHz)
// 600ms = 6,000,000 ticks = 100 RPM minimum
// Periods longer than this indicate stopped engine or sensor failure
#define MAX_VALID_PERIOD_TICKS  6000000UL

// Cold start protection - wait for N valid triggers before firing
// RACING MODE: Minimal startup delay, rely on period filter for EMI rejection
#define STARTUP_TRIGGER_COUNT   2     // Just 2 triggers to start (minimal delay)
#define STARTUP_PERIOD_TOLERANCE 2    // 50% tolerance (very lenient for racing)

// LED Status Patterns (for MCU health indication)
#define LED_PATTERN_IDLE        0   // Slow blink - system idle, no engine
#define LED_PATTERN_RUNNING     1   // Fast blink - engine running, all OK
#define LED_PATTERN_WARNING     2   // Double blink - warning (overheat/low batt)
#define LED_PATTERN_ERROR       3   // Triple blink - error (SD fail, using default map)
#define LED_PATTERN_MAP_CHANGE  4   // Blink N times for map number

// CPU usage measurement interval
#define CPU_MEASURE_INTERVAL_MS 100

// Enable/disable CPU measurement (disable in production to reduce overhead)
#define DEBUG_CPU_USAGE

// ============================================================================
// CALIBRATION CONSTANTS - 2D Map with Fuzzy Logic (0.01° precision)
// ============================================================================

// Calibration table dimensions
#define CAL_TIMING_POINTS   61    // 0°, 1°, 2°, ... 60° BTDC
#define CAL_RPM_POINTS      81    // 0, 250, 500, ... 20000 RPM
#define CAL_TOTAL_POINTS    (CAL_RPM_POINTS * CAL_TIMING_POINTS)  // 4941 points

// Magic number and version for calibration file
#define CAL_MAGIC           0xCA12D002
#define CAL_VERSION         2

// Calibration modes
#define CAL_MODE_OFF        0   // Normal operation
#define CAL_MODE_RUNNING    1   // Calibration in progress
#define CAL_MODE_PAUSED     2   // Paused by user
#define CAL_MODE_COMPLETE   3   // All points calibrated

// Precision settings
// Internal: 0.001° (millidegrees) for fuzzy calculations
// Storage: 0.01° (centidegrees) in int16_t
#define CAL_INTERNAL_SCALE  1000  // 1° = 1000 millidegrees
#define CAL_STORAGE_SCALE   100   // 1° = 100 centidegrees

// Fuzzy logic thresholds (in millidegrees = 0.001°)
#define FUZZY_HUGE_THRESH   5000  // > 5.0° error
#define FUZZY_LARGE_THRESH  2000  // > 2.0° error
#define FUZZY_MEDIUM_THRESH 500   // > 0.5° error
#define FUZZY_SMALL_THRESH  100   // > 0.1° error
#define FUZZY_TINY_THRESH   10    // > 0.01° error

// Fuzzy step sizes (in millidegrees = 0.001°)
#define FUZZY_STEP_HUGE     1000  // 1.000° step
#define FUZZY_STEP_LARGE    500   // 0.500° step
#define FUZZY_STEP_MEDIUM   100   // 0.100° step
#define FUZZY_STEP_SMALL    10    // 0.010° step
#define FUZZY_STEP_TINY     1     // 0.001° step (minimum)

// Convergence settings
#define CAL_MAX_ITERATIONS  50    // Max iterations per point
#define CAL_CONVERGE_COUNT  5     // Must be stable for N consecutive checks
#define CAL_SAMPLES_PER_CHECK 10  // Samples per convergence check
#define CAL_DEFAULT_MARGIN  5     // Default margin: 0.05° (in 0.01° units)

// Misfire detection settings
#define CAL_MISFIRE_TIMEOUT_TRIGGERS 5   // Misfires if no capture after N triggers
#define CAL_MISFIRE_MAX_PER_POINT    10  // Max misfires before marking point failed
#define CAL_MISFIRE_RETRY_DELAY_MS   50  // Delay before retry after misfire
#define CAL_MISFIRE_SAFETY_RETARD    200 // Safety retard: 2.00° (in 0.01° units) for misfire-prone points

// RPM sweep parameters
#define CAL_RPM_START       500   // Start RPM
#define CAL_RPM_END         15000 // End RPM (practical limit for calibration)
#define CAL_RPM_HOLD_MS     150   // Hold time at each RPM point
#define CAL_SETTLE_MS       50    // Settle time after RPM change

// Timer definitions
#define TIM_CAL_RPM         TIM16
// TIM_CAL_IGN removed - using EXTI interrupt instead

// File paths
#define CAL_FILE_BIN        "racing-cdi/calibration.bin"
#define CAL_FILE_TXT        "racing-cdi/calibration.txt"

// ============================================================================
// CPU/RAM MONITORING - ACCURATE MEASUREMENT USING DWT CYCLE COUNTER
// ============================================================================

#ifdef DEBUG_CPU_USAGE
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
#else
// When CPU measurement disabled, just provide a placeholder for UI
static volatile uint8_t cpuUsagePercent = 0;
#endif

// External symbols from linker for RAM calculation
extern "C" char _end;       // End of .bss section (start of heap)
extern "C" char _estack;    // End of stack (top of RAM)
extern "C" char _sdata;     // Start of .data section
extern "C" char _ebss;      // End of .bss section

#ifdef DEBUG_CPU_USAGE
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
#else
void initDWT(void) { }  // No-op when disabled
#endif

// ============================================================================
// WATCHDOG TIMER - Auto-recovery from MCU hang
// ============================================================================
// Uses IWDG (Independent Watchdog) running from LSI (~32kHz)
// Timeout ~2 seconds - if loop() doesn't run for 2s, MCU resets

void initWatchdog(void) {
  // Enable IWDG with safe timeout
  // STM32H5: LSI = 32kHz, Prescaler /256 (PR=6) -> 125Hz tick
  // Timeout = RLR / 125 seconds

  IWDG->KR = 0xCCCC;  // Start watchdog (cannot be stopped after this!)
  IWDG->KR = 0x5555;  // Enable write access to PR, RLR

  IWDG->PR = 6;       // Prescaler /256 (slower tick)
  IWDG->RLR = 250;    // Reload value for ~2.0s timeout (250/125 = 2s)

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

#ifdef DEBUG_CPU_USAGE
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
#else
// No-op stubs when CPU measurement disabled
static inline void cpuLoopStart(void) { }
static inline void cpuLoopEnd(void) { }
void updateCpuUsage(void) { }
#endif

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
  int16_t retardStartTempC;      // Temperature to start progressive retard (default 80°C)
  uint8_t retardPer10C;         // Retard per 10°C in 0.1° units (10 = 1.0°, 15 = 1.5°)
  uint8_t reserved;
};

// Quick Shifter configuration (Strain Gauge / Load Cell sensor)
struct __attribute__((aligned(4))) QuickShifterConfig {
  uint8_t enabled;              // 0=off, 1=on
  uint8_t sensitivity;          // Debounce in ms (1-50ms, default 10)
  uint16_t baseline;            // ADC value at rest (default 2048 for 12-bit ADC mid-point)
  uint16_t threshold;           // ADC value to trigger shift (default 2500)
  uint16_t minRpm;              // Minimum RPM to activate (default 3000)
  uint16_t maxRpm;              // Maximum RPM (default 15000, safety limit)
  // Cut time map: 21 points (0-20000 RPM in 1000 RPM steps)
  // Values in milliseconds (10-200ms typical)
  uint8_t cutTimeMap[QS_TABLE_SIZE];
};

// Calibration settings (minimal, stored in CDIConfig)
// Full calibration data stored separately in calibration.bin
struct __attribute__((aligned(4))) CalibrationConfig {
  uint8_t enabled;              // 0=off, 1=use calibration offsets
  uint8_t marginError;          // Allowed error margin in 0.01° units (default 5 = 0.05°)
  uint8_t complete;             // 1 = calibration data loaded and valid
  uint8_t reserved;
};

// ============================================================================
// 2D CALIBRATION DATA STRUCTURE (Stored in separate file: calibration.bin)
// Total size: ~10.5 KB
// ============================================================================
struct __attribute__((aligned(4))) CalibrationData {
  uint32_t magic;               // CAL_MAGIC (0xCA12D002)
  uint8_t version;              // CAL_VERSION
  uint8_t enabled;              // Copy of config enabled flag
  uint8_t marginError;          // Margin in 0.01° units
  uint8_t complete;             // All points calibrated

  // 2D offset map: [RPM index 0-80][Timing degree 0-60]
  // Storage: 0.01° (centidegrees) per unit
  // Range: -327.67° to +327.67° (way more than needed)
  // Actual expected range: ±10°
  int16_t offsets[CAL_RPM_POINTS][CAL_TIMING_POINTS];  // 81 × 61 × 2 = 9,882 bytes

  // Calibration quality bitmap: 1 bit per point
  // Bit set = point successfully calibrated within margin
  uint8_t calibrated[CAL_RPM_POINTS][(CAL_TIMING_POINTS + 7) / 8];  // 81 × 8 = 648 bytes

  // Misfire-prone bitmap: 1 bit per point
  // Bit set = point had misfires during calibration (apply safety retard)
  uint8_t misfireProne[CAL_RPM_POINTS][(CAL_TIMING_POINTS + 7) / 8];  // 81 × 8 = 648 bytes

  uint32_t checksum;
};
// sizeof(CalibrationData) ≈ 10,540 bytes

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
  QuickShifterConfig quickShifter;
  CalibrationConfig calibration;

  uint16_t peakRpm;
  uint16_t reserved1;

  uint32_t checksum;
};

// Runtime data - critical timing data first for cache optimization
struct __attribute__((aligned(4))) RuntimeData {
  // === CRITICAL TIMING DATA (accessed in ISR) ===
  volatile uint32_t lastCapture;          // Last capture value
  volatile uint32_t period;               // Period in timer ticks (current/latest)
  volatile uint32_t scheduledPeriod;      // Period at ignition scheduling (for late fire check)
  volatile uint32_t scheduledCapture;     // Capture time at ignition scheduling (for late fire check)
  volatile uint32_t nextFireTick;         // When to fire next
  volatile uint16_t currentRpm;           // Current RPM
  volatile int16_t currentTimingScaled;   // Current timing x100
  volatile uint8_t ignitionPending;       // Ignition scheduled flag
  volatile uint8_t fourStrokeCycle;       // 4-stroke toggle
  volatile uint8_t killActive;            // Kill switch active
  volatile uint8_t limiterStage;          // Current limiter stage
  volatile uint8_t limiterCounter;        // Counter for pattern-based cut
  volatile uint8_t predictiveMode;        // 1 = predictive mode (timing > trigger angle)

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
  volatile uint8_t configSource;      // 0=SD, 1=Flash, 2=Hardcoded
  volatile uint8_t timingClamped;     // Timing was clamped to min/max (map value out of range)

  // === QUICK SHIFTER STATE ===
  volatile uint8_t qsActive;          // Quick shift cut currently active
  volatile uint8_t qsTriggered;       // Sensor triggered (debounced)
  volatile uint8_t qsArmed;           // Ready for next trigger (sensor returned to baseline)
  volatile uint16_t qsCurrentAdc;     // Current ADC reading (for calibration UI)
  volatile uint32_t qsCutStartMs;     // When cut started
  volatile uint32_t qsCutDurationMs;  // Current cut duration
  volatile uint32_t qsLastTriggerMs;  // For debounce

  // === COUNTERS ===
  volatile uint32_t ignitionCount;
  volatile uint32_t cutCount;
  volatile uint32_t triggerCount;     // Valid trigger count since startup (for cold-start protection)
  volatile uint32_t skippedTriggers;  // Race condition skip count (ignitionPending)

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

  // === PRECISION TIMING (Expert Review Optimizations) ===
  // Blind Window - ignore captures after ignition to filter EMI
  volatile uint32_t lastIgnitionTick;     // Timer tick when ignition fired
  volatile uint8_t inBlindWindow;         // Currently in blind window

  // Per-Cycle Correction - phase-lock ignition to actual crank position
  volatile uint32_t predictedPeriod;      // Period used for scheduling
  volatile int16_t phaseCorrectionUs;     // Accumulated phase correction (µs x10)

  // dRPM Compensation - predictive ignition for acceleration
  volatile uint16_t prevRpm;              // Previous cycle RPM
  volatile int16_t dRpm;                  // RPM change rate (RPM per cycle)
};

// Calibration runtime state with Fuzzy Logic support
struct __attribute__((aligned(4))) CalibrationState {
  // === MODE AND POSITION ===
  volatile uint8_t mode;                  // CAL_MODE_OFF, CAL_MODE_RUNNING, etc.
  volatile uint8_t currentTimingDeg;      // Current timing degree (0-60)
  volatile uint8_t currentRpmIdx;         // Current RPM index (0-80)
  volatile uint8_t phase;                 // 0=accel sweep, 1=done with this timing

  // === RPM GENERATOR STATE ===
  volatile uint32_t rpmPeriodTicks;       // Current half-period for TIM16 output
  volatile uint16_t targetRpm;            // Target RPM for generator
  volatile uint8_t rpmStable;             // RPM has stabilized

  // === IGNITION CAPTURE STATE ===
  volatile uint32_t triggerTime;          // When trigger was sent (timer ticks)
  volatile uint32_t captureTime;          // When ignition was captured
  volatile uint32_t capturedDelay;        // Delay from trigger to ignition (ticks)
  volatile uint8_t captureReady;          // New capture available

  // === FUZZY LOGIC CONVERGENCE STATE ===
  volatile int32_t accumulatedOffset;     // In millidegrees (0.001°) for precision
  volatile int32_t lastErrors[5];         // Error history for convergence check
  volatile uint8_t errorHistoryIdx;       // Circular buffer index
  volatile uint8_t iteration;             // Current iteration at this point
  volatile uint8_t convergedCount;        // Consecutive stable measurements

  // === MEASUREMENT ACCUMULATOR ===
  volatile int32_t sampleSum;             // Sum of error samples (millidegrees)
  volatile uint8_t sampleCount;           // Number of samples collected
  volatile int32_t lastAvgError;          // Last calculated average error

  // === STATISTICS ===
  volatile uint32_t totalPoints;          // Total points (4941)
  volatile uint32_t completedPoints;      // Points completed
  volatile uint32_t passedPoints;         // Points within margin
  volatile uint32_t failedPoints;         // Points that hit max iterations
  volatile uint32_t totalSamples;         // Total samples collected

  // === MISFIRE DETECTION ===
  volatile uint32_t triggersSent;         // Triggers sent since last capture
  volatile uint32_t lastTriggerMs;        // When last trigger was sent
  volatile uint32_t misfireCount;         // Misfires at current point
  volatile uint32_t totalMisfires;        // Total misfires across all points
  volatile uint32_t misfirePoints;        // Points with excessive misfires

  // === TIMING ===
  volatile uint32_t lastStateChangeMs;    // For state machine timing
  volatile uint32_t pointStartMs;         // When current point started
  volatile uint32_t settleStartMs;        // When RPM settle started

  // === OVERRIDE FLAG ===
  volatile uint8_t overrideTimingEnabled; // 1 = override map timing with target
  volatile int16_t overrideTimingScaled;  // Target timing in 0.01° units

  // === CAPTURE POLLING (main loop, NOT ISR) ===
  volatile uint8_t waitingForCapture;     // 1 = waiting for ignition capture
  volatile uint8_t lastPb3State;          // Previous PB3 state for edge detection

  // === DEBUG ===
  volatile uint32_t debugCaptureCount;    // Count of EXTI captures (for debug)
  volatile uint32_t debugTriggerCount;    // Count of triggers sent (for debug)
  volatile uint32_t debugRawExtiCount;    // Count of ALL EXTI fires (even when not calibrating)
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
static CalibrationState calState __attribute__((aligned(4)));
static CalibrationData calData __attribute__((aligned(4)));  // ~10.5KB - 2D calibration map

// Calibration data loaded flag (separate from calData.complete)
static uint8_t calDataLoaded = 0;

// Timer pointers
static TIM_TypeDef* TIM_CAPTURE = TIM2;   // 32-bit timer for input capture
static TIM_TypeDef* TIM_IGNITION = TIM5;  // Timer for ignition delay (32-bit for predictive mode)
static TIM_TypeDef* TIM_PULSE = TIM4;     // Timer for CDI pulse width (replaces NOP loop)

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

// Binary telemetry mode flag (0 = ASCII, 1 = Binary)
// Declared early so it can be used in processUSB() which is defined before sendRealtimeData()
static uint8_t binaryTelemetryMode = 0;

// ============================================================================
// FLASH STORAGE FOR DEFAULTS (Settings + Safety Map)
// ============================================================================
// Store settings and ONE safety map in internal Flash as fallback when SD fails
// Settings and Map are saved/loaded independently
// Write cycles: ~10,000 (more than enough for occasional saves)

// STM32H562 Flash: 1MB total (2 banks x 512KB)
// Bank 1: 0x08000000 - 0x0807FFFF (sectors 0-63)
// Bank 2: 0x08080000 - 0x080FFFFF (sectors 64-127) - NOT accessible by default!
// Use last sector of Bank 1 for user data (sector 63)
#define FLASH_USER_SECTOR       63                            // Last sector of Bank 1
#define FLASH_USER_START_ADDR   0x0807E000UL                  // Last 8KB of Bank 1 (504-512KB)
#define FLASH_DEFAULTS_MAGIC    0xCDF1A533UL                  // Magic v4 - checksum at end of struct
#define FLASH_DEFAULTS_VERSION  3                             // Version 3 = settings + map

// Flash Calibration Storage - Uses sectors 61-62 (16KB) for ~10.5KB CalibrationData
// Sector 61: 0x0807A000 - 0x0807BFFF (8KB)
// Sector 62: 0x0807C000 - 0x0807DFFF (8KB)
#define FLASH_CAL_SECTOR_1      61
#define FLASH_CAL_SECTOR_2      62
#define FLASH_CAL_START_ADDR    0x0807A000UL                  // 16KB for calibration (sectors 61-62)
#define FLASH_CAL_MAGIC         0xCA1F1A55UL                  // Magic for Flash calibration
#define FLASH_CAL_VERSION       1

// Flags to indicate what's stored in Flash (can be combined)
#define FLASH_HAS_SETTINGS      0x01  // Basic settings (trigger, limiter, warning, etc)
#define FLASH_HAS_MAP           0x02  // Ignition timing map
#define FLASH_HAS_QS            0x04  // Quick Shifter config + cut time map

// Structure to store in Flash - settings + 1 safety map
// IMPORTANT: Must be 16-byte aligned for STM32H5 QUADWORD Flash programming
// IMPORTANT: checksum MUST be the LAST field (after padding)
struct __attribute__((aligned(16))) FlashDefaults {
  uint32_t magic;                           // FLASH_DEFAULTS_MAGIC if valid
  uint8_t version;                          // Structure version
  uint8_t writeCount;                       // Track number of writes (for UI warning)
  uint8_t sourceMapNum;                     // Which map was saved (1-6, 0=none)
  uint8_t flags;                            // FLASH_HAS_SETTINGS | FLASH_HAS_MAP | FLASH_HAS_QS

  // Settings (without maps)
  TriggerConfig trigger;
  RevLimiterConfig revLimiter;
  ShiftLightConfig shiftLight;
  ADCCalibration adcCal;
  CrankingConfig cranking;
  WarningConfig warning;
  QuickShifterConfig quickShifter;          // Quick shifter config + map
  uint8_t engineType;
  uint8_t reserved[3];                      // Alignment padding

  // Safety map - 81 bytes
  int8_t safetyMap[RPM_TABLE_SIZE];

  // Padding BEFORE checksum to make struct size multiple of 16 bytes
  // Total: 196 (data) + 8 (padding) + 4 (checksum) = 208 = 13 x 16 ✓
  uint8_t padding[8];

  uint32_t checksum;                        // Checksum MUST BE LAST - validates all bytes before it
};

// Global to track Flash state
static uint8_t flashWriteCount = 0;
static uint8_t flashSourceMap = 0;   // Which map is stored (0 = none)
static uint8_t flashFlags = 0;       // What's stored in Flash

// Calculate checksum for Flash defaults
uint32_t calculateFlashChecksum(FlashDefaults* fd) {
  uint32_t sum = 0;
  uint8_t* ptr = (uint8_t*)fd;
  for (size_t i = 0; i < sizeof(FlashDefaults) - sizeof(uint32_t); i++) {
    sum += ptr[i];
  }
  return sum ^ 0xF1A50DEF;
}

// Safe Flash read using volatile pointer (required for STM32H5 TrustZone)
static bool flashAccessible = true;  // Assume accessible until proven otherwise

bool testFlashAccess(void) {
  // Test if Flash region is accessible by reading first word
  volatile uint32_t* testAddr = (volatile uint32_t*)FLASH_USER_START_ADDR;

  // Try to read - if this fails, MCU will hard fault
  uint32_t testVal = *testAddr;

  // If we get here, Flash is readable
  (void)testVal;  // Suppress unused warning
  return true;
}

// Check if Flash has valid data
// Using Bank 1 sector 63 (0x0807E000) - Bank 2 not accessible by default
bool hasCustomDefaults(void) {
  if (!flashAccessible) return false;

  // Use volatile pointer to prevent optimization issues on STM32H5
  volatile FlashDefaults* fd = (volatile FlashDefaults*)FLASH_USER_START_ADDR;

  // Read magic with volatile access
  uint32_t magic = fd->magic;
  if (magic != FLASH_DEFAULTS_MAGIC) return false;

  uint8_t version = fd->version;
  if (version != FLASH_DEFAULTS_VERSION) return false;

  // Calculate checksum by reading entire structure through volatile
  uint32_t sum = 0;
  volatile uint8_t* ptr = (volatile uint8_t*)fd;
  for (size_t i = 0; i < sizeof(FlashDefaults) - sizeof(uint32_t); i++) {
    sum += ptr[i];
  }
  uint32_t calcChecksum = sum ^ 0xF1A50DEF;

  uint32_t storedChecksum = fd->checksum;
  if (storedChecksum != calcChecksum) return false;

  return true;
}

// Check if Flash has settings
bool hasFlashSettings(void) {
  if (!hasCustomDefaults()) return false;
  volatile FlashDefaults* fd = (volatile FlashDefaults*)FLASH_USER_START_ADDR;
  return (fd->flags & FLASH_HAS_SETTINGS) != 0;
}

// Check if Flash has safety map
bool hasFlashMap(void) {
  if (!hasCustomDefaults()) return false;
  volatile FlashDefaults* fd = (volatile FlashDefaults*)FLASH_USER_START_ADDR;
  return (fd->flags & FLASH_HAS_MAP) != 0;
}

// Check if Flash has QS config
bool hasFlashQS(void) {
  if (!hasCustomDefaults()) return false;
  volatile FlashDefaults* fd = (volatile FlashDefaults*)FLASH_USER_START_ADDR;
  return (fd->flags & FLASH_HAS_QS) != 0;
}

// Get write count from Flash
uint8_t getFlashWriteCount(void) {
  if (!hasCustomDefaults()) return 0;
  volatile FlashDefaults* fd = (volatile FlashDefaults*)FLASH_USER_START_ADDR;
  return fd->writeCount;
}

// Get source map number from Flash
uint8_t getFlashSourceMap(void) {
  if (!hasCustomDefaults()) return 0;
  volatile FlashDefaults* fd = (volatile FlashDefaults*)FLASH_USER_START_ADDR;
  return fd->sourceMapNum;
}

// Get flags from Flash
uint8_t getFlashFlags(void) {
  if (!hasCustomDefaults()) return 0;
  volatile FlashDefaults* fd = (volatile FlashDefaults*)FLASH_USER_START_ADDR;
  return fd->flags;
}

// Load current Flash data into a structure (for partial updates)
// Uses volatile read then copies to non-volatile destination
void loadFlashData(FlashDefaults* dest) {
  if (hasCustomDefaults()) {
    // Use volatile-safe copy helper for STM32H5 compatibility
    copyFromFlash(dest, (volatile void*)FLASH_USER_START_ADDR, sizeof(FlashDefaults));
  } else {
    memset(dest, 0, sizeof(FlashDefaults));
    dest->magic = FLASH_DEFAULTS_MAGIC;
    dest->version = FLASH_DEFAULTS_VERSION;
  }
}

// Write FlashDefaults to Flash
bool writeFlashData(FlashDefaults* fd) {
  __disable_irq();
  HAL_FLASH_Unlock();

  // Erase sector
  FLASH_EraseInitTypeDef eraseInit;
  uint32_t sectorError = 0;
  eraseInit.TypeErase = FLASH_TYPEERASE_SECTORS;
  eraseInit.Banks = FLASH_BANK_1;
  eraseInit.Sector = FLASH_USER_SECTOR;
  eraseInit.NbSectors = 1;

  HAL_StatusTypeDef status = HAL_FLASHEx_Erase(&eraseInit, &sectorError);
  if (status != HAL_OK) {
    HAL_FLASH_Lock();
    __enable_irq();
    return false;
  }

  // Write data in 16-byte chunks
  uint32_t dest = FLASH_USER_START_ADDR;
  size_t writeSize = ((sizeof(FlashDefaults) + 15) / 16) * 16;

  for (size_t offset = 0; offset < writeSize; offset += 16) {
    status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_QUADWORD, dest + offset, (uint32_t)((uint8_t*)fd + offset));
    if (status != HAL_OK) {
      HAL_FLASH_Lock();
      __enable_irq();
      return false;
    }
  }

  HAL_FLASH_Lock();
  __enable_irq();
  return true;
}

// Helper: Copy from volatile Flash to RAM (memcpy doesn't work with volatile)
void copyFromFlash(void* dest, volatile void* src, size_t len) {
  volatile uint8_t* s = (volatile uint8_t*)src;
  uint8_t* d = (uint8_t*)dest;
  for (size_t i = 0; i < len; i++) {
    d[i] = s[i];
  }
}

// READ ONLY: Get settings stored in Flash (does NOT overwrite active config)
// Returns data for UI display only
void getFlashSettingsReadOnly(void) {
  if (!hasFlashSettings()) {
    USB_SERIAL.println(F("FLASHSETTINGS:NO_DATA"));
    return;
  }

  volatile FlashDefaults* fd = (volatile FlashDefaults*)FLASH_USER_START_ADDR;

  // Read and send each setting - UI will parse and display
  // Format: FLASHSETTINGS:trigAngle,edge,noiseFilter,pulse,soft,med,hard,full,shiftOn,shiftBlink,shiftFast,
  //         crankEn,crankTiming,crankMax,warnOverrev,warnOverheat,warnLowBatt,warnRetard,retardStart,retardPer10,engType,
  //         calTempOffset,calTempScale,calBattScale,calChargingScale
  USB_SERIAL.print(F("FLASHSETTINGS:"));
  USB_SERIAL.print(fd->trigger.triggerAngleScaled); USB_SERIAL.print(F(","));
  USB_SERIAL.print(fd->trigger.risingEdge); USB_SERIAL.print(F(","));
  USB_SERIAL.print(fd->trigger.noiseFilterTicks); USB_SERIAL.print(F(","));
  USB_SERIAL.print(fd->trigger.cdiPulseUs); USB_SERIAL.print(F(","));
  USB_SERIAL.print(fd->revLimiter.softRpm); USB_SERIAL.print(F(","));
  USB_SERIAL.print(fd->revLimiter.mediumRpm); USB_SERIAL.print(F(","));
  USB_SERIAL.print(fd->revLimiter.hardRpm); USB_SERIAL.print(F(","));
  USB_SERIAL.print(fd->revLimiter.fullCutRpm); USB_SERIAL.print(F(","));
  USB_SERIAL.print(fd->shiftLight.onRpm); USB_SERIAL.print(F(","));
  USB_SERIAL.print(fd->shiftLight.blinkRpm); USB_SERIAL.print(F(","));
  USB_SERIAL.print(fd->shiftLight.fastBlinkRpm); USB_SERIAL.print(F(","));
  USB_SERIAL.print(fd->cranking.enabled); USB_SERIAL.print(F(","));
  USB_SERIAL.print(fd->cranking.timingScaled); USB_SERIAL.print(F(","));
  USB_SERIAL.print(fd->cranking.maxRpm); USB_SERIAL.print(F(","));
  USB_SERIAL.print(fd->warning.overRevRpm); USB_SERIAL.print(F(","));
  USB_SERIAL.print(fd->warning.overheatTempC); USB_SERIAL.print(F(","));
  USB_SERIAL.print(fd->warning.lowBatteryMv); USB_SERIAL.print(F(","));
  USB_SERIAL.print(fd->warning.overheatRetard); USB_SERIAL.print(F(","));
  USB_SERIAL.print(fd->warning.retardStartTempC); USB_SERIAL.print(F(","));
  USB_SERIAL.print(fd->warning.retardPer10C); USB_SERIAL.print(F(","));
  USB_SERIAL.print(fd->engineType); USB_SERIAL.print(F(","));
  // ADC Calibration (4 fields)
  USB_SERIAL.print(fd->adcCal.tempOffsetScaled); USB_SERIAL.print(F(","));
  USB_SERIAL.print(fd->adcCal.tempScaleScaled); USB_SERIAL.print(F(","));
  USB_SERIAL.print(fd->adcCal.battScaleScaled); USB_SERIAL.print(F(","));
  USB_SERIAL.println(fd->adcCal.chargingScaleScaled);
}

// READ ONLY: Get QS config stored in Flash (does NOT overwrite active config)
void getFlashQSReadOnly(void) {
  if (!hasFlashQS()) {
    USB_SERIAL.println(F("FLASHQS:NO_DATA"));
    return;
  }

  volatile FlashDefaults* fd = (volatile FlashDefaults*)FLASH_USER_START_ADDR;

  // Format: FLASHQS:enabled,sensitivity,baseline,threshold,minRpm,maxRpm
  USB_SERIAL.print(F("FLASHQS:"));
  USB_SERIAL.print(fd->quickShifter.enabled); USB_SERIAL.print(F(","));
  USB_SERIAL.print(fd->quickShifter.sensitivity); USB_SERIAL.print(F(","));
  USB_SERIAL.print(fd->quickShifter.baseline); USB_SERIAL.print(F(","));
  USB_SERIAL.print(fd->quickShifter.threshold); USB_SERIAL.print(F(","));
  USB_SERIAL.print(fd->quickShifter.minRpm); USB_SERIAL.print(F(","));
  USB_SERIAL.println(fd->quickShifter.maxRpm);
}

// READ ONLY: Get QS map stored in Flash
void getFlashQSMapReadOnly(void) {
  if (!hasFlashQS()) {
    USB_SERIAL.println(F("FLASHQSMAP:NO_DATA"));
    return;
  }

  volatile FlashDefaults* fd = (volatile FlashDefaults*)FLASH_USER_START_ADDR;

  USB_SERIAL.print(F("FLASHQSMAP:"));
  for (int i = 0; i < QS_TABLE_SIZE; i++) {
    if (i > 0) USB_SERIAL.print(F(","));
    USB_SERIAL.print(fd->quickShifter.cutTimeMap[i]);
  }
  USB_SERIAL.println();
}

// READ ONLY: Get ignition map stored in Flash
void getFlashMapReadOnly(void) {
  if (!hasFlashMap()) {
    USB_SERIAL.println(F("FLASHMAP:NO_DATA"));
    return;
  }

  volatile FlashDefaults* fd = (volatile FlashDefaults*)FLASH_USER_START_ADDR;

  USB_SERIAL.print(F("FLASHMAP:"));
  USB_SERIAL.print(fd->sourceMapNum); USB_SERIAL.print(F(":"));
  for (int i = 0; i < RPM_TABLE_SIZE; i++) {
    if (i > 0) USB_SERIAL.print(F(","));
    USB_SERIAL.print(fd->safetyMap[i]);
  }
  USB_SERIAL.println();
}

// Load basic settings from Flash (overwrites active MCU config)
// Note: This is kept for backward compatibility, but UI now uses GET FLASHSETTINGS (read-only)
bool loadSettingsFromFlash(void) {
  if (!hasFlashSettings()) {
    USB_SERIAL.println(F("LOADSETTINGS:NO_DATA"));
    return false;
  }

  volatile FlashDefaults* fd = (volatile FlashDefaults*)FLASH_USER_START_ADDR;

  // Copy settings (NOT QS, NOT map)
  copyFromFlash(&config.trigger, (volatile void*)&fd->trigger, sizeof(TriggerConfig));
  copyFromFlash(&config.revLimiter, (volatile void*)&fd->revLimiter, sizeof(RevLimiterConfig));
  copyFromFlash(&config.shiftLight, (volatile void*)&fd->shiftLight, sizeof(ShiftLightConfig));
  copyFromFlash(&config.adcCal, (volatile void*)&fd->adcCal, sizeof(ADCCalibration));
  copyFromFlash(&config.cranking, (volatile void*)&fd->cranking, sizeof(CrankingConfig));
  copyFromFlash(&config.warning, (volatile void*)&fd->warning, sizeof(WarningConfig));
  config.engineType = fd->engineType;

  // Update trigger edge
  updateTriggerEdge();

  // Recalculate checksum
  config.checksum = calculateChecksum(&config);

  USB_SERIAL.println(F("LOADSETTINGS:OK"));
  return true;
}

// Load QS config + map from Flash
bool loadQSFromFlash(void) {
  if (!hasFlashQS()) {
    USB_SERIAL.println(F("LOADQS:NO_DATA"));
    return false;
  }

  volatile FlashDefaults* fd = (volatile FlashDefaults*)FLASH_USER_START_ADDR;

  // Copy QS config + map
  copyFromFlash(&config.quickShifter, (volatile void*)&fd->quickShifter, sizeof(QuickShifterConfig));

  // Recalculate checksum
  config.checksum = calculateChecksum(&config);

  USB_SERIAL.println(F("LOADQS:OK"));
  return true;
}

// Load SAFETY MAP from Flash into ALL maps in config
bool loadMapFromFlash(void) {
  if (!hasFlashMap()) {
    USB_SERIAL.println(F("LOADMAP:NO_DATA"));
    return false;
  }

  volatile FlashDefaults* fd = (volatile FlashDefaults*)FLASH_USER_START_ADDR;

  // Copy safety map to ALL 6 maps using volatile-safe copy
  for (int m = 0; m < NUM_MAPS; m++) {
    copyFromFlash(config.timingMaps[m], (volatile void*)fd->safetyMap, RPM_TABLE_SIZE);
  }

  // Update globals
  flashSourceMap = fd->sourceMapNum;

  USB_SERIAL.print(F("LOADMAP:OK,"));
  USB_SERIAL.println(flashSourceMap);
  return true;
}

// Load ALL defaults from Flash (used at startup when SD fails)
// Loads: settings + QS + map (whatever is available)
bool loadDefaultsFromFlash(void) {
  if (!hasCustomDefaults()) {
    USB_SERIAL.println(F("No defaults in Flash"));
    return false;
  }

  volatile FlashDefaults* fd = (volatile FlashDefaults*)FLASH_USER_START_ADDR;

  // Load basic settings if available
  if (fd->flags & FLASH_HAS_SETTINGS) {
    copyFromFlash(&config.trigger, (volatile void*)&fd->trigger, sizeof(TriggerConfig));
    copyFromFlash(&config.revLimiter, (volatile void*)&fd->revLimiter, sizeof(RevLimiterConfig));
    copyFromFlash(&config.shiftLight, (volatile void*)&fd->shiftLight, sizeof(ShiftLightConfig));
    copyFromFlash(&config.adcCal, (volatile void*)&fd->adcCal, sizeof(ADCCalibration));
    copyFromFlash(&config.cranking, (volatile void*)&fd->cranking, sizeof(CrankingConfig));
    copyFromFlash(&config.warning, (volatile void*)&fd->warning, sizeof(WarningConfig));
    config.engineType = fd->engineType;
  }

  // Load QS config + map if available (separate from settings)
  if (fd->flags & FLASH_HAS_QS) {
    copyFromFlash(&config.quickShifter, (volatile void*)&fd->quickShifter, sizeof(QuickShifterConfig));
  }

  // Load ignition map if available
  // Flash only has 1 safety map - copy to all 6 slots
  if (fd->flags & FLASH_HAS_MAP) {
    for (int m = 0; m < NUM_MAPS; m++) {
      copyFromFlash(config.timingMaps[m], (volatile void*)fd->safetyMap, RPM_TABLE_SIZE);
    }
    // Flash only has 1 map, so always use map slot 0
    config.activeMap = 0;
  }

  // Update globals
  flashWriteCount = fd->writeCount;
  flashSourceMap = fd->sourceMapNum;
  flashFlags = fd->flags;

  // Recalculate config checksum
  config.checksum = calculateChecksum(&config);
  updatePeriodThresholds();
  updatePulseWidth();

  USB_SERIAL.println(F("Defaults loaded from Flash"));
  return true;
}

// Save SETTINGS to Flash (basic settings only, NOT QS - preserves existing map/QS)
bool saveSettingsToFlash(void) {
  if (runtime.currentRpm >= 100) {
    USB_SERIAL.println(F("SAVESETTINGS:BUSY"));
    return false;
  }

  // Load existing Flash data (to preserve map and QS if any)
  // MUST be 16-byte aligned for STM32H5 QUADWORD Flash programming
  __attribute__((aligned(16))) FlashDefaults fd;
  memset(&fd, 0, sizeof(fd));  // Clear padding bytes
  loadFlashData(&fd);

  // Update settings ONLY (not QS - that's separate)
  memcpy(&fd.trigger, &config.trigger, sizeof(TriggerConfig));
  memcpy(&fd.revLimiter, &config.revLimiter, sizeof(RevLimiterConfig));
  memcpy(&fd.shiftLight, &config.shiftLight, sizeof(ShiftLightConfig));
  memcpy(&fd.adcCal, &config.adcCal, sizeof(ADCCalibration));
  memcpy(&fd.cranking, &config.cranking, sizeof(CrankingConfig));
  memcpy(&fd.warning, &config.warning, sizeof(WarningConfig));
  fd.engineType = config.engineType;
  // NOTE: QuickShifter NOT saved here - use SAVEQS command

  // Update metadata
  fd.writeCount = flashWriteCount + 1;
  fd.flags |= FLASH_HAS_SETTINGS;
  fd.checksum = calculateFlashChecksum(&fd);

  if (!writeFlashData(&fd)) {
    USB_SERIAL.println(F("SAVESETTINGS:FAIL"));
    return false;
  }

  flashWriteCount = fd.writeCount;
  flashFlags = fd.flags;

  USB_SERIAL.print(F("SAVESETTINGS:OK,"));
  USB_SERIAL.println(flashWriteCount);
  return true;
}

// Save QS config + map to Flash (preserves existing settings/map)
bool saveQSToFlash(void) {
  if (runtime.currentRpm >= 100) {
    USB_SERIAL.println(F("SAVEQS:BUSY"));
    return false;
  }

  // Load existing Flash data (to preserve settings and map)
  __attribute__((aligned(16))) FlashDefaults fd;
  memset(&fd, 0, sizeof(fd));
  loadFlashData(&fd);

  // Update QS config + map
  memcpy(&fd.quickShifter, &config.quickShifter, sizeof(QuickShifterConfig));

  // Update metadata
  fd.writeCount = flashWriteCount + 1;
  fd.flags |= FLASH_HAS_QS;
  fd.checksum = calculateFlashChecksum(&fd);

  if (!writeFlashData(&fd)) {
    USB_SERIAL.println(F("SAVEQS:FAIL"));
    return false;
  }

  flashWriteCount = fd.writeCount;
  flashFlags = fd.flags;

  USB_SERIAL.print(F("SAVEQS:OK,"));
  USB_SERIAL.println(flashWriteCount);
  return true;
}

// Save MAP to Flash (preserves existing settings if any)
bool saveMapToFlash(uint8_t mapNum) {
  if (mapNum < 1 || mapNum > NUM_MAPS) {
    USB_SERIAL.println(F("SAVEMAP:INVALID"));
    return false;
  }

  if (runtime.currentRpm >= 100) {
    USB_SERIAL.println(F("SAVEMAP:BUSY"));
    return false;
  }

  // Load existing Flash data (to preserve settings if any)
  // MUST be 16-byte aligned for STM32H5 QUADWORD Flash programming
  __attribute__((aligned(16))) FlashDefaults fd;
  memset(&fd, 0, sizeof(fd));  // Clear padding bytes
  loadFlashData(&fd);

  // Update map
  memcpy(fd.safetyMap, config.timingMaps[mapNum - 1], RPM_TABLE_SIZE);
  fd.sourceMapNum = mapNum;

  // Update metadata
  fd.writeCount = flashWriteCount + 1;
  fd.flags |= FLASH_HAS_MAP;
  fd.checksum = calculateFlashChecksum(&fd);

  if (!writeFlashData(&fd)) {
    USB_SERIAL.println(F("SAVEMAP:FAIL"));
    return false;
  }

  flashWriteCount = fd.writeCount;
  flashSourceMap = mapNum;
  flashFlags = fd.flags;

  USB_SERIAL.print(F("SAVEMAP:OK,"));
  USB_SERIAL.print(mapNum);
  USB_SERIAL.print(F(","));
  USB_SERIAL.println(flashWriteCount);
  return true;
}

// Clear all Flash defaults
bool clearDefaultsFromFlash(void) {
  if (runtime.currentRpm >= 100) {
    USB_SERIAL.println(F("CLEARDEFAULT:BUSY"));
    return false;
  }

  __disable_irq();
  HAL_FLASH_Unlock();

  FLASH_EraseInitTypeDef eraseInit;
  uint32_t sectorError = 0;
  eraseInit.TypeErase = FLASH_TYPEERASE_SECTORS;
  eraseInit.Banks = FLASH_BANK_1;
  eraseInit.Sector = FLASH_USER_SECTOR;
  eraseInit.NbSectors = 1;

  HAL_StatusTypeDef status = HAL_FLASHEx_Erase(&eraseInit, &sectorError);

  HAL_FLASH_Lock();
  __enable_irq();

  if (status != HAL_OK) {
    USB_SERIAL.println(F("CLEARDEFAULT:FAIL"));
    return false;
  }

  flashWriteCount = 0;
  flashSourceMap = 0;
  flashFlags = 0;
  USB_SERIAL.println(F("CLEARDEFAULT:OK"));
  return true;
}

// ============================================================================
// FLASH CALIBRATION STORAGE - Backup for SD Card failure
// ============================================================================

// Config source tracking
#define CAL_SOURCE_SD       0   // Loaded from SD card
#define CAL_SOURCE_FLASH    1   // Loaded from Flash backup
#define CAL_SOURCE_DEFAULT  2   // Using hardcoded defaults (all zeros)

static uint8_t calSource = CAL_SOURCE_DEFAULT;

// Initialize calibration data with hardcoded defaults (all zeros)
void initHardcodedCalibration(void) {
  memset(&calData, 0, sizeof(CalibrationData));
  calData.magic = CAL_MAGIC;
  calData.version = CAL_VERSION;
  calData.enabled = 0;
  calData.marginError = CAL_DEFAULT_MARGIN;
  calData.complete = 0;
  // All offsets are already 0 from memset
  // All calibrated bitmap bits are 0 (uncalibrated)
  // All misfireProne bitmap bits are 0 (no misfires)
  calData.checksum = calculateCalChecksum(&calData);
  calDataLoaded = 1;  // Mark as loaded (defaults are valid)
  calSource = CAL_SOURCE_DEFAULT;
}

// Check if Flash calibration is valid
bool hasFlashCalibration(void) {
  if (!flashAccessible) return false;

  volatile uint32_t* magic = (volatile uint32_t*)FLASH_CAL_START_ADDR;
  if (*magic != FLASH_CAL_MAGIC) return false;

  // Verify checksum
  volatile CalibrationData* fcd = (volatile CalibrationData*)FLASH_CAL_START_ADDR;

  // Calculate checksum using volatile read
  uint32_t sum = 0;
  volatile uint8_t* ptr = (volatile uint8_t*)fcd;
  for (size_t i = 0; i < sizeof(CalibrationData) - sizeof(uint32_t); i++) {
    sum += ptr[i];
  }
  uint32_t calcChecksum = sum ^ CAL_MAGIC;

  return (calcChecksum == fcd->checksum);
}

// Load calibration from Flash
bool loadCalibrationFromFlash(void) {
  if (!hasFlashCalibration()) {
    return false;
  }

  // Copy from Flash to RAM using volatile-safe copy
  copyFromFlash(&calData, (volatile void*)FLASH_CAL_START_ADDR, sizeof(CalibrationData));

  // Verify after copy
  if (!validateCalData(&calData)) {
    return false;
  }

  calDataLoaded = 1;
  calSource = CAL_SOURCE_FLASH;
  return true;
}

// Save calibration to Flash (erases 2 sectors)
bool saveCalibrationToFlash(void) {
  if (runtime.currentRpm >= 100) {
    USB_SERIAL.println(F("FLASHCAL:BUSY"));
    return false;
  }

  // Update checksum before saving
  calData.checksum = calculateCalChecksum(&calData);

  __disable_irq();
  HAL_FLASH_Unlock();

  // Erase sector 61
  FLASH_EraseInitTypeDef eraseInit;
  uint32_t sectorError = 0;
  eraseInit.TypeErase = FLASH_TYPEERASE_SECTORS;
  eraseInit.Banks = FLASH_BANK_1;
  eraseInit.Sector = FLASH_CAL_SECTOR_1;
  eraseInit.NbSectors = 1;

  HAL_StatusTypeDef status = HAL_FLASHEx_Erase(&eraseInit, &sectorError);
  if (status != HAL_OK) {
    HAL_FLASH_Lock();
    __enable_irq();
    USB_SERIAL.println(F("FLASHCAL:ERASE1_FAIL"));
    return false;
  }

  // Erase sector 62
  eraseInit.Sector = FLASH_CAL_SECTOR_2;
  status = HAL_FLASHEx_Erase(&eraseInit, &sectorError);
  if (status != HAL_OK) {
    HAL_FLASH_Lock();
    __enable_irq();
    USB_SERIAL.println(F("FLASHCAL:ERASE2_FAIL"));
    return false;
  }

  // Write data in 16-byte QUADWORD chunks (required for STM32H5)
  uint32_t dest = FLASH_CAL_START_ADDR;
  size_t writeSize = ((sizeof(CalibrationData) + 15) / 16) * 16;

  // Use aligned buffer for Flash write
  __attribute__((aligned(16))) uint8_t alignedBuf[16];

  for (size_t offset = 0; offset < writeSize; offset += 16) {
    // Copy 16 bytes to aligned buffer
    memset(alignedBuf, 0xFF, 16);  // Fill with 0xFF (erased state)
    size_t copyLen = (offset + 16 <= sizeof(CalibrationData)) ? 16 : (sizeof(CalibrationData) - offset);
    if (copyLen > 0 && offset < sizeof(CalibrationData)) {
      memcpy(alignedBuf, ((uint8_t*)&calData) + offset, copyLen);
    }

    status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_QUADWORD, dest + offset, (uint32_t)alignedBuf);
    if (status != HAL_OK) {
      HAL_FLASH_Lock();
      __enable_irq();
      USB_SERIAL.print(F("FLASHCAL:WRITE_FAIL,"));
      USB_SERIAL.println(offset);
      return false;
    }
  }

  HAL_FLASH_Lock();
  __enable_irq();

  // Verify
  if (!hasFlashCalibration()) {
    USB_SERIAL.println(F("FLASHCAL:VERIFY_FAIL"));
    return false;
  }

  USB_SERIAL.print(F("FLASHCAL:SAVED,"));
  USB_SERIAL.print(sizeof(CalibrationData));
  USB_SERIAL.println(F(" bytes"));
  return true;
}

// Clear calibration from Flash
bool clearCalibrationFromFlash(void) {
  if (runtime.currentRpm >= 100) {
    USB_SERIAL.println(F("FLASHCAL:BUSY"));
    return false;
  }

  __disable_irq();
  HAL_FLASH_Unlock();

  // Erase both sectors
  FLASH_EraseInitTypeDef eraseInit;
  uint32_t sectorError = 0;
  eraseInit.TypeErase = FLASH_TYPEERASE_SECTORS;
  eraseInit.Banks = FLASH_BANK_1;
  eraseInit.Sector = FLASH_CAL_SECTOR_1;
  eraseInit.NbSectors = 1;

  HAL_StatusTypeDef status = HAL_FLASHEx_Erase(&eraseInit, &sectorError);
  if (status != HAL_OK) {
    HAL_FLASH_Lock();
    __enable_irq();
    USB_SERIAL.println(F("FLASHCAL:CLEAR_FAIL"));
    return false;
  }

  eraseInit.Sector = FLASH_CAL_SECTOR_2;
  status = HAL_FLASHEx_Erase(&eraseInit, &sectorError);

  HAL_FLASH_Lock();
  __enable_irq();

  if (status != HAL_OK) {
    USB_SERIAL.println(F("FLASHCAL:CLEAR_FAIL"));
    return false;
  }

  USB_SERIAL.println(F("FLASHCAL:CLEARED"));
  return true;
}

// Get calibration source name
const char* getCalSourceName(void) {
  switch (calSource) {
    case CAL_SOURCE_SD: return "SD";
    case CAL_SOURCE_FLASH: return "Flash";
    case CAL_SOURCE_DEFAULT: return "Default";
    default: return "Unknown";
  }
}

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
  // DON'T reset counter - let it run continuously for consistent pattern
  // when hovering around limiter threshold (expert review recommendation)
  // runtime.limiterCounter stays as-is
  return 0;
}

// ============================================================================
// INTERRUPT CALLBACKS - MAXIMUM OPTIMIZATION
// ============================================================================

// HardwareTimer objects
HardwareTimer *TimerCapture;
HardwareTimer *TimerIgnition;
HardwareTimer *TimerPulse;      // For CDI pulse width (non-blocking)
HardwareTimer *TimerCalRpm;     // TIM16 for calibration RPM output
// TimerCalIgn removed - using polling on PB3 instead of TIM17

// CDI Pulse end callback - turns off CDI output after pulse duration
// This replaces the blocking NOP loop for better system responsiveness
void cdiPulseEndCallback(void) {
  CDI_LOW();
  // Timer stops automatically in one-pulse mode
}

// Fire CDI using hardware timer for pulse width (non-blocking)
// pulseUs: pulse duration in microseconds (50-250us typical)
// At 10MHz timer clock, 1 tick = 0.1us, so pulseUs * 10 = ticks
static inline void fireCdiWithTimer(void) __attribute__((always_inline));
static inline void fireCdiWithTimer(void) {
  // Record ignition time for blind window calculation
  runtime.lastIgnitionTick = TIM_CAPTURE->CNT;

  // Calculate pulse ticks from configured pulse width
  // pulseUs is 50-250, at 10MHz: ticks = pulseUs * 10
  uint32_t pulseTicks = config.trigger.cdiPulseUs * 10;

  // Set CDI HIGH
  CDI_HIGH();

  // Start TIM4 one-shot for pulse width
  TIM_PULSE->CR1 = 0;                    // Stop timer
  TIM_PULSE->CNT = 0;                    // Reset counter
  TIM_PULSE->ARR = pulseTicks;           // Set pulse duration
  TIM_PULSE->SR = 0;                     // Clear flags
  TIM_PULSE->DIER = TIM_DIER_UIE;        // Enable update interrupt
  TIM_PULSE->CR1 = TIM_CR1_CEN | TIM_CR1_OPM;  // Start one-pulse mode

  runtime.ignitionCount++;
}

// VR Trigger callback - called on input capture
void vrCaptureCallback(void) {
  // Don't process trigger until config is ready
  if (!runtime.configReady) {
    return;
  }

  // Direct register access for maximum speed
  uint32_t capture = TIM_CAPTURE->CCR1;

  // =========================================================================
  // ADAPTIVE BLIND WINDOW - Mode & RPM aware EMI filter
  // Scales with RPM to ensure blind window < gap before next trigger
  // PREDICTIVE MODE: ~0.1% of period (gap is small: timing - trigger degrees)
  // NORMAL MODE: ~3% of period (gap is ~360°, plenty of margin)
  // =========================================================================
  if (runtime.lastIgnitionTick > 0) {
    uint32_t ticksSinceIgnition = capture - runtime.lastIgnitionTick;
    uint32_t blindTicks;

    if (runtime.predictiveMode) {
      // =====================================================================
      // PREDICTIVE MODE: Adaptive blind window based on expected gap
      //
      // Gap = (timing - trigger) degrees = time before next valid trigger
      // Blind window must be < gap to avoid filtering valid triggers
      //
      // Strategy: Look up EXPECTED timing from map based on current RPM
      // This is more reliable than using previous cycle's timing
      // =====================================================================

      // Estimate RPM index from period (same method as main timing calc)
      uint8_t rpmIdx = periodToIndex(runtime.period);

      // Look up expected timing from active map
      int8_t mapTiming = config.timingMaps[config.activeMap][rpmIdx];
      int32_t expectedTimingScaled = (int32_t)mapTiming * DEG_SCALE;

      // Calculate expected gap: timing - trigger angle
      int32_t gapScaled = expectedTimingScaled - config.trigger.triggerAngleScaled;

      if (gapScaled >= 100) {  // At least 1° gap (100 = 1.00°)
        // gap_ticks = gapScaled * period / 36000
        uint32_t gapTicks = ((uint64_t)gapScaled * runtime.period) / 36000UL;

        // Blind window = 10% of gap (conservative, leaves 90% margin)
        blindTicks = gapTicks / 10;

        // Clamp to safe range
        if (blindTicks < 30) blindTicks = 30;     // min 3µs @ 10MHz
        if (blindTicks > 500) blindTicks = 500;   // max 50µs @ 10MHz
      } else {
        // Very small or zero gap - timing ≈ trigger angle
        // Use minimal blind window
        blindTicks = 30;  // 3µs @ 10MHz
      }
    } else {
      // NORMAL MODE: Larger RPM-scaled blind window
      // Gap is ~360° (full rotation), plenty of margin
      // Formula: ~3% of period, min 50µs, max 300µs
      blindTicks = runtime.period >> 5;
      if (blindTicks < 500) blindTicks = 500;    // min 50µs @ 10MHz
      if (blindTicks > 3000) blindTicks = 3000;  // max 300µs @ 10MHz
    }

    if (ticksSinceIgnition < blindTicks) {
      return;
    }
  }

  uint32_t period = capture - runtime.lastCapture;

  // FIRST TRIGGER FIX: If lastCapture is 0, this is first trigger ever
  // Just capture the timestamp and return - can't calculate valid period yet
  if (runtime.lastCapture == 0) {
    runtime.lastCapture = capture;
    return;
  }

  // Noise filter - reject too short pulses (don't update lastCapture)
  if (period < config.trigger.noiseFilterTicks) {
    return;
  }

  // Max period check - reject excessively long periods (engine stopped/sensor fail)
  // Faster detection than engine timeout in main loop
  if (period > MAX_VALID_PERIOD_TICKS) {
    runtime.engineRunning = 0;
    runtime.triggerCount = 0;  // Reset cold-start counter
    runtime.lastCapture = capture;  // Update so next trigger has valid reference
    return;
  }

  // =========================================================================
  // PERIOD VALIDATION FILTER - SMART EMI REJECTION
  // Only active at higher RPM where EMI spikes are problematic
  // At low RPM, large period changes are normal during acceleration
  // Key: Always update lastCapture so next measurement is valid
  // =========================================================================
  // 400,000 ticks = 1500 RPM at 10MHz timer
  // Only apply strict filter above 1500 RPM
  if (runtime.period > 0 && runtime.period < 400000 && runtime.engineRunning) {
    // Allow 50% to 200% of previous period (2x RPM max change per trigger)
    // This is physically realistic for racing engines at high RPM
    uint32_t minPeriod = runtime.period >> 1;        // 50% = 2x RPM jump max
    uint32_t maxPeriod = runtime.period << 1;        // 200% = 0.5x RPM drop max

    if (period < minPeriod || period > maxPeriod) {
      // EMI spike detected - reject this trigger BUT update lastCapture
      // This prevents cascade failure on next trigger
      runtime.lastCapture = capture;
      return;
    }
  }

  // Validation passed - now update lastCapture
  runtime.lastCapture = capture;

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

  // Cold start protection - RACING SIMPLIFIED
  // Just count valid triggers that passed period filter
  // Period filter already rejects EMI, no need for complex validation
  if (runtime.triggerCount < STARTUP_TRIGGER_COUNT) {
    runtime.triggerCount++;
    return;  // Don't fire yet, wait for minimum trigger count
  }

  // Skip if kill switch active
  if (runtime.killActive) {
    return;
  }

  // Skip if ignition disabled (software kill)
  if (!runtime.ignitionEnabled) {
    return;
  }

  // =========================================================================
  // CALIBRATION MODE: Bypass all limiters and cuts
  // During calibration, we need EVERY trigger to fire at exact timing
  // =========================================================================
  bool inCalibrationMode = (calState.mode == CAL_MODE_RUNNING && calState.overrideTimingEnabled);

  // Skip if quick shifter cut active (UNLESS in calibration mode)
  if (runtime.qsActive && !inCalibrationMode) {
    runtime.cutCount++;
    runtime.lastCycleWasCut = 1;
    // CRITICAL FIX: Toggle 4-stroke cycle to maintain sync during QS cut
    // Without this, cycle tracking gets out of sync and ignition fires on wrong stroke
    if (config.engineType == ENGINE_4_STROKE) {
      runtime.fourStrokeCycle ^= 1;
    }
    return;
  }

  // Rev limiter check using period (UNLESS in calibration mode)
  if (!inCalibrationMode && shouldCutByPeriod(period)) {
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

  // =========================================================================
  // PER-CYCLE PHASE CORRECTION (Expert Review Optimization)
  // Compare predicted vs actual period, correct next cycle
  // This reduces timing error from ±0.5° to ±0.1° during acceleration
  // =========================================================================
  int32_t phaseErrorTicks = 0;
  if (runtime.predictedPeriod > 0) {
    // Error = actual - predicted (positive = engine slower than expected)
    phaseErrorTicks = (int32_t)period - (int32_t)runtime.predictedPeriod;

    // Apply small gain (1/16) for stability - prevents oscillation
    int16_t correction = phaseErrorTicks >> 4;

    // Clamp correction to ±30 ticks (±3µs @ 10MHz) - safety limit
    if (correction > 30) correction = 30;
    if (correction < -30) correction = -30;

    // Accumulate (with decay to prevent drift)
    runtime.phaseCorrectionUs = (runtime.phaseCorrectionUs * 7 + correction * 10) >> 3;

    // Clamp total correction to ±50 ticks (±5µs)
    if (runtime.phaseCorrectionUs > 50) runtime.phaseCorrectionUs = 50;
    if (runtime.phaseCorrectionUs < -50) runtime.phaseCorrectionUs = -50;
  }
  // Save current period as prediction for next cycle
  runtime.predictedPeriod = period;

  // =========================================================================
  // dRPM COMPENSATION - Predictive Ignition (Expert Review Optimization)
  // Anticipate RPM change during acceleration for sharper response
  // advance_effective = advance_map + k * dRPM
  // DISABLED during calibration - we need exact timing
  // =========================================================================
  // Calculate dRPM using period difference (no division needed)
  // If period decreases (dPeriod negative), RPM is increasing
  int16_t dRpmCompensation = 0;
  if (runtime.prevRpm > 0 && rpmIndex > 0) {
    // Approximate current RPM from index (rpmIndex * 250 RPM)
    uint16_t approxRpm = rpmIndex * RPM_STEP;

    // dRPM = current - previous
    int16_t dRpm = (int16_t)approxRpm - (int16_t)runtime.prevRpm;
    runtime.dRpm = dRpm;

    // Only apply compensation if:
    // - RPM > 4000 (stable operation)
    // - Not in limiter (limiter handles its own timing)
    // - NOT in calibration mode (need exact timing)
    if (approxRpm > 4000 && runtime.limiterStage == LIMITER_NONE && !inCalibrationMode) {
      // Compensation: k * dRPM, where k ≈ 0.002 (1/512)
      // At dRPM = +500 (fast accel): compensation ≈ +1° (100 scaled)
      // Positive dRPM (accel) → advance more (positive compensation)
      // Negative dRPM (decel) → retard slightly (negative compensation)
      dRpmCompensation = dRpm >> 3;  // ~0.003 deg per RPM change

      // Clamp to ±150 scaled (±1.5°) - don't over-compensate
      if (dRpmCompensation > 150) dRpmCompensation = 150;
      if (dRpmCompensation < -150) dRpmCompensation = -150;
    }

    runtime.prevRpm = approxRpm;
  } else if (rpmIndex > 0) {
    // First valid reading - initialize
    runtime.prevRpm = rpmIndex * RPM_STEP;
  }

  // =========================================================================
  // CALIBRATION OVERRIDE - Use target timing during calibration
  // This ensures CDI fires at exact calibration target angle, not map timing
  // =========================================================================
  if (calState.overrideTimingEnabled && calState.mode == CAL_MODE_RUNNING) {
    // During calibration, use the exact target timing (no map lookup)
    runtime.currentTimingScaled = calState.overrideTimingScaled;
  }
  // Cranking mode check (low index = low RPM)
  else if (config.cranking.enabled && rpmIndex <= (config.cranking.maxRpm / RPM_STEP)) {
    runtime.currentTimingScaled = config.cranking.timingScaled * DEG_SCALE;
  } else {
    // Get timing from lookup table (NO DIVISION, NO INTERPOLATION)
    runtime.currentTimingScaled = getTimingByIndex(rpmIndex);
  }

  int16_t timingScaled = runtime.currentTimingScaled;

  // DISABLED: timingClamped was causing MCU freeze - needs investigation
  // runtime.timingClamped = (timingScaled < TIMING_MIN_SCALED || timingScaled > TIMING_MAX_SCALED) ? 1 : 0;

  // Apply dRPM compensation (add advance during acceleration)
  // Skip during calibration - timing is already exact
  if (!inCalibrationMode) {
    timingScaled += dRpmCompensation;
  }

  // Apply soft limiter timing retard (more gentle than cut)
  // Skip during calibration - we need exact timing
  if (runtime.limiterStage == LIMITER_SOFT && !inCalibrationMode) {
    timingScaled -= SOFT_RETARD_SCALED;  // Retard by 5 degrees
  }

  // Clamp timing to valid range (after all modifications)
  if (timingScaled < TIMING_MIN_SCALED) {
    timingScaled = TIMING_MIN_SCALED;  // Clamp to minimum (-10°)
  }
  if (timingScaled > TIMING_MAX_SCALED) {
    timingScaled = TIMING_MAX_SCALED;  // Clamp to maximum (60°)
  }

  // Apply 2D calibration offset if enabled (RPM × Timing interpolation)
  // calData.offsets are in 0.01° units (centidegrees), same as timingScaled
  // Uses bilinear interpolation for maximum precision
  // NOTE: Works with partial calibration too - uncalibrated points use fallback/interpolation
  // SKIP during calibration - don't apply old calibration while running new calibration
  if (config.calibration.enabled && calDataLoaded && calData.magic == CAL_MAGIC && !inCalibrationMode) {
    // Get approximate RPM from index (avoids division in ISR)
    uint16_t approxRpmForCal = rpmIndex * RPM_STEP;

    // Get interpolated offset from 2D calibration map
    // Input: rpm, timingScaled (0.01° units)
    // Output: offset in 0.01° units
    // For uncalibrated points, uses spiral search to find nearest calibrated point
    int16_t calOffset = getCalibrationOffsetInterpolated(approxRpmForCal, timingScaled);

    // Apply offset (already in same scale as timingScaled)
    timingScaled += calOffset;

    // Re-clamp after calibration offset
    if (timingScaled < TIMING_MIN_SCALED) timingScaled = TIMING_MIN_SCALED;
    if (timingScaled > TIMING_MAX_SCALED) timingScaled = TIMING_MAX_SCALED;

    // Apply additional safety retard for misfire-prone points
    // These points had misfires during calibration, so we back off timing for safety
    uint8_t timingDegForMisfire = (timingScaled > 0) ? (timingScaled / 100) : 0;
    if (timingDegForMisfire < CAL_TIMING_POINTS &&
        isPointMisfireProne(rpmIndex, timingDegForMisfire)) {
      timingScaled -= CAL_MISFIRE_SAFETY_RETARD;  // Retard by 2° for safety
      if (timingScaled < TIMING_MIN_SCALED) timingScaled = TIMING_MIN_SCALED;
    }
  }

  runtime.currentTimingScaled = timingScaled;  // Update for display

  // Calculate delay in timer ticks
  // Trigger fires at triggerAngle BTDC, need to fire at timing BTDC
  // Delay = (triggerAngle - timing) degrees
  int32_t angleDelayScaled = config.trigger.triggerAngleScaled - timingScaled;

  // RACE CONDITION HANDLING
  // If previous ignition still pending, we have two options:
  // 1. Normal mode: skip this trigger (safe, ignition will fire soon)
  // 2. Predictive mode: cancel old, schedule new (prevents stall during accel)
  if (runtime.ignitionPending) {
    // Check if this will be predictive mode
    int32_t checkAngleDelay = config.trigger.triggerAngleScaled - timingScaled;
    if (checkAngleDelay < 0) {
      // PREDICTIVE MODE: Don't skip - cancel old timer and continue
      // This prevents engine stall during fast acceleration
      TIM_IGNITION->CR1 = 0;  // Stop old timer
      runtime.ignitionPending = 0;
      // Don't increment skippedTriggers - we're handling it, not skipping
      // Continue to schedule new ignition below
    } else {
      // NORMAL MODE: Safe to skip, ignition will fire very soon
      runtime.skippedTriggers++;  // Only count actual skips
      return;
    }
  }

  uint32_t ticksPerDeg = ticksPerDegTable[rpmIndex];
  uint32_t delayTicks;

  if (angleDelayScaled < 0) {
    // =====================================================================
    // PREDICTIVE MODE: Timing more advanced than trigger angle
    // Fire on NEXT cycle: delay = 360° - (timing - triggerAngle)
    // Example: trigger=8°, timing=15° → delay = 360° - 7° = 353°
    // =====================================================================
    runtime.predictiveMode = 1;
    runtime.timingClamped = 0;  // Reset, will be set if clamping occurs

    // Calculate angle to next-cycle firing point
    // predictiveAngle = 360° + angleDelayScaled (angleDelayScaled is negative)
    // In scaled units: 36000 + angleDelayScaled
    int32_t predictiveAngleScaled = 36000 + angleDelayScaled;

    // Convert to ticks
    delayTicks = ((uint64_t)predictiveAngleScaled * ticksPerDeg) / 10000UL;

    // Apply phase correction
    int32_t correctedDelay = (int32_t)delayTicks + runtime.phaseCorrectionUs;
    if (correctedDelay < (int32_t)(period / 2)) {
      correctedDelay = period / 2;  // Minimum: fire after half rotation
    }
    delayTicks = (uint32_t)correctedDelay;

    // Sanity check: max delay is ~1.5 periods (540° for safety)
    uint32_t maxDelay = period + (period >> 1);
    if (delayTicks > maxDelay) {
      delayTicks = maxDelay;
    }

    // GENTLE ACCELERATION COMPENSATION
    // Only reduce delay proportionally to dRpm, NO minimum clamp
    // This prevents stalls while still adjusting for acceleration
    if (runtime.dRpm > 0) {
      uint32_t rpmApprox = 600000000UL / period;
      if (rpmApprox > 0) {
        // Calculate how much period will shrink
        uint32_t periodReduction = ((uint32_t)runtime.dRpm * period) / rpmApprox;
        // Add small 1% safety margin
        uint32_t safetyMargin = period / 100;
        uint32_t safeDelay = period - periodReduction - safetyMargin;

        // Only clamp if calculated delay exceeds safe delay
        // NO minimum clamp - let timing be what it needs to be
        if (delayTicks > safeDelay && safeDelay > (period / 2)) {
          runtime.timingClamped = 1;
          delayTicks = safeDelay;
        }
      }
    }

    // STEADY STATE: 99% max for accuracy (only 3.6° margin)
    if (runtime.dRpm <= 0) {
      uint32_t steadyMaxDelay = (period * 99) / 100;
      if (delayTicks > steadyMaxDelay) {
        runtime.timingClamped = 1;
        delayTicks = steadyMaxDelay;
      }
    }

  } else {
    // =====================================================================
    // NORMAL MODE: Timing within trigger angle range
    // Standard same-cycle firing
    // =====================================================================
    runtime.predictiveMode = 0;
    runtime.timingClamped = 0;  // Normal mode doesn't clamp

    // Convert angle to ticks
    delayTicks = ((uint64_t)angleDelayScaled * ticksPerDeg) / 10000UL;

    // Apply phase correction
    int32_t correctedDelay = (int32_t)delayTicks + runtime.phaseCorrectionUs;
    if (correctedDelay < 0) correctedDelay = 0;
    delayTicks = (uint32_t)correctedDelay;

    // Sanity check - max delay is half the period (normal mode)
    if (delayTicks > period / 2) {
      delayTicks = 0;
    }
  }

  // Fire ignition
  if (delayTicks < 100) {
    // Very short delay, fire immediately
    fireCdiWithTimer();
  } else {
    // Setup TIM5 (32-bit) for one-shot delay
    TIM_IGNITION->CR1 = 0;                    // Stop timer
    TIM_IGNITION->CNT = 0;                    // Reset counter
    TIM_IGNITION->ARR = delayTicks;           // Set delay (32-bit capable)
    TIM_IGNITION->SR = 0;                     // Clear flags
    TIM_IGNITION->DIER = TIM_DIER_UIE;        // Enable update interrupt
    TIM_IGNITION->CR1 = TIM_CR1_CEN | TIM_CR1_OPM;  // Start one-pulse mode
    runtime.scheduledPeriod = period;              // Store period for late fire check
    runtime.scheduledCapture = runtime.lastCapture; // Store capture time for late fire check
    runtime.ignitionPending = 1;
  }
}

// Ignition output callback - fires the CDI (called after delay timer expires)
void ignitionFireCallback(void) {
  if (runtime.ignitionPending) {
    runtime.ignitionPending = 0;

    // LATE FIRE PREVENTION - DISABLED
    // The dynamic safety clamp already ensures delay doesn't exceed safe limits
    // Late fire prevention was causing false skips, especially during acceleration
    // Trust the clamp and interrupt priorities to handle timing correctly
    //
    // NOTE: If timing issues occur, this can be re-enabled with proper threshold
    // that matches the dynamic clamp values (not a fixed 95%)

    fireCdiWithTimer();  // Non-blocking pulse using TIM4
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

  // === TIM3: Ignition delay timing ===
  TimerIgnition = new HardwareTimer(TIM5);
  TimerIgnition->setPrescaleFactor(actualPrescaler);  // Same prescaler as TIM2
  TimerIgnition->setOverflow(0xFFFFFFFF);  // 32-bit for large delays in predictive mode

  // Attach callback
  TimerIgnition->attachInterrupt(ignitionFireCallback);
  // Note: NVIC priority set in setup() after all timers initialized

  // Don't start yet - will be triggered by capture callback
  TimerIgnition->pause();

  // === TIM4: CDI pulse width timer (replaces blocking NOP loop) ===
  // This timer handles the CDI pulse duration (50-250us typical)
  // Using hardware timer instead of NOP loop prevents ISR blocking
  TimerPulse = new HardwareTimer(TIM4);
  TimerPulse->setPrescaleFactor(actualPrescaler);  // Same prescaler for 10MHz tick (0.1us resolution)
  TimerPulse->setOverflow(0xFFFF);

  // Attach callback - turns CDI LOW after pulse duration
  TimerPulse->attachInterrupt(cdiPulseEndCallback);

  // Priority is set in setup() after all timers are configured

  // Don't start yet - will be triggered when CDI fires
  TimerPulse->pause();
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
// FUZZY LOGIC ENGINE FOR CALIBRATION
// ============================================================================

// Triangular membership function
// Returns 0-1000 (0.0 to 1.0 scaled by 1000) for value position in triangle (a, b, c)
int32_t triangularMembership(int32_t value, int32_t a, int32_t b, int32_t c) {
  if (value <= a || value >= c) return 0;
  if (value <= b) return ((value - a) * 1000) / (b - a);
  return ((c - value) * 1000) / (c - b);
}

// Trapezoidal membership for edge cases
// Returns 0-1000 (0.0 to 1.0 scaled by 1000)
int32_t trapezoidalMembership(int32_t value, int32_t a, int32_t b, int32_t c, int32_t d) {
  if (value <= a || value >= d) return 0;
  if (value >= b && value <= c) return 1000;
  if (value < b) return ((value - a) * 1000) / (b - a);
  return ((d - value) * 1000) / (d - c);
}

// Calculate fuzzy adjustment based on error magnitude
// Input: error in millidegrees (0.001°)
// Output: adjustment in millidegrees (0.001°)
int32_t fuzzyCalibrationAdjust(int32_t errorMilliDeg) {
  if (errorMilliDeg == 0) return 0;

  int32_t absError = abs(errorMilliDeg);

  // Calculate membership values for each error category (0-1000 scale)
  // Using overlapping triangular/trapezoidal functions for smooth transitions
  int32_t muHuge   = trapezoidalMembership(absError, 3000, 5000, 100000, 100001);
  int32_t muLarge  = triangularMembership(absError, 1000, 3000, 5000);
  int32_t muMedium = triangularMembership(absError, 200, 750, 2000);
  int32_t muSmall  = triangularMembership(absError, 30, 150, 500);
  int32_t muTiny   = triangularMembership(absError, 3, 15, 100);
  int32_t muMicro  = trapezoidalMembership(absError, 0, 0, 5, 20);

  // Defuzzification: weighted average (centroid method)
  int32_t sumWeight = muHuge + muLarge + muMedium + muSmall + muTiny + muMicro;

  if (sumWeight < 10) {
    // No significant membership, use minimum step
    return (errorMilliDeg > 0) ? -1 : 1;
  }

  // Calculate weighted step size
  int32_t weightedSum = (
    muHuge   * FUZZY_STEP_HUGE +
    muLarge  * FUZZY_STEP_LARGE +
    muMedium * FUZZY_STEP_MEDIUM +
    muSmall  * FUZZY_STEP_SMALL +
    muTiny   * FUZZY_STEP_TINY +
    muMicro  * FUZZY_STEP_TINY
  );

  int32_t adjustment = weightedSum / sumWeight;

  // Apply adaptive gain based on error magnitude
  // Larger errors get more aggressive correction (0.5 to 1.0)
  int32_t gain = 500 + (absError * 500) / 10000;  // 0.5 + error/10° * 0.5
  if (gain > 1000) gain = 1000;

  adjustment = (adjustment * gain) / 1000;

  // Ensure minimum step of 1 millidegree
  if (adjustment < 1) adjustment = 1;

  // Apply direction (negate to compensate for error)
  return (errorMilliDeg > 0) ? -adjustment : adjustment;
}

// Check if calibration has converged for current point
// Returns: 0 = not converged, 1 = converged within margin, 2 = stable oscillation
uint8_t checkCalibrationConvergence(int32_t currentError, int32_t marginMilliDeg) {
  // Add to error history (circular buffer)
  calState.lastErrors[calState.errorHistoryIdx] = currentError;
  calState.errorHistoryIdx = (calState.errorHistoryIdx + 1) % 5;

  // Need at least 5 iterations for convergence check
  if (calState.iteration < 5) return 0;

  // Check if within margin
  if (abs(currentError) <= marginMilliDeg) {
    calState.convergedCount++;
    if (calState.convergedCount >= CAL_CONVERGE_COUNT) {
      return 1;  // Converged!
    }
  } else {
    calState.convergedCount = 0;
  }

  // Check for stable oscillation (error bouncing around target)
  int32_t sumErrors = 0;
  int32_t minError = calState.lastErrors[0];
  int32_t maxError = calState.lastErrors[0];

  for (int i = 0; i < 5; i++) {
    sumErrors += calState.lastErrors[i];
    if (calState.lastErrors[i] < minError) minError = calState.lastErrors[i];
    if (calState.lastErrors[i] > maxError) maxError = calState.lastErrors[i];
  }

  // If oscillation range is small, consider converged (use average)
  int32_t oscillation = maxError - minError;
  if (oscillation < marginMilliDeg * 2 && calState.convergedCount >= 3) {
    return 2;  // Stable oscillation, use average
  }

  return 0;  // Not converged
}

// Check if a calibration point is marked as calibrated
bool isPointCalibrated(uint8_t rpmIdx, uint8_t timingDeg) {
  if (rpmIdx >= CAL_RPM_POINTS || timingDeg >= CAL_TIMING_POINTS) return false;
  uint8_t byteIdx = timingDeg / 8;
  uint8_t bitIdx = timingDeg % 8;
  return (calData.calibrated[rpmIdx][byteIdx] & (1 << bitIdx)) != 0;
}

// Mark a calibration point as calibrated
void markPointCalibrated(uint8_t rpmIdx, uint8_t timingDeg) {
  if (rpmIdx >= CAL_RPM_POINTS || timingDeg >= CAL_TIMING_POINTS) return;
  uint8_t byteIdx = timingDeg / 8;
  uint8_t bitIdx = timingDeg % 8;
  calData.calibrated[rpmIdx][byteIdx] |= (1 << bitIdx);
}

// Check if a calibration point is marked as misfire-prone
bool isPointMisfireProne(uint8_t rpmIdx, uint8_t timingDeg) {
  if (rpmIdx >= CAL_RPM_POINTS || timingDeg >= CAL_TIMING_POINTS) return false;
  uint8_t byteIdx = timingDeg / 8;
  uint8_t bitIdx = timingDeg % 8;
  return (calData.misfireProne[rpmIdx][byteIdx] & (1 << bitIdx)) != 0;
}

// Mark a calibration point as misfire-prone
void markPointMisfireProne(uint8_t rpmIdx, uint8_t timingDeg) {
  if (rpmIdx >= CAL_RPM_POINTS || timingDeg >= CAL_TIMING_POINTS) return;
  uint8_t byteIdx = timingDeg / 8;
  uint8_t bitIdx = timingDeg % 8;
  calData.misfireProne[rpmIdx][byteIdx] |= (1 << bitIdx);
}

// Count total calibrated points from bitmap
uint32_t countCalibratedPoints(void) {
  uint32_t count = 0;
  for (uint8_t r = 0; r < CAL_RPM_POINTS; r++) {
    for (uint8_t b = 0; b < (CAL_TIMING_POINTS + 7) / 8; b++) {
      uint8_t byte = calData.calibrated[r][b];
      // Count set bits using Brian Kernighan's algorithm
      while (byte) {
        byte &= (byte - 1);
        count++;
      }
    }
  }
  return count;
}

// Count total misfire-prone points from bitmap
uint32_t countMisfirePronePoints(void) {
  uint32_t count = 0;
  for (uint8_t r = 0; r < CAL_RPM_POINTS; r++) {
    for (uint8_t b = 0; b < (CAL_TIMING_POINTS + 7) / 8; b++) {
      uint8_t byte = calData.misfireProne[r][b];
      // Count set bits using Brian Kernighan's algorithm
      while (byte) {
        byte &= (byte - 1);
        count++;
      }
    }
  }
  return count;
}

// Find nearest calibrated point using spiral search
int16_t findNearestCalibratedOffset(uint8_t rpmIdx, uint8_t timingDeg) {
  // Search in expanding squares around the target point
  for (int radius = 1; radius < 20; radius++) {
    for (int dr = -radius; dr <= radius; dr++) {
      for (int dt = -radius; dt <= radius; dt++) {
        // Only check edge of square (interior already checked)
        if (abs(dr) != radius && abs(dt) != radius) continue;

        int r = (int)rpmIdx + dr;
        int t = (int)timingDeg + dt;

        // Bounds check
        if (r < 0 || r >= CAL_RPM_POINTS) continue;
        if (t < 0 || t >= CAL_TIMING_POINTS) continue;

        if (isPointCalibrated((uint8_t)r, (uint8_t)t)) {
          return calData.offsets[r][t];
        }
      }
    }
  }

  return 0;  // No calibrated point found
}

// Bilinear interpolation for calibration offset
// Input: rpm (actual), timingScaled (0.01° units)
// Output: offset in 0.01° units (centidegrees)
int16_t getCalibrationOffsetInterpolated(uint16_t rpm, int16_t timingScaled) {
  // Convert to float-like fixed point for interpolation
  // RPM index: rpm / 250
  // Timing index: timingScaled / 100

  int32_t rpmIdx1000 = ((int32_t)rpm * 1000) / 250;  // Index × 1000
  int32_t timingIdx1000 = ((int32_t)timingScaled * 1000) / 100;  // Index × 1000

  uint8_t rpmLo = rpmIdx1000 / 1000;
  uint8_t rpmHi = rpmLo + 1;
  uint8_t timingLo = timingIdx1000 / 1000;
  uint8_t timingHi = timingLo + 1;

  // Clamp indices
  if (rpmHi >= CAL_RPM_POINTS) { rpmHi = CAL_RPM_POINTS - 1; rpmLo = rpmHi > 0 ? rpmHi - 1 : 0; }
  if (timingHi >= CAL_TIMING_POINTS) { timingHi = CAL_TIMING_POINTS - 1; timingLo = timingHi > 0 ? timingHi - 1 : 0; }
  if (timingLo < 0) timingLo = 0;

  // Get 4 corner offsets (with fallback for uncalibrated points)
  int16_t o00, o01, o10, o11;

  if (isPointCalibrated(rpmLo, timingLo)) {
    o00 = calData.offsets[rpmLo][timingLo];
  } else {
    o00 = findNearestCalibratedOffset(rpmLo, timingLo);
  }

  if (isPointCalibrated(rpmLo, timingHi)) {
    o01 = calData.offsets[rpmLo][timingHi];
  } else {
    o01 = findNearestCalibratedOffset(rpmLo, timingHi);
  }

  if (isPointCalibrated(rpmHi, timingLo)) {
    o10 = calData.offsets[rpmHi][timingLo];
  } else {
    o10 = findNearestCalibratedOffset(rpmHi, timingLo);
  }

  if (isPointCalibrated(rpmHi, timingHi)) {
    o11 = calData.offsets[rpmHi][timingHi];
  } else {
    o11 = findNearestCalibratedOffset(rpmHi, timingHi);
  }

  // Calculate interpolation fractions (0-1000 scale)
  int32_t rpmFrac = rpmIdx1000 - (int32_t)rpmLo * 1000;
  int32_t timingFrac = timingIdx1000 - (int32_t)timingLo * 1000;

  // Clamp fractions
  if (rpmFrac < 0) rpmFrac = 0;
  if (rpmFrac > 1000) rpmFrac = 1000;
  if (timingFrac < 0) timingFrac = 0;
  if (timingFrac > 1000) timingFrac = 1000;

  // Bilinear interpolation
  int32_t top = ((int32_t)o00 * (1000 - timingFrac) + (int32_t)o01 * timingFrac) / 1000;
  int32_t bot = ((int32_t)o10 * (1000 - timingFrac) + (int32_t)o11 * timingFrac) / 1000;
  int32_t result = (top * (1000 - rpmFrac) + bot * rpmFrac) / 1000;

  return (int16_t)result;
}

// ============================================================================
// CALIBRATION TIMER CALLBACKS
// ============================================================================

// TIM16 callback - toggles PB8 to generate VR-like signal
// IMPORTANT: NO BLOCKING! Just toggle and record time, capture done in main loop
void calRpmCallback(void) {
  static bool state = false;
  state = !state;

  if (state) {
    // Rising edge - send trigger pulse
    GPIOB->BSRR = (1U << 8);  // PB8 HIGH
    calState.triggerTime = TIM_CAPTURE->CNT;  // Record trigger time
    calState.triggersSent++;  // Track for misfire detection
    calState.debugTriggerCount++;  // DEBUG: total trigger count
    calState.waitingForCapture = 1;  // Signal main loop to poll for capture
  } else {
    GPIOB->BSRR = (1U << (8 + 16));  // PB8 LOW
  }
}

// Poll for ignition capture on PB3 - called from main loop (NOT ISR!)
// This is non-blocking and checks for rising edge on PB3
void pollCalibrationCapture(void) {
  // Only poll if calibration is running and waiting for capture
  if (calState.mode != CAL_MODE_RUNNING || !calState.waitingForCapture) {
    return;
  }

  // Check timeout (5ms = 50000 ticks at 10MHz)
  uint32_t now = TIM_CAPTURE->CNT;
  uint32_t trigTime = calState.triggerTime;
  uint32_t elapsed;
  if (now >= trigTime) {
    elapsed = now - trigTime;
  } else {
    elapsed = (0xFFFFFFFF - trigTime) + now + 1;
  }

  if (elapsed > 50000) {
    // Timeout - no capture detected
    calState.waitingForCapture = 0;
    return;
  }

  // Check for rising edge on PB3
  uint8_t pb3Now = (GPIOB->IDR & (1U << 3)) ? 1 : 0;
  if (pb3Now && !calState.lastPb3State) {
    // Rising edge detected - capture!
    calState.captureTime = now;
    calState.debugCaptureCount++;
    calState.debugRawExtiCount++;

    // Calculate delay from trigger to ignition
    uint32_t delay;
    if (now >= trigTime) {
      delay = now - trigTime;
    } else {
      delay = (0xFFFFFFFF - trigTime) + now + 1;
    }

    calState.capturedDelay = delay;
    calState.captureReady = 1;
    calState.triggersSent = 0;
    calState.waitingForCapture = 0;
  }
  calState.lastPb3State = pb3Now;
}

// ============================================================================
// CALIBRATION TIMER SETUP
// ============================================================================

void setupCalibrationTimers(void) {
  USB_SERIAL.println(F("Setting up calibration timers..."));
  USB_SERIAL.flush();

  // === VDDIO2 Check for PB8 ===
  // On STM32H562, PB8 is on VDDIO2 power domain (PB1 is on VDD, no issue)
  #ifdef PWR_VMSR_VDDIO2RDY
  USB_SERIAL.print(F("  VDDIO2 status: "));
  if (PWR->VMSR & PWR_VMSR_VDDIO2RDY) {
    USB_SERIAL.println(F("READY"));
  } else {
    USB_SERIAL.println(F("NOT READY - PB8 may not work!"));
    delay(10);
  }
  #else
  USB_SERIAL.println(F("  VDDIO2: N/A (using VDD)"));
  #endif

  // === TIM16: RPM Generator Output on PB8 ===
  // TIM16 is a 16-bit timer, we'll use it to generate a square wave
  // Period controls RPM: period = 60,000,000 / RPM (at 10MHz tick, period in ticks)

  // Enable GPIOB clock first (needed for PB8/PB1)
  RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;
  __DSB();

  // Enable TIM16 clock
  RCC->APB2ENR |= RCC_APB2ENR_TIM16EN;
  __DSB();

  USB_SERIAL.println(F("  TIM16 clock enabled"));
  USB_SERIAL.flush();

  TimerCalRpm = new HardwareTimer(TIM16);

  USB_SERIAL.println(F("  TIM16 HardwareTimer created"));
  USB_SERIAL.flush();

  // Calculate prescaler for 10MHz tick (same as other timers)
  uint32_t timerClk = HAL_RCC_GetPCLK2Freq();  // TIM16 is on APB2
  uint32_t apb2Prescaler = (RCC->CFGR2 & RCC_CFGR2_PPRE2) >> RCC_CFGR2_PPRE2_Pos;
  if (apb2Prescaler >= 4) {
    timerClk *= 2;
  }
  uint32_t prescaler = timerClk / TIMER_TICK_HZ;

  TimerCalRpm->setPrescaleFactor(prescaler);
  TimerCalRpm->setOverflow(30000);  // Default: 2000 RPM (60M/2000/2 = 15000 half-period)

  USB_SERIAL.println(F("  TIM16 configured"));
  USB_SERIAL.flush();

  // Configure PB8 as TIM16_CH1 output (alternate function)
  // We'll toggle in software for more flexibility
  TimerCalRpm->attachInterrupt(calRpmCallback);

  // Configure PB8 as output
  GPIOB->MODER &= ~(3U << (8 * 2));
  GPIOB->MODER |= (1U << (8 * 2));   // Output mode
  GPIOB->OSPEEDR |= (3U << (8 * 2)); // High speed
  GPIOB->BSRR = (1U << (8 + 16));    // Start LOW

  // Don't start yet - will be started when calibration begins
  TimerCalRpm->pause();

  USB_SERIAL.println(F("  TIM16 OK (PB8 output)"));
  USB_SERIAL.flush();

  // === PB3: Ignition Capture (dedicated calibration pin) ===
  // Configure as input with pull-down - polling mode (no EXTI)
  GPIOB->MODER &= ~(3U << (3 * 2));  // Clear mode bits = INPUT
  GPIOB->PUPDR &= ~(3U << (3 * 2));  // Clear pull-up/down
  GPIOB->PUPDR |= (2U << (3 * 2));   // Pull-down

  USB_SERIAL.println(F("  PB3 configured as INPUT (cal capture)"));
  USB_SERIAL.flush();

  USB_SERIAL.println(F("Calibration timers configured"));
}

// Start/Stop calibration timers
void startCalibrationTimers(uint16_t targetRpm) {
  // Calculate period for target RPM
  // Period = 60,000,000 / RPM / 2 (half-period for toggle)
  // At 10MHz tick: period_ticks = 300,000,000 / RPM
  if (targetRpm < 100) targetRpm = 100;  // Minimum 100 RPM
  if (targetRpm > 20000) targetRpm = 20000;  // Maximum 20000 RPM

  uint32_t halfPeriodTicks = 300000000UL / targetRpm;
  if (halfPeriodTicks > 65535) halfPeriodTicks = 65535;  // 16-bit timer limit

  calState.rpmPeriodTicks = halfPeriodTicks;
  calState.targetRpm = targetRpm;

  // Ensure PB3 is INPUT for ignition capture (dedicated calibration pin)
  // PB3 doesn't conflict with any CDI functionality
  GPIOB->MODER &= ~(3U << (3 * 2));  // Clear mode bits (set to INPUT)
  GPIOB->PUPDR &= ~(3U << (3 * 2));  // Clear pull-up/down
  GPIOB->PUPDR |= (2U << (3 * 2));   // Pull-down

  USB_SERIAL.print(F("CAL:PB3 as INPUT, MODER=0x"));
  USB_SERIAL.println(GPIOB->MODER, HEX);

  TimerCalRpm->setOverflow(halfPeriodTicks);
  TimerCalRpm->setCount(0);
  TimerCalRpm->resume();

  // Using polling in calRpmCallback (EXTI unreliable on STM32H5)
  USB_SERIAL.println(F("CAL:Using polling mode for capture"));

  calState.mode = CAL_MODE_RUNNING;
}

void stopCalibrationTimers(void) {
  TimerCalRpm->pause();

  // Set PB8 LOW
  GPIOB->BSRR = (1U << (8 + 16));

  // PB3 stays as INPUT - dedicated calibration pin, no conflict with CDI
  USB_SERIAL.println(F("CAL:Timers stopped"));

  // Disable timing override so ISR uses normal map timing
  calState.overrideTimingEnabled = 0;

  calState.mode = CAL_MODE_OFF;
}

void updateCalibrationRpm(uint16_t targetRpm) {
  if (targetRpm < 100) targetRpm = 100;
  if (targetRpm > 20000) targetRpm = 20000;

  uint32_t halfPeriodTicks = 300000000UL / targetRpm;
  if (halfPeriodTicks > 65535) halfPeriodTicks = 65535;

  calState.rpmPeriodTicks = halfPeriodTicks;
  calState.targetRpm = targetRpm;

  TimerCalRpm->setOverflow(halfPeriodTicks);
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
  pinMode(PIN_QUICK_SHIFT, INPUT);  // Quick shifter sensor (analog input)
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
    USB_SERIAL.println(F("SD init failed, using Flash/hardcoded defaults"));
    runtime.sdCardOk = 0;
    // Don't set usingDefaultMap=1 here!
    // config already loaded from Flash or hardcoded in loadDefaultConfig()
    runtime.usingDefaultMap = 0;  // Use config.timingMaps
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

// Load hardcoded factory defaults (fallback)
void loadHardcodedDefaults(void) {
  memset(&config, 0, sizeof(CDIConfig));

  config.magic = 0xCD112346;
  config.version = 3;  // v3: QS with strain gauge sensor
  config.engineType = ENGINE_2_STROKE;
  config.activeMap = 0;

  // Trigger: single tooth at 8° BTDC
  config.trigger.triggerAngleScaled = 800;  // 8.00° BTDC
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

  // Quick Shifter defaults (disabled by default, strain gauge sensor)
  config.quickShifter.enabled = 0;
  config.quickShifter.sensitivity = 10;   // 10ms debounce
  config.quickShifter.baseline = 2048;    // ADC mid-point (12-bit ADC)
  config.quickShifter.threshold = 2500;   // Trigger when ADC exceeds this
  config.quickShifter.minRpm = 3000;      // Min 3000 RPM
  config.quickShifter.maxRpm = 15000;     // Max 15000 RPM
  // Default QS cut time map: 50ms for all RPM points
  memset(config.quickShifter.cutTimeMap, 50, QS_TABLE_SIZE);

  // Calibration defaults (disabled until user runs calibration)
  // Note: Actual calibration data stored in separate calData structure (~10.5KB)
  config.calibration.enabled = 0;
  config.calibration.marginError = CAL_DEFAULT_MARGIN;  // 0.05° default margin (in 0.01° units)
  config.calibration.complete = 0;

  config.peakRpm = 0;

  config.checksum = calculateChecksum(&config);

  // Update period thresholds for ISR
  updatePeriodThresholds();
  updatePulseWidth();
}

// Load defaults: try Flash first, then hardcoded
void loadDefaultConfig(void) {
  USB_SERIAL.println(F("  [1] Checking Flash...")); USB_SERIAL.flush();

  // First try to load custom defaults from Flash
  if (hasCustomDefaults()) {
    USB_SERIAL.println(F("  [2] Flash valid, loading...")); USB_SERIAL.flush();
    if (loadDefaultsFromFlash()) {
      runtime.configSource = 1;  // Flash
      USB_SERIAL.println(F("LOADDEFAULT:OK,FLASH"));
      return;
    }
  }

  USB_SERIAL.println(F("  [3] Using hardcoded...")); USB_SERIAL.flush();

  // Fall back to hardcoded defaults
  loadHardcodedDefaults();
  USB_SERIAL.println(F("  [4] Hardcoded done.")); USB_SERIAL.flush();
  runtime.configSource = 2;  // Hardcoded
  USB_SERIAL.println(F("LOADDEFAULT:OK,HARDCODED"));
}

bool validateConfig(struct CDIConfig* cfg) {
  if (cfg->magic != 0xCD112346) return false;
  if (cfg->version == 0 || cfg->version > 10) return false;
  if (cfg->engineType != ENGINE_2_STROKE && cfg->engineType != ENGINE_4_STROKE) return false;
  if (cfg->activeMap >= NUM_MAPS) return false;
  if (cfg->checksum != calculateChecksum(cfg)) return false;
  return true;
}

// Validate ignition map: values must be -10 to 60, not all zeros or 0xFF
bool validateMap(int8_t* map, uint8_t size) {
  uint8_t zeroCount = 0;
  uint8_t ffCount = 0;

  for (uint8_t i = 0; i < size; i++) {
    // Check range: -10 to 60 degrees
    if (map[i] < -10 || map[i] > 60) {
      return false;
    }
    if (map[i] == 0) zeroCount++;
    if (map[i] == -1) ffCount++;  // 0xFF as int8_t = -1
  }

  // Map invalid if all zeros or all 0xFF (erased flash)
  if (zeroCount == size || ffCount == size) {
    return false;
  }

  return true;
}

// Validate QS cut time map: values must be 10-200ms, not all zeros
bool validateQSMap(uint8_t* map, uint8_t size) {
  uint8_t zeroCount = 0;
  uint8_t ffCount = 0;

  for (uint8_t i = 0; i < size; i++) {
    // Check range: 10-200ms
    if (map[i] < 10 || map[i] > 200) {
      return false;
    }
    if (map[i] == 0) zeroCount++;
    if (map[i] == 0xFF) ffCount++;
  }

  // Invalid if all zeros or all 0xFF
  if (zeroCount == size || ffCount == size) {
    return false;
  }

  return true;
}

// Validate all maps in config (active map + QS map)
bool validateConfigMaps(struct CDIConfig* cfg) {
  // Validate active ignition map
  if (!validateMap(cfg->timingMaps[cfg->activeMap], RPM_TABLE_SIZE)) {
    USB_SERIAL.println(F("Active ignition map invalid"));
    return false;
  }

  // Validate QS map if QS enabled
  if (cfg->quickShifter.enabled) {
    if (!validateQSMap(cfg->quickShifter.cutTimeMap, QS_TABLE_SIZE)) {
      USB_SERIAL.println(F("QS cut time map invalid"));
      return false;
    }
  }

  return true;
}

// Load config from SD with validation (for boot sequence)
// Returns: true if config AND maps are valid, false otherwise
bool loadConfigFromSDWithValidation(void) {
  if (!runtime.sdCardOk) {
    return false;
  }

  // Check primary location first, then fallback
  const char* loadFile = CONFIG_FILE;  // "racing-cdi/config.bin"
  if (!SD.exists(loadFile)) {
    loadFile = "cdi_config.bin";  // Fallback in root
    if (!SD.exists(loadFile)) {
      USB_SERIAL.println(F("    No config file on SD"));
      return false;
    }
  }

  File f = SD.open(loadFile, FILE_READ);
  if (!f) {
    USB_SERIAL.println(F("    Cannot open SD config file"));
    return false;
  }

  CDIConfig temp;
  size_t bytesRead = f.read((uint8_t*)&temp, sizeof(CDIConfig));
  f.close();

  // Validate structure
  if (bytesRead != sizeof(CDIConfig)) {
    USB_SERIAL.print(F("    SD config size mismatch: "));
    USB_SERIAL.print(bytesRead);
    USB_SERIAL.print(F("/"));
    USB_SERIAL.println(sizeof(CDIConfig));
    return false;
  }

  if (!validateConfig(&temp)) {
    USB_SERIAL.println(F("    SD config structure invalid"));
    return false;
  }

  // Validate maps (ignition map + QS map)
  if (!validateConfigMaps(&temp)) {
    USB_SERIAL.println(F("    SD maps invalid"));
    return false;
  }

  // All OK - copy to config
  memcpy(&config, &temp, sizeof(CDIConfig));
  runtime.usingDefaultMap = 0;
  runtime.configSource = 0;  // SD card

  // Update period thresholds for ISR
  updatePeriodThresholds();
  updatePulseWidth();

  USB_SERIAL.print(F("    Config loaded from SD, map "));
  USB_SERIAL.println(config.activeMap + 1);
  return true;
}

// Legacy loadConfigFromSD for runtime reload (called by USB command)
void loadConfigFromSD(void) {
  // If SD not OK, keep using current config
  if (!runtime.sdCardOk) {
    USB_SERIAL.println(F("SD not OK"));
    return;
  }

  // Skip if SD busy
  if (runtime.sdBusy) {
    USB_SERIAL.println(F("Load failed: SD busy"));
    return;
  }

  runtime.sdBusy = 1;
  bool result = loadConfigFromSDWithValidation();
  runtime.sdBusy = 0;

  if (!result) {
    USB_SERIAL.println(F("Load from SD failed"));
  }
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
// CALIBRATION DATA FILE STORAGE
// ============================================================================

uint32_t calculateCalChecksum(CalibrationData* cd) {
  uint32_t sum = 0;
  uint8_t* p = (uint8_t*)cd;
  size_t len = sizeof(CalibrationData) - sizeof(uint32_t);  // Exclude checksum itself
  for (size_t i = 0; i < len; i++) {
    sum += p[i];
  }
  return sum ^ 0xCA12CA12;
}

bool validateCalData(CalibrationData* cd) {
  if (cd->magic != CAL_MAGIC) return false;
  if (cd->version != CAL_VERSION) return false;
  if (cd->checksum != calculateCalChecksum(cd)) return false;
  return true;
}

// Save calibration data to SD card
void saveCalibrationToSD(void) {
  if (!runtime.sdCardOk) {
    USB_SERIAL.println(F("CAL:SAVE,ERROR,SD not OK"));
    return;
  }

  if (runtime.sdBusy) {
    USB_SERIAL.println(F("CAL:SAVE,ERROR,SD busy"));
    return;
  }

  runtime.sdBusy = 1;

  // Update header
  calData.magic = CAL_MAGIC;
  calData.version = CAL_VERSION;
  calData.enabled = config.calibration.enabled;
  calData.marginError = config.calibration.marginError;
  calData.complete = config.calibration.complete;
  calData.checksum = calculateCalChecksum(&calData);

  // Create folder if needed
  SD.mkdir(CONFIG_FOLDER);

  // Remove old file
  SD.remove(CAL_FILE_BIN);

  // Save
  File f = SD.open(CAL_FILE_BIN, FILE_WRITE);
  if (!f) {
    USB_SERIAL.println(F("CAL:SAVE,ERROR,cannot open file"));
    runtime.sdBusy = 0;
    return;
  }

  size_t written = f.write((uint8_t*)&calData, sizeof(CalibrationData));
  f.close();

  runtime.sdBusy = 0;

  if (written != sizeof(CalibrationData)) {
    USB_SERIAL.print(F("CAL:SAVE,ERROR,wrote "));
    USB_SERIAL.print(written);
    USB_SERIAL.print(F("/"));
    USB_SERIAL.println(sizeof(CalibrationData));
  } else {
    USB_SERIAL.print(F("CAL:SAVE,OK,"));
    USB_SERIAL.print(sizeof(CalibrationData));
    USB_SERIAL.println(F(" bytes"));
    calDataLoaded = 1;
  }
}

// Load calibration data from SD card
bool loadCalibrationFromSD(void) {
  if (!runtime.sdCardOk) {
    USB_SERIAL.println(F("CAL:LOAD,ERROR,SD not OK"));
    return false;
  }

  if (runtime.sdBusy) {
    USB_SERIAL.println(F("CAL:LOAD,ERROR,SD busy"));
    return false;
  }

  runtime.sdBusy = 1;

  File f = SD.open(CAL_FILE_BIN, FILE_READ);
  if (!f) {
    USB_SERIAL.println(F("CAL:LOAD,NOFILE"));
    runtime.sdBusy = 0;
    calDataLoaded = 0;
    return false;
  }

  size_t readBytes = f.read((uint8_t*)&calData, sizeof(CalibrationData));
  f.close();

  runtime.sdBusy = 0;

  if (readBytes != sizeof(CalibrationData)) {
    USB_SERIAL.print(F("CAL:LOAD,ERROR,read "));
    USB_SERIAL.print(readBytes);
    USB_SERIAL.print(F("/"));
    USB_SERIAL.println(sizeof(CalibrationData));
    calDataLoaded = 0;
    return false;
  }

  if (!validateCalData(&calData)) {
    USB_SERIAL.println(F("CAL:LOAD,ERROR,invalid data"));
    calDataLoaded = 0;
    return false;
  }

  // Sync with config
  config.calibration.enabled = calData.enabled;
  config.calibration.marginError = calData.marginError;
  config.calibration.complete = calData.complete;
  calDataLoaded = 1;
  calSource = CAL_SOURCE_SD;

  USB_SERIAL.print(F("CAL:LOAD,OK,"));
  USB_SERIAL.print(sizeof(CalibrationData));
  USB_SERIAL.print(F(" bytes,complete="));
  USB_SERIAL.println(calData.complete);

  return true;
}

// Export calibration to text file
void exportCalibrationToText(void) {
  if (!runtime.sdCardOk) {
    USB_SERIAL.println(F("CAL:EXPORT,ERROR,SD not OK"));
    return;
  }

  runtime.sdBusy = 1;

  SD.remove(CAL_FILE_TXT);
  File f = SD.open(CAL_FILE_TXT, FILE_WRITE);
  if (!f) {
    USB_SERIAL.println(F("CAL:EXPORT,ERROR,cannot open file"));
    runtime.sdBusy = 0;
    return;
  }

  // Header
  f.println(F("# CDI Calibration Data v2"));
  f.println(F("# 2D Map with Fuzzy Logic"));
  f.print(F("# Margin: "));
  f.print((float)config.calibration.marginError / 100.0, 2);
  f.println(F(" deg"));
  f.println(F("#"));
  f.println(F("# Format: RPM,TimingDeg,Offset(x0.01deg),Calibrated"));
  f.println(F("# RPM: 0-20000 step 250 (81 points)"));
  f.println(F("# Timing: 0-60 deg BTDC (61 points)"));
  f.println(F("#"));

  uint32_t pointCount = 0;

  for (uint8_t r = 0; r < CAL_RPM_POINTS; r++) {
    for (uint8_t t = 0; t < CAL_TIMING_POINTS; t++) {
      f.print(r * 250);
      f.print(F(","));
      f.print(t);
      f.print(F(","));
      f.print(calData.offsets[r][t]);
      f.print(F(","));
      f.println(isPointCalibrated(r, t) ? 1 : 0);
      pointCount++;
    }
  }

  f.println(F("# EOF"));
  f.close();

  runtime.sdBusy = 0;

  USB_SERIAL.print(F("CAL:EXPORT,OK,"));
  USB_SERIAL.print(pointCount);
  USB_SERIAL.println(F(" points"));
}

// Import calibration from text file
bool importCalibrationFromText(void) {
  if (!runtime.sdCardOk) {
    USB_SERIAL.println(F("CAL:IMPORT,ERROR,SD not OK"));
    return false;
  }

  runtime.sdBusy = 1;

  File f = SD.open(CAL_FILE_TXT, FILE_READ);
  if (!f) {
    USB_SERIAL.println(F("CAL:IMPORT,NOFILE"));
    runtime.sdBusy = 0;
    return false;
  }

  // Clear existing data
  memset(&calData, 0, sizeof(CalibrationData));
  calData.magic = CAL_MAGIC;
  calData.version = CAL_VERSION;

  uint32_t pointCount = 0;
  uint32_t calibratedCount = 0;
  char line[64];

  while (f.available()) {
    int len = f.readBytesUntil('\n', line, sizeof(line) - 1);
    line[len] = '\0';

    // Skip comments and empty lines
    if (line[0] == '#' || line[0] == '\0' || line[0] == '\r') continue;

    // Parse: RPM,TimingDeg,Offset,Calibrated
    int rpm, timing, offset, cal;
    if (sscanf(line, "%d,%d,%d,%d", &rpm, &timing, &offset, &cal) == 4) {
      uint8_t rpmIdx = rpm / 250;
      uint8_t timingDeg = timing;

      if (rpmIdx < CAL_RPM_POINTS && timingDeg < CAL_TIMING_POINTS) {
        calData.offsets[rpmIdx][timingDeg] = (int16_t)offset;
        if (cal) {
          markPointCalibrated(rpmIdx, timingDeg);
          calibratedCount++;
        }
        pointCount++;
      }
    }
  }

  f.close();

  // Update status
  calData.enabled = (calibratedCount > 0) ? 1 : 0;
  calData.complete = (calibratedCount == CAL_TOTAL_POINTS) ? 1 : 0;
  calData.marginError = config.calibration.marginError;
  calData.checksum = calculateCalChecksum(&calData);

  // Sync with config
  config.calibration.enabled = calData.enabled;
  config.calibration.complete = calData.complete;
  calDataLoaded = 1;

  runtime.sdBusy = 0;

  USB_SERIAL.print(F("CAL:IMPORT,OK,"));
  USB_SERIAL.print(pointCount);
  USB_SERIAL.print(F(" points,"));
  USB_SERIAL.print(calibratedCount);
  USB_SERIAL.println(F(" calibrated"));

  return true;
}

// Clear calibration data
void clearCalibrationData(void) {
  memset(&calData, 0, sizeof(CalibrationData));
  calData.magic = CAL_MAGIC;
  calData.version = CAL_VERSION;
  calData.marginError = CAL_DEFAULT_MARGIN;

  config.calibration.enabled = 0;
  config.calibration.complete = 0;
  calDataLoaded = 0;

  USB_SERIAL.println(F("CAL:CLEAR,OK"));
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
  f.println();

  f.println(F("[QUICK_SHIFTER]"));
  f.print(F("enabled=")); f.println(config.quickShifter.enabled ? F("ON") : F("OFF"));
  f.print(F("sensitivity_ms=")); f.println(config.quickShifter.sensitivity);
  f.print(F("baseline_adc=")); f.println(config.quickShifter.baseline);
  f.print(F("threshold_adc=")); f.println(config.quickShifter.threshold);
  f.print(F("min_rpm=")); f.println(config.quickShifter.minRpm);
  f.print(F("max_rpm=")); f.println(config.quickShifter.maxRpm);

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

// Export QS cut time map to text file
static const char* QS_MAP_FILE = "racing-cdi/qs_map.txt";

void exportQSMapToText(void) {
  if (!runtime.sdCardOk) return;

  if (SD.exists(QS_MAP_FILE)) {
    SD.remove(QS_MAP_FILE);
  }

  File f = SD.open(QS_MAP_FILE, FILE_WRITE);
  if (!f) return;

  f.println(F("# QUICK SHIFTER CUT TIME MAP"));
  f.println(F("# Format: RPM=CUT_TIME_MS"));
  f.println(F("# Cut time in milliseconds (0-255)"));
  f.println();

  for (int i = 0; i < QS_TABLE_SIZE; i++) {
    uint16_t rpm = i * 1000;  // QS map uses 1000 RPM steps
    f.print(rpm);
    f.print(F("="));
    f.println(config.quickShifter.cutTimeMap[i]);
  }

  f.close();
  USB_SERIAL.println(F("QS map exported"));
}

// Import QS cut time map from text file
void importQSMapFromText(void) {
  if (!runtime.sdCardOk) return;

  if (!SD.exists(QS_MAP_FILE)) {
    USB_SERIAL.println(F("No qs_map.txt"));
    return;
  }

  File f = SD.open(QS_MAP_FILE, FILE_READ);
  if (!f) return;

  char line[64];
  while (f.available()) {
    int len = f.readBytesUntil('\n', line, sizeof(line) - 1);
    line[len] = '\0';

    // Skip comments and empty lines
    if (line[0] == '#' || line[0] == '\0' || line[0] == '\r') continue;

    // Parse RPM=VALUE
    char* eq = strchr(line, '=');
    if (eq) {
      *eq = '\0';
      uint16_t rpm = atoi(line);
      uint8_t cutTime = atoi(eq + 1);

      int idx = rpm / 1000;
      if (idx >= 0 && idx < QS_TABLE_SIZE && cutTime <= 255) {
        config.quickShifter.cutTimeMap[idx] = cutTime;
      }
    }
  }

  f.close();
  config.checksum = calculateChecksum(&config);
  USB_SERIAL.println(F("QS map imported"));
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
  char currentSection[20] = "";  // Track which section we're in

  while (f.available()) {
    int len = f.readBytesUntil('\n', line, sizeof(line) - 1);
    line[len] = '\0';

    // Skip comments and empty lines
    if (line[0] == '#' || len < 3) continue;

    // Track section headers
    if (line[0] == '[') {
      // Extract section name
      char* end = strchr(line, ']');
      if (end) {
        *end = '\0';
        strncpy(currentSection, line + 1, sizeof(currentSection) - 1);
      }
      continue;
    }

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
      // Check which section we're in
      if (strcmp(currentSection, "CRANKING") == 0) {
        config.cranking.enabled = (strstr(val, "ON") != NULL) ? 1 : 0;
      } else if (strcmp(currentSection, "QUICK_SHIFTER") == 0) {
        config.quickShifter.enabled = (strstr(val, "ON") != NULL) ? 1 : 0;
      }
    }
    else if (strcmp(key, "timing") == 0) {
      config.cranking.timingScaled = atoi(val);
    }
    else if (strcmp(key, "max_rpm") == 0) {
      // Check which section
      if (strcmp(currentSection, "CRANKING") == 0) {
        config.cranking.maxRpm = atoi(val);
      } else if (strcmp(currentSection, "QUICK_SHIFTER") == 0) {
        config.quickShifter.maxRpm = atoi(val);
      }
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
    // Quick Shifter settings (only in QUICK_SHIFTER section)
    else if (strcmp(key, "sensitivity_ms") == 0) {
      config.quickShifter.sensitivity = atoi(val);
    }
    else if (strcmp(key, "baseline_adc") == 0) {
      config.quickShifter.baseline = atoi(val);
    }
    else if (strcmp(key, "threshold_adc") == 0) {
      config.quickShifter.threshold = atoi(val);
    }
    else if (strcmp(key, "min_rpm") == 0 && strcmp(currentSection, "QUICK_SHIFTER") == 0) {
      config.quickShifter.minRpm = atoi(val);
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
  kickWatchdog();
  runtime.sdBusy = 1;
  createReadmeFile();
  kickWatchdog();
  exportSettingsToText();
  kickWatchdog();
  exportAllMapsToText();
  kickWatchdog();
  exportQSMapToText();  // Also export QS cut time map
  kickWatchdog();
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
  importQSMapFromText();  // Also import QS cut time map

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
// QUICK SHIFTER PROCESSING
// ============================================================================

// Get cut time from QS map based on current RPM
uint8_t getQsCutTime(uint16_t rpm) {
  if (rpm >= RPM_MAX) {
    return config.quickShifter.cutTimeMap[QS_TABLE_SIZE - 1];
  }

  // Calculate index and interpolate
  uint16_t index = rpm / QS_RPM_STEP;
  if (index >= QS_TABLE_SIZE - 1) {
    return config.quickShifter.cutTimeMap[QS_TABLE_SIZE - 1];
  }

  // Linear interpolation between points
  uint16_t rpmLow = index * QS_RPM_STEP;
  uint8_t timeLow = config.quickShifter.cutTimeMap[index];
  uint8_t timeHigh = config.quickShifter.cutTimeMap[index + 1];

  // Interpolate
  uint16_t fraction = rpm - rpmLow;
  int16_t diff = (int16_t)timeHigh - (int16_t)timeLow;
  return timeLow + (diff * fraction / QS_RPM_STEP);
}

void processQuickShifter(void) {
  // Skip during startup (first 500ms) to let ADC stabilize
  static uint32_t startupTime = 0;
  if (startupTime == 0) {
    startupTime = millis();
    runtime.qsArmed = 1;  // Start armed
  }
  if (millis() - startupTime < 500) return;

  // Read ADC with rate limiting (every 10ms max for calibration UI)
  static uint32_t lastAdcRead = 0;
  uint32_t now = millis();
  if (now - lastAdcRead >= 10) {
    lastAdcRead = now;
    runtime.qsCurrentAdc = analogRead(PIN_QUICK_SHIFT);
  }

  // Skip processing if disabled
  if (!config.quickShifter.enabled) {
    __disable_irq();
    runtime.qsActive = 0;
    __enable_irq();
    runtime.qsArmed = 1;  // Reset to armed when disabled
    return;
  }

  uint16_t adc = runtime.qsCurrentAdc;
  uint16_t baseline = config.quickShifter.baseline;
  uint16_t threshold = config.quickShifter.threshold;

  // Re-arm hysteresis: must drop below baseline + 10% of (threshold - baseline)
  // This ensures user has actually released the lever
  uint16_t rearmLevel = baseline + ((threshold - baseline) / 10);

  // Check if cut is currently active and should end
  // ATOMIC: qsActive is accessed by ISR, use critical section
  __disable_irq();
  uint8_t cutActive = runtime.qsActive;
  if (cutActive && (now - runtime.qsCutStartMs) >= runtime.qsCutDurationMs) {
    runtime.qsActive = 0;  // End cut
    // Note: qsArmed stays 0, must wait for sensor to return to baseline
  }
  __enable_irq();

  if (cutActive) {
    return;  // Don't process new triggers while cut active
  }

  // Re-arm logic: sensor must return near baseline before another cut is allowed
  // This prevents continuous cutting if user keeps lever pressed
  if (!runtime.qsArmed) {
    if (adc <= rearmLevel) {
      runtime.qsArmed = 1;  // Ready for next shift
    }
    return;  // Not armed yet, skip trigger detection
  }

  // Check RPM limits
  uint16_t rpm = runtime.currentRpm;
  if (rpm < config.quickShifter.minRpm || rpm > config.quickShifter.maxRpm) {
    return;
  }

  // Strain gauge detection: triggered when ADC exceeds threshold
  // (assuming sensor output increases when shift lever is pressed)
  uint8_t triggered = (adc >= threshold);

  // Debounce
  if (triggered != runtime.qsTriggered) {
    runtime.qsLastTriggerMs = now;
    runtime.qsTriggered = triggered;
  }

  // Check for valid trigger (debounced) - only if armed!
  if (triggered && (now - runtime.qsLastTriggerMs) >= config.quickShifter.sensitivity) {
    // Start ignition cut!
    runtime.qsCutDurationMs = getQsCutTime(rpm);
    runtime.qsCutStartMs = now;

    // Atomic write - ISR reads qsActive
    __disable_irq();
    runtime.qsActive = 1;
    runtime.qsArmed = 0;  // Disarm until sensor returns to baseline
    __enable_irq();
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

  // Kick watchdog before potentially blocking read
  kickWatchdog();

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

  // Flash defaults management - Settings, QS, and Map stored separately
  // SAVESETTINGS - save basic settings to Flash (trigger, limiter, warning, etc)
  // LOADSETTINGS - load basic settings from Flash
  // SAVEQS       - save QS config + cut time map to Flash
  // LOADQS       - load QS config + cut time map from Flash
  // SAVEMAP n    - save ignition map n (1-6) to Flash
  // LOADMAP      - load safety ignition map from Flash
  // CLEARDEFAULT - clear all Flash data
  } else if (cmd == "SAVESETTINGS") {
    saveSettingsToFlash();
  } else if (cmd == "LOADSETTINGS") {
    loadSettingsFromFlash();
  } else if (cmd == "SAVEQS") {
    saveQSToFlash();
  } else if (cmd == "LOADQS") {
    loadQSFromFlash();
  } else if (cmd.startsWith("SAVEMAP")) {
    uint8_t mapNum = config.activeMap + 1;  // Default: current active map
    if (cmd.length() > 8) {
      mapNum = cmd.substring(8).toInt();
    }
    saveMapToFlash(mapNum);
  } else if (cmd == "LOADMAP") {
    loadMapFromFlash();
  } else if (cmd == "LOADDEFAULT") {
    loadDefaultConfig();  // Load both settings and map from Flash
  } else if (cmd == "CLEARDEFAULT") {
    clearDefaultsFromFlash();

  // Note: "GET DEFAULTS" is handled by handleGet("DEFAULTS")

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

  // Quick Shifter on/off commands
  } else if (cmd == "QS ON") {
    config.quickShifter.enabled = 1;
    config.checksum = calculateChecksum(&config);
    USB_SERIAL.println(F("QS:ON"));
  } else if (cmd == "QS OFF") {
    config.quickShifter.enabled = 0;
    __disable_irq();
    runtime.qsActive = 0;
    __enable_irq();
    config.checksum = calculateChecksum(&config);
    USB_SERIAL.println(F("QS:OFF"));

  // Binary telemetry mode toggle (for faster/more efficient data transfer)
  } else if (cmd == "BINARY ON") {
    binaryTelemetryMode = 1;
    USB_SERIAL.println(F("BINARY:ON"));
  } else if (cmd == "BINARY OFF") {
    binaryTelemetryMode = 0;
    USB_SERIAL.println(F("BINARY:OFF"));

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
  } else if (cmd == "EXPORTQS") {
    if (runtime.currentRpm >= 100) {
      USB_SERIAL.println(F("EXPORT:BUSY"));
    } else {
      NVIC_DisableIRQ(TIM2_IRQn);
      exportQSMapToText();
      NVIC_EnableIRQ(TIM2_IRQn);
    }
  } else if (cmd == "IMPORTQS") {
    if (runtime.currentRpm >= 100) {
      USB_SERIAL.println(F("IMPORT:BUSY"));
    } else {
      NVIC_DisableIRQ(TIM2_IRQn);
      importQSMapFromText();
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

  // ============================================================================
  // CALIBRATION COMMANDS (2D Map: RPM × Timing)
  // ============================================================================
  } else if (cmd == "CAL START" || cmd == "CAL:START") {
    // Start self-calibration mode
    if (runtime.currentRpm >= 100) {
      USB_SERIAL.println(F("CAL:BUSY"));  // Engine running
    } else if (calState.mode == CAL_MODE_RUNNING) {
      USB_SERIAL.println(F("CAL:ALREADY"));
    } else {
      // Initialize calibration state
      memset(&calState, 0, sizeof(CalibrationState));
      calState.currentTimingDeg = 0;      // Start at 0° BTDC
      calState.currentRpmIdx = 2;         // Start at index 2 = 500 RPM (0,250,500)
      calState.targetRpm = CAL_RPM_START;
      calState.mode = CAL_MODE_RUNNING;
      calState.lastStateChangeMs = millis();
      calState.totalPoints = CAL_TOTAL_POINTS;

      // CRITICAL: Set override timing BEFORE starting timers!
      // Without this, first calibration point uses map timing instead of target
      calState.overrideTimingEnabled = 1;
      calState.overrideTimingScaled = calState.currentTimingDeg * 100;  // 0° initially

      // Reset debug counters
      calState.debugTriggerCount = 0;
      calState.debugCaptureCount = 0;
      calState.debugRawExtiCount = 0;

      // Clear calibration data
      clearCalibrationData();

      // Start calibration timers
      startCalibrationTimers(calState.targetRpm);

      USB_SERIAL.println(F("CAL:STARTED"));
      USB_SERIAL.print(F("CAL:TOTAL,"));
      USB_SERIAL.println(CAL_TOTAL_POINTS);
      USB_SERIAL.print(F("CAL:MARGIN,"));
      USB_SERIAL.print(config.calibration.marginError);
      USB_SERIAL.println(F(" (0.01deg)"));

      // DEBUG: Using polling on PB3 for ignition capture
      USB_SERIAL.print(F("CAL:PB3=")); USB_SERIAL.println(((GPIOB->IDR >> 3) & 1));
    }

  } else if (cmd == "CAL STOP" || cmd == "CAL:STOP") {
    // Stop calibration
    if (calState.mode != CAL_MODE_OFF) {
      stopCalibrationTimers();
      USB_SERIAL.println(F("CAL:STOPPED"));
      USB_SERIAL.print(F("CAL:PROGRESS,"));
      USB_SERIAL.print(calState.completedPoints);
      USB_SERIAL.print(F("/"));
      USB_SERIAL.println(CAL_TOTAL_POINTS);
      USB_SERIAL.print(F("CAL:PASSED,"));
      USB_SERIAL.println(calState.passedPoints);
      USB_SERIAL.print(F("CAL:FAILED,"));
      USB_SERIAL.println(calState.failedPoints);
    } else {
      USB_SERIAL.println(F("CAL:NOTRUNNING"));
    }

  } else if (cmd == "CAL PAUSE" || cmd == "CAL:PAUSE") {
    if (calState.mode == CAL_MODE_RUNNING) {
      calState.mode = CAL_MODE_PAUSED;
      TimerCalRpm->pause();
      USB_SERIAL.println(F("CAL:PAUSED"));
    } else {
      USB_SERIAL.println(F("CAL:NOTRUNNING"));
    }

  } else if (cmd == "CAL RESUME" || cmd == "CAL:RESUME") {
    if (calState.mode == CAL_MODE_PAUSED) {
      calState.mode = CAL_MODE_RUNNING;
      calState.lastStateChangeMs = millis();
      TimerCalRpm->resume();
      USB_SERIAL.println(F("CAL:RESUMED"));
    } else {
      USB_SERIAL.println(F("CAL:NOTPAUSED"));
    }

  } else if (cmd == "CAL TEST" || cmd == "CAL:TEST") {
    // Test jumper connections
    USB_SERIAL.println(F("CAL:TEST_START"));

    // Stop calibration if running
    if (calState.mode != CAL_MODE_OFF) {
      stopCalibrationTimers();
    }

    // === TEST 1: PB8 → PA0 (Trigger path) ===
    USB_SERIAL.println(F("CAL:TEST1_PB8_PA0"));

    // Configure PB8 as output (should already be)
    GPIOB->MODER &= ~(3U << (8 * 2));
    GPIOB->MODER |= (1U << (8 * 2));
    GPIOB->BSRR = (1U << (8 + 16));  // Start LOW
    delay(5);

    // Read PA0 initial state
    bool pa0Before = (GPIOA->IDR & (1U << 0)) ? true : false;

    // Toggle PB8 HIGH
    GPIOB->BSRR = (1U << 8);
    delayMicroseconds(100);
    bool pa0AfterHigh = (GPIOA->IDR & (1U << 0)) ? true : false;

    // Toggle PB8 LOW
    GPIOB->BSRR = (1U << (8 + 16));
    delayMicroseconds(100);
    bool pa0AfterLow = (GPIOA->IDR & (1U << 0)) ? true : false;

    USB_SERIAL.print(F("  PB8=LOW  -> PA0="));
    USB_SERIAL.println(pa0Before ? "HIGH" : "LOW");
    USB_SERIAL.print(F("  PB8=HIGH -> PA0="));
    USB_SERIAL.println(pa0AfterHigh ? "HIGH" : "LOW");
    USB_SERIAL.print(F("  PB8=LOW  -> PA0="));
    USB_SERIAL.println(pa0AfterLow ? "HIGH" : "LOW");

    bool test1Pass = (!pa0Before && pa0AfterHigh && !pa0AfterLow);
    USB_SERIAL.print(F("CAL:TEST1_RESULT,"));
    USB_SERIAL.println(test1Pass ? "PASS" : "FAIL");

    // === TEST 2: PB0 → PB3 (Ignition capture path) ===
    USB_SERIAL.println(F("CAL:TEST2_PB0_PB3"));

    // Configure PB3 as input with pull-down
    GPIOB->MODER &= ~(3U << (3 * 2));
    GPIOB->PUPDR &= ~(3U << (3 * 2));
    GPIOB->PUPDR |= (2U << (3 * 2));
    delay(1);

    // PB0 is CDI output, set LOW first
    CDI_LOW();
    delay(1);

    // Read PB3 initial state
    bool pb3Before = (GPIOB->IDR & (1U << 3)) ? true : false;

    // Set PB0 HIGH
    CDI_HIGH();
    delayMicroseconds(100);
    bool pb3AfterHigh = (GPIOB->IDR & (1U << 3)) ? true : false;

    // Set PB0 LOW
    CDI_LOW();
    delayMicroseconds(100);
    bool pb3AfterLow = (GPIOB->IDR & (1U << 3)) ? true : false;

    USB_SERIAL.print(F("  PB0=LOW  -> PB3="));
    USB_SERIAL.println(pb3Before ? "HIGH" : "LOW");
    USB_SERIAL.print(F("  PB0=HIGH -> PB3="));
    USB_SERIAL.println(pb3AfterHigh ? "HIGH" : "LOW");
    USB_SERIAL.print(F("  PB0=LOW  -> PB3="));
    USB_SERIAL.println(pb3AfterLow ? "HIGH" : "LOW");

    bool test2Pass = (!pb3Before && pb3AfterHigh && !pb3AfterLow);
    USB_SERIAL.print(F("CAL:TEST2_RESULT,"));
    USB_SERIAL.println(test2Pass ? "PASS" : "FAIL");

    // === SUMMARY ===
    USB_SERIAL.print(F("CAL:TEST_SUMMARY,"));
    if (test1Pass && test2Pass) {
      USB_SERIAL.println(F("ALL_PASS"));
    } else if (!test1Pass && !test2Pass) {
      USB_SERIAL.println(F("BOTH_FAIL"));
    } else if (!test1Pass) {
      USB_SERIAL.println(F("PB8_PA0_FAIL"));
    } else {
      USB_SERIAL.println(F("PB0_PB3_FAIL"));
    }
    USB_SERIAL.println(F("CAL:TEST_END"));

  } else if (cmd == "CAL STATUS" || cmd == "CAL:STATUS") {
    // Report 2D calibration status
    USB_SERIAL.print(F("CAL:MODE,"));
    USB_SERIAL.println(calState.mode);
    USB_SERIAL.print(F("CAL:POS,RPM="));
    USB_SERIAL.print(calState.currentRpmIdx * 250);
    USB_SERIAL.print(F(",TIMING="));
    USB_SERIAL.println(calState.currentTimingDeg);
    USB_SERIAL.print(F("CAL:RPMIDX,"));
    USB_SERIAL.print(calState.currentRpmIdx);
    USB_SERIAL.print(F("/"));
    USB_SERIAL.println(CAL_RPM_POINTS);
    USB_SERIAL.print(F("CAL:TIMINGDEG,"));
    USB_SERIAL.print(calState.currentTimingDeg);
    USB_SERIAL.print(F("/"));
    USB_SERIAL.println(CAL_TIMING_POINTS);
    USB_SERIAL.print(F("CAL:PROGRESS,"));
    USB_SERIAL.print(calState.completedPoints);
    USB_SERIAL.print(F("/"));
    USB_SERIAL.println(CAL_TOTAL_POINTS);
    USB_SERIAL.print(F("CAL:PASSED,"));
    USB_SERIAL.println(calState.passedPoints);
    USB_SERIAL.print(F("CAL:FAILED,"));
    USB_SERIAL.println(calState.failedPoints);
    USB_SERIAL.print(F("CAL:MISFIRES,"));
    USB_SERIAL.println(calState.totalMisfires);
    USB_SERIAL.print(F("CAL:MISFIRE_POINTS,"));
    USB_SERIAL.println(calState.misfirePoints);
    USB_SERIAL.print(F("CAL:MISFIRE_CURRENT,"));
    USB_SERIAL.println(calState.misfireCount);
    // DEBUG: Show trigger and capture counts to diagnose misfire issues
    USB_SERIAL.print(F("CAL:DEBUG_TRIGGERS,"));
    USB_SERIAL.println(calState.debugTriggerCount);
    USB_SERIAL.print(F("CAL:DEBUG_IGNITIONS,"));
    USB_SERIAL.println(runtime.ignitionCount);
    USB_SERIAL.print(F("CAL:DEBUG_CAPTURES,"));
    USB_SERIAL.println(calState.debugCaptureCount);
    // Pin state debug (using polling on PB3)
    USB_SERIAL.print(F("CAL:PB3_STATE,"));
    USB_SERIAL.println(((GPIOB->IDR >> 3) & 1));
    USB_SERIAL.print(F("CAL:PB0_STATE,"));
    USB_SERIAL.println((GPIOB->IDR >> 0) & 1);
    USB_SERIAL.print(F("CAL:ITERATION,"));
    USB_SERIAL.println(calState.iteration);
    USB_SERIAL.print(F("CAL:OFFSET,"));
    // Current accumulated offset in millidegrees
    USB_SERIAL.print(calState.accumulatedOffset / 10);  // Show as centidegrees
    USB_SERIAL.println(F(" (0.01deg)"));
    USB_SERIAL.print(F("CAL:ENABLED,"));
    USB_SERIAL.println(config.calibration.enabled);
    USB_SERIAL.print(F("CAL:COMPLETE,"));
    USB_SERIAL.println(calData.complete);
    USB_SERIAL.print(F("CAL:CALIBRATED_BITMAP,"));
    USB_SERIAL.println(countCalibratedPoints());
    USB_SERIAL.print(F("CAL:MISFIREMAP_POINTS,"));
    USB_SERIAL.println(countMisfirePronePoints());
    USB_SERIAL.print(F("CAL:LOADED,"));
    USB_SERIAL.println(calDataLoaded);
    USB_SERIAL.print(F("CAL:SOURCE,"));
    USB_SERIAL.println(getCalSourceName());
    USB_SERIAL.print(F("CAL:FLASH_BACKUP,"));
    USB_SERIAL.println(hasFlashCalibration() ? F("YES") : F("NO"));

  } else if (cmd.startsWith("CAL MARGIN ") || cmd.startsWith("CAL:MARGIN,")) {
    // Set error margin (in 0.01° units, 1-100 = 0.01° to 1.00°)
    int idx = cmd.indexOf(' ', 4);
    if (idx < 0) idx = cmd.indexOf(',', 4);
    if (idx > 0) {
      uint8_t margin = cmd.substring(idx + 1).toInt();
      if (margin >= 1 && margin <= 100) {
        config.calibration.marginError = margin;
        calData.marginError = margin;  // Sync to calData
        config.checksum = calculateChecksum(&config);
        USB_SERIAL.print(F("CAL:MARGIN,"));
        USB_SERIAL.print(margin);
        USB_SERIAL.println(F(" (0.01deg)"));
      } else {
        USB_SERIAL.println(F("CAL:MARGIN,INVALID (1-100)"));
      }
    }

  } else if (cmd == "CAL ENABLE" || cmd == "CAL:ENABLE") {
    config.calibration.enabled = 1;
    calData.enabled = 1;
    config.checksum = calculateChecksum(&config);
    USB_SERIAL.println(F("CAL:ENABLED"));

  } else if (cmd == "CAL DISABLE" || cmd == "CAL:DISABLE") {
    config.calibration.enabled = 0;
    calData.enabled = 0;
    config.checksum = calculateChecksum(&config);
    USB_SERIAL.println(F("CAL:DISABLED"));

  } else if (cmd == "CAL SAVE" || cmd == "CAL:SAVE") {
    // Save calibration to SD card (binary)
    USB_SERIAL.println(F("CAL:SAVING..."));
    saveCalibrationToSD();
    USB_SERIAL.println(F("CAL:SAVED"));

  } else if (cmd == "CAL LOAD" || cmd == "CAL:LOAD") {
    // Load calibration from SD card (binary)
    USB_SERIAL.println(F("CAL:LOADING..."));
    if (loadCalibrationFromSD()) {
      USB_SERIAL.println(F("CAL:LOADED,OK"));
      USB_SERIAL.print(F("CAL:COMPLETE,"));
      USB_SERIAL.println(calData.complete);
    } else {
      USB_SERIAL.println(F("CAL:LOADED,FAIL"));
    }

  } else if (cmd == "CAL EXPORT" || cmd == "CAL:EXPORT") {
    // Export calibration to text file
    USB_SERIAL.println(F("CAL:EXPORTING..."));
    exportCalibrationToText();
    USB_SERIAL.println(F("CAL:EXPORTED,calibration.txt"));

  } else if (cmd == "CAL IMPORT" || cmd == "CAL:IMPORT") {
    // Import calibration from text file
    USB_SERIAL.println(F("CAL:IMPORTING..."));
    if (importCalibrationFromText()) {
      USB_SERIAL.println(F("CAL:IMPORTED,OK"));
    } else {
      USB_SERIAL.println(F("CAL:IMPORTED,FAIL"));
    }

  } else if (cmd == "CAL FLASHSAVE" || cmd == "CAL:FLASHSAVE") {
    // Save calibration to Flash backup
    USB_SERIAL.println(F("CAL:FLASH,SAVING..."));
    if (saveCalibrationToFlash()) {
      USB_SERIAL.print(F("CAL:FLASH,SAVED,"));
      USB_SERIAL.print(countCalibratedPoints());
      USB_SERIAL.println(F(" points"));
    }

  } else if (cmd == "CAL FLASHLOAD" || cmd == "CAL:FLASHLOAD") {
    // Load calibration from Flash backup
    USB_SERIAL.println(F("CAL:FLASH,LOADING..."));
    if (loadCalibrationFromFlash()) {
      USB_SERIAL.print(F("CAL:FLASH,LOADED,"));
      USB_SERIAL.print(countCalibratedPoints());
      USB_SERIAL.print(F(" points,complete="));
      USB_SERIAL.println(calData.complete);
    } else {
      USB_SERIAL.println(F("CAL:FLASH,NODATA"));
    }

  } else if (cmd == "CAL FLASHCLEAR" || cmd == "CAL:FLASHCLEAR") {
    // Clear calibration from Flash
    USB_SERIAL.println(F("CAL:FLASH,CLEARING..."));
    if (clearCalibrationFromFlash()) {
      USB_SERIAL.println(F("CAL:FLASH,CLEARED"));
    }

  } else if (cmd == "CAL FLASHINFO" || cmd == "CAL:FLASHINFO") {
    // Show Flash calibration info
    USB_SERIAL.print(F("CAL:FLASH,HAS_DATA,"));
    USB_SERIAL.println(hasFlashCalibration() ? F("YES") : F("NO"));
    USB_SERIAL.print(F("CAL:SOURCE,"));
    USB_SERIAL.println(getCalSourceName());

  } else if (cmd == "CAL SOURCE" || cmd == "CAL:SOURCE") {
    // Show calibration source
    USB_SERIAL.print(F("CAL:SOURCE,"));
    USB_SERIAL.println(getCalSourceName());

  } else if (cmd == "CAL OFFSETS" || cmd == "CAL:OFFSETS") {
    // Print 2D calibration offsets (compact format)
    // Format: RPM,T0,T1,T2,...,T60
    USB_SERIAL.println(F("CAL:OFFSETS,2D"));
    USB_SERIAL.println(F("# RPM,BTDC0,BTDC1,...,BTDC60 (values in 0.01deg)"));
    for (uint8_t rpmIdx = 0; rpmIdx < CAL_RPM_POINTS; rpmIdx++) {
      USB_SERIAL.print(rpmIdx * 250);  // RPM value
      for (uint8_t timingDeg = 0; timingDeg < CAL_TIMING_POINTS; timingDeg++) {
        USB_SERIAL.print(F(","));
        USB_SERIAL.print(calData.offsets[rpmIdx][timingDeg]);
      }
      USB_SERIAL.println();
    }
    USB_SERIAL.println(F("CAL:END"));

  } else if (cmd.startsWith("CAL OFFSET ") || cmd.startsWith("CAL:OFFSET,")) {
    // Get single offset: CAL OFFSET <rpmIdx>,<timingDeg>
    int idx = cmd.indexOf(' ', 4);
    if (idx < 0) idx = cmd.indexOf(',', 4);
    if (idx > 0) {
      String params = cmd.substring(idx + 1);
      int comma = params.indexOf(',');
      if (comma > 0) {
        uint8_t rpmIdx = params.substring(0, comma).toInt();
        uint8_t timingDeg = params.substring(comma + 1).toInt();
        if (rpmIdx < CAL_RPM_POINTS && timingDeg < CAL_TIMING_POINTS) {
          int16_t offset = calData.offsets[rpmIdx][timingDeg];
          uint8_t calibrated = (calData.calibrated[rpmIdx][timingDeg / 8] >> (timingDeg % 8)) & 1;
          USB_SERIAL.print(F("CAL:OFFSET,"));
          USB_SERIAL.print(rpmIdx * 250);
          USB_SERIAL.print(F("RPM,"));
          USB_SERIAL.print(timingDeg);
          USB_SERIAL.print(F("deg,"));
          USB_SERIAL.print(offset);
          USB_SERIAL.print(F(","));
          USB_SERIAL.println(calibrated ? F("CAL") : F("UNCAL"));
        } else {
          USB_SERIAL.println(F("CAL:OFFSET,INVALID"));
        }
      }
    }

  } else if (cmd == "CAL CLEAR" || cmd == "CAL:CLEAR") {
    // Clear calibration data
    clearCalibrationData();
    config.calibration.complete = 0;
    config.checksum = calculateChecksum(&config);
    USB_SERIAL.println(F("CAL:CLEARED"));

  } else if (cmd == "CAL INFO" || cmd == "CAL:INFO") {
    // Show calibration info and statistics
    USB_SERIAL.println(F("=== CALIBRATION INFO ==="));
    USB_SERIAL.print(F("Grid: "));
    USB_SERIAL.print(CAL_RPM_POINTS);
    USB_SERIAL.print(F(" RPM x "));
    USB_SERIAL.print(CAL_TIMING_POINTS);
    USB_SERIAL.print(F(" Timing = "));
    USB_SERIAL.print(CAL_TOTAL_POINTS);
    USB_SERIAL.println(F(" points"));
    USB_SERIAL.print(F("RPM Range: 0-"));
    USB_SERIAL.print((CAL_RPM_POINTS - 1) * 250);
    USB_SERIAL.println(F(" (step 250)"));
    USB_SERIAL.print(F("Timing Range: 0-"));
    USB_SERIAL.print(CAL_TIMING_POINTS - 1);
    USB_SERIAL.println(F(" BTDC (step 1deg)"));
    USB_SERIAL.print(F("Storage Precision: 0.01 deg/unit"));
    USB_SERIAL.print(F(" (int16: ±327.67deg)"));
    USB_SERIAL.println();
    USB_SERIAL.print(F("Internal Precision: 0.001 deg"));
    USB_SERIAL.println(F(" (fuzzy logic)"));
    USB_SERIAL.print(F("Enabled: "));
    USB_SERIAL.println(config.calibration.enabled ? F("YES") : F("NO"));
    USB_SERIAL.print(F("Margin: "));
    USB_SERIAL.print(config.calibration.marginError);
    USB_SERIAL.println(F(" (0.01deg)"));
    USB_SERIAL.print(F("Complete: "));
    USB_SERIAL.println(calData.complete ? F("YES") : F("NO"));
    USB_SERIAL.print(F("Magic: 0x"));
    USB_SERIAL.println(calData.magic, HEX);

    // Count calibrated points
    uint32_t calibratedCount = 0;
    for (uint8_t r = 0; r < CAL_RPM_POINTS; r++) {
      for (uint8_t t = 0; t < CAL_TIMING_POINTS; t++) {
        if ((calData.calibrated[r][t / 8] >> (t % 8)) & 1) {
          calibratedCount++;
        }
      }
    }
    USB_SERIAL.print(F("Calibrated: "));
    USB_SERIAL.print(calibratedCount);
    USB_SERIAL.print(F("/"));
    USB_SERIAL.print(CAL_TOTAL_POINTS);
    USB_SERIAL.print(F(" ("));
    USB_SERIAL.print((calibratedCount * 100) / CAL_TOTAL_POINTS);
    USB_SERIAL.println(F("%)"));
    USB_SERIAL.println(F("========================"));

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
  } else if (p == "QS") {
    // Quick Shifter config: enabled,baseline,threshold,minRpm,maxRpm,sensitivity,currentAdc
    USB_SERIAL.print(F("QS:"));
    USB_SERIAL.print(config.quickShifter.enabled);
    USB_SERIAL.print(F(","));
    USB_SERIAL.print(config.quickShifter.baseline);
    USB_SERIAL.print(F(","));
    USB_SERIAL.print(config.quickShifter.threshold);
    USB_SERIAL.print(F(","));
    USB_SERIAL.print(config.quickShifter.minRpm);
    USB_SERIAL.print(F(","));
    USB_SERIAL.print(config.quickShifter.maxRpm);
    USB_SERIAL.print(F(","));
    USB_SERIAL.print(config.quickShifter.sensitivity);
    USB_SERIAL.print(F(","));
    USB_SERIAL.println(runtime.qsCurrentAdc);
  } else if (p == "QSADC") {
    // Live ADC reading for calibration
    USB_SERIAL.print(F("QSADC:"));
    USB_SERIAL.println(runtime.qsCurrentAdc);
  } else if (p == "QSMAP") {
    // QS cut time map: 21 values (0-20000 RPM in 1000 RPM steps)
    USB_SERIAL.print(F("QSMAP:"));
    for (int i = 0; i < QS_TABLE_SIZE; i++) {
      if (i > 0) USB_SERIAL.print(F(","));
      USB_SERIAL.print(config.quickShifter.cutTimeMap[i]);
    }
    USB_SERIAL.println();
  } else if (p == "DEFAULTS") {
    // Flash defaults info: flags,writeCount,sourceMap
    // flags: 0=none, 1=settings, 2=map, 4=QS
    USB_SERIAL.print(F("DEFAULTS:"));
    USB_SERIAL.print(getFlashFlags());
    USB_SERIAL.print(F(","));
    USB_SERIAL.print(getFlashWriteCount());
    USB_SERIAL.print(F(","));
    USB_SERIAL.println(getFlashSourceMap());
  } else if (p == "FLASHSETTINGS") {
    // READ ONLY: Get settings stored in Flash (for UI display)
    getFlashSettingsReadOnly();
  } else if (p == "FLASHQS") {
    // READ ONLY: Get QS config stored in Flash
    getFlashQSReadOnly();
  } else if (p == "FLASHQSMAP") {
    // READ ONLY: Get QS map stored in Flash
    getFlashQSMapReadOnly();
  } else if (p == "FLASHMAP") {
    // READ ONLY: Get ignition map stored in Flash
    getFlashMapReadOnly();
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
  } else if (name == "QS") {
    // Format: enabled,baseline,threshold,minRpm,maxRpm,sensitivity
    int idx = 0;
    int values[6];
    int start = 0;
    for (int i = 0; i <= val.length() && idx < 6; i++) {
      if (i == val.length() || val.charAt(i) == ',') {
        values[idx++] = val.substring(start, i).toInt();
        start = i + 1;
      }
    }
    if (idx >= 6) {
      config.quickShifter.enabled = values[0] ? 1 : 0;
      config.quickShifter.baseline = constrain(values[1], 0, 4095);
      config.quickShifter.threshold = constrain(values[2], 0, 4095);
      config.quickShifter.minRpm = constrain(values[3], 0, 15000);
      config.quickShifter.maxRpm = constrain(values[4], 1000, 20000);
      config.quickShifter.sensitivity = constrain(values[5], 1, 50);
      config.checksum = calculateChecksum(&config);
      USB_SERIAL.println(F("OK"));
    } else {
      USB_SERIAL.println(F("Invalid QS params"));
    }
  } else if (name == "QSMAP") {
    // Format: v0,v1,v2,...,v20 (21 values)
    int idx = 0;
    int start = 0;
    for (int i = 0; i <= val.length() && idx < QS_TABLE_SIZE; i++) {
      if (i == val.length() || val.charAt(i) == ',') {
        int v = val.substring(start, i).toInt();
        config.quickShifter.cutTimeMap[idx++] = constrain(v, 10, 250);
        start = i + 1;
      }
    }
    if (idx == QS_TABLE_SIZE) {
      config.checksum = calculateChecksum(&config);
      USB_SERIAL.println(F("OK"));
    } else {
      USB_SERIAL.print(F("Invalid QSMAP: got "));
      USB_SERIAL.print(idx);
      USB_SERIAL.println(F(" values"));
    }
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
// 23 fields max, each ~10 chars + commas + header = ~280 chars max
static char rtBuffer[384];  // Increased for calibration debug fields

// Binary telemetry packet structure (packed for efficient transmission)
// Size: 18 bytes vs ~120+ bytes for ASCII - 6x reduction in USB traffic
struct __attribute__((packed)) TelemetryPacket {
  uint8_t start;        // 0xAA - frame start marker
  uint16_t rpm;         // 0-20000
  int16_t timing;       // scaled x10 (-100 to 600 = -10.0° to 60.0°)
  uint16_t period;      // timer ticks / 100 (for RPM calc)
  uint8_t map;          // 0-5 (active map)
  uint8_t limiter;      // 0-4 (limiter stage)
  int16_t temp;         // temperature x10 (C)
  uint16_t battery;     // millivolts
  uint8_t flags;        // status flags (engineRunning, overheat, lowBatt, kill, defMap, ignEn, sdOk, predictive)
  uint8_t flags2;       // extended flags (bit0=timingClamped)
  uint8_t checksum;     // XOR checksum of all bytes
  uint8_t end;          // 0x55 - frame end marker
};

// Calculate XOR checksum for binary packet
static uint8_t calcPacketChecksum(uint8_t* data, uint8_t len) {
  uint8_t chk = 0;
  for (uint8_t i = 0; i < len; i++) {
    chk ^= data[i];
  }
  return chk;
}

// Send binary telemetry (much faster than ASCII)
void sendBinaryTelemetry(void) {
  TelemetryPacket pkt;
  pkt.start = 0xAA;
  pkt.rpm = runtime.currentRpm;
  pkt.timing = runtime.currentTimingScaled / 10;  // scale down for int16
  pkt.period = (uint16_t)(runtime.period / 100);  // scale down to fit uint16
  pkt.map = config.activeMap;
  pkt.limiter = runtime.limiterStage;
  pkt.temp = rawToTempX10(runtime.tempRaw);
  pkt.battery = rawToVoltageX100(runtime.batteryRaw, config.adcCal.battScaleScaled);

  // Build flags byte
  pkt.flags = 0;
  if (runtime.engineRunning) pkt.flags |= 0x01;
  if (runtime.overheating) pkt.flags |= 0x02;
  if (runtime.lowBattery) pkt.flags |= 0x04;
  if (runtime.killActive) pkt.flags |= 0x08;
  if (runtime.usingDefaultMap) pkt.flags |= 0x10;
  if (runtime.ignitionEnabled) pkt.flags |= 0x20;
  if (runtime.sdCardOk) pkt.flags |= 0x40;
  if (runtime.predictiveMode) pkt.flags |= 0x80;

  // Build flags2 byte (extended)
  pkt.flags2 = 0;
  if (runtime.timingClamped) pkt.flags2 |= 0x01;

  // Calculate checksum (exclude start, checksum, end)
  pkt.checksum = calcPacketChecksum((uint8_t*)&pkt.rpm, sizeof(pkt) - 4);
  pkt.end = 0x55;

  // Single write - minimal blocking
  USB_SERIAL.write((uint8_t*)&pkt, sizeof(pkt));
}

void sendRealtimeData(void) {
  // Use binary protocol if enabled (6x less USB traffic, faster parsing)
  if (binaryTelemetryMode) {
    sendBinaryTelemetry();
    return;
  }

  // ASCII Format: RT:RPM,TIMING,TEMP,BATT,CHARGING,MAP,LIMITER,FLAGS,PEAK,CPU,RAM,TRIG,CUT,ENGTYPE,CFGSRC,QSADC,TEMPRAW,BATTRAW,CHRGRAW,DRPM,PHASE,FLAGS2,SKIP
  // CFGSRC: 0=SD, 1=Flash, 2=Hardcoded
  // QSADC: Quick Shifter ADC raw value (0-4095)
  // TEMPRAW,BATTRAW,CHRGRAW: Raw ADC values for calibration (0-4095)
  // DRPM: RPM change per cycle (+ = accel, - = decel)
  // PHASE: Phase correction in ticks (±50 max)
  // FLAGS2: Extended flags (bit0=timingClamped, bit1-7=reserved)
  // SKIP: Skipped triggers count (race condition diagnostic)
  // Using single buffer write instead of 14+ print() calls to reduce blocking

  // Build flags byte (original)
  uint8_t flags = 0;
  if (runtime.engineRunning) flags |= 0x01;
  if (runtime.overheating) flags |= 0x02;
  if (runtime.lowBattery) flags |= 0x04;
  if (runtime.killActive) flags |= 0x08;
  if (runtime.usingDefaultMap) flags |= 0x10;
  if (runtime.ignitionEnabled) flags |= 0x20;
  if (runtime.sdCardOk) flags |= 0x40;
  if (runtime.predictiveMode) flags |= 0x80;

  // Build flags2 byte (extended)
  uint8_t flags2 = 0;
  if (runtime.timingClamped) flags2 |= 0x01;

  // Build entire string in buffer (single write = less blocking)
  // Calibration fields: calMode, calRpmIdx, calTimingDeg, calMisfires, calIteration, debugTriggers, debugIgnitions, debugCaptures, rawExti
  int len = snprintf(rtBuffer, sizeof(rtBuffer),
    "RT:%u,%d,%d,%d,%d,%u,%u,%u,%u,%u,%u,%d,%u,%u,%u,%u,%u,%u,%u,%d,%d,%u,%lu,%u,%u,%u,%lu,%u,%lu,%lu,%lu,%lu\n",
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
    config.engineType,
    runtime.configSource,
    runtime.qsCurrentAdc,
    runtime.tempRaw,
    runtime.batteryRaw,
    runtime.chargingRaw,
    runtime.dRpm,                // RPM change per cycle
    runtime.phaseCorrectionUs,   // Phase correction in ticks
    flags2,                      // Extended flags (timingClamped, etc)
    runtime.skippedTriggers,     // Race condition skip counter
    // Calibration fields (24-32)
    calState.mode,               // 24: 0=off, 1=running, 2=paused, 3=complete
    calState.currentRpmIdx,      // 25: 0-80
    calState.currentTimingDeg,   // 26: 0-60
    calState.totalMisfires,      // 27: Total misfires
    calState.iteration,          // 28: Current iteration
    calState.debugTriggerCount,  // 29: Total triggers sent
    runtime.ignitionCount,       // 30: Total ignitions fired
    calState.debugCaptureCount,  // 31: Total captures detected (when calibrating)
    calState.debugRawExtiCount   // 32: Total raw EXTI fires (always counted)
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
  USB_SERIAL.println(F("BINARY ON|OFF - Toggle binary telemetry"));
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
  // SD_DETECT pin is HIGH when card is inserted (directly connected in socket)
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

  // DEBUG: Print struct sizes
  USB_SERIAL.print(F("sizeof CDIConfig: ")); USB_SERIAL.println(sizeof(CDIConfig));
  USB_SERIAL.print(F("sizeof FlashDefaults: ")); USB_SERIAL.println(sizeof(FlashDefaults));
  USB_SERIAL.print(F("sizeof QuickShifterConfig: ")); USB_SERIAL.println(sizeof(QuickShifterConfig));
  USB_SERIAL.flush();

  // ========================================
  // BOOT SEQUENCE: SD -> Flash -> Hardcoded
  // ========================================
  USB_SERIAL.println(F("Loading config..."));
  USB_SERIAL.flush();

  // Step 1: Load hardcoded defaults as baseline
  USB_SERIAL.println(F("  [1] Loading hardcoded baseline..."));
  loadHardcodedDefaults();
  runtime.configSource = 2;  // Hardcoded (baseline)

  // Step 2: Try SD card first
  USB_SERIAL.println(F("  [2] Checking SD card..."));
  USB_SERIAL.flush();
  bool sdConfigOk = false;

  if (SD.begin(SD_DETECT_NONE)) {
    runtime.sdCardOk = 1;
    USB_SERIAL.println(F("  SD init OK"));
    SD.mkdir(CONFIG_FOLDER);

    // Try to load config from SD
    sdConfigOk = loadConfigFromSDWithValidation();

    if (sdConfigOk) {
      USB_SERIAL.println(F("  SD config + maps OK"));
    } else {
      USB_SERIAL.println(F("  SD config/maps invalid, trying Flash..."));
    }
  } else {
    runtime.sdCardOk = 0;
    USB_SERIAL.println(F("  SD init failed, trying Flash..."));
  }

  // Step 3: If SD failed or maps invalid, try Flash
  if (!sdConfigOk) {
    USB_SERIAL.println(F("  [3] Checking Flash..."));
    USB_SERIAL.flush();

    if (hasCustomDefaults()) {
      if (loadDefaultsFromFlash()) {
        // Validate Flash maps
        if (validateConfigMaps(&config)) {
          runtime.configSource = 1;  // Flash
          config.activeMap = 0;  // Flash only has 1 map, use slot 0
          USB_SERIAL.println(F("  Flash config + maps OK"));
        } else {
          USB_SERIAL.println(F("  Flash maps invalid, using hardcoded"));
          loadHardcodedDefaults();
          runtime.configSource = 2;  // Hardcoded
        }
      } else {
        USB_SERIAL.println(F("  Flash load failed, using hardcoded"));
      }
    } else {
      USB_SERIAL.println(F("  No Flash data, using hardcoded"));
    }
  }

  // Step 4: If SD OK but no config file, save current config to SD
  if (runtime.sdCardOk && !sdConfigOk) {
    createAllSDFiles();
    saveConfigToSD();
    if (runtime.sdCardOk) {
      USB_SERIAL.println(F("  Config saved to SD for next boot"));
    }
  } else if (runtime.sdCardOk) {
    createAllSDFiles();
  }

  USB_SERIAL.print(F("Config source: "));
  USB_SERIAL.println(runtime.configSource == 0 ? F("SD") :
                     (runtime.configSource == 1 ? F("FLASH") : F("HARDCODED")));
  USB_SERIAL.flush();

  // ========================================
  // LOAD 2D CALIBRATION DATA (Priority: SD → Flash → Hardcoded)
  // ========================================
  USB_SERIAL.println(F("Loading calibration..."));

  // Start with hardcoded defaults (all zeros)
  initHardcodedCalibration();

  bool calLoaded = false;

  // Priority 1: Try SD Card first (if available and enabled)
  if (runtime.sdCardOk && config.calibration.enabled) {
    if (loadCalibrationFromSD()) {
      calSource = CAL_SOURCE_SD;
      calLoaded = true;
      USB_SERIAL.print(F("  Source: SD Card - "));
      USB_SERIAL.print(calData.complete ? F("COMPLETE") : F("PARTIAL"));
      USB_SERIAL.print(F(" ("));
      USB_SERIAL.print(countCalibratedPoints());
      USB_SERIAL.print(F("/"));
      USB_SERIAL.print(CAL_TOTAL_POINTS);
      USB_SERIAL.println(F(" points)"));
    }
  }

  // Priority 2: Try Flash backup if SD failed
  if (!calLoaded && flashAccessible) {
    if (loadCalibrationFromFlash()) {
      calLoaded = true;
      USB_SERIAL.print(F("  Source: Flash Backup - "));
      USB_SERIAL.print(calData.complete ? F("COMPLETE") : F("PARTIAL"));
      USB_SERIAL.print(F(" ("));
      USB_SERIAL.print(countCalibratedPoints());
      USB_SERIAL.print(F("/"));
      USB_SERIAL.print(CAL_TOTAL_POINTS);
      USB_SERIAL.println(F(" points)"));
    }
  }

  // Priority 3: Use hardcoded defaults (already initialized)
  if (!calLoaded) {
    USB_SERIAL.println(F("  Source: Hardcoded Defaults (all zeros)"));
    USB_SERIAL.println(F("  Calibration will use map values only"));
    config.calibration.complete = 0;
    calData.complete = 0;
  }

  USB_SERIAL.print(F("  Calibration Source: "));
  USB_SERIAL.println(getCalSourceName());

  // Initialize lookup tables
  initLookupTables();
  initPeriodLookup();  // Period-to-index table for fast ISR (no division)

  // Initialize DWT cycle counter for accurate CPU measurement
  initDWT();

  // Setup timers with direct register access
  setupTimers();

  // Setup calibration timers (TIM16 for RPM gen, EXTI for capture)
  setupCalibrationTimers();
  memset(&calState, 0, sizeof(CalibrationState));

  // CRITICAL: Set interrupt priorities for deterministic ignition timing
  // Priority 0 = highest, larger number = lower priority
  // Ignition MUST always win over capture for guaranteed timing accuracy
  //
  // Priority order (from expert review):
  // 0: SysTick - system timing (millis, USB)
  // 1: TIM3 - Ignition fire (MOST CRITICAL - timing accuracy)
  // 2: TIM4 - CDI pulse end (less critical than fire timing)
  // 3: TIM2 - VR capture (can be preempted by ignition)
  //
  NVIC_SetPriority(SysTick_IRQn, 0);           // millis() - highest
  HAL_NVIC_SetPriority(TIM5_IRQn, 1, 0);       // Ignition fire - CRITICAL (32-bit)
  HAL_NVIC_SetPriority(TIM4_IRQn, 2, 0);       // CDI pulse end
  HAL_NVIC_SetPriority(TIM2_IRQn, 3, 0);       // VR capture - can wait

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
  USB_SERIAL.println(F("Watchdog: Ready"));

  // System ready - enable trigger processing
  runtime.configReady = 1;
  USB_SERIAL.println(F("READY!\n"));
}

// ============================================================================
// CALIBRATION STATE MACHINE - 2D with Fuzzy Logic
// ============================================================================

// Store calibration offset for current point
void storeCalibrationOffset(void) {
  // Convert from millidegrees (0.001°) to centidegrees (0.01°) storage
  // accumulatedOffset is in 0.001°, storage is in 0.01°
  // Divide by 10 with rounding
  int32_t storedValue = (calState.accumulatedOffset + 5) / 10;

  // Clamp to int16_t range (±327.67°, way more than needed)
  if (storedValue > 32767) storedValue = 32767;
  if (storedValue < -32767) storedValue = -32767;

  // Store in 2D map
  uint8_t rpmIdx = calState.currentRpmIdx;
  uint8_t timingDeg = calState.currentTimingDeg;

  calData.offsets[rpmIdx][timingDeg] = (int16_t)storedValue;
  markPointCalibrated(rpmIdx, timingDeg);

  calState.completedPoints++;
}

// Move to next calibration point
void moveToNextCalPoint(void) {
  // Reset fuzzy state
  calState.accumulatedOffset = 0;
  calState.iteration = 0;
  calState.convergedCount = 0;
  calState.sampleSum = 0;
  calState.sampleCount = 0;
  calState.errorHistoryIdx = 0;
  calState.misfireCount = 0;        // Reset misfire counter for new point
  calState.triggersSent = 0;        // Reset trigger counter
  memset((void*)calState.lastErrors, 0, sizeof(calState.lastErrors));

  // Move to next RPM
  calState.currentRpmIdx++;

  // Check if done with this timing degree
  if (calState.currentRpmIdx >= CAL_RPM_POINTS ||
      (calState.currentRpmIdx * 250) > CAL_RPM_END) {
    // Move to next timing degree
    calState.currentTimingDeg++;
    calState.currentRpmIdx = CAL_RPM_START / 250;  // Reset to start RPM

    // Check if all done
    if (calState.currentTimingDeg >= CAL_TIMING_POINTS) {
      // Calibration complete!
      calData.complete = 1;
      calData.enabled = 1;
      config.calibration.complete = 1;
      config.calibration.enabled = 1;
      calDataLoaded = 1;

      stopCalibrationTimers();
      calState.mode = CAL_MODE_COMPLETE;

      USB_SERIAL.println(F("CAL:COMPLETE"));
      USB_SERIAL.print(F("CAL:TOTAL,"));
      USB_SERIAL.println(calState.completedPoints);
      USB_SERIAL.print(F("CAL:PASSED,"));
      USB_SERIAL.println(calState.passedPoints);
      USB_SERIAL.print(F("CAL:FAILED,"));
      USB_SERIAL.println(calState.failedPoints);
      USB_SERIAL.print(F("CAL:MISFIRES,"));
      USB_SERIAL.println(calState.totalMisfires);
      USB_SERIAL.print(F("CAL:MISFIRE_POINTS,"));
      USB_SERIAL.println(calState.misfirePoints);
      USB_SERIAL.println(F("CAL:Use 'CAL SAVE' to store to SD"));
      return;
    }

    // Print progress for new timing degree
    USB_SERIAL.print(F("CAL:TIMING,"));
    USB_SERIAL.print(calState.currentTimingDeg);
    USB_SERIAL.println(F("°"));
  }

  // Set target RPM for new point
  uint16_t targetRpm = calState.currentRpmIdx * 250;
  if (targetRpm < CAL_RPM_START) targetRpm = CAL_RPM_START;
  updateCalibrationRpm(targetRpm);

  // Set override timing for calibration
  calState.overrideTimingEnabled = 1;
  calState.overrideTimingScaled = calState.currentTimingDeg * 100;  // Convert to 0.01° units

  calState.pointStartMs = millis();
  calState.settleStartMs = millis();
  calState.rpmStable = 0;
}

void processCalibration(void) {
  // Only process if calibration is running
  if (calState.mode != CAL_MODE_RUNNING) return;

  // Ignition capture is handled by polling in calRpmCallback
  // Polling on PB3 sets captureReady flag when rising edge detected

  uint32_t now = millis();

  // Wait for RPM to stabilize after change
  if (!calState.rpmStable) {
    if (now - calState.settleStartMs >= CAL_SETTLE_MS) {
      calState.rpmStable = 1;
      calState.triggersSent = 0;  // Reset trigger counter when stable
    }
    return;
  }

  // =========================================================================
  // MISFIRE DETECTION
  // If we've sent N triggers without getting a capture, it's a misfire
  // =========================================================================
  if (calState.triggersSent >= CAL_MISFIRE_TIMEOUT_TRIGGERS) {
    calState.misfireCount++;
    calState.totalMisfires++;
    calState.triggersSent = 0;  // Reset counter

    // Report misfire
    USB_SERIAL.print(F("CAL:MISFIRE,"));
    USB_SERIAL.print(calState.currentRpmIdx * 250);
    USB_SERIAL.print(F("RPM,"));
    USB_SERIAL.print(calState.currentTimingDeg);
    USB_SERIAL.print(F("°,#"));
    USB_SERIAL.println(calState.misfireCount);

    // Check if too many misfires at this point
    if (calState.misfireCount >= CAL_MISFIRE_MAX_PER_POINT) {
      // Mark point as failed due to excessive misfires
      storeCalibrationOffset();  // Store current offset anyway
      calState.misfirePoints++;
      calState.failedPoints++;

      // Mark this point as misfire-prone for runtime safety retard
      markPointMisfireProne(calState.currentRpmIdx, calState.currentTimingDeg);

      USB_SERIAL.print(F("CAL:MISFIRE_FAIL,"));
      USB_SERIAL.print(calState.currentRpmIdx * 250);
      USB_SERIAL.print(F(","));
      USB_SERIAL.print(calState.currentTimingDeg);
      USB_SERIAL.print(F(",misfires="));
      USB_SERIAL.println(calState.misfireCount);

      // Move to next point
      moveToNextCalPoint();
      return;
    }

    // Small delay before retry
    calState.settleStartMs = now;
    calState.rpmStable = 0;
    return;
  }

  // Process captured ignition timing
  if (calState.captureReady) {
    calState.captureReady = 0;

    // Calculate actual timing from captured delay
    // delay (in ticks) -> degrees
    // At 10MHz: 1 tick = 0.1µs
    // degrees = (delay_ticks / period_ticks) * 360

    uint32_t periodTicks = calState.rpmPeriodTicks * 2;  // Full period

    if (periodTicks > 0) {
      // Convert measured delay to millidegrees (0.001°)
      // measuredDelay = (delay * 360000) / period
      int32_t measuredDelayMilliDeg = ((int64_t)calState.capturedDelay * 360000LL) / periodTicks;

      // =========================================================================
      // CRITICAL FIX: Convert expected timing to expected delay
      // This accounts for trigger angle and predictive/normal mode
      // =========================================================================
      // Trigger angle in millidegrees
      int32_t triggerAngleMilliDeg = (config.trigger.triggerAngleScaled * 10);  // triggerAngleScaled is in 0.01° units

      // Target timing in millidegrees
      int32_t targetTimingMilliDeg = calState.currentTimingDeg * 1000;

      // Calculate expected delay based on mode
      // Normal mode: delay = triggerAngle - timing (timing <= triggerAngle)
      // Predictive mode: delay = 360 + (triggerAngle - timing) (timing > triggerAngle)
      int32_t expectedDelayMilliDeg = triggerAngleMilliDeg - targetTimingMilliDeg;

      // Handle predictive mode (when timing > triggerAngle, delay becomes negative)
      if (expectedDelayMilliDeg < 0) {
        expectedDelayMilliDeg += 360000;  // Wrap around: fire next cycle
      }

      // Handle wrap-around for measured delay (in case of timer overflow detection)
      // If measured delay is close to 360°, it might be predictive mode
      // If expected is predictive (> 180°) but measured is small, add 360°
      if (expectedDelayMilliDeg > 180000 && measuredDelayMilliDeg < 90000) {
        // Likely missed the predictive cycle, measurement wrapped
        measuredDelayMilliDeg += 360000;
      }

      // Calculate error (measured delay - expected delay) accounting for accumulated offset
      // Positive error = ignition fired too late = need to advance = reduce delay
      // Negative error = ignition fired too early = need to retard = increase delay
      int32_t error = measuredDelayMilliDeg - expectedDelayMilliDeg - calState.accumulatedOffset;

      // Accumulate for averaging
      calState.sampleSum += error;
      calState.sampleCount++;
      calState.totalSamples++;
    }
  }

  // State machine update (every 30ms for faster convergence)
  if (now - calState.lastStateChangeMs < 30) return;
  calState.lastStateChangeMs = now;

  // Check if we have enough samples
  if (calState.sampleCount >= CAL_SAMPLES_PER_CHECK) {
    // Calculate average error in millidegrees
    int32_t avgError = calState.sampleSum / calState.sampleCount;
    calState.lastAvgError = avgError;

    // Reset sample accumulator
    calState.sampleSum = 0;
    calState.sampleCount = 0;

    // Convert margin from config (0.01° units) to millidegrees
    int32_t marginMilliDeg = config.calibration.marginError * 10;

    // Check convergence using fuzzy logic
    uint8_t convergeStatus = checkCalibrationConvergence(avgError, marginMilliDeg);

    if (convergeStatus > 0) {
      // Converged! Store offset
      storeCalibrationOffset();
      calState.passedPoints++;

      // Print progress (compact format)
      USB_SERIAL.print(F("CAL:OK,"));
      USB_SERIAL.print(calState.currentRpmIdx * 250);
      USB_SERIAL.print(F(","));
      USB_SERIAL.print(calState.currentTimingDeg);
      USB_SERIAL.print(F(","));
      USB_SERIAL.print((float)calState.accumulatedOffset / 1000.0, 3);
      USB_SERIAL.println(F("°"));

      // Move to next point
      moveToNextCalPoint();
      return;
    }

    // Not converged - apply fuzzy adjustment
    int32_t adjustment = fuzzyCalibrationAdjust(avgError);
    calState.accumulatedOffset += adjustment;

    calState.iteration++;

    // Check max iterations
    if (calState.iteration >= CAL_MAX_ITERATIONS) {
      // Store best effort
      storeCalibrationOffset();
      calState.failedPoints++;

      USB_SERIAL.print(F("CAL:MAXITER,"));
      USB_SERIAL.print(calState.currentRpmIdx * 250);
      USB_SERIAL.print(F(","));
      USB_SERIAL.print(calState.currentTimingDeg);
      USB_SERIAL.print(F(","));
      USB_SERIAL.print((float)avgError / 1000.0, 3);
      USB_SERIAL.println(F("°"));

      // Move to next point
      moveToNextCalPoint();
      return;
    }

    // Print retry info periodically
    if (calState.iteration % 10 == 0) {
      USB_SERIAL.print(F("CAL:ITER,"));
      USB_SERIAL.print(calState.iteration);
      USB_SERIAL.print(F(","));
      USB_SERIAL.print((float)avgError / 1000.0, 3);
      USB_SERIAL.print(F(",ADJ,"));
      USB_SERIAL.println((float)adjustment / 1000.0, 3);
    }
  }

  // Update RPM generator periodically (for stability)
  if (now - calState.settleStartMs >= CAL_RPM_HOLD_MS) {
    // RPM has been stable long enough, continue measurement
  }
}

// ============================================================================
// MAIN LOOP
// ============================================================================

void loop() {
  // Kick watchdog immediately at start of every loop iteration
  // This ensures watchdog is fed even if loop takes long time
  kickWatchdog();

  // Poll for calibration capture (non-blocking, called frequently)
  // This must be early in loop for best timing accuracy
  pollCalibrationCapture();

  // Mark start of loop iteration for CPU measurement
  cpuLoopStart();

  uint32_t now = millis();

  static uint32_t lastADC = 0;
  static uint32_t lastOutput = 0;
  static uint32_t lastUSB = 0;
  static uint32_t lastLog = 0;
  static uint32_t lastSdBusyCheck = 0;

  // Update CPU usage measurement
  updateCpuUsage();

  // Check SD card safety (detect removal/insertion)
  checkSDCardSafety();

  // Handle inputs (every loop)
  handleInputs();

  // Process quick shifter (needs frequent polling for fast response)
  processQuickShifter();

  // Calculate RPM from period (moved from ISR to reduce ISR time)
  // Only recalculate when period changes (avoid redundant division)
  static uint32_t lastPeriodForRpm = 0;
  if (runtime.period > 0 && runtime.engineRunning) {
    if (runtime.period != lastPeriodForRpm) {
      runtime.currentRpm = periodToRpm(runtime.period);
      lastPeriodForRpm = runtime.period;
    }
  } else {
    lastPeriodForRpm = 0;  // Reset when engine not running
  }

  // Update status LED (rate limited - 50ms is sufficient for visible patterns)
  static uint32_t lastLedUpdate = 0;
  if (now - lastLedUpdate >= 50) {
    lastLedUpdate = now;
    updateStatusLED();
  }

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

  // Process calibration state machine (if running)
  processCalibration();

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
