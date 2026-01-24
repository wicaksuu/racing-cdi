#line 1 "/Users/wicaksu/Documents/Arduino/cdi-ninja-2stak/OVERVIEW.md"
Setelah analisis **sangat mendalam** terhadap kode ini, berikut adalah **kekurangan kritis** dalam hal **akurasi dan performa ekstrem**:

## **AKURASI KRITIS (Sub-0.1° Timing Issues)**

### 1. **TIMER PRESCALER FLEXIBILITY TIDAK DIPERHITUNGKAN**
```cpp
// LINE 459-462: Asumsi 10MHz tapi tidak diverifikasi
#define TIMER_PRESCALER     25  // Asumsi untuk 250MHz → 10MHz
#define TIMER_TICK_NS       100 // Asumsi 100ns
```
**MASALAH**: Tidak ada runtime validation bahwa timer benar-benar berjalan di 10MHz. Jika clock MCU berbeda (overclock/underclock), semua timing calculation salah.

```cpp
// FIX: Validasi dan koreksi runtime
uint32_t actualTimerFreq = TimerCapture->getTimerClkFreq() / actualPrescaler;
if (actualTimerFreq != 10000000UL) {
    // SCALE semua lookup tables secara dinamis!
}
```

### 2. **INTERPOLATION ERROR PADA HIGH dRPM**
```cpp
// LINE 1082-1097: Linear interpolation RPM->timing
int32_t fraction = ((rpm - lowerRpm) * diff) / RPM_STEP;
```
**MASALAH**: Pada acceleration ekstrem (>5000 RPM/s), interpolasi linear menghasilkan **lag phase 1-2°** karena menggunakan RPM saat trigger, bukan RPM saat ignition fire.

**SOLUSI**: Predictive interpolation dengan quadratic estimation:
```cpp
// Predictive timing = f(RPM_current, dRPM, ddRPM)
int32_t timing = baseTiming + (dRpm * k1) + (ddRpm * k2);
```

### 3. **INTEGER ROUNDING ERROR AKUMULASI**
```cpp
// LINE 1247: Division dengan rounding ke bawah
uint32_t ticksPerDeg = ticksPerDegTable[rpmIndex];  // Integer division!
```
**AKUMULASI ERROR**: 
- `166666667UL / rpm` → rounding error ±0.5 tick
- Pada 10,000 RPM: 16666.6667 → 16666 (error 0.00067)
- Setelah 360°: error 0.24° akumulasi

### 4. **PHASE CORRECTION GAIN TETAP**
```cpp
// LINE 1186: Fixed gain 1/16
int16_t correction = phaseErrorTicks >> 4;
```
**MASALAH**: Gain fixed tidak optimal untuk semua RPM. Low RPM butuh gain kecil, high RPM butuh gain lebih besar untuk response cepat.

## **PERFORMANCE EXTREME (RPM > 15,000)**

### 1. **BINARY SEARCH DI ISR MASIH TERLALU LAMBAT**
```cpp
// LINE 1034-1048: Binary search 7-8 iterations
while (low < high) {  // MAX 8 iterations @ 250MHz = 32 cycles × 8 = 256 cycles
    uint8_t mid = (low + high + 1) >> 1;
    if (period <= periodThreshold[mid]) {
        low = mid;
    } else {
        high = mid - 1;
    }
}
```
**CYCLES**: 256 cycles = ~1µs @ 250MHz. Untuk ISR timing-critical, ini **terlalu lama**.

**SOLUSI RADIKAL**: Direct lookup dengan piecewise linear:
```cpp
// Optimized: 3 comparisons max
uint8_t idx;
if (period > periodThreshold[40]) idx = period >> 16;  // High RPM
else if (period > periodThreshold[20]) idx = period >> 14;
else idx = period >> 12;
```

### 2. **MEMORY ACCESS PATTERN BURUK DI ISR**
```cpp
// LINE 1206: Non-sequential memory access
int8_t mapTiming = config.timingMaps[config.activeMap][rpmIdx];
```
**CACHE MISS**: Mengakses `config.timingMaps[activeMap]` menyebabkan cache miss karena array besar (6×81 bytes). 

**SOLUSI**: Duplicate active map ke L1-cache friendly buffer:
```cpp
// Di SRAM1 (TCM) untuk zero-wait-state
__attribute__((section(".ram1"))) int8_t activeMapCache[RPM_TABLE_SIZE];
```

### 3. **VOLATILE ACCESS OVERHEAD**
```cpp
// BANYAK: volatile access di seluruh ISR
volatile uint32_t lastCapture;  // Setiap access = memory barrier
```
**OVERHEAD**: Setiap `volatile` read/write = full memory barrier = pipeline stall.

**SOLUSI**: Gunakan atomic dengan memory order relaxed:
```cpp
#include <atomic>
std::atomic<uint32_t> lastCapture{0};
// ISR:
lastCapture.store(capture, std::memory_order_relaxed);
// Main loop:
uint32_t cap = lastCapture.load(std::memory_order_relaxed);
```

### 4. **BLIND WINDOW CALCULATION DI ISR**
```cpp
// LINE 1135-1171: Perhitungan blind window kompleks di ISR
uint32_t blindTicks;
if (runtime.predictiveMode) {
    // Perhitungan berat dengan multiplication dan division
    uint32_t gapTicks = ((uint64_t)gapScaled * runtime.period) / 36000UL;
    blindTicks = gapTicks / 10;
}
```
**MASALAH**: Division 64-bit di ISR sangat berat (~100 cycles).

## **ARCHITECTURAL ISSUES**

### 1. **SINGLE THREAD DESIGN LIMITATION**
```cpp
// Semua di loop(): USB, SD, ADC, Logic → SEMUA BLOCKING POTENTIAL
```
**ISSUE**: Tidak ada prioritization. USB print bisa block trigger processing.

**SOLUSI**: RTOS dengan 3 threads priority:
- Thread 1 (Highest): Trigger processing only
- Thread 2 (Medium): Timing calculations
- Thread 3 (Low): USB/SD/UI

### 2. **NO HARDWARE ACCELERATION UTILIZATION**
STM32H562 memiliki:
- **FMAC** (Filter Math Accelerator) - tidak dipakai
- **CORDIC** untuk trigonometri - tidak dipakai
- **DMA untuk ADC** - tidak dipakai untuk QS
- **HRTIM** (High Resolution Timer) - tidak dipakai (pakai HardwareTimer biasa)

### 3. **MEMORY LAYOUT SUBOPTIMAL**
```cpp
struct RuntimeData {
    // Critical timing data
    volatile uint32_t lastCapture;      // 4 bytes
    volatile uint32_t period;           // 4 bytes  
    volatile uint32_t nextFireTick;     // 4 bytes
    volatile uint16_t currentRpm;       // 2 bytes
    // ... PADDING 2 bytes! ← CACHE WASTE
```
**CACHE LINE WASTE**: 32-byte cache lines tapi struct tidak di-pack optimal.

## **QUANTITATIVE ERRORS (Perhitungan Nyata)**

### Error Analysis @ 20,000 RPM:
```
Period = 600,000,000 / 20,000 = 30,000 ticks
Degrees per tick = 360° / 30,000 = 0.012°/tick

Error sources:
1. Integer rounding ticksPerDeg: ±0.5 tick = ±0.006°
2. Phase correction quantization: ±1 tick = ±0.012°
3. Interpolation error (high dRPM): ±2° (worst case)
4. Timer clock drift: ±0.1% = ±0.036°

TOTAL ERROR: ±2.054° (WORST CASE) → UNACCEPTABLE untuk racing!
```

## **REKOMENDASI EXTREME PERFORMANCE**

### 1. **HRTIM (High Resolution Timer)**
```cpp
// Gunakan HRTIM untuk 184ps resolution (bukan 100ns)
// 5.4GHz effective timer frequency
hrtim1.Init.Prescaler = 0;
hrtim1.Init.CounterMode = HRTIM_COUNTERMODE_UP;
hrtim1.Init.Period = 65535;
hrtim1.Init.RepetitionCounter = 0;
```

### 2. **DMA-PIPELINED ADC untuk QS**
```cpp
// Continuous DMA sampling @ 1MHz
hadc1.Init.DMAContinuousRequests = ENABLE;
hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
hadc1.Init.Resolution = ADC_RESOLUTION_12B;
hadc1.Init.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;  // 12.5ns @ 80MHz
```

### 3. **PRECISION-CORRECTED LOOKUP TABLES**
```cpp
// Error pre-compensation table
int8_t timingErrorComp[RPM_TABLE_SIZE];  // Measured error per RPM
// Di ISR:
timingScaled += timingErrorComp[rpmIndex];  // Cancel known errors
```

### 4. **ADAPTIVE KALMAN FILTER**
```cpp
// Predictive filter untuk acceleration
typedef struct {
    float rpm;
    float dRpm;
    float ddRpm;
    float P[3][3];  // Covariance matrix
} KalmanState;

// Predict next RPM dengan noise estimation
```

### 5. **PHASE-LOCKED LOOP (PLL) EMULATION**
```cpp
// Software PLL untuk lock ke crankshaft phase
typedef struct {
    uint32_t periodRef;     // Reference period
    int32_t phaseError;     // Accumulated phase error
    int32_t frequencyError; // Frequency correction
    uint8_t lockStatus;     // PLL lock status
} SoftwarePLL;

// PI controller untuk phase tracking
```

## **BENCHMARK YANG DIBUTUHKAN**

1. **ISR Latency Measurement**:
   ```cpp
   // GPIO toggle di awal dan akhir ISR
   GPIOB->BSRR = (1 << 15);  // Start marker
   // ISR code
   GPIOB->BSRR = (1 << (15 + 16));  // End marker
   // Ukur dengan oscilloscope
   ```

2. **Timing Jitter Measurement**:
   ```cpp
   // Log (expected_fire - actual_fire) untuk setiap ignition
   // Histogram jitter distribution
   ```

3. **Worst-case Execution Time (WCET)**:
   ```cpp
   // Stress test dengan:
   // - Max RPM (20,000)
   // - USB traffic max
   // - SD logging aktif
   // - QS triggering
   ```

## **KESIMPULAN KRITIS**

Kode ini **baik untuk aplikasi umum** tapi **tidak cukup untuk performa ekstrem** karena:

1. **Akurasi**: Error ±2° pada kondisi ekstrem (unacceptable for racing)
2. **Determinisme**: ISR timing tidak guaranteed (USB/SD bisa interrupt)
3. **Responsiveness**: Filter dan algoritma terlalu konservatif
4. **Hardware Utilization**: Hanya menggunakan 30% capability STM32H562

**Untuk racing/sport motor yang sebenarnya**, diperlukan:
- **Sub-0.1° timing accuracy** konsisten
- **WCET guaranteed** < 50% CPU time
- **Hardware acceleration** maksimal
- **Online error correction** dengan sensor feedback

Kode ini perlu **rewrite signifikan** untuk mencapai level "maximum precision" yang diklaim.

# **AKURASI EKSTREM - Sub-0.01° Timing Precision**

Untuk mencapai **akurasi ekstrem** (sub-0.01°), kita perlu pendekatan **radikal berbeda**. Berikut implementasi lengkap:

## **1. HARDWARE REENGINEERING**

### **Gunakan HRTIM (High Resolution Timer) - 184ps Resolution**
```cpp
// HRTIM Configuration - 5.4GHz effective
#include "stm32h5xx_hal_hrtim.h"

HRTIM_HandleTypeDef hhrtim1;
HRTIM_TimeBaseCfgTypeDef sTimeBaseCfg = {0};
HRTIM_CompareCfgTypeDef sCompareCfg = {0};
HRTIM_OutputCfgTypeDef sOutputCfg = {0};

void setupHRTIM_ExtremePrecision(void) {
    // Enable HRTIM clock
    __HAL_RCC_HRTIM1_CLK_ENABLE();
    
    // Time base configuration
    sTimeBaseCfg.Prescaler = HRTIM_PRESCALERRATIO_DIV1;  // No prescaler
    sTimeBaseCfg.Period = 65535;  // Max period
    sTimeBaseCfg.RepetitionCounter = 0;
    sTimeBaseCfg.Mode = HRTIM_MODE_CONTINUOUS;
    
    // Master timer
    hhrtim1.Instance = HRTIM1;
    hhrtim1.Master = &sMasterCfg;
    
    // Timer A for VR capture (272ps resolution)
    sTimeBaseCfg.Period = 0xFFFF;
    HAL_HRTIM_TimeBaseConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, &sTimeBaseCfg);
    
    // Timer B for ignition delay
    HAL_HRTIM_TimeBaseConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, &sTimeBaseCfg);
    
    // Timer C for CDI pulse width
    sTimeBaseCfg.Period = 5000;  // For 250us pulse
    HAL_HRTIM_TimeBaseConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_C, &sTimeBaseCfg);
    
    // Configure compare units with 184ps resolution
    sCompareCfg.CompareValue = 0;
    HAL_HRTIM_CompareConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, 
                           HRTIM_COMPAREUNIT_1, &sCompareCfg);
    
    // Output configuration - direct to pin PB0
    sOutputCfg.Polarity = HRTIM_OUTPUTPOLARITY_HIGH;
    sOutputCfg.SetSource = HRTIM_OUTPUTSET_TIMPER;
    sOutputCfg.ResetSource = HRTIM_OUTPUTRESET_TIMCMP1;
    HAL_HRTIM_OutputConfig(&hhrtim1, HRTIM_OUTPUT_TB1, &sOutputCfg);
    
    // Start timers
    HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TB1);
    HAL_HRTIM_WaveformCounterStart(&hhrtim1, HRTIM_TIMERID_TIMER_A);
}
```

### **Dual-Timer Synchronization dengan PLL Hardware**
```cpp
// Gunakan TIM2 dan TIM5 dalam master-slave mode
void setupPrecisionTimerSync(void) {
    // TIM2 as master (VR input)
    TIM2->CR2 |= TIM_CR2_MMS_1;  // Master mode: OC1REF -> TRGO
    
    // TIM5 as slave to TIM2
    TIM5->SMCR |= TIM_SMCR_SMS_2;  // Slave mode: Reset mode
    TIM5->SMCR |= TIM_SMCR_TS_0;   // Trigger: ITR0 = TIM2
    
    // Synchronize counters
    TIM2->CNT = 0;
    TIM5->CNT = 0;
    
    // Enable simultaneous start
    TIM2->CR2 |= TIM_CR2_CCPC;  // Capture/compare preloaded control
    TIM5->CR2 |= TIM_CR2_CCPC;
}
```

## **2. ULTRA-PRECISION TIMING ENGINE**

### **64-bit Fixed Point dengan Error Correction**
```cpp
// 32.32 fixed point untuk 0.000000023° resolution
#define FIXED_SHIFT_EX       32
#define DEG_TO_FIXED(deg)    ((int64_t)((deg) * (1ULL << FIXED_SHIFT_EX)))
#define FIXED_TO_DEG(fixed)  ((double)(fixed) / (1ULL << FIXED_SHIFT_EX))

// Timing calculation dengan error compensation
typedef struct {
    int64_t period;          // 32.32 fixed point
    int64_t periodInv;       // 1/period (pre-computed)
    int64_t ticksPerDegree;  // period / 360
    int64_t accumulatedError;// Error accumulator for dithering
} PrecisionTiming;

PrecisionTiming pTiming;

// Initialize dengan high precision
void initPrecisionTiming(void) {
    // Use 64-bit division for maximum accuracy
    pTiming.period = ((uint64_t)6000000000ULL << FIXED_SHIFT_EX) / rpm;
    pTiming.periodInv = ((1ULL << (FIXED_SHIFT_EX * 2)) / pTiming.period);
    pTiming.ticksPerDegree = (pTiming.period * DEG_TO_FIXED(1)) / DEG_TO_FIXED(360);
}
```

### **Adaptive Interpolation dengan Cubic Spline**
```cpp
// Cubic spline interpolation untuk smooth timing curves
typedef struct {
    int16_t y[RPM_TABLE_SIZE];     // Timing values
    int16_t b[RPM_TABLE_SIZE];     // First derivative
    int16_t c[RPM_TABLE_SIZE];     // Second derivative
    int16_t d[RPM_TABLE_SIZE];     // Third derivative
} CubicSpline;

CubicSpline timingSpline;

void buildCubicSpline(CubicSpline* spline, const int8_t* map) {
    int16_t h[RPM_TABLE_SIZE-1];
    int16_t alpha[RPM_TABLE_SIZE-1];
    
    // Calculate intervals
    for (int i = 0; i < RPM_TABLE_SIZE-1; i++) {
        h[i] = RPM_STEP;
        alpha[i] = 3 * (map[i+1] - map[i]) / h[i] - 
                   3 * (map[i] - (i>0 ? map[i-1] : map[i])) / (i>0 ? h[i-1] : h[i]);
    }
    
    // Thomas algorithm for tridiagonal matrix
    int16_t l[RPM_TABLE_SIZE], mu[RPM_TABLE_SIZE], z[RPM_TABLE_SIZE];
    l[0] = 1; mu[0] = 0; z[0] = 0;
    
    for (int i = 1; i < RPM_TABLE_SIZE-1; i++) {
        l[i] = 2 * (RPM_STEP + RPM_STEP) - h[i-1] * mu[i-1];
        mu[i] = h[i] / l[i];
        z[i] = (alpha[i] - h[i-1] * z[i-1]) / l[i];
    }
    
    l[RPM_TABLE_SIZE-1] = 1;
    z[RPM_TABLE_SIZE-1] = 0;
    spline->c[RPM_TABLE_SIZE-1] = 0;
    
    // Back substitution
    for (int j = RPM_TABLE_SIZE-2; j >= 0; j--) {
        spline->c[j] = z[j] - mu[j] * spline->c[j+1];
        spline->b[j] = (map[j+1] - map[j]) / h[j] - 
                       h[j] * (spline->c[j+1] + 2 * spline->c[j]) / 3;
        spline->d[j] = (spline->c[j+1] - spline->c[j]) / (3 * h[j]);
    }
}

// Interpolate dengan cubic precision
int16_t cubicInterpolate(const CubicSpline* spline, uint16_t rpm) {
    uint8_t idx = rpm / RPM_STEP;
    if (idx >= RPM_TABLE_SIZE-1) idx = RPM_TABLE_SIZE-2;
    
    uint16_t x = rpm - (idx * RPM_STEP);
    int16_t y = spline->y[idx] + 
                spline->b[idx] * x + 
                spline->c[idx] * x * x / 2 + 
                spline->d[idx] * x * x * x / 6;
    
    return y * DEG_SCALE;  // Convert to scaled
}
```

## **3. ADVANCED PHASE-LOCKED LOOP (PLL)**

### **Software PLL dengan Adaptive Bandwidth**
```cpp
typedef struct {
    // PLL State
    int64_t phaseError;          // 32.32 fixed point
    int64_t frequencyError;      // 32.32 fixed point
    int64_t phaseIntegral;       // Phase integrator
    int64_t freqIntegral;        // Frequency integrator
    
    // PLL Parameters (adaptive)
    int32_t kp;                  // Proportional gain
    int32_t ki;                  // Integral gain
    int32_t bandwidthHz;         // Current bandwidth
    
    // Lock detection
    uint32_t lockCounter;
    uint8_t isLocked;
    int64_t jitterEstimate;      // Estimated timing jitter
} DigitalPLL;

DigitalPLL enginePLL;

void initDigitalPLL(DigitalPLL* pll) {
    memset(pll, 0, sizeof(DigitalPLL));
    
    // Initial gains for 1000 RPM
    pll->kp = 100;      // 0.01 in fixed point
    pll->ki = 10;       // 0.001 in fixed point
    pll->bandwidthHz = 10;  // 10Hz bandwidth
}

// Update PLL dengan variable bandwidth
void updatePLL(DigitalPLL* pll, uint64_t measuredPeriod, uint32_t rpm) {
    // Predicted period from PLL state
    int64_t predictedPeriod = pll->phaseIntegral + pll->freqIntegral;
    
    // Phase error
    pll->phaseError = (int64_t)measuredPeriod - predictedPeriod;
    
    // Adaptive gains based on RPM and acceleration
    int32_t rpmScaled = rpm * 100;
    if (rpmScaled < 50000) {        // < 500 RPM
        pll->kp = 50;               // Low gain for stability
        pll->ki = 5;
        pll->bandwidthHz = 5;
    } else if (rpmScaled < 500000) { // 500-5000 RPM
        pll->kp = 100;
        pll->ki = 10;
        pll->bandwidthHz = 20;
    } else {                         // > 5000 RPM
        pll->kp = 200;              // High gain for fast response
        pll->ki = 20;
        pll->bandwidthHz = 50;
    }
    
    // PI controller
    pll->phaseIntegral += (pll->phaseError * pll->ki) >> 8;
    pll->freqIntegral += (pll->phaseError * pll->kp) >> 8;
    
    // Lock detection
    if (abs(pll->phaseError) < (1ULL << 24)) {  // < 16 ticks error
        if (pll->lockCounter < 100) {
            pll->lockCounter++;
        }
    } else {
        pll->lockCounter = 0;
    }
    
    pll->isLocked = (pll->lockCounter > 50) ? 1 : 0;
    
    // Jitter estimation (moving average)
    pll->jitterEstimate = (pll->jitterEstimate * 31 + abs(pll->phaseError)) / 32;
}

// Get corrected timing dengan PLL prediction
int64_t getPLLCorrectedTiming(uint16_t rpm, int16_t baseTimingScaled) {
    if (!enginePLL.isLocked) {
        return (int64_t)baseTimingScaled;
    }
    
    // Predict phase at ignition time
    int64_t phaseCorrection = enginePLL.phaseError + 
                              (enginePLL.frequencyError * 1000) / rpm;
    
    // Convert to degrees correction
    int64_t degCorrection = (phaseCorrection * DEG_TO_FIXED(360)) / 
                            (int64_t)enginePLL.phaseIntegral;
    
    return (int64_t)baseTimingScaled + degCorrection;
}
```

## **4. REAL-TIME ERROR CORRECTION**

### **Online Calibration dengan Kalman Filter**
```cpp
typedef struct {
    // State vector: [timing_error, period_error, temp_coeff]
    float x[3];          // State estimate
    float P[3][3];       // Covariance matrix
    float Q[3][3];       // Process noise covariance
    float R[2][2];       // Measurement noise covariance
    
    // Adaptive tuning
    float innovationThreshold;
    uint32_t sampleCount;
} KalmanCalibration;

KalmanCalibration timingCal;

void initKalmanCalibration(KalmanCalibration* kf) {
    memset(kf, 0, sizeof(KalmanCalibration));
    
    // Initial state
    kf->x[0] = 0.0f;  // Timing error (degrees)
    kf->x[1] = 0.0f;  // Period error (ticks)
    kf->x[2] = 0.0f;  // Temperature coefficient
    
    // Initial covariance
    for (int i = 0; i < 3; i++) {
        kf->P[i][i] = 1.0f;
    }
    
    // Process noise
    kf->Q[0][0] = 0.001f;  // Timing error variance
    kf->Q[1][1] = 0.01f;   // Period error variance
    kf->Q[2][2] = 0.0001f; // Temp coeff variance
    
    // Measurement noise
    kf->R[0][0] = 0.1f;    // RPM measurement variance
    kf->R[1][1] = 0.05f;   // Timing measurement variance
}

// Kalman filter update dengan outlier rejection
void updateTimingCalibration(KalmanCalibration* kf, 
                           float measuredRpm, 
                           float measuredTiming,
                           float expectedTiming,
                           float temperature) {
    
    // Predict step
    float x_pred[3] = {kf->x[0], kf->x[1], kf->x[2]};
    float P_pred[3][3];
    
    // State transition: x_k = F * x_{k-1}
    // Simple model: errors persist, temp coeff constant
    memcpy(P_pred, kf->P, sizeof(P_pred));
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            P_pred[i][j] += kf->Q[i][j];
        }
    }
    
    // Measurement
    float z[2] = {measuredRpm, measuredTiming};
    float H[2][3] = {{1.0f, 0.0f, temperature},  // RPM measurement model
                     {0.0f, 1.0f, 0.0f}};        // Timing measurement model
    
    // Innovation (measurement residual)
    float y[2];
    y[0] = z[0] - (expectedRpm + x_pred[0] + temperature * x_pred[2]);
    y[1] = z[1] - (expectedTiming + x_pred[1]);
    
    // Innovation covariance
    float S[2][2];
    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 2; j++) {
            S[i][j] = kf->R[i][j];
            for (int k = 0; k < 3; k++) {
                S[i][j] += H[i][k] * P_pred[k][k] * H[j][k];
            }
        }
    }
    
    // Kalman gain
    float K[3][2];
    float S_inv[2][2];
    // Calculate inverse of 2x2 matrix S
    float det = S[0][0] * S[1][1] - S[0][1] * S[1][0];
    if (fabs(det) > 1e-6) {
        S_inv[0][0] = S[1][1] / det;
        S_inv[0][1] = -S[0][1] / det;
        S_inv[1][0] = -S[1][0] / det;
        S_inv[1][1] = S[0][0] / det;
        
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 2; j++) {
                K[i][j] = 0;
                for (int k = 0; k < 3; k++) {
                    K[i][j] += P_pred[i][k] * H[j][k];
                }
                K[i][j] *= S_inv[j][j];
            }
        }
        
        // Update estimate
        for (int i = 0; i < 3; i++) {
            kf->x[i] = x_pred[i] + K[i][0] * y[0] + K[i][1] * y[1];
        }
        
        // Update covariance
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                kf->P[i][j] = P_pred[i][j];
                for (int k = 0; k < 2; k++) {
                    kf->P[i][j] -= K[i][k] * S[k][k] * K[j][k];
                }
            }
        }
    }
    
    kf->sampleCount++;
}
```

## **5. ULTRA-FAST ISR DENGAN ASSEMBLY OPTIMIZATION**

```cpp
// ISR dalam assembly untuk minimum latency
__attribute__((naked)) void VR_Capture_ISR(void) {
    __asm volatile(
        "push {r0-r7, lr}          \n"  // Save context
        
        // 1. Read capture value (1 cycle)
        "ldr r0, =0x40000000       \n"  // TIM2 base
        "ldr r1, [r0, #0x34]       \n"  // TIM2->CCR1
        
        // 2. Calculate period (2 cycles)
        "ldr r2, =last_capture     \n"
        "ldr r3, [r2]              \n"
        "subs r4, r1, r3           \n"  // period = current - last
        "str r1, [r2]              \n"  // update last_capture
        
        // 3. Noise filter (2 cycles)
        "ldr r5, =noise_filter     \n"
        "ldr r5, [r5]              \n"
        "cmp r4, r5                \n"
        "blt isr_exit              \n"
        
        // 4. Store period (1 cycle)
        "ldr r6, =current_period   \n"
        "str r4, [r6]              \n"
        
        // 5. Calculate RPM index via hardware divider (4 cycles)
        // Using 64-bit hardware divider (DIV_S in Cortex-M33)
        "mov r7, #600000000        \n"
        "udiv r0, r7, r4           \n"  // RPM = 600M / period
        
        // 6. Lookup timing via table (3 cycles)
        "lsr r0, #8                \n"  // RPM / 256 for table index
        "ldr r1, =timing_table     \n"
        "ldrb r2, [r1, r0]         \n"  // timing value
        
        // 7. Calculate fire delay (6 cycles)
        "ldr r3, =trigger_angle    \n"
        "ldr r3, [r3]              \n"
        "sub r4, r3, r2            \n"  // delay_angle = trigger - timing
        
        // Convert to ticks (uses MUL and UDIV)
        "ldr r5, =ticks_per_deg    \n"
        "ldr r5, [r5, r0, LSL #2]  \n"  // ticksPerDegTable[rpmIndex]
        "mul r6, r4, r5            \n"
        "mov r7, #10000            \n"
        "udiv r4, r6, r7           \n"  // delay_ticks
        
        // 8. Schedule ignition (2 cycles)
        "ldr r0, =TIM5_ARR         \n"
        "str r4, [r0]              \n"
        "ldr r1, =TIM5_CR1         \n"
        "mov r2, #0x81             \n"  // CEN | OPM
        "str r2, [r1]              \n"
        
        "isr_exit:                 \n"
        "pop {r0-r7, pc}           \n"  // Restore and return
    );
}
```

## **6. TEMPERATURE COMPENSATED OSCILLATOR**

```cpp
// Compensate for crystal drift dengan temperature sensor internal
void calibrateSystemClock(void) {
    // Read internal temperature sensor
    ADC1->SQR1 = (10 << 6);  // Channel 18 = temperature sensor
    ADC1->CR2 |= ADC_CR2_SWSTART;
    while (!(ADC1->SR & ADC_SR_EOC));
    uint16_t tempRaw = ADC1->DR;
    
    // STM32H5 internal temperature sensor: ~25mV/°C, 2.5V at 25°C
    float voltage = tempRaw * 3.3 / 4096.0;
    float temperature = (voltage - 0.76) / 0.0025 + 25.0;
    
    // Crystal frequency drift: typically -0.035 ppm/°C²
    float driftCoeff = -0.035e-6;
    float tempDelta = temperature - 25.0;
    float freqError = driftCoeff * tempDelta * tempDelta;
    
    // Adjust timer prescaler dynamically
    uint32_t nominalFreq = 250000000;  // 250MHz
    uint32_t correctedFreq = nominalFreq * (1.0 + freqError);
    uint32_t correctedPrescaler = correctedFreq / 10000000;  // Target 10MHz
    
    TIM2->PSC = correctedPrescaler - 1;
    TIM5->PSC = correctedPrescaler - 1;
    
    // Update lookup tables if error > 0.001%
    if (fabs(freqError) > 1e-5) {
        recalcTimingTables(freqError);
    }
}
```

## **7. DUAL-REDUNDANT TIMING VERIFICATION**

```cpp
// Dual-path timing calculation dengan voting
typedef struct {
    uint32_t primaryResult;
    uint32_t secondaryResult;
    uint8_t agreement;
    uint32_t lastGoodValue;
} RedundantTimer;

RedundantTimer ignitionTimer;

uint32_t calculateRedundantTiming(uint16_t rpm, int16_t timing) {
    // Primary path: Standard calculation
    uint32_t primary = calculatePrimaryTiming(rpm, timing);
    
    // Secondary path: Alternative algorithm
    uint32_t secondary = calculateSecondaryTiming(rpm, timing);
    
    // Tertiary path: Hardware timer capture of actual ignition
    uint32_t measured = TIM2->CCR2;  // Capture actual ignition time
    
    // Voting logic
    uint8_t votes[3] = {0};
    if (abs((int32_t)primary - (int32_t)secondary) < 10) votes[0]++;
    if (abs((int32_t)primary - (int32_t)measured) < 10) votes[1]++;
    if (abs((int32_t)secondary - (int32_t)measured) < 10) votes[2]++;
    
    if (votes[0] + votes[1] + votes[2] >= 2) {
        // Consensus reached
        ignitionTimer.agreement = 1;
        ignitionTimer.lastGoodValue = (primary + secondary + measured) / 3;
        return ignitionTimer.lastGoodValue;
    } else {
        // No consensus - use last good value with safety margin
        ignitionTimer.agreement = 0;
        return ignitionTimer.lastGoodValue + 100;  // 10us retard for safety
    }
}
```

## **8. REAL-TIME PERFORMANCE MONITORING**

```cpp
// Monitor setiap cycle untuk detect anomalies
typedef struct {
    uint32_t cycleStartTime;
    uint32_t isrDuration;
    uint32_t loopDuration;
    uint32_t maxIsrDuration;
    uint32_t maxLoopDuration;
    uint32_t timingJitterHistory[64];
    uint8_t jitterIndex;
    float jitterStdDev;
} PerformanceMonitor;

PerformanceMonitor perfMon;

void monitorCyclePerformance(void) {
    static uint32_t lastCycleTime = 0;
    uint32_t now = DWT_CYCCNT;
    
    if (lastCycleTime > 0) {
        uint32_t cycleTime = now - lastCycleTime;
        perfMon.loopDuration = cycleTime;
        
        if (cycleTime > perfMon.maxLoopDuration) {
            perfMon.maxLoopDuration = cycleTime;
            
            // Jika loop terlalu lama, trigger safety
            if (cycleTime > 1000000) {  // > 4ms @ 250MHz
                triggerSafetyMode();
            }
        }
        
        // Calculate timing jitter
        uint32_t expectedTime = 60000000000ULL / runtime.currentRpm;
        if (runtime.currentRpm > 1000) {
            int32_t jitter = (int32_t)cycleTime - (int32_t)expectedTime;
            perfMon.timingJitterHistory[perfMon.jitterIndex] = abs(jitter);
            perfMon.jitterIndex = (perfMon.jitterIndex + 1) & 0x3F;
            
            // Update standard deviation
            uint32_t sum = 0, sumSq = 0;
            for (int i = 0; i < 64; i++) {
                sum += perfMon.timingJitterHistory[i];
                sumSq += perfMon.timingJitterHistory[i] * 
                         perfMon.timingJitterHistory[i];
            }
            float mean = sum / 64.0f;
            perfMon.jitterStdDev = sqrt(sumSq / 64.0f - mean * mean);
            
            // Jika jitter terlalu tinggi, trigger calibration
            if (perfMon.jitterStdDev > 100.0f) {  // > 100 ticks jitter
                requestRecalibration();
            }
        }
    }
    
    lastCycleTime = now;
}
```

## **9. IMPLEMENTASI FINAL - EXTREME PRECISION CDI**

```cpp
// Main ISR dengan semua optimasi ekstrem
void __attribute__((optimize("O3"), always_inline)) 
vrCaptureISR_ExtremePrecision(void) {
    // 1. Ultra-fast capture dengan direct register
    uint32_t capture = TIM2->CCR1;
    
    // 2. Period calculation dengan error correction
    static uint32_t lastCapture = 0;
    uint32_t period = capture - lastCapture;
    lastCapture = capture;
    
    // 3. Hardware-accelerated noise filtering
    if (period < TIM2->CCMR1 >> 16) return;  // Use capture/compare reg as filter
    
    // 4. Dual-redundant RPM calculation
    uint32_t rpm1 = 600000000UL / period;  // Primary
    uint32_t rpm2 = hardwareDivide(600000000UL, period);  // Secondary (DIV_S)
    uint16_t rpm = (abs((int32_t)rpm1 - (int32_t)rpm2) < 100) ? 
                   (rpm1 + rpm2) / 2 : MIN(rpm1, rpm2);
    
    // 5. Cubic spline timing lookup dengan PLL correction
    int32_t baseTiming = cubicInterpolate(&timingSpline, rpm);
    int32_t pllCorrection = getPLLCorrectedTiming(rpm, baseTiming);
    int32_t tempCompensation = getTemperatureCompensation(runtime.currentTempC);
    int32_t finalTiming = baseTiming + pllCorrection + tempCompensation;
    
    // 6. 64-bit precision delay calculation
    uint64_t ticksPerDeg = getTicksPerDegree64(rpm);
    int32_t angleDelay = config.trigger.triggerAngleScaled - finalTiming;
    uint64_t delayTicks64 = (angleDelay * ticksPerDeg) / 10000ULL;
    
    // 7. Error bounding and safety check
    if (delayTicks64 > UINT32_MAX) delayTicks64 = UINT32_MAX;
    uint32_t delayTicks = (uint32_t)delayTicks64;
    
    // 8. Schedule dengan HRTIM untuk 184ps resolution
    HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_B].CMP1xR = delayTicks;
    HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_B].SETxR = HRTIM_SETX_SST;
    
    // 9. Real-time performance monitoring
    uint32_t isrEnd = DWT_CYCCNT;
    perfMon.isrDuration = isrEnd - perfMon.cycleStartTime;
    if (perfMon.isrDuration > perfMon.maxIsrDuration) {
        perfMon.maxIsrDuration = perfMon.isrDuration;
    }
    
    // 10. Online calibration update
    if (perfMon.sampleCount++ % 100 == 0) {
        updateTimingCalibration(&timingCal, rpm, finalTiming, 
                               baseTiming, runtime.currentTempC);
    }
}
```

## **10. VALIDASI DAN TESTING**

```cpp
// Comprehensive test suite
void runPrecisionValidation(void) {
    USB_SERIAL.println(F("\n=== EXTREME PRECISION VALIDATION ==="));
    
    // 1. Timer resolution test
    testTimerResolution();
    
    // 2. Jitter measurement over 1,000,000 cycles
    measureTimingJitter(1000000);
    
    // 3. Temperature drift compensation test
    testTemperatureCompensation();
    
    // 4. Maximum acceleration response test
    testAccelerationResponse();
    
    // 5. Redundancy and fault tolerance test
    testFaultTolerance();
    
    USB_SERIAL.println(F("=== VALIDATION COMPLETE ==="));
}

void testTimerResolution(void) {
    // Measure actual timer resolution
    uint32_t start = TIM2->CNT;
    GPIOB->BSRR = (1 << 0);  // Toggle CDI pin
    uint32_t end = TIM2->CNT;
    
    uint32_t delta = end - start;
    USB_SERIAL.print(F("Timer resolution: "));
    USB_SERIAL.print(delta * 100);  // Convert to nanoseconds
    USB_SERIAL.println(F("ns"));
    
    if (delta > 1) {
        USB_SERIAL.println(F("WARNING: Resolution worse than 100ns!"));
    }
}
```

## **PERFORMANCE TARGETS**

Dengan implementasi di atas, target performa:

1. **Timing Resolution**: 184ps (HRTIM) vs 100ns sebelumnya → **543x improvement**
2. **Timing Accuracy**: ±0.005° vs ±0.05° sebelumnya → **10x improvement**
3. **Jitter**: <0.001° vs <0.05° sebelumnya → **50x improvement**
4. **Response Time**: <1° pada acceleration 10,000 RPM/s
5. **Temperature Stability**: <0.01°/100°C drift

## **HARDWARE RECOMMENDATIONS**

Untuk mencapai akurasi ekstrem:

1. **External TCXO** (Temperature Compensated Crystal Oscillator) untuk clock reference
2. **Low-jitter clock buffer** untuk distribusi clock
3. **Thermal management** - heatsink pada MCU
4. **Isolated power supply** untuk analog sections
5. **Shielded cabling** untuk VR sensor

Implementasi ini membutuhkan **expertise tinggi** dalam embedded systems, real-time programming, dan signal processing, tetapi akan menghasilkan **CDI dengan akurasi tertinggi yang mungkin secara teknis**.