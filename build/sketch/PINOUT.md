#line 1 "/Users/wicaksu/Documents/Arduino/cdi-ninja-2stak/PINOUT.md"
# STM32H562RGT6 CDI Racing - Pinout Documentation

## 🎯 Board Information

**MCU:** WeAct Studio STM32H562RGT6  
**Core:** ARM Cortex-M33 @ 250MHz  
**Package:** LQFP64  
**Framework:** Arduino STM32

---

## 📌 Complete Pinout Table

### GPIO Assignments

| Pin  | Port  | Function      | Type         | Description                       | Timer/Peripheral          |
| ---- | ----- | ------------- | ------------ | --------------------------------- | ------------------------- |
| PA0  | GPIOA | VR Input      | Input        | VR sensor signal (MAX9926)        | TIM2_CH1 (Input Capture)  |
| PA1  | GPIOA | Kill Switch   | Input        | Engine kill switch (active LOW)   | GPIO with pull-up         |
| PA2  | GPIOA | Map Switch    | Input        | Map selection button (active LOW) | GPIO with pull-up         |
| PA3  | GPIOA | Quick Shifter | Analog Input | Strain gauge / pressure sensor    | ADC1                      |
| PA8  | GPIOA | SD Detect     | Input        | SD card detect pin                | GPIO with pull-up         |
| PB0  | GPIOB | CDI Output    | Output       | CDI trigger pulse (50-250µs)      | TIM3_CH3 (Output Compare) |
| PB1  | GPIOB | Shift Light   | Output       | Shift light output (12V tolerant) | GPIO Push-Pull            |
| PB2  | GPIOB | Status LED    | Output       | Onboard blue LED (active LOW)     | GPIO Push-Pull            |
| PB4  | GPIOB | Warning       | Output       | Warning buzzer/LED output         | GPIO Push-Pull            |
| PC0  | GPIOC | Head Temp     | Analog Input | Cylinder head temperature sensor  | ADC1_IN10                 |
| PC1  | GPIOC | Battery       | Analog Input | Battery voltage (via divider)     | ADC1_IN11                 |
| PC2  | GPIOC | Charging      | Analog Input | Charging voltage monitor          | ADC1_IN12                 |
| PC8  | GPIOC | SD D0         | SDMMC        | SD card data line 0               | SDMMC1_D0                 |
| PC9  | GPIOC | SD D1         | SDMMC        | SD card data line 1               | SDMMC1_D1                 |
| PC10 | GPIOC | SD D2         | SDMMC        | SD card data line 2               | SDMMC1_D2                 |
| PC11 | GPIOC | SD D3         | SDMMC        | SD card data line 3               | SDMMC1_D3                 |
| PC12 | GPIOC | SD CLK        | SDMMC        | SD card clock                     | SDMMC1_CK                 |
| PC13 | GPIOC | User Button   | Input        | Onboard button (active LOW)       | GPIO (backup domain)      |
| PD2  | GPIOD | SD CMD        | SDMMC        | SD card command                   | SDMMC1_CMD                |

---

## 🔌 Pinout Diagram (LQFP64 - Top View)

```
                        STM32H562RGT6
                    ┌─────────────────┐
                    │                 │
         VBAT    1  │●                │ 64  PB15
         PC13    2  │● (User Button)  │ 63  PB14
         PC14    3  │●                │ 62  PB13
         PC15    4  │●                │ 61  PB12
         VSS     5  │●                │ 60  PB11
         VDD     6  │●                │ 59  VCAP
         PH0     7  │●                │ 58  PB10
         PH1     8  │●                │ 57  PB9
         NRST    9  │●                │ 56  PB8
        VSSA    10  │●                │ 55  BOOT0
        VDDA    11  │●                │ 54  PB7
  [VR]  PA0    12  │● TIM2_CH1       │ 53  PB6
  [KIL] PA1    13  │●                │ 52  PB5
  [MAP] PA2    14  │●                │ 51  PB4  [WARN]●
  [QS]  PA3    15  │● ADC1           │ 50  PB3
         VSS    16  │●                │ 49  PD7
         VDD    17  │●                │ 48  PD6
         PA4    18  │●                │ 47  PD5
         PA5    19  │●                │ 46  PD4
         PA6    20  │●                │ 45  PD3
         PA7    21  │●                │ 44  PD2  [SD_CMD]●
 [TMP]  PC0    22  │● ADC1_IN10      │ 43  PD1
 [BAT]  PC1    23  │● ADC1_IN11      │ 42  PD0
 [CHG]  PC2    24  │● ADC1_IN12      │ 41  PC12 [SD_CLK]●
         PC3    25  │●                │ 40  PC11 [SD_D3]●
         VSS    26  │●                │ 39  PC10 [SD_D2]●
         VDD    27  │●                │ 38  PC9  [SD_D1]●
         PA8    28  │● (SD_DETECT)    │ 37  PC8  [SD_D0]●
         PA9    29  │●                │ 36  PC7
        PA10    30  │●                │ 35  PC6
        PA11    31  │●                │ 34  PB15
        PA12    32  │●                │ 33  PB14
                    │   [USB C Port]  │
                    └─────────────────┘

[VR]   = VR Sensor Input
[KIL]  = Kill Switch
[MAP]  = Map Switch
[QS]   = Quick Shifter
[TMP]  = Head Temperature
[BAT]  = Battery Voltage
[CHG]  = Charging Voltage
[CDI]  = CDI Output
[SFT]  = Shift Light
[LED]  = Status LED
[WARN] = Warning Output
```

---

## 📋 Functional Pin Groups

### 🔥 Engine Control (Critical Timing)

| Pin | Function   | Direction | Voltage | Description                                   |
| --- | ---------- | --------- | ------- | --------------------------------------------- |
| PA0 | VR Input   | Input     | 5V TTL  | VR sensor via MAX9926, hardware input capture |
| PB0 | CDI Output | Output    | 5V/12V  | CDI trigger pulse, 50-250µs configurable      |

**Notes:**

- PA0: Connected to TIM2_CH1 for hardware timestamp (0.1µs resolution)
- PB0: Fast GPIO with direct register access for <50ns switching
- Both pins are timing-critical - NO capacitive load!

---

### 🎮 User Interface Inputs

| Pin  | Function      | Direction | Voltage | Pull     | Description                          |
| ---- | ------------- | --------- | ------- | -------- | ------------------------------------ |
| PA1  | Kill Switch   | Input     | 3.3V    | Pull-up  | Active LOW, immediate engine cut     |
| PA2  | Map Switch    | Input     | 3.3V    | Pull-up  | Active LOW, cycle through 6 maps     |
| PC13 | User Button   | Input     | 3.3V    | External | Onboard button, secondary map switch |
| PA3  | Quick Shifter | Analog    | 3.3V    | -        | Strain gauge ADC input (0-4095)      |

**Connection Examples:**

```
Kill Switch:
  GND ──┬── [Switch] ─── PA1
        └── Normally Open

Map Switch:
  GND ──┬── [Button] ─── PA2
        └── Momentary push

Quick Shifter:
  Sensor+ ─── PA3
  Sensor- ─── GND
  (Voltage output 0-3.3V)
```

---

### 📊 Analog Inputs (ADC1)

| Pin | Channel   | Function      | Range  | Voltage Divider   | Description                     |
| --- | --------- | ------------- | ------ | ----------------- | ------------------------------- |
| PC0 | ADC1_IN10 | Head Temp     | 0-3.3V | Direct            | NTC thermistor or K-type sensor |
| PC1 | ADC1_IN11 | Battery       | 0-20V  | R1=10kΩ, R2=2.2kΩ | Battery voltage monitor         |
| PC2 | ADC1_IN12 | Charging      | 0-20V  | R1=10kΩ, R2=2.2kΩ | Alternator/charging voltage     |
| PA3 | ADC1      | Quick Shifter | 0-3.3V | Direct            | Strain gauge sensor             |

**ADC Configuration:**

- Resolution: 12-bit (0-4095)
- Reference: VDDA (3.3V)
- Sample rate: Every 10ms
- Calibration: Configurable offset and scale

**Voltage Divider Formula:**

```
Vout = Vin × (R2 / (R1 + R2))
For battery (0-20V → 0-3.3V):
  R1 = 10kΩ, R2 = 2.2kΩ
  Vout = 20V × (2.2 / 12.2) = 3.61V (slightly over, use 18V max)
```

**Temperature Sensor Wiring:**

```
Option 1 - NTC Thermistor:
  3.3V ─── [10kΩ] ─┬─── PC0
                   └─── [NTC] ─── GND

Option 2 - K-Type with MAX6675:
  MAX6675_VCC ─── 3.3V
  MAX6675_GND ─── GND
  MAX6675_SO  ─── PC0 (analog voltage out)
```

---

### 💡 Output Signals

| Pin | Function    | Type      | Voltage | Current | Description                     |
| --- | ----------- | --------- | ------- | ------- | ------------------------------- |
| PB0 | CDI Trigger | Push-Pull | 5V      | 50mA    | Direct to CDI module trigger    |
| PB1 | Shift Light | Push-Pull | 3.3V    | 20mA    | Via transistor to LED/12V relay |
| PB2 | Status LED  | Push-Pull | 3.3V    | 10mA    | Onboard blue LED (active LOW)   |
| PB4 | Warning     | Push-Pull | 3.3V    | 20mA    | Via transistor to buzzer/LED    |

**Output Driver Circuits:**

```
CDI Output (PB0):
  PB0 ───[330Ω]─── CDI Trigger Input
  (Direct connection, CDI has internal pull-down)

Shift Light (PB1):
  PB1 ───[1kΩ]───┬─── NPN Base (2N2222)
                 │
             [10kΩ] to GND
                 │
         NPN Collector ─── [Shift Light +] 12V
         NPN Emitter ──────── GND

Warning Output (PB4):
  PB4 ───[1kΩ]───┬─── NPN Base (2N2222)
                 │
             [10kΩ] to GND
                 │
         NPN Collector ─── [Buzzer +] 12V
         NPN Emitter ──────── GND

Status LED (PB2):
  Built-in on WeAct board
  Active LOW (3.3V = OFF, 0V = ON)
```

---

### 💾 SD Card Interface (SDMMC1)

| Pin  | Signal | Function    | Speed       |
| ---- | ------ | ----------- | ----------- |
| PC8  | D0     | Data line 0 | Up to 50MHz |
| PC9  | D1     | Data line 1 | Up to 50MHz |
| PC10 | D2     | Data line 2 | Up to 50MHz |
| PC11 | D3     | Data line 3 | Up to 50MHz |
| PC12 | CLK    | Clock       | Up to 50MHz |
| PD2  | CMD    | Command     | Up to 50MHz |
| PA8  | CD     | Card detect | GPIO        |

**SD Card Wiring:**

```
SD Card Slot:
  Pin 1 (DAT2/D2) ─── PC10
  Pin 2 (DAT3/D3) ─── PC11
  Pin 3 (CMD)     ─── PD2
  Pin 4 (VDD)     ─── 3.3V
  Pin 5 (CLK)     ─── PC12
  Pin 6 (VSS)     ─── GND
  Pin 7 (DAT0/D0) ─── PC8
  Pin 8 (DAT1/D1) ─── PC9
  Card Detect     ─── PA8 (via switch to GND)

Pull-ups required on CMD and data lines (10kΩ to 3.3V)
```

**Notes:**

- 4-bit wide bus mode for fast transfer
- No CS pin needed (SDMMC protocol)
- Built-in CRC error detection
- Hot-swap detection via PA8

---

## 🔧 Hardware Requirements

### Power Supply

| Rail     | Voltage  | Current | Description               |
| -------- | -------- | ------- | ------------------------- |
| VBAT     | 3.0-3.6V | 5µA     | Backup domain (RTC, PC13) |
| VDD      | 3.0-3.6V | 250mA   | Main digital supply       |
| VDDA     | 3.0-3.6V | 50mA    | Analog supply (ADC)       |
| VSS/VSSA | 0V       | -       | Ground                    |

**Decoupling Capacitors:**

- VDD: 100nF ceramic + 10µF tantalum per pair
- VDDA: 100nF ceramic + 1µF tantalum
- VBAT: 100nF ceramic

---

### External Components

**Required:**

1. **VR Sensor Conditioner** - MAX9926 or similar

   - Input: VR sensor AC signal
   - Output: 5V TTL to PA0
   - Adjustable threshold and hysteresis

2. **Voltage Dividers** - For battery/charging monitoring

   - Resistor tolerance: 1% or better
   - Power rating: 0.25W minimum

3. **Pull-up Resistors** - SD card data/cmd lines

   - Value: 10kΩ
   - Location: Close to SD socket

4. **Output Drivers** - For shift light and warning
   - NPN transistor: 2N2222 or BC547
   - Base resistor: 1kΩ
   - Pull-down: 10kΩ

**Optional:**

1. **TVS Diodes** - For input protection

   - VR input: 5.6V bidirectional
   - ADC inputs: 3.6V bidirectional
   - Kill/Map switches: 5.6V bidirectional

2. **LC Filter** - For power supply noise
   - Inductor: 10µH
   - Capacitor: 100µF

---

## 📐 PCB Layout Guidelines

### Critical Traces

**High-Speed Signals (Keep Short!):**

1. PA0 (VR Input): <50mm, avoid crossing digital traces
2. PB0 (CDI Output): <30mm, thick trace (0.5mm min)
3. SD card bus: Match length ±10mm, impedance control

**Analog Signals (Quiet Routing):**

1. PC0-PC2, PA3 (ADC): Away from switching signals
2. Star ground for VSSA
3. Guard traces around ADC inputs

**Power Distribution:**

1. VDD/VSS: Wide traces (1mm+), star topology
2. Decoupling caps: <5mm from pins
3. Separate analog/digital grounds, join at one point

### Grounding

```
Main Ground Strategy:

Digital GND ─┐
              ├──── [Single Point] ──── Battery GND
Analog GND ──┘

Keep digital and analog grounds separate until the join point!
```

---

## 🔌 Connector Pinouts

### Main Connector (Recommended: 20-pin header)

| Pin   | Signal    | Type   | Description                   |
| ----- | --------- | ------ | ----------------------------- |
| 1     | VDD       | Power  | +3.3V input (250mA)           |
| 2     | GND       | Ground | Ground reference              |
| 3     | VR_IN     | Input  | VR sensor (from MAX9926)      |
| 4     | CDI_OUT   | Output | CDI trigger pulse             |
| 5     | KILL_SW   | Input  | Kill switch (to GND)          |
| 6     | MAP_SW    | Input  | Map switch (to GND)           |
| 7     | QS_IN     | Analog | Quick shifter sensor          |
| 8     | SHIFT_OUT | Output | Shift light (transistor base) |
| 9     | WARN_OUT  | Output | Warning (transistor base)     |
| 10    | TEMP_IN   | Analog | Head temperature              |
| 11    | BATT_IN   | Analog | Battery voltage (divided)     |
| 12    | CHG_IN    | Analog | Charging voltage (divided)    |
| 13    | GND       | Ground | Analog ground                 |
| 14-20 | -         | -      | Reserved for expansion        |

### Programming Interface (SWD)

| Pin | Signal | Description           |
| --- | ------ | --------------------- |
| 1   | SWDIO  | Serial Wire Debug I/O |
| 2   | SWCLK  | Serial Wire Clock     |
| 3   | GND    | Ground                |
| 4   | 3.3V   | Power (optional)      |
| 5   | NRST   | Reset (optional)      |

---

## 🧪 Testing & Validation

### Pin Testing Checklist

**Power-On Tests:**

- [ ] Measure VDD = 3.3V ±0.1V
- [ ] Measure VDDA = 3.3V ±0.1V
- [ ] Check all GND pins = 0V
- [ ] Verify no shorts between VDD/GND

**Input Tests:**

- [ ] Kill switch: 3.3V (open), 0V (closed)
- [ ] Map switch: 3.3V (open), 0V (closed)
- [ ] VR input: Verify signal from MAX9926
- [ ] Quick shifter: 0-3.3V range

**Output Tests:**

- [ ] CDI output: 0V idle, 5V pulse when triggered
- [ ] Shift light: Verify transistor switching
- [ ] Warning: Verify transistor switching
- [ ] Status LED: Blinks on power-up

**ADC Tests:**

- [ ] Battery: Measure with known voltage
- [ ] Charging: Measure with known voltage
- [ ] Temperature: Verify reading vs actual temp
- [ ] Quick shifter: Verify ADC reading (GET QSADC)

**SD Card Tests:**

- [ ] Insert card: PA8 should read LOW
- [ ] Remove card: PA8 should read HIGH
- [ ] Verify file creation
- [ ] Verify read/write speed

---

## 📱 USB Connection

**Built-in USB-C Port:**

- USB 2.0 Full Speed (12 Mbps)
- Virtual COM Port (CDC)
- No external USB circuit needed
- Automatic enumeration

**Commands:**

```bash
# Connect via Serial Terminal
Baud: 115200
Data: 8 bits
Parity: None
Stop: 1 bit

# Test connection
GET RPM
GET TIMING
STATUS
```

---

## 🎨 LED Status Patterns

**Status LED (PB2) - Built-in Blue LED:**

| Pattern      | Meaning                     | Blink Rate          |
| ------------ | --------------------------- | ------------------- |
| Slow blink   | Idle (no engine)            | 500ms on/off        |
| Fast blink   | Running OK                  | 100ms on/off        |
| Double blink | Warning (overheat/low batt) | 100-100-100-100-600 |
| Triple blink | Error (SD fail/default map) | 100×6-600           |
| N blinks     | Map number (1-6)            | 150ms per blink     |

---

## 🔐 Safety Features

**Hardware Protection:**

1. Kill switch: Hardware interrupt, immediate cut
2. Overvoltage: TVS diodes on inputs
3. Reverse polarity: Schottky diode on VDD
4. ESD protection: On all exposed pins

**Software Protection:**

1. Watchdog timer: 4-second timeout
2. Blind window: EMI rejection after ignition
3. Noise filter: Reject short pulses
4. Cold start: Wait for 5 stable triggers
5. Rev limiter: 4-stage progressive limiting

---

## 📚 Quick Reference

### Most Used Pins (Top 5)

1. **PA0** - VR Input (most critical!)
2. **PB0** - CDI Output (timing-critical!)
3. **PA1** - Kill Switch (safety!)
4. **PC1** - Battery Monitor (essential!)
5. **PB1** - Shift Light (user feedback!)

### Pin Voltage Levels

| Pin Type       | Logic LOW | Logic HIGH | Absolute Max |
| -------------- | --------- | ---------- | ------------ |
| Digital Input  | <0.8V     | >2.0V      | 5.5V         |
| Digital Output | <0.4V     | >2.4V      | -            |
| Analog Input   | 0V        | 3.3V       | 3.6V         |
| 5V Tolerant    | 0V        | 5V         | 5.5V         |

### Current Limits

| Pin       | Source | Sink  | Notes                 |
| --------- | ------ | ----- | --------------------- |
| GPIO      | 8mA    | 8mA   | Typical               |
| GPIO Max  | 20mA   | 20mA  | Absolute maximum      |
| Total I/O | -      | 120mA | All pins combined     |
| VDD Max   | -      | 250mA | MCU total consumption |

---

## 🛠️ Troubleshooting

### No Communication

- Check USB cable (data pins connected)
- Verify driver installation (STM32 Virtual COM)
- Check baud rate: 115200

### VR Signal Not Detected

- Check MAX9926 wiring
- Verify 5V power to MAX9926
- Check PA0 connection
- Measure VR sensor resistance (100-1000Ω typical)

### CDI Not Firing

- Check PB0 connection to CDI
- Measure voltage: Should pulse 5V
- Verify CDI ground connection
- Check kill switch (must be open/HIGH)

### SD Card Issues

- Verify 3.3V power to card
- Check pull-up resistors (10kΩ on CMD, D0-D3)
- Format card as FAT32
- Use quality card (Class 10+)

### ADC Reading Wrong

- Check voltage divider values
- Verify 3.3V VDDA
- Calibrate using SET CAL\_\* commands
- Check for noise on analog inputs

---

## 📄 Revision History

| Version | Date     | Changes                      |
| ------- | -------- | ---------------------------- |
| 1.0     | Jan 2026 | Initial pinout documentation |

---

## 📞 Support

**Documentation:** README.md in SD card  
**USB Commands:** Type "HELP" via serial  
**Pin Test:** Type "STATUS" for diagnostic info

---

**End of Pinout Documentation** 🎯
