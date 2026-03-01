# Winding Machine - Visual UI Guide

## Menu Navigation Flow

```
┌─────────────┐
│  MAIN MENU  │
└──────┬──────┘
       │
   ┌───┴─────────────────────────┬──────────┐
   │                             │          │
┌──▼────────┐        ┌───────────▼──┐  ┌───▼──────┐
│  WINDING  │        │    MANUAL     │  │ SETTINGS │
│  (Auto)   │        │   WINDING     │  │          │
└──────┬────┘        └───────────────┘  └────┬─────┘
       │                                       │
   ┌───▼──────────────────┐          ┌────────┴────────┐
   │                      │          │                 │
┌──▼──────────────┐  ┌───▼────────┐ │    ┌────────────▼──┐
│ Coil Config     │  │ Winding    │ │    │ MOTOR PARAMS   │
│ - Wire dia      │  │ Progress   │ │    │ - Gear Ratio   │
│ - Coil width    │  │ - Turns    │ │    │ - mm/rev B     │
│ - Inner radius  │  │ - Layers   │ │    │ - Wind speed   │
│ - Target       │  │ - Progress │ │    │ - Home speed   │
│   (turns/uH)   │  │   bar      │ │    │ - Accel        │
│ - Mode         │  │ - PAUSE    │ │    │ - Backoff      │
└────────────────┘  │ - STOP     │ │    │ - Film thick   │
                    └────────────┘ │    └────────────────┘
                                   │
          ┌────────────────────────┴─────────────────┐
          │                                          │
    ┌─────▼──────┐  ┌──────────┐  ┌──────────┐  ┌──▼──────┐
    │ TOUCH CAL  │  │MOTOR     │  │ UNITS    │  │ SENSORS │
    │            │  │PARAMS    │  │ mH / uH  │  │ ON / OFF│
    └────────────┘  └──────────┘  └──────────┘  └────┬────┘
                                                       │
                                                 ┌─────▼──────┐
                                                 │  DIAM CAL  │
                                                 │  ON / OFF  │
                                                 └────────────┘
```

## Screen Layouts

### 1. Main Screen (320×240px)
```
┌─────────────────────────────────┐
│   COIL WINDING MACHINE v3.2      │  <- Header
├─────────────────────────────────┤
│                                 │
│  ┌──────────┐   ┌──────────┐   │
│  │ WINDING  │   │  MANUAL  │   │
│  │  (AUTO)  │   │  WINDING │   │
│  └──────────┘   └──────────┘   │
│                                 │
│  ┌──────────┐   ┌──────────┐   │
│  │ SETTINGS │   │  HOMING  │   │
│  │          │   │          │   │
│  └──────────┘   └──────────┘   │
│                                 │
│  Wire: 0.50mm | Turns: 100     │  <- Coil Summary
│  Width: 20mm  | Layers: 20     │
│  Inner R: 10mm                 │
│                                 │
└─────────────────────────────────┘
```

### 2. Settings Menu (2×2 Grid)
```
┌─────────────────────────────────┐
│  SETTINGS               [BACK]   │  <- Header
├─────────────────────────────────┤
│                                 │
│  ┌────────────────┐ ┌─────────┐ │
│  │  TOUCH CAL     │ │ MOTOR   │ │  Row 1
│  │   (blue)       │ │ PARAMS  │ │
│  └────────────────┘ │ (orange)│ │
│                     └─────────┘ │
│  ┌────────────────┐ ┌─────────┐ │
│  │ UNITS: mH      │ │SENSORS: │ │  Row 2
│  │ (green toggle) │ │ ON(green)│ │
│  └────────────────┘ │         │ │
│                     └─────────┘ │
│  ┌────────────────┐              │
│  │ DIAM CAL: OFF  │              │  Row 3
│  │ (orange toggle)│              │
│  └────────────────┘              │
│                                 │
└─────────────────────────────────┘
```

### 3. Motor Parameters Screen
```
┌──────────────────────────────────────────────┐
│ MOTOR PARAMS  [🗑]  [SAVE]  [BACK]           │  <- Header
├──────────────────────────────────────────────┤
│                                              │
│  Gear Ratio A    [-] 4.148 [x] [+]          │  <- Row 0
│  mm/rev B        [-] 84.0 [mm] [+]          │  <- Row 1
│  Wind speed      [-] 1600 [s/s] [+]         │  <- Row 2
│  Home speed      [-] 400 [s/s] [+]          │  <- Row 3
│  Accel steps     [-] 200 [stp] [+]          │  <- Row 4
│  Backoff mm      [-] 2.0 [mm] [+]           │  <- Row 5
│  Film thick      [-] 0.05 [mm] [+]          │  <- Row 6
│                                              │
│  Diameter calibration mode: see Settings   │  <- Info
│                                              │
└──────────────────────────────────────────────┘
```

### 4. Winding Progress Screen
```
┌──────────────────────────────────┐
│ WINDING  [PAUSE]  [STOP]         │  <- Header
├──────────────────────────────────┤
│                                  │
│  Turn: 45 / 100                  │  <- Turn counter
│                                  │
│  Layer: 3 / 5                    │  <- Layer counter
│                                  │
│  ████████████░░░░░░░░░░░░░░░░   │  <- Progress bar
│         45%                      │  <- Percentage
│                                  │
│  Status: WINDING                 │  <- Status line
│                                  │
│  ENC A: 589234  ENC B: 8942      │  <- Step counters
│                                  │
└──────────────────────────────────┘
```

### 5. Coil Configuration Flow
```
┌──────────────────────────────────────────┐
│ 1. SELECT MODE                           │
│    • Turn count mode                     │
│    • Inductance mode (air core)          │
│    • Inductance + core mode              │
└──────────────────────────────────────────┘
                  ↓
┌──────────────────────────────────────────┐
│ 2. ENTER PARAMETERS                      │
│    • Wire diameter (mm)                  │
│    • Coil width (mm)                     │
│    • Inner radius (mm)                   │
│    • Target: turns OR inductance         │
│    • Core permeability (if mode 3)       │
└──────────────────────────────────────────┘
                  ↓
┌──────────────────────────────────────────┐
│ 3. CALCULATE & REVIEW                    │
│    • Display calculated inductance       │
│    • Show turn/layer breakdown           │
│    • Confirm or modify                   │
└──────────────────────────────────────────┘
                  ↓
┌──────────────────────────────────────────┐
│ 4. START WINDING                         │
│    HOME A → HOME B → BEGIN               │
└──────────────────────────────────────────┘
```

## Motor Synchronization Pattern (Winding View)

```
Layer 0 (Even):
 Motor A: ═════════════════════  (1 rotation)
 Motor B: ════════════════════   (forward, wire_dia distance)
 
Layer 1 (Odd):
 Motor A: ═════════════════════  (1 rotation)
 Motor B: ════════════════════   (reverse, wire_dia distance)
           ↑ Traverse reverses each layer for neat stacking
 
Layer 2 (Even):
 Motor A: ═════════════════════  (1 rotation)
 Motor B: ════════════════════   (forward again)
```

## Diameter Calibration Effect

```
Without Diam Calib (Fixed traverse):
┌─────────────────────────┐   ← Inner layer (small circumference)
│  Wire spacing 0.50mm    │
├─────────────────────────┤
│  Wire spacing 0.50mm    │   ← Outer layer (large circumference)
│  (spacing too narrow!)  │
└─────────────────────────┘


With Diam Calib (ON):
┌─────────────────────────┐   ← Inner layer (d = 20.1mm)
│  Wire spacing 0.50mm    │      Traverse steps: 100%
├─────────────────────────┤
│    Wire spacing 0.53mm  │   ← Outer layer (d = 20.6mm)
│    (properly adjusted)  │      Traverse steps: 94.5%
└─────────────────────────┘      (compensates for larger circumference)
```

## Hardware Block Diagram

```
┌──────────────────────────────────────────────┐
│         ESP32-2432S028R (CYD)                │
│                                              │
│  ┌────────────────────────────────────────┐  │
│  │  Core: 240MHz dual-core processor      │  │
│  │  RAM: 320KB internal + 4MB PSRAM       │  │
│  │  Storage: NVS (preferences)            │  │
│  └────────────────────────────────────────┘  │
│                                              │
│  Display: 320×240 TFT ILI9341 via SPI       │
│  Touch: XPT2046 capacitive via I2C          │
│                                              │
└─────────────────────────┬────────────────────┘
                          │
        ┌─────────────────┼─────────────────┐
        │                 │                 │
        │          GPIO Pins               │
        │                 │                 │
   ┌────▼─────┐   ┌──────▼──────┐   ┌─────▼────┐
   │ Motor A  │   │  Motor B   │   │ Endstops │
   │ (STEP 16)│   │ (STEP 17)  │   │          │
   │ (DIR 27) │   │ (DIR 22)   │   │  A: 4    │
   │ (EN 26)  │   │ (EN 26)    │   │  B: 34   │
   └────┬─────┘   └──────┬──────┘   └─────┬────┘
        │                │                │
   ┌────▼─────┐   ┌──────▼──────┐        │
   │HR4988 DRV│   │HR4988 DRV  │        │
   │ 1:8 M/S  │   │ 1:8 M/S    │        │
   └────┬─────┘   └──────┬──────┘   ┌────▼────┐
        │                │          │ Optical │
   ┌────▼─────────────────▼──┐      │ Sensors │
   │   Stepper Motors        │      └─────────┘
   │ NEMA-17 Coils (0.4A)    │
   └────┬────────────────────┘
        │
   ┌────▼────────────────────────┐
   │   Mechanical System          │
   │ Motor A: 4.148:1 reduction   │
   │          (55.5mm / 13.39mm)  │
   │ Motor B: Linear traverse     │
   │          84mm per rotation   │
   └─────────────────────────────┘
```

## State Machine Diagram

```
                    WIND_IDLE
                       │
              ┌────────┼────────┐
              │        │        │
              ▼        ▼        ▼
           WINDING  MANUAL_FWD  HOMING
           (AUTO)   & MANUAL_REV & BACKOFF
              │        │        │
              └────────┼────────┘
                       │
                    ┌──▼──┐
                    │ALARM│ (Endstop/Error)
                    └─────┘
                       
              WINDING → PAUSED → WINDING (toggle)
                    PAUSED (windPaused=true)
              Motors stop but state preserves counters
              Resume: windPaused=false, continue timing
              Stop: Reset windPaused=false, goto WIND_IDLE
```

---

**Generated for Winding Machine v3.2 | March 2026**
