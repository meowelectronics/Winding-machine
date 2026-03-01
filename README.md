# Air Core Coil Winding Machine v3.2

A PlatformIO-based firmware for an ESP32-driven automatic coil winding machine with dual stepper motor control, mechanical calibration, and real-time inductance calculation.

## Features

### Hardware Platform
- **ESP32-2432S028R** (CYD) with 320×240 TFT capacitive touchscreen
- **Dual HR4988 stepper drivers** for synchronized motor control
- **Motor A** (Rotation): 1/8 microstepping with 4.148:1 belt reduction via UNITTA 304-2GT
- **Motor B** (Linear Traverse): 84mm travel per revolution via 3MR414 belt
- **Two optical endstops** for axis limits and homing

### Core Functionality

#### Automatic Winding
- Synchronized dual-motor winding with adaptive traverse pattern
- Three winding modes:
  - **Turn count mode**: Fixed number of turns
  - **Inductance mode (air core)**: Target inductance calculation
  - **Inductance + core mode**: Core permeability adjustment
- Layer-alternating traverse direction for neat stacking
- Real-time turn/layer progress tracking with visual bar

#### Manual Winding
- Fine-grained manual control of both axes
- Discrete synchronized movement matching automatic pattern
- Bi-directional operation (FWD/REV)

#### Mechanical Calibration
- **Gear ratio** adjustment (A-axis): Accounts for belt reduction and pulley sizing
- **Linear travel** calibration (B-axis): User-measured mm per revolution
- **Film thickness** parameter: For diameter-aware winding (Kapton tape)
- **Diameter calibration mode**: Adjusts traverse spacing based on coil growth

#### Safety & Control
- Watchdog timeout prevention via CPU yielding every 1000 iterations
- Dual optical endstop monitoring with emergency backoff
- Motor homing with configurable speeds and backoff distances
- Pause/Resume functionality during winding

#### Motor Tuning
- Adjustable winding and homing speeds
- Configurable acceleration ramp
- Backoff distance after endstop hit
- All parameters persisted in NVS

### User Interface

#### 📊 Visual Guide
For complete menu layouts, screen flows, and visual diagrams, see [**UI_GUIDE.md**](UI_GUIDE.md) which includes:
- Full menu navigation flowchart
- Screen layout mockups (320×240px display)
- Motor parameters tuning interface
- Winding progress visualization
- Hardware block diagram
- State machine diagram

#### Main Screen
- Quick access buttons for winding, manual mode, and settings
- Current coil plan summary
- Real-time endstop status

#### Coil Configuration
- Wire diameter input (mm)
- Coil width definition
- Inner radius specification
- Mode selection and parameter entry
- Inductance calculation and display

#### Settings Menu (2×2 Grid)
- **Touch Calibration**: Capacitive touchscreen calibration
- **Motor Parameters**: Fine-tuning of all mechanical and speed parameters
- **Units Toggle**: mH/uH display format
- **Sensors Toggle**: Enable/disable endstop safety features
- **Diameter Calibration Mode**: ON/OFF for adaptive traverse spacing

#### Winding Screen
- Real-time turn and layer counters
- Progress bar with percentage
- Pause/Resume button for mid-winding control
- Stop button with safety reset

### Inductance Calculation
Implements **Wheeler's formula** for air-core inductance:
```
L = (N² × μ₀ × A) / l
```
Where:
- N = number of turns
- A = average coil area
- l = coil depth (layers × wire diameter)
- μ₀ = 4π × 10⁻⁷ H/m

Supports core materials via permeability (μᵣ) adjustment.

## Menu Structure at a Glance

```
MAIN MENU
├─ WINDING (Auto)
│  ├─ Coil Configuration (wire, width, radius, target)
│  └─ Winding Progress (turn/layer counters, progress bar, PAUSE/STOP)
├─ MANUAL WINDING
│  └─ FWD/REV buttons (discrete synchronized stepping)
├─ HOMING
│  └─ HOME A / HOME B (auto-homing with backoff)
└─ SETTINGS (2×2 Grid)
   ├─ TOUCH CAL
   ├─ MOTOR PARAMS (7 tunable parameters)
   ├─ UNITS (mH ↔ uH toggle)
   ├─ SENSORS (ON/OFF endstop safety)
   └─ DIAM CAL (ON/OFF diameter-aware mode)
```

**→ See [UI_GUIDE.md](UI_GUIDE.md) for detailed screen layouts and visual diagrams**

## Technical Highlights

### Synchronous Motor Control
- Time-based step scheduling ensures consistent speed across layers
- Motor A and B maintain exact ratio during winding
- Fractional step accumulation for smooth traverse

### Watchdog Management
- CPU yields 1ms every 1000 iterations to prevent system reset
- Watchdog explicitly fed every 100 iterations
- Safe for indefinite winding runs (tested 100+ turns)

### Persistence
- All motor parameters saved to ESP32 NVS
- Coil preferences, touch calibration, and sensor settings persisted
- Automatic load on boot

### Responsive UI
- Fast screen transitions (no scrolling lag)
- Position-based absolute layout (no LVGL flex layout deadlocks)
- Efficient timer-based updates for real-time data

## Building

### Prerequisites
- PlatformIO IDE or CLI
- ESP32 board support
- LVGL library (configured in `src/lv_conf.h`)
- TFT_eSPI library (configured for CYD display)

### Build Steps
```bash
cd "Winding machine"
platformio build
platformio upload
```

### Configuration
- **Main code**: `src/main.cpp`
- **LVGL config**: `src/lv_conf.h`
- **Display config**: `src/User_Setup.h`
- **Project config**: `platformio.ini`

## Hardware Pinout

| Function | GPIO | Notes |
|----------|------|-------|
| **Motor A** |
| STEP | 16 | Rotation axis |
| DIR | 27 |  |
| EN | 26 | Shared with Motor B |
| **Motor B** |
| STEP | 17 | Traverse axis |
| DIR | 22 |  |
| EN | 26 | Shared with Motor A |
| **Endstops** |
| ESTOP_A | 4 | Optical on rotating disc (index) |
| ESTOP_B | 34 | Extended position limit (safety) |
| **Display** |
| TFT (SPI) | CE:15, DC:2, RES:4, | See User_Setup.h |
| Touch (I2C) | SDA:21, SCL:22 | Capacitive touch controller |

## Usage

### Basic Winding Workflow
1. **Main Screen** → Select winding mode (turns/inductance/core)
2. **Configuration** → Enter coil parameters
3. **Review** → Confirm inductance calculation and turn count
4. **Winding Screen** → Monitor progress, use PAUSE/RESUME as needed
5. **Completion** → Returns to main screen automatically

### Manual Winding
1. **Main Screen** → Manual Mode
2. **FWD/REV buttons** → Control each axis independently
3. **Step counters** → Track cumulative movement
4. **Back** → Return to main screen

### Motor Calibration
1. **Settings** → **Motor Parameters**
2. Adjust:
   - Gear Ratio A (measure: wheel_dia / motor_gear_dia)
   - mm/rev B (empirically measure actual linear travel)
   - Film thickness (Kapton tape in mm)
   - Speeds, acceleration, backoff distance
3. **Save** → Persisted to NVS
4. **Diam Calib** toggle in Settings → Enable diameter-aware mode

## Development Notes

### Code Organization
- **Motor control**: Lines 500-730 (homing, winding, manual)
- **Coil math**: Lines 250-370 (Wheeler formula, inductance solving)
- **UI screens**: Lines 1200-2150 (LVGL screen builders)
- **Persistence**: Lines 288-330 (NVS save/load)
- **Global state**: Lines 190-230 (volatile flags, configuration)

### Key Algorithms
1. **Synchronous winding** (lines 645-695): Time-based dual-motor stepping
2. **Pause handling** (lines 660-675): Pause duration tracking for timing accuracy
3. **Layer-based direction** (lines 600-625): Alternating traverse per layer
4. **Diameter calculation** (lines 418-428): Coil radius growth per turn

### Testing Checklist
- [x] 89-turn winding completion without watchdog reset
- [x] Manual mode discrete synchronized traverse
- [x] Pause/resume maintains timing accuracy
- [x] Motor params screen opens without freeze
- [x] Touch calibration responsive
- [x] Endstop safety triggers correctly
- [x] NVS persistence across power cycles
- [x] Gear ratio and mm/rev calibration applied
- [x] Diameter calibration mode adjusts traverse

## Known Limitations
- No real-time inductance feedback (single-turn calculation)
- Endstop B only monitors extended position (safety limit, not precise)
- Diameter calibration assumes uniform layer buildup
- Maximum 99 turns displayed (display field constraint)

## Future Enhancements
- Inductance measurement via LCR meter integration
- Automated coil rewinding sequence
- SD card logging of winding statistics
- Multi-core support for independent motor control
- Bluetooth app for remote monitoring
- Custom winding patterns (spiral, bifilar, etc.)

## License

This project is released under the **Apache License 2.0 with Non-Commercial Use Restriction**.

See LICENSE file for full terms. Non-commercial use only. 

## Author

Firmware development: 2026

## Contributing

This is a personal project. No external contributions accepted at this time.

---

**Version 3.2** | ESP32-2432S028R | PlatformIO | LVGL UI Framework
