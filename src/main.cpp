/*
 * coil_winder.ino — Air Core Coil Winding Machine  v3.2
 * Target:  ESP32-2432S028R (CYD — Cheap Yellow Display)
 *
 * Motors:   Vexta P0213-9212PE  0.9deg/step, 2.8A/phase
 * Drivers:  HR4988 (A4988 compatible) Step/Dir, 8x microsteps -> 3200 steps/rev
 * Endstops: NPN optical, 4-wire. 4.7k pull-up to 3.3V on signal wire.
 *
 * GPIO MAP (conflict-free on CYD):
 *   STEP_A=26  DIR_A=27  EN_A=16 (shared enable for both drivers)
 *   STEP_B=17  DIR_B=22
 *   ENDSTOP_A=4 (optical gate on rotating disc, detects position from any direction)
 *   ENDSTOP_B_MIN=34 (single endstop at extended position)
 *
 * CALIBRATE BEFORE USE:
 *   MM_PER_REV_B   — measure your lead-screw pitch in mm
 *   cfg.speed_wind  — tune for your wire tension
 */

// ============================================================================
// INCLUDES
// ============================================================================
#include <Arduino.h>
#include <SPI.h>
#include <Preferences.h>
#include <TFT_eSPI.h>
#include <XPT2046_Touchscreen.h>
#include <lvgl.h>
#include <esp_task_wdt.h>

// ============================================================================
// HARDWARE PINS
// ============================================================================
#define XPT2046_IRQ   36
#define XPT2046_MOSI  32  // Zdieľané s TFT
#define XPT2046_MISO  39  // Zdieľané s TFT
#define XPT2046_CLK   25  // Zdieľané s TFT
#define XPT2046_CS    33  // Jedinečný pre dotyk
#define TFT_BL        21

#define STEP_A  16
#define DIR_A   27
#define EN_A    26

#define STEP_B  17
#define DIR_B   22

#define ENDSTOP_A    4   // Optical gate on winding axis
#define ENDSTOP_B_MIN  34  // Single endstop for traverse axis (extended/anticlockwise position)

// ============================================================================
// MOTION CONSTANTS
// ============================================================================
#define FULL_STEPS_PER_REV  400
#define MICROSTEPS          8

// ── Motor config — all tunable at runtime via Settings screen ──────────────
struct MotorConfig {
    int   steps_rev_a  = FULL_STEPS_PER_REV * MICROSTEPS; // 3200
    int   steps_rev_b  = FULL_STEPS_PER_REV * MICROSTEPS; // 3200
    float gear_ratio_a = 4.148f;   // 55.5mm wheel / 13.39mm motor gear (UNITTA 304-2GT belt)
    float mm_per_rev_b = 84.0f;    // belt-driven linear axis: 84mm per motor rotation (3MR414)
    int   speed_wind   = 1600;     // steps/sec winding
    int   speed_home   = 400;      // steps/sec homing
    int   speed_backoff= 400;      // steps/sec back-off after endstop
    int   accel_steps  = 200;      // ramp-up steps (0=no ramp)
    float back_off_mm  = 2.0f;     // mm to back off after endstop
    bool  diam_calib   = false;    // Diameter calibration mode: adjust traverse based on coil diameter
    float film_thick_mm = 0.05f;   // Kapton film thickness (mm) for diameter calculation
};
static MotorConfig cfg;

// Derived — recalculated whenever cfg changes
static float cfg_steps_mm_b = (float)(FULL_STEPS_PER_REV * MICROSTEPS) / 84.0f;
static float cfg_steps_rev_a_eff = (float)(FULL_STEPS_PER_REV * MICROSTEPS) * 4.148f;  // effective steps with gear ratio

static void recalc_cfg() {
    cfg_steps_mm_b = (float)cfg.steps_rev_b / cfg.mm_per_rev_b;
    cfg_steps_rev_a_eff = (float)cfg.steps_rev_a * cfg.gear_ratio_a;  // Apply gear ratio to A-axis
}

#define STEP_PULSE_US       2

// ============================================================================
// SCREEN
// ============================================================================
// physical panel dimensions (native orientation)
#define PHYS_W    320
#define PHYS_H    240

// choose the LVGL rotation you need for your panel mounting:
// 0 = no rotation (native), 90/180/270 are clockwise in degrees.
// changing this value automatically swaps SCREEN_W/H below.
#define LV_ROT    LV_DISPLAY_ROTATION_0

// logical screen size used by the UI after applying rotation
#if LV_ROT == LV_DISPLAY_ROTATION_0 || LV_ROT == LV_DISPLAY_ROTATION_180
  #define SCREEN_W PHYS_W
  #define SCREEN_H PHYS_H
#else
  #define SCREEN_W PHYS_H
  #define SCREEN_H PHYS_W
#endif

#define TOUCH_X_MIN_DEFAULT  200
#define TOUCH_X_MAX_DEFAULT 3800
#define TOUCH_Y_MIN_DEFAULT  200
#define TOUCH_Y_MAX_DEFAULT 3800

static int touch_x_min = TOUCH_X_MIN_DEFAULT;
static int touch_x_max = TOUCH_X_MAX_DEFAULT;
static int touch_y_min = TOUCH_Y_MIN_DEFAULT;
static int touch_y_max = TOUCH_Y_MAX_DEFAULT;

// ============================================================================
// STRUCTS  — must be declared before any function that uses them
// ============================================================================

struct CoilParams {
    float wire_dia_mm   = 0.50f;
    float coil_width_mm = 20.0f;
    float r_inner_mm    = 10.0f;
    int   mode          = 0;
    int   target_turns  = 100;
    float target_uh     = 100.0f;
    float core_ur       = 1.0f;
};

struct CoilPlan {
    int   turns_per_layer  = 0;
    int   layers           = 0;
    int   total_turns      = 0;
    float achieved_uh      = 0.0f;
    float trav_steps_turn  = 0.0f;
    int   trav_speed       = 10;
    bool  valid            = false;
};

struct ParamRow {
    lv_obj_t* val_lbl;  // Label objekt pre LVGL
    float*    f_ptr;    // Pointer na float hodnotu
    int32_t*  i_ptr;    // Pointer na int hodnotu
    float     mn;       // Minimum
    float     mx;       // Maximum
    float     step;     // Krok zmeny
    int       dec;      // Počet desatinných miest
    const char* unit;   // Jednotka (mm, T, atď.)

    // Konštruktor pre jednoduchú inicializáciu
    ParamRow(lv_obj_t* l, float* f, int32_t* i, float mi, float ma, float s, int d, const char* u)
        : val_lbl(l), f_ptr(f), i_ptr(i), mn(mi), mx(ma), step(s), dec(d), unit(u) {}

    // Prázdny konštruktor
    ParamRow() : val_lbl(nullptr), f_ptr(nullptr), i_ptr(nullptr), mn(0), mx(0), step(0), dec(0), unit("") {}
};

// ============================================================================
// GLOBAL INSTANCES
// ============================================================================
SPIClass            touchSPI(VSPI);
XPT2046_Touchscreen touch(XPT2046_CS, XPT2046_IRQ);
TFT_eSPI            tft;
Preferences         prefs;
// 1. Najprv definícia počtu bodov
static const int CAL_PTS = 3;

static uint8_t draw_buf_1[SCREEN_W * 24 * 2]; // SCREEN_W * 24 * 2 (two 24‑line partial buffers)
static uint8_t draw_buf_2[SCREEN_W * 24 * 2];


CoilParams p;
CoilPlan   plan;

//static ParamRow prows[6];
static bool display_in_mh = false;
static bool end_sensors_enabled = true;

static ParamRow prows[6] = {
    ParamRow(nullptr, &p.wire_dia_mm,   nullptr, 0.05f, 2.00f, 0.01f, 2, "mm"),
    // width step will be updated to match wire diameter during runtime
    ParamRow(nullptr, &p.coil_width_mm, nullptr, 1.0f,  100.f, 0.50f,  1, "mm"),
    ParamRow(nullptr, &p.r_inner_mm,    nullptr, 1.0f,  100.f, 0.5f,  1, "mm"),
    ParamRow(nullptr, nullptr, (int32_t*)&p.target_turns, 1.0f, 9999.0f, 1.0f, 0, "T"),
    ParamRow(nullptr, &p.target_uh,     nullptr, 1.0f, 99999.f, 1.0f, 0, "uH"),
    ParamRow(nullptr, &p.core_ur,       nullptr, 1.0f, 9999.0f, 1.0f, 0, "ur")
};
// ============================================================================
// ENCODER STATE
// ============================================================================
// Step position tracking (no encoder feedback needed)
volatile int32_t step_count_a = 0;  // winding axis cumulative step count
volatile int32_t step_count_b = 0;  // traverse axis cumulative step count

// ============================================================================
// ENDSTOP STATE
// ============================================================================
volatile bool estop_a = false;      // Winding axis endstop (optical gate)
volatile bool estop_b_min = false;  // Traverse endstop at extended position

void IRAM_ATTR isr_estop_a() { estop_a = (digitalRead(ENDSTOP_A) == LOW); }
void IRAM_ATTR isr_estop_b_min() { estop_b_min = (digitalRead(ENDSTOP_B_MIN) == LOW); }

inline bool safe_dir_a(bool cw)  { return true; }  // A sensor is index/homing reference, not a hard endstop
inline bool safe_dir_b(bool fwd) {
    if (!end_sensors_enabled) return true;
    return fwd ? true : !estop_b_min;  // Allow forward freely, only protect backward
}

// ============================================================================
// WIND STATE
// ============================================================================
enum WindState {
    WIND_IDLE,
    WIND_RUNNING,
    WIND_DONE,
    WIND_MANUAL_FWD,
    WIND_MANUAL_REV,
    WIND_HOMING_A,
    WIND_HOMING_B,
    WIND_ALARM
};

volatile WindState  windState     = WIND_IDLE;
volatile int        windTurnsDone = 0;
volatile int        windLayerDone = 0;
volatile char       alarmMsg[48]  = "";
volatile bool       windPaused    = false;  // Pause flag for winding

// ============================================================================
// COIL MATH
// ============================================================================
float wheeler_uh(int N, float r_inner_mm, float wire_d_mm,
                 int layers, float width_mm) {
    if (layers == 0 || N == 0) return 0.0f;
    float depth = layers * wire_d_mm;
    float a = (r_inner_mm + depth / 2.0f) / 10.0f;
    float l = width_mm / 10.0f;
    float c = depth / 10.0f;
    float d = 6.0f*a + 9.0f*l + 10.0f*c;
    return d > 0 ? (0.8f * a*a * (float)N*(float)N) / d : 0.0f;
}

bool solve_turns(float target_uh, float r_inner, float wire_d, float width,
                 float core_ur, int &out_N, int &out_L, float &out_uh) {
    int tpl = max(1, (int)(width / wire_d));
    for (int N = 1; N <= 50000; N++) {
        int   layers = (N + tpl - 1) / tpl;
        float L = wheeler_uh(N, r_inner, wire_d, layers, width) * core_ur;
        if (L >= target_uh) { out_N=N; out_L=layers; out_uh=L; return true; }
    }
    int layers = (50000 + tpl - 1) / tpl;
    out_N  = 50000;
    out_L  = layers;
    out_uh = wheeler_uh(50000, r_inner, wire_d, layers, width) * core_ur;
    return false;
}

// ============================================================================
// NVS PERSISTENCE
// ============================================================================
void save_prefs() {
    prefs.begin("cw", false);
    prefs.putFloat("wire",  p.wire_dia_mm);
    prefs.putFloat("width", p.coil_width_mm);
    prefs.putFloat("rin",   p.r_inner_mm);
    prefs.putInt  ("mode",  p.mode);
    prefs.putInt  ("turns", p.target_turns);
    prefs.putFloat("uh",    p.target_uh);
    prefs.putFloat("ur",    p.core_ur);
    prefs.putBool ("mh",    display_in_mh);
    prefs.putBool ("esens", end_sensors_enabled);
    prefs.end();
}

void load_prefs() {
    prefs.begin("cw", true);
    p.wire_dia_mm   = prefs.getFloat("wire",  0.50f);
    p.coil_width_mm = prefs.getFloat("width", 20.0f);
    p.r_inner_mm    = prefs.getFloat("rin",   10.0f);
    p.mode          = prefs.getInt  ("mode",  0);
    p.target_turns  = prefs.getInt  ("turns", 100);
    p.target_uh     = prefs.getFloat("uh",    100.0f);
    p.core_ur       = prefs.getFloat("ur",    1.0f);
    display_in_mh   = prefs.getBool ("mh",    false);
    end_sensors_enabled = prefs.getBool("esens", true);
    prefs.end();
}

void save_cfg() {
    prefs.begin("cfg", false);
    prefs.putInt  ("sra",  cfg.steps_rev_a);
    prefs.putInt  ("srb",  cfg.steps_rev_b);
    prefs.putFloat("gra",  cfg.gear_ratio_a);
    prefs.putFloat("mprb", cfg.mm_per_rev_b);
    prefs.putInt  ("sw",   cfg.speed_wind);
    prefs.putInt  ("sh",   cfg.speed_home);
    prefs.putInt  ("sb",   cfg.speed_backoff);
    prefs.putInt  ("acc",  cfg.accel_steps);
    prefs.putFloat("bom",  cfg.back_off_mm);
    prefs.putBool ("dc",   cfg.diam_calib);
    prefs.putFloat("ft",   cfg.film_thick_mm);
    prefs.end();
    recalc_cfg();
}

void load_cfg() {
    prefs.begin("cfg", true);
    cfg.steps_rev_a  = prefs.getInt  ("sra",  FULL_STEPS_PER_REV * MICROSTEPS);
    cfg.steps_rev_b  = prefs.getInt  ("srb",  FULL_STEPS_PER_REV * MICROSTEPS);
    cfg.gear_ratio_a = prefs.getFloat("gra",  4.148f);
    cfg.mm_per_rev_b = prefs.getFloat("mprb", 84.0f);
    cfg.speed_wind   = prefs.getInt  ("sw",   1600);
    cfg.speed_home   = prefs.getInt  ("sh",   400);
    cfg.speed_backoff= prefs.getInt  ("sb",   400);
    cfg.accel_steps  = prefs.getInt  ("acc",  200);
    cfg.back_off_mm  = prefs.getFloat("bom",  2.0f);
    cfg.diam_calib   = prefs.getBool ("dc",   false);
    cfg.film_thick_mm = prefs.getFloat("ft",   0.05f);
    prefs.end();
    recalc_cfg();
}

void reset_cfg() {
    cfg = MotorConfig();
    prefs.begin("cfg", false);
    prefs.clear();
    prefs.end();
    recalc_cfg();
}

void save_touch_cal() {
    prefs.begin("tcal", false);
    prefs.putInt("xmin", touch_x_min);
    prefs.putInt("xmax", touch_x_max);
    prefs.putInt("ymin", touch_y_min);
    prefs.putInt("ymax", touch_y_max);
    prefs.putInt("rot",  LV_ROT);  // also save the rotation it was calibrated for
    prefs.end();
}

bool touch_cal_exists() {
    prefs.begin("tcal", true);
    bool exists = prefs.isKey("xmin") && prefs.isKey("rot");
    int saved_rot = prefs.getInt("rot", -1);
    prefs.end();
    // Only consider calibration valid if rotation matches current rotation
    return exists && (saved_rot == LV_ROT);
}

void load_touch_cal() {
    prefs.begin("tcal", true);
    touch_x_min = prefs.getInt("xmin", TOUCH_X_MIN_DEFAULT);
    touch_x_max = prefs.getInt("xmax", TOUCH_X_MAX_DEFAULT);
    touch_y_min = prefs.getInt("ymin", TOUCH_Y_MIN_DEFAULT);
    touch_y_max = prefs.getInt("ymax", TOUCH_Y_MAX_DEFAULT);
    int saved_rot = prefs.getInt("rot", -1);
    prefs.end();

    // If rotation has changed since last calibration, reject cached values.
    // This forces recalibration when LV_ROT is changed, preventing misalignment.
    if (saved_rot != -1 && saved_rot != LV_ROT) {
        Serial.println("Touch cal rotation mismatch — invalidating cached cal");
        touch_x_min = TOUCH_X_MIN_DEFAULT;
        touch_x_max = TOUCH_X_MAX_DEFAULT;
        touch_y_min = TOUCH_Y_MIN_DEFAULT;
        touch_y_max = TOUCH_Y_MAX_DEFAULT;
    }
}

void save_slot(int n) {
    char ns[8]; snprintf(ns, sizeof(ns), "sl%d", n);
    prefs.begin(ns, false);
    prefs.putFloat("wire",  p.wire_dia_mm);
    prefs.putFloat("width", p.coil_width_mm);
    prefs.putFloat("rin",   p.r_inner_mm);
    prefs.putInt  ("mode",  p.mode);
    prefs.putInt  ("turns", p.target_turns);
    prefs.putFloat("uh",    p.target_uh);
    prefs.putFloat("ur",    p.core_ur);
    prefs.putBool ("used",  true);
    prefs.end();
}

bool load_slot(int n) {
    char ns[8]; snprintf(ns, sizeof(ns), "sl%d", n);
    prefs.begin(ns, true);
    bool used = prefs.getBool("used", false);
    if (used) {
        p.wire_dia_mm   = prefs.getFloat("wire",  0.50f);
        p.coil_width_mm = prefs.getFloat("width", 20.0f);
        p.r_inner_mm    = prefs.getFloat("rin",   10.0f);
        p.mode          = prefs.getInt  ("mode",  0);
        p.target_turns  = prefs.getInt  ("turns", 100);
        p.target_uh     = prefs.getFloat("uh",    100.0f);
        p.core_ur       = prefs.getFloat("ur",    1.0f);
    }
    prefs.end();
    return used;
}

bool slot_used(int n) {
    char ns[8]; snprintf(ns, sizeof(ns), "sl%d", n);
    prefs.begin(ns, true);
    bool used = prefs.getBool("used", false);
    prefs.end();
    return used;
}

void reset_prefs() {
    prefs.begin("cw", false);
    prefs.clear();
    prefs.end();
    p = CoilParams();  // reset to struct defaults
}

// Helper: Get effective coil diameter at a specific layer
float get_layer_diameter(int layer) {
    if (!cfg.diam_calib) {
        // Without calibration, use inner diameter
        return 2.0f * p.r_inner_mm;
    }
    // With calibration: diameter grows as we add layers
    // Each layer adds film + wire thickness on both sides
    float buildup_per_side = cfg.film_thick_mm + p.wire_dia_mm;
    return 2.0f * (p.r_inner_mm + layer * buildup_per_side);
}

void calculate_coil() {
    int   tpl = max(1, (int)(p.coil_width_mm / p.wire_dia_mm));
    int   turns, layers;
    float uh;

    if (p.mode == 1) {
        solve_turns(p.target_uh, p.r_inner_mm, p.wire_dia_mm,
                    p.coil_width_mm, 1.0f, turns, layers, uh);
    } else if (p.mode == 2) {
        solve_turns(p.target_uh, p.r_inner_mm, p.wire_dia_mm,
                    p.coil_width_mm, p.core_ur, turns, layers, uh);
    } else {
        layers = (p.target_turns + tpl - 1) / tpl;
        turns  = p.target_turns;
        uh     = wheeler_uh(turns, p.r_inner_mm, p.wire_dia_mm,
                            layers, p.coil_width_mm) * p.core_ur;
    }

    float trav_steps = cfg_steps_mm_b * p.wire_dia_mm;
    float rot_time   = cfg_steps_rev_a_eff / cfg.speed_wind;  // Use effective steps with gear ratio
    int   trav_spd   = max(10, (int)(trav_steps / rot_time));

    plan.turns_per_layer = tpl;
    plan.layers          = layers;
    plan.total_turns     = turns;
    plan.achieved_uh     = uh;
    plan.trav_steps_turn = trav_steps;
    plan.trav_speed      = trav_spd;
    plan.valid           = true;

    // Debug output
    Serial.printf("COIL PLAN: mode=%d wire=%.2f width=%.1f target_turns=%d\n",
                  p.mode, p.wire_dia_mm, p.coil_width_mm, p.target_turns);
    Serial.printf("  tpl=%d layers=%d total_turns=%d trav_steps=%.1f\n",
                  tpl, layers, turns, trav_steps);
    Serial.printf("  gear_ratio_a=%.3f mm_per_rev_b=%.1f eff_steps_a=%.0f\n",
                  cfg.gear_ratio_a, cfg.mm_per_rev_b, cfg_steps_rev_a_eff);
}

String fmt_uh(float uh) {
    if (uh >= 1000.0f) return String(uh/1000.0f, 2) + " mH";
    if (uh >= 1.0f)    return String(uh, 1) + " uH";
    return String(uh*1000.0f, 0) + " nH";
}

// ============================================================================
// STEPPER HELPERS
// ============================================================================
inline void pulse(uint8_t pin) {
    digitalWrite(pin, HIGH);
    delayMicroseconds(STEP_PULSE_US);
    digitalWrite(pin, LOW);
    delayMicroseconds(STEP_PULSE_US);
}

void enable_motors(bool on) {
    int v = on ? LOW : HIGH;
    digitalWrite(EN_A, v);
}

bool move_steps_a(int steps, int speed_sps) {
    if (steps == 0) return true;
    bool cw = steps > 0;
    if (!safe_dir_a(cw)) return false;
    digitalWrite(DIR_A, cw ? HIGH : LOW);
    int delay_us = max(100, 1000000 / speed_sps);
    for (int i = 0; i < abs(steps); i++) {
        if (!safe_dir_a(cw)) return false;
        pulse(STEP_A);
        delayMicroseconds(delay_us - STEP_PULSE_US * 2);
        if (i % 100 == 0) esp_task_wdt_reset();  // Feed watchdog periodically
    }
    return true;
}

bool move_steps_b(int steps, int speed_sps) {
    if (steps == 0) return true;
    bool fwd = steps > 0;
    if (!safe_dir_b(fwd)) return false;
    digitalWrite(DIR_B, fwd ? HIGH : LOW);
    int delay_us = max(100, 1000000 / speed_sps);
    for (int i = 0; i < abs(steps); i++) {
        if (!safe_dir_b(fwd)) return false;
        pulse(STEP_B);
        delayMicroseconds(delay_us - STEP_PULSE_US * 2);
        if (i % 100 == 0) esp_task_wdt_reset();  // Feed watchdog periodically
    }
    return true;
}

// ============================================================================
// HOMING
// ============================================================================
void home_axis_b() {
    enable_motors(true);
    vTaskDelay(pdMS_TO_TICKS(50));

    if (!end_sensors_enabled) return;

    // If sensor is already triggered, back off first
    if (estop_b_min) {
        // Back off: move forward (opposite of home direction) slowly until sensor releases
        digitalWrite(DIR_B, HIGH);  // Forward
        int backoff_us = 1000000 / (cfg.speed_home / 2);  // Half speed
        int iterations = 0;
        while (estop_b_min) {
            pulse(STEP_B);
            delayMicroseconds(backoff_us - STEP_PULSE_US * 2);
            if (++iterations % 100 == 0) esp_task_wdt_reset();
        }
        vTaskDelay(pdMS_TO_TICKS(100));  // Settle
    }

    // Move slowly toward home position until sensor triggers
    digitalWrite(DIR_B, LOW);  // Backward - home direction
    int slow_us = 1000000 / (cfg.speed_home / 4);  // Quarter speed
    int iterations = 0;
    while (!estop_b_min) {
        pulse(STEP_B);
        delayMicroseconds(slow_us - STEP_PULSE_US * 2);
        if (++iterations % 100 == 0) esp_task_wdt_reset();
    }

    // Small backoff from home position
    move_steps_b((int)(cfg_steps_mm_b * cfg.back_off_mm), cfg.speed_backoff);
}

void home_axis_a() {
    enable_motors(true);
    vTaskDelay(pdMS_TO_TICKS(50));

    if (!end_sensors_enabled) return;

    // If sensor is already triggered, back off first
    if (estop_a) {
        // Back off: move CW (opposite of home direction) slowly until sensor releases
        digitalWrite(DIR_A, HIGH);  // CW
        int backoff_us = 1000000 / (cfg.speed_home / 2);  // Half speed
        int iterations = 0;
        while (estop_a) {
            pulse(STEP_A);
            delayMicroseconds(backoff_us - STEP_PULSE_US * 2);
            if (++iterations % 100 == 0) esp_task_wdt_reset();
        }
        vTaskDelay(pdMS_TO_TICKS(100));  // Settle
    }

    // Move slowly toward home position until sensor triggers
    digitalWrite(DIR_A, LOW);  // CCW - home direction
    int slow_us = 1000000 / (cfg.speed_home / 4);  // Quarter speed
    int iterations = 0;
    while (!estop_a) {
        pulse(STEP_A);
        delayMicroseconds(slow_us - STEP_PULSE_US * 2);
        if (++iterations % 100 == 0) esp_task_wdt_reset();
    }

    // Small backoff from home position
    move_steps_a((int)(cfg_steps_rev_a_eff / 20), cfg.speed_backoff);  // Use effective steps with gear ratio
}

// ============================================================================
// WINDING
// ============================================================================
void run_winding() {
    // Disable endstop interrupts during winding to avoid flooding from rotating optical disc
    if (end_sensors_enabled) {
        detachInterrupt(digitalPinToInterrupt(ENDSTOP_A));
        detachInterrupt(digitalPinToInterrupt(ENDSTOP_B_MIN));
    }

    enable_motors(true);
    vTaskDelay(pdMS_TO_TICKS(100));

    int rot_us   = 1000000 / cfg.speed_wind;
    int trav_us  = 1000000 / max(10, plan.trav_speed);

    int32_t step_count = 0;

    digitalWrite(DIR_A, HIGH);

    for (int layer = 0; layer < plan.layers && windState == WIND_RUNNING; layer++) {
        windLayerDone = layer + 1;
        bool trav_fwd = (layer % 2 == 0);
        int remaining_turns = plan.total_turns - windTurnsDone;
        int turns_this_layer = min(plan.turns_per_layer, remaining_turns);

        if (turns_this_layer <= 0) {
            break;
        }
        
        // Recalculate traverse timing if diameter calibration is enabled
        float layer_trav_steps = plan.trav_steps_turn;
        int layer_trav_us = 1000000 / max(10, plan.trav_speed);
        
        if (cfg.diam_calib) {
            float diam = get_layer_diameter(layer);
            float ref_diam = get_layer_diameter(0);  // Reference: innermost layer
            // Adjust traverse steps: outer layers need proportionally fewer steps per rotation
            // to maintain same physical wire spacing
            layer_trav_steps = plan.trav_steps_turn * (ref_diam / diam);
            // Recalculate speed for this layer
            float rot_time_us = cfg_steps_rev_a_eff / cfg.speed_wind;
            layer_trav_us = max(10, (int)(layer_trav_steps / (rot_time_us / 1.0e6f)));
            Serial.printf("Layer %d diam=%.2fmm ref=%.2fmm trav_steps=%.1f->%.1f\n",
                          layer, diam, ref_diam, plan.trav_steps_turn, layer_trav_steps);
        }

        if (!safe_dir_b(trav_fwd)) {
            snprintf((char*)alarmMsg, sizeof(alarmMsg),
                     "Traverse endstop L%d", layer+1);
            Serial.printf("ALARM: Traverse endstop at layer %d! trav_fwd=%d\n", layer+1, trav_fwd);
            windState = WIND_ALARM;
            break;
        }
        digitalWrite(DIR_B, trav_fwd ? HIGH : LOW);

        for (int t = 0; t < turns_this_layer && windState == WIND_RUNNING; t++) {
            windTurnsDone++;

            uint32_t rot_rem  = (uint32_t)cfg_steps_rev_a_eff;  // Use effective steps with gear ratio
            uint32_t trav_rem = (uint32_t)layer_trav_steps;     // Use layer-specific traverse steps
            uint32_t pulse_count_a = 0;
            uint32_t pulse_count_b = 0;
            uint32_t t0 = micros();
            uint32_t pause_start = 0;  // Track pause start time for timing adjustment
            int loop_iterations = 0;

            while (rot_rem > 0 || trav_rem > 0) {
                if (windState != WIND_RUNNING) goto winding_done;
                
                // Handle pause: yield CPU and wait without stopping
                if (windPaused) {
                    if (pause_start == 0) {
                        pause_start = micros();  // Record when pause started
                    }
                    vTaskDelay(pdMS_TO_TICKS(10));  // Yield for 10ms while paused
                    continue;  // Don't execute steps, just keep looping
                }
                
                // If resuming from pause, adjust the start time to account for pause duration
                if (pause_start > 0) {
                    uint32_t pause_duration = micros() - pause_start;
                    t0 += pause_duration;  // Shift start time forward by pause duration
                    pause_start = 0;  // Clear pause marker
                }
                
                uint32_t elapsed = micros() - t0;

                if (rot_rem > 0) {
                    // Calculate steps due based on elapsed time, with safe underflow handling
                    uint32_t total_due = elapsed / rot_us;
                    if (total_due > pulse_count_a) {
                        uint32_t due = min(total_due - pulse_count_a, rot_rem);
                        for (uint32_t s = 0; s < due; s++) {
                            pulse(STEP_A);
                            step_count++;
                        }
                        pulse_count_a += due;
                        rot_rem -= due;
                    }
                }
                if (trav_rem > 0) {
                    // Calculate steps due based on elapsed time, with safe underflow handling
                    uint32_t total_due = elapsed / layer_trav_us;
                    if (total_due > pulse_count_b) {
                        uint32_t due = min(total_due - pulse_count_b, trav_rem);
                        for (uint32_t s = 0; s < due; s++) pulse(STEP_B);
                        pulse_count_b += due;
                        trav_rem -= due;
                    }
                }
                
                loop_iterations++;
                // Yield CPU occasionally to allow IDLE task to run
                if (loop_iterations % 1000 == 0) {
                    vTaskDelay(pdMS_TO_TICKS(1));  // Yield for 1ms
                    esp_task_wdt_reset();
                } else if (loop_iterations % 100 == 0) {
                    esp_task_wdt_reset();
                }
                
                delayMicroseconds(5);
            }

            // Step counting complete for this turn - continue to next turn
        }
    }

    winding_done:
    enable_motors(false);
    
    Serial.printf("WINDING DONE: windTurnsDone=%d plan.total_turns=%d windState=%d\n",
                  windTurnsDone, plan.total_turns, windState);
    
    // Re-enable endstop interrupts after winding if they were enabled
    if (end_sensors_enabled) {
        attachInterrupt(digitalPinToInterrupt(ENDSTOP_A), isr_estop_a, CHANGE);
        attachInterrupt(digitalPinToInterrupt(ENDSTOP_B_MIN), isr_estop_b_min, CHANGE);
    }
    
    if (windState == WIND_RUNNING) windState = WIND_DONE;
}

// ============================================================================
// MANUAL MODE
// ============================================================================
void run_manual() {
    // Disable endstop interrupts during manual mode if they're enabled
    if (end_sensors_enabled) {
        detachInterrupt(digitalPinToInterrupt(ENDSTOP_A));
        detachInterrupt(digitalPinToInterrupt(ENDSTOP_B_MIN));
    }

    enable_motors(true);
    int   rot_us        = 1000000 / cfg.speed_wind;
    float trav_per_step = plan.valid ?
        plan.trav_steps_turn / cfg_steps_rev_a_eff : 0.0f;
    float trav_accum    = 0.0f;
    int loop_iterations = 0;
    int32_t manual_step_count = 0;          // Signed count of motor A steps
    bool prev_cw = true;                    // Track previous direction

    while (windState == WIND_MANUAL_FWD || windState == WIND_MANUAL_REV) {
        bool cw = (windState == WIND_MANUAL_FWD);
        if (!safe_dir_a(cw)) { windState = WIND_IDLE; break; }

        digitalWrite(DIR_A, cw ? HIGH : LOW);
        pulse(STEP_A);
        
        // Update step count based on actual rotation direction
        if (cw) {
            manual_step_count++;
        } else {
            manual_step_count--;
        }
        
        // Calculate current layer based on absolute step count
        int current_layer = abs(manual_step_count) / (int)cfg_steps_rev_a_eff;
        
        // Determine traverse direction: layer parity XOR with rotation direction
        // This makes B reverse when we change rotation direction or complete a layer
        bool layer_fwd = (current_layer % 2 == 0);
        bool trav_fwd = (layer_fwd == cw);  // XOR: same direction in even layers, opposite in odd
        
        // Update DIR_B whenever direction changes or layer changes
        if (cw != prev_cw || (loop_iterations == 0)) {
            digitalWrite(DIR_B, trav_fwd ? HIGH : LOW);
            prev_cw = cw;
        }
        
        // Accumulate and execute traverse steps
        if (plan.valid && safe_dir_b(trav_fwd)) {
            trav_accum += trav_per_step;
            if (trav_accum >= 1.0f) {
                int w = (int)trav_accum;
                for (int i = 0; i < w; i++) pulse(STEP_B);
                trav_accum -= w;
            }
        }
        
        loop_iterations++;
        // Yield CPU occasionally to allow IDLE task to run
        if (loop_iterations % 1000 == 0) {
            vTaskDelay(pdMS_TO_TICKS(1));
            esp_task_wdt_reset();
        } else if (loop_iterations % 100 == 0) {
            esp_task_wdt_reset();
        }
        
        delayMicroseconds(rot_us - STEP_PULSE_US * 2);
    }
    enable_motors(false);
    
    // Re-enable endstop interrupts after manual mode if they were enabled
    if (end_sensors_enabled) {
        attachInterrupt(digitalPinToInterrupt(ENDSTOP_A), isr_estop_a, CHANGE);
        attachInterrupt(digitalPinToInterrupt(ENDSTOP_B_MIN), isr_estop_b_min, CHANGE);
    }
}

// ============================================================================
// STEPPER TASK  (Core 0)
// ============================================================================
void stepperTask(void *pv) {
    while (true) {
        switch (windState) {
            case WIND_RUNNING:
                run_winding();
                break;
            case WIND_MANUAL_FWD:
            case WIND_MANUAL_REV:
                run_manual();
                break;
            case WIND_HOMING_A:
                home_axis_a();
                windState = WIND_IDLE;
                break;
            case WIND_HOMING_B:
                home_axis_b();
                windState = WIND_IDLE;
                break;
            default:
                vTaskDelay(pdMS_TO_TICKS(20));
                break;
        }
    }
}

// ============================================================================
// LVGL DISPLAY + TOUCH CALLBACKS
// ============================================================================
static void lvgl_flush(lv_display_t *disp, const lv_area_t *area, uint8_t *px) {
    uint32_t w = (area->x2 - area->x1 + 1);
    uint32_t h = (area->y2 - area->y1 + 1);

    tft.startWrite();
    tft.setAddrWindow(area->x1, area->y1, w, h);
    tft.pushColors((uint16_t *)px, w * h, true);
    tft.endWrite();

    // Release HSPI so touch can use it
    digitalWrite(TFT_CS, HIGH);

    lv_display_flush_ready(disp);
}

static void lvgl_touch_read(lv_indev_t *indev, lv_indev_data_t *data) {
    if (touch.tirqTouched() && touch.touched()) {
        TS_Point pt = touch.getPoint();
        int px = map(pt.x, touch_x_min, touch_x_max, 1, SCREEN_W);
        int py = map(pt.y, touch_y_min, touch_y_max, 1, SCREEN_H);
        data->point.x = constrain(px, 0, SCREEN_W-1);
        data->point.y = constrain(py, 0, SCREEN_H-1);
        data->state   = LV_INDEV_STATE_PRESSED;
    } else {
        data->state = LV_INDEV_STATE_RELEASED;
    }
}
// LVGL COLOUR PALETTE
// ============================================================================
#define CLR_BG      lv_color_hex(0x1A1A2E)
#define CLR_PANEL   lv_color_hex(0x16213E)
#define CLR_ACCENT  lv_color_hex(0x0F3460)
#define CLR_GREEN   lv_color_hex(0x00B894)
#define CLR_ORANGE  lv_color_hex(0xE17055)
#define CLR_RED     lv_color_hex(0xD63031)
#define CLR_GREY    lv_color_hex(0x636E72)
#define CLR_WHITE   lv_color_hex(0xDFE6E9)
#define CLR_YELLOW  lv_color_hex(0xFDCB6E)
#define CLR_PURPLE  lv_color_hex(0x6C5CE7)

// ============================================================================
// LVGL SCREEN POINTERS
// ============================================================================
static lv_obj_t *scr_main    = nullptr;
static lv_obj_t *scr_wind    = nullptr;
static lv_obj_t *scr_manual  = nullptr;
static lv_obj_t *scr_confirm = nullptr;
static lv_obj_t *scr_alarm   = nullptr;
static lv_obj_t *scr_settings   = nullptr;
static lv_obj_t *scr_motorparams= nullptr;
static lv_obj_t *scr_cal      = nullptr;
static lv_obj_t *homing_overlay = nullptr;  // Modal overlay for homing status

static lv_obj_t *lbl_result     = nullptr;
static lv_obj_t *lbl_wind_turns = nullptr;
static lv_obj_t *lbl_wind_layer = nullptr;
static lv_obj_t *lbl_wind_pct   = nullptr;
static lv_obj_t *bar_wind       = nullptr;
static lv_obj_t *lbl_wind_enc   = nullptr;
static lv_obj_t *lbl_manual_st  = nullptr;
static lv_obj_t *lbl_manual_enc = nullptr;

static lv_timer_t *wind_timer   = nullptr;
static lv_timer_t *manual_timer = nullptr;
static lv_timer_t *mparam_hold_timer = nullptr;
static lv_timer_t *cal_timer = nullptr;
static lv_timer_t *homing_timer = nullptr;  // Timer to check homing completion
static bool homing_overlay_fading = false;  // Track if overlay is fading out

// Hold-to-repeat state variables (used by multiple screens)
static ParamRow *held_mparam = nullptr;
static int held_mdir = 0;

// ============================================================================
// LVGL HELPER FUNCTIONS
// ============================================================================
static lv_obj_t* make_btn(lv_obj_t *parent, const char *txt, lv_color_t bg,
                           lv_event_cb_t cb, void *ud, int w, int h) {
    lv_obj_t *b = lv_btn_create(parent);
    lv_obj_set_size(b, w, h);
    lv_obj_set_style_bg_color(b, bg, 0);
    lv_obj_set_style_radius(b, 6, 0);
    lv_obj_set_style_border_width(b, 0, 0);
    if (cb) lv_obj_add_event_cb(b, cb, LV_EVENT_CLICKED, ud);
    lv_obj_t *l = lv_label_create(b);
    lv_label_set_text(l, txt);
    lv_obj_set_style_text_color(l, CLR_WHITE, 0);
    lv_obj_set_style_text_font(l, &lv_font_montserrat_14, 0);
    lv_obj_center(l);
    return b;
}

static lv_obj_t* make_lbl(lv_obj_t *parent, const char *txt,
                            const lv_font_t *f, lv_color_t c) {
    lv_obj_t *l = lv_label_create(parent);
    lv_label_set_text(l, txt);
    lv_obj_set_style_text_font(l, f, 0);
    lv_obj_set_style_text_color(l, c, 0);
    return l;
}

// Disable scroll + shadow + border on any container
static void fix_obj(lv_obj_t *o) {
    lv_obj_clear_flag(o, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_border_width(o, 0, 0);
    lv_obj_set_style_shadow_width(o, 0, 0);
    lv_obj_set_style_outline_width(o, 0, 0);
}

static void update_result() {
    if (!lbl_result || !plan.valid) return;
    char buf[80];
    snprintf(buf, sizeof(buf), "Predicted:  %dT   %d layers   L = %s",
             plan.total_turns, plan.layers,
             fmt_uh(plan.achieved_uh).c_str());
    lv_label_set_text(lbl_result, buf);
}

// ============================================================================
// PARAM ROW CALLBACKS
// ============================================================================
static void update_param_label(ParamRow *r) {
    if (!r->val_lbl) return;
    char buf[32];
    if (r->f_ptr) {
        float val = *r->f_ptr;
        const char *unit = r->unit;
        if (r->f_ptr == &p.target_uh && display_in_mh) {
            val = val / 1000.0f;
            unit = "mH";
        }
        snprintf(buf, sizeof(buf), "%.*f %s", r->dec, val, unit);
    } else {
        snprintf(buf, sizeof(buf), "%d %s", *r->i_ptr, r->unit);
    }
    lv_label_set_text(r->val_lbl, buf);
}

void nudge(ParamRow *r, int dir) {
    if (r->f_ptr) {
        // special case: width increments use wire diameter
        if (r->f_ptr == &p.coil_width_mm) {
            float step = p.wire_dia_mm;
            float nv = *r->f_ptr + dir * step;
            nv = roundf(nv / step) * step;                // keep whole # turns per layer
            nv = constrain(nv, r->mn, r->mx);
            *r->f_ptr = nv;
        }
        else if (r->f_ptr == &p.target_uh && display_in_mh) {
            float step = 1000.0f * dir;
            float nv = *r->f_ptr + step;
            nv = constrain(nv, r->mn, r->mx);
            *r->f_ptr = nv;
        } else {
            float nv = roundf((*r->f_ptr + dir * r->step) / r->step) * r->step;
            *r->f_ptr = constrain(nv, r->mn, r->mx);
        }
        update_param_label(r);

        // if the wire diameter changed, adjust width row step
        if (r->f_ptr == &p.wire_dia_mm) {
            prows[1].step = p.wire_dia_mm;
            // optionally snap width to multiple of new wire dia
            float w = p.coil_width_mm;
            float m = roundf(w / p.wire_dia_mm);
            p.coil_width_mm = constrain(m * p.wire_dia_mm, prows[1].mn, prows[1].mx);
            update_param_label(&prows[1]);
        }
    } else {
        int nv = constrain(*r->i_ptr + dir*(int)r->step, (int)r->mn, (int)r->mx);
        *r->i_ptr = nv;
        char buf[24];
        snprintf(buf, sizeof(buf), "%d %s", nv, r->unit);
        lv_label_set_text(r->val_lbl, buf);
    }
    calculate_coil();
    update_result();
    // save_prefs() moved to on_param_btn_released() to avoid blocking on every increment
}

// ============================================================================
// PARAM ROW CALLBACKS
// ============================================================================
// State for hold-to-repeat on +/- buttons
static ParamRow *held_param = nullptr;
static int held_dir = 0;
static lv_timer_t *param_hold_timer = nullptr;

static void param_hold_timer_cb(lv_timer_t *t) {
    if (held_param && held_dir != 0) {
        nudge(held_param, held_dir);
    }
}

static void on_param_btn_pressed(lv_event_t *e, ParamRow *r, int dir) {
    if (!r || dir == 0) return;
    
    // Immediate single nudge on first press
    nudge(r, dir);
    
    // Start hold timer for repeat
    held_param = r;
    held_dir = dir;
    if (param_hold_timer) lv_timer_del(param_hold_timer);
    param_hold_timer = lv_timer_create(param_hold_timer_cb, 200, nullptr);
}

static void on_param_btn_released(lv_event_t *e) {
    // Stop repeat timer immediately
    if (param_hold_timer) {
        lv_timer_del(param_hold_timer);
        param_hold_timer = nullptr;
    }
    // Save preferences only once when button is released, not on every increment
    save_prefs();
    held_param = nullptr;
    held_dir = 0;
}

static void on_minus(lv_event_t *e) {
    ParamRow *r = (ParamRow*)lv_event_get_user_data(e);
    lv_event_code_t code = lv_event_get_code(e);
    if (code == LV_EVENT_PRESSED) {
        on_param_btn_pressed(e, r, -1);
    } else if (code == LV_EVENT_RELEASED || code == LV_EVENT_PRESS_LOST) {
        on_param_btn_released(e);
    }
}
static void on_plus(lv_event_t *e) {
    ParamRow *r = (ParamRow*)lv_event_get_user_data(e);
    lv_event_code_t code = lv_event_get_code(e);
    if (code == LV_EVENT_PRESSED) {
        on_param_btn_pressed(e, r, +1);
    } else if (code == LV_EVENT_RELEASED || code == LV_EVENT_PRESS_LOST) {
        on_param_btn_released(e);
    }
}

static lv_obj_t* add_param_row(lv_obj_t *cont, const char *name,
                                ParamRow *r) {
    lv_obj_t *row = lv_obj_create(cont);
    lv_obj_set_size(row, lv_pct(100), 32);
    lv_obj_set_style_bg_opa(row, LV_OPA_TRANSP, 0);
    lv_obj_clear_flag(row, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_border_width(row, 0, 0);
    lv_obj_set_style_pad_all(row, 2, 0);
    lv_obj_set_style_pad_column(row, 2, 0);
    lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(row, LV_FLEX_ALIGN_START,
                          LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

    // Name label — fixed 44px, small font so it fits
    lv_obj_t *nl = lv_label_create(row);
    lv_label_set_text(nl, name);
    lv_obj_set_width(nl, 44);
    lv_obj_set_style_text_font(nl, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_color(nl, CLR_WHITE, 0);

    // Minus button — fixed 24px
    lv_obj_t *btn_minus = make_btn(row, "-", CLR_RED, on_minus, r, 24, 24);
    lv_obj_add_event_cb(btn_minus, on_minus, LV_EVENT_PRESSED, r);
    lv_obj_add_event_cb(btn_minus, on_minus, LV_EVENT_RELEASED, r);

    // Value label — flex_grow=1 so it fills all remaining space between buttons
    lv_obj_t *vl = lv_label_create(row);
    lv_obj_set_flex_grow(vl, 1);
    lv_obj_set_style_text_font(vl, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(vl, CLR_YELLOW, 0);
    lv_obj_set_style_text_align(vl, LV_TEXT_ALIGN_CENTER, 0);

    // set initial text using helper (handles unit toggle)
    r->val_lbl = vl;
    update_param_label(r);

    // Plus button — fixed 24px
    lv_obj_t *btn_plus = make_btn(row, "+", CLR_GREEN, on_plus, r, 24, 24);
    lv_obj_add_event_cb(btn_plus, on_plus, LV_EVENT_PRESSED, r);
    lv_obj_add_event_cb(btn_plus, on_plus, LV_EVENT_RELEASED, r);
    return vl;
}

// ============================================================================
// MODE DROPDOWN CALLBACK
// ============================================================================
static lv_obj_t *slot_btns[4] = {nullptr};

static void refresh_slot_btns() {
    for (int i = 0; i < 4; i++) {
        if (!slot_btns[i]) continue;
        lv_obj_set_style_bg_color(slot_btns[i],
            slot_used(i) ? CLR_GREEN : CLR_GREY, 0);
    }
}

static void on_slot_short(lv_event_t *e) {
    int n = (int)(intptr_t)lv_event_get_user_data(e);
    if (!load_slot(n)) return;   // empty slot — ignore
    // refresh all param value labels
    for (int i = 0; i < 6; i++) {
        update_param_label(&prows[i]);
    }
    calculate_coil();
    update_result();
}

static void on_slot_long(lv_event_t *e) {
    int n = (int)(intptr_t)lv_event_get_user_data(e);
    save_slot(n);
    refresh_slot_btns();
}

static void on_reset(lv_event_t *e) {
    reset_prefs();
    for (int i = 0; i < 6; i++) {
        update_param_label(&prows[i]);
    }
    calculate_coil();
    update_result();
}

// Helper to clean up all active timers before returning to main
static void cleanup_all_timers() {
    if (param_hold_timer) {
        lv_timer_del(param_hold_timer);
        param_hold_timer = nullptr;
    }
    if (mparam_hold_timer) {
        lv_timer_del(mparam_hold_timer);
        mparam_hold_timer = nullptr;
    }
    if (manual_timer) {
        lv_timer_del(manual_timer);
        manual_timer = nullptr;
    }
    if (cal_timer) {
        lv_timer_del(cal_timer);
        cal_timer = nullptr;
    }
    if (wind_timer) {
        lv_timer_del(wind_timer);
        wind_timer = nullptr;
    }
    if (homing_timer) {
        lv_timer_del(homing_timer);
        homing_timer = nullptr;
    }
    homing_overlay_fading = false;
    // Reset hold-to-repeat state
    held_param = nullptr;
    held_dir = 0;
    held_mparam = nullptr;
    held_mdir = 0;
}

// Forward declarations for screen builders
static void build_main_screen();
static void build_settings_screen();

// Helper: Navigate back to main screen (used by all submenus)
static void navigate_to_main() {
    windState = WIND_IDLE;
    cleanup_all_timers();
    // Rebuild and load main screen FIRST to avoid white flash
    build_main_screen();
    lv_scr_load(scr_main);
    // Remove homing overlay if present
    if (homing_overlay) { lv_obj_del(homing_overlay); homing_overlay = nullptr; }
    // Then delete all submenu screens to free memory
    if (scr_settings) { lv_obj_del(scr_settings); scr_settings = nullptr; }
    if (scr_motorparams) { lv_obj_del(scr_motorparams); scr_motorparams = nullptr; }
    if (scr_manual) { lv_obj_del(scr_manual); scr_manual = nullptr; }
    if (scr_confirm) { lv_obj_del(scr_confirm); scr_confirm = nullptr; }
    if (scr_cal) { lv_obj_del(scr_cal); scr_cal = nullptr; }
    if (scr_wind) { lv_obj_del(scr_wind); scr_wind = nullptr; }
    if (scr_alarm) { lv_obj_del(scr_alarm); scr_alarm = nullptr; }
}

// Helper: Navigate back to settings screen (used by motor params, etc.)
static void navigate_to_settings() {
    cleanup_all_timers();
    // Rebuild and load settings screen FIRST to avoid white flash
    build_settings_screen();
    lv_scr_load(scr_settings);
    // Then delete old screens
    if (scr_motorparams) { lv_obj_del(scr_motorparams); scr_motorparams = nullptr; }
    if (scr_cal) { lv_obj_del(scr_cal); scr_cal = nullptr; }
}

// Forward declaration: build_main_screen is defined later but used in multiple places
static void build_main_screen();

// ============================================================================
// WINDING SCREEN
// ============================================================================
static lv_obj_t *btn_pause_wind = nullptr;

static void on_pause_winding(lv_event_t *e) {
    windPaused = !windPaused;
    if (btn_pause_wind) {
        if (windPaused) {
            lv_label_set_text(lv_obj_get_child(btn_pause_wind, 0), "RESUME");
            lv_obj_set_style_bg_color(btn_pause_wind, CLR_ORANGE, 0);
        } else {
            lv_label_set_text(lv_obj_get_child(btn_pause_wind, 0), "PAUSE");
            lv_obj_set_style_bg_color(btn_pause_wind, CLR_ACCENT, 0);
        }
    }
}

static void on_stop_winding(lv_event_t *e) {
    windPaused = false;
    navigate_to_main();
}

static void wind_update_cb(lv_timer_t *t) {
    if (windState == WIND_ALARM) {
        if (wind_timer) { lv_timer_del(wind_timer); wind_timer = nullptr; }
        if (scr_alarm) lv_obj_del(scr_alarm);
        scr_alarm = lv_obj_create(nullptr); fix_obj(scr_alarm);
        lv_obj_set_style_bg_color(scr_alarm, CLR_RED, 0);

        lv_obj_t *hl = make_lbl(scr_alarm, "! ALARM !",
                                 &lv_font_montserrat_24, CLR_WHITE);
        lv_obj_align(hl, LV_ALIGN_TOP_MID, 0, 20);

        lv_obj_t *ml = make_lbl(scr_alarm, (const char*)alarmMsg,
                                 &lv_font_montserrat_20, CLR_WHITE);
        lv_obj_align(ml, LV_ALIGN_CENTER, 0, 0);

        char enc_buf[64];
        snprintf(enc_buf, sizeof(enc_buf), "Steps A: %ld   Steps B: %ld",
                 (long)step_count_a, (long)step_count_b);
        lv_obj_t *el = make_lbl(scr_alarm, enc_buf,
                                 &lv_font_montserrat_14, CLR_WHITE);
        lv_obj_align(el, LV_ALIGN_CENTER, 0, 40);

        lv_obj_t *bb = make_btn(scr_alarm, "BACK TO MENU", CLR_ACCENT,
            [](lv_event_t*) { navigate_to_main(); },
            nullptr, 160, 44);
        lv_obj_align(bb, LV_ALIGN_BOTTOM_MID, 0, -16);
        lv_scr_load(scr_alarm);
        return;
    }

    char buf[40];
    snprintf(buf, sizeof(buf), "Turn: %d / %d", windTurnsDone, plan.total_turns);
    lv_label_set_text(lbl_wind_turns, buf);

    snprintf(buf, sizeof(buf), "Layer: %d / %d", windLayerDone, plan.layers);
    lv_label_set_text(lbl_wind_layer, buf);

    int pct = plan.total_turns > 0 ?
              constrain(windTurnsDone * 100 / plan.total_turns, 0, 100) : 0;
    lv_bar_set_value(bar_wind, pct, LV_ANIM_OFF);
    snprintf(buf, sizeof(buf), "%d%%", pct);
    lv_label_set_text(lbl_wind_pct, buf);

    if (windState == WIND_DONE) {
        if (wind_timer) { lv_timer_del(wind_timer); wind_timer = nullptr; }
        lv_label_set_text(lbl_wind_turns, "COMPLETE!");
        lv_bar_set_value(bar_wind, 100, LV_ANIM_OFF);
        lv_label_set_text(lbl_wind_pct, "100%");
    }
}

static void build_winding_screen() {
    scr_wind = lv_obj_create(nullptr); fix_obj(scr_wind);
    lv_obj_set_style_bg_color(scr_wind, CLR_BG, 0);

    lv_obj_t *hdr = lv_obj_create(scr_wind); fix_obj(hdr);
    lv_obj_set_size(hdr, SCREEN_W, 36);
    lv_obj_set_pos(hdr, 0, 0);
    lv_obj_set_style_bg_color(hdr, CLR_ACCENT, 0);
    lv_obj_set_style_border_width(hdr, 0, 0);

    lv_obj_t *hl = make_lbl(hdr, "WINDING", &lv_font_montserrat_20, CLR_WHITE);
    lv_obj_align(hl, LV_ALIGN_LEFT_MID, 8, 0);

    btn_pause_wind = make_btn(hdr, "PAUSE", CLR_ACCENT,
                              on_pause_winding, nullptr, 65, 28);
    lv_obj_align(btn_pause_wind, LV_ALIGN_RIGHT_MID, -82, 0);

    lv_obj_t *sb = make_btn(hdr, "STOP", CLR_RED,
                             on_stop_winding, nullptr, 70, 28);
    lv_obj_align(sb, LV_ALIGN_RIGHT_MID, -6, 0);

    lbl_wind_turns = make_lbl(scr_wind, "Turn: 0 / 0",
                               &lv_font_montserrat_24, CLR_WHITE);
    lv_obj_align(lbl_wind_turns, LV_ALIGN_TOP_LEFT, 8, 44);

    lbl_wind_layer = make_lbl(scr_wind, "Layer: 0 / 0",
                               &lv_font_montserrat_24, CLR_WHITE);
    lv_obj_align(lbl_wind_layer, LV_ALIGN_TOP_LEFT, 8, 80);

    bar_wind = lv_bar_create(scr_wind);
    lv_obj_set_size(bar_wind, 290, 18);
    lv_obj_align(bar_wind, LV_ALIGN_TOP_MID, 0, 118);
    lv_bar_set_range(bar_wind, 0, 100);
    lv_obj_set_style_bg_color(bar_wind, CLR_ACCENT, 0);
    lv_obj_set_style_bg_color(bar_wind, CLR_GREEN, LV_PART_INDICATOR);

    lbl_wind_pct = make_lbl(scr_wind, "0%",
                             &lv_font_montserrat_20, CLR_YELLOW);
    lv_obj_align(lbl_wind_pct, LV_ALIGN_TOP_MID, 0, 142);

    lbl_wind_enc = make_lbl(scr_wind, "ENC A: 0",
                             &lv_font_montserrat_14, CLR_GREY);
    lv_obj_align(lbl_wind_enc, LV_ALIGN_BOTTOM_LEFT, 8, -8);
}

// ============================================================================
// MANUAL SCREEN
// ============================================================================
static void manual_update_cb(lv_timer_t *t) {
    const char *s;
    lv_color_t  c;
    if      (windState == WIND_MANUAL_FWD) { s = ">> FORWARD"; c = CLR_GREEN;  }
    else if (windState == WIND_MANUAL_REV) { s = "<< REVERSE"; c = CLR_ORANGE; }
    else                                   { s = "-- STOPPED"; c = CLR_GREY;   }
    lv_label_set_text(lbl_manual_st, s);
    lv_obj_set_style_text_color(lbl_manual_st, c, 0);

    char buf[48];
    snprintf(buf, sizeof(buf), "Steps A: %ld   Steps B: %ld",
             (long)step_count_a, (long)step_count_b);
    lv_label_set_text(lbl_manual_enc, buf);
}

static void on_manual_back(lv_event_t *e) {
    navigate_to_main();
}

static void build_manual_screen() {
    if (manual_timer) {
        lv_timer_del(manual_timer);
        manual_timer = nullptr;
    }
    if (scr_manual) lv_obj_del(scr_manual);
    scr_manual = lv_obj_create(nullptr); fix_obj(scr_manual);
    lv_obj_set_style_bg_color(scr_manual, CLR_BG, 0);

    lv_obj_t *hdr = lv_obj_create(scr_manual); fix_obj(hdr);
    lv_obj_set_size(hdr, SCREEN_W, 36);
    lv_obj_set_pos(hdr, 0, 0);
    lv_obj_set_style_bg_color(hdr, CLR_ACCENT, 0);
    lv_obj_set_style_border_width(hdr, 0, 0);

    lv_obj_t *hl = make_lbl(hdr, "MANUAL WIND",
                             &lv_font_montserrat_20, CLR_WHITE);
    lv_obj_align(hl, LV_ALIGN_LEFT_MID, 8, 0);

    lv_obj_t *bb = make_btn(hdr, "BACK", CLR_GREY,
                             on_manual_back, nullptr, 60, 28);
    lv_obj_align(bb, LV_ALIGN_RIGHT_MID, -6, 0);

    lbl_manual_st = make_lbl(scr_manual, "-- STOPPED",
                              &lv_font_montserrat_24, CLR_GREY);
    lv_obj_align(lbl_manual_st, LV_ALIGN_TOP_MID, 0, 46);

    char info[56];
    snprintf(info, sizeof(info), "Wire %.2fmm  Width %.1fmm  T/layer %d",
             p.wire_dia_mm, p.coil_width_mm,
             plan.valid ? plan.turns_per_layer : 0);
    lv_obj_t *il = make_lbl(scr_manual, info,
                             &lv_font_montserrat_14, CLR_GREY);
    lv_obj_align(il, LV_ALIGN_TOP_MID, 0, 86);

    lbl_manual_enc = make_lbl(scr_manual, "Steps A: 0   Steps B: 0",
                               &lv_font_montserrat_14, CLR_YELLOW);
    lv_obj_align(lbl_manual_enc, LV_ALIGN_TOP_MID, 0, 110);

    // FWD hold button
    lv_obj_t *fb = lv_btn_create(scr_manual);
    lv_obj_set_size(fb, 128, 64);
    lv_obj_align(fb, LV_ALIGN_BOTTOM_LEFT, 16, -12);
    lv_obj_set_style_bg_color(fb, CLR_GREEN, 0);
    lv_obj_set_style_radius(fb, 8, 0);
    lv_obj_set_style_border_width(fb, 0, 0);
    lv_obj_add_event_cb(fb, [](lv_event_t*){ windState = WIND_MANUAL_FWD; },
                        LV_EVENT_PRESSED,  nullptr);
    lv_obj_add_event_cb(fb, [](lv_event_t*){ windState = WIND_IDLE; },
                        LV_EVENT_RELEASED, nullptr);
    lv_obj_t *fl = make_lbl(fb, "FWD\n(hold)", &lv_font_montserrat_16, CLR_WHITE);
    lv_obj_center(fl);

    // REV hold button
    lv_obj_t *rb = lv_btn_create(scr_manual);
    lv_obj_set_size(rb, 128, 64);
    lv_obj_align(rb, LV_ALIGN_BOTTOM_RIGHT, -16, -12);
    lv_obj_set_style_bg_color(rb, CLR_ORANGE, 0);
    lv_obj_set_style_radius(rb, 8, 0);
    lv_obj_set_style_border_width(rb, 0, 0);
    lv_obj_add_event_cb(rb, [](lv_event_t*){ windState = WIND_MANUAL_REV; },
                        LV_EVENT_PRESSED,  nullptr);
    lv_obj_add_event_cb(rb, [](lv_event_t*){ windState = WIND_IDLE; },
                        LV_EVENT_RELEASED, nullptr);
    lv_obj_t *rl = make_lbl(rb, "REV\n(hold)", &lv_font_montserrat_16, CLR_WHITE);
    lv_obj_center(rl);

    manual_timer = lv_timer_create(manual_update_cb, 100, nullptr);
}

// ============================================================================
// CONFIRM SCREEN
// ============================================================================
static void on_confirm_go(lv_event_t *e) {
    windTurnsDone = 0;
    windLayerDone = 0;
    windState     = WIND_RUNNING;
    
    // Rebuild winding screen if it was deleted
    if (!scr_wind) {
        build_winding_screen();
    }
    
    lv_label_set_text(lbl_wind_turns, "Turn: 0 / ...");
    lv_label_set_text(lbl_wind_layer, "Layer: 0 / ...");
    lv_bar_set_value(bar_wind, 0, LV_ANIM_OFF);
    lv_label_set_text(lbl_wind_pct, "0%");
    wind_timer = lv_timer_create(wind_update_cb, 250, nullptr);
    lv_scr_load(scr_wind);
}

static void on_confirm_cancel(lv_event_t *e) {
    navigate_to_main();
}

static void build_confirm_screen() {
    if (scr_confirm) lv_obj_del(scr_confirm);
    scr_confirm = lv_obj_create(nullptr); fix_obj(scr_confirm);
    lv_obj_set_style_bg_color(scr_confirm, CLR_BG, 0);

    lv_obj_t *hdr = lv_obj_create(scr_confirm); fix_obj(hdr);
    lv_obj_set_size(hdr, SCREEN_W, 36);
    lv_obj_set_pos(hdr, 0, 0);
    lv_obj_set_style_bg_color(hdr, CLR_GREEN, 0);
    lv_obj_set_style_border_width(hdr, 0, 0);
    lv_obj_t *hl = make_lbl(hdr, "CONFIRM WINDING",
                             &lv_font_montserrat_20, CLR_WHITE);
    lv_obj_center(hl);

    char buf[180];
    snprintf(buf, sizeof(buf),
        "Turns:      %d\n"
        "Layers:     %d  (%d T/layer)\n"
        "Inductance: %s\n"
        "Wire dia:   %.2f mm\n"
        "Width:      %.1f mm",
        plan.total_turns, plan.layers, plan.turns_per_layer,
        fmt_uh(plan.achieved_uh).c_str(),
        p.wire_dia_mm, p.coil_width_mm);

    lv_obj_t *dl = make_lbl(scr_confirm, buf,
                             &lv_font_montserrat_14, CLR_WHITE);
    lv_obj_align(dl, LV_ALIGN_TOP_LEFT, 14, 44);
    lv_label_set_long_mode(dl, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(dl, SCREEN_W - 28);

    lv_obj_t *go = make_btn(scr_confirm, "WIND NOW", CLR_GREEN,
                             on_confirm_go, nullptr, 130, 44);
    lv_obj_align(go, LV_ALIGN_BOTTOM_RIGHT, -14, -12);

    lv_obj_t *ca = make_btn(scr_confirm, "CANCEL", CLR_GREY,
                             on_confirm_cancel, nullptr, 100, 44);
    lv_obj_align(ca, LV_ALIGN_BOTTOM_LEFT, 14, -12);
}

// ============================================================================
// MAIN SCREEN
// ============================================================================

static void on_start(lv_event_t *e) {
    calculate_coil();
    build_confirm_screen();
    lv_scr_load(scr_confirm);
}

static void on_manual(lv_event_t *e) {
    cleanup_all_timers();
    build_manual_screen();
    lv_scr_load(scr_manual);
}

// Homing overlay functions
static void homing_timer_cb(lv_timer_t *t) {
    // If overlay is fading, delete it now
    if (homing_overlay_fading) {
        if (homing_overlay) {
            // Move off-screen before deleting to prevent visible redraw
            lv_obj_set_pos(homing_overlay, -500, -500);
            lv_obj_del(homing_overlay);
            homing_overlay = nullptr;
        }
        if (homing_timer) {
            lv_timer_del(homing_timer);
            homing_timer = nullptr;
        }
        homing_overlay_fading = false;
        return;
    }
    
    // Check if homing is still in progress
    if (windState != WIND_HOMING_A && windState != WIND_HOMING_B) {
        // Homing complete - fade out overlay smoothly
        if (homing_overlay) {
            // Make overlay and all children transparent
            lv_obj_set_style_opa(homing_overlay, LV_OPA_TRANSP, LV_PART_MAIN | LV_STATE_DEFAULT);
            homing_overlay_fading = true;
            // Will delete on next timer tick
        }
    }
}

static void show_homing_overlay(const char *msg) {
    // Remove existing overlay if any
    if (homing_overlay) {
        lv_obj_del(homing_overlay);
        homing_overlay = nullptr;
    }
    
    // Create semi-transparent background overlay
    homing_overlay = lv_obj_create(lv_scr_act());
    lv_obj_set_size(homing_overlay, SCREEN_W, SCREEN_H);
    lv_obj_set_pos(homing_overlay, 0, 0);
    lv_obj_set_style_bg_color(homing_overlay, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(homing_overlay, LV_OPA_70, 0);
    lv_obj_set_style_border_width(homing_overlay, 0, 0);
    lv_obj_clear_flag(homing_overlay, LV_OBJ_FLAG_SCROLLABLE);
    
    // Create message box in center
    lv_obj_t *box = lv_obj_create(homing_overlay);
    lv_obj_set_size(box, 200, 100);
    lv_obj_center(box);
    lv_obj_set_style_bg_color(box, CLR_ACCENT, 0);
    lv_obj_set_style_border_width(box, 2, 0);
    lv_obj_set_style_border_color(box, CLR_WHITE, 0);
    lv_obj_set_style_radius(box, 8, 0);
    
    lv_obj_t *lbl = make_lbl(box, msg, &lv_font_montserrat_20, CLR_WHITE);
    lv_obj_center(lbl);
    
    // Start timer to check for completion
    if (homing_timer) lv_timer_del(homing_timer);
    homing_timer = lv_timer_create(homing_timer_cb, 100, nullptr);
}

static void on_home_a(lv_event_t *e) { 
    windState = WIND_HOMING_A;
    show_homing_overlay("HOMING A...");
}
static void on_home_b(lv_event_t *e) { 
    windState = WIND_HOMING_B;
    show_homing_overlay("HOMING B...");
}


// ============================================================================
// TOUCH CALIBRATION SCREEN
// ============================================================================
// 3-point calibration: top-left, top-right, bottom-centre
/*static const int CAL_PTS = 3;
static const int cal_screen_x[CAL_PTS] = { 20,          SCREEN_W-20, SCREEN_W/2  };
static const int cal_screen_y[CAL_PTS] = { 20,          20,          SCREEN_H-20 };
static int  cal_raw_x[CAL_PTS];
static int  cal_raw_y[CAL_PTS];*/
// 2. Definujte polia ako premenné (bez 'const'), aby sa dali v build_cal_screen prepísať
static int cal_screen_x[CAL_PTS] = { 40, 280, 160 };
static int cal_screen_y[CAL_PTS] = { 40, 40, 200 };
static int cal_raw_x[CAL_PTS];
static int cal_raw_y[CAL_PTS];
static int  cal_step = 0;
static lv_obj_t *lbl_cal_inst = nullptr;
static lv_obj_t *cal_cross_h  = nullptr;
static lv_obj_t *cal_cross_v  = nullptr;

static void cal_draw_cross(int x, int y) {
    lv_obj_set_pos(cal_cross_h, x - 15, y - 1);
    lv_obj_set_pos(cal_cross_v, x - 1,  y - 15);
}

static bool cal_waiting_release = false;
static void cal_timer_cb(lv_timer_t *t) {
    static uint32_t last_touch_time = 0;
    bool touched = touch.touched();

    // 1. Ochrana proti rýchlym klikom (Debounce)
    // Ak od posledného úspešného bodu neprešlo aspoň 400ms, ignoruj dotyk
    if (millis() - last_touch_time < 400) return;

    // 2. Čakanie na pustenie prsta medzi bodmi
    if (cal_waiting_release) {
        if (!touched) cal_waiting_release = false;
        return;
    }

    if (!touched) return;

    // 3. Získanie súradníc
    TS_Point pt = touch.getPoint();
    if (pt.z < 500) return;  // Ignoruj slabé dotyky

    cal_raw_x[cal_step] = pt.x;
    cal_raw_y[cal_step] = pt.y;
    Serial.printf("Bod %d uložený: x=%d y=%d\n", cal_step + 1, pt.x, pt.y);
    
    cal_step++;
    cal_waiting_release = true;
    last_touch_time = millis(); // Zaznamenaj čas tohto dotyku

    // 4. Vyhodnotenie po získaní všetkých 3 bodov
    if (cal_step >= CAL_PTS) {
        // Získanie min/max z nameraných hodnôt
        touch_x_min = min({cal_raw_x[0], cal_raw_x[1], cal_raw_x[2]});
        touch_x_max = max({cal_raw_x[0], cal_raw_x[1], cal_raw_x[2]});
        touch_y_min = min({cal_raw_y[0], cal_raw_y[1], cal_raw_y[2]});
        touch_y_max = max({cal_raw_y[0], cal_raw_y[1], cal_raw_y[2]});

        // Výpočet mierky (vzdialenosť medzi bodmi v kóde je 240 pre X a 160 pre Y)
        float x_scale = (float)(touch_x_max - touch_x_min) / 240.0f; 
        float y_scale = (float)(touch_y_max - touch_y_min) / 160.0f;

        // Extrapolácia na úplné okraje displeja (0-320 a 0-240)
        touch_x_min -= (int)(x_scale * 40);
        touch_x_max += (int)(x_scale * 40);
        touch_y_min -= (int)(y_scale * 40);
        touch_y_max += (int)(y_scale * 40);

        Serial.printf("Kalibrácia hotová: xmin=%d xmax=%d ymin=%d ymax=%d\n",
                      touch_x_min, touch_x_max, touch_y_min, touch_y_max);

        save_touch_cal();

        // BEZPEČNÉ vymazanie časovača
        if (cal_timer != nullptr) {
            lv_timer_del(cal_timer);
            cal_timer = nullptr; 
        }

        // Návrat na hlavnú obrazovku
        navigate_to_main();
        return;
    }

    // 5. Zobrazenie ďalšieho krížika
    char buf[48];
    snprintf(buf, sizeof(buf), "Tap point %d / %d", cal_step + 1, CAL_PTS);
    lv_label_set_text(lbl_cal_inst, buf);
    cal_draw_cross(cal_screen_x[cal_step], cal_screen_y[cal_step]);
}


static void build_cal_screen() {
        // Získajte aktuálne rozmery tak, ako ich vidí LVGL po rotácii
    int curr_w = lv_display_get_horizontal_resolution(lv_display_get_default());
    int curr_h = lv_display_get_vertical_resolution(lv_display_get_default());

    cal_screen_x[0] = 40;
    cal_screen_y[0] = 40;
    cal_screen_x[1] = curr_w - 40;
    cal_screen_y[1] = 40;
    cal_screen_x[2] = curr_w / 2;
    cal_screen_y[2] = curr_h - 40;

    if (scr_cal) { lv_obj_del(scr_cal); scr_cal = nullptr; }
    scr_cal = lv_obj_create(nullptr);
    lv_obj_set_size(scr_cal, curr_w, curr_h);
    lv_obj_set_style_bg_color(scr_cal, CLR_BG, 0);
    fix_obj(scr_cal);

    lv_obj_t *title = make_lbl(scr_cal, "TOUCH CALIBRATION",
                                &lv_font_montserrat_16, CLR_YELLOW);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 8);

    lbl_cal_inst = make_lbl(scr_cal, "Tap point 1 / 3",
                             &lv_font_montserrat_14, CLR_WHITE);
    lv_obj_align(lbl_cal_inst, LV_ALIGN_TOP_MID, 0, 32);

    // Crosshair — horizontal bar
    cal_cross_h = lv_obj_create(scr_cal);
    lv_obj_set_size(cal_cross_h, 30, 2);
    lv_obj_set_style_bg_color(cal_cross_h, CLR_RED, 0);
    lv_obj_set_style_border_width(cal_cross_h, 0, 0);

    // Crosshair — vertical bar
    cal_cross_v = lv_obj_create(scr_cal);
    lv_obj_set_size(cal_cross_v, 2, 30);
    lv_obj_set_style_bg_color(cal_cross_v, CLR_RED, 0);
    lv_obj_set_style_border_width(cal_cross_v, 0, 0);

    cal_draw_cross(cal_screen_x[0], cal_screen_y[0]);

    make_lbl(scr_cal, "Tap each crosshair precisely",
             &lv_font_montserrat_12, CLR_GREY);
    lv_obj_t *hint = lv_obj_get_child(scr_cal, -1);
    lv_obj_align(hint, LV_ALIGN_BOTTOM_MID, 0, -8);
}

// ============================================================================
// SETTINGS SCREEN
// ============================================================================

// ParamRows for motor cfg — reuse same widget as coil params
static ParamRow mrows[9];

static void on_settings_back(lv_event_t *e) {
    navigate_to_main();
}
static void on_motorparams_back(lv_event_t *e) {
    navigate_to_settings();
}
static void on_motor_save(lv_event_t *e) {
    save_cfg();
}
static void on_motor_reset(lv_event_t *e) {
    reset_cfg();
    // Refresh all motor param labels
    char buf[24];
    for (int i = 0; i < 9; i++) {
        if (!mrows[i].val_lbl) continue;
        if (mrows[i].f_ptr)
            snprintf(buf, sizeof(buf), "%.*f %s", mrows[i].dec, *mrows[i].f_ptr, mrows[i].unit);
        else
            snprintf(buf, sizeof(buf), "%d %s", *mrows[i].i_ptr, mrows[i].unit);
        lv_label_set_text(mrows[i].val_lbl, buf);
    }
}

static void on_motor_nudge_done(ParamRow *r) {
    save_cfg();
}

static void mnudge(ParamRow *r, int dir) {
    if (r->f_ptr) {
        float nv = constrain(*r->f_ptr + dir * r->step, r->mn, r->mx);
        *r->f_ptr = nv;
        char buf[24]; snprintf(buf, sizeof(buf), "%.*f %s", r->dec, nv, r->unit);
        lv_label_set_text(r->val_lbl, buf);
    } else {
        int nv = constrain(*r->i_ptr + dir * (int)r->step, (int)r->mn, (int)r->mx);
        *r->i_ptr = nv;
        char buf[24]; snprintf(buf, sizeof(buf), "%d %s", nv, r->unit);
        lv_label_set_text(r->val_lbl, buf);
    }
    // save_cfg() moved to on_mnudge_btn_released() to avoid blocking on every increment
}

static void mparam_hold_timer_cb(lv_timer_t *t) {
    if (held_mparam && held_mdir != 0) {
        mnudge(held_mparam, held_mdir);
    }
}

static void on_mnudge_btn_pressed(lv_event_t *e, ParamRow *r, int dir) {
    if (!r || dir == 0) return;
    
    mnudge(r, dir);
    held_mparam = r;
    held_mdir = dir;
    if (mparam_hold_timer) lv_timer_del(mparam_hold_timer);
    mparam_hold_timer = lv_timer_create(mparam_hold_timer_cb, 200, nullptr);
}

static void on_mnudge_btn_released(lv_event_t *e) {
    if (mparam_hold_timer) {
        lv_timer_del(mparam_hold_timer);
        mparam_hold_timer = nullptr;
    }
    // Don't save automatically - user must click SAVE button to persist to NVS
    held_mparam = nullptr;
    held_mdir = 0;
}

static void on_mnudge_minus(lv_event_t *e) {
    ParamRow *r = (ParamRow*)lv_event_get_user_data(e);
    lv_event_code_t code = lv_event_get_code(e);
    if (code == LV_EVENT_PRESSED) {
        on_mnudge_btn_pressed(e, r, -1);
    } else if (code == LV_EVENT_RELEASED || code == LV_EVENT_PRESS_LOST) {
        on_mnudge_btn_released(e);
    }
}
static void on_mnudge_plus(lv_event_t *e) {
    ParamRow *r = (ParamRow*)lv_event_get_user_data(e);
    lv_event_code_t code = lv_event_get_code(e);
    if (code == LV_EVENT_PRESSED) {
        on_mnudge_btn_pressed(e, r, +1);
    } else if (code == LV_EVENT_RELEASED || code == LV_EVENT_PRESS_LOST) {
        on_mnudge_btn_released(e);
    }
}

static lv_obj_t* add_motor_row(lv_obj_t *cont, const char *name, ParamRow *r) {
    lv_obj_t *row = lv_obj_create(cont);
    lv_obj_set_size(row, lv_pct(100), 32);
    lv_obj_set_style_bg_opa(row, LV_OPA_TRANSP, 0);
    lv_obj_clear_flag(row, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_border_width(row, 0, 0);
    lv_obj_set_style_shadow_width(row, 0, 0);
    lv_obj_set_style_pad_all(row, 2, 0);
    lv_obj_set_style_pad_column(row, 2, 0);
    lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(row, LV_FLEX_ALIGN_START,
                          LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

    lv_obj_t *nl = lv_label_create(row);
    lv_label_set_text(nl, name);
    lv_obj_set_width(nl, 90);
    lv_obj_set_style_text_font(nl, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_color(nl, CLR_WHITE, 0);

    lv_obj_t *btn_m = make_btn(row, "-", CLR_RED, on_mnudge_minus, r, 24, 24);
    lv_obj_add_event_cb(btn_m, on_mnudge_minus, LV_EVENT_PRESSED, r);
    lv_obj_add_event_cb(btn_m, on_mnudge_minus, LV_EVENT_RELEASED, r);

    lv_obj_t *vl = lv_label_create(row);
    lv_obj_set_flex_grow(vl, 1);
    lv_obj_set_style_text_font(vl, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_color(vl, CLR_YELLOW, 0);
    lv_obj_set_style_text_align(vl, LV_TEXT_ALIGN_CENTER, 0);
    char buf[24];
    if (r->f_ptr) snprintf(buf, sizeof(buf), "%.*f %s", r->dec, *r->f_ptr, r->unit);
    else          snprintf(buf, sizeof(buf), "%d %s",   *r->i_ptr, r->unit);
    lv_label_set_text(vl, buf);
    r->val_lbl = vl;

    lv_obj_t *btn_p = make_btn(row, "+", CLR_GREEN, on_mnudge_plus, r, 24, 24);
    lv_obj_add_event_cb(btn_p, on_mnudge_plus, LV_EVENT_PRESSED, r);
    lv_obj_add_event_cb(btn_p, on_mnudge_plus, LV_EVENT_RELEASED, r);
    return vl;
}

static void build_motorparams_screen() {
    if (scr_motorparams) { lv_obj_del(scr_motorparams); scr_motorparams = nullptr; }
    scr_motorparams = lv_obj_create(nullptr);
    lv_obj_set_size(scr_motorparams, SCREEN_W, SCREEN_H);
    fix_obj(scr_motorparams);
    lv_obj_set_style_bg_color(scr_motorparams, CLR_BG, 0);

    // Header with title, reset, save, back buttons
    lv_obj_t *hdr = lv_obj_create(scr_motorparams);
    lv_obj_set_size(hdr, SCREEN_W, 36);
    lv_obj_set_pos(hdr, 0, 0);
    fix_obj(hdr);
    lv_obj_set_style_bg_color(hdr, CLR_ACCENT, 0);
    
    make_lbl(hdr, "MOTOR PARAMS", &lv_font_montserrat_16, CLR_WHITE);
    lv_obj_align(lv_obj_get_child(hdr, -1), LV_ALIGN_LEFT_MID, 8, 0);

    lv_obj_t *brst = make_btn(hdr, LV_SYMBOL_TRASH, CLR_RED, on_motor_reset, nullptr, 30, 28);
    lv_obj_align(brst, LV_ALIGN_RIGHT_MID, -118, 0);
    
    lv_obj_t *bsv = make_btn(hdr, "SAVE", CLR_GREEN, on_motor_save, nullptr, 50, 28);
    lv_obj_align(bsv, LV_ALIGN_RIGHT_MID, -66, 0);
    
    lv_obj_t *bbk = make_btn(hdr, "BACK", CLR_GREY, on_motorparams_back, nullptr, 60, 28);
    lv_obj_align(bbk, LV_ALIGN_RIGHT_MID, -4, 0);

    // Simple row-based layout - stack rows vertically without scrolling
    int y_pos = 44;
    const int row_height = 28;  // Reduced from 32 to fit more rows
    
    // 8 parameters total: 6 motor + 2 diameter calibration
    const int max_rows = 7;
    
    // Mechanical calibration parameters
    mrows[0] = {nullptr, &cfg.gear_ratio_a, nullptr,     3.0f, 5.5f,   0.01f, 3, "x"};
    mrows[1] = {nullptr, &cfg.mm_per_rev_b, nullptr,     10.0f, 150.f, 1.0f, 1, "mm"};
    // Motor control parameters
    mrows[2] = {nullptr, nullptr, &cfg.speed_wind,       100, 20000, 100,  0, "s/s"};
    mrows[3] = {nullptr, nullptr, &cfg.speed_home,       50,  5000,  50,   0, "s/s"};
    mrows[4] = {nullptr, nullptr, &cfg.accel_steps,      0,   2000,  10,   0, "stp"};
    mrows[5] = {nullptr, &cfg.back_off_mm, nullptr,      0.1f, 20.f, 0.1f, 1, "mm"};
    // Diameter calibration parameters
    mrows[6] = {nullptr, &cfg.film_thick_mm, nullptr,    0.01f, 0.5f, 0.01f, 2, "mm"};

    // Create parameter rows
    for (int i = 0; i < max_rows && y_pos + row_height < SCREEN_H; i++) {
        lv_obj_t *row = lv_obj_create(scr_motorparams);
        lv_obj_set_size(row, SCREEN_W, row_height);
        lv_obj_set_pos(row, 0, y_pos);
        lv_obj_set_style_bg_color(row, CLR_PANEL, 0);
        fix_obj(row);
        lv_obj_set_style_pad_all(row, 2, 0);
        // Remove flex flow to avoid freezing issues
        
        // Parameter name label
        const char *names[] = {
            "Gear Ratio A",  // 55.5mm wheel / 13.39mm motor gear
            "mm/rev B",      // Linear axis: mm per rotation
            "Wind speed",    // Winding speed steps/sec
            "Home speed",    // Homing speed steps/sec
            "Accel steps",   // Acceleration ramp
            "Backoff mm",    // Backoff distance after endstop
            "Film thick"     // Kapton tape thickness for diameter calibration
        };
        lv_obj_t *lbl_name = make_lbl(row, names[i], &lv_font_montserrat_12, CLR_WHITE);
        lv_obj_set_pos(lbl_name, 4, 6);
        lv_obj_set_width(lbl_name, 80);

        // Minus button
        lv_obj_t *btn_m = make_btn(row, "-", CLR_RED, on_mnudge_minus, &mrows[i], 20, 20);
        lv_obj_set_pos(btn_m, 90, 4);
        lv_obj_add_event_cb(btn_m, on_mnudge_minus, LV_EVENT_PRESSED, &mrows[i]);
        lv_obj_add_event_cb(btn_m, on_mnudge_minus, LV_EVENT_RELEASED, &mrows[i]);

        // Value label
        lv_obj_t *lbl_val = lv_label_create(row);
        lv_obj_set_pos(lbl_val, 115, 6);
        lv_obj_set_width(lbl_val, 70);
        lv_obj_set_style_text_font(lbl_val, &lv_font_montserrat_12, 0);
        lv_obj_set_style_text_color(lbl_val, CLR_YELLOW, 0);
        lv_obj_set_style_text_align(lbl_val, LV_TEXT_ALIGN_CENTER, 0);
        
        char buf[24];
        if (mrows[i].f_ptr) 
            snprintf(buf, sizeof(buf), "%.*f %s", mrows[i].dec, *mrows[i].f_ptr, mrows[i].unit);
        else
            snprintf(buf, sizeof(buf), "%d %s", *mrows[i].i_ptr, mrows[i].unit);
        lv_label_set_text(lbl_val, buf);
        mrows[i].val_lbl = lbl_val;

        // Plus button
        lv_obj_t *btn_p = make_btn(row, "+", CLR_GREEN, on_mnudge_plus, &mrows[i], 20, 20);
        lv_obj_set_pos(btn_p, 190, 4);
        lv_obj_add_event_cb(btn_p, on_mnudge_plus, LV_EVENT_PRESSED, &mrows[i]);
        lv_obj_add_event_cb(btn_p, on_mnudge_plus, LV_EVENT_RELEASED, &mrows[i]);
        
        y_pos += row_height;
    }
}

static void on_open_motorparams(lv_event_t *e) {
    // Clean up any active timers before opening motor params
    if (mparam_hold_timer) {
        lv_timer_del(mparam_hold_timer);
        mparam_hold_timer = nullptr;
    }
    held_mparam = nullptr;
    held_mdir = 0;
    build_motorparams_screen();
    lv_scr_load(scr_motorparams);
}
static void on_open_touchcal(lv_event_t *e) {
    cal_step = 0;
    if (cal_timer) {
        lv_timer_del(cal_timer);
        cal_timer = nullptr;
    }
    build_cal_screen();
    lv_scr_load(scr_cal);
    cal_timer = lv_timer_create(cal_timer_cb, 50, nullptr);
}

static void build_settings_screen() {
    if (scr_settings) { lv_obj_del(scr_settings); scr_settings = nullptr; }
    scr_settings = lv_obj_create(nullptr); fix_obj(scr_settings);
    lv_obj_set_size(scr_settings, SCREEN_W, SCREEN_H);
    lv_obj_set_style_bg_color(scr_settings, CLR_BG, 0);

    // Header
    lv_obj_t *hdr = lv_obj_create(scr_settings); fix_obj(hdr);
    lv_obj_set_size(hdr, SCREEN_W, 36);
    lv_obj_set_pos(hdr, 0, 0);
    lv_obj_set_style_bg_color(hdr, CLR_ACCENT, 0);
    lv_obj_set_style_border_width(hdr, 0, 0);
    lv_obj_set_style_pad_all(hdr, 0, 0);
    make_lbl(hdr, "SETTINGS", &lv_font_montserrat_16, CLR_WHITE);
    lv_obj_align(lv_obj_get_child(hdr,-1), LV_ALIGN_LEFT_MID, 8, 0);
    lv_obj_t *bbk = make_btn(hdr, "BACK", CLR_GREY,
                              on_settings_back, nullptr, 60, 28);
    lv_obj_align(bbk, LV_ALIGN_RIGHT_MID, -4, 0);

    // 2x2 grid layout
    const int btn_w = (SCREEN_W - 30) / 2;  // ~145px each
    const int btn_h = 36;
    const int y0 = 50;
    const int y_step = 42;
    const int x_left = 10;
    const int x_right = SCREEN_W / 2 + 5;

    // Row 1, Left: Touch calibration button
    lv_obj_t *btc = make_btn(scr_settings, "TOUCH CAL",
                              lv_color_hex(0x0984e3),
                              on_open_touchcal, nullptr, btn_w, btn_h);
    lv_obj_set_pos(btc, x_left, y0);
    lv_obj_set_style_text_font(
        lv_obj_get_child(btc, 0), &lv_font_montserrat_14, 0);

    // Row 1, Right: Motor params button
    lv_obj_t *bmp = make_btn(scr_settings, "MOTOR PARAMS",
                              CLR_ORANGE,
                              on_open_motorparams, nullptr, btn_w, btn_h);
    lv_obj_set_pos(bmp, x_right, y0);
    lv_obj_set_style_text_font(
        lv_obj_get_child(bmp, 0), &lv_font_montserrat_14, 0);

    // Row 2, Left: Units toggle button
    lv_obj_t *bun = make_btn(scr_settings,
                              display_in_mh ? "UNITS: mH" : "UNITS: uH",
                              CLR_GREEN,
                              [](lv_event_t *e) {
                                  display_in_mh = !display_in_mh;
                                  lv_obj_t *b = (lv_obj_t*)lv_event_get_target(e);
                                  lv_label_set_text(lv_obj_get_child(b,0),
                                               display_in_mh ? "UNITS: mH" : "UNITS: uH");
                                  update_param_label(&prows[4]);
                                  save_prefs();
                              },
                              nullptr, btn_w, btn_h);
    lv_obj_set_pos(bun, x_left, y0 + y_step);
    lv_obj_set_style_text_font(
        lv_obj_get_child(bun, 0), &lv_font_montserrat_14, 0);

    // Row 2, Right: End sensors toggle button
    lv_obj_t *bes = make_btn(scr_settings,
                              end_sensors_enabled ? "SENSORS: ON" : "SENSORS: OFF",
                              end_sensors_enabled ? CLR_GREEN : CLR_ORANGE,
                              [](lv_event_t *e) {
                                  bool was_enabled = end_sensors_enabled;
                                  end_sensors_enabled = !end_sensors_enabled;
                                  
                                  // Attach/detach interrupts based on new state
                                  if (end_sensors_enabled && !was_enabled) {
                                      // Turning ON: attach interrupts
                                      attachInterrupt(digitalPinToInterrupt(ENDSTOP_A), isr_estop_a, CHANGE);
                                      attachInterrupt(digitalPinToInterrupt(ENDSTOP_B_MIN), isr_estop_b_min, CHANGE);
                                  } else if (!end_sensors_enabled && was_enabled) {
                                      // Turning OFF: detach interrupts
                                      detachInterrupt(digitalPinToInterrupt(ENDSTOP_A));
                                      detachInterrupt(digitalPinToInterrupt(ENDSTOP_B_MIN));
                                  }
                                  
                                  lv_obj_t *b = (lv_obj_t*)lv_event_get_target(e);
                                  lv_label_set_text(lv_obj_get_child(b,0),
                                                    end_sensors_enabled ? "SENSORS: ON" : "SENSORS: OFF");
                                  lv_obj_set_style_bg_color(
                                      b,
                                      end_sensors_enabled ? CLR_GREEN : CLR_ORANGE,
                                      0);
                                  save_prefs();
                              },
                              nullptr, btn_w, btn_h);
    lv_obj_set_pos(bes, x_right, y0 + y_step);
    lv_obj_set_style_text_font(
        lv_obj_get_child(bes, 0), &lv_font_montserrat_14, 0);

    // Row 3, Left: Diameter calibration toggle button
    lv_obj_t *bdc = make_btn(scr_settings,
                              cfg.diam_calib ? "DIAM CAL: ON" : "DIAM CAL: OFF",
                              cfg.diam_calib ? CLR_GREEN : CLR_ORANGE,
                              [](lv_event_t *e) {
                                  cfg.diam_calib = !cfg.diam_calib;
                                  lv_obj_t *b = (lv_obj_t*)lv_event_get_target(e);
                                  lv_label_set_text(lv_obj_get_child(b,0),
                                                    cfg.diam_calib ? "DIAM CAL: ON" : "DIAM CAL: OFF");
                                  lv_obj_set_style_bg_color(
                                      b,
                                      cfg.diam_calib ? CLR_GREEN : CLR_ORANGE,
                                      0);
                                  save_cfg();
                              },
                              nullptr, btn_w, btn_h);
    lv_obj_set_pos(bdc, x_left, y0 + 2 * y_step);
    lv_obj_set_style_text_font(
        lv_obj_get_child(bdc, 0), &lv_font_montserrat_14, 0);
}

static void on_calibrate(lv_event_t *e) {
    build_settings_screen();
    lv_scr_load(scr_settings);
}

static void on_mode(lv_event_t *e) {
    p.mode = lv_dropdown_get_selected((lv_obj_t*)lv_event_get_target(e));
    calculate_coil();
    update_result();
    save_prefs();
}

static void build_main_screen() {
    if (scr_main) lv_obj_del(scr_main);
    scr_main = lv_obj_create(nullptr); fix_obj(scr_main);
    lv_obj_set_style_bg_color(scr_main, CLR_BG, 0);
    lv_obj_set_style_pad_all(scr_main, 0, 0);
    lv_obj_set_style_border_width(scr_main, 0, 0);

    // ── ROW 1: Title bar (height 36px) ───────────────────────────────────────
    // Contains: "COIL WINDER" title | MANUAL button | START button
    lv_obj_t *hdr1 = lv_obj_create(scr_main); fix_obj(hdr1);
    lv_obj_set_size(hdr1, SCREEN_W, 36);
    lv_obj_set_pos(hdr1, 0, 0);
    lv_obj_set_style_bg_color(hdr1, CLR_ACCENT, 0);
    lv_obj_set_style_border_width(hdr1, 0, 0);
    lv_obj_set_style_pad_all(hdr1, 0, 0);

    lv_obj_t *title = make_lbl(hdr1, "COIL WINDER",
                                &lv_font_montserrat_12, CLR_WHITE);
    lv_obj_align(title, LV_ALIGN_LEFT_MID, 8, 0);

    lv_obj_t *bs = make_btn(hdr1, "HOME B", CLR_GREY,
                             on_home_b, nullptr, 60, 28);
    lv_obj_align(bs, LV_ALIGN_RIGHT_MID, -4, 0);

    lv_obj_t *bm = make_btn(hdr1, "HOME A", CLR_PURPLE,
                             on_home_a, nullptr, 60, 28);
    lv_obj_align(bm, LV_ALIGN_RIGHT_MID, -70, 0);

    lv_obj_t *bcal = make_btn(hdr1, LV_SYMBOL_EDIT, lv_color_hex(0x0984e3),
                               on_calibrate, nullptr, 30, 28);
    lv_obj_align(bcal, LV_ALIGN_RIGHT_MID, -136, 0);

    // Bin/reset button
    lv_obj_t *brst = make_btn(hdr1, LV_SYMBOL_TRASH, CLR_RED,
                               on_reset, nullptr, 30, 28);
    lv_obj_align(brst, LV_ALIGN_RIGHT_MID, -172, 0);

    // ── ROW 2: MANUAL + START — most used, prominent position
    // Contains: MANUAL | START  — each half width
    lv_obj_t *hdr2 = lv_obj_create(scr_main); fix_obj(hdr2);
    lv_obj_set_size(hdr2, SCREEN_W, 30);
    lv_obj_set_pos(hdr2, 0, 36);
    lv_obj_set_style_bg_color(hdr2, lv_color_hex(0x0A0A1A), 0);
    lv_obj_set_style_border_width(hdr2, 0, 0);
    lv_obj_set_style_pad_all(hdr2, 0, 0);

    lv_obj_t *bha = make_btn(hdr2, "MANUAL", CLR_ORANGE,
                              on_manual, nullptr, 148, 24);
    lv_obj_align(bha, LV_ALIGN_LEFT_MID, 4, 0);

    lv_obj_t *bhb = make_btn(hdr2, "START", CLR_GREEN,
                              on_start, nullptr, 148, 24);
    lv_obj_align(bhb, LV_ALIGN_RIGHT_MID, -4, 0);

    // ── Result bar at bottom (height 32px) ───────────────────────────────────
    lv_obj_t *rb = lv_obj_create(scr_main); fix_obj(rb);
    lv_obj_set_size(rb, SCREEN_W, 32);
    lv_obj_set_pos(rb, 0, 208);
    lv_obj_set_style_bg_color(rb, CLR_ACCENT, 0);
    lv_obj_set_style_border_width(rb, 0, 0);
    lv_obj_set_style_shadow_width(rb, 0, 0);
    lv_obj_set_style_outline_width(rb, 0, 0);
    lv_obj_set_style_pad_hor(rb, 8, 0);
    lbl_result = make_lbl(rb, "Predicted: --",
                          &lv_font_montserrat_12, CLR_YELLOW);
    lv_obj_align(lbl_result, LV_ALIGN_LEFT_MID, 0, 0);

    // ── Param columns: top=66, height=142 (fits between row2 and result bar) ─
    // Left column
    lv_obj_t *cl = lv_obj_create(scr_main); fix_obj(cl);
    lv_obj_set_pos(cl, 0, 66);
    lv_obj_set_size(cl, 160, 142);
    lv_obj_set_style_bg_color(cl, CLR_PANEL, 0);
    lv_obj_set_style_border_width(cl, 0, 0);
    lv_obj_set_style_shadow_width(cl, 0, 0);
    lv_obj_set_style_outline_width(cl, 0, 0);
    lv_obj_set_style_pad_all(cl, 4, 0);
    lv_obj_set_style_pad_row(cl, 2, 0);
    lv_obj_set_flex_flow(cl, LV_FLEX_FLOW_COLUMN);

    // Right column
    lv_obj_t *cr = lv_obj_create(scr_main); fix_obj(cr);
    lv_obj_set_pos(cr, 162, 66);
    lv_obj_set_size(cr, 158, 142);
    lv_obj_set_style_bg_color(cr, CLR_PANEL, 0);
    lv_obj_set_style_border_width(cr, 0, 0);
    lv_obj_set_style_shadow_width(cr, 0, 0);
    lv_obj_set_style_outline_width(cr, 0, 0);
    lv_obj_set_style_pad_all(cr, 4, 0);
    lv_obj_set_style_pad_row(cr, 2, 0);
    lv_obj_set_flex_flow(cr, LV_FLEX_FLOW_COLUMN);

    // Divider
    lv_obj_t *dv = lv_obj_create(scr_main); fix_obj(dv);
    lv_obj_set_pos(dv, 160, 66);
    lv_obj_set_size(dv, 2, 142);
    lv_obj_set_style_bg_color(dv, CLR_ACCENT, 0);
    lv_obj_set_style_border_width(dv, 0, 0);

    // Left column params
    prows[0] = {nullptr, &p.wire_dia_mm,   nullptr, 0.05f, 2.00f, 0.01f, 2, "mm"};
    prows[1] = {nullptr, &p.coil_width_mm, nullptr, 1.0f,  100.f, 0.5f,  1, "mm"};
    prows[2] = {nullptr, &p.r_inner_mm,    nullptr, 1.0f,  100.f, 0.5f,  1, "mm"};
    add_param_row(cl, "Wire d",  &prows[0]);
    add_param_row(cl, "Width",   &prows[1]);
    add_param_row(cl, "Inner r", &prows[2]);

    // Memory slots row — M1 M2 M3 M4
    // Short tap = load, long press = save
    lv_obj_t *mslot_row = lv_obj_create(cl); fix_obj(mslot_row);
    lv_obj_set_size(mslot_row, lv_pct(100), 34);
    lv_obj_set_style_bg_opa(mslot_row, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(mslot_row, 0, 0);
    lv_obj_set_style_pad_all(mslot_row, 2, 0);
    lv_obj_set_style_pad_column(mslot_row, 3, 0);
    lv_obj_set_flex_flow(mslot_row, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(mslot_row, LV_FLEX_ALIGN_SPACE_EVENLY,
                          LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

    const char *slot_names[] = {"M1","M2","M3","M4"};
    for (int i = 0; i < 4; i++) {
        lv_obj_t *sb = lv_btn_create(mslot_row);
        lv_obj_set_size(sb, 34, 28);
        lv_obj_set_style_bg_color(sb,
            slot_used(i) ? CLR_GREEN : CLR_GREY, 0);
        lv_obj_set_style_radius(sb, 5, 0);
        lv_obj_set_style_border_width(sb, 0, 0);
        lv_obj_set_style_pad_all(sb, 0, 0);
        lv_obj_add_event_cb(sb, on_slot_short, LV_EVENT_SHORT_CLICKED,
                            (void*)(intptr_t)i);
        lv_obj_add_event_cb(sb, on_slot_long,  LV_EVENT_LONG_PRESSED,
                            (void*)(intptr_t)i);
        lv_obj_t *sl = lv_label_create(sb);
        lv_label_set_text(sl, slot_names[i]);
        lv_obj_set_style_text_font(sl, &lv_font_montserrat_12, 0);
        lv_obj_set_style_text_color(sl, CLR_WHITE, 0);
        lv_obj_center(sl);
        slot_btns[i] = sb;
    }

    // Right column: mode dropdown
    lv_obj_t *mrow = lv_obj_create(cr); fix_obj(mrow);
    lv_obj_set_size(mrow, lv_pct(100), 32);
    lv_obj_set_style_bg_opa(mrow, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(mrow, 0, 0);
    lv_obj_set_style_shadow_width(mrow, 0, 0);
    lv_obj_set_style_outline_width(mrow, 0, 0);
    lv_obj_set_style_pad_all(mrow, 0, 0);
    lv_obj_set_flex_flow(mrow, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(mrow, LV_FLEX_ALIGN_START,
                          LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

    lv_obj_t *ml = make_lbl(mrow, "Mode", &lv_font_montserrat_12, CLR_WHITE);
    lv_obj_set_width(ml, 38);

    lv_obj_t *dd = lv_dropdown_create(mrow);
    lv_dropdown_set_options(dd, "Turns\nAir L\nCore L");
    lv_dropdown_set_selected(dd, p.mode);
    lv_obj_set_width(dd, 108);
    lv_obj_set_style_bg_color(dd, CLR_ACCENT, 0);
    lv_obj_set_style_text_font(dd, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_color(dd, CLR_WHITE, 0);
    lv_obj_set_style_border_width(dd, 0, 0);
    lv_obj_set_style_shadow_width(dd, 0, 0);
    lv_obj_set_style_outline_width(dd, 0, 0);
    lv_obj_add_event_cb(dd, on_mode, LV_EVENT_VALUE_CHANGED, nullptr);

    // Right column params
    prows[3] = {nullptr, nullptr, &p.target_turns, 1,    9999,  1,    0, "T"};
    prows[4] = {nullptr, &p.target_uh,  nullptr,   1.0f, 99999, 1.0f, 0, "uH"};
    prows[5] = {nullptr, &p.core_ur,    nullptr,   1.0f, 9999,  1.0f, 0, "ur"};
    add_param_row(cr, "Turns",   &prows[3]);
    add_param_row(cr, "uH tgt",  &prows[4]);
    add_param_row(cr, "Core ur", &prows[5]);

    calculate_coil();
    update_result();
}

// ============================================================================
// SETUP
// ============================================================================
void setup() {
    Serial.begin(115200);
    Serial.println("Coil Winder v3.2");

    // Stepper outputs
    int out_pins[] = {STEP_A, DIR_A, EN_A, STEP_B, DIR_B};
    for (int pin : out_pins) pinMode(pin, OUTPUT);
    enable_motors(false);

    // Load preferences FIRST to know if sensors are enabled
    load_prefs();

    // Endstops — external 4.7k pull-up to 3.3V, use INPUT not INPUT_PULLUP
    pinMode(ENDSTOP_A, INPUT);
    pinMode(ENDSTOP_B_MIN, INPUT);

    // Only attach interrupts if sensors are enabled
    if (end_sensors_enabled) {
        attachInterrupt(digitalPinToInterrupt(ENDSTOP_A), isr_estop_a, CHANGE);
        attachInterrupt(digitalPinToInterrupt(ENDSTOP_B_MIN), isr_estop_b_min, CHANGE);
    }

    estop_a = digitalRead(ENDSTOP_A) == LOW;
    estop_b_min = digitalRead(ENDSTOP_B_MIN) == LOW;

    // Stepper task on Core 0
    xTaskCreatePinnedToCore(stepperTask, "steppers", 8192,
                            nullptr, 2, nullptr, 0);

    // Touch init FIRST before lv_tft_espi_create — matches working example
    touchSPI.begin(XPT2046_CLK, XPT2046_MISO, XPT2046_MOSI, XPT2046_CS);
    touch.begin(touchSPI);
    touch.setRotation(3);

    // LVGL + display
    lv_init();
    tft.initDMA(); 
    //static uint8_t draw_buf[SCREEN_W * SCREEN_H / 10 * 2];
    // NOTE: lv_tft_espi_create expects the physical panel dimensions.
    //       the rotation applied afterward selects the working orientation.
    //       change LV_ROT at the top of the file to flip between portrait
    //       and landscape without touching the rest of the code.
    lv_display_t *disp = lv_tft_espi_create(PHYS_W, PHYS_H, draw_buf_1, sizeof(draw_buf_1));
    lv_display_set_rotation(disp, LV_ROT);
    lv_display_set_buffers(disp, draw_buf_1, draw_buf_2, sizeof(draw_buf_1), LV_DISPLAY_RENDER_MODE_PARTIAL);

    pinMode(TFT_BL, OUTPUT);
    digitalWrite(TFT_BL, HIGH);

    lv_indev_t *indev = lv_indev_create();
    lv_indev_set_type(indev, LV_INDEV_TYPE_POINTER);
    lv_indev_set_read_cb(indev, lvgl_touch_read);

    load_cfg();
    load_touch_cal();
    build_main_screen();
    build_winding_screen();

    if (!touch_cal_exists()) {
        // First boot — run calibration before showing main screen
        // cal_timer_cb will navigate to scr_main when done
        cal_step = 0;
        build_cal_screen();
        lv_scr_load(scr_cal);
        cal_timer = lv_timer_create(cal_timer_cb, 50, nullptr);
    } else {
        lv_scr_load(scr_main);
    }

    Serial.printf("Ready. Steps/rev=%d\n",
                  cfg.steps_rev_a);
}

// ============================================================================
// LOOP
// ============================================================================
void loop() {
    lv_task_handler();
    lv_tick_inc(5);
    delay(5);
}
