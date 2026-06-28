// ================================================================
//  CURIO_AutoTune_Dual_PID.ino
//  雙環串接 (Angle外環+Rate內環) 自調校飛行控制韌體
//  硬體平台：CURIO (RP2350 / RP2040)，火箭鳥創客倉庫
//  ELRS/CRSF 接收機處理已拆分至 elrs.h / elrs.cpp
// ================================================================

#include <Arduino.h>
#include <Wire.h>
#include "BMI088.h"
#include "elrs.h"

// ================================================================
// 📑 自訂型態與結構定義 (提前移至最上方，防編譯器解析失敗)
// ================================================================
enum AxisMode { 
    AXIS_NORMAL, 
    AXIS_TUNE_RATE, 
    AXIS_TUNE_ANGLE 
};

struct RelayTuneState {
    int8_t relay_dir         = 1;
    unsigned long start_time       = 0;
    unsigned long last_switch_time = 0;
    int    cycle_count       = 0;
    int    valid_cycles      = 0;
    float  cycle_min         = 0.0f;
    float  cycle_max         = 0.0f;
    float  sum_period_s      = 0.0f;
    float  sum_amplitude     = 0.0f;
    unsigned long timeout_ms = 10000;
    bool   done              = false;
    bool   timed_out         = false;
};

struct AxisState {
    float target_angle = 0.0f;
    float pid_p_angle = 4.0f, pid_i_angle = 0.0f, pid_d_angle = 0.0f;
    float err_prev_angle = 0.0f, int_angle = 0.0f;

    float target_rate = 0.0f;
    float pid_p_rate = 0.8f, pid_i_rate = 0.02f, pid_d_rate = 0.04f;
    float err_prev_rate = 0.0f, int_rate = 0.0f;
    AxisMode mode = AXIS_NORMAL;
    RelayTuneState rt_rate;
    RelayTuneState rt_angle;
};

enum TuningRule { 
    ZN_CLASSIC_PID, 
    ZN_PESSEN_INTEGRAL, 
    ZN_SOME_OVERSHOOT, 
    ZN_NO_OVERSHOOT 
};

enum SystemMode {
    MODE_NORMAL = 0,
    MODE_AUTOTUNE_RATE_ROLL,
    MODE_AUTOTUNE_RATE_PITCH,
    MODE_AUTOTUNE_ANGLE_ROLL,
    MODE_AUTOTUNE_ANGLE_PITCH,
    MODE_EMERGENCY_DESCEND,
};

enum ChainMode { 
    CHAIN_NONE, 
    CHAIN_RATE_ONLY, 
    CHAIN_FULL, 
    CHAIN_ANGLE_ONLY 
};

// ================================================================
// 🛠️ 手動函式原型宣告 (Function Prototypes)
// ================================================================
void resetRelayState(RelayTuneState &rt, float current_value, unsigned long timeout_ms);
float relayStep(RelayTuneState &rt, float measured_value, float target_value, unsigned long now_ms, float relay_amplitude, float hysteresis);
void computeGainsFromRelay(float Ku, float Pu, float &Kp, float &Ki, float &Kd);
void printAutotuneResult(const char* loop_name, float Ku, float Pu, float Kp, float Ki, float Kd);
void finalizeAxisTuning(RelayTuneState &rt, const char* loop_name, float relay_amplitude, float &out_p, float &out_i, float &out_d);
void resetAxisIntegrators(AxisState &axis);
float computeAxisOutput(AxisState &axis, float measured_angle, float measured_rate, bool run_outer, float outer_dt, float inner_dt, unsigned long now_ms);
bool autotunePreconditionsOk(unsigned long roll_us, unsigned long pitch_us, unsigned long throttle_us);
void advanceAutotuneChain(SystemMode finished_phase);
void startFullAutotune();
void startRateOnlyAutotune();
void startAngleOnlyAutotune();
void abortAutotune(const char* reason);
const char* modeName(SystemMode m);

// ==========================================
// 🛠️ 硬體接腳與參數定義
// ==========================================
const int M1_PIN = 0;   // 右前 (CW)
const int M2_PIN = 11;  // 右後 (CCW)
const int M3_PIN = 14;  // 左後 (CW)
const int M4_PIN = 28;  // 左前 (CCW)

const int LED_A = 7;    // 解鎖狀態 (常亮=已解鎖)
const int LED_B = 8;    // 狀態指示燈 (藍)
const int LED_C = 9;    // 狀態指示燈 (紅)

const int I2C0_SDA = 20;
const int I2C0_SCL = 25;
#define BMI088_ACC_ADDR  0x18
#define BMI088_GYRO_ADDR 0x69

const int PWM_FREQ = 20000;
const int PWM_RANGE = 255;
const int MOTOR_MAX_LIMIT = 216;   // 85% 物理電流防護限制
const int MOTOR_IDLE_PWM  = 15;    // 解鎖時最低馬達轉速

const int THROTTLE_HEADROOM_RESERVE = 36;
const int THROTTLE_MAX_MAPPED = MOTOR_MAX_LIMIT - THROTTLE_HEADROOM_RESERVE; // = 180

// ==========================================
// 🕹️ 飛行安全參數
// ==========================================
const unsigned long RC_US_MIN = 1000, RC_US_MID = 1500, RC_US_MAX = 2000;
const unsigned long ARM_SWITCH_THRESHOLD_US = 1500;
const unsigned long ARM_THROTTLE_MAX_US     = 1100;
const unsigned long FAILSAFE_TIMEOUT_MS     = 300;   

const float SEVERE_TILT_CUTOFF_DEG = 75.0f;

const float MAX_STICK_ANGLE    = 30.0f;  
const float MAX_STICK_YAWRATE  = 180.0f;
const unsigned long STICK_DEADBAND_US = 15;

const unsigned long AUTOTUNE_SWITCH_THRESHOLD_US = 1500;
const unsigned long AUTOTUNE_STICK_DEADBAND_US   = 40;
const unsigned long AUTOTUNE_MIN_THROTTLE_US     = 1400;
const unsigned long AUTOTUNE_MAX_THROTTLE_US     = 1750;

const unsigned long EMERGENCY_SWITCH_THRESHOLD_US = 1500;
const float EMERGENCY_DESCENT_RATE_PWM_PER_S = 35.0f; 
const int   EMERGENCY_LAND_PWM_THRESHOLD = MOTOR_IDLE_PWM + 5;
const unsigned long EMERGENCY_LAND_CONFIRM_MS = 1000; 

// ==========================================
// 📐 姿態估計 (BMI088 + Mahony Filter)
// ==========================================
BMI088 bmi088(Wire, BMI088_ACC_ADDR, BMI088_GYRO_ADDR);
float ax = 0, ay = 0, az = 0;
float gx = 0, gy = 0, gz = 0;
float ax_offset = 0, ay_offset = 0, az_offset = 0;
float gx_offset = 0, gy_offset = 0, gz_offset = 0;
#define kp_mahony 2.0f
#define ki_mahony 0.005f
float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
float eIntX = 0.0f, eIntY = 0.0f, eIntZ = 0.0f;
unsigned long lastUpdate = 0;
float roll = 0.0f, pitch = 0.0f;

float target_yaw_rate = 0.0f; 
float pid_p_yaw = 1.5f;
unsigned long last_pid_time = 0;
bool is_armed = false;

// ==========================================
// 🎛️ 雙環串接 PID 變數宣告
// ==========================================
const int ANGLE_LOOP_DIVIDER = 5;
const float MAX_TARGET_RATE  = 250.0f; 

int angle_loop_counter = 0;
unsigned long last_angle_loop_time = 0;

AxisState roll_axis;
AxisState pitch_axis;

const int   AUTOTUNE_SKIP_CYCLES = 2;
const int   AUTOTUNE_AVG_CYCLES  = 6;
const unsigned long AUTOTUNE_TIMEOUT_RATE_MS  = 10000;
const unsigned long AUTOTUNE_TIMEOUT_ANGLE_MS = 25000;

const float RELAY_AMPLITUDE_RATE   = 35.0f;
const float RELAY_HYSTERESIS_RATE  = 3.0f;   

const float RELAY_AMPLITUDE_ANGLE  = 15.0f;
const float RELAY_HYSTERESIS_ANGLE = 0.6f;   

const TuningRule AUTOTUNE_RULE = ZN_NO_OVERSHOOT; 

SystemMode current_mode = MODE_NORMAL;
ChainMode autotune_chain_mode = CHAIN_NONE;
bool autotune_switch_prev = false;
bool autotune_request_pending = false;
unsigned long tune_done_flash_until = 0;

float emergency_descend_base = 0.0f;
unsigned long emergency_descend_start_ms = 0;
unsigned long emergency_low_throttle_since_ms = 0;
int last_motor_base = 0; 

// ==========================================
// 🕒 姿態估計核心函數
// ==========================================
void calibrateSensors() {
    digitalWrite(LED_C, HIGH);
    Serial.println("正在校正感測器，請保持機體靜止水平...");
    long samples = 500;
    for (int i = 0; i < samples; i++) {
        bmi088.getAcceleration(&ax, &ay, &az);
        bmi088.getGyroscope(&gx, &gy, &gz);
        ax_offset += ax; ay_offset += ay; az_offset += (az - 9.80665f);
        gx_offset += gx; gy_offset += gy;
        gz_offset += gz;
        delay(4);
    }
    ax_offset /= samples; ay_offset /= samples; az_offset /= samples;
    gx_offset /= samples; gy_offset /= samples; gz_offset /= samples;
    digitalWrite(LED_C, LOW);
    Serial.println("✅ 校正完成！");
}

void updateMahony(float ax, float ay, float az, float gx, float gy, float gz, float dt) {
    float recipNorm;
    float halfvx, halfvy, halfvz;
    float halfex, halfey, halfez;

    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
        recipNorm = 1.0f / sqrtf(ax * ax + ay * ay + az * az);
        ax *= recipNorm; ay *= recipNorm; az *= recipNorm;

        halfvx = q1 * q3 - q0 * q2;
        halfvy = q0 * q1 + q2 * q3;
        halfvz = q0 * q0 - 0.5f + q3 * q3;
        halfex = (ay * halfvz - az * halfvy);
        halfey = (az * halfvx - ax * halfvz);
        halfez = (ax * halfvy - ay * halfvx);

        if (ki_mahony > 0.0f) {
            eIntX += halfex * ki_mahony * dt;
            eIntY += halfey * ki_mahony * dt;
            eIntZ += halfez * ki_mahony * dt;
            gx += eIntX; gy += eIntY;
            gz += eIntZ;
        }
        gx += halfex * kp_mahony;
        gy += halfey * kp_mahony;
        gz += halfez * kp_mahony;
    }

    gx *= (0.5f * dt);
    gy *= (0.5f * dt); gz *= (0.5f * dt);
    float qa = q0, qb = q1, qc = q2;
    q0 += (-qb * gx - qc * gy - q3 * gz);
    q1 += (qa * gx + qc * gz - q3 * gy);
    q2 += (qa * gy - qb * gz + q3 * gx);
    q3 += (qa * gz + qb * gy - qc * gx);
    recipNorm = 1.0f / sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm; q1 *= recipNorm; q2 *= recipNorm; q3 *= recipNorm;
    roll  = atan2f(q0*q1 + q2*q3, 0.5f - q1*q1 - q2*q2) * 57.29578f;
    pitch = asinf(-2.0f * (q1*q3 - q0*q2)) * 57.29578f;
}

// ==========================================
// 🕹️ 搖桿 / 油門 映射函數
// ==========================================
float stickToTarget(unsigned long us, float max_value) {
    long centered = (long)us - (long)RC_US_MID;
    if (abs(centered) < (long)STICK_DEADBAND_US) centered = 0;
    float normalized = constrain((float)centered / 500.0f, -1.0f, 1.0f);
    return normalized * max_value;
}

int throttleToBase(unsigned long us) {
    long mapped = map((long)us, (long)RC_US_MIN, (long)RC_US_MAX, 0, THROTTLE_MAX_MAPPED);
    return (int)constrain(mapped, 0L, (long)THROTTLE_MAX_MAPPED);
}

// ------------------------------------------
// 繼電器回授自動調校 — 核心控制實作
// ------------------------------------------
void resetRelayState(RelayTuneState &rt, float current_value, unsigned long timeout_ms) {
    rt.relay_dir = 1;
    rt.start_time = millis();
    rt.last_switch_time = 0;
    rt.cycle_count = 0;
    rt.valid_cycles = 0;
    rt.cycle_min = current_value;
    rt.cycle_max = current_value;
    rt.sum_period_s = 0.0f;
    rt.sum_amplitude = 0.0f;
    rt.timeout_ms = timeout_ms;
    rt.done = false;
    rt.timed_out = false;
}

float relayStep(RelayTuneState &rt, float measured_value, float target_value, unsigned long now_ms,
                 float relay_amplitude, float hysteresis) {
    float error = target_value - measured_value;
    if (measured_value > rt.cycle_max) rt.cycle_max = measured_value;
    if (measured_value < rt.cycle_min) rt.cycle_min = measured_value;
    if (rt.relay_dir < 0 && error > hysteresis) {
        rt.relay_dir = +1;
        if (rt.last_switch_time != 0) {
            float period_s = (now_ms - rt.last_switch_time) / 1000.0f;
            rt.cycle_count++;
            if (rt.cycle_count > AUTOTUNE_SKIP_CYCLES) {
                float amplitude = (rt.cycle_max - rt.cycle_min) / 2.0f;
                rt.sum_period_s  += period_s;
                rt.sum_amplitude += amplitude;
                rt.valid_cycles++;
            }
        }
        rt.last_switch_time = now_ms;
        rt.cycle_min = measured_value;
        rt.cycle_max = measured_value;
    } else if (rt.relay_dir > 0 && error < -hysteresis) {
        rt.relay_dir = -1;
    }

    if (rt.valid_cycles >= AUTOTUNE_AVG_CYCLES) rt.done = true;
    if (now_ms - rt.start_time > rt.timeout_ms) rt.timed_out = true;
    return rt.relay_dir * relay_amplitude;
}

void computeGainsFromRelay(float Ku, float Pu, float &Kp, float &Ki, float &Kd) {
    float Ti = 0, Td = 0;
    switch (AUTOTUNE_RULE) {
        case ZN_CLASSIC_PID:
            Kp = 0.60f * Ku;
            Ti = 0.50f * Pu;  Td = 0.125f * Pu;
            break;
        case ZN_PESSEN_INTEGRAL:
            Kp = 0.70f * Ku;
            Ti = 0.40f * Pu;  Td = 0.150f * Pu;
            break;
        case ZN_SOME_OVERSHOOT:
            Kp = 0.33f * Ku;
            Ti = 0.50f * Pu;  Td = 0.333f * Pu;
            break;
        case ZN_NO_OVERSHOOT:
        default:
            Kp = 0.20f * Ku;
            Ti = 0.50f * Pu;  Td = 0.333f * Pu;
            break;
    }
    Ki = Kp / Ti;
    Kd = Kp * Td;
}

void printAutotuneResult(const char* loop_name, float Ku, float Pu, float Kp, float Ki, float Kd) {
    Serial.println("==========================================");
    Serial.print("✅ ["); Serial.print(loop_name); Serial.println("] 自動調校完成");
    Serial.print("  Ku="); Serial.print(Ku, 4);
    Serial.print(" Pu="); Serial.print(Pu, 4); Serial.println("s");
    Serial.print("  -> Kp=");
    Serial.print(Kp, 4);
    Serial.print(" Ki="); Serial.print(Ki, 4);
    Serial.print(" Kd="); Serial.println(Kd, 4);
    Serial.println("==========================================");
}

void finalizeAxisTuning(RelayTuneState &rt, const char* loop_name, float relay_amplitude,
                         float &out_p, float &out_i, float &out_d) {
    float Pu = rt.sum_period_s / rt.valid_cycles;
    float a  = rt.sum_amplitude / rt.valid_cycles;

    if (a < 0.1f) {
        Serial.println("⚠️ 振盪振幅過小，調校結果可能不可靠！");
        a = 0.1f;
    }

    float Ku = (4.0f * relay_amplitude) / (PI * a);
    float Kp, Ki, Kd;
    computeGainsFromRelay(Ku, Pu, Kp, Ki, Kd);
    printAutotuneResult(loop_name, Ku, Pu, Kp, Ki, Kd);

    out_p = Kp;
    out_i = Ki; out_d = Kd;
}

void resetAxisIntegrators(AxisState &axis) {
    axis.int_rate = 0.0f;  axis.err_prev_rate = 0.0f;
    axis.int_angle = 0.0f; axis.err_prev_angle = 0.0f;
}

float computeAxisOutput(AxisState &axis, float measured_angle, float measured_rate,
                         bool run_outer, float outer_dt, float inner_dt, unsigned long now_ms) {

    if (axis.mode == AXIS_TUNE_RATE) {
        return relayStep(axis.rt_rate, measured_rate, 0.0f, now_ms,
                          RELAY_AMPLITUDE_RATE, RELAY_HYSTERESIS_RATE);
    }

    if (axis.mode == AXIS_TUNE_ANGLE) {
        if (run_outer) {
            axis.target_rate = relayStep(axis.rt_angle, measured_angle, axis.target_angle, now_ms,
                                          RELAY_AMPLITUDE_ANGLE, RELAY_HYSTERESIS_ANGLE);
        }
        float err_rate = axis.target_rate - measured_rate;
        axis.int_rate += err_rate * inner_dt;
        axis.int_rate = constrain(axis.int_rate, -50.0f, 50.0f);
        float deriv_rate = (err_rate - axis.err_prev_rate) / inner_dt;
        float output = (axis.pid_p_rate * err_rate) + (axis.pid_i_rate * axis.int_rate) + (axis.pid_d_rate * deriv_rate);
        axis.err_prev_rate = err_rate;
        return output;
    }

    if (run_outer) {
        float err_angle = axis.target_angle - measured_angle;
        axis.int_angle += err_angle * outer_dt;
        axis.int_angle = constrain(axis.int_angle, -50.0f, 50.0f);
        float deriv_angle = (err_angle - axis.err_prev_angle) / outer_dt;
        float new_target_rate = (axis.pid_p_angle * err_angle) + (axis.pid_i_angle * axis.int_angle) + (axis.pid_d_angle * deriv_angle);
        axis.err_prev_angle = err_angle;
        axis.target_rate = constrain(new_target_rate, -MAX_TARGET_RATE, MAX_TARGET_RATE);
    }

    float err_rate = axis.target_rate - measured_rate;
    axis.int_rate += err_rate * inner_dt;
    axis.int_rate = constrain(axis.int_rate, -50.0f, 50.0f);
    float deriv_rate = (err_rate - axis.err_prev_rate) / inner_dt;
    float output = (axis.pid_p_rate * err_rate) + (axis.pid_i_rate * axis.int_rate) + (axis.pid_d_rate * deriv_rate);
    axis.err_prev_rate = err_rate;
    return output;
}

bool autotunePreconditionsOk(unsigned long roll_us, unsigned long pitch_us, unsigned long throttle_us) {
    bool sticksCentered = (abs((long)roll_us  - (long)RC_US_MID) < (long)AUTOTUNE_STICK_DEADBAND_US) &&
                          (abs((long)pitch_us - (long)RC_US_MID) < (long)AUTOTUNE_STICK_DEADBAND_US);
    bool throttleOk = (throttle_us > AUTOTUNE_MIN_THROTTLE_US) && (throttle_us < AUTOTUNE_MAX_THROTTLE_US);
    return sticksCentered && throttleOk;
}

void advanceAutotuneChain(SystemMode finished_phase) {
    switch (finished_phase) {
        case MODE_AUTOTUNE_RATE_ROLL:
            roll_axis.mode = AXIS_NORMAL;
            resetAxisIntegrators(roll_axis);
            Serial.println("Roll Rate 完成 -> Pitch Rate");
            pitch_axis.mode = AXIS_TUNE_RATE;
            resetRelayState(pitch_axis.rt_rate, gy, AUTOTUNE_TIMEOUT_RATE_MS);
            current_mode = MODE_AUTOTUNE_RATE_PITCH;
            break;
        case MODE_AUTOTUNE_RATE_PITCH:
            pitch_axis.mode = AXIS_NORMAL;
            resetAxisIntegrators(pitch_axis);
            if (autotune_chain_mode == CHAIN_FULL) {
                Serial.println("Rate雙軸完成 -> Angle Roll");
                roll_axis.mode = AXIS_TUNE_ANGLE;
                resetRelayState(roll_axis.rt_angle, roll, AUTOTUNE_TIMEOUT_ANGLE_MS);
                current_mode = MODE_AUTOTUNE_ANGLE_ROLL;
            } else {
                Serial.println("Rate Loop 雙軸調校完成");
                current_mode = MODE_NORMAL;
                tune_done_flash_until = millis() + 3000;
            }
            break;
        case MODE_AUTOTUNE_ANGLE_ROLL:
            roll_axis.mode = AXIS_NORMAL;
            resetAxisIntegrators(roll_axis);
            Serial.println("Angle Roll 完成 -> Angle Pitch");
            pitch_axis.mode = AXIS_TUNE_ANGLE;
            resetRelayState(pitch_axis.rt_angle, pitch, AUTOTUNE_TIMEOUT_ANGLE_MS);
            current_mode = MODE_AUTOTUNE_ANGLE_PITCH;
            break;
        case MODE_AUTOTUNE_ANGLE_PITCH:
            pitch_axis.mode = AXIS_NORMAL;
            resetAxisIntegrators(pitch_axis);
            Serial.println("================ 自動調校結束，已套用 ================");
            Serial.print("Rate  Roll : Kp="); Serial.print(roll_axis.pid_p_rate, 4);
            Serial.print(" Ki="); Serial.print(roll_axis.pid_i_rate, 4);
            Serial.print(" Kd="); Serial.println(roll_axis.pid_d_rate, 4);
            Serial.print("Rate  Pitch: Kp=");
            Serial.print(pitch_axis.pid_p_rate, 4);
            Serial.print(" Ki="); Serial.print(pitch_axis.pid_i_rate, 4);
            Serial.print(" Kd="); Serial.println(pitch_axis.pid_d_rate, 4);
            Serial.print("Angle Roll : Kp="); Serial.print(roll_axis.pid_p_angle, 4);
            Serial.print(" Ki="); Serial.print(roll_axis.pid_i_angle, 4);
            Serial.print(" Kd="); Serial.println(roll_axis.pid_d_angle, 4);
            Serial.print("Angle Pitch: Kp="); Serial.print(pitch_axis.pid_p_angle, 4);
            Serial.print(" Ki="); Serial.print(pitch_axis.pid_i_angle, 4);
            Serial.print(" Kd="); Serial.println(pitch_axis.pid_d_angle, 4);
            current_mode = MODE_NORMAL;
            tune_done_flash_until = millis() + 3000;
            break;
        default:
            current_mode = MODE_NORMAL;
            break;
    }
}

void startFullAutotune() {
    Serial.println("🚀 開始飛行中自動調校：Rate(Roll->Pitch) -> Angle(Roll->Pitch)");
    autotune_chain_mode = CHAIN_FULL;
    roll_axis.mode = AXIS_TUNE_RATE;
    resetRelayState(roll_axis.rt_rate, gx, AUTOTUNE_TIMEOUT_RATE_MS);
    current_mode = MODE_AUTOTUNE_RATE_ROLL;
}

void startRateOnlyAutotune() {
    Serial.println("🚀 開始內環 Rate Loop 自動調校 (Roll -> Pitch)");
    autotune_chain_mode = CHAIN_RATE_ONLY;
    roll_axis.mode = AXIS_TUNE_RATE;
    resetRelayState(roll_axis.rt_rate, gx, AUTOTUNE_TIMEOUT_RATE_MS);
    current_mode = MODE_AUTOTUNE_RATE_ROLL;
}

void startAngleOnlyAutotune() {
    Serial.println("🚀 開始外環 Angle Loop 自動調校 (Roll -> Pitch)，假設內環增益已可用");
    autotune_chain_mode = CHAIN_ANGLE_ONLY;
    roll_axis.mode = AXIS_TUNE_ANGLE;
    resetRelayState(roll_axis.rt_angle, roll, AUTOTUNE_TIMEOUT_ANGLE_MS);
    current_mode = MODE_AUTOTUNE_ANGLE_ROLL;
}

void abortAutotune(const char* reason) {
    Serial.print("⛔ 自動調校中止：");
    Serial.println(reason);
    current_mode = MODE_NORMAL;
    roll_axis.mode = AXIS_NORMAL;
    pitch_axis.mode = AXIS_NORMAL;
    resetAxisIntegrators(roll_axis);
    resetAxisIntegrators(pitch_axis);
}

const char* modeName(SystemMode m) {
    switch (m) {
        case MODE_NORMAL: return "NORMAL";
        case MODE_AUTOTUNE_RATE_ROLL: return "TUNE-RATE-ROLL";
        case MODE_AUTOTUNE_RATE_PITCH: return "TUNE-RATE-PITCH";
        case MODE_AUTOTUNE_ANGLE_ROLL: return "TUNE-ANGLE-ROLL";
        case MODE_AUTOTUNE_ANGLE_PITCH: return "TUNE-ANGLE-PITCH";
        case MODE_EMERGENCY_DESCEND: return "EMERGENCY-DESCEND";
        default: return "?";
    }
}

// ==========================================
// 🚀 Arduino 核心 Setup & Loop
// ==========================================
void setup() {
    Serial.begin(115200);
    delay(200);
    pinMode(M1_PIN, OUTPUT); pinMode(M2_PIN, OUTPUT);
    pinMode(M3_PIN, OUTPUT); pinMode(M4_PIN, OUTPUT);
    pinMode(LED_A, OUTPUT);  pinMode(LED_B, OUTPUT); pinMode(LED_C, OUTPUT);

    analogWriteFreq(PWM_FREQ);
    analogWriteRange(PWM_RANGE);
    analogWrite(M1_PIN, 0); analogWrite(M2_PIN, 0);
    analogWrite(M3_PIN, 0); analogWrite(M4_PIN, 0);

    elrsInit();   // ELRS/CRSF 接收機初始化 (見 elrs.cpp)

    Wire.setSDA(I2C0_SDA);
    Wire.setSCL(I2C0_SCL);
    Wire.begin();
    while (!bmi088.isConnection()) {
        Serial.println("❌ BMI088 連線失敗，檢查硬體中...");
        digitalWrite(LED_C, HIGH); delay(500); digitalWrite(LED_C, LOW); delay(500);
    }
    bmi088.initialize();

    calibrateSensors();

    if (RELAY_AMPLITUDE_RATE > (float)MOTOR_MAX_LIMIT / 2.0f) {
        Serial.println("⚠️ 警告: RELAY_AMPLITUDE_RATE 設定過大，請重新確認！");
    }

    Serial.println("==========================================");
    Serial.println(" 飛控就緒");
    Serial.println(" 解鎖   : CH5開 + 油門最低");
    Serial.println(" 自動調校: 起飛懸停後 CH6(AUX1)開 (需搖桿置中+油門在懸停區間)");
    Serial.println(" 緊急緩降: 任何狀態下 CH7(AUX2)開，扳回可取消");
    Serial.println(" 地面測試 Serial 指令: t/r/a/x");
    Serial.println("==========================================");

    lastUpdate = micros();
    last_pid_time = millis();
    last_angle_loop_time = millis();
    digitalWrite(LED_A, LOW);
}

void loop() {
    // 0. 讀取 ELRS/CRSF (見 elrs.cpp)
    elrsUpdate();

    // 0b. 地面測試用 Serial 指令
    if (Serial.available() > 0) {
        char cmd = Serial.read();
        if (cmd == 'x' || cmd == 'X') {
            abortAutotune("使用者手動中止 (Serial)");
        } else if (current_mode == MODE_NORMAL && is_armed) {
            bool ok = autotunePreconditionsOk(elrsGetChannel(CH_ROLL), elrsGetChannel(CH_PITCH), elrsGetChannel(CH_THROTTLE));
            if (!ok) {
                Serial.println("⚠️ 搖桿未置中或油門不在懸停區間，拒絕開始自動調校");
            } else {
                if (cmd == 't' || cmd == 'T') startFullAutotune();
                else if (cmd == 'r' || cmd == 'R') startRateOnlyAutotune();
                else if (cmd == 'a' || cmd == 'A') startAngleOnlyAutotune();
            }
        }
    }

    // 1. IMU 讀取與姿態估計 (Mahony)
    bmi088.getAcceleration(&ax, &ay, &az);
    bmi088.getGyroscope(&gx, &gy, &gz);
    ax -= ax_offset; ay -= ay_offset; az -= az_offset;
    gx -= gx_offset; gy -= gy_offset;
    gz -= gz_offset;

    unsigned long now = micros();
    float dt = (now - lastUpdate) / 1000000.0f;
    lastUpdate = now;
    if (dt <= 0.0f || dt > 0.1f) dt = 0.002f;

    float gx_rad = gx * 0.0174533f;
    float gy_rad = gy * 0.0174533f;
    float gz_rad = gz * 0.0174533f;
    updateMahony(ax, ay, az, gx_rad, gy_rad, gz_rad, dt);

    // 2. 核心控制迴路 (內環固定頻率：500Hz / 2ms)
    unsigned long currentTime = millis();
    float pid_dt = (currentTime - last_pid_time) / 1000.0f;

    if (pid_dt >= 0.002f) {
        last_pid_time = currentTime;
        unsigned long roll_us     = elrsGetChannel(CH_ROLL);
        unsigned long pitch_us    = elrsGetChannel(CH_PITCH);
        unsigned long yaw_us      = elrsGetChannel(CH_YAW);
        unsigned long throttle_us = elrsGetChannel(CH_THROTTLE);
        bool armSwitchOn       = elrsGetChannel(CH_ARM)               > ARM_SWITCH_THRESHOLD_US;
        bool autotuneSwitchOn  = elrsGetChannel(CH_AUTOTUNE)          > AUTOTUNE_SWITCH_THRESHOLD_US;
        bool emergencySwitchOn = elrsGetChannel(CH_EMERGENCY_DESCEND) > EMERGENCY_SWITCH_THRESHOLD_US;
        bool linkOk = elrsLinkOk(FAILSAFE_TIMEOUT_MS);

        // --- 失聯保護 (Failsafe) ---
        if (!linkOk) {
            if (is_armed) {
                is_armed = false;
                if (current_mode != MODE_NORMAL) abortAutotune("ELRS 失聯 (Failsafe)");
            }
        } else {
            // --- 解鎖 / 上鎖互鎖 ---
            if (!is_armed && armSwitchOn && throttle_us < ARM_THROTTLE_MAX_US) {
                is_armed = true;
                resetAxisIntegrators(roll_axis);
                resetAxisIntegrators(pitch_axis);
            } else if (is_armed && !armSwitchOn) {
                is_armed = false;
                if (current_mode != MODE_NORMAL) abortAutotune("解鎖開關關閉");
            }
        }
        digitalWrite(LED_A, is_armed ? HIGH : LOW);

        // --- 嚴重傾斜安全斷電 ---
        if (is_armed && (abs(roll) > SEVERE_TILT_CUTOFF_DEG || abs(pitch) > SEVERE_TILT_CUTOFF_DEG)) {
            is_armed = false;
            if (current_mode != MODE_NORMAL) abortAutotune("傾斜角度超過安全門檻");
        }

        // --- 外環(Angle Loop)分頻判斷 ---
        angle_loop_counter++;
        bool run_outer = false;
        float outer_dt = 0.0f;
        if (angle_loop_counter >= ANGLE_LOOP_DIVIDER) {
            angle_loop_counter = 0;
            run_outer = true;
            outer_dt = (currentTime - last_angle_loop_time) / 1000.0f;
            last_angle_loop_time = currentTime;
            if (outer_dt <= 0.0f || outer_dt > 0.5f) outer_dt = ANGLE_LOOP_DIVIDER * 0.002f;
        }

        float output_roll = 0, output_pitch = 0, output_yaw = 0;
        int motor_base = 0;

        if (is_armed && linkOk) {
            // --- 自動調校開關 edge-trigger 偵測 ---
            bool autotune_rising_edge = autotuneSwitchOn && !autotune_switch_prev;
            autotune_switch_prev = autotuneSwitchOn;
            if (autotune_rising_edge) autotune_request_pending = true;
            if (!autotuneSwitchOn) autotune_request_pending = false;

            // --- 緊急自動減速緩降 ---
            if (emergencySwitchOn && current_mode != MODE_EMERGENCY_DESCEND) {
                if (current_mode != MODE_NORMAL) abortAutotune("緊急自動減速緩降已觸發，調校中止");
                current_mode = MODE_EMERGENCY_DESCEND;
                emergency_descend_base = (float)last_motor_base;
                emergency_descend_start_ms = currentTime;
                emergency_low_throttle_since_ms = 0;
                autotune_request_pending = false;
                Serial.println("🛑 緊急自動減速緩降啟動！");
            } else if (!emergencySwitchOn && current_mode == MODE_EMERGENCY_DESCEND) {
                current_mode = MODE_NORMAL;
                Serial.println("緊急緩降已取消，恢復遙控操作");
            }

            if (current_mode == MODE_EMERGENCY_DESCEND) {
                roll_axis.target_angle  = 0.0f;
                pitch_axis.target_angle = 0.0f;
                target_yaw_rate = 0.0f;

                float err_yaw_rate = target_yaw_rate - gz;
                output_yaw = pid_p_yaw * err_yaw_rate;
                output_roll  = computeAxisOutput(roll_axis,  roll,  gx, run_outer, outer_dt, pid_dt, currentTime);
                output_pitch = computeAxisOutput(pitch_axis, pitch, gy, run_outer, outer_dt, pid_dt, currentTime);

                float elapsed_s = (currentTime - emergency_descend_start_ms) / 1000.0f;
                float ramped = emergency_descend_base - EMERGENCY_DESCENT_RATE_PWM_PER_S * elapsed_s;
                motor_base = (int)constrain(ramped, 0.0f, (float)MOTOR_MAX_LIMIT);
                if (motor_base <= EMERGENCY_LAND_PWM_THRESHOLD) {
                    if (emergency_low_throttle_since_ms == 0) emergency_low_throttle_since_ms = currentTime;
                    if (currentTime - emergency_low_throttle_since_ms > EMERGENCY_LAND_CONFIRM_MS) {
                        is_armed = false;
                        current_mode = MODE_NORMAL;
                        Serial.println("✅ 緊急緩降完成，已自動上鎖");
                    }
                } else {
                    emergency_low_throttle_since_ms = 0;
                }

            } else {
                // --- 正常飛行 / 自動調校 ---
                roll_axis.target_angle  = stickToTarget(roll_us,  MAX_STICK_ANGLE);
                pitch_axis.target_angle = stickToTarget(pitch_us, MAX_STICK_ANGLE);
                target_yaw_rate         = stickToTarget(yaw_us,   MAX_STICK_YAWRATE);
                if (current_mode == MODE_NORMAL) {
                    if (autotune_request_pending && autotunePreconditionsOk(roll_us, pitch_us, throttle_us)) {
                        startFullAutotune();
                        autotune_request_pending = false; 
                    }
                } else {
                    if (!autotuneSwitchOn || !autotunePreconditionsOk(roll_us, pitch_us, throttle_us)) {
                        abortAutotune("飛行中安全前提不再滿足 (開關/搖桿/油門)");
                    }
                }

                float err_yaw_rate = target_yaw_rate - gz;
                output_yaw = pid_p_yaw * err_yaw_rate;
                output_roll  = computeAxisOutput(roll_axis,  roll,  gx, run_outer, outer_dt, pid_dt, currentTime);
                output_pitch = computeAxisOutput(pitch_axis, pitch, gy, run_outer, outer_dt, pid_dt, currentTime);

                switch (current_mode) {
                    case MODE_AUTOTUNE_RATE_ROLL:
                        if (roll_axis.rt_rate.done) {
                            finalizeAxisTuning(roll_axis.rt_rate, "Roll-RATE", RELAY_AMPLITUDE_RATE,
                                             roll_axis.pid_p_rate, roll_axis.pid_i_rate, roll_axis.pid_d_rate);
                            advanceAutotuneChain(MODE_AUTOTUNE_RATE_ROLL);
                        } else if (roll_axis.rt_rate.timed_out) {
                            abortAutotune("Roll Rate Loop 逾時");
                        }
                        break;
                    case MODE_AUTOTUNE_RATE_PITCH:
                        if (pitch_axis.rt_rate.done) {
                            finalizeAxisTuning(pitch_axis.rt_rate, "Pitch-RATE", RELAY_AMPLITUDE_RATE,
                                             pitch_axis.pid_p_rate, pitch_axis.pid_i_rate, pitch_axis.pid_d_rate);
                            advanceAutotuneChain(MODE_AUTOTUNE_RATE_PITCH);
                        } else if (pitch_axis.rt_rate.timed_out) {
                            abortAutotune("Pitch Rate Loop 逾時");
                        }
                        break;
                    case MODE_AUTOTUNE_ANGLE_ROLL:
                        if (roll_axis.rt_angle.done) {
                            finalizeAxisTuning(roll_axis.rt_angle, "Roll-ANGLE", RELAY_AMPLITUDE_ANGLE,
                                             roll_axis.pid_p_angle, roll_axis.pid_i_angle, roll_axis.pid_d_angle);
                            advanceAutotuneChain(MODE_AUTOTUNE_ANGLE_ROLL);
                        } else if (roll_axis.rt_angle.timed_out) {
                            abortAutotune("Roll Angle Loop 逾時");
                        }
                        break;
                    case MODE_AUTOTUNE_ANGLE_PITCH:
                        if (pitch_axis.rt_angle.done) {
                            finalizeAxisTuning(pitch_axis.rt_angle, "Pitch-ANGLE", RELAY_AMPLITUDE_ANGLE,
                                             pitch_axis.pid_p_angle, pitch_axis.pid_i_angle, pitch_axis.pid_d_angle);
                            advanceAutotuneChain(MODE_AUTOTUNE_ANGLE_PITCH);
                        } else if (pitch_axis.rt_angle.timed_out) {
                            abortAutotune("Pitch Angle Loop 逾時");
                        }
                        break;
                    default:
                        break;
                }

                motor_base = max(throttleToBase(throttle_us), MOTOR_IDLE_PWM);
            }

            last_motor_base = motor_base;
        }

        // 3. 馬達混控 (X型混控架構)
        int m1 = motor_base - output_roll + output_pitch + output_yaw;
        int m2 = motor_base - output_roll - output_pitch - output_yaw;
        int m3 = motor_base + output_roll - output_pitch + output_yaw;
        int m4 = motor_base + output_roll + output_pitch - output_yaw;
        if (is_armed && linkOk) {
            m1 = constrain(m1, 0, MOTOR_MAX_LIMIT);
            m2 = constrain(m2, 0, MOTOR_MAX_LIMIT);
            m3 = constrain(m3, 0, MOTOR_MAX_LIMIT);
            m4 = constrain(m4, 0, MOTOR_MAX_LIMIT);
        } else {
            m1 = m2 = m3 = m4 = 0;
        }

        analogWrite(M1_PIN, m1);
        analogWrite(M2_PIN, m2);
        analogWrite(M3_PIN, m3);
        analogWrite(M4_PIN, m4);
    }

    // 4. LED 狀態燈號 (10Hz 統一更新)
    static unsigned long led_timer = 0;
    if (millis() - led_timer > 100) {
        led_timer = millis();
        bool linkOkNow = elrsLinkOk(FAILSAFE_TIMEOUT_MS);

        if (!linkOkNow) {
            digitalWrite(LED_C, HIGH);
            digitalWrite(LED_B, LOW);
        } else if (current_mode == MODE_EMERGENCY_DESCEND) {
            static bool toggle = false;
            toggle = !toggle;
            digitalWrite(LED_B, toggle ? HIGH : LOW);
            digitalWrite(LED_C, toggle ? LOW : HIGH);
        } else if (current_mode != MODE_NORMAL) {
            digitalWrite(LED_C, !digitalRead(LED_C));
            digitalWrite(LED_B, LOW);
        } else if (millis() < tune_done_flash_until) {
            digitalWrite(LED_B, HIGH);
            digitalWrite(LED_C, LOW);
        } else if (is_armed) {
            digitalWrite(LED_C, LOW);
            static byte blink_cnt = 0;
            blink_cnt++;
            if (blink_cnt % 5 == 0) digitalWrite(LED_B, !digitalRead(LED_B));
        } else {
            digitalWrite(LED_B, LOW);
            digitalWrite(LED_C, LOW);
        }
    }

    // 5. 低頻地面除錯輸出 (10Hz)
    static unsigned long last_debug = 0;
    if (millis() - last_debug > 100) {
        last_debug = millis();
        Serial.print("ARM:");
        Serial.print(is_armed ? 1 : 0);
        Serial.print(" LINK:"); Serial.print(elrsLinkOk(FAILSAFE_TIMEOUT_MS) ? 1 : 0);
        Serial.print(" MODE:"); Serial.print(modeName(current_mode));
        Serial.print(" R:"); Serial.print(roll, 1);
        Serial.print(" P:"); Serial.print(pitch, 1);
        Serial.print(" THR:"); Serial.print(elrsGetChannel(CH_THROTTLE));
        Serial.print(" AUX1:"); Serial.print(elrsGetChannel(CH_AUTOTUNE));
        Serial.print(" AUX2:"); Serial.print(elrsGetChannel(CH_EMERGENCY_DESCEND));
        Serial.print(" CRC_ERR:"); Serial.println(elrsGetCrcErrorCount());
    }
}
