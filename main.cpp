// Copyright 2023-2026 KAIA.AI
// Licensed under the Apache License, Version 2.0

#ifndef ESP32
  #error This example runs on ESP32
#endif

#include <Arduino.h>
#include "lds_all_models.h"

// ============================================================================
// SYSTEM CONSTANTS & CONFIGURATION
// ============================================================================

// LiDAR Configuration (LD20)
const uint8_t LIDAR_GPIO_EN  = 14;
const uint8_t LIDAR_GPIO_RX  = 3;   
const uint8_t LIDAR_GPIO_TX  = 1;   
const uint8_t LIDAR_GPIO_PWM = 13;

const uint32_t SERIAL_MONITOR_BAUD = 115200;

#define LDROBOT_LD14P // LD20 protocol equivalent

// --- L298N Motor Driver Pins ---
// Left Side
const uint8_t L298N_ENA = 27; // PWM Left
const uint8_t L298N_IN1 = 26; // Dir Left A
const uint8_t L298N_IN2 = 25; // Dir Left B
// Right Side
const uint8_t L298N_ENB = 12; // PWM Right
const uint8_t L298N_IN3 = 33; // Dir Right A
const uint8_t L298N_IN4 = 32; // Dir Right B

// PWM Settings
const uint8_t MOTOR_PWM_CHANNEL_LEFT = 3;
const uint8_t MOTOR_PWM_CHANNEL_RIGHT = 4;
const uint32_t MOTOR_PWM_FREQ = 5000;
const uint8_t MOTOR_PWM_RESOLUTION = 8; // 0-255

// 20% Power Calculation: 255 * 0.20 = 51
const int16_t PWM_20_PERCENT = 51; 
const uint16_t COLLISION_THRESHOLD_MM = 300; // 30cm keep-out zone

// ============================================================================
// STATIC MEMORY & GLOBALS (No heap allocation during runtime)
// ============================================================================

HardwareSerial LidarSerial(1);
LDS *lidar;

// 360-degree topographical map. Index = angle in degrees, Value = distance in mm.
static uint16_t scan_data[360] = {0}; 
static bool scan_ready = false;

// FSM States
enum RobotState {
    STATE_INIT,
    STATE_CALIBRATE_TURN,
    STATE_FIND_CORNER,
    STATE_NAVIGATE_CORNER,
    STATE_WANDER
};

static RobotState current_state = STATE_INIT;

// Calibration Tracking
static uint8_t calibration_spins = 0;
static uint32_t calibration_start_time = 0;
static float millis_per_360_deg = 0.0f;
static int16_t tracked_angle_start = -1;

// Navigation Tracking
static int16_t target_heading = -1;
static uint32_t turn_start_time = 0;
static uint32_t turn_duration = 0;
static bool turning_right = true;

// ============================================================================
// L298N MOTOR CONTROL ABSTRACTION
// ============================================================================

// Accepts speeds from -255 to 255. Negative = Reverse, Positive = Forward.
void setMotors(int16_t left_speed, int16_t right_speed) {
    
    // Constrain to valid 8-bit PWM bounds just in case
    left_speed = constrain(left_speed, -255, 255);
    right_speed = constrain(right_speed, -255, 255);

    // Left Motor Logic
    if (left_speed > 0) {
        digitalWrite(L298N_IN1, HIGH);
        digitalWrite(L298N_IN2, LOW);
        ledcWrite(MOTOR_PWM_CHANNEL_LEFT, left_speed);
    } else if (left_speed < 0) {
        digitalWrite(L298N_IN1, LOW);
        digitalWrite(L298N_IN2, HIGH);
        ledcWrite(MOTOR_PWM_CHANNEL_LEFT, abs(left_speed));
    } else {
        digitalWrite(L298N_IN1, LOW);
        digitalWrite(L298N_IN2, LOW);
        ledcWrite(MOTOR_PWM_CHANNEL_LEFT, 0);
    }

    // Right Motor Logic
    if (right_speed > 0) {
        digitalWrite(L298N_IN3, HIGH);
        digitalWrite(L298N_IN4, LOW);
        ledcWrite(MOTOR_PWM_CHANNEL_RIGHT, right_speed);
    } else if (right_speed < 0) {
        digitalWrite(L298N_IN3, LOW);
        digitalWrite(L298N_IN4, HIGH);
        ledcWrite(MOTOR_PWM_CHANNEL_RIGHT, abs(right_speed));
    } else {
        digitalWrite(L298N_IN3, LOW);
        digitalWrite(L298N_IN4, LOW);
        ledcWrite(MOTOR_PWM_CHANNEL_RIGHT, 0);
    }
}

// ============================================================================
// LiDAR CALLBACKS
// ============================================================================

void lidar_scan_point_callback(float angle_deg, float distance_mm, float quality, bool scan_completed) {
    int angle_idx = ((int)(angle_deg + 0.5f)) % 360;
    if (angle_idx < 0) angle_idx += 360;
    
    if (distance_mm > 0) {
        scan_data[angle_idx] = (uint16_t)distance_mm;
    }

    if (scan_completed) scan_ready = true;
}

void lidar_info_callback(LDS::info_t code, String info) {
    Serial.printf("[LiDAR INFO] %s\n", info.c_str());
}

void lidar_error_callback(LDS::result_t code, String aux_info) {
    Serial.printf("[LiDAR ERROR] %s\n", aux_info.c_str());
}

// ============================================================================
// NAVIGATION MATH UTILITIES
// ============================================================================

int16_t getClosestObjectAngle() {
    uint16_t min_dist = 0xFFFF;
    int16_t best_angle = -1;
    for (int i = 0; i < 360; i++) {
        if (scan_data[i] > 0 && scan_data[i] < min_dist) {
            min_dist = scan_data[i];
            best_angle = i;
        }
    }
    return best_angle;
}

int16_t findClosestCorner() {
    int16_t closest_corner_angle = -1;
    uint16_t min_corner_dist = 0xFFFF;

    for (int i = 0; i < 360; i++) {
        int idx_minus = (i - 15 + 360) % 360;
        int idx_plus = (i + 15) % 360;

        if (scan_data[i] > 0 && scan_data[idx_minus] > 0 && scan_data[idx_plus] > 0) {
            if (scan_data[i] > scan_data[idx_minus] && scan_data[i] > scan_data[idx_plus]) {
                if (scan_data[i] < min_corner_dist) {
                    min_corner_dist = scan_data[i];
                    closest_corner_angle = i;
                }
            }
        }
    }
    return closest_corner_angle;
}

bool isPathBlocked() {
    for (int i = 330; i < 360; i++) {
        if (scan_data[i] > 0 && scan_data[i] < COLLISION_THRESHOLD_MM) return true;
    }
    for (int i = 0; i <= 30; i++) {
        if (scan_data[i] > 0 && scan_data[i] < COLLISION_THRESHOLD_MM) return true;
    }
    return false;
}

// Calculates shortest turn direction and angular difference (accounts for 360 wrap)
int16_t getShortestTurn(int16_t target_angle, bool &turn_right) {
    int16_t diff = target_angle - 0; // 0 is forward
    if (diff > 180) diff -= 360;
    if (diff < -180) diff += 360;
    
    turn_right = (diff > 0);
    return abs(diff);
}

// ============================================================================
// STATE MACHINE LOGIC
// ============================================================================

void processStateMachine() {
    switch (current_state) {
        
        case STATE_INIT:
            Serial.println("[FSM] Entering Calibration Mode...");
            tracked_angle_start = getClosestObjectAngle();
            calibration_start_time = millis();
            // True zero-radius pivot turn: Left forward, Right reverse
            setMotors(PWM_20_PERCENT, -PWM_20_PERCENT); 
            current_state = STATE_CALIBRATE_TURN;
            break;

        case STATE_CALIBRATE_TURN: {
            int16_t current_closest = getClosestObjectAngle();
            if (current_closest == -1 || tracked_angle_start == -1) break;

            int diff = abs(current_closest - tracked_angle_start);
            if (millis() - calibration_start_time > 1000) { 
                if (diff < 5 || diff > 355) {
                    uint32_t spin_time = millis() - calibration_start_time;
                    millis_per_360_deg += spin_time; 
                    calibration_spins++;
                    
                    Serial.printf("[CALIBRATION] Pivot %d complete: %lu ms.\n", calibration_spins, spin_time);
                    
                    if (calibration_spins >= 3) {
                        millis_per_360_deg /= 3.0f; 
                        setMotors(0, 0); // Stop
                        Serial.printf("[CALIBRATION] Done. Avg ms/360deg pivot: %.2f\n", millis_per_360_deg);
                        current_state = STATE_FIND_CORNER;
                    } else {
                        tracked_angle_start = getClosestObjectAngle();
                        calibration_start_time = millis();
                    }
                }
            }
            break;
        }

        case STATE_FIND_CORNER:
            target_heading = findClosestCorner();
            if (target_heading != -1) {
                int16_t shortest_angle = getShortestTurn(target_heading, turning_right);
                Serial.printf("[FSM] Corner at %d deg. Pivot turning %d deg...\n", target_heading, shortest_angle);
                
                float turn_ratio = (float)shortest_angle / 360.0f;
                turn_duration = (uint32_t)(millis_per_360_deg * turn_ratio);
                turn_start_time = millis();
                
                if (turning_right) setMotors(PWM_20_PERCENT, -PWM_20_PERCENT);
                else setMotors(-PWM_20_PERCENT, PWM_20_PERCENT);
                
                current_state = STATE_NAVIGATE_CORNER;
            } else {
                Serial.println("[FSM] No corner found. Wander mode.");
                current_state = STATE_WANDER;
            }
            break;

        case STATE_NAVIGATE_CORNER:
            if (millis() - turn_start_time >= turn_duration) {
                Serial.println("[FSM] Pivot complete. Driving to target.");
                setMotors(PWM_20_PERCENT, PWM_20_PERCENT); // Drive straight
                current_state = STATE_WANDER;
            }
            break;

        case STATE_WANDER:
            if (isPathBlocked()) {
                Serial.println("[NAV] Obstacle! Executing 90-degree evasion pivot...");
                
                turn_duration = (uint32_t)(millis_per_360_deg * (90.0f / 360.0f));
                turn_start_time = millis();
                
                // Pivot right
                setMotors(PWM_20_PERCENT, -PWM_20_PERCENT); 
                current_state = STATE_NAVIGATE_CORNER;
            } else {
                setMotors(PWM_20_PERCENT, PWM_20_PERCENT);
            }
            break;
    }
}

// ============================================================================
// SYSTEM SETUP & MAIN LOOP
// ============================================================================

void setup() {
    Serial.begin(SERIAL_MONITOR_BAUD);
    Serial.println("\n--- 4-WHEEL TANK BOT INITIALIZING ---");

    // Initialize L298N Direction Pins
    pinMode(L298N_IN1, OUTPUT);
    pinMode(L298N_IN2, OUTPUT);
    pinMode(L298N_IN3, OUTPUT);
    pinMode(L298N_IN4, OUTPUT);

    // Initialize Motor PWM
    ledcSetup(MOTOR_PWM_CHANNEL_LEFT, MOTOR_PWM_FREQ, MOTOR_PWM_RESOLUTION);
    ledcAttachPin(L298N_ENA, MOTOR_PWM_CHANNEL_LEFT);
    ledcSetup(MOTOR_PWM_CHANNEL_RIGHT, MOTOR_PWM_FREQ, MOTOR_PWM_RESOLUTION);
    ledcAttachPin(L298N_ENB, MOTOR_PWM_CHANNEL_RIGHT);
    
    setMotors(0, 0); // Ensure stopped

    // Initialize LiDAR 
    static LDS_LDROBOT_LD14P static_lidar_instance;
    lidar = &static_lidar_instance;

    lidar->setScanPointCallback(lidar_scan_point_callback);
    lidar->setInfoCallback(lidar_info_callback);
    lidar->setErrorCallback(lidar_error_callback);

    LidarSerial.begin(lidar->getSerialBaudRate(), SERIAL_8N1, LIDAR_GPIO_TX, LIDAR_GPIO_RX);
    lidar->init();
    
    Serial.print("LiDAR model: ");
    Serial.println(lidar->getModelName());

    lidar->start();
}

void loop() {
    lidar->loop(); 

    if (scan_ready) {
        processStateMachine();
        scan_ready = false; 
    }
}
