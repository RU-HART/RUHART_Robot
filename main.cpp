// Copyright 2023-2026 KAIA.AI
// Licensed under the Apache License, Version 2.0

#ifndef ESP32
  #error This code only runs on an ESP32
#endif

#include <Arduino.h>
#include "lds_all_models.h"

// ============================================================================
// SYSTEM CONSTANTS & CONFIGURATION
// ============================================================================

// LiDAR Configuration
const uint8_t LIDAR_GPIO_EN  = 14;
const uint8_t LIDAR_GPIO_RX  = 3;   
const uint8_t LIDAR_GPIO_TX  = 1;   
const uint8_t LIDAR_GPIO_PWM = 13;

const uint32_t SERIAL_MONITOR_BAUD = 115200;
const uint32_t LIDAR_PWM_FREQ = 10000;
const uint8_t LIDAR_PWM_BITS = 11;
const uint8_t LIDAR_PWM_CHANNEL = 2;

// Assuming LDROBOT LD14P protocol is compatible with LD20, 
// otherwise use the generic serial parser provided by your library.
#define LDROBOT_LD14P 

// Motor & Navigation Constants
const uint8_t LEFT_MOTOR_PIN = 25;
const uint8_t RIGHT_MOTOR_PIN = 26;
const uint8_t MOTOR_PWM_CHANNEL_LEFT = 3;
const uint8_t MOTOR_PWM_CHANNEL_RIGHT = 4;
const uint8_t MOTOR_PWM_FREQ = 5000;
const uint8_t MOTOR_PWM_RESOLUTION = 8; // 0-255

// 20% Power Calculation: 255 * 0.20 = 51
const uint8_t PWM_20_PERCENT = 51; 
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

// ============================================================================
// MOTOR CONTROL ABSTRACTION
// ============================================================================

/*
THIS IS SUBJECT TO CHANGE

Assuming 1 pin per motor for simplicity based on prompt: 
Applying PWM moves the track forward. Setting to 0 stops it.
To turn right, the right track stops and the left track drives.
*/
void setMotors(uint8_t left_pwm, uint8_t right_pwm) {
    ledcWrite(MOTOR_PWM_CHANNEL_LEFT, left_pwm);
    ledcWrite(MOTOR_PWM_CHANNEL_RIGHT, right_pwm);
}
/* END OF SUBJECT TO CHANGE */

// ============================================================================
// LiDAR CALLBACKS
// ============================================================================

void lidar_scan_point_callback(float angle_deg, float distance_mm, float quality, bool scan_completed) {
    // Round angle to nearest integer to act as an array index [0-359]
    int angle_idx = ((int)(angle_deg + 0.5f)) % 360;
    if (angle_idx < 0) angle_idx += 360;
    
    // Store distance. Ignore 0 readings (invalid/too far)
    if (distance_mm > 0) {
        scan_data[angle_idx] = (uint16_t)distance_mm;
    }

    if (scan_completed) {
        scan_ready = true;
    }
}

void lidar_info_callback(LDS::info_t code, String info) {
    Serial.printf("[LiDAR INFO] %s\n", info.c_str());
}

void lidar_error_callback(LDS::result_t code, String aux_info) {
    Serial.printf("[LiDAR ERROR] %s\n", aux_info.c_str());
}

// ============================================================================
// NAVIGATION & MATH UTILITIES
// ============================================================================

// Finds the angle with the absolute minimum distance (closest object)
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

// Heuristic: A corner is a local maximum distance flanked by closer walls.
int16_t findClosestCorner() {
    int16_t closest_corner_angle = -1;
    uint16_t min_corner_dist = 0xFFFF;

    for (int i = 0; i < 360; i++) {
        // Look ahead and behind by 15 degrees to check for local maximum
        int idx_minus = (i - 15 + 360) % 360;
        int idx_plus = (i + 15) % 360;

        if (scan_data[i] > 0 && scan_data[idx_minus] > 0 && scan_data[idx_plus] > 0) {
            if (scan_data[i] > scan_data[idx_minus] && scan_data[i] > scan_data[idx_plus]) {
                // It's a local maximum (potential corner)
                if (scan_data[i] < min_corner_dist) {
                    min_corner_dist = scan_data[i];
                    closest_corner_angle = i;
                }
            }
        }
    }
    return closest_corner_angle;
}

// Checks the front 60 degree cone (330 to 30 degrees) for objects within threshold
bool isPathBlocked() {
    for (int i = 330; i < 360; i++) {
        if (scan_data[i] > 0 && scan_data[i] < COLLISION_THRESHOLD_MM) return true;
    }
    for (int i = 0; i <= 30; i++) {
        if (scan_data[i] > 0 && scan_data[i] < COLLISION_THRESHOLD_MM) return true;
    }
    return false;
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
            setMotors(PWM_20_PERCENT, 0); // Spin right
            current_state = STATE_CALIBRATE_TURN;
            break;

        case STATE_CALIBRATE_TURN: {
            int16_t current_closest = getClosestObjectAngle();
            if (current_closest == -1 || tracked_angle_start == -1) break;

            // If the closest point has shifted almost 360 degrees (back to its starting relative position)
            // We use a +/- 5 degree tolerance window to account for sensor noise.
            int diff = abs(current_closest - tracked_angle_start);
            if (millis() - calibration_start_time > 1000) { // Prevent immediate trigger
                if (diff < 5 || diff > 355) {
                    uint32_t spin_time = millis() - calibration_start_time;
                    millis_per_360_deg += spin_time; // Accumulate time
                    calibration_spins++;
                    
                    Serial.printf("[CALIBRATION] Spin %d completed in %lu ms.\n", calibration_spins, spin_time);
                    
                    if (calibration_spins >= 3) {
                        millis_per_360_deg /= 3.0f; // Average the 3 runs
                        setMotors(0, 0); // Stop
                        Serial.printf("[CALIBRATION] Done. Avg ms/360deg: %.2f\n", millis_per_360_deg);
                        current_state = STATE_FIND_CORNER;
                    } else {
                        // Reset for next spin
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
                Serial.printf("[FSM] Closest corner found at %d degrees. Turning...\n", target_heading);
                
                // Calculate turn duration based on calibration
                float turn_ratio = (float)target_heading / 360.0f;
                turn_duration = (uint32_t)(millis_per_360_deg * turn_ratio);
                turn_start_time = millis();
                
                setMotors(PWM_20_PERCENT, 0); // Execute turn
                current_state = STATE_NAVIGATE_CORNER;
            } else {
                Serial.println("[FSM] No distinct corner found. Defaulting to Wander mode.");
                current_state = STATE_WANDER;
            }
            break;

        case STATE_NAVIGATE_CORNER:
            // Wait for time-based turn to complete based on calibration
            if (millis() - turn_start_time >= turn_duration) {
                Serial.println("[FSM] Turn complete. Driving to corner.");
                setMotors(PWM_20_PERCENT, PWM_20_PERCENT);
                current_state = STATE_WANDER;
            }
            break;

        case STATE_WANDER:
            if (isPathBlocked()) {
                Serial.println("[NAV] Obstacle detected! Evading...");
                // Stop, pick a clear direction (e.g., 90 degrees right)
                target_heading = 90; 
                turn_duration = (uint32_t)(millis_per_360_deg * (90.0f / 360.0f));
                turn_start_time = millis();
                
                setMotors(PWM_20_PERCENT, 0); // Turn right in place
                current_state = STATE_NAVIGATE_CORNER; // Reuse the timed-turn state
            } else {
                // Coast is clear, keep driving
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
    Serial.println("\n--- TANK BOT INITIALIZING ---");

    // Initialize Motor PWM
    ledcSetup(MOTOR_PWM_CHANNEL_LEFT, MOTOR_PWM_FREQ, MOTOR_PWM_RESOLUTION);
    ledcAttachPin(LEFT_MOTOR_PIN, MOTOR_PWM_CHANNEL_LEFT);
    ledcSetup(MOTOR_PWM_CHANNEL_RIGHT, MOTOR_PWM_FREQ, MOTOR_PWM_RESOLUTION);
    ledcAttachPin(RIGHT_MOTOR_PIN, MOTOR_PWM_CHANNEL_RIGHT);
    setMotors(0, 0); // Ensure stopped

    // Initialize LiDAR (Static Allocation)
    static LDS_LDROBOT_LD14P static_lidar_instance;
    lidar = &static_lidar_instance;

    lidar->setScanPointCallback(lidar_scan_point_callback);
    lidar->setInfoCallback(lidar_info_callback);
    lidar->setErrorCallback(lidar_error_callback);

    LidarSerial.begin(lidar->getSerialBaudRate(), SERIAL_8N1, LIDAR_GPIO_TX, LIDAR_GPIO_RX);
    lidar->init();
    
    Serial.print("LiDAR model: ");
    Serial.println(lidar->getModelName());

    LDS::result_t result = lidar->start();
    Serial.printf("LiDAR Start Result: %s\n", lidar->resultCodeToString(result).c_str());
}

void loop() {
    lidar->loop(); // Must be called continuously

    // Only process state machine once a full 360-degree sweep is updated
    if (scan_ready) {
        processStateMachine();
        scan_ready = false; 
    }
}