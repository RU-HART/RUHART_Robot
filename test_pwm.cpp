// Copyright 2026 
// Standalone PWM & Motor Driver Validation Test

#ifndef ESP32
  #error This test is strictly for the ESP32 architecture
#endif

#include <Arduino.h>

// ============================================================================
// HARDWARE PIN MAPPING (Must match main firmware)
// ============================================================================

// --- L298N Motor Driver Pins ---
// Left Side
const uint8_t L298N_ENA = 27; // PWM Left
const uint8_t L298N_IN1 = 26; // Dir Left A
const uint8_t L298N_IN2 = 25; // Dir Left B

// Right Side
const uint8_t L298N_ENB = 12; // PWM Right
const uint8_t L298N_IN3 = 33; // Dir Right A
const uint8_t L298N_IN4 = 32; // Dir Right B

// ============================================================================
// PWM CONFIGURATION
// ============================================================================
const uint8_t MOTOR_PWM_CHANNEL_LEFT = 3;
const uint8_t MOTOR_PWM_CHANNEL_RIGHT = 4;
const uint32_t MOTOR_PWM_FREQ = 5000;
const uint8_t MOTOR_PWM_RESOLUTION = 8; // 8-bit resolution (0 to 255)

// Testing parameters
const uint16_t RAMP_DELAY_MS = 50; // Milliseconds between each PWM increment

void setup() {
    Serial.begin(115200);
    Serial.println("\n--- L298N MOTOR & PWM DIAGNOSTIC TEST ---");

    // 1. Initialize Direction Pins
    pinMode(L298N_IN1, OUTPUT);
    pinMode(L298N_IN2, OUTPUT);
    pinMode(L298N_IN3, OUTPUT);
    pinMode(L298N_IN4, OUTPUT);

    // 2. Configure ESP32 ledc (PWM) timers
    ledcSetup(MOTOR_PWM_CHANNEL_LEFT, MOTOR_PWM_FREQ, MOTOR_PWM_RESOLUTION);
    ledcAttachPin(L298N_ENA, MOTOR_PWM_CHANNEL_LEFT);
    
    ledcSetup(MOTOR_PWM_CHANNEL_RIGHT, MOTOR_PWM_FREQ, MOTOR_PWM_RESOLUTION);
    ledcAttachPin(L298N_ENB, MOTOR_PWM_CHANNEL_RIGHT);

    // 3. Ensure motors are initially stopped
    digitalWrite(L298N_IN1, LOW);
    digitalWrite(L298N_IN2, LOW);
    digitalWrite(L298N_IN3, LOW);
    digitalWrite(L298N_IN4, LOW);
    ledcWrite(MOTOR_PWM_CHANNEL_LEFT, 0);
    ledcWrite(MOTOR_PWM_CHANNEL_RIGHT, 0);

    Serial.println("Setup complete. Beginning PWM ramp cycle in 3 seconds...");
    delay(3000);
}

void loop() {
    Serial.println("\n[TEST] Setting direction: SPIN RIGHT (Left FWD, Right REV)");

    // Left Wheels: FORWARD
    digitalWrite(L298N_IN1, HIGH);
    digitalWrite(L298N_IN2, LOW);

    // Right Wheels: REVERSE
    digitalWrite(L298N_IN3, LOW);
    digitalWrite(L298N_IN4, HIGH);

    // Slowly ramp PWM from 0 to 255 (0% to 100%)
    Serial.println("[TEST] Ramping up power...");
    for (int pwm_val = 0; pwm_val <= 255; pwm_val++) {
        ledcWrite(MOTOR_PWM_CHANNEL_LEFT, pwm_val);
        ledcWrite(MOTOR_PWM_CHANNEL_RIGHT, pwm_val);
        
        // Print status every 25 steps to avoid flooding the serial monitor
        if (pwm_val % 25 == 0 || pwm_val == 255) {
            float percent = ((float)pwm_val / 255.0f) * 100.0f;
            Serial.printf("Power: %3d/255 (%.1f%%)\n", pwm_val, percent);
        }
        
        delay(RAMP_DELAY_MS);
    }

    Serial.println("[TEST] Maximum power reached. Holding for 3 seconds...");
    delay(3000);

    Serial.println("[TEST] Stopping motors. Resting for 2 seconds...");
    // Cut power and coast to a stop
    ledcWrite(MOTOR_PWM_CHANNEL_LEFT, 0);
    ledcWrite(MOTOR_PWM_CHANNEL_RIGHT, 0);
    digitalWrite(L298N_IN1, LOW);
    digitalWrite(L298N_IN2, LOW);
    digitalWrite(L298N_IN3, LOW);
    digitalWrite(L298N_IN4, LOW);

    delay(2000);
    Serial.println("[TEST] Cycle complete. Restarting...");
}
