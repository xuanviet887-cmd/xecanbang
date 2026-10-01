#include <Arduino.h>
#include <Wire.h>
#include <MPU6050_light.h>
#include "pinout.h"
#include "fuzzy_control.h"

MPU6050 mpu(Wire);

// Biến lưu trữ góc và đạo hàm
float current_angle = 0;
float last_angle = 0;
float angle_velocity = 0;
unsigned long last_time = 0;

void setup_motors() {
    pinMode(MOTOR_A_PWM, OUTPUT);
    pinMode(MOTOR_A_IN1, OUTPUT);
    pinMode(MOTOR_A_IN2, OUTPUT);
    
    pinMode(MOTOR_B_PWM, OUTPUT);
    pinMode(MOTOR_B_IN1, OUTPUT);
    pinMode(MOTOR_B_IN2, OUTPUT);
    
    pinMode(MOTOR_STBY, OUTPUT);
    digitalWrite(MOTOR_STBY, HIGH);
}

void set_motor_speed(int speed) {
    speed = constrain(speed, -255, 255);
    
    // Motor A
    if (speed > 0) {
        digitalWrite(MOTOR_A_IN1, HIGH);
        digitalWrite(MOTOR_A_IN2, LOW);
    } else if (speed < 0) {
        digitalWrite(MOTOR_A_IN1, LOW);
        digitalWrite(MOTOR_A_IN2, HIGH);
    } else {
        digitalWrite(MOTOR_A_IN1, LOW);
        digitalWrite(MOTOR_A_IN2, LOW);
    }
    analogWrite(MOTOR_A_PWM, abs(speed));

    // Motor B (Thường đảo chiều so với A tùy cách lắp)
    if (speed > 0) {
        digitalWrite(MOTOR_B_IN1, HIGH);
        digitalWrite(MOTOR_B_IN2, LOW);
    } else if (speed < 0) {
        digitalWrite(MOTOR_B_IN1, LOW);
        digitalWrite(MOTOR_B_IN2, HIGH);
    } else {
        digitalWrite(MOTOR_B_IN1, LOW);
        digitalWrite(MOTOR_B_IN2, LOW);
    }
    analogWrite(MOTOR_B_PWM, abs(speed));
}

void setup() {
    Serial.begin(115200);
    Wire.begin(SDA_PIN, SCL_PIN);
    
    setup_motors();
    setup_fuzzy();
    
    byte status = mpu.begin();
    if (status != 0) {
        Serial.println("MPU6050 Error!");
        while (1);
    }
    
    Serial.println("Calibrating MPU6050...");
    mpu.calcOffsets(true, true);
    Serial.println("Ready!");
    last_time = millis();
}

void loop() {
    mpu.update();
    unsigned long now = millis();
    float dt = (now - last_time) / 1000.0;
    
    if (dt >= 0.01) { // Chạy ở tần số 100Hz
        current_angle = mpu.getAngleX();
        angle_velocity = (current_angle - last_angle) / dt;
        
        // --- THỰC THI FUZZY LOGIC ---
        // Input 1: Sai số góc (Error)
        fuzzy->setInput(1, current_angle); 
        // Input 2: Vận tốc góc (dError)
        fuzzy->setInput(2, angle_velocity);
        
        fuzzy->fuzzify();
        
        // Output 1: PWM
        float output_pwm = fuzzy->defuzzify(1);
        
        // Điều khiển động cơ
        // Nếu xe bị đổ quá 45 độ thì ngắt động cơ cho an toàn
        if (abs(current_angle) > 45) {
            set_motor_speed(0);
        } else {
            set_motor_speed((int)output_pwm);
        }
        
        // Cập nhật biến
        last_angle = current_angle;
        last_time = now;

        // Debug
        /*
        Serial.print("Angle: "); Serial.print(current_angle);
        Serial.print(" | Vel: "); Serial.print(angle_velocity);
        Serial.print(" | PWM: "); Serial.println(output_pwm);
        */
    }
}
