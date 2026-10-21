#include <Arduino.h>
#include <Wire.h>
#include <MPU6050_light.h>

// --- CẤU HÌNH PIN (Theo sơ đồ phần cứng của bạn) ---
#define ENA_PIN 25 // PWM Motor 1
#define ENB_PIN 26 // PWM Motor 2
#define IN12_PIN 12 // Direction Motor 1
#define IN34_PIN 14 // Direction Motor 2

// --- CẤU HÌNH BÙ SAI SỐ ĐỘNG CƠ (Điều chỉnh thực tế nếu xe bị lệch trái/phải) ---
#define MOTOR1_OFFSET 5  
#define MOTOR2_OFFSET 10 

// --- THÔNG SỐ PID CẦN DÒ (Tune các hệ số này) ---
float Kp = 15.0; // Hệ số tỉ lệ (Proportional)
float Ki = 0.0;  // Hệ số tích phân (Integral) - Thường để 0 hoặc rất nhỏ khi bắt đầu dò
float Kd = 1.5;  // Hệ số vi phân (Derivative)

// --- BIẾN ĐIỀU KHIỂN & DÒ PID ---
float error = 0, last_error = 0, integral = 0;
float target_angle = 0.0; // Góc cân bằng lý tưởng (có thể lệch nhẹ do trọng tâm xe, ví dụ -1.0 đến 1.0 độ)
unsigned long last_time = 0;

// --- ĐỐI TƯỢNG CẢM BIẾN ---
MPU6050 mpu(Wire);

// --- HÀM ĐIỀU KHIỂN MOTOR ---
void set_motor_speed(int pwm_input) {
    pwm_input = constrain(pwm_input, -255, 255);
    
    // Điều khiển chiều quay dựa vào dấu của PWM
    if (pwm_input >= 0) {
        digitalWrite(IN12_PIN, HIGH);
        digitalWrite(IN34_PIN, HIGH);
    } else {
        digitalWrite(IN12_PIN, LOW);
        digitalWrite(IN34_PIN, LOW);
    }
    
    // Tính toán PWM có cộng Offset bù ma sát động cơ
    int speed1 = abs(pwm_input) + MOTOR1_OFFSET;
    int speed2 = abs(pwm_input) + MOTOR2_OFFSET;

    analogWrite(ENA_PIN, constrain(speed1, 0, 255));
    analogWrite(ENB_PIN, constrain(speed2, 0, 255));
}

// --- HÀM ĐỌC LỆNH TỪ SERIAL MONITOR ĐỂ DÒ PID TRỰC TIẾP ---
// Cú pháp nhập trên Serial Monitor (chọn No line ending hoặc Newline):
// P15.5 -> đổi Kp thành 15.5
// I0.2  -> đổi Ki thành 0.2
// D2.1  -> đổi Kd thành 2.1
void parseSerialTuning() {
    if (Serial.available() > 0) {
        char cmd = Serial.read();
        float val = Serial.parseFloat();
        if (cmd == 'p' || cmd == 'P') {
            Kp = val;
            Serial.print("-> Updated Kp = "); Serial.println(Kp);
        } else if (cmd == 'i' || cmd == 'I') {
            Ki = val;
            Serial.print("-> Updated Ki = "); Serial.println(Ki);
        } else if (cmd == 'd' || cmd == 'D') {
            Kd = val;
            Serial.print("-> Updated Kd = "); Serial.println(Kd);
        } else if (cmd == 't' || cmd == 'T') {
            target_angle = val;
            Serial.print("-> Updated Target Angle = "); Serial.println(target_angle);
        }
    }
}

void setup() {
    Serial.begin(115200);
    pinMode(ENA_PIN, OUTPUT); 
    pinMode(ENB_PIN, OUTPUT);
    pinMode(IN12_PIN, OUTPUT); 
    pinMode(IN34_PIN, OUTPUT);
    
    Wire.begin(21, 22);
    mpu.begin();
    
    Serial.println(F("Đang tính toán Offset MPU6050... Giữ yên xe!"));
    delay(1000);
    mpu.calcOffsets(true, true); // Tự động căn chỉnh cảm biến
    Serial.println(F("Sẵn sàng chạy! Nhập Pxx, Ixx, Dxx trên Serial để dò PID."));
    
    last_time = millis();
}

void loop() {
    // Kiểm tra dữ liệu Serial để thay đổi thông số PID không cần nạp lại code
    parseSerialTuning();

    unsigned long now = millis();
    float dt = (now - last_time) / 1000.0;
    if (dt < 0.01) return; // Chu kỳ lấy mẫu 10ms (100Hz)
    last_time = now;

    // Đọc góc từ MPU6050
    mpu.update();
    float current_angle = mpu.getAngleX();

    // 1. Kiểm tra an toàn: Nếu xe đổ quá 45 độ -> Tắt động cơ
    if (abs(current_angle - target_angle) > 45.0f) {
        set_motor_speed(0);
        integral = 0;
        last_error = 0;
        return;
    }

    // 2. Tính toán PID góc nghiêng
    error = target_angle - current_angle;
    
    // Tích phân chống bão hòa (Anti-windup)
    integral = constrain(integral + error * dt, -100, 100); 
    
    float derivative = (error - last_error) / dt;
    
    float output_pid = (Kp * error) + (Ki * integral) + (Kd * derivative);
    
    // Xuất tín hiệu ra động cơ
    set_motor_speed((int)output_pid);
    last_error = error;

    // 3. XUẤT DỮ LIỆU LÊN SERIAL PLOTTER (Tools -> Serial Plotter)
    // Bạn có thể mở Serial Plotter trên Arduino IDE để xem đồ thị góc nghiêng và tín hiệu điều khiển
    Serial.print("Angle:");
    Serial.print(current_angle);
    Serial.print(",Error:");
    Serial.print(error);
    Serial.print(",PWM:");
    Serial.println(output_pid);
}
