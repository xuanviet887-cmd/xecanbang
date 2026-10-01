#include <Arduino.h>
#include <Wire.h>
#include <MPU6050_light.h>
#include <Fuzzy.h>

// --- CẤU HÌNH PIN (Theo bảng cung cấp) ---
#define ENA_PIN 25 // PWM Motor 1
#define ENB_PIN 26 // PWM Motor 2
#define IN12_PIN 12 // Direction Motor 1
#define IN34_PIN 14 // Direction Motor 2

// --- CẤU HÌNH BÙ SAI SỐ ĐỘNG CƠ (Cần chỉnh thực tế) ---
#define MOTOR1_OFFSET 5  // Bù cho động cơ 1
#define MOTOR2_OFFSET 10 // Bù cho động cơ 2

// --- CẤU HÌNH PID GỐC ---
float Kp_base = 10.0;
float Ki_base = 0.1;
float Kd_base = 2.0;

// --- BIẾN ĐIỀU KHIỂN ---
float Kp, Ki, Kd;
float error, last_error, integral;
unsigned long last_time;

// --- ĐỐI TƯỢNG ---
MPU6050 mpu(Wire);
Fuzzy *fuzzyTuner = new Fuzzy();

// --- ĐỊNH NGHĨA CÁC TẬP MỜ (Global) ---
FuzzySet *eNB, *eNS, *eZE, *ePS, *ePB;
FuzzySet *kpSMALL, *kpLARGE, *kdSMALL, *kdLARGE;

// --- HÀM ĐIỀU KHIỂN MOTOR (Đã bao gồm OFFSET) ---
void set_motor_speed(int pwm_input) {
    pwm_input = constrain(pwm_input, -255, 255);
    digitalWrite(IN12_PIN, pwm_input > 0);
    digitalWrite(IN34_PIN, pwm_input > 0);
    
    // Tính toán PWM có bù Offset
    int speed1 = abs(pwm_input) + MOTOR1_OFFSET;
    int speed2 = abs(pwm_input) + MOTOR2_OFFSET;

    analogWrite(ENA_PIN, constrain(speed1, 0, 255));
    analogWrite(ENB_PIN, constrain(speed2, 0, 255));
}

// --- KHỞI TẠO FUZZY TUNER ---
void setup_fuzzy_tuner() {
    FuzzyInput *err = new FuzzyInput(1);
    eNB = new FuzzySet(-20, -20, -15, -10);
    eNS = new FuzzySet(-15, -10, -5, 0);
    eZE = new FuzzySet(-5, 0, 0, 5);
    ePS = new FuzzySet(0, 5, 10, 15);
    ePB = new FuzzySet(10, 15, 20, 20);
    err->addFuzzySet(eNB); err->addFuzzySet(eNS); err->addFuzzySet(eZE);
    err->addFuzzySet(ePS); err->addFuzzySet(ePB);
    fuzzyTuner->addFuzzyInput(err);

    FuzzyOutput *dKp = new FuzzyOutput(1);
    kpSMALL = new FuzzySet(0, 0, 2, 4);
    kpLARGE = new FuzzySet(2, 6, 10, 10);
    dKp->addFuzzySet(kpSMALL); dKp->addFuzzySet(kpLARGE);
    fuzzyTuner->addFuzzyOutput(dKp);

    FuzzyOutput *dKd = new FuzzyOutput(2);
    kdSMALL = new FuzzySet(0, 0, 1, 2);
    kdLARGE = new FuzzySet(1, 2, 3, 3);
    dKd->addFuzzySet(kdSMALL); dKd->addFuzzySet(kdLARGE);
    fuzzyTuner->addFuzzyOutput(dKd);

    // Luật 1: Cân bằng tốt -> P nhỏ, D lớn
    FuzzyRuleAntecedent *ifZE = new FuzzyRuleAntecedent(); ifZE->joinSingle(eZE);
    FuzzyRuleConsequent *thenZE = new FuzzyRuleConsequent();
    thenZE->addOutput(kpSMALL); thenZE->addOutput(kdLARGE);
    fuzzyTuner->addFuzzyRule(new FuzzyRule(1, ifZE, thenZE));

    // Luật 2: Lệch lớn -> P lớn, D nhỏ
    FuzzyRuleAntecedent *ifPB = new FuzzyRuleAntecedent(); ifPB->joinSingle(ePB);
    FuzzyRuleConsequent *thenPB = new FuzzyRuleConsequent();
    thenPB->addOutput(kpLARGE); thenPB->addOutput(kdSMALL);
    fuzzyTuner->addFuzzyRule(new FuzzyRule(2, ifPB, thenPB));
    
    FuzzyRuleAntecedent *ifNB = new FuzzyRuleAntecedent(); ifNB->joinSingle(eNB);
    FuzzyRuleConsequent *thenNB = new FuzzyRuleConsequent();
    thenNB->addOutput(kpLARGE); thenNB->addOutput(kdSMALL);
    fuzzyTuner->addFuzzyRule(new FuzzyRule(3, ifNB, thenNB));
}

void setup() {
    Serial.begin(115200);
    pinMode(ENA_PIN, OUTPUT); pinMode(ENB_PIN, OUTPUT);
    pinMode(IN12_PIN, OUTPUT); pinMode(IN34_PIN, OUTPUT);
    
    Wire.begin(21, 22);
    mpu.begin();
    mpu.calcOffsets(true, true);
    
    setup_fuzzy_tuner();
    last_time = millis();
}

void loop() {
    unsigned long now = millis();
    float dt = (now - last_time) / 1000.0;
    if (dt < 0.01) return;
    last_time = now;

    mpu.update();
    float current_angle = mpu.getAngleX();

    // 1. Kiểm tra an toàn
    if (abs(current_angle) > 40.0f) {
        set_motor_speed(0);
        integral = 0;
        return;
    }

    // 2. Chạy Fuzzy Tuner để điều chỉnh PID
    error = 0 - current_angle;
    fuzzyTuner->setInput(1, error);
    fuzzyTuner->fuzzify();
    
    Kp = Kp_base + fuzzyTuner->defuzzify(1);
    Kd = Kd_base + fuzzyTuner->defuzzify(2);
    Ki = Ki_base; // Tạm thời giữ Ki cố định

    // 3. Tính PID
    integral = constrain(integral + error * dt, -50, 50); // Chống windup
    float derivative = (error - last_error) / dt;
    float output_pid = (Kp * error) + (Ki * integral) + (Kd * derivative);
    
    set_motor_speed((int)output_pid);
    last_error = error;
}