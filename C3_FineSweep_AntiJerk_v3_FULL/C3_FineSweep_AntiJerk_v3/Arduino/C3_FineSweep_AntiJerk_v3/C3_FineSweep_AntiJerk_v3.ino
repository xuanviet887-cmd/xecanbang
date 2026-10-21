/* C3 FineSweep AntiJerk v3: JY61 UART, latest measured motor models, BLE diagnostics.
   No automatic motor start. Z=stationary zero. M=10s motor-off trace,
   B/C=3s prep then10s bench/floor. X/BOOT/BLE loss stops immediately.
   On-device trace includes every control tick; BLE live is decimated.
   Rsession,index downloads a frozen row, four CRC-checked20byte packets.
*/
#include <Arduino.h>
#include <esp_arduino_version.h>
#include <math.h>
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>

// TraceWire.h embedded below so this sketch needs ONE .ino only.

#include <stdint.h>
#include <string.h>
#include <math.h>
namespace Trace {
constexpr int SIZE=48,MAX_ROWS=2001;
inline void u16(uint8_t*p,uint32_t v){p[0]=v;p[1]=v>>8;}
inline void u32(uint8_t*p,uint32_t v){for(int i=0;i<4;i++)p[i]=v>>(8*i);}
inline void fixed(uint8_t*p,float v,float scale){long n=isfinite(v)?lroundf(v*scale):0;if(n<-32768)n=-32768;if(n>32767)n=32767;u16(p,(uint16_t)(int16_t)n);}
inline uint16_t crc(const uint8_t*p,int n){uint16_t c=0xffff;while(n--){c^=uint16_t(*p++)<<8;for(int i=0;i<8;i++)c=(c&0x8000)?(c<<1)^0x1021:c<<1;}return c;}
inline void chunk(const uint8_t*row,uint16_t session,uint16_t index,int part,uint8_t*out,bool live=false){out[0]=(live?0xb0:0xa0)+part;out[1]=1;u16(out+2,session);u16(out+4,index);u16(out+6,crc(row,SIZE));memcpy(out+8,row+part*12,12);}
}

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#ifndef LAB_NATIVE
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#endif

// ================= CẤU HÌNH CHIỀU & TRỤC CẢM BIẾN =================
// QUAN TRỌNG: Nhấc xe lên, nghiêng đầu xe về phía trước:
// - Nếu 2 bánh quay CÙNG chiều nghiêng (quay tiến): giữ IMU_SIGN = +1.0f
// - Nếu 2 bánh quay NGƯỢC chiều nghiêng (quay lùi): đổi IMU_SIGN = -1.0f
// - Nếu nghiêng trước/sau bánh không quay mà nghiêng trái/phải mới quay: đổi USE_X_AXIS = true
constexpr float IMU_SIGN = +1.0f;
constexpr bool USE_X_AXIS = false;

constexpr int PWM_PIN[2] = {0, 1}, DIR_PIN[2] = {3, 21}, MOTOR_SIGN[2] = {-1, +1};
constexpr int ENCODER_A[2] = {7, 20}, ENCODER_B[2] = {4, 10};
constexpr int ENC_SIGN[2] = {+1, +1};
constexpr float CPR[2] = {495.1f, 496.4f};
constexpr float WHEEL_DIAMETER_CM = 6.74f, TRACK_CM = 18.22f;
constexpr int JY_RX_PIN = 6, JY_TX_PIN = 5, STOP_PIN = 9, LED_PIN = 8;
constexpr bool USE_LED = true;

// ================= THÔNG SỐ ĐIỀU KHIỂN PD & ĐỘNG CƠ =================
// Tăng KP và KD để tạo mô-men cứu xe tức thời ở góc nghiêng nhỏ (1..3 độ)
constexpr float KP_THETA = 10.5f;
constexpr float KD_THETA = 0.25f;
constexpr float THETA_TRIM_DEG = 0.0f;

// Nguong da do khi ke banh, chua xac nhan duoi tai tren san.
// PWM startup thresholds measured in free wheel tests; must be checked under load.
constexpr float START_PWM[2] = {120.0f, 110.0f};
constexpr float REVERSE_PWM[2] = {130.0f, 120.0f};
constexpr float MOTOR_ON_CM_EQ_S=2.0f,MOTOR_OFF_CM_EQ_S=0.9f;
// Software smoothing only near upright: quick rescue remains enabled if tilt/rate large.
constexpr float FAST_RESCUE_ANGLE_DEG=1.8f;
constexpr float FAST_RESCUE_RATE_DEG_S=30.0f;
constexpr int SLEW_RISE_NEAR=22, SLEW_FALL_NEAR=30;
constexpr int SLEW_RISE_FAST=75, SLEW_FALL_FAST=90;
constexpr uint32_t REVERSE_NEUTRAL_US=5000UL;
constexpr float GYRO_FILTER_TAU_S=0.010f;
constexpr int PWM_MAX = 220; // Giu gioi han220 theo yeu cau truoc.
// Safety: cut both motors if measured pitch exceeds this limit during any PD experiment.
constexpr float FALL_LIMIT_DEG = 35.0f;
uint32_t controlPeriodUs=2500; // 2.5ms default; app may change to 2.5..10.0ms
constexpr uint32_t PERIOD_MIN_US = 2500;
constexpr uint32_t PERIOD_MAX_US = 10000;
constexpr uint32_t PERIOD_STEP_US = 100;
constexpr uint32_t TRACE_DURATION_US = 4500000UL; // Same test duration at all Ts
constexpr uint32_t MAX_CONTROL_US=25000;

// Vòng PI tốc độ ngoài, tạo góc đích để hãm/trả xe về tốc độ đặt.
constexpr bool ENABLE_SPEED_PI = true;
constexpr float KP_SPEED = 0.022f, KI_SPEED = 0.006f;
constexpr float SPEED_TARGET_CM_S = 0.0f, SPEED_ANGLE_MAX_DEG = 2.5f;
constexpr float SPEED_FILTER_TAU_S = 0.18f, REF_SLEW_DEG_PER_S = 3.0f;
constexpr uint32_t SPEED_PI_DELAY_MS = 0, SPEED_US = 50000;

// JY61 UART: 11 bytes, 0x55 + type + 8 payload bytes + additive checksum.
struct JY61Parser {
  uint8_t buffer[11] = {};
  size_t used = 0;
  uint32_t valid = 0, bad = 0;
  int16_t accel[3] = {}, gyro[3] = {}, angle[3] = {};

  static int16_t le16(const uint8_t* p) {
    return static_cast<int16_t>(uint16_t(p[0]) | (uint16_t(p[1]) << 8));
  }

  uint8_t feed(uint8_t c) {
    if (!used && c != 0x55) return 0;
    buffer[used++] = c;
    if (used < 11) return 0;
    uint8_t sum = 0;
    for (int i = 0; i < 10; i++) sum = uint8_t(sum + buffer[i]);
    uint8_t type = buffer[1];
    if (sum == buffer[10] && type >= 0x50 && type <= 0x53) {
      int16_t* out = (type == 0x51) ? accel : (type == 0x52) ? gyro : angle;
      if (type != 0x50) {
        for (int i = 0; i < 3; i++) out[i] = le16(buffer + 2 + i * 2);
      }
      used = 0;
      valid++;
      return type;
    }
    bad++;
    size_t start = 1;
    while (start < 11 && buffer[start] != 0x55) start++;
    used = 11 - start;
    for (size_t i = 0; i < used; i++) buffer[i] = buffer[start + i];
    return 0;
  }
};

namespace Lab {
inline float clamp(float x, float lo, float hi) { return x < lo ? lo : x > hi ? hi : x; }

struct Model {
  float a = 0, b = 0, lo = 0;
};

struct BalanceConfig {
  float kp = KP_THETA, kd = KD_THETA, target = 0;
  int maxPwm = PWM_MAX;
};

struct BalancePD {
  BalanceConfig config;
  float angle = 0, rate = 0, target = 0, p = 0, d = 0, u = 0;
  bool initialized = false;

  void reset() {
    angle = rate = p = d = u = 0;
    target = config.target;
    initialized = false;
  }

  float step(float theta, float gyro, float dt, float targetAngle) {
    if (!initialized) {
      angle = theta;
      rate = gyro;
      initialized = true;
    }
    // Lấy trực tiếp góc và gyro với lọc cực nhẹ để không trễ pha
    angle += dt / (0.002f + dt) * (theta - angle);
    rate  += dt / (GYRO_FILTER_TAU_S + dt) * (gyro - rate);
    target = targetAngle;
    p = config.kp * (angle - target);
    d = config.kd * rate;
    u = clamp(p + d, -55.0f, 55.0f);
    return u;
  }
};

// Pure target mapping: threshold is a floor, never a substituted intercept.
inline int computeDirectPwm(float command,int w,const Model& model,int maxPwm,bool confirmed=false){
  if(!isfinite(command)||fabsf(command)<MOTOR_OFF_CM_EQ_S||maxPwm<1)return 0;
  int dir=command>0?1:-1;
  // Never drop floor to 55 merely because encoder briefly reports motion:
  // this abrupt torque decrease was a major suspect for ON/OFF chatter.
  float lower=(dir>0?START_PWM[w]:REVERSE_PWM[w]);
  float pwm=fmaxf(lower,model.a*fabsf(command)+model.b);
  return dir*lroundf(clamp(pwm,0,float(maxPwm)));
}
struct MotorMap {
  bool enabled=false,confirmed=false;int direction=0;int32_t base=0;
  void reset(){enabled=confirmed=false;direction=0;base=0;}
  int step(float command,int w,const Model& model,int maxPwm,bool moving,int32_t net){
    float magnitude=fabsf(command);
    if(!isfinite(command)||magnitude<MOTOR_OFF_CM_EQ_S){reset();return 0;}
    int dir=command>0?1:-1;
    if(!enabled||direction!=dir){
      if(magnitude<MOTOR_ON_CM_EQ_S){reset();return 0;}
      enabled=true;confirmed=false;direction=dir;base=net;
    }
    if(!confirmed&&moving&&(int64_t(net)-base)*dir>=3)confirmed=true;
    if(confirmed&&!moving){confirmed=false;base=net;}
    return computeDirectPwm(command,w,model,maxPwm,confirmed);
  }
};
} // namespace Lab

// ================= STATE / ENCODER =================
enum RunState : uint8_t { WAIT_SENSOR=0, IDLE=1, CALIBRATING=2, PREPARING=3, RUNNING=4, FAULTED=5 };
RunState state = WAIT_SENSOR;
uint8_t faultCode = 0;
HardwareSerial jy(1);
JY61Parser imu;
Lab::BalancePD pd;
Lab::Model models[4];
Lab::MotorMap motorMap[2];
int32_t motionNet[2]={0,0};uint32_t motionAt[2]={0,0};int motionSign[2]={0,0};
portMUX_TYPE encoderMux = portMUX_INITIALIZER_UNLOCKED;
volatile int32_t encoder[2] = {0, 0};
volatile uint32_t absEncoder[2] = {0, 0};
int32_t speedCount[2] = {0, 0};
int actualPWM[2] = {0, 0}, actualDir[2] = {0, 0};
// Output slew and direction guard, independent from the motor model.
int lastMotorDirection[2] = {0, 0};
uint32_t motorZeroSinceUs[2] = {0, 0};
int pwmRequested[2] = {0, 0};
bool pwmReady[2] = {false, false};
bool autoZeroAttempted=false;
uint32_t bootAt = 0, baudAt = 0, baud = 115200, lastAngleAt = 0, lastGyroAt = 0;
uint32_t rateAt = 0, angleFrames = 0, gyroFrames = 0, angleHz = 0, gyroHz = 0;
uint32_t controlAtUs = 0, speedAtUs = 0, armedAt = 0, prepareAt = 0;
float zeroAngle = 0, gyroBias = 0;
float speedFiltered[2] = {0, 0}, speedAverage = 0, speedIntegral = 0, speedRef = 0, speedApplied = 0;
bool outputSaturated = false;
uint32_t calBegin = 0, calLastAngle = 0, calSessionAt=0;
int calSamples = 0;
double calAngle = 0, calAngle2 = 0, calGyro = 0, calGyro2 = 0;
float calOrigin = 0;
int32_t calCounts[2] = {0, 0};
uint32_t calAbs[2] = {0, 0};

// Quản lý nút bấm BOOT (GPIO9): Bấm nhả nhanh = Lấy lại góc 0; Nhấn giữ = Dừng
bool btnPressed = false;
uint32_t btnDownAt = 0;
bool btnLongHandled = false;

// Trace status is independent of phone/display cadence.
uint8_t traceMode=0;bool zeroValid=false,releaseMarked=false;
uint16_t traceSession=0,rowCount=0;uint32_t captureAtUs=0;
uint8_t traceRows[Trace::MAX_ROWS][Trace::SIZE]={};uint8_t liveRow[Trace::SIZE]={};
uint32_t previousAngleUs=0,previousGyroUs=0,anglePeriodUs=0,gyroPeriodUs=0,angleSequence=0;
int32_t logNet[2]={0,0};uint32_t telemetryAt=0;
volatile bool connected=false,advertiseAgain=false;
volatile bool bleNotifyReady=false, bleConnectedEvent=false, bleDisconnectedEvent=false;
volatile uint32_t heartbeatAt=0, bleDisconnectedAt=0;
portMUX_TYPE commandMux=portMUX_INITIALIZER_UNLOCKED;
char commands[8][20]={};uint8_t commandCount=0;
BLECharacteristic* tx=nullptr;
struct RadioBatch{uint8_t count=0;uint8_t packets[4][20];};
void sendBatch(const RadioBatch&b);
#ifndef LAB_NATIVE
QueueHandle_t radioQueue=nullptr;
void radioTask(void*) {
  RadioBatch b;
  for (;;) {
    if (xQueueReceive(radioQueue, &b, portMAX_DELAY) != pdTRUE) continue;
    if (!connected || !bleNotifyReady || !tx) continue;
    for (int i=0; i<b.count && connected && bleNotifyReady; i++) {
      tx->setValue(b.packets[i],20);
      tx->notify();
      vTaskDelay(1); // Let BLE stack process notifications; not a PD-loop delay.
    }
  }
}
#endif
void sendBatch(const RadioBatch&b){
#ifdef LAB_NATIVE
  if(tx)for(int i=0;i<b.count;i++){tx->setValue((uint8_t*)b.packets[i],20);tx->notify();}
#else
  if(radioQueue&&connected&&bleNotifyReady)xQueueSend(radioQueue,&b,0);
#endif
}
void sendStatus(){
  RadioBatch b;b.count=2;auto*p=b.packets[0];memset(p,0,20);p[0]=0x90;p[1]=1;p[2]=state;p[3]=faultCode;Trace::u16(p+4,rowCount);Trace::u16(p+6,traceSession);p[8]=controlPeriodUs/100;p[9]=zeroValid;Trace::u16(p+10,angleHz);Trace::u16(p+12,gyroHz);p[14]=traceMode;p[15]=releaseMarked;Trace::u16(p+16,calSamples);p[18]=2; // protocol v2: period in units 0.1 ms
  p=b.packets[1];memset(p,0,20);p[0]=0x91;p[1]=1;Trace::u16(p+2,traceSession);Trace::fixed(p+4,pd.config.kp,100);Trace::fixed(p+6,pd.config.kd,1000);Trace::fixed(p+8,pd.config.target,100);Trace::u16(p+10,pd.config.maxPwm);Trace::u16(p+12,controlPeriodUs/100);p[14]=traceMode;Trace::fixed(p+16,zeroAngle,100);Trace::fixed(p+18,gyroBias,1000);sendBatch(b);
}
class LinkCallbacks:public BLEServerCallbacks{
  void onConnect(BLEServer*) override {
    connected=true;
    bleNotifyReady=false; // Wait for app CCCD registration, then its first H command.
    heartbeatAt=millis();
    bleConnectedEvent=true;
  }
  void onDisconnect(BLEServer*) override {
    connected=false;
    bleNotifyReady=false;
    bleDisconnectedAt=millis();
    bleDisconnectedEvent=true;
    advertiseAgain=true;
  }
};
class CommandCallbacks:public BLECharacteristicCallbacks{
  void onWrite(BLECharacteristic*c)override{
    auto value=c->getValue();if(value.length()<1||value.length()>19)return;
    if(value.length()==1&&value.c_str()[0]=='H'){heartbeatAt=millis();bleNotifyReady=true;return;}
    portENTER_CRITICAL(&commandMux);
    if(value.length()==1&&value.c_str()[0]=='X'){commandCount=1;strcpy(commands[0],"X");}
    else if(commandCount<8&&(commandCount==0||commands[0][0]!='X')){memcpy(commands[commandCount],value.c_str(),value.length());commands[commandCount++][value.length()]=0;}
    portEXIT_CRITICAL(&commandMux);
  }
};

void ARDUINO_ISR_ATTR encoderLeftISR() {
  int step = digitalRead(ENCODER_B[0]) ? -1 : +1;
  portENTER_CRITICAL_ISR(&encoderMux);
  encoder[0] += ENC_SIGN[0] * step;
  absEncoder[0]++;
  portEXIT_CRITICAL_ISR(&encoderMux);
}

void ARDUINO_ISR_ATTR encoderRightISR() {
  int step = digitalRead(ENCODER_B[1]) ? -1 : +1;
  portENTER_CRITICAL_ISR(&encoderMux);
  encoder[1] += ENC_SIGN[1] * step;
  absEncoder[1]++;
  portEXIT_CRITICAL_ISR(&encoderMux);
}

void readCounts(int32_t* net, uint32_t* total) {
  portENTER_CRITICAL(&encoderMux);
  for (int w = 0; w < 2; w++) {
    net[w] = encoder[w];
    total[w] = absEncoder[w];
  }
  portEXIT_CRITICAL(&encoderMux);
}

float wrapDegrees(float value) {
  while (value > 180) value -= 360;
  while (value < -180) value += 360;
  return value;
}

float rawAngleDeg() {
  int16_t raw = USE_X_AXIS ? imu.angle[0] : imu.angle[1];
  return IMU_SIGN * (raw * 180.0f / 32768.0f);
}

float rawGyroDegS() {
  int16_t raw = USE_X_AXIS ? imu.gyro[0] : imu.gyro[1];
  return IMU_SIGN * (raw * 2000.0f / 32768.0f);
}

bool freshSensor(uint32_t now) {
  return lastAngleAt && lastGyroAt && uint32_t(now - lastAngleAt) <= 40 && uint32_t(now - lastGyroAt) <= 40;
}

void setLed(bool on) {
  if (USE_LED) digitalWrite(LED_PIN, on ? LOW : HIGH);
}

void writeDuty(int w, int magnitude) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(PWM_PIN[w], magnitude);
#else
  ledcWrite(w, magnitude);
#endif
}

// Chỉ ngắt PWM về 0 khi ĐỔI CHIỀU quay, không ngắt khi đang tăng/giảm tốc cùng chiều
void writeMotor(int w, int value) {
  if (!pwmReady[w]) return;
  value = (int)Lab::clamp(value, -pd.config.maxPwm, pd.config.maxPwm);
  if (value == actualPWM[w]) return;

  if (value == 0) {
    writeDuty(w, 0);
    actualPWM[w] = 0;
    actualDir[w] = 0;
    return;
  }

  int dir = (value * MOTOR_SIGN[w] > 0) ? 1 : -1;
  if (dir != actualDir[w]) {
    writeDuty(w, 0);
    digitalWrite(DIR_PIN[w], dir > 0 ? HIGH : LOW);
    actualDir[w] = dir;
  }
  writeDuty(w, abs(value));
  actualPWM[w] = value;
}

// Rate-limit sudden 0->120 jumps and reversals near balance.
// Safety stopMotors() always bypasses this limiter.
int smoothCommandPwm(int w, int target, bool urgent, uint32_t nowUs) {
  pwmRequested[w] = target;
  int previous = actualPWM[w];
  const int previousSign = (previous > 0) - (previous < 0);
  int wantedSign = (target > 0) - (target < 0);
  if (previousSign && wantedSign && previousSign != wantedSign) {
    // Drive to zero first, do not instantly switch H-bridge direction.
    target = 0;
    wantedSign = 0;
  }
  if (!previousSign && wantedSign && lastMotorDirection[w] &&
      wantedSign != lastMotorDirection[w] &&
      (uint32_t)(nowUs - motorZeroSinceUs[w]) < REVERSE_NEUTRAL_US) {
    target = 0; // Maintain brief neutral interval (non-blocking).
    wantedSign = 0;
  }
  const int rise = urgent ? SLEW_RISE_FAST : SLEW_RISE_NEAR;
  const int fall = urgent ? SLEW_FALL_FAST : SLEW_FALL_NEAR;
  int desired = abs(target), current = abs(previous);
  // Avoid going through opposite direction in the same control tick.
  if (previousSign && wantedSign && previousSign != wantedSign) desired = 0;
  int nextMag = current;
  if (desired > current) nextMag = current + (rise < desired-current ? rise : desired-current);
  else if (desired < current) nextMag = current - (fall < current-desired ? fall : current-desired);
  const int nextSign = (nextMag == 0) ? 0 : (previousSign ? previousSign : wantedSign);
  int next = nextSign * nextMag;
  if (previous != 0 && next == 0) {
    lastMotorDirection[w] = previousSign;
    motorZeroSinceUs[w] = nowUs;
  } else if (previous == 0 && next != 0) {
    lastMotorDirection[w] = nextSign;
  }
  return next;
}

void stopMotors() {
  for (int w = 0; w < 2; w++) {
    if (pwmReady[w]) writeDuty(w, 0);
    actualPWM[w] = 0;
    actualDir[w] = 0;
    motorMap[w].reset();
    lastMotorDirection[w] = 0;
    motorZeroSinceUs[w] = 0;
    pwmRequested[w] = 0;
  }
}

void latchFault(uint8_t code) {
  stopMotors();
  faultCode = code;
  state = FAULTED;
  pd.reset();
  speedIntegral = speedRef = speedApplied = 0;
}

void showState(uint32_t now) {
  if (state == RUNNING) {
    setLed(true);
    return;
  }
  if (state == FAULTED) {
    uint32_t cycle = 1000UL + faultCode * 350UL;
    uint32_t phase = uint32_t(now - bootAt) % cycle;
    setLed(phase < faultCode * 350UL && phase % 350 < 150);
    return;
  }
  setLed((now / (state == CALIBRATING ? 120 : 400)) % 2 == 0);
}

void openBaud(uint32_t value, uint32_t now) {
  jy.end();
  imu.used = 0;
  baud = value;
  jy.begin(baud, SERIAL_8N1, JY_RX_PIN, JY_TX_PIN);
  baudAt = rateAt = now;
  lastAngleAt = lastGyroAt = 0;
  angleFrames = gyroFrames = angleHz = gyroHz = 0;
}

void pollSensor(uint32_t now) {
  int budget = 512;
  while (budget-- && jy.available()) {
    uint8_t kind = imu.feed((uint8_t)jy.read());
    if (kind == 0x53) {
      uint32_t stamp=micros();if(previousAngleUs)anglePeriodUs=stamp-previousAngleUs;previousAngleUs=stamp;angleSequence++;
      lastAngleAt = now;
      angleFrames++;
    } else if (kind == 0x52) {
      uint32_t stamp=micros();if(previousGyroUs)gyroPeriodUs=stamp-previousGyroUs;previousGyroUs=stamp;
      lastGyroAt = now;
      gyroFrames++;
    }
  }
  uint32_t elapsed = now - rateAt;
  if (elapsed >= 1000) {
    angleHz = angleFrames * 1000UL / elapsed;
    gyroHz = gyroFrames * 1000UL / elapsed;
    angleFrames = gyroFrames = 0;
    rateAt = now;
  }
  if (state == WAIT_SENSOR && uint32_t(now - baudAt) > 2500 && !freshSensor(now)) {
    openBaud(baud == 115200 ? 9600 : 115200, now);
  }
}

void resetCalibration(uint32_t now) {
  calBegin = now;
  calLastAngle = 0;
  calSamples = 0;
  calAngle = calAngle2 = calGyro = calGyro2 = 0;
  calOrigin = rawAngleDeg();
  readCounts(calCounts, calAbs);
}

void armMotors(uint32_t now) {
  int32_t net[2];
  uint32_t total[2];
  readCounts(net, total);
  for (int w = 0; w < 2; w++) {
    speedCount[w] = net[w];
    speedFiltered[w] = 0;
    motorMap[w].reset();motionNet[w]=net[w];motionAt[w]=now;motionSign[w]=0;
  }
  pd.reset();
  speedAverage = speedIntegral = speedRef = speedApplied = 0;
  outputSaturated = false;
  armedAt = now;
  controlAtUs = speedAtUs = micros();
  state = RUNNING;
  captureAtUs=micros();for(int w=0;w<2;w++)logNet[w]=net[w];
}

// Re-zero is a real calibration, never a single sample and gyroBias=0.
void quickReZero(uint32_t now){
  stopMotors();pd.reset();speedIntegral=speedRef=speedApplied=0;
  if(!pwmReady[0]||!pwmReady[1]){latchFault(1);return;}
  if(!freshSensor(now)||angleHz<50||gyroHz<50){latchFault(5);return;}
  faultCode=0;state=CALIBRATING;calSessionAt=now;resetCalibration(now);
}
void handleBootButton(uint32_t now){if(digitalRead(STOP_PIN)==LOW&&(state==RUNNING||state==PREPARING||state==CALIBRATING))latchFault(10);}

void takeZero(uint32_t now){
  if(!freshSensor(now)||lastAngleAt==calLastAngle)return;
  calLastAngle=lastAngleAt;
  int32_t net[2];uint32_t total[2];readCounts(net,total);
  float a=wrapDegrees(rawAngleDeg()-calOrigin),g=rawGyroDegS();
  if(total[0]!=calAbs[0]||total[1]!=calAbs[1]||fabsf(g)>5||fabsf(a)>2){resetCalibration(now);return;}
  calSamples++;calAngle+=a;calAngle2+=double(a)*a;calGyro+=g;calGyro2+=double(g)*g;
  if(uint32_t(now-calBegin)<1500||calSamples<60)return;
  double meanA=calAngle/calSamples,meanG=calGyro/calSamples;
  float sdA=sqrtf(fmaxf(0,float(calAngle2/calSamples-meanA*meanA)));
  float sdG=sqrtf(fmaxf(0,float(calGyro2/calSamples-meanG*meanG)));
  if(sdA>.35f||sdG>1.5f){resetCalibration(now);return;}
  zeroAngle=wrapDegrees(calOrigin+meanA);gyroBias=meanG;
  pd.reset();zeroValid=true;faultCode=0;state=IDLE;
}

void updateSpeed(uint32_t nowUs, uint32_t now) {
  uint32_t elapsed = nowUs - speedAtUs;
  if (elapsed < SPEED_US) return;
  float dt = elapsed * 1e-6f;
  speedAtUs = nowUs;
  int32_t net[2];
  uint32_t total[2];
  readCounts(net, total);
  float beta = dt / (SPEED_FILTER_TAU_S + dt);
  for (int w = 0; w < 2; w++) {
    int32_t delta = net[w] - speedCount[w];
    speedCount[w] = net[w];
    float measured = delta * (PI * WHEEL_DIAMETER_CM) / (CPR[w] * dt);
    speedFiltered[w] += beta * (measured - speedFiltered[w]);
  }
  speedAverage = (speedFiltered[0] + speedFiltered[1]) * 0.5f;
  if (!ENABLE_SPEED_PI || uint32_t(now - armedAt) < SPEED_PI_DELAY_MS) {
    speedIntegral = speedRef = speedApplied = 0;
    return;
  }
  // Forward wheel speed needs a positive angle target so the inner PD commands braking.
  float error = speedAverage - SPEED_TARGET_CM_S;
  float p = KP_SPEED * error, deltaI = KI_SPEED * error * dt;
  float candidate = Lab::clamp(speedIntegral + deltaI, -SPEED_ANGLE_MAX_DEG, SPEED_ANGLE_MAX_DEG);
  float proposed = p + candidate;
  bool clipped = (proposed > SPEED_ANGLE_MAX_DEG && deltaI > 0) || (proposed < -SPEED_ANGLE_MAX_DEG && deltaI < 0);
  if (!clipped && !outputSaturated && fabsf(pd.angle) < 6) speedIntegral = candidate;
  speedRef = Lab::clamp(p + speedIntegral, -SPEED_ANGLE_MAX_DEG, SPEED_ANGLE_MAX_DEG);
}

void controlTick(uint32_t nowUs, uint32_t now) {
  uint32_t elapsed = nowUs - controlAtUs;
  if (elapsed < controlPeriodUs) return;
  controlAtUs = nowUs;
  if(elapsed>(controlPeriodUs*3>MAX_CONTROL_US?controlPeriodUs*3:MAX_CONTROL_US)){latchFault(6);return;}
  if(!freshSensor(now)||angleHz<50||gyroHz<50){latchFault(5);return;}

  float theta = wrapDegrees(rawAngleDeg() - zeroAngle);
  float rate  = rawGyroDegS() - gyroBias;
  if(!isfinite(theta)||!isfinite(rate)){latchFault(7);return;}
  // Fail-safe: never keep driving after the robot has fallen far from upright.
  if(fabsf(theta)>FALL_LIMIT_DEG ||
     (fabsf(theta)>8.0f && fabsf(rate)>300.0f)){
    latchFault(8);return;
  }

  updateSpeed(nowUs, now);
  float dt = elapsed * 1e-6f;
  // Do not hide a late loop by pretending dt was5ms.
  float maxStep = REF_SLEW_DEG_PER_S * dt;
  speedApplied += Lab::clamp(speedRef - speedApplied, -maxStep, maxStep);
  // The speed PI adjusts the app's upright trim without changing its stored setting.

  float command = pd.step(theta, rate, dt, pd.config.target + speedApplied);
  outputSaturated = fabsf(command) >= 50.0f;

  int32_t net[2];uint32_t total[2];readCounts(net,total);
  for (int w = 0; w < 2; w++) {
    if(net[w]!=motionNet[w]){motionSign[w]=net[w]>motionNet[w]?1:-1;motionNet[w]=net[w];motionAt[w]=now;}
    bool moving=uint32_t(now-motionAt[w])<80&&motionSign[w]==(command>0?1:-1);
    int profile = w * 2 + (command < 0 ? 1 : 0);
    int pwmDemand=traceMode==1?0:motorMap[w].step(command,w,models[profile],pd.config.maxPwm,moving,net[w]);
    bool urgent=fabsf(pd.angle)>FAST_RESCUE_ANGLE_DEG || fabsf(pd.rate)>FAST_RESCUE_RATE_DEG_S;
    int pwmOut=smoothCommandPwm(w,pwmDemand,urgent,nowUs);
    outputSaturated |= (abs(pwmDemand) >= pd.config.maxPwm);
    writeMotor(w, pwmOut);
  }
}

void makeRecord(uint8_t*out,uint32_t stamp,uint32_t dt){
  memset(out,0,Trace::SIZE);Trace::u32(out,stamp);Trace::u16(out+4,dt>65535?65535:dt);
  Trace::u16(out+6,lastAngleAt?((millis()-lastAngleAt)>65535?65535:(millis()-lastAngleAt)):65535);
  Trace::u16(out+8,lastGyroAt?((millis()-lastGyroAt)>65535?65535:(millis()-lastGyroAt)):65535);
  Trace::u16(out+10,anglePeriodUs>65535?65535:anglePeriodUs);
  Trace::fixed(out+12,wrapDegrees(rawAngleDeg()-zeroAngle),100);Trace::fixed(out+14,pd.angle,100);Trace::fixed(out+16,pd.target,100);
  Trace::fixed(out+18,rawGyroDegS()-gyroBias,10);Trace::fixed(out+20,pd.rate,10);Trace::fixed(out+22,pd.p,100);Trace::fixed(out+24,pd.d,100);
  Trace::fixed(out+26,pd.p+pd.d,100);Trace::fixed(out+28,pd.u,100);
  for(int w=0;w<2;w++){
    Trace::u16(out+30+4*w,(int16_t)pwmRequested[w]);
    Trace::u16(out+32+4*w,(int16_t)actualPWM[w]);
  }
  int32_t net[2];uint32_t total[2];readCounts(net,total);for(int w=0;w<2;w++){int64_t n=int64_t(net[w])-logNet[w];if(n<-32768)n=-32768;if(n>32767)n=32767;Trace::u16(out+38+2*w,(int16_t)n);logNet[w]=net[w];}
  out[42]=(fabsf(pd.p+pd.d)>55?1:0)|(abs(actualPWM[0])>=pd.config.maxPwm?2:0)|(abs(actualPWM[1])>=pd.config.maxPwm?4:0)|(state==RUNNING&&traceMode!=1?8:0)|(zeroValid?16:0)|(dt>controlPeriodUs*12/10?32:0)|(releaseMarked?64:0)|((fabsf(pd.p)>327.67f||fabsf(pd.d)>327.67f||fabsf(pd.p+pd.d)>327.67f)?128:0);
  out[43]=faultCode;Trace::u16(out+44,angleSequence);Trace::u16(out+46,gyroPeriodUs>65535?65535:gyroPeriodUs);
}
void recordTick(uint32_t stamp,uint32_t dt){makeRecord(liveRow,stamp,dt);if(rowCount<Trace::MAX_ROWS)memcpy(traceRows[rowCount++],liveRow,Trace::SIZE);else latchFault(21);}
void beginTrace(uint8_t selected,uint32_t now){
  if(state!=IDLE&&state!=FAULTED)return;
  if(rowCount){faultCode=24;return;}
  if(!connected||now-heartbeatAt>1000){faultCode=20;return;}
  if(!freshSensor(now)||angleHz<50||gyroHz<50){faultCode=5;return;}
  if(selected!=1&&!zeroValid){faultCode=23;return;}
  stopMotors();traceMode=selected;rowCount=0;traceSession++;faultCode=0;releaseMarked=false;pd.reset();
  if(selected==1)armMotors(now);else{state=PREPARING;prepareAt=now;int32_t net[2];readCounts(net,calAbs);}
}
void command(const char*text,uint32_t now){
  char op=text[0];
  if(op=='X'){if(state==RUNNING){uint32_t t=micros();latchFault(10);recordTick(t,t-controlAtUs);}else if(state==PREPARING||state==CALIBRATING)latchFault(10);else stopMotors();return;}
  if(op=='Q'){sendStatus();return;}
  if(op=='K'){if(state==RUNNING)releaseMarked=true;return;}
  if(op=='R'){
    unsigned session=0,index=0;if(sscanf(text+1,"%u,%u",&session,&index)!=2)return;
    if(state==RUNNING||state==PREPARING||state==CALIBRATING||session!=traceSession||index>=rowCount)return;
    RadioBatch b;b.count=4;for(int part=0;part<4;part++)Trace::chunk(traceRows[index],traceSession,index,part,b.packets[part]);sendBatch(b);return;
  }
  if(state==RUNNING||state==PREPARING||state==CALIBRATING)return;
  if(op=='Z'){if(rowCount){faultCode=24;return;}quickReZero(now);return;}
  if(op=='M'||op=='B'||op=='C'){beginTrace(op=='M'?1:op=='B'?2:3,now);return;}
  if(op=='E'){rowCount=0;traceSession++;faultCode=0;if(state==FAULTED)state=IDLE;sendStatus();return;}
  if(rowCount){faultCode=24;return;}
  char*end=nullptr;float value=strtof(text+1,&end);if(!end||*end||!isfinite(value)){faultCode=22;return;}
  bool ok=true;
  if(op=='P'&&value>=0&&value<=20)pd.config.kp=value;
  else if(op=='D'&&value>=0&&value<=2)pd.config.kd=value;
  else if(op=='O'&&fabsf(value)<=3)pd.config.target=value;
  else if(op=='L'&&value>=110&&value<=255&&value==floorf(value))pd.config.maxPwm=value;
  else if(op=='T'&&value>=2.5f&&value<=10.0f){
    // Accept only exact multiples of 0.1 ms (within ASCII float tolerance).
    float stepsF = value * 10.0f;
    int steps = (int)lroundf(stepsF);
    if(fabsf(stepsF-steps)<0.001f){
      uint32_t candidate = (uint32_t)steps * PERIOD_STEP_US;
      if(candidate>=PERIOD_MIN_US&&candidate<=PERIOD_MAX_US)controlPeriodUs=candidate;
      else ok=false;
    } else ok=false;
  }
  else ok=false;
  faultCode=ok?0:22;sendStatus();
}
void initBle(){
  BLEDevice::init("C3_RESPONSE_LAB");auto*server=BLEDevice::createServer();server->setCallbacks(new LinkCallbacks());
  auto*service=server->createService("6e400001-b5a3-f393-e0a9-e50e24dcca9e");
  tx=service->createCharacteristic("6e400003-b5a3-f393-e0a9-e50e24dcca9e",BLECharacteristic::PROPERTY_NOTIFY);tx->addDescriptor(new BLE2902());
  auto*rx=service->createCharacteristic("6e400002-b5a3-f393-e0a9-e50e24dcca9e",BLECharacteristic::PROPERTY_WRITE);rx->setCallbacks(new CommandCallbacks());service->start();
#ifndef LAB_NATIVE
  radioQueue=xQueueCreate(8,sizeof(RadioBatch));if(!radioQueue){latchFault(1);return;}xTaskCreate(radioTask,"trace_radio",4096,nullptr,1,nullptr);vTaskPrioritySet(nullptr,3);
#endif
  auto*ad=BLEDevice::getAdvertising();ad->addServiceUUID("6e400001-b5a3-f393-e0a9-e50e24dcca9e");ad->setScanResponse(true);ad->start();
}

void setup() {
// Serial/UART0 disabled intentionally. GPIO20/GPIO21 are encoder/motor pins.
  bootAt = millis();
  pinMode(STOP_PIN, INPUT_PULLUP);
  if (USE_LED) {
    pinMode(LED_PIN, OUTPUT);
    setLed(false);
  }
  for (int w = 0; w < 2; w++) {
    digitalWrite(PWM_PIN[w], LOW);
    pinMode(PWM_PIN[w], OUTPUT);
    digitalWrite(DIR_PIN[w], LOW);
    pinMode(DIR_PIN[w], OUTPUT);
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    pwmReady[w] = ledcAttach(PWM_PIN[w], 20000, 8);
#else
    pwmReady[w] = ledcSetup(w, 20000, 8) > 0;
    ledcAttachPin(PWM_PIN[w], w);
#endif
    if (pwmReady[w]) writeDuty(w, 0);
    pinMode(ENCODER_A[w], INPUT_PULLUP);
    pinMode(ENCODER_B[w], INPUT_PULLUP);
  }
  if (!pwmReady[0] || !pwmReady[1]) {
    latchFault(1);
    return;
  }
  attachInterrupt(digitalPinToInterrupt(ENCODER_A[0]), encoderLeftISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A[1]), encoderRightISR, RISING);

  // Model derived from unloaded experiments; not guaranteed under floor load.
  const float a[4] = {4.811015f, 4.789974f, 3.972431f, 4.176047f};
  const float b[4] = {31.626482f, 30.222054f, 66.105141f, 60.363602f};
  const float lo[4] = {9.79f, 10.25f, 10.92f, 9.29f};
  for (int i = 0; i < 4; i++) {
    models[i].a = a[i];
    models[i].b = b[i];
    models[i].lo = lo[i];
  }
  pd.config.kp = KP_THETA;
  pd.config.kd = KD_THETA;
  pd.config.target = THETA_TRIM_DEG;
  pd.config.maxPwm = PWM_MAX;

  jy.setRxBufferSize(512);
  openBaud(115200, millis());
  initBle();
}

void loop(){
  uint32_t now=millis();showState(now);pollSensor(now);bool wasRunningBeforeButton=state==RUNNING;handleBootButton(now);if(wasRunningBeforeButton&&state==FAULTED)recordTick(micros(),micros()-controlAtUs);
  if(bleConnectedEvent){
    bleConnectedEvent=false;
  }
  if(bleDisconnectedEvent){
    bleDisconnectedEvent=false;
#ifndef LAB_NATIVE
    if(radioQueue)xQueueReset(radioQueue);
#endif
  }
  // Delay advertising restart slightly to let the BLE host finish disconnection cleanup.
  if(advertiseAgain && !connected && uint32_t(now-bleDisconnectedAt)>=300){
    advertiseAgain=false;
    BLEDevice::startAdvertising();
  }
  if((state==RUNNING||state==PREPARING)&&(!connected||uint32_t(now-heartbeatAt)>1000)){
    uint32_t t=micros();bool wasRunning=state==RUNNING;latchFault(20);if(wasRunning)recordTick(t,t-controlAtUs);
  }
  char text[20]={};portENTER_CRITICAL(&commandMux);if(commandCount){strcpy(text,commands[0]);for(int i=1;i<commandCount;i++)strcpy(commands[i-1],commands[i]);commandCount--;}portEXIT_CRITICAL(&commandMux);
  if(text[0])command(text,now);
  if(state==WAIT_SENSOR){if(freshSensor(now)&&angleHz>=50&&gyroHz>=50)state=IDLE;}
  // Auto-zero starts NO EARLIER than 5 s after power-on; motors remain disabled.
  // Manual Z remains available for repositioning/recalibration from the phone.
  if(state==IDLE && !zeroValid && !autoZeroAttempted &&
     uint32_t(now-bootAt)>=5000 && freshSensor(now) && angleHz>=50 && gyroHz>=50 &&
     fabsf(rawAngleDeg())<=8.0f && fabsf(rawGyroDegS())<=5.0f) {
    autoZeroAttempted=true;
    quickReZero(now);
  }
  else if(state==CALIBRATING){if(uint32_t(now-calSessionAt)>30000)latchFault(3);else takeZero(now);}
  else if(state==PREPARING){
    if(!freshSensor(now))latchFault(5);
    else {int32_t net[2];uint32_t total[2];readCounts(net,total);if(total[0]!=calAbs[0]||total[1]!=calAbs[1]||fabsf(rawGyroDegS()-gyroBias)>15){prepareAt=now;calAbs[0]=total[0];calAbs[1]=total[1];}else if(uint32_t(now-prepareAt)>=3000)armMotors(now);}
  }else if(state==RUNNING){
    uint32_t before=controlAtUs,stamp=micros();controlTick(stamp,now);
    if(controlAtUs!=before){recordTick(stamp,stamp-before);if(state==RUNNING&&(uint32_t(stamp-captureAtUs)>=TRACE_DURATION_US||rowCount>=Trace::MAX_ROWS-1)){stopMotors();state=IDLE;}}
  }
  if(now-telemetryAt>=200){telemetryAt=now;sendStatus();if((state==RUNNING&&rowCount)||rowCount==0){if(state!=RUNNING)makeRecord(liveRow,micros(),0);RadioBatch b;b.count=4;for(int part=0;part<4;part++)Trace::chunk(liveRow,traceSession,65535,part,b.packets[part],true);sendBatch(b);}}
  // Do not block for a full RTOS tick while RUNNING: short waits reduce
  // overshoot of fractional control periods. BLE radioTask runs separately.
  if (state == RUNNING) {
    uint32_t spent = (uint32_t)(micros() - controlAtUs);
    uint32_t remaining = spent < controlPeriodUs ? controlPeriodUs-spent : 0;
    if (remaining > 700UL) delayMicroseconds(150);
    else if (remaining > 120UL) delayMicroseconds(30);
    // Close to deadline: immediately check again; no Serial/BLE blocking calls.
  } else {
    delay(1);
  }
}
