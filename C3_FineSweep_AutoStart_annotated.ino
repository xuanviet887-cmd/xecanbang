// ▶ TÀI LIỆU GHI CHÚ TIẾNG VIỆT - code gốc giữ nguyên, chỉ thêm các dòng bắt đầu bằng "// ▶"
//   === TỔNG QUAN ===
//   Xe tự cân bằng 2 bánh trên ESP32-C3: IMU JY61 (UART) -> PD góc nghiêng (vòng trong)
//   + PI tốc độ (vòng ngoài, giữ xe đứng yên tại chỗ) -> PWM 2 motor có encoder.
//   BLE chỉ dùng để chỉnh tham số, xem trạng thái và tải dữ liệu trace.
//   === MÁY TRẠNG THÁI ===
//   WAIT_SENSOR -> IDLE -> (tự calib sau 5s) CALIBRATING -> IDLE(zeroValid)
//     -> (autoStartTick: đứng thẳng & yên 1.5s) RUNNING -> (ngã) FAULTED(8) -> tự chạy lại khi dựng thẳng
//   === MÃ LỖI (faultCode, LED nháy đúng số lần) ===
//    1: PWM/queue khởi tạo lỗi   3: calib quá 30s       5: mất IMU / IMU < 50Hz
//    6: vòng điều khiển trễ      7: NaN                 8: NGÃ
//   10: bấm X/nút BOOT          20: mất BLE (khi không autoRun)
//   21: đầy bộ nhớ trace        22: tham số sai        23: chưa có zero    24: trace chưa xóa
//   === LUỒNG MỖI VÒNG loop() ===
//   đọc IMU -> BLE/lệnh -> xử lý theo state -> (RUNNING: controlTick mỗi 2.5ms) -> autoStartTick -> gửi telemetry
/* C3 FineSweep AntiJerk v3 + AUTO START: JY61 UART, latest measured motor models, BLE diagnostics.
   AUTO_START: bat nguon -> tu calib goc 0 -> tu can bang, khong can app.
   Z=stationary zero. M=10s motor-off trace,
   B/C=3s prep then10s bench/floor. X/BOOT stops immediately.
   On-device trace includes every control tick; BLE live is decimated.
   Rsession,index downloads a frozen row, four CRC-checked 20 byte packets.
*/
// ▶ --- Thư viện hệ thống ---
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
// ▶ Namespace Trace: đóng gói 1 "row" dữ liệu 48 byte thành 4 gói BLE 20 byte để gửi về app.
namespace Trace {
// ▶ SIZE = 48 byte/row. MAX_ROWS = 2001 row (~96KB RAM) ~ 5 giây dữ liệu ở chu kỳ 2.5ms.
constexpr int SIZE=48,MAX_ROWS=2001;
// ▶ Ghi số 16-bit little-endian vào bộ đệm.
inline void u16(uint8_t*p,uint32_t v){p[0]=v;p[1]=v>>8;}
// ▶ Ghi số 32-bit little-endian vào bộ đệm.
inline void u32(uint8_t*p,uint32_t v){for(int i=0;i<4;i++)p[i]=v>>(8*i);}
// ▶ Đổi float -> int16 có dấu: round(v*scale), chặn [-32768,32767]; NaN/Inf -> 0. VD: góc 1.23° với scale 100 -> 123.
inline void fixed(uint8_t*p,float v,float scale){long n=isfinite(v)?lroundf(v*scale):0;if(n<-32768)n=-32768;if(n>32767)n=32767;u16(p,(uint16_t)(int16_t)n);}
// ▶ CRC16-CCITT (poly 0x1021, init 0xFFFF) để app kiểm tra gói không bị lỗi.
inline uint16_t crc(const uint8_t*p,int n){uint16_t c=0xffff;while(n--){c^=uint16_t(*p++)<<8;for(int i=0;i<8;i++)c=(c&0x8000)?(c<<1)^0x1021:c<<1;}return c;}
// ▶ Tạo gói thứ 'part' (0..3) của 1 row:
//   byte0 = 0xA0+part (0xB0+part nếu là dữ liệu live) | byte1 = version(1) | byte2-3 = session
//   byte4-5 = index row (65535 = live) | byte6-7 = CRC của cả 48 byte | byte8..19 = 12 byte dữ liệu.
inline void chunk(const uint8_t*row,uint16_t session,uint16_t index,int part,uint8_t*out,bool live=false){out[0]=(live?0xb0:0xa0)+part;out[1]=1;u16(out+2,session);u16(out+4,index);u16(out+6,crc(row,SIZE));memcpy(out+8,row+part*12,12);}
}

// ▶ Thư viện BLE của ESP32 (Nordic UART style).
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
// ▶ LAB_NATIVE = chế độ build thử trên PC; bình thường (ESP32) sẽ dùng FreeRTOS.
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
// ▶ IMU_SIGN: đảo dấu góc/gyro nếu bánh quay ngược chiều nghiêng. USE_X_AXIS: chọn trục X thay vì Y.
constexpr float IMU_SIGN = +1.0f;
constexpr bool USE_X_AXIS = false;

// ▶ PWM_PIN: chân PWM 2 motor | DIR_PIN: chân chiều quay | MOTOR_SIGN: đảo chiều logic từng motor cho khớp cách lắp.
constexpr int PWM_PIN[2] = {0, 1}, DIR_PIN[2] = {3, 21}, MOTOR_SIGN[2] = {-1, +1};
// ▶ Chân encoder: A (ngắt cạnh lên) và B (xác định chiều) của mỗi bánh.
constexpr int ENCODER_A[2] = {7, 20}, ENCODER_B[2] = {4, 10};
// ▶ ENC_SIGN: đảo dấu đếm encoder nếu cần (bánh tiến mà số đếm giảm thì đổi thành -1).
constexpr int ENC_SIGN[2] = {+1, +1};
// ▶ CPR: số xung đếm được trên 1 vòng bánh (đo thực tế, 2 bánh hơi khác nhau). Dùng để đổi xung -> cm/s.
constexpr float CPR[2] = {495.1f, 496.4f};
// ▶ Đường kính bánh (dùng tính tốc độ). TRACK_CM (khoảng cách 2 bánh) khai báo nhưng code này không dùng.
constexpr float WHEEL_DIAMETER_CM = 6.74f, TRACK_CM = 18.22f;
// ▶ JY_RX_PIN: chân ESP nhận dữ liệu từ IMU | STOP_PIN=9: nút BOOT | LED_PIN=8.
constexpr int JY_RX_PIN = 6, JY_TX_PIN = 5, STOP_PIN = 9, LED_PIN = 8;
// ▶ USE_LED: bật/tắt LED báo trạng thái (LED tích cực mức thấp).
constexpr bool USE_LED = true;

// ===== TỰ KHỞI ĐỘNG KHÔNG CẦN APP =====
// ▶ AUTO_START: bật nguồn tự calib + tự thả motor, không cần app.
constexpr bool AUTO_START = true;               // bật nguồn là tự chạy
// ▶ Sau khi ngã (fault 8) mà dựng lại thẳng thì tự chạy tiếp.
constexpr bool AUTO_RESTART_AFTER_FALL = true;  // ngã xong dựng lại thì tự chạy tiếp
// ▶ Xe phải đứng thẳng + yên liên tục bấy nhiêu ms mới thả motor.
constexpr uint32_t AUTO_START_DELAY_MS = 1500;  // giữ yên bao lâu thì mới thả motor
// ▶ Điều kiện 'đứng thẳng': |góc| < 4° và |tốc độ góc| < 15°/s.
constexpr float AUTO_START_MAX_ANGLE_DEG = 4.0f;
constexpr float AUTO_START_MAX_RATE_DEG_S = 15.0f;

// ================= THÔNG SỐ ĐIỀU KHIỂN PD & ĐỘNG CƠ =================
// Tăng KP và KD để tạo mô-men cứu xe tức thời ở góc nghiêng nhỏ (1..3 độ)
// ▶ PD góc nghiêng: command = KP*(góc - đích) + KD*gyro.
//   VD: lệch 1° -> P = 10.5; gyro 100°/s -> D = 25. Kết quả bị chặn ±55 (xem BalancePD::step).
constexpr float KP_THETA = 10.5f;
constexpr float KD_THETA = 0.25f;
// ▶ Bù lệch điểm cân bằng cơ khí (độ). 0 = không bù.
constexpr float THETA_TRIM_DEG = 0.0f;

// Nguong da do khi ke banh, chua xac nhan duoi tai tren san.
// PWM startup thresholds measured in free wheel tests; must be checked under load.
// ▶ PWM tối thiểu để motor BẮT ĐẦU quay (tiến) cho bánh 0 và bánh 1. Đo khi bánh nhấc khỏi sàn, cần kiểm tra lại khi có tải.
constexpr float START_PWM[2] = {120.0f, 110.0f};
// ▶ PWM tối thiểu khi quay LÙI (cao hơn tiến một chút).
constexpr float REVERSE_PWM[2] = {130.0f, 120.0f};
// ▶ Hysteresis bật/tắt motor theo |command|: bật khi >= 2.0, tắt khi < 0.9 (đơn vị 'cm/s tương đương').
//   Tránh motor bật tắt liên tục (chattering) quanh điểm cân bằng.
constexpr float MOTOR_ON_CM_EQ_S=2.0f,MOTOR_OFF_CM_EQ_S=0.9f;
// Software smoothing only near upright: quick rescue remains enabled if tilt/rate large.
// ▶ Cứu nhanh: nếu |góc| > 1.8° hoặc |gyro| > 30°/s thì dùng slew nhanh (tăng PWM nhanh).
constexpr float FAST_RESCUE_ANGLE_DEG=1.8f;
constexpr float FAST_RESCUE_RATE_DEG_S=30.0f;
// ▶ Slew gần đứng thẳng: mỗi tick tăng tối đa 22, giảm tối đa 30 (PWM thang 0..255) -> êm.
constexpr int SLEW_RISE_NEAR=22, SLEW_FALL_NEAR=30;
// ▶ Slew khi cứu nhanh: tăng 75, giảm 90 mỗi tick -> phản ứng mạnh.
constexpr int SLEW_RISE_FAST=75, SLEW_FALL_FAST=90;
// ▶ Khi đảo chiều motor phải nghỉ (PWM=0) ít nhất 5000 µs rồi mới quay chiều ngược.
constexpr uint32_t REVERSE_NEUTRAL_US=5000UL;
// ▶ Hằng số thời gian lọc gyro = 10 ms (lọc thông thấp bậc 1).
constexpr float GYRO_FILTER_TAU_S=0.010f;
// ▶ PWM tối đa 220 (chống quá dòng / quá tốc).
constexpr int PWM_MAX = 220; // Giu gioi han 220 theo yeu cau truoc.
// Safety: cut both motors if measured pitch exceeds this limit during any PD experiment.
// ▶ Góc ngã: |góc| > 35° là cắt motor (fault 8).
constexpr float FALL_LIMIT_DEG = 35.0f;
// ▶ Chu kỳ điều khiển hiện tại (µs). Là biến thường (không constexpr) vì app đổi được 2.5..10 ms. 2500µs = 400Hz.
uint32_t controlPeriodUs=2500; // 2.5ms default; app may change to 2.5..10.0ms
// ▶ Giới hạn và bước chỉnh chu kỳ: 2.5ms .. 10ms, bước 0.1ms.
constexpr uint32_t PERIOD_MIN_US = 2500;
constexpr uint32_t PERIOD_MAX_US = 10000;
constexpr uint32_t PERIOD_STEP_US = 100;
// ▶ Mỗi lần trace chạy 4.5 giây (chỉ áp dụng khi KHÔNG autoRun).
constexpr uint32_t TRACE_DURATION_US = 4500000UL; // Same test duration at all Ts
// ▶ Nếu 1 vòng điều khiển trễ hơn max(3×chu kỳ, 25ms) -> fault 6.
constexpr uint32_t MAX_CONTROL_US=25000;

// Vòng PI tốc độ ngoài, tạo góc đích để hãm/trả xe về tốc độ đặt.
// ▶ Vòng PI tốc độ (ngoài): đo tốc độ xe -> sinh ra 'góc đích' nhỏ để hãm xe,
//   giúp xe đứng yên tại chỗ thay vì trôi.
constexpr bool ENABLE_SPEED_PI = true;
// ▶ KP_SPEED, KI_SPEED: góc đích (độ) = KP*lỗi_tốc_độ(cm/s) + tích phân.
constexpr float KP_SPEED = 0.022f, KI_SPEED = 0.006f;
// ▶ Tốc độ mục tiêu = 0 (đứng yên). Góc đích do PI tạo ra bị chặn ±2.5°.
constexpr float SPEED_TARGET_CM_S = 0.0f, SPEED_ANGLE_MAX_DEG = 2.5f;
// ▶ SPEED_FILTER_TAU: lọc tốc độ đo (0.18s). REF_SLEW: góc đích đổi tối đa 3°/giây (cho mượt).
constexpr float SPEED_FILTER_TAU_S = 0.18f, REF_SLEW_DEG_PER_S = 3.0f;
// ▶ SPEED_PI_DELAY_MS=0: PI chạy ngay khi arm. SPEED_US=50000: cập nhật PI mỗi 50 ms (20 Hz).
constexpr uint32_t SPEED_PI_DELAY_MS = 0, SPEED_US = 50000;

// ▶ --- Bộ phân tích khung JY61 ---
// JY61 UART: 11 bytes, 0x55 + type + 8 payload bytes + additive checksum.
// ▶ Bộ parser máy trạng thái từng byte: tìm 0x55 -> gom đủ 11 byte -> kiểm checksum.
struct JY61Parser {
  // ▶ buffer: gom 1 khung 11 byte; used: số byte đã gom.
  uint8_t buffer[11] = {};
  size_t used = 0;
  // ▶ valid/bad: đếm khung đúng/sai (để chẩn đoán).
  uint32_t valid = 0, bad = 0;
  // ▶ Dữ liệu thô int16 mới nhất: accel[x,y,z], gyro[x,y,z], angle[roll,pitch,yaw].
  int16_t accel[3] = {}, gyro[3] = {}, angle[3] = {};

  // ▶ Ghép 2 byte little-endian thành int16 có dấu.
  static int16_t le16(const uint8_t* p) {
    return static_cast<int16_t>(uint16_t(p[0]) | (uint16_t(p[1]) << 8));
  }

  // ▶ Nạp 1 byte. Trả về loại khung (0x51/52/53) nếu vừa hoàn tất 1 khung hợp lệ, ngược lại trả 0.
  uint8_t feed(uint8_t c) {
    // ▶ Chưa bắt đầu khung mà byte không phải 0x55 -> bỏ qua (chờ đầu khung).
    if (!used && c != 0x55) return 0;
    buffer[used++] = c;
    // ▶ Chưa đủ 11 byte thì chờ tiếp.
    if (used < 11) return 0;
    // ▶ Checksum = tổng 10 byte đầu (cắt 8 bit) phải bằng byte thứ 11.
    uint8_t sum = 0;
    for (int i = 0; i < 10; i++) sum = uint8_t(sum + buffer[i]);
    uint8_t type = buffer[1];
    // ▶ Hợp lệ khi checksum đúng và loại 0x50..0x53 (0x50 thời gian, 0x51 gia tốc, 0x52 gyro, 0x53 góc).
    if (sum == buffer[10] && type >= 0x50 && type <= 0x53) {
      // ▶ Chọn mảng đích theo loại khung.
      int16_t* out = (type == 0x51) ? accel : (type == 0x52) ? gyro : angle;
      // ▶ Khung thời gian 0x50 thì bỏ qua nội dung.
      if (type != 0x50) {
        // ▶ Đọc 3 giá trị x,y,z (mỗi giá trị 2 byte).
        for (int i = 0; i < 3; i++) out[i] = le16(buffer + 2 + i * 2);
      }
      used = 0;
      valid++;
      // ▶ Trả về loại khung để pollSensor biết cập nhật góc hay gyro.
      return type;
    }
    // ▶ Khung sai: tăng 'bad', rồi ĐỒNG BỘ LẠI - tìm byte 0x55 kế tiếp trong buffer,
    //   dịch phần còn lại lên đầu buffer (không bỏ cả khung).
    bad++;
    size_t start = 1;
    while (start < 11 && buffer[start] != 0x55) start++;
    used = 11 - start;
    for (size_t i = 0; i < used; i++) buffer[i] = buffer[start + i];
    return 0;
  }
};

// ▶ --- Namespace Lab: bộ điều khiển PD và ánh xạ command -> PWM ---
namespace Lab {
// ▶ Chặn giá trị trong [lo, hi].
inline float clamp(float x, float lo, float hi) { return x < lo ? lo : x > hi ? hi : x; }

// ▶ Mô hình motor tuyến tính: PWM = a*|command| + b (lo hiện không dùng).
struct Model {
  float a = 0, b = 0, lo = 0;
};

// ▶ Tham số PD có thể chỉnh từ app: kp, kd, góc đích, PWM tối đa.
struct BalanceConfig {
  float kp = KP_THETA, kd = KD_THETA, target = 0;
  int maxPwm = PWM_MAX;
};

// ▶ Bộ cân bằng PD. Lưu góc/gyro đã lọc và các thành phần p, d, u để ghi trace.
struct BalancePD {
  BalanceConfig config;
  float angle = 0, rate = 0, target = 0, p = 0, d = 0, u = 0;
  bool initialized = false;

  // ▶ reset(): xóa trạng thái, lần step sau sẽ khởi tạo lại bằng giá trị đo (tránh nhảy).
  void reset() {
    angle = rate = p = d = u = 0;
    target = config.target;
    initialized = false;
  }

  // ▶ step(): tính command từ góc theta (độ), gyro (°/s), dt (s), góc đích. Gọi mỗi tick điều khiển.
  float step(float theta, float gyro, float dt, float targetAngle) {
    // ▶ Lần đầu: khởi tạo bộ lọc bằng đúng giá trị đo để không bị giật.
    if (!initialized) {
      angle = theta;
      rate = gyro;
      initialized = true;
    }
    // Lấy trực tiếp góc và gyro với lọc cực nhẹ để không trễ pha
    // ▶ Lọc góc: alpha = dt/(0.002+dt). Với dt=2.5ms -> alpha ~0.55, gần như không lọc (không trễ pha).
    angle += dt / (0.002f + dt) * (theta - angle);
    // ▶ Lọc gyro: alpha = dt/(0.010+dt). Với dt=2.5ms -> alpha ~0.2 (lọc nhiễu gyro).
    rate  += dt / (GYRO_FILTER_TAU_S + dt) * (gyro - rate);
    // ▶ Góc đích = trim từ app + hiệu chỉnh từ PI tốc độ (speedApplied).
    target = targetAngle;
    // ▶ P = KP * (góc - góc đích): lệch càng nhiều đẩy càng mạnh.
    p = config.kp * (angle - target);
    // ▶ D = KD * gyro: dùng thẳng tốc độ góc đo từ gyro (không đạo hàm góc nên ít nhiễu).
    d = config.kd * rate;
    // ▶ Command = P + D, chặn ±55. Đơn vị 'cm/s tương đương'; sau đó map sang PWM qua mô hình a,b.
    u = clamp(p + d, -55.0f, 55.0f);
    return u;
  }
};

// ▶ computeDirectPwm: đổi command -> PWM có dấu.
// Pure target mapping: threshold is a floor, never a substituted intercept.
// ▶ w = bánh (0/1); model chứa a,b; 'confirmed' hiện KHÔNG được dùng trong hàm này.
inline int computeDirectPwm(float command,int w,const Model& model,int maxPwm,bool confirmed=false){
  // ▶ |command| < 0.9 (vùng chết) hoặc dữ liệu hỏng -> PWM = 0.
  if(!isfinite(command)||fabsf(command)<MOTOR_OFF_CM_EQ_S||maxPwm<1)return 0;
  // ▶ Chiều: command dương = +1 (tiến), âm = -1 (lùi).
  int dir=command>0?1:-1;
  // Never drop floor to 55 merely because encoder briefly reports motion:
  // this abrupt torque decrease was a major suspect for ON/OFF chatter.
  // ▶ Sàn PWM: START_PWM nếu tiến, REVERSE_PWM nếu lùi.
  float lower=(dir>0?START_PWM[w]:REVERSE_PWM[w]);
  // ▶ PWM = max(sàn, a*|cmd| + b). VD bánh 0 tiến, cmd=10: 4.81*10+31.6=79.7 -> nâng lên sàn 120.
  float pwm=fmaxf(lower,model.a*fabsf(command)+model.b);
  // ▶ Chặn tối đa maxPwm, rồi gắn dấu chiều.
  return dir*lroundf(clamp(pwm,0,float(maxPwm)));
}
// ▶ MotorMap: theo dõi từng bánh đã 'bật' chưa, hướng nào, và đã quay thật chưa (qua encoder).
struct MotorMap {
  // ▶ enabled: đang bật | confirmed: encoder xác nhận bánh đã quay | direction | base: số xung lúc bật.
  bool enabled=false,confirmed=false;int direction=0;int32_t base=0;
  void reset(){enabled=confirmed=false;direction=0;base=0;}
  // ▶ step(): ra PWM cho 1 bánh. moving = bánh đang quay theo hướng command; net = số xung hiện tại.
  int step(float command,int w,const Model& model,int maxPwm,bool moving,int32_t net){
    float magnitude=fabsf(command);
    // ▶ |command| < 0.9: tắt hẳn, reset, PWM=0.
    if(!isfinite(command)||magnitude<MOTOR_OFF_CM_EQ_S){reset();return 0;}
    // ▶ Xác định chiều từ dấu command.
    int dir=command>0?1:-1;
    // ▶ Chưa bật hoặc vừa đổi chiều -> phải xét lại điều kiện bật.
    if(!enabled||direction!=dir){
      // ▶ Chưa đủ lớn để bật (< 2.0) -> giữ tắt (hysteresis).
      if(magnitude<MOTOR_ON_CM_EQ_S){reset();return 0;}
      // ▶ Bật motor, lưu số xung gốc để theo dõi.
      enabled=true;confirmed=false;direction=dir;base=net;
    }
    // ▶ Nếu đã quay đủ >= 3 xung theo đúng chiều -> xác nhận 'confirmed'.
    if(!confirmed&&moving&&(int64_t(net)-base)*dir>=3)confirmed=true;
    // ▶ Đang confirmed mà bánh dừng -> hủy xác nhận.
    if(confirmed&&!moving){confirmed=false;base=net;}
    // ▶ Trả về PWM theo công thức computeDirectPwm.
    return computeDirectPwm(command,w,model,maxPwm,confirmed);
  }
};
} // namespace Lab

// ▶ --- Biến trạng thái toàn cục ---
// ================= STATE / ENCODER =================
// ▶ Các trạng thái: chờ cảm biến, rảnh, đang calib, chuẩn bị (đếm 3s), đang chạy, lỗi.
enum RunState : uint8_t { WAIT_SENSOR=0, IDLE=1, CALIBRATING=2, PREPARING=3, RUNNING=4, FAULTED=5 };
// ▶ state: trạng thái hiện tại. faultCode: mã lỗi gần nhất.
RunState state = WAIT_SENSOR;
uint8_t faultCode = 0;
// ▶ jy: UART1 nối với IMU JY61.
HardwareSerial jy(1);
// ▶ imu: bộ parser JY61.
JY61Parser imu;
// ▶ pd: bộ điều khiển PD cân bằng.
Lab::BalancePD pd;
// ▶ models[4]: mô hình motor, chỉ số = bánh*2 + (lùi ? 1 : 0).
Lab::Model models[4];
// ▶ motorMap[2]: logic bật/tắt cho từng bánh.
Lab::MotorMap motorMap[2];
// ▶ Theo dõi chuyển động bánh: số xung lần trước, thời điểm đổi, hướng quay gần nhất.
int32_t motionNet[2]={0,0};uint32_t motionAt[2]={0,0};int motionSign[2]={0,0};
// ▶ Khóa (spinlock) bảo vệ encoder[] giữa ISR và loop.
portMUX_TYPE encoderMux = portMUX_INITIALIZER_UNLOCKED;
// ▶ encoder[]: số xung CÓ DẤU (tiến + / lùi -).
volatile int32_t encoder[2] = {0, 0};
// ▶ absEncoder[]: tổng số xung không dấu (dùng phát hiện bánh có động đậy không).
volatile uint32_t absEncoder[2] = {0, 0};
// ▶ speedCount: số xung lần đo tốc độ trước.
int32_t speedCount[2] = {0, 0};
// ▶ actualPWM/actualDir: PWM và chiều thực tế đang xuất ra motor.
int actualPWM[2] = {0, 0}, actualDir[2] = {0, 0};
// ▶ Nhóm biến cho slew + chống đảo chiều đột ngột.
// Output slew and direction guard, independent from the motor model.
int lastMotorDirection[2] = {0, 0};
uint32_t motorZeroSinceUs[2] = {0, 0};
// ▶ pwmRequested: PWM yêu cầu trước khi làm mượt (để ghi trace).
int pwmRequested[2] = {0, 0};
// ▶ pwmReady: kênh PWM khởi tạo thành công chưa.
bool pwmReady[2] = {false, false};
// ▶ Đã thử tự calib 1 lần chưa.
bool autoZeroAttempted=false;
// ▶ autoRun: đang ở chế độ tự chạy (không cắt khi mất BLE, không giới hạn 4.5s). autoStartDone: đã tự thả motor lần đầu.
bool autoRun=false, autoStartDone=false;
// ▶ autoReadyAt: thời điểm bắt đầu đếm 1.5s 'đứng yên' (0 = chưa đếm).
uint32_t autoReadyAt=0;
// ▶ Mốc thời gian: lúc bật nguồn, lúc mở baud, thời điểm nhận góc/gyro cuối, baud hiện tại.
uint32_t bootAt = 0, baudAt = 0, baud = 115200, lastAngleAt = 0, lastGyroAt = 0;
// ▶ Đếm tần số khung IMU mỗi giây (angleHz, gyroHz).
uint32_t rateAt = 0, angleFrames = 0, gyroFrames = 0, angleHz = 0, gyroHz = 0;
// ▶ controlAtUs: mốc vòng điều khiển trước. speedAtUs: mốc PI tốc độ trước. armedAt: lúc thả motor.
uint32_t controlAtUs = 0, speedAtUs = 0, armedAt = 0, prepareAt = 0;
// ▶ zeroAngle: góc thô khi xe đứng thẳng (điểm cân bằng). gyroBias: độ lệch gyro khi đứng yên.
float zeroAngle = 0, gyroBias = 0;
// ▶ Biến PI tốc độ: tốc độ lọc từng bánh, trung bình, tích phân, góc đích (ref), góc đích đã làm mượt (applied).
float speedFiltered[2] = {0, 0}, speedAverage = 0, speedIntegral = 0, speedRef = 0, speedApplied = 0;
// ▶ outputSaturated: PWM đang bão hòa -> dừng tích phân (anti-windup).
bool outputSaturated = false;
// ▶ Biến phục vụ calib: mốc thời gian, mẫu, tổng và tổng bình phương (để tính độ lệch chuẩn).
uint32_t calBegin = 0, calLastAngle = 0, calSessionAt=0;
int calSamples = 0;
double calAngle = 0, calAngle2 = 0, calGyro = 0, calGyro2 = 0;
float calOrigin = 0;
// ▶ calCounts/calAbs: số xung encoder lúc calib (để phát hiện bánh bị động đậy).
int32_t calCounts[2] = {0, 0};
uint32_t calAbs[2] = {0, 0};

// ▶ Nút BOOT (GPIO9). Lưu ý: code thực tế chỉ DỪNG khi bấm lúc đang chạy/chuẩn bị/calib (fault 10).
// Quản lý nút bấm BOOT (GPIO9): Bấm nhả nhanh = Lấy lại góc 0; Nhấn giữ = Dừng
bool btnPressed = false;
uint32_t btnDownAt = 0;
bool btnLongHandled = false;

// ▶ Trace độc lập với nhịp cập nhật của điện thoại.
// Trace status is independent of phone/display cadence.
// ▶ traceMode: 0=không, 1=M(motor tắt), 2=B, 3=C. zeroValid: đã có điểm zero. releaseMarked: đã đánh dấu thả (K).
uint8_t traceMode=0;bool zeroValid=false,releaseMarked=false;
// ▶ traceSession: số phiên dữ liệu. rowCount: số row đã ghi. captureAtUs: mốc bắt đầu ghi.
uint16_t traceSession=0,rowCount=0;uint32_t captureAtUs=0;
// ▶ traceRows: bộ nhớ ghi mỗi tick điều khiển. liveRow: row gửi live qua BLE.
uint8_t traceRows[Trace::MAX_ROWS][Trace::SIZE]={};uint8_t liveRow[Trace::SIZE]={};
// ▶ Đo chu kỳ khung góc/gyro (µs) và bộ đếm khung góc để ghi vào trace.
uint32_t previousAngleUs=0,previousGyroUs=0,anglePeriodUs=0,gyroPeriodUs=0,angleSequence=0;
// ▶ logNet: số xung lần ghi trước (để tính delta encoder). telemetryAt: mốc gửi telemetry 200ms.
int32_t logNet[2]={0,0};uint32_t telemetryAt=0;
// ▶ Cờ BLE: connected, advertiseAgain (cần quảng bá lại).
volatile bool connected=false,advertiseAgain=false;
// ▶ Cờ sự kiện BLE đặt trong callback, xử lý trong loop.
volatile bool bleNotifyReady=false, bleConnectedEvent=false, bleDisconnectedEvent=false;
// ▶ heartbeatAt: lúc nhận lệnh 'H' cuối. bleDisconnectedAt: lúc ngắt kết nối.
volatile uint32_t heartbeatAt=0, bleDisconnectedAt=0;
// ▶ Khóa bảo vệ hàng đợi lệnh.
portMUX_TYPE commandMux=portMUX_INITIALIZER_UNLOCKED;
// ▶ Hàng đợi lệnh BLE: tối đa 8 lệnh, mỗi lệnh <= 19 ký tự.
char commands[8][20]={};uint8_t commandCount=0;
// ▶ tx: characteristic gửi notify về app.
BLECharacteristic* tx=nullptr;
// ▶ RadioBatch: 1 lô tối đa 4 gói × 20 byte gửi cho radioTask.
struct RadioBatch{uint8_t count=0;uint8_t packets[4][20];};
void sendBatch(const RadioBatch&b);
#ifndef LAB_NATIVE
// ▶ radioQueue: hàng đợi lô gói; radioTask gửi nền để BLE không làm chậm vòng điều khiển.
QueueHandle_t radioQueue=nullptr;
// ▶ radioTask: task FreeRTOS riêng, lấy lô từ queue và notify từng gói.
void radioTask(void*) {
  RadioBatch b;
  for (;;) {
    // ▶ Chờ vô hạn cho tới khi có lô mới.
    if (xQueueReceive(radioQueue, &b, portMAX_DELAY) != pdTRUE) continue;
    // ▶ Chưa kết nối / chưa sẵn sàng -> bỏ lô.
    if (!connected || !bleNotifyReady || !tx) continue;
    // ▶ Gửi từng gói; thoát nếu ngắt kết nối giữa chừng.
    for (int i=0; i<b.count && connected && bleNotifyReady; i++) {
      tx->setValue(b.packets[i],20);
      tx->notify();
      vTaskDelay(1); // Let BLE stack process notifications; not a PD-loop delay.
    }
  }
}
#endif
// ▶ sendBatch: đẩy lô vào queue (timeout 0 = không bao giờ chặn vòng điều khiển; đầy thì bỏ).
void sendBatch(const RadioBatch&b){
#ifdef LAB_NATIVE
  if(tx)for(int i=0;i<b.count;i++){tx->setValue((uint8_t*)b.packets[i],20);tx->notify();}
#else
  if(radioQueue&&connected&&bleNotifyReady)xQueueSend(radioQueue,&b,0);
#endif
}
// ▶ sendStatus: gửi 2 gói trạng thái.
//   Gói 0x90: state, fault, số row, session, chu kỳ, zeroValid, angleHz, gyroHz, traceMode, releaseMarked, calSamples.
//   Gói 0x91: KP(x100), KD(x1000), target(x100), maxPwm, chu kỳ, zeroAngle, gyroBias.
void sendStatus(){
  RadioBatch b;b.count=2;auto*p=b.packets[0];memset(p,0,20);p[0]=0x90;p[1]=1;p[2]=state;p[3]=faultCode;Trace::u16(p+4,rowCount);Trace::u16(p+6,traceSession);p[8]=controlPeriodUs/100;p[9]=zeroValid;Trace::u16(p+10,angleHz);Trace::u16(p+12,gyroHz);p[14]=traceMode;p[15]=releaseMarked;Trace::u16(p+16,calSamples);p[18]=2; // protocol v2: period in units 0.1 ms
  p=b.packets[1];memset(p,0,20);p[0]=0x91;p[1]=1;Trace::u16(p+2,traceSession);Trace::fixed(p+4,pd.config.kp,100);Trace::fixed(p+6,pd.config.kd,1000);Trace::fixed(p+8,pd.config.target,100);Trace::u16(p+10,pd.config.maxPwm);Trace::u16(p+12,controlPeriodUs/100);p[14]=traceMode;Trace::fixed(p+16,zeroAngle,100);Trace::fixed(p+18,gyroBias,1000);sendBatch(b);
}
// ▶ Callback kết nối/ngắt kết nối BLE.
class LinkCallbacks:public BLEServerCallbacks{
  // ▶ onConnect: đánh dấu đã kết nối; chờ app gửi 'H' mới cho phép notify.
  void onConnect(BLEServer*) override {
    connected=true;
    bleNotifyReady=false; // Wait for app CCCD registration, then its first H command.
    heartbeatAt=millis();
    bleConnectedEvent=true;
  }
  // ▶ onDisconnect: reset cờ, ghi thời điểm, yêu cầu quảng bá lại (loop sẽ làm sau 300ms).
  void onDisconnect(BLEServer*) override {
    connected=false;
    bleNotifyReady=false;
    bleDisconnectedAt=millis();
    bleDisconnectedEvent=true;
    advertiseAgain=true;
  }
};
// ▶ Callback nhận lệnh từ app (ghi vào characteristic RX).
class CommandCallbacks:public BLECharacteristicCallbacks{
  void onWrite(BLECharacteristic*c)override{
    // ▶ Chấp nhận lệnh dài 1..19 byte.
    auto value=c->getValue();if(value.length()<1||value.length()>19)return;
    // ▶ 'H' = heartbeat: cập nhật thời điểm và bật cho phép gửi notify.
    if(value.length()==1&&value.c_str()[0]=='H'){heartbeatAt=millis();bleNotifyReady=true;return;}
    // ▶ Vào vùng găng: hàng đợi lệnh dùng chung giữa task BLE và loop.
    portENTER_CRITICAL(&commandMux);
    // ▶ 'X' = DỪNG: ưu tiên cao nhất, xóa mọi lệnh khác và chỉ giữ X.
    if(value.length()==1&&value.c_str()[0]=='X'){commandCount=1;strcpy(commands[0],"X");}
    // ▶ Lệnh khác: thêm vào hàng đợi (nếu chưa đầy và chưa có X chờ).
    else if(commandCount<8&&(commandCount==0||commands[0][0]!='X')){memcpy(commands[commandCount],value.c_str(),value.length());commands[commandCount++][value.length()]=0;}
    portEXIT_CRITICAL(&commandMux);
  }
};

// ▶ ISR encoder bánh trái: cạnh lên của kênh A, đọc kênh B để biết chiều (B cao = -1, thấp = +1).
void ARDUINO_ISR_ATTR encoderLeftISR() {
  int step = digitalRead(ENCODER_B[0]) ? -1 : +1;
  // ▶ Vùng găng: cập nhật số xung có dấu và tổng xung.
  portENTER_CRITICAL_ISR(&encoderMux);
  encoder[0] += ENC_SIGN[0] * step;
  absEncoder[0]++;
  portEXIT_CRITICAL_ISR(&encoderMux);
}

// ▶ ISR encoder bánh phải: giống trái, dùng chân tương ứng.
void ARDUINO_ISR_ATTR encoderRightISR() {
  int step = digitalRead(ENCODER_B[1]) ? -1 : +1;
  portENTER_CRITICAL_ISR(&encoderMux);
  encoder[1] += ENC_SIGN[1] * step;
  absEncoder[1]++;
  portEXIT_CRITICAL_ISR(&encoderMux);
}

// ▶ readCounts: đọc nguyên tử số xung có dấu và tổng xung của cả 2 bánh.
void readCounts(int32_t* net, uint32_t* total) {
  portENTER_CRITICAL(&encoderMux);
  for (int w = 0; w < 2; w++) {
    net[w] = encoder[w];
    total[w] = absEncoder[w];
  }
  portEXIT_CRITICAL(&encoderMux);
}

// ▶ Chuẩn hóa góc về [-180, 180].
float wrapDegrees(float value) {
  while (value > 180) value -= 360;
  while (value < -180) value += 360;
  return value;
}

// ▶ Góc thô (độ) từ IMU: raw*180/32768, chọn trục X hoặc Y, nhân IMU_SIGN.
float rawAngleDeg() {
  int16_t raw = USE_X_AXIS ? imu.angle[0] : imu.angle[1];
  return IMU_SIGN * (raw * 180.0f / 32768.0f);
}

// ▶ Tốc độ góc thô (°/s): raw*2000/32768 (thang đo ±2000°/s).
float rawGyroDegS() {
  int16_t raw = USE_X_AXIS ? imu.gyro[0] : imu.gyro[1];
  return IMU_SIGN * (raw * 2000.0f / 32768.0f);
}

// ▶ Cảm biến 'còn tươi': cả góc và gyro mới trong 40ms gần nhất.
bool freshSensor(uint32_t now) {
  return lastAngleAt && lastGyroAt && uint32_t(now - lastAngleAt) <= 40 && uint32_t(now - lastGyroAt) <= 40;
}

// ▶ LED tích cực mức thấp (LOW = sáng).
void setLed(bool on) {
  if (USE_LED) digitalWrite(LED_PIN, on ? LOW : HIGH);
}

// ▶ writeDuty: ghi duty PWM, hỗ trợ cả ESP32 core v2 (kênh) và v3 (chân).
void writeDuty(int w, int magnitude) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(PWM_PIN[w], magnitude);
#else
  ledcWrite(w, magnitude);
#endif
}

// ▶ writeMotor: xuất PWM có dấu ra motor. Chỉ ngắt về 0 khi ĐỔI CHIỀU.
// Chỉ ngắt PWM về 0 khi ĐỔI CHIỀU quay, không ngắt khi đang tăng/giảm tốc cùng chiều
void writeMotor(int w, int value) {
  if (!pwmReady[w]) return;
  // ▶ Chặn theo maxPwm; không đổi thì bỏ qua.
  value = (int)Lab::clamp(value, -pd.config.maxPwm, pd.config.maxPwm);
  if (value == actualPWM[w]) return;

  // ▶ value=0 -> tắt duty.
  if (value == 0) {
    writeDuty(w, 0);
    actualPWM[w] = 0;
    actualDir[w] = 0;
    return;
  }

  // ▶ Tính chiều vật lý (có xét MOTOR_SIGN).
  int dir = (value * MOTOR_SIGN[w] > 0) ? 1 : -1;
  // ▶ Đổi chiều: duty=0 trước, rồi mới đổi chân DIR (an toàn cho H-bridge).
  if (dir != actualDir[w]) {
    writeDuty(w, 0);
    digitalWrite(DIR_PIN[w], dir > 0 ? HIGH : LOW);
    actualDir[w] = dir;
  }
  // ▶ Ghi duty = |value| và lưu PWM thực tế.
  writeDuty(w, abs(value));
  actualPWM[w] = value;
}

// ▶ smoothCommandPwm: bộ chống giật (slew limiter) giữa PWM yêu cầu và PWM xuất ra.
//   Mỗi tick chỉ được đổi tối đa rise/fall. stopMotors() bỏ qua bộ này.
// Rate-limit sudden 0->120 jumps and reversals near balance.
// Safety stopMotors() always bypasses this limiter.
int smoothCommandPwm(int w, int target, bool urgent, uint32_t nowUs) {
  pwmRequested[w] = target;
  int previous = actualPWM[w];
  const int previousSign = (previous > 0) - (previous < 0);
  // ▶ Dấu PWM đang xuất vs dấu PWM yêu cầu.
  int wantedSign = (target > 0) - (target < 0);
  // ▶ Hiện tại và yêu cầu TRÁI DẤU: ép target về 0 trước, không đảo chiều đột ngột.
  if (previousSign && wantedSign && previousSign != wantedSign) {
    // Drive to zero first, do not instantly switch H-bridge direction.
    target = 0;
    wantedSign = 0;
  }
  // ▶ Vừa dừng và muốn quay NGƯỢC chiều cũ: giữ trung tính 5ms (REVERSE_NEUTRAL_US), không chặn.
  if (!previousSign && wantedSign && lastMotorDirection[w] &&
      wantedSign != lastMotorDirection[w] &&
      (uint32_t)(nowUs - motorZeroSinceUs[w]) < REVERSE_NEUTRAL_US) {
    target = 0; // Maintain brief neutral interval (non-blocking).
    wantedSign = 0;
  }
  // ▶ Chọn tốc độ slew: urgent (nghiêng nhiều / quay nhanh) dùng 75/90, còn lại 22/30.
  const int rise = urgent ? SLEW_RISE_FAST : SLEW_RISE_NEAR;
  const int fall = urgent ? SLEW_FALL_FAST : SLEW_FALL_NEAR;
  // ▶ Tiến dần |PWM| về giá trị mong muốn theo bước rise/fall.
  int desired = abs(target), current = abs(previous);
  // Avoid going through opposite direction in the same control tick.
  if (previousSign && wantedSign && previousSign != wantedSign) desired = 0;
  int nextMag = current;
  if (desired > current) nextMag = current + (rise < desired-current ? rise : desired-current);
  else if (desired < current) nextMag = current - (fall < current-desired ? fall : current-desired);
  // ▶ Giữ chiều cũ khi đang quay, chỉ lấy chiều mới khi từ 0 lên.
  const int nextSign = (nextMag == 0) ? 0 : (previousSign ? previousSign : wantedSign);
  int next = nextSign * nextMag;
  // ▶ Ghi nhớ chiều cuối và thời điểm về 0 (dùng cho khoảng nghỉ đảo chiều) / chiều mới khi bắt đầu quay.
  if (previous != 0 && next == 0) {
    lastMotorDirection[w] = previousSign;
    motorZeroSinceUs[w] = nowUs;
  } else if (previous == 0 && next != 0) {
    lastMotorDirection[w] = nextSign;
  }
  return next;
}

// ▶ stopMotors: dừng khẩn cấp - duty về 0 ngay (bỏ qua slew), reset toàn bộ trạng thái liên quan motor.
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

// ▶ latchFault: chốt lỗi - dừng motor, lưu mã lỗi, vào FAULTED, reset PD và PI.
void latchFault(uint8_t code) {
  stopMotors();
  faultCode = code;
  state = FAULTED;
  pd.reset();
  speedIntegral = speedRef = speedApplied = 0;
}

// ▶ showState: LED. RUNNING sáng liên tục; FAULTED nháy 'faultCode' lần rồi nghỉ; CALIBRATING nháy nhanh 120ms; còn lại nháy chậm 400ms.
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

// ▶ openBaud: đóng/mở lại UART IMU với baud mới, reset bộ đếm tần số.
void openBaud(uint32_t value, uint32_t now) {
  jy.end();
  imu.used = 0;
  baud = value;
  jy.begin(baud, SERIAL_8N1, JY_RX_PIN, JY_TX_PIN);
  baudAt = rateAt = now;
  lastAngleAt = lastGyroAt = 0;
  angleFrames = gyroFrames = angleHz = gyroHz = 0;
}

// ▶ pollSensor: đọc IMU (tối đa 512 byte mỗi lần) và cập nhật tần số.
void pollSensor(uint32_t now) {
  int budget = 512;
  while (budget-- && jy.available()) {
    uint8_t kind = imu.feed((uint8_t)jy.read());
    // ▶ Khung góc 0x53: ghi chu kỳ (µs), tăng số thứ tự, cập nhật thời điểm nhận.
    if (kind == 0x53) {
      uint32_t stamp=micros();if(previousAngleUs)anglePeriodUs=stamp-previousAngleUs;previousAngleUs=stamp;angleSequence++;
      lastAngleAt = now;
      angleFrames++;
    // ▶ Khung gyro 0x52: ghi chu kỳ và thời điểm.
    } else if (kind == 0x52) {
      uint32_t stamp=micros();if(previousGyroUs)gyroPeriodUs=stamp-previousGyroUs;previousGyroUs=stamp;
      lastGyroAt = now;
      gyroFrames++;
    }
  }
  // ▶ Mỗi 1 giây: tính tần số khung góc/gyro (Hz) rồi reset bộ đếm.
  uint32_t elapsed = now - rateAt;
  if (elapsed >= 1000) {
    angleHz = angleFrames * 1000UL / elapsed;
    gyroHz = gyroFrames * 1000UL / elapsed;
    angleFrames = gyroFrames = 0;
    rateAt = now;
  }
  // ▶ Chưa thấy dữ liệu IMU hợp lệ sau 2.5s thì đổi baud 115200 <-> 9600 để dò.
  if (state == WAIT_SENSOR && uint32_t(now - baudAt) > 2500 && !freshSensor(now)) {
    openBaud(baud == 115200 ? 9600 : 115200, now);
  }
}

// ▶ resetCalibration: bắt đầu phiên lấy mẫu zero từ đầu (gọi khi có động đậy hoặc không đạt).
void resetCalibration(uint32_t now) {
  calBegin = now;
  calLastAngle = 0;
  calSamples = 0;
  calAngle = calAngle2 = calGyro = calGyro2 = 0;
  calOrigin = rawAngleDeg();
  readCounts(calCounts, calAbs);
}

// ▶ armMotors: thả motor - reset toàn bộ bộ điều khiển, ghi mốc xung, vào RUNNING.
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

// ▶ quickReZero: calib lại thật sự (không phải 1 mẫu đơn). Điều kiện: PWM ok, IMU ổn định.
// Re-zero is a real calibration, never a single sample and gyroBias=0.
void quickReZero(uint32_t now){
  stopMotors();pd.reset();speedIntegral=speedRef=speedApplied=0;
  if(!pwmReady[0]||!pwmReady[1]){latchFault(1);return;}
  // ▶ IMU không ổn (mất hoặc < 50Hz) -> fault 5.
  if(!freshSensor(now)||angleHz<50||gyroHz<50){latchFault(5);return;}
  faultCode=0;state=CALIBRATING;calSessionAt=now;resetCalibration(now);
}
// ▶ Bấm nút BOOT (LOW) khi đang RUNNING/PREPARING/CALIBRATING -> dừng ngay (fault 10).
void handleBootButton(uint32_t now){if(digitalRead(STOP_PIN)==LOW&&(state==RUNNING||state==PREPARING||state==CALIBRATING))latchFault(10);}

// ▶ takeZero: tìm điểm zero. Mẫu hợp lệ khi xe đứng yên (bánh không quay, |gyro|<=5, |góc|<=2).
//   Cần >= 1.5s và >= 60 mẫu, độ lệch chuẩn góc <= 0.35°, gyro <= 1.5°/s thì chấp nhận.
void takeZero(uint32_t now){
  // ▶ Chỉ xử lý khi có khung góc MỚI.
  if(!freshSensor(now)||lastAngleAt==calLastAngle)return;
  calLastAngle=lastAngleAt;
  int32_t net[2];uint32_t total[2];readCounts(net,total);
  float a=wrapDegrees(rawAngleDeg()-calOrigin),g=rawGyroDegS();
  // ▶ Bánh xe quay, hoặc |gyro| > 5, hoặc |góc| > 2 so với điểm bắt đầu -> làm lại từ đầu.
  if(total[0]!=calAbs[0]||total[1]!=calAbs[1]||fabsf(g)>5||fabsf(a)>2){resetCalibration(now);return;}
  // ▶ Cộng dồn mẫu và bình phương mẫu (để tính độ lệch chuẩn).
  calSamples++;calAngle+=a;calAngle2+=double(a)*a;calGyro+=g;calGyro2+=double(g)*g;
  // ▶ Chưa đủ 1.5s hoặc 60 mẫu thì tiếp tục lấy.
  if(uint32_t(now-calBegin)<1500||calSamples<60)return;
  // ▶ Tính trung bình và độ lệch chuẩn.
  double meanA=calAngle/calSamples,meanG=calGyro/calSamples;
  float sdA=sqrtf(fmaxf(0,float(calAngle2/calSamples-meanA*meanA)));
  float sdG=sqrtf(fmaxf(0,float(calGyro2/calSamples-meanG*meanG)));
  // ▶ Quá nhiễu (xe rung) -> làm lại.
  if(sdA>.35f||sdG>1.5f){resetCalibration(now);return;}
  // ▶ Lưu zeroAngle = góc thô trung bình, gyroBias = gyro trung bình.
  zeroAngle=wrapDegrees(calOrigin+meanA);gyroBias=meanG;
  // ▶ Xong: reset PD, đánh dấu zeroValid, về IDLE. autoStartTick sẽ tiếp quản.
  pd.reset();zeroValid=true;faultCode=0;state=IDLE;
}

// ▶ updateSpeed: vòng PI tốc độ, chạy mỗi 50ms. Tạo 'góc đích' phụ để hãm xe, giữ xe tại chỗ.
void updateSpeed(uint32_t nowUs, uint32_t now) {
  uint32_t elapsed = nowUs - speedAtUs;
  // ▶ Chưa đủ 50ms thì bỏ qua.
  if (elapsed < SPEED_US) return;
  float dt = elapsed * 1e-6f;
  speedAtUs = nowUs;
  int32_t net[2];
  uint32_t total[2];
  readCounts(net, total);
  // ▶ beta: hệ số lọc thông thấp cho tốc độ.
  float beta = dt / (SPEED_FILTER_TAU_S + dt);
  for (int w = 0; w < 2; w++) {
    int32_t delta = net[w] - speedCount[w];
    speedCount[w] = net[w];
    // ▶ Tốc độ (cm/s) = Δxung * π*D / (CPR*dt), lọc thông thấp từng bánh.
    float measured = delta * (PI * WHEEL_DIAMETER_CM) / (CPR[w] * dt);
    speedFiltered[w] += beta * (measured - speedFiltered[w]);
  }
  // ▶ Tốc độ xe = trung bình 2 bánh.
  speedAverage = (speedFiltered[0] + speedFiltered[1]) * 0.5f;
  // ▶ PI tắt hoặc chưa tới thời gian trễ -> reset các biến PI.
  if (!ENABLE_SPEED_PI || uint32_t(now - armedAt) < SPEED_PI_DELAY_MS) {
    speedIntegral = speedRef = speedApplied = 0;
    return;
  }
  // ▶ Chạy tiến -> cần góc đích dương để PD hãm lại.
  // Forward wheel speed needs a positive angle target so the inner PD commands braking.
  // ▶ error = tốc độ - mục tiêu. p = KP*error; deltaI = KI*error*dt.
  float error = speedAverage - SPEED_TARGET_CM_S;
  float p = KP_SPEED * error, deltaI = KI_SPEED * error * dt;
  // ▶ candidate: tích phân mới, chặn ±2.5°.
  float candidate = Lab::clamp(speedIntegral + deltaI, -SPEED_ANGLE_MAX_DEG, SPEED_ANGLE_MAX_DEG);
  float proposed = p + candidate;
  // ▶ clipped: đầu ra đã chạm giới hạn và tích phân còn đẩy theo hướng đó.
  bool clipped = (proposed > SPEED_ANGLE_MAX_DEG && deltaI > 0) || (proposed < -SPEED_ANGLE_MAX_DEG && deltaI < 0);
  // ▶ Anti-windup: chỉ cộng tích phân khi không bị clip, PWM không bão hòa và |góc| < 6°.
  if (!clipped && !outputSaturated && fabsf(pd.angle) < 6) speedIntegral = candidate;
  // ▶ speedRef = p + tích phân, chặn ±2.5° -> góc đích cho PD.
  speedRef = Lab::clamp(p + speedIntegral, -SPEED_ANGLE_MAX_DEG, SPEED_ANGLE_MAX_DEG);
}

// ▶ controlTick: TRÁI TIM điều khiển, chạy mỗi 2.5ms khi RUNNING.
void controlTick(uint32_t nowUs, uint32_t now) {
  uint32_t elapsed = nowUs - controlAtUs;
  // ▶ Chưa đến chu kỳ thì thoát.
  if (elapsed < controlPeriodUs) return;
  controlAtUs = nowUs;
  // ▶ Vòng điều khiển trễ quá (> max(3×chu kỳ, 25ms)) -> fault 6.
  if(elapsed>(controlPeriodUs*3>MAX_CONTROL_US?controlPeriodUs*3:MAX_CONTROL_US)){latchFault(6);return;}
  // ▶ IMU mất hoặc tần số < 50Hz -> fault 5.
  if(!freshSensor(now)||angleHz<50||gyroHz<50){latchFault(5);return;}

  // ▶ theta = góc nghiêng so với điểm zero; rate = tốc độ góc đã bù bias.
  float theta = wrapDegrees(rawAngleDeg() - zeroAngle);
  float rate  = rawGyroDegS() - gyroBias;
  // ▶ Giá trị không hợp lệ (NaN/Inf) -> fault 7.
  if(!isfinite(theta)||!isfinite(rate)){latchFault(7);return;}
  // ▶ An toàn: ngã (|θ|>35° hoặc |θ|>8° kèm |rate|>300°/s) -> cắt motor, fault 8.
  // Fail-safe: never keep driving after the robot has fallen far from upright.
  if(fabsf(theta)>FALL_LIMIT_DEG ||
     (fabsf(theta)>8.0f && fabsf(rate)>300.0f)){
    latchFault(8);return;
  }

  // ▶ Cập nhật vòng PI tốc độ (chỉ chạy thật mỗi 50ms).
  updateSpeed(nowUs, now);
  // ▶ dt thật của tick này (không giả định cố định 2.5ms).
  float dt = elapsed * 1e-6f;
  // Do not hide a late loop by pretending dt was 5ms.
  // ▶ Làm mượt góc đích: speedApplied chỉ đổi tối đa 3°/s.
  float maxStep = REF_SLEW_DEG_PER_S * dt;
  speedApplied += Lab::clamp(speedRef - speedApplied, -maxStep, maxStep);
  // The speed PI adjusts the app's upright trim without changing its stored setting.

  // ▶ PD: command = KP*(θ - (target + speedApplied)) + KD*rate.
  float command = pd.step(theta, rate, dt, pd.config.target + speedApplied);
  // ▶ Command >= 50 thì coi là bão hòa (để chặn tích phân).
  outputSaturated = fabsf(command) >= 50.0f;

  // ▶ Đọc số xung mới nhất của cả 2 bánh.
  int32_t net[2];uint32_t total[2];readCounts(net,total);
  // ▶ Lặp cho từng bánh w = 0 (trái), 1 (phải):
  for (int w = 0; w < 2; w++) {
    // ▶ Phát hiện bánh vừa quay: lưu hướng và thời điểm.
    if(net[w]!=motionNet[w]){motionSign[w]=net[w]>motionNet[w]?1:-1;motionNet[w]=net[w];motionAt[w]=now;}
    // ▶ moving = bánh quay trong 80ms gần nhất và cùng hướng command.
    bool moving=uint32_t(now-motionAt[w])<80&&motionSign[w]==(command>0?1:-1);
    // ▶ profile = chọn mô hình theo bánh và chiều (tiến/lùi).
    int profile = w * 2 + (command < 0 ? 1 : 0);
    // ▶ PWM yêu cầu: traceMode=1 (M) cho 0 (motor tắt); ngược lại qua MotorMap (hysteresis + sàn + mô hình).
    int pwmDemand=traceMode==1?0:motorMap[w].step(command,w,models[profile],pd.config.maxPwm,moving,net[w]);
    // ▶ urgent: nghiêng > 1.8° hoặc gyro > 30°/s -> cho slew nhanh.
    bool urgent=fabsf(pd.angle)>FAST_RESCUE_ANGLE_DEG || fabsf(pd.rate)>FAST_RESCUE_RATE_DEG_S;
    // ▶ Làm mượt PWM (slew + chống đảo chiều).
    int pwmOut=smoothCommandPwm(w,pwmDemand,urgent,nowUs);
    // ▶ Đánh dấu bão hòa nếu PWM yêu cầu chạm trần.
    outputSaturated |= (abs(pwmDemand) >= pd.config.maxPwm);
    // ▶ Xuất PWM ra motor.
    writeMotor(w, pwmOut);
  }
}

// ▶ makeRecord: đóng gói 1 row 48 byte.
//   Offset: 0-3 timestamp µs | 4-5 dt | 6-7 tuổi dữ liệu góc (ms) | 8-9 tuổi gyro | 10-11 chu kỳ góc µs
//   12 góc (x100) | 14 góc lọc | 16 target | 18 gyro (x10) | 20 gyro lọc | 22 P | 24 D | 26 P+D | 28 u
//   30-37 PWM yêu cầu/thực tế 2 bánh | 38-41 Δ encoder 2 bánh | 42 cờ bit | 43 faultCode
//   44-45 số thứ tự góc | 46-47 chu kỳ gyro.
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
  // ▶ Byte cờ: bit0 |P+D|>55 | bit1 PWM bánh0 chạm trần | bit2 PWM bánh1 chạm trần | bit3 đang RUNNING & motor bật
  //   bit4 zeroValid | bit5 dt trễ >120% chu kỳ | bit6 releaseMarked | bit7 P/D tràn thang fixed-point.
  out[42]=(fabsf(pd.p+pd.d)>55?1:0)|(abs(actualPWM[0])>=pd.config.maxPwm?2:0)|(abs(actualPWM[1])>=pd.config.maxPwm?4:0)|(state==RUNNING&&traceMode!=1?8:0)|(zeroValid?16:0)|(dt>controlPeriodUs*12/10?32:0)|(releaseMarked?64:0)|((fabsf(pd.p)>327.67f||fabsf(pd.d)>327.67f||fabsf(pd.p+pd.d)>327.67f)?128:0);
  out[43]=faultCode;Trace::u16(out+44,angleSequence);Trace::u16(out+46,gyroPeriodUs>65535?65535:gyroPeriodUs);
}
// ▶ recordTick: ghi row vào bộ nhớ. Đầy thì: autoRun bỏ qua, còn lại fault 21.
// Khi autoRun: day bo nho trace thi chi dung ghi, khong bao loi 21.
void recordTick(uint32_t stamp,uint32_t dt){makeRecord(liveRow,stamp,dt);if(rowCount<Trace::MAX_ROWS)memcpy(traceRows[rowCount++],liveRow,Trace::SIZE);else if(!autoRun)latchFault(21);}
// ▶ beginTrace: bắt đầu M/B/C. Cần: state IDLE/FAULTED, trace đã xóa, có BLE+heartbeat, IMU ổn, đã có zero (trừ M).
void beginTrace(uint8_t selected,uint32_t now){
  if(state!=IDLE&&state!=FAULTED)return;
  if(rowCount){faultCode=24;return;}
  if(!connected||now-heartbeatAt>1000){faultCode=20;return;}
  if(!freshSensor(now)||angleHz<50||gyroHz<50){faultCode=5;return;}
  if(selected!=1&&!zeroValid){faultCode=23;return;}
  // ▶ Lệnh M/B/C từ app -> TẮT autoRun (quay lại chế độ an toàn qua BLE).
  autoRun=false; // lenh M/B/C tu app: tat che do tu chay, giu an toan BLE
  stopMotors();traceMode=selected;rowCount=0;traceSession++;faultCode=0;releaseMarked=false;pd.reset();
  // ▶ M chạy thẳng RUNNING (motor tắt); B/C vào PREPARING (đợi xe đứng yên 3s rồi chạy).
  if(selected==1)armMotors(now);else{state=PREPARING;prepareAt=now;int32_t net[2];readCounts(net,calAbs);}
}
// ▶ command: xử lý 1 lệnh từ app. op = ký tự đầu.
void command(const char*text,uint32_t now){
  char op=text[0];
  // ▶ X: dừng khẩn cấp (ghi 1 row cuối nếu đang RUNNING).
  if(op=='X'){if(state==RUNNING){uint32_t t=micros();latchFault(10);recordTick(t,t-controlAtUs);}else if(state==PREPARING||state==CALIBRATING)latchFault(10);else stopMotors();return;}
  // ▶ Q: gửi trạng thái.
  if(op=='Q'){sendStatus();return;}
  // ▶ K: đánh dấu thời điểm thả.
  if(op=='K'){if(state==RUNNING)releaseMarked=true;return;}
  // ▶ R<session>,<index>: tải lại 1 row (4 gói có CRC). Không cho khi đang chạy.
  if(op=='R'){
    unsigned session=0,index=0;if(sscanf(text+1,"%u,%u",&session,&index)!=2)return;
    if(state==RUNNING||state==PREPARING||state==CALIBRATING||session!=traceSession||index>=rowCount)return;
    RadioBatch b;b.count=4;for(int part=0;part<4;part++)Trace::chunk(traceRows[index],traceSession,index,part,b.packets[part]);sendBatch(b);return;
  }
  // ▶ Các lệnh sau chỉ nhận khi KHÔNG ở RUNNING/PREPARING/CALIBRATING.
  if(state==RUNNING||state==PREPARING||state==CALIBRATING)return;
  // ▶ Z: calib lại (phải xóa trace trước).
  if(op=='Z'){if(rowCount){faultCode=24;return;}quickReZero(now);return;}
  // ▶ M/B/C: chạy trace (M=motor tắt, B/C=chạy thật).
  if(op=='M'||op=='B'||op=='C'){beginTrace(op=='M'?1:op=='B'?2:3,now);return;}
  // ▶ E: xóa trace, tăng session, thoát FAULTED.
  if(op=='E'){rowCount=0;traceSession++;faultCode=0;if(state==FAULTED)state=IDLE;sendStatus();return;}
  // ▶ Từ đây là lệnh chỉnh tham số, bắt buộc trace đã xóa.
  if(rowCount){faultCode=24;return;}
  // ▶ Đọc số sau ký tự đầu bằng strtof, phải hợp lệ.
  char*end=nullptr;float value=strtof(text+1,&end);if(!end||*end||!isfinite(value)){faultCode=22;return;}
  bool ok=true;
  // ▶ P: KP 0..20.
  if(op=='P'&&value>=0&&value<=20)pd.config.kp=value;
  // ▶ D: KD 0..2.
  else if(op=='D'&&value>=0&&value<=2)pd.config.kd=value;
  // ▶ O: góc đích (trim) ±3°.
  else if(op=='O'&&fabsf(value)<=3)pd.config.target=value;
  // ▶ L: PWM tối đa 110..255 (số nguyên).
  else if(op=='L'&&value>=110&&value<=255&&value==floorf(value))pd.config.maxPwm=value;
  // ▶ T: chu kỳ điều khiển 2.5..10.0 ms.
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
  // ▶ Thành công: xóa lỗi, sai: fault 22; rồi gửi trạng thái.
  faultCode=ok?0:22;sendStatus();
}
// ▶ initBle: tạo BLE 'C3_RESPONSE_LAB', service kiểu Nordic UART.
void initBle(){
  BLEDevice::init("C3_RESPONSE_LAB");auto*server=BLEDevice::createServer();server->setCallbacks(new LinkCallbacks());
  auto*service=server->createService("6e400001-b5a3-f393-e0a9-e50e24dcca9e");
  // ▶ TX (…0003): notify gửi dữ liệu về app.
  tx=service->createCharacteristic("6e400003-b5a3-f393-e0a9-e50e24dcca9e",BLECharacteristic::PROPERTY_NOTIFY);tx->addDescriptor(new BLE2902());
  // ▶ RX (…0002): app ghi lệnh vào đây.
  auto*rx=service->createCharacteristic("6e400002-b5a3-f393-e0a9-e50e24dcca9e",BLECharacteristic::PROPERTY_WRITE);rx->setCallbacks(new CommandCallbacks());service->start();
#ifndef LAB_NATIVE
  // ▶ Tạo queue (8 lô) và task radio ưu tiên thấp.
  radioQueue=xQueueCreate(8,sizeof(RadioBatch));if(!radioQueue){latchFault(1);return;}xTaskCreate(radioTask,"trace_radio",4096,nullptr,1,nullptr);vTaskPrioritySet(nullptr,3);
#endif
  // ▶ Bắt đầu quảng bá để app tìm thấy.
  auto*ad=BLEDevice::getAdvertising();ad->addServiceUUID("6e400001-b5a3-f393-e0a9-e50e24dcca9e");ad->setScanResponse(true);ad->start();
}

// ▶ autoStartTick: TỰ KHỞI ĐỘNG. Sau calib (IDLE) hoặc sau khi ngã (fault 8 + autoRun),
//   nếu xe đứng thẳng và yên đủ lâu thì tự thả motor.
// Tu khoi dong: sau khi calib xong (IDLE) hoac sau khi nga (fault 8),
// neu xe dung thang va dung yen du lau thi tu tha motor.
void autoStartTick(uint32_t now){
  // ▶ AUTO_START tắt thì không làm gì.
  if(!AUTO_START)return;
  bool canStart=false;
  // ▶ Trường hợp 1: vừa calib xong, chưa tự chạy lần nào.
  if(state==IDLE&&zeroValid&&!autoStartDone)canStart=true;
  // ▶ Trường hợp 2: đã tự chạy, vừa ngã (fault 8) và còn zero hợp lệ -> cho chạy lại.
  else if(AUTO_RESTART_AFTER_FALL&&autoRun&&state==FAULTED&&faultCode==8&&zeroValid)canStart=true;
  // ▶ Không đủ điều kiện hoặc IMU không ổn -> reset bộ đếm 1.5s.
  if(!canStart||!freshSensor(now)||angleHz<50||gyroHz<50){autoReadyAt=0;return;}
  float theta=wrapDegrees(rawAngleDeg()-zeroAngle);
  float rate=rawGyroDegS()-gyroBias;
  // ▶ Nghiêng quá 4° hoặc đang xoay > 15°/s -> chưa 'yên', đếm lại.
  if(fabsf(theta)>AUTO_START_MAX_ANGLE_DEG||fabsf(rate)>AUTO_START_MAX_RATE_DEG_S){autoReadyAt=0;return;}
  // ▶ Lần đầu thỏa điều kiện: bắt đầu bấm giờ.
  if(!autoReadyAt){autoReadyAt=now?now:1;return;}
  // ▶ Chưa đủ 1.5s liên tục thì chờ.
  if(uint32_t(now-autoReadyAt)<AUTO_START_DELAY_MS)return;
  // ▶ Đủ rồi: đánh dấu đã tự chạy + bật autoRun.
  autoReadyAt=0;autoStartDone=true;autoRun=true;
  faultCode=0;traceMode=0;releaseMarked=false;
  // ▶ Xóa trace cũ, sang phiên mới.
  rowCount=0;traceSession++;
  // ▶ Thả motor (vào RUNNING).
  armMotors(now);   // vao RUNNING
}

// ▶ setup(): khởi tạo phần cứng một lần khi bật nguồn.
void setup() {
// ▶ Không dùng Serial (UART0) vì GPIO20/21 dùng cho encoder/motor.
// Serial/UART0 disabled intentionally. GPIO20/GPIO21 are encoder/motor pins.
  bootAt = millis();
  // ▶ Nút BOOT dùng pull-up (bấm = LOW).
  pinMode(STOP_PIN, INPUT_PULLUP);
  if (USE_LED) {
    pinMode(LED_PIN, OUTPUT);
    setLed(false);
  }
  // ▶ Khởi tạo từng motor: kéo PWM/DIR về LOW, cấu hình PWM 20kHz 8-bit, encoder có pull-up.
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
  // ▶ PWM khởi tạo thất bại -> fault 1 và dừng.
  if (!pwmReady[0] || !pwmReady[1]) {
    latchFault(1);
    return;
  }
  // ▶ Gắn ngắt encoder (cạnh lên kênh A).
  attachInterrupt(digitalPinToInterrupt(ENCODER_A[0]), encoderLeftISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A[1]), encoderRightISR, RISING);

  // ▶ Mô hình motor: PWM = a*|cmd| + b. Thứ tự: [bánh0 tiến, bánh0 lùi, bánh1 tiến, bánh1 lùi].
  // Model derived from unloaded experiments; not guaranteed under floor load.
  const float a[4] = {4.811015f, 4.789974f, 3.972431f, 4.176047f};
  const float b[4] = {31.626482f, 30.222054f, 66.105141f, 60.363602f};
  const float lo[4] = {9.79f, 10.25f, 10.92f, 9.29f};
  for (int i = 0; i < 4; i++) {
    models[i].a = a[i];
    models[i].b = b[i];
    models[i].lo = lo[i];
  }
  // ▶ Nạp tham số điều khiển mặc định vào pd.config.
  pd.config.kp = KP_THETA;
  pd.config.kd = KD_THETA;
  pd.config.target = THETA_TRIM_DEG;
  pd.config.maxPwm = PWM_MAX;

  // ▶ Tăng buffer nhận UART để không mất byte IMU.
  jy.setRxBufferSize(512);
  // ▶ Mở UART IMU ở 115200 (pollSensor sẽ tự dò baud nếu cần).
  openBaud(115200, millis());
  // ▶ Khởi tạo BLE.
  initBle();
}

// ▶ loop(): vòng chính. Thứ tự: IMU -> BLE/lệnh -> xử lý state -> điều khiển -> telemetry.
void loop(){
  // ▶ Cập nhật LED, đọc IMU, kiểm tra nút BOOT (nếu bấm lúc RUNNING thì ghi row cuối).
  uint32_t now=millis();showState(now);pollSensor(now);bool wasRunningBeforeButton=state==RUNNING;handleBootButton(now);if(wasRunningBeforeButton&&state==FAULTED)recordTick(micros(),micros()-controlAtUs);
  // ▶ Xử lý sự kiện BLE (hiện chỉ xóa cờ).
  if(bleConnectedEvent){
    bleConnectedEvent=false;
  }
  // ▶ Ngắt kết nối BLE: xóa hàng đợi gói chờ gửi.
  if(bleDisconnectedEvent){
    bleDisconnectedEvent=false;
#ifndef LAB_NATIVE
    if(radioQueue)xQueueReset(radioQueue);
#endif
  }
  // ▶ Quảng bá lại sau 300ms kể từ lúc ngắt kết nối (để BLE dọn dẹp xong).
  // Delay advertising restart slightly to let the BLE host finish disconnection cleanup.
  if(advertiseAgain && !connected && uint32_t(now-bleDisconnectedAt)>=300){
    advertiseAgain=false;
    BLEDevice::startAdvertising();
  }
  // ▶ Chỉ khi KHÔNG autoRun: mất BLE/heartbeat > 1s lúc RUNNING/PREPARING -> fault 20. autoRun thì không cắt.
  // Che do tu chay (autoRun) khong bi cat khi mat BLE.
  if(!autoRun&&(state==RUNNING||state==PREPARING)&&(!connected||uint32_t(now-heartbeatAt)>1000)){
    uint32_t t=micros();bool wasRunning=state==RUNNING;latchFault(20);if(wasRunning)recordTick(t,t-controlAtUs);
  }
  // ▶ Lấy 1 lệnh ra khỏi hàng đợi (trong vùng găng).
  char text[20]={};portENTER_CRITICAL(&commandMux);if(commandCount){strcpy(text,commands[0]);for(int i=1;i<commandCount;i++)strcpy(commands[i-1],commands[i]);commandCount--;}portEXIT_CRITICAL(&commandMux);
  // ▶ Có lệnh thì xử lý.
  if(text[0])command(text,now);
  // ▶ WAIT_SENSOR -> IDLE khi IMU ổn (khung >= 50Hz).
  if(state==WAIT_SENSOR){if(freshSensor(now)&&angleHz>=50&&gyroHz>=50)state=IDLE;}
  // ▶ Tự calib: chỉ sau >= 5s từ khi bật nguồn, xe còn nằm gần thẳng (|góc thô|<=8°) và yên (|gyro|<=5°/s).
  //   Motor luôn tắt trong lúc này. Z từ app vẫn dùng được.
  // Auto-zero starts NO EARLIER than 5 s after power-on; motors remain disabled.
  // Manual Z remains available for repositioning/recalibration from the phone.
  if(state==IDLE && !zeroValid && !autoZeroAttempted &&
     uint32_t(now-bootAt)>=5000 && freshSensor(now) && angleHz>=50 && gyroHz>=50 &&
     fabsf(rawAngleDeg())<=8.0f && fabsf(rawGyroDegS())<=5.0f) {
    autoZeroAttempted=true;
    quickReZero(now);
  }
  // ▶ CALIBRATING: quá 30s thì fault 3, không thì tiếp tục lấy mẫu zero.
  else if(state==CALIBRATING){if(uint32_t(now-calSessionAt)>30000)latchFault(3);else takeZero(now);}
  // ▶ PREPARING (lệnh B/C): đợi xe đứng yên liên tục 3s rồi arm. Nếu bánh quay hoặc |gyro|>15 thì đếm lại.
  else if(state==PREPARING){
    if(!freshSensor(now))latchFault(5);
    else {int32_t net[2];uint32_t total[2];readCounts(net,total);if(total[0]!=calAbs[0]||total[1]!=calAbs[1]||fabsf(rawGyroDegS()-gyroBias)>15){prepareAt=now;calAbs[0]=total[0];calAbs[1]=total[1];}else if(uint32_t(now-prepareAt)>=3000)armMotors(now);}
  // ▶ RUNNING: chạy controlTick; nếu vừa xong 1 tick thì ghi trace.
  }else if(state==RUNNING){
    uint32_t before=controlAtUs,stamp=micros();controlTick(stamp,now);
    // ▶ Khi không autoRun: dừng sau 4.5s hoặc gần đầy bộ nhớ (về IDLE). autoRun thì không giới hạn.
    // autoRun: khong gioi han 4,5 giay / day bo nho trace.
    if(controlAtUs!=before){recordTick(stamp,stamp-before);if(!autoRun&&state==RUNNING&&(uint32_t(stamp-captureAtUs)>=TRACE_DURATION_US||rowCount>=Trace::MAX_ROWS-1)){stopMotors();state=IDLE;}}
  }
  // ▶ Kiểm tra điều kiện tự khởi động.
  autoStartTick(now);
  // ▶ Cứ 200ms: gửi status + 1 row live (4 gói) nếu có dữ liệu hoặc chưa có row nào. Tần số BLE thấp hơn nhiều so với vòng điều khiển.
  if(now-telemetryAt>=200){telemetryAt=now;sendStatus();if((state==RUNNING&&rowCount)||rowCount==0){if(state!=RUNNING)makeRecord(liveRow,micros(),0);RadioBatch b;b.count=4;for(int part=0;part<4;part++)Trace::chunk(liveRow,traceSession,65535,part,b.packets[part],true);sendBatch(b);}}
  // ▶ Chờ ngắn để bám sát chu kỳ điều khiển.
  // Do not block for a full RTOS tick while RUNNING: short waits reduce
  // overshoot of fractional control periods. BLE radioTask runs separately.
  // ▶ RUNNING: dùng delayMicroseconds (150µs hoặc 30µs) thay vì delay(1) để giảm sai lệch chu kỳ; gần tới hạn thì quay lại kiểm tra ngay.
  if (state == RUNNING) {
    uint32_t spent = (uint32_t)(micros() - controlAtUs);
    uint32_t remaining = spent < controlPeriodUs ? controlPeriodUs-spent : 0;
    if (remaining > 700UL) delayMicroseconds(150);
    else if (remaining > 120UL) delayMicroseconds(30);
    // Close to deadline: immediately check again; no Serial/BLE blocking calls.
  } else {
    // ▶ Các state khác: nghỉ 1ms cho nhẹ CPU.
    delay(1);
  }
}
