# C3 FineSweep chống giật v3 (ESP32-C3 + JY61 UART)

## Bản mới đã thay đổi
- Tốc độ vòng PD mặc định: 2,5 ms, có thể đổi qua BLE 2,5–10,0 ms, bước 0,1 ms.
- Kp=10,5; Kd=0,25; lọc gyro với hằng số 10 ms (trước là 5 ms).
- Đã bật vòng PI tốc độ bánh ngoài, mục tiêu mặc định 0 cm/s. Tốc độ tiến dương tạo góc đích dương để vòng PD góc ra lệnh hãm; góc bù giới hạn ±2,5° và thay đổi tối đa 3°/s.
- Làm mềm lệnh PWM gần thẳng: tăng ≤22, giảm ≤30 đơn vị PWM/chu kỳ. Khi |góc|>1,8° hoặc |gyro lọc|>30°/s, cho phép tăng nhanh ≤75, giảm nhanh ≤90 đơn vị/chu kỳ.
- Đảo chiều qua PWM=0, giữ trung tính tối thiểu 5 ms, không delay chặn CPU.
- Không giảm PWM tối thiểu xuống 55 chỉ vì encoder vừa có xung. Giữ mức tham chiếu theo chiều: tiến trái 120, tiến phải 110, lùi trái 130, lùi phải 120. Cần hiệu chỉnh khi chạy trên sàn.
- App khóa nút thử motor nếu chưa nhận gói cấu hình xác nhận đúng Ts/Kp/Kd/PWM từ ESP32. Lệnh Áp dụng gửi P, D, O, L, T và truy vấn Q; thông báo 'ESP32 đã xác nhận…' chỉ hiện sau gói xác nhận.
- Giữ BLE C3_RESPONSE_LAB, giao thức trace 48 byte, lưu CSV, so sánh các chu kỳ, dừng bằng nút BOOT hoặc app, timeout BLE, ngã >35°.

## Nạp và thử
1. Arduino IDE > chọn ESP32C3 Dev Module > mở C3_FineSweep_AntiJerk_v3.ino > Upload. Không cần TraceWire.h.
2. Android Studio > mở thư mục Android (có gradle/wrapper) > Sync > Build APK > cài app mới. Cần JDK 17, Gradle Wrapper 8.6 và Android Gradle Plugin 8.3.2.
3. Kê cả hai bánh, giữ xe thẳng và bất động lúc tự hiệu chuẩn sau 5 giây. Tuyệt đối không bật thử motor nếu phản ứng sai chiều.
4. Kết nối BLE > chọn chu kỳ 2,5 ms > bấm Áp dụng > ĐỢI app hiện 'ESP32 đã xác nhận Ts 2.5 ms…'.
5. Chọn KÊ HAI BÁNH để kiểm tra chiều phản hồi, trước khi thử TRÊN SÀN với giá đỡ chống ngã.
6. Chỉ có thể so sánh hiệu quả trên sàn khi `motor_enabled=1` và có đánh dấu lúc thả tay.
7. Vòng tốc độ mới cần xác nhận bằng thử nghiệm có người giữ xe: khi encoder báo xe chạy tiến, góc đích tốc độ phải chuyển dương và tổng lệnh PD phải có xu hướng hãm. Dừng ngay nếu xe tăng tốc hoặc PWM giữ trần mà góc vẫn tăng.

## Giới hạn
- Không thể bảo đảm xe tự đứng chỉ bằng sửa phần mềm; cần kiểm tra chiều motor, mômen, nguồn L298N, trọng tâm và dữ liệu JY61.
- JY61 thực tế đo ~100 Hz; PD 400 Hz sẽ dùng lại dữ liệu góc một số chu kỳ. Đây không phải 400 mẫu góc mới/giây.
- Các ngưỡng PWM được đo khi bánh không tải. Giới hạn bước nhảy PWM có thể làm phản ứng chậm hơn; nếu mất khả năng cứu khi nghiêng lớn, dừng ngay và đánh giá lại.
- Không khởi tạo Serial/UART0: bảo vệ GPIO20 (encoder) và GPIO21 (DIR motor) không bị chiếm bởi UART0 trong các cấu hình ESP32-C3. ESP cấp nguồn pin; BLE dùng để giám sát và điều khiển thử nghiệm.
