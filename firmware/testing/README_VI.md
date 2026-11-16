# Bốn bài test — nạp từng bài, làm theo thứ tự

Đặt đúng `ROBOT_ID` trong `firmware/config.h` trước khi nạp mỗi xe. Dùng Serial Monitor 115200 baud, Newline; thay COM5 bằng cổng thật và đóng monitor trước upload. Lệnh dưới chạy từ thư mục gốc repo. Các bài không dùng Wi-Fi. **Riêng `test_speed` tự chạy sau bật nguồn/reset theo thời gian đặt sẵn; ba bài còn lại chờ lệnh.**




Các lệnh dưới đây chạy từ **thư mục gốc repo**. Nếu đã đứng trong thư mục firmware thì bỏ `-d firmware`.

| Thứ tự | File nạp | Mục tiêu | Environment |
|---|---|---|---|
| 1 | [test_pwm.cpp](test_pwm.cpp) | Tìm duty bánh bắt đầu quay, xác nhận chiều trái/phải | `test_pwm` |
| 2 | [test_speed.cpp](test_speed.cpp) | Đo quãng đường/thời gian để tìm v, thử cả cua | `test_speed` |
| 3 | [test_imu.cpp](test_imu.cpp) | Xem gyro/yaw khi đứng yên và xoay tay | `test_imu` |
| 4 | [test_pallet.cpp](test_pallet.cpp) | Chốt góc nâng/hạ và thử gắp thật | `test_pallet` |

`serial_line.h` và `motor_pulse.h` chỉ là phần dùng chung; **không nạp riêng**. Firmware vận hành mặc định vẫn là `esp32dev`.

## Test 1 — PWM

Nạp rồi mở monitor:

```powershell
pio run -d firmware -e test_pwm --target upload --upload-port COM5
pio device monitor -p COM5 -b 115200
```

Kê bánh khỏi mặt đất ở lần thử đầu. Gõ lần lượt, đợi xung trước kết thúc:

```text
PWM 20 0 1000
PWM 25 0 1000
PWM 30 0 1000
```

Đây là duty DC **trực tiếp**, không phải lệnh servo quanh 90. Chưa quay thì tăng từng bước nhỏ; thấy bánh quay đều thì ghi lại. Không cần thử đến mức cao nhất.

```text
PWM 0 30 1000
PWM -30 0 1000
PWM 0 -30 1000
PWM 30 30 1000
STOP
```

Các số 20/25/30 chỉ là mức bắt đầu khảo sát; không bảo đảm bánh sẽ quay. Nếu duty đã đủ mà vẫn không quay, xem `STATUS`, nFAULT, nguồn và dây trước khi tăng tiếp. Trần hiện tại lấy từ `DC_MOTOR_MAX_PWM`, in lúc khởi động; lệnh vượt trần bị từ chối. Mỗi xung từ 100–5000 ms rồi tự dừng; `STOP` cắt sớm.

**Ghi được:** chiều từng bánh, duty bắt đầu quay và duy trì quay cho mỗi chiều. Lặp tối thiểu 3 lần. Raw test không bù gyro, không trim, không chạy IMU nên dễ nhìn đặc tính motor.

## Test 2 - automatic speed test (no USB while moving)

See [SPEED_AUTO_VI.md](SPEED_AUTO_VI.md) for the updated Vietnamese instructions.

Edit TEST_LEFT, TEST_RIGHT, TEST_RUN_MS and TEST_START_DELAY_MS at the top of test_speed.cpp, then upload environment test_speed. The robot waits 5 seconds after reset/power-up, runs once for 2 seconds at raw duty 40 by default, then stops. No BOOT or Serial command is required. RUN/DIST commands from the previous version are no longer used.

## Test 3 — IMU

```powershell
pio run -d firmware -e test_imu --target upload --upload-port COM5
pio device monitor -p COM5 -b 115200
```

Giữ xe **đứng yên lúc bật/reset**, vì firmware lấy bias gyro lúc khởi tạo. Test này luôn dừng motor, không nhận lệnh chạy bánh.

1. Để yên và nhìn cột `gyro_z_deg_s`: xem giá trị có gần 0 và ổn định không, ghi lại độ dao động.
2. Gõ `ZERO`, xoay xe **bằng tay** một góc 90° theo mốc/thước góc. Xem `yaw_deg` đổi bao nhiêu và mang dấu gì.
3. Quay chiều ngược để kiểm dấu; thử 180° và quan sát yaw trôi khi đứng yên.

Lệnh: `ZERO`, `STATUS`, `PAUSE`, `STREAM`. `ZERO` chỉ đặt gốc yaw, không hiệu chuẩn lại bias. Không lấy chính yaw hiển thị làm góc chuẩn để đánh giá độ chính xác.

**Ghi được:** dấu khi quay trái/phải, gyro lúc đứng yên và sai số yaw so với góc đo ngoài. Chưa tune gain trợ lái trong bài này.

## Test 4 — nâng/hạ pallet

```powershell
pio run -d firmware -e test_pallet --target upload --upload-port COM5
pio device monitor -p COM5 -b 115200
```

Bánh xe luôn dừng. Servo **chưa attach** khi vừa khởi động. Kiểm khoảng trống cơ cấu trước khi gõ:

```text
ARM
STATUS
```

`ARM` attach servo và đưa tới góc hạ đang cấu hình. Sau đó:

```text
SERVO 10 10
SERVO 15 15
STATUS
```

Chỉnh từng góc nhỏ theo cơ khí của bạn; không quét tự động 0–180°. Tìm góc hạ/nâng không kẹt, riêng A và B. Ghi kết quả vào bốn `SERVO_*_ANGLE` trong `config.h`, nạp lại bài test, rồi thử:

```text
LIFT
LOWER
```

Đặt xe đúng vị trí dưới/trước pallet theo cơ cấu thật và kiểm **pallet có được nâng/nhả thật hay không**. `commanded_lifted` chỉ là trạng thái lệnh. Quay video để đo thời gian cơ cấu ổn định, thử nhiều lần có tải. Servo bị ép vào điểm kẹt thì ngắt nguồn; không tiếp tục tăng góc.

**Ghi được:** bốn góc servo, thời gian nâng/hạ và vị trí/hướng gắp thành công. Chưa cho xe tự chạy đến pallet trong bài này.

## Đo xong một xe

Ghi kết quả theo xe vào [results/README.md](results/README.md); hồ sơ chi tiết cũ ở `firmware/test/results/robot_1/profile.json` vẫn dùng được, không cần điền hết ngay.

Nạp lại firmware vận hành trước khi chạy server:

```powershell
pio run -d firmware -e esp32dev --target upload --upload-port COM5
```

**Không nạp cả bốn bài cùng lúc.** Chọn một environment bằng `-e`; thay bài test nghĩa là thay firmware đang nằm trên ESP32. Các bài này giúp lấy số thực, chưa thay thế kiểm thử xe đi đường cong rồi gắp pallet trong toàn hệ.
