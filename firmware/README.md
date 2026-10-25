# Hướng dẫn Nạp Firmware và Cấu hình ESP32 cho Xe Robot

Firmware được viết bằng C++ (PlatformIO / Arduino Framework) cho ESP32 DOIT DevKit, bám sát **100% Sơ đồ nguyên lý phần cứng** và đồng bộ chuẩn xác với bộ điều khiển Server Python (`server/main.py`, `server/control/controller_main.py`, `server/comms/robot_network.py`).

---

## 1. Bảng Ánh Xạ Chân Phần Cứng (Khớp 100% Sơ Đồ Nguyên Lý)

| Thiết bị ngoại vi | Chân trên Sơ đồ | Chân ESP32 | Chức năng trong Firmware |
| :--- | :--- | :--- | :--- |
| **Động cơ Bánh Trái (DC)** | `IN1`, `IN2` | **`D26`**, **`D25`** | Cầu H DRV8833 điều khiển bánh trái (OUT1, OUT2) |
| **Động cơ Bánh Phải (DC)** | `in3`, `in4` | **`D33`**, **`D32`** | Cầu H DRV8833 điều khiển bánh phải (out3, out4) |
| **DRV8833 nSLEEP** | `EEP` | **`D15`** | Kích hoạt mạch cầu H (mức HIGH để bật) |
| **DRV8833 nFAULT** | `UTL` | **`D5`** | Báo lỗi quá nhiệt / quá tải (kèm trở 10k kéo lên 3V3) |
| **ServoA (Nâng/Gắp Pallet)** | `PWM3` | **`D13`** | Servo điều khiển góc cơ cấu pallet A |
| **ServoB (Nâng/Gắp Pallet)** | `PWM4` | **`D14`** | Servo điều khiển góc cơ cấu pallet B |
| **MPU6050 SCL** | `SCL` | **`D23`** | I2C Clock |
| **MPU6050 SDA** | `SDA` | **`D21`** | I2C Data |
| **Đèn LED Đỏ** | `LED_RED` | **`D4`** | Báo mất Wi-Fi / Timeout lệnh / Lỗi DRV8833 |
| **Đèn LED Xanh** | `LED_BLUE` | **`D2`** | Báo Wi-Fi online / Nháy khi nhận gói tin lệnh |

---

## 2. Giao Thức Kết Nối và Đồng Bộ với Server Python

* **Địa chỉ IP Static**: Khớp bảng `ROBOT_IP_MAP` trong `server/comms/robot_network.py`:
  * Xe 1: `192.168.1.101`
  * Xe 2: `192.168.1.102`
  * Xe 3: `192.168.1.103`
* **Cổng dịch vụ**: Cổng **80** (HTTP)
* **Các Endpoint**:
  * `GET /`: Phản hồi `PARROT Robot OK` (dùng cho hàm `network.ping_all()`).
  * `POST /cmd`: Nhận JSON `{"l": <left>, "r": <right>, "action": <0|1|2>}`
    * `l`: Tốc độ bánh trái (Server gửi quanh 90: `50` lùi tối đa, `90` dừng, `130` tiến tối đa. Firmware tự động map mượt sang PWM động cơ DC `[0..255]`).
    * `r`: Tốc độ bánh phải (Tương tự bánh trái).
    * `action`:
      * `0`: Giữ nguyên vị trí servo pallet
      * `1`: **Nâng / Gắp Pallet** (khi xe đến vị trí pallet)
      * `2`: **Hạ / Trả Pallet** (khi xe đến vị trí goal)
  * `GET /stop`: Phanh xe dừng khẩn cấp.
  * `GET /status`: Trả về JSON tình trạng (tốc độ motor, góc servo, yaw IMU, fault).
* **Failsafe Watchdog**: Nếu mất kết nối với server quá **800ms**, xe tự động dừng động cơ bánh xe và bật đèn đỏ cảnh báo an toàn.

---

## 3. Cách Cấu Hình và Nạp Code Cho Từng Xe

### Bước 1: Chọn ID cho xe cần nạp
Mở file `firmware/config.h`:
* Nếu nạp cho **Xe 1**: đặt `constexpr uint8_t ROBOT_ID = 1;` (IP: `192.168.1.101`)
* Nếu nạp cho **Xe 2**: đặt `constexpr uint8_t ROBOT_ID = 2;` (IP: `192.168.1.102`)
* Nếu nạp cho **Xe 3**: đặt `constexpr uint8_t ROBOT_ID = 3;` (IP: `192.168.1.103`)

Kiểm tra tên Wi-Fi và mật khẩu trong `firmware/config_private.h`:
```cpp
constexpr char WIFI_SSID[] = "Hang";
constexpr char WIFI_PASSWORD[] = "0971397361";
```

### Bước 2: Cắm cáp USB và Nạp code
Mở terminal tại thư mục `firmware`:
```powershell
cd firmware
pio run --target upload
```
*(Hoặc dùng nút mũi tên **PlatformIO: Upload** trong VS Code).*

### Bước 3: Xem Serial Monitor
```powershell
pio device monitor
```
Màn hình sẽ hiển thị:
```text
=================================
  PARROT MULTI-ROBOT (ROBOT 1)
=================================
[Wi-Fi] Đang kết nối tới 'Hang'...
[Wi-Fi] KẾT NỐI THÀNH CÔNG! Robot ID: 1 | IP: 192.168.1.101
[HTTP Server] Sẵn sàng lắng nghe trên cổng 80 tại endpoint /cmd
IMU READY
[System] KHỞI ĐỘNG HOÀN TẤT. SẴN SÀNG NHẬN LỆNH.
```

---

## 4. Lệnh Test Trực Tiếp Qua Serial Monitor (USB)

Bạn có thể gõ các lệnh sau vào Serial Monitor để test động cơ và servo trước khi chạy qua mạng:
* `M 110 110`: Cho 2 bánh xe DC quay tiến
* `M 70 70`: Cho 2 bánh xe DC quay lùi
* `STOP`: Dừng bánh xe ngay lập tức
* `LIFT`: Thử nâng cơ cấu pallet (ServoA & ServoB quay góc nâng)
* `LOWER`: Thử hạ cơ cấu pallet (ServoA & ServoB quay góc hạ)
* `SERVO 45 45`: Đặt góc servo cụ thể (0 - 180 độ)
* `STATUS`: Xem toàn bộ thông số tốc độ, góc servo, IMU và lỗi
