# Hướng dẫn Cài đặt & Chạy Live Stream Camera (Windows)

Tài liệu này hướng dẫn chi tiết cách thiết lập môi trường, cài đặt thư viện và chạy test luồng camera trực tiếp (Live Stream) để kiểm tra nhận diện ArUco trên hệ điều hành Windows. Đặc biệt bao gồm hướng dẫn khi sử dụng điện thoại làm webcam qua các phần mềm như Camo, EpocCam, DroidCam.

---

## Phần 1: Thiết lập môi trường ảo (Virtual Environment)

Sử dụng môi trường ảo giúp cô lập các thư viện của dự án này khỏi hệ thống Python trên máy tính của bạn.

1. **Mở Terminal (Command Prompt hoặc PowerShell)**
   Mở terminal và trỏ vào thư mục `server` của dự án:
   ```powershell
   cd \Robot-Controller-main\Robot-Controller-main\server
   ```

2. **Tạo môi trường ảo (.venv)**
   Chạy lệnh sau để tạo một môi trường ảo có tên `.venv`:
   ```powershell
   python -m venv .venv
   ```

3. **Kích hoạt môi trường ảo**
   Trên Windows, bạn kích hoạt bằng lệnh:
   ```powershell
   .venv\Scripts\activate
   ```
   *(Thành công khi bạn thấy chữ `(.venv)` xuất hiện ở đầu dòng lệnh).*

---

## Phần 2: Cài đặt thư viện

Đảm bảo bạn đã kích hoạt `.venv`. Sau đó, cài đặt các thư viện cần thiết từ file `requirements.txt`:

```powershell
pip install -r requirements.txt
```

*Lưu ý:* Thư viện quan trọng nhất là `opencv-contrib-python`. Chữ `contrib` là bắt buộc vì nó chứa module `cv.aruco` độc quyền.

---

## Phần 3: Cấu hình Camera Điện thoại (Camo / DroidCam / v.v.)

Khi bạn dùng điện thoại làm webcam trên Windows, phần mềm (như Camo) sẽ tạo ra một **Camera ảo (Virtual Webcam)**. OpenCV hoàn toàn có thể đọc được camera ảo này, nhưng bạn cần cấu hình đúng ID.

1. Mở file `server/config/constants.py`
2. Tìm dòng `WEBCAM_ID = 1` (khoảng dòng 29)
3. **Cách xác định ID:**
   - `WEBCAM_ID = 0`: Thường là webcam có sẵn tích hợp trên Laptop.
   - `WEBCAM_ID = 1` (hoặc `2`, `3`): Thường là camera cắm ngoài qua USB hoặc Camera ảo (Camo).
   - *Mẹo:* Nếu chạy lỗi hoặc nó mở nhầm webcam của laptop, hãy thử đổi qua lại giữa `0`, `1`, `2`.

### ⚠️ Lưu ý kỹ thuật cực kỳ quan trọng cho Windows:
Đôi khi OpenCV trên Windows đọc Camera ảo bị lỗi (màn hình đen, giật lag, hoặc văng lỗi). Để khắc phục chắc chắn 100%, bạn nên báo cho tôi biết để thêm cờ `cv.CAP_DSHOW` (DirectShow) vào file `cv_main.py` ở đoạn gọi camera.

Cụ thể, bên trong file `server/vision/cv_main.py` (dòng 63):
```python
# Code hiện tại:
cap = cv.VideoCapture(webcam_id)

# Nên sửa thành (Nếu gặp lỗi mở camera):
cap = cv.VideoCapture(webcam_id, cv.CAP_DSHOW)
```

---

## Phần 4: Chạy Test Live Stream (`main_system.py`)

Sau khi đã bật phần mềm Camo/ứng dụng camera trên điện thoại và kết nối với máy tính, hãy thực hiện test:

1. Dán 4 mã góc ArUco (ID: 1, 4, 7, 10) tạo thành một hình chữ nhật trên sàn.
2. Đặt camera điện thoại ngó từ trên xuống (cố gắng giữ góc thẳng đứng nhất có thể để tránh Lỗi thị sai - Parallax Error).
3. Đảm bảo camera nhìn thấy đủ 4 góc.
4. Chạy lệnh:
   ```powershell
   python vision/main_system.py
   ```

**Kịch bản xảy ra:**
- Hệ thống sẽ in ra terminal: `[CV] Camera thread started... warming up...`
- Sau đó in ra: `Waiting for sandbox corners to be detected...`
- Nếu camera nhìn rõ 4 mã 1, 4, 7, 10, hệ thống sẽ chớp lấy ảnh, cắt phẳng, và hiện lên cửa sổ tên **"Robot Visualizer"**.
- Ném một con Robot (có dán mã 0, 3, 5) hoặc Pallet (mã 2, 6) vào giữa 4 góc, bạn sẽ thấy chấm đỏ và mũi tên liên tục nhảy theo tụi nó trên màn hình máy tính theo thời gian thực (Live)!

**Cách thoát:** 
Bấm phím `q` hoặc phím `ESC` khi cửa sổ hình ảnh đang mở để thoát luồng an toàn.
