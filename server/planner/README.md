# Hướng dẫn Build và Kiểm Thử `planner_core` C++ Extension

## 1. Yêu cầu môi trường

| Công cụ | Phiên bản tối thiểu | Ghi chú |
|---|---|---|
| Python | 3.10+ | Đã kích hoạt `.venv` của `server/` |
| CMake | 3.12+ | Cần thêm vào PATH |
| Trình biên dịch | MSVC 2019+ (Windows) / GCC 9+ (Linux) | Cần cài Build Tools |
| Git | Bất kỳ | Dùng để FetchContent tải pybind11 |

> **Windows:** Cài Visual Studio Build Tools 2019+ với gói "Desktop development with C++".  
> **Linux:** `sudo apt install build-essential cmake`

---

## 2. Build trên Windows (PowerShell)

```powershell
# Bước 1: Di chuyển vào thư mục planner
cd server\planner

# Bước 2: Cấu hình CMake (tải pybind11 và generate project files)
cmake -B build

# Bước 3: Biên dịch
cmake --build build --config Release

# Kiểm tra file output
dir planner_core*.pyd
```

Sau khi thành công, bạn sẽ thấy file như:
```
planner_core.cp310-win_amd64.pyd   ← Python 3.10 trên Windows x64
```

---

## 3. Build trên Linux / macOS

```bash
# Bước 1: Di chuyển vào thư mục planner
cd server/planner

# Bước 2: Cấu hình và build
cmake -B build
cmake --build build --config Release

# Kiểm tra file output
ls planner_core*.so
```

---

## 4. Chạy kiểm thử độc lập

```powershell
# Windows (từ thư mục server/planner)
python test_planner.py
```

```bash
# Linux/macOS
python3 test_planner.py
```

**Output mong đợi:**
```
==============================================================
  PARROT Planner Module — Standalone Verification Test
==============================================================
[PlannerService] Khởi tạo thành công cho 2 robot(s).

[Test 1] Lập kế hoạch cơ bản...
  Thời gian: 3842.5 ms
  Số robot có quỹ đạo: 2/2
  Robot 0: 127 waypoints | tag=1: ✓ | tag=2: ✓
  Robot 1: 114 waypoints | tag=1: ✓ | tag=2: ✓

[Test 2] Stress test 50 chu kỳ kiểm tra rò rỉ RAM...
  ...
  ✓ RAM ổn định (tăng +2.1 MB — trong mức cho phép)

[Test 3] Kiểm tra validation đầu vào...
  ✓ ValueError bắt đúng: ...
  ✓ Không có pallet -> trả về paths rỗng bình thường

==============================================================
  KẾT QUẢ KIỂM THỬ
==============================================================
  ✓ PASS  Lập kế hoạch cơ bản
  ✓ PASS  Stress test RAM
  ✓ PASS  Validation đầu vào
==============================================================
  ✅ Tất cả kiểm thử ĐẠT — Module sẵn sàng tích hợp!
==============================================================
```

---

## 5. Sử dụng trong `main_system.py`

```python
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'planner'))

from planner_service import PlannerService

# Khởi tạo (1 lần duy nhất)
planner = PlannerService(num_robots=3)

# Trong vòng lặp chính — sau khi CV cập nhật poses
paths = planner.plan_trajectories(
    map_size=[2000, 1500],          # mm
    robot_poses=robots,             # [[x, y, theta], ...]
    pallet_poses=pallets,           # [[x, y, theta], ...]
    goal_poses=goals                # [[x, y, theta], ...]
)

# paths[i] = [[x(mm), y(mm), theta(rad), time(s), tag], ...]
# tag: 0=đang đi, 1=gắp pallet, 2=giao pallet
```

---

## 6. Cấu trúc module sau khi build

```
server/planner/
├── planner_core.cp310-win_amd64.pyd  ← File biên dịch (Windows)
├── planner_core.cpython-310.so       ← File biên dịch (Linux)
├── CMakeLists.txt
├── include/
│   ├── planner_constants.hpp
│   ├── search_node.hpp               ← SỬA: Hash collision fixed
│   ├── task_assigner.hpp
│   ├── motion_planner_4d.hpp         ← SỬA: Memory leak fixed
│   └── planner_coordinator.hpp       ← SỬA: No global state
├── src/
│   ├── search_node.cpp
│   ├── task_assigner.cpp
│   ├── motion_planner_4d.cpp
│   ├── planner_coordinator.cpp
│   └── planner_bindings.cpp          ← SỬA: Class binding, no global ptr
├── path_postprocessor.py             ← SỬA: Moved from mainHelper.py
├── planner_service.py                ← Python Facade API
├── test_planner.py                   ← Kiểm thử độc lập
└── build/                            ← CMake build directory (git-ignored)
```

---

## 7. Xử lý lỗi thường gặp

| Lỗi | Nguyên nhân | Giải pháp |
|---|---|---|
| `ImportError: No module named 'planner_core'` | Chưa biên dịch | Chạy `cmake -B build && cmake --build build --config Release` |
| `cmake` không tìm thấy Python | Python không trong PATH | Thêm `-DPYTHON_EXECUTABLE=$(which python3)` vào lệnh cmake |
| `MSVC not found` trên Windows | Chưa cài Build Tools | Cài Visual Studio Build Tools, chọn "Desktop development with C++" |
| Lỗi `M_PI` undeclared | Compiler thiếu define | Đã được xử lý trong `planner_constants.hpp` với `#ifndef M_PI` |
| Build thành công nhưng không thấy `.pyd` | Output dir sai | File nằm trong `server/planner/` (xem `LIBRARY_OUTPUT_DIRECTORY` trong CMakeLists.txt) |
