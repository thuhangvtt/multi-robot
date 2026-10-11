"""
test_planner.py
===============
Script kiểm thử độc lập cho module planner.

Mục đích:
    - Xác nhận C++ extension planner_core đã được biên dịch và hoạt động đúng.
    - Kiểm tra tính đúng đắn của thuật toán (waypoints hợp lệ, tag chính xác).
    - Phát hiện rò rỉ bộ nhớ (RAM tăng dần khi chạy nhiều chu kỳ).

Yêu cầu:
    - Đã biên dịch planner_core: cmake -B build && cmake --build build --config Release
    - Không cần camera, không cần phần cứng robot.

Cách chạy:
    cd server/planner
    python test_planner.py
"""

import time
import os
import sys

if sys.stdout and hasattr(sys.stdout, 'reconfigure'):
    try:
        sys.stdout.reconfigure(encoding='utf-8', errors='replace')
    except Exception:
        pass
if sys.stderr and hasattr(sys.stderr, 'reconfigure'):
    try:
        sys.stderr.reconfigure(encoding='utf-8', errors='replace')
    except Exception:
        pass

# Đảm bảo import được planner_service và planner_core
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
if BASE_DIR not in sys.path:
    sys.path.insert(0, BASE_DIR)

from planner_service import PlannerService


# =============================================================================
# Cấu hình test
# =============================================================================
MAP_SIZE = [2000, 1500]  # mm (sandbox 2000mm x 1500mm)

ROBOT_POSES = [
    [300.0, 300.0, 0.0],   # Robot 0: góc dưới bên trái, hướng phải
    [300.0, 1200.0, 0.0],  # Robot 1: góc trên bên trái, hướng phải
]

PALLET_POSES = [
    [800.0, 500.0, 0.0],   # Pallet 0: giữa bên trái
    [800.0, 1000.0, 0.0],  # Pallet 1: giữa bên phải
]

GOAL_POSES = [
    [1500.0, 500.0, 0.0],  # Goal 0 (cho Pallet 0): bên phải
    [1500.0, 1000.0, 0.0], # Goal 1 (cho Pallet 1): bên phải
]

NUM_ROBOTS = len(ROBOT_POSES)
STRESS_TEST_CYCLES = 50


# =============================================================================
# Test 1: Kiểm tra cơ bản (sinh waypoints, kiểm tra tag)
# =============================================================================
def test_basic_planning(service: PlannerService) -> bool:
    print("\n[Test 1] Lập kế hoạch cơ bản...")

    start_t = time.time()
    paths = service.plan_trajectories(MAP_SIZE, ROBOT_POSES, PALLET_POSES, GOAL_POSES)
    elapsed_ms = (time.time() - start_t) * 1000

    print(f"  Thời gian: {elapsed_ms:.1f} ms")
    print(f"  Số robot có quỹ đạo: {sum(1 for p in paths if p)}/{NUM_ROBOTS}")

    all_passed = True

    for i, path in enumerate(paths):
        if not path:
            print(f"  ❌ Robot {i}: Không tìm được đường!")
            all_passed = False
            continue

        tags = [wp[4] for wp in path]
        has_pallet_tag  = 1 in tags  # tag=1: điểm gắp pallet
        has_goal_tag    = 2 in tags  # tag=2: điểm giao pallet

        print(f"  Robot {i}: {len(path)} waypoints | "
              f"tag=1: {'✓' if has_pallet_tag else '❌'} | "
              f"tag=2: {'✓' if has_goal_tag else '❌'}")

        if not has_pallet_tag:
            print(f"    ❌ Robot {i}: Thiếu waypoint pickup (tag=1)!")
            all_passed = False
        if not has_goal_tag:
            print(f"    ❌ Robot {i}: Thiếu waypoint drop-off (tag=2)!")
            all_passed = False

        if path:
            print(f"    Điểm đầu : {[round(v, 2) for v in path[0]]}")
            print(f"    Điểm cuối: {[round(v, 2) for v in path[-1]]}")

    return all_passed


# =============================================================================
# Test 2: Kiểm tra không rò rỉ bộ nhớ (50 chu kỳ)
# =============================================================================
def test_memory_stress(service: PlannerService) -> bool:
    print(f"\n[Test 2] Stress test {STRESS_TEST_CYCLES} chu kỳ kiểm tra rò rỉ RAM...")

    try:
        import psutil
        process = psutil.Process(os.getpid())
        mem_before = process.memory_info().rss / 1024 / 1024  # MB
        has_psutil = True
    except ImportError:
        has_psutil = False
        print("  (psutil không có sẵn — bỏ qua đo RAM, chỉ kiểm tra không crash)")

    start_t = time.time()
    for cycle in range(STRESS_TEST_CYCLES):
        _ = service.plan_trajectories(MAP_SIZE, ROBOT_POSES, PALLET_POSES, GOAL_POSES)
        if (cycle + 1) % 10 == 0:
            print(f"  Chu kỳ {cycle + 1}/{STRESS_TEST_CYCLES}...")
    elapsed = time.time() - start_t

    if has_psutil:
        mem_after = process.memory_info().rss / 1024 / 1024  # MB
        mem_delta = mem_after - mem_before
        print(f"  RAM trước: {mem_before:.1f} MB | Sau: {mem_after:.1f} MB | Delta: {mem_delta:+.1f} MB")

        # Cho phép tăng tối đa 50MB (bình thường do Python heap management)
        if mem_delta > 50:
            print(f"  ❌ RAM tăng quá nhiều ({mem_delta:.1f} MB) — nghi ngờ memory leak!")
            return False
        else:
            print(f"  ✓ RAM ổn định (tăng {mem_delta:.1f} MB — trong mức cho phép)")
    else:
        print(f"  ✓ Hoàn thành {STRESS_TEST_CYCLES} chu kỳ không crash ({elapsed:.1f}s)")

    return True


# =============================================================================
# Test 3: Kiểm tra validation đầu vào
# =============================================================================
def test_input_validation(service: PlannerService) -> bool:
    print("\n[Test 3] Kiểm tra validation đầu vào...")

    # Pallet nhiều hơn goal -> phải raise ValueError
    try:
        service.plan_trajectories(
            MAP_SIZE,
            ROBOT_POSES,
            PALLET_POSES + [[100.0, 100.0, 0.0]],  # Thêm pallet thừa
            GOAL_POSES
        )
        print("  ❌ Không bắt được ValueError khi pallet != goal!")
        return False
    except ValueError as e:
        print(f"  ✓ ValueError bắt đúng: {e}")

    # Không có pallet -> trả về paths rỗng, không crash
    try:
        empty_paths = service.plan_trajectories(MAP_SIZE, ROBOT_POSES, [], [])
        all_empty = all(len(p) == 0 for p in empty_paths)
        if all_empty:
            print("  ✓ Không có pallet -> trả về paths rỗng bình thường")
        else:
            print("  ❌ Không có pallet nhưng paths không rỗng!")
            return False
    except Exception as e:
        print(f"  ❌ Exception không mong đợi: {e}")
        return False

    return True


# =============================================================================
# Main
# =============================================================================
def main():
    print("=" * 62)
    print("  PARROT Planner Module — Standalone Verification Test")
    print("=" * 62)

    # Khởi tạo service
    service = PlannerService(num_robots=NUM_ROBOTS)

    if not service.is_available:
        print("\n❌ planner_core chưa sẵn sàng — dừng test.")
        print("   Biên dịch trước bằng lệnh:")
        print("   cmake -B build && cmake --build build --config Release")
        sys.exit(1)

    # Chạy tất cả test
    results = {
        "Lập kế hoạch cơ bản": test_basic_planning(service),
        "Stress test RAM":      test_memory_stress(service),
        "Validation đầu vào":  test_input_validation(service),
    }

    # Tổng kết
    print("\n" + "=" * 62)
    print("  KẾT QUẢ KIỂM THỬ")
    print("=" * 62)
    all_passed = True
    for name, passed in results.items():
        status = "✓ PASS" if passed else "❌ FAIL"
        print(f"  {status}  {name}")
        if not passed:
            all_passed = False

    print("=" * 62)
    if all_passed:
        print("  ✅ Tất cả kiểm thử ĐẠT — Module sẵn sàng tích hợp!")
    else:
        print("  ❌ Một số kiểm thử THẤT BẠI — Xem chi tiết bên trên.")
    print("=" * 62)

    sys.exit(0 if all_passed else 1)


if __name__ == "__main__":
    main()
