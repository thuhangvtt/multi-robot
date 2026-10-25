"""
test_vision_plan.py
-------------------
Kiểm thử kết hợp Vision (ArUco detect) + Motion Planning (A*) trên ảnh tĩnh.

Workflow:
  1. Load ảnh từ  test_vision_plan/images/
  2. Vision pipeline: detect marker góc, warp sandbox, detect robot/pallet/goal
  3. Planner: tính đường đi A* cho từng robot
  4. Visualize: vẽ robot, pallet, goal, planned path lên ảnh
  5. Lưu kết quả ra test_vision_plan/results/

Đặt ảnh đầu vào tại:
    server/vision/test_vision_plan/images/
    (bất kỳ tên file *.jpg / *.jpeg / *.png / *.bmp đều được xử lý)

Kết quả lưu tại:
    server/vision/test_vision_plan/results/
"""

import sys
import os
import glob
import traceback
import cv2 as cv
import numpy as np

# ---------------------------------------------------------------------------
# Path setup — thêm thư mục server và vision vào sys.path
# ---------------------------------------------------------------------------
_VISION_DIR = os.path.dirname(os.path.abspath(__file__))          # server/vision/
_SERVER_DIR = os.path.dirname(_VISION_DIR)                         # server/
_PLANNER_DIR = os.path.join(_SERVER_DIR, "planner")
_CONFIG_DIR  = os.path.join(_SERVER_DIR, "config")

for _p in (_CONFIG_DIR, _VISION_DIR, _PLANNER_DIR):
    if _p not in sys.path:
        sys.path.insert(0, _p)

if sys.stdout and hasattr(sys.stdout, 'reconfigure'):
    try:
        sys.stdout.reconfigure(encoding='utf-8', errors='replace')
    except Exception:
        pass

import constants
from cv_fiducial import CV_Fiducial
from cv_visualize import CVVisualizer
from planner_service import PlannerService

# ---------------------------------------------------------------------------
# Thư mục ảnh đầu vào / kết quả
# ---------------------------------------------------------------------------
_TEST_DIR    = os.path.join(_VISION_DIR, "test_vision_plan")
INPUT_DIR    = os.path.join(_TEST_DIR, "images")
OUTPUT_DIR   = os.path.join(_TEST_DIR, "results")
VALID_EXTS   = {".jpg", ".jpeg", ".png", ".bmp"}

# ---------------------------------------------------------------------------
# Màu sắc vẽ kết quả
# ---------------------------------------------------------------------------
PATH_COLORS = [
    (0, 220, 255),   # vàng-xanh — robot 0
    (255, 80, 200),  # hồng      — robot 1
    (80, 255, 160),  # xanh lá   — robot 2
    (255, 200, 0),   # vàng      — robot 3
    (100, 180, 255), # xanh nhạt — robot 4
]


# ---------------------------------------------------------------------------
# StaticImageCV — adapter ảnh tĩnh
# ---------------------------------------------------------------------------
class StaticImageCV:
    """Mô phỏng lớp CV nhưng sử dụng ảnh tĩnh để kiểm thử ngoại tuyến."""

    def __init__(self, image_path: str):
        self.image_path     = image_path
        self.cv_fiducial    = CV_Fiducial()
        self.latestSandboxImage = None
        self._raw_image     = None
        self.sandbox_warped = False

    def load(self):
        img = cv.imread(self.image_path)
        if img is None:
            raise FileNotFoundError(f"Không đọc được ảnh: {self.image_path}")
        self._raw_image = img
        print(f"  Loaded: {os.path.basename(self.image_path)}  "
              f"({img.shape[1]}x{img.shape[0]})")
        return self

    def run_pipeline(self):
        """Detect sandbox, warp, detect robot/pallet/goal."""
        # --- Detect sandbox corners ---
        detected = False
        try:
            detected = self.cv_fiducial._cv_fiducial_detectSandboxCorners(self._raw_image)
        except Exception as e:
            print(f"  [WARN] detectSandboxCorners lỗi: {e}")

        if not detected:
            print("  [WARN] Không tìm đủ 4 corner marker — dùng ảnh gốc.")
            self.latestSandboxImage = self._raw_image.copy()
        else:
            try:
                self.latestSandboxImage = self.cv_fiducial.cv_fiducial_flattenSandboxImage(
                    self._raw_image
                )
                self.sandbox_warped = True
                h, w = self.latestSandboxImage.shape[:2]
                print(f"  Sandbox warped: {w}x{h}")
            except Exception as e:
                print(f"  [WARN] flattenSandboxImage lỗi: {e}")
                self.latestSandboxImage = self._raw_image.copy()

        # --- Detect pallet / goal / robot ---
        try:
            self.cv_fiducial.cv_fiducial_generatePalletLocations(self.latestSandboxImage)
        except Exception as e:
            print(f"  [WARN] generatePalletLocations lỗi: {e}")

        robot_poses, robot_ids = self.cv_fiducial.cv_fiducial_getRobotPositions()
        pallet_poses           = self.cv_fiducial.cv_fiducial_getPalletPositions()
        goal_poses             = self.cv_fiducial.cv_fiducial_getGoalPositions()
        corner_poses           = self.cv_fiducial.cv_fiducial_getCornerPositions()

        print(f"  Detect: {len(robot_ids)} robot | "
              f"{len(pallet_poses)} pallet | "
              f"{len(goal_poses)} goal | "
              f"{len(corner_poses)} corner")
        if robot_ids:
            print(f"  Robot IDs: {robot_ids}")
        return self

    # --- Interface giống cv_main.py ---
    def cv_getLatestSandboxImage(self):
        return self.latestSandboxImage

    def cv_GetRobotPositions(self):
        return self.cv_fiducial.cv_fiducial_getRobotPositions()

    def cv_GetPalletPositions(self):
        return self.cv_fiducial.cv_fiducial_getPalletPositions()

    def cv_GetGoalPositions(self):
        return self.cv_fiducial.cv_fiducial_getGoalPositions()

    def cv_GetCornerPositions(self):
        return self.cv_fiducial.cv_fiducial_getCornerPositions()


# ---------------------------------------------------------------------------
# Planner helper
# ---------------------------------------------------------------------------
def run_planner(cv_obj: StaticImageCV, planner: PlannerService):
    """
    Lấy dữ liệu từ vision, gọi planner, trả về (paths, robot_ids).
    paths: list[list[waypoint]]  — mỗi phần tử là path của 1 robot (tọa độ mm).
    """
    robot_poses, robot_ids = cv_obj.cv_GetRobotPositions()
    pallet_poses           = cv_obj.cv_GetPalletPositions()
    goal_poses             = cv_obj.cv_GetGoalPositions()

    if not robot_ids:
        print("  [Planner] Không có robot — bỏ qua planning.")
        return [], []

    if not pallet_poses or not goal_poses:
        print("  [Planner] Không có pallet/goal — bỏ qua planning.")
        return [], robot_ids

    if len(pallet_poses) != len(goal_poses):
        n = min(len(pallet_poses), len(goal_poses))
        print(f"  [Planner] pallet({len(pallet_poses)}) != goal({len(goal_poses)})"
              f" — cắt còn {n}.")
        pallet_poses = pallet_poses[:n]
        goal_poses   = goal_poses[:n]

    # map_size có thêm margin an toàn để điểm đích sát viền không bị coi là va chạm tường
    all_x = [p[0] for p in robot_poses + pallet_poses + goal_poses] + [constants.CV_SANDBOX_WIDTH]
    all_y = [p[1] for p in robot_poses + pallet_poses + goal_poses] + [constants.CV_SANDBOX_HEIGHT]
    if cv_obj.latestSandboxImage is not None:
        h, w = cv_obj.latestSandboxImage.shape[:2]
        all_x.append(w)
        all_y.append(h)
    map_size = [int(max(all_x) + 200), int(max(all_y) + 200)]

    robot_poses_3d  = [[p[0], p[1], p[2]] for p in robot_poses]
    pallet_poses_3d = [[p[0], p[1], p[2]] for p in pallet_poses]
    goal_poses_3d   = [[p[0], p[1], p[2]] for p in goal_poses]

    try:
        paths = planner.plan_trajectories(
            map_size, robot_poses_3d, pallet_poses_3d, goal_poses_3d
        )
        planned = sum(1 for p in paths if p)
        print(f"  [Planner] Kế hoạch xong: {planned}/{len(robot_ids)} robot có path.")
        for path, fid in zip(paths, robot_ids):
            if path:
                print(f"    R{fid}: {len(path)} waypoints")
            else:
                print(f"    R{fid}: khong tim duoc duong")
        return paths, robot_ids
    except Exception as e:
        print(f"  [Planner] LỖI plan_trajectories: {e}")
        print(traceback.format_exc())
        return [], robot_ids


# ---------------------------------------------------------------------------
# Render — vẽ kết quả lên ảnh numpy
# ---------------------------------------------------------------------------
def render(cv_obj: StaticImageCV, paths: list, robot_ids: list) -> np.ndarray:
    """
    Vẽ toàn bộ thông tin vision + planned path lên frame và trả về ảnh kết quả.
    Tọa độ đầu vào ở đơn vị mm, scale=1 (1 px = 1 mm sau warp).
    """
    sandbox_image = cv_obj.cv_getLatestSandboxImage()
    if sandbox_image is None:
        raise RuntimeError("Không có sandbox image để render.")

    frame = sandbox_image.copy()
    scale = 1.0

    # --- Cảnh báo nếu chưa warp ---
    if not cv_obj.sandbox_warped:
        cv.putText(frame, "WARP SKIPPED: need all 4 corner markers",
                   (10, 50), cv.FONT_HERSHEY_SIMPLEX,
                   0.8, (0, 0, 255), 2, cv.LINE_AA)
        for fid, data in cv_obj.cv_fiducial.cv_fiducial_cornerMarkerDict.items():
            center = (int(data[0]), int(data[1]))
            cv.circle(frame, center, 8, (0, 0, 255), 2)
            cv.putText(frame, f"ID {fid}", (center[0] + 10, center[1] - 10),
                       cv.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2, cv.LINE_AA)

    # --- Corner markers ---
    for pose in cv_obj.cv_GetCornerPositions():
        cx_s, cy_s = int(pose[0] * scale), int(pose[1] * scale)
        cv.circle(frame, (cx_s, cy_s), 6, (0, 0, 255), -1)
        cv.circle(frame, (cx_s, cy_s), 9, (0, 0, 180), 2)
        cv.putText(frame, "C", (cx_s + 10, cy_s - 5),
                   cv.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1, cv.LINE_AA)

    # --- Pallet ---
    for i, pose in enumerate(cv_obj.cv_GetPalletPositions()):
        cx_s, cy_s = int(pose[0] * scale), int(pose[1] * scale)
        cv.circle(frame, (cx_s, cy_s), 7, (0, 128, 255), -1)
        cv.putText(frame, f"P{i}", (cx_s + 10, cy_s - 5),
                   cv.FONT_HERSHEY_SIMPLEX, 0.45, (0, 128, 255), 1, cv.LINE_AA)

    # --- Goal ---
    for i, pose in enumerate(cv_obj.cv_GetGoalPositions()):
        cx_s, cy_s = int(pose[0] * scale), int(pose[1] * scale)
        cv.drawMarker(frame, (cx_s, cy_s), (0, 220, 100),
                      cv.MARKER_CROSS, 14, 2, cv.LINE_AA)
        cv.putText(frame, f"G{i}", (cx_s + 10, cy_s - 5),
                   cv.FONT_HERSHEY_SIMPLEX, 0.45, (0, 220, 100), 1, cv.LINE_AA)

    # --- Robot vị trí + mũi tên hướng ---
    robot_positions, _ = cv_obj.cv_GetRobotPositions()
    for pose, fid in zip(robot_positions, robot_ids):
        cx, cy, orientation = pose[0], pose[1], pose[2]
        cx_s, cy_s = int(cx * scale), int(cy * scale)

        cv.circle(frame, (cx_s, cy_s), 5, (255, 60, 60), -1)
        arrow_len = 60 * scale
        end_x = int(cx_s + arrow_len * np.cos(orientation))
        end_y = int(cy_s - arrow_len * np.sin(orientation))
        cv.arrowedLine(frame, (cx_s, cy_s), (end_x, end_y),
                       (255, 60, 60), 2, tipLength=0.3)
        cv.putText(frame, f"R{fid}", (cx_s + 10, cy_s - 10),
                   cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 60, 60), 1, cv.LINE_AA)

    # --- Planned paths ---
    _draw_planned_paths(frame, paths, robot_ids, scale)

    # --- Summary HUD ---
    n_robots  = len(robot_ids)
    n_pallets = len(cv_obj.cv_GetPalletPositions())
    n_goals   = len(cv_obj.cv_GetGoalPositions())
    n_planned = sum(1 for p in paths if p)
    summary   = (f"Robots:{n_robots}  Pallets:{n_pallets}  "
                 f"Goals:{n_goals}  Planned:{n_planned}/{n_robots}")
    cv.putText(frame, summary, (10, 24),
               cv.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 200), 2, cv.LINE_AA)

    return frame


def _draw_planned_paths(frame: np.ndarray, paths: list,
                        robot_ids: list, scale: float):
    """Vẽ đường path A* với waypoint, mũi tên đích và label robot."""
    for i, (path, fid) in enumerate(zip(paths, robot_ids)):
        if not path or len(path) < 2:
            continue

        color = PATH_COLORS[i % len(PATH_COLORS)]
        pts   = [[int(wp[0] * scale), int(wp[1] * scale)] for wp in path]
        pts_np = np.array(pts, dtype=np.int32).reshape((-1, 1, 2))

        # Đường path mờ blend
        overlay = frame.copy()
        cv.polylines(overlay, [pts_np], isClosed=False,
                     color=color, thickness=2, lineType=cv.LINE_AA)
        cv.addWeighted(overlay, 0.65, frame, 0.35, 0, frame)

        # Chấm tròn tại waypoint
        for pt in pts:
            cv.circle(frame, tuple(pt), 3, color, -1)

        # Mũi tên tại waypoint cuối
        p1, p2 = tuple(pts[-2]), tuple(pts[-1])
        cv.arrowedLine(frame, p1, p2, color, 2, tipLength=0.4)

        # Label tại điểm đầu
        cv.putText(frame, f"P-R{fid}", (pts[0][0] + 6, pts[0][1] - 8),
                   cv.FONT_HERSHEY_SIMPLEX, 0.4, color, 1, cv.LINE_AA)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main():
    print("=" * 60)
    print("  Vision + Motion Planning -- Static Image Test")
    print("=" * 60)
    print(f"  Input dir : {INPUT_DIR}")
    print(f"  Output dir: {OUTPUT_DIR}")
    print()

    os.makedirs(OUTPUT_DIR, exist_ok=True)

    # --- Khởi tạo Planner ---
    planner = None
    try:
        planner = PlannerService(num_robots=5)
        if not planner.is_available:
            print("[WARN] planner_core chua build -- se bo qua buoc planning.\n"
                  "       Build: cmake -B build && cmake --build build --config Release\n"
                  "       (thuc hien trong server/planner/)\n")
            planner = None
    except Exception as e:
        print(f"[WARN] Khong khoi tao duoc PlannerService: {e}\n")
        planner = None

    # --- Tìm ảnh đầu vào ---
    image_paths = [
        p for p in glob.glob(os.path.join(INPUT_DIR, "*"))
        if os.path.splitext(p)[1].lower() in VALID_EXTS
    ]

    if not image_paths:
        print(f"[LOI] Khong tim thay anh trong: {INPUT_DIR}")
        print(f"       Dat anh *.jpg / *.png vao thu muc tren roi chay lai.")
        return

    print(f"Tim thay {len(image_paths)} anh. Bat dau xu ly...\n")

    pass_count = fail_count = 0

    for img_path in sorted(image_paths):
        print("-" * 50)
        print(f"[Anh] {os.path.basename(img_path)}")

        name, ext = os.path.splitext(os.path.basename(img_path))
        out_path  = os.path.join(OUTPUT_DIR, f"result_{name}{ext}")

        try:
            # 1. Vision
            cv_obj = StaticImageCV(img_path)
            cv_obj.load().run_pipeline()

            # 2. Planning
            paths, robot_ids = [], []
            if planner is not None:
                paths, robot_ids = run_planner(cv_obj, planner)
            else:
                _, robot_ids = cv_obj.cv_GetRobotPositions()
                print("  [Planner] Bo qua (planner_core chua san sang).")

            # 3. Render + lưu
            result_frame = render(cv_obj, paths, robot_ids)
            cv.imwrite(out_path, result_frame)
            print(f"  OK Luu ket qua --> {out_path}")
            pass_count += 1

        except Exception as e:
            print(f"  LOI: {e}")
            print(traceback.format_exc())
            fail_count += 1

    print("\n" + "=" * 60)
    print(f"  Hoan thanh: {pass_count} thanh cong, {fail_count} that bai")
    print(f"  Ket qua tai: {OUTPUT_DIR}")
    print("=" * 60)


if __name__ == "__main__":
    main()
