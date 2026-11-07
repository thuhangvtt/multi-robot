import cv2 as cv
import numpy as np
import traceback
from collections import deque
import constants

TRAIL_COLORS = [
    (0, 140, 255),
    (200, 0, 200),
    (0, 255, 128),
    (0, 215, 255),
    (255, 144, 30),
]

# Màu đường path hoạch định (sáng hơn trail)
PATH_COLORS = [
    (0, 220, 255),    # vàng-xanh
    (255, 80, 200),   # hồng
    (80, 255, 160),   # xanh lá
    (255, 200, 0),    # vàng
    (100, 180, 255),  # xanh dương nhạt
]

TRAIL_MOVEMENT_THRESHOLD = 5
TRAIL_MAX_LENGTH = 500
DASH_LENGTH = 10
DASH_GAP = 6


class CVVisualizer:
    """Vẽ thông tin định vị và quỹ đạo robot lên khung hình sandbox."""

    def __init__(self):
        self.robot_trails = {}
        self._trail_color_map = {}
        self._trail_color_index = 0
        self._planned_paths = []   # [[waypoint, ...], ...] mỗi phần tử là path của 1 robot
        self._planned_fid_ids = [] # fiducial ID tương ứng

    def set_paths(self, paths, fid_ids):
        """Cập nhật đường path hoạch định để hiển thị. paths: list path mỗi robot (pixel mm), fid_ids: danh sách fiducial ID."""
        self._planned_paths = paths if paths else []
        self._planned_fid_ids = fid_ids if fid_ids else []

    def visualize(self, cv_obj, window_name="PARROT Visualizer"):
        """Hiển thị hình ảnh sandbox với các đánh dấu robot, pallet và goal."""
        try:
            sandbox_image = cv_obj.cv_getLatestSandboxImage()
            if sandbox_image is None:
                return

            frame = sandbox_image.copy()
            scale = constants.CV_VIZ_SCALE

            if scale != 1.0:
                dsize = (int(frame.shape[1] * scale), int(frame.shape[0] * scale))
                frame = cv.resize(frame, dsize)

            self._draw_corners(frame, cv_obj, scale)
            self._draw_planned_paths(frame, scale)
            self._draw_pallets(frame, cv_obj, scale)
            self._draw_goals(frame, cv_obj, scale)
            self._draw_robots(frame, cv_obj, scale)

            cv.imshow(window_name, frame)
        except Exception as e:
            print(f"[CVVisualizer] LỖI: visualize: {e}")
            print(traceback.format_exc())

    def reset_trails(self):
        """Xóa toàn bộ đường vết di chuyển của robot."""
        self.robot_trails.clear()

    def _draw_planned_paths(self, frame, scale):
        """Vẽ đường path hoạch định (A* waypoints) lên frame bằng polylines."""
        try:
            for i, path in enumerate(self._planned_paths):
                if not path or len(path) < 2:
                    continue

                color = PATH_COLORS[i % len(PATH_COLORS)]
                fid = self._planned_fid_ids[i] if i < len(self._planned_fid_ids) else i

                # Tạo danh sách điểm pixel
                pts = []
                buffer_x = constants.CV_SANDBOX_WIDTH * constants.CV_SANDBOX_IMAGE_BUFFER_PERCENT / 2
                buffer_y = constants.CV_SANDBOX_HEIGHT * constants.CV_SANDBOX_IMAGE_BUFFER_PERCENT / 2
                for wp in path:
                    px = int((wp[0] + buffer_x) * scale)
                    py = int((wp[1] + buffer_y) * scale)
                    pts.append([px, py])

                pts_np = np.array(pts, dtype=np.int32).reshape((-1, 1, 2))

                # Vẽ đường path (trong suốt hơn bằng cách blend)
                overlay = frame.copy()
                cv.polylines(overlay, [pts_np], isClosed=False, color=color, thickness=2, lineType=cv.LINE_AA)
                cv.addWeighted(overlay, 0.6, frame, 0.4, 0, frame)

                # Vẽ chấm tròn tại mỗi waypoint
                for pt in pts:
                    cv.circle(frame, tuple(pt), 3, color, -1)

                # Vẽ mũi tên ở waypoint cuối
                if len(pts) >= 2:
                    p1 = tuple(pts[-2])
                    p2 = tuple(pts[-1])
                    cv.arrowedLine(frame, p1, p2, color, 2, tipLength=0.4)

                # Label robot tại điểm đầu path
                if pts:
                    cv.putText(frame, f"P-R{fid}", (pts[0][0] + 5, pts[0][1] - 8),
                               cv.FONT_HERSHEY_SIMPLEX, 0.4, color, 1, cv.LINE_AA)
        except Exception as e:
            print(f"[CVVisualizer] LỖI: _draw_planned_paths: {e}")
            print(traceback.format_exc())

    def _draw_corners(self, frame, cv_obj, scale):
        """Vẽ điểm đánh dấu các góc sandbox."""
        try:
            corner_positions = cv_obj.cv_GetCornerPositions()
            for pose in corner_positions:
                cx_s = int(pose[0] * scale)
                cy_s = int(pose[1] * scale)
                cv.circle(frame, (cx_s, cy_s), 1, (0, 0, 255), -1)
                cv.circle(frame, (cx_s, cy_s), 2, (0, 0, 180), 1)
                cv.putText(frame, "C", (cx_s + 10, cy_s - 5),
                           cv.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1, cv.LINE_AA)
        except Exception as e:
            print(f"[CVVisualizer] LỖI: _draw_corners: {e}")
            print(traceback.format_exc())

    def _draw_pallets(self, frame, cv_obj, scale):
        """Vẽ điểm đánh dấu pallet."""
        try:
            pallet_positions = cv_obj.cv_GetPalletPositions()
            for i, pose in enumerate(pallet_positions):
                cx_s = int(pose[0] * scale)
                cy_s = int(pose[1] * scale)
                cv.circle(frame, (cx_s, cy_s), 2, (0, 0, 255), -1)
                cv.putText(frame, f"P{i}", (cx_s + 10, cy_s - 5),
                           cv.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1, cv.LINE_AA)
        except Exception as e:
            print(f"[CVVisualizer] LỖI: _draw_pallets: {e}")
            print(traceback.format_exc())

    def _draw_goals(self, frame, cv_obj, scale):
        """Vẽ điểm đánh dấu đích đến."""
        try:
            goal_positions = cv_obj.cv_GetGoalPositions()
            for i, pose in enumerate(goal_positions):
                cx_s = int(pose[0] * scale)
                cy_s = int(pose[1] * scale)
                cv.circle(frame, (cx_s, cy_s), 2, (0, 0, 255), -1)
                cv.putText(frame, f"G{i}", (cx_s + 10, cy_s - 5),
                           cv.FONT_HERSHEY_SIMPLEX, 0.4, (0, 200, 0), 1, cv.LINE_AA)
        except Exception as e:
            print(f"[CVVisualizer] LỖI: _draw_goals: {e}")
            print(traceback.format_exc())

    def _draw_robots(self, frame, cv_obj, scale):
        """Vẽ vị trí, hướng và vệt di chuyển nét đứt của robot."""
        try:
            robot_positions, robot_ids = cv_obj.cv_GetRobotPositions()

            for pose, fid in zip(robot_positions, robot_ids):
                cx, cy = pose[0], pose[1]
                orientation = pose[2] + constants.CONTROLS_ROBOT_HEADING_OFFSET_RAD
                cx_s = int(cx * scale)
                cy_s = int(cy * scale)

                cv.circle(frame, (cx_s, cy_s), 2, (0, 0, 255), -1)

                arrow_len = 60 * scale
                end_x = int(cx_s + arrow_len * np.cos(orientation))
                end_y = int(cy_s - arrow_len * np.sin(orientation))
                cv.arrowedLine(frame, (cx_s, cy_s), (end_x, end_y),
                               (255, 0, 0), 2, tipLength=0.3)

                cv.putText(frame, f"R{fid}", (cx_s + 10, cy_s - 10),
                           cv.FONT_HERSHEY_SIMPLEX, 0.45, (255, 0, 0), 1, cv.LINE_AA)

                self._update_trail(fid, cx, cy)

                trail_color = self._get_trail_color(fid)
                self._draw_dashed_trail(frame, fid, scale, trail_color)
        except Exception as e:
            print(f"[CVVisualizer] LỖI: _draw_robots: {e}")
            print(traceback.format_exc())

    def _update_trail(self, fiducial_id, x, y):
        """Cập nhật tọa độ vào lịch sử vệt di chuyển khi robot di chuyển qua ngưỡng tối thiểu."""
        try:
            if fiducial_id not in self.robot_trails:
                self.robot_trails[fiducial_id] = deque(maxlen=TRAIL_MAX_LENGTH)

            trail = self.robot_trails[fiducial_id]
            if len(trail) == 0:
                trail.append((x, y))
                return

            last_x, last_y = trail[-1]
            dist = np.hypot(x - last_x, y - last_y)
            if dist > TRAIL_MOVEMENT_THRESHOLD:
                trail.append((x, y))
        except Exception as e:
            print(f"[CVVisualizer] LỖI: _update_trail fid={fiducial_id}: {e}")
            print(traceback.format_exc())

    def _get_trail_color(self, fiducial_id):
        """Lấy màu vệt hiển thị cố định cho từng robot."""
        try:
            if fiducial_id not in self._trail_color_map:
                idx = self._trail_color_index % len(TRAIL_COLORS)
                self._trail_color_map[fiducial_id] = TRAIL_COLORS[idx]
                self._trail_color_index += 1
            return self._trail_color_map[fiducial_id]
        except Exception as e:
            print(f"[CVVisualizer] LỖI: _get_trail_color fid={fiducial_id}: {e}")
            print(traceback.format_exc())
            return (0, 255, 0)

    def _draw_dashed_trail(self, img, fiducial_id, scale, color, thickness=2):
        """Vẽ đường nét đứt biểu diễn vệt di chuyển của robot."""
        try:
            trail = self.robot_trails.get(fiducial_id)
            if trail is None or len(trail) < 2:
                return

            points = [(int(p[0] * scale), int(p[1] * scale)) for p in trail]

            acc = 0.0
            drawing = True
            phase_limit = DASH_LENGTH

            for i in range(1, len(points)):
                x0, y0 = points[i - 1]
                x1, y1 = points[i]
                seg_len = np.hypot(x1 - x0, y1 - y0)
                if seg_len < 0.5:
                    continue

                dx = (x1 - x0) / seg_len
                dy = (y1 - y0) / seg_len
                d = 0.0

                while d < seg_len:
                    remaining_phase = phase_limit - acc
                    remaining_seg = seg_len - d
                    advance = min(remaining_phase, remaining_seg)

                    if drawing:
                        sp = (int(x0 + dx * d), int(y0 + dy * d))
                        ep = (int(x0 + dx * (d + advance)), int(y0 + dy * (d + advance)))
                        cv.line(img, sp, ep, color, thickness, cv.LINE_AA)

                    d += advance
                    acc += advance

                    if acc >= phase_limit:
                        drawing = not drawing
                        phase_limit = DASH_LENGTH if drawing else DASH_GAP
                        acc = 0.0
        except Exception as e:
            print(f"[CVVisualizer] LỖI: _draw_dashed_trail fid={fiducial_id}: {e}")
            print(traceback.format_exc())
