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

PATH_COLORS = [
    (0, 220, 255),
    (255, 80, 200),
    (80, 255, 160),
    (255, 200, 0),
    (100, 180, 255),
]

TRAIL_MOVEMENT_THRESHOLD = 5
TRAIL_MAX_LENGTH = 500
DASH_LENGTH = 10
DASH_GAP = 6

class CVVisualizer:

    def __init__(self):
        self.robot_trails = {}
        self._trail_color_map = {}
        self._trail_color_index = 0
        self._planned_paths = []
        self._planned_fid_ids = []
        self._wheel_speeds = {}

    def set_wheel_speeds(self, speeds):
        if isinstance(speeds, dict):
            self._wheel_speeds = speeds

    def set_paths(self, paths, fid_ids):
        self._planned_paths = paths if paths else []
        self._planned_fid_ids = fid_ids if fid_ids else []

    def visualize(self, cv_obj, window_name="PARROT Visualizer"):

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

        self.robot_trails.clear()

    def _draw_planned_paths(self, frame, scale):
        try:
            for i, path in enumerate(self._planned_paths):
                if not path or len(path) < 2:
                    continue

                color = PATH_COLORS[i % len(PATH_COLORS)]
                fid = self._planned_fid_ids[i] if i < len(self._planned_fid_ids) else i

                buffer_x = constants.CV_SANDBOX_WIDTH * constants.CV_SANDBOX_IMAGE_BUFFER_PERCENT / 2
                buffer_y = constants.CV_SANDBOX_HEIGHT * constants.CV_SANDBOX_IMAGE_BUFFER_PERCENT / 2
                from controller_main import trajectory_reference

                pts = []
                samples = []
                for segment in range(len(path) - 1):
                    for u in np.linspace(0.0, 1.0, 12, endpoint=False):
                        t = path[segment][3] + u * (path[segment+1][3] - path[segment][3])
                        samples.append(trajectory_reference(path, t, segment, segment+1)[1])
                samples.append(path[-1])
                for wp in samples:
                    px = int((wp[0] + buffer_x) * scale)
                    py = int((wp[1] + buffer_y) * scale)
                    pts.append([px, py])

                pts_np = np.array(pts, dtype=np.int32).reshape((-1, 1, 2))

                overlay = frame.copy()
                cv.polylines(overlay, [pts_np], isClosed=False, color=color, thickness=2, lineType=cv.LINE_AA)
                cv.addWeighted(overlay, 0.75, frame, 0.25, 0, frame)

                for wp in path:
                    wx = int((wp[0] + buffer_x) * scale)
                    wy = int((wp[1] + buffer_y) * scale)
                    w_th = wp[2]
                    arr_l = 18 * scale
                    w_end_x = int(wx + arr_l * np.cos(w_th))
                    w_end_y = int(wy - arr_l * np.sin(w_th))
                    cv.arrowedLine(frame, (wx, wy), (w_end_x, w_end_y), color, 1, tipLength=0.35)
                    cv.circle(frame, (wx, wy), 2, color, -1)

                if pts:
                    cv.putText(frame, f"P-R{fid}", (pts[0][0] + 5, pts[0][1] - 8),
                               cv.FONT_HERSHEY_SIMPLEX, 0.4, color, 1, cv.LINE_AA)
        except Exception as e:
            print(f"[CVVisualizer] LỖI: _draw_planned_paths: {e}")
            print(traceback.format_exc())

    def _draw_corners(self, frame, cv_obj, scale):

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

        try:
            pallet_positions = cv_obj.cv_GetPalletPositions()
            for i, pose in enumerate(pallet_positions):
                cx_s = int(pose[0] * scale)
                cy_s = int(pose[1] * scale)
                cv.circle(frame, (cx_s, cy_s), 3, (0, 0, 255), -1)

                if len(pose) > 2 and pose[2] is not None:
                    p_th = (pose[2] + getattr(constants, 'CV_PALLET_APPROACH_ANGLE_OFFSET', 0.0)) % (2 * np.pi)
                    offset = getattr(constants, 'CV_PALLET_OFFSET', 0)
                    if offset > 0:
                        dock_x = int((pose[0] - offset * np.cos(p_th)) * scale)
                        dock_y = int((pose[1] + offset * np.sin(p_th)) * scale)

                        cv.circle(frame, (dock_x, dock_y), 4, (0, 255, 255), 1)

                        cv.arrowedLine(frame, (dock_x, dock_y), (cx_s, cy_s),
                                       (0, 165, 255), 2, tipLength=0.3)
                    else:
                        arr_len = 35 * scale
                        p_end_x = int(cx_s + arr_len * np.cos(p_th))
                        p_end_y = int(cy_s - arr_len * np.sin(p_th))
                        cv.arrowedLine(frame, (cx_s, cy_s), (p_end_x, p_end_y),
                                       (0, 165, 255), 2, tipLength=0.3)

                cv.putText(frame, f"P{i}", (cx_s + 10, cy_s - 5),
                           cv.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1, cv.LINE_AA)
        except Exception as e:
            print(f"[CVVisualizer] LỖI: _draw_pallets: {e}")
            print(traceback.format_exc())

    def _draw_goals(self, frame, cv_obj, scale):

        try:
            goal_positions = cv_obj.cv_GetGoalPositions()
            for i, pose in enumerate(goal_positions):
                cx_s = int(pose[0] * scale)
                cy_s = int(pose[1] * scale)
                cv.circle(frame, (cx_s, cy_s), 3, (0, 0, 255), -1)
                cv.putText(frame, f"G{i}", (cx_s + 10, cy_s - 5),
                           cv.FONT_HERSHEY_SIMPLEX, 0.4, (0, 200, 0), 1, cv.LINE_AA)
        except Exception as e:
            print(f"[CVVisualizer] LỖI: _draw_goals: {e}")
            print(traceback.format_exc())

    def _draw_robots(self, frame, cv_obj, scale):

        try:
            robot_positions, robot_ids = cv_obj.cv_GetRobotPositions()

            for pose, fid in zip(robot_positions, robot_ids):
                cx, cy = pose[0], pose[1]
                orientation = pose[2] + constants.CONTROLS_ROBOT_HEADING_OFFSET_RAD
                cx_s = int(cx * scale)
                cy_s = int(cy * scale)

                cv.circle(frame, (cx_s, cy_s), 3, (0, 0, 255), -1)

                blue_len = 65 * scale
                blue_end_x = int(cx_s + blue_len * np.cos(orientation))
                blue_end_y = int(cy_s - blue_len * np.sin(orientation))
                cv.arrowedLine(frame, (cx_s, cy_s), (blue_end_x, blue_end_y),
                               (255, 0, 0), 2, tipLength=0.3)

                v_left, v_right = self._wheel_speeds.get(fid, (0.0, 0.0))
                if abs(v_left) > 1e-3 or abs(v_right) > 1e-3:
                    magnitude = (abs(v_right) + abs(v_left)) * 15 * scale * scale
                    steer_angle = ((v_right - v_left) / max(constants.CONTROLS_CALIBRATED_SPEED_MM_S, 1.0)) * (np.pi / 2.0)
                    cmd_angle = orientation + steer_angle
                    white_len = max(magnitude, 25 * scale)
                    white_end_x = int(cx_s + white_len * np.cos(cmd_angle))
                    white_end_y = int(cy_s - white_len * np.sin(cmd_angle))
                    cv.arrowedLine(frame, (cx_s, cy_s), (white_end_x, white_end_y),
                                   (255, 255, 255), 2, tipLength=0.35)
                else:

                    white_len = 25 * scale
                    white_end_x = int(cx_s + white_len * np.cos(orientation))
                    white_end_y = int(cy_s - white_len * np.sin(orientation))
                    cv.arrowedLine(frame, (cx_s, cy_s), (white_end_x, white_end_y),
                                   (255, 255, 255), 1, tipLength=0.35)

                cv.putText(frame, f"R{fid}", (cx_s + 10, cy_s - 10),
                           cv.FONT_HERSHEY_SIMPLEX, 0.45, (255, 140, 0), 1, cv.LINE_AA)

                self._update_trail(fid, cx, cy)

                trail_color = self._get_trail_color(fid)
                self._draw_dashed_trail(frame, fid, scale, trail_color)
        except Exception as e:
            print(f"[CVVisualizer] LỖI: _draw_robots: {e}")
            print(traceback.format_exc())

    def _update_trail(self, fiducial_id, x, y):

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
