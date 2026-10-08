'''
Computer Vision Visualization Module.

Draws detection results on the sandbox image in real-time:
  - Red dots at ArUco marker centers (robots, pallets, goals, corners)
  - Blue orientation arrows for robots
  - Dashed trail lines showing robot movement history (each robot a unique color)

This module is READ-ONLY with respect to CV data — it only queries
the CV object for positions and draws overlays. Separating visualization
from detection makes the vision pipeline easier to maintain and extend
(e.g., adding planner path overlays later requires changes only here).
'''

import cv2 as cv
import numpy as np
from collections import deque
import constants


# Trail Colors (BGR) — MUST be distinct from robot arrow color (Blue)
TRAIL_COLORS = [
    (0, 140, 255),    # Orange
    (200, 0, 200),    # Magenta
    (0, 255, 128),    # Lime green
    (0, 215, 255),    # Gold
    (255, 144, 30),   # Dodger blue (lighter)
]

# Movement threshold in warped-image pixels; suppresses jitter noise
TRAIL_MOVEMENT_THRESHOLD = 5
# Maximum number of trail points stored per robot
TRAIL_MAX_LENGTH = 500
# Dashed line segment lengths (in display pixels after scaling)
DASH_LENGTH = 10
DASH_GAP = 6


class CVVisualizer:
    '''Draws detection overlays on the sandbox image and manages robot trails.'''

    def __init__(self):
        # Robot movement trails: {fiducial_id: deque([(x, y), ...])}
        self.robot_trails = {}
        # Assigned trail color per robot: {fiducial_id: (B, G, R)}
        self._trail_color_map = {}
        self._trail_color_index = 0

    #  Public API
    

    def visualize(self, cv_obj, window_name="PARROT Visualizer"):

        sandbox_image = cv_obj.cv_getLatestSandboxImage()
        if sandbox_image is None:
            return

        frame = sandbox_image.copy()
        scale = constants.CV_VIZ_SCALE

        # Resize for display
        if scale != 1.0:
            dsize = (int(frame.shape[1] * scale), int(frame.shape[0] * scale))
            frame = cv.resize(frame, dsize)

        # --- 1. Corner markers (red dots + "C" label) ---
        self._draw_corners(frame, cv_obj, scale)

        # --- 2. Pallet markers (red dots + "Pn" label) ---
        self._draw_pallets(frame, cv_obj, scale)

        # --- 3. Goal markers (red dots + "Gn" label) ---
        self._draw_goals(frame, cv_obj, scale)

        # --- 4. Robot markers (red dot + blue arrow + trail) ---
        self._draw_robots(frame, cv_obj, scale)

        cv.imshow(window_name, frame)

    def reset_trails(self):
        '''Clear all stored robot trails (e.g., after a replan).'''
        self.robot_trails.clear()

    #  Drawing helpers

    def _draw_corners(self, frame, cv_obj, scale):
        '''Draw red dots at detected sandbox corner markers.'''
        corner_positions = cv_obj.cv_GetCornerPositions()
        for pose in corner_positions:
            cx_s = int(pose[0] * scale)
            cy_s = int(pose[1] * scale)
            # Filled red dot
            cv.circle(frame, (cx_s, cy_s), 1, (0, 0, 255), -1)
            # Darker border ring
            cv.circle(frame, (cx_s, cy_s), 2, (0, 0, 180), 1)
            # Label
            cv.putText(frame, "C", (cx_s + 10, cy_s - 5),
                       cv.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1, cv.LINE_AA)

    def _draw_pallets(self, frame, cv_obj, scale):
        '''Draw red dots at detected pallet markers.'''
        pallet_positions = cv_obj.cv_GetPalletPositions()
        for i, pose in enumerate(pallet_positions):
            cx_s = int(pose[0] * scale)
            cy_s = int(pose[1] * scale)
            # Filled red dot
            cv.circle(frame, (cx_s, cy_s), 2, (0, 0, 255), -1)
            # Label
            cv.putText(frame, f"P{i}", (cx_s + 10, cy_s - 5),
                       cv.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1, cv.LINE_AA)

    def _draw_goals(self, frame, cv_obj, scale):
        '''Draw red dots at detected goal markers.'''
        goal_positions = cv_obj.cv_GetGoalPositions()
        for i, pose in enumerate(goal_positions):
            cx_s = int(pose[0] * scale)
            cy_s = int(pose[1] * scale)
            # Filled red dot
            cv.circle(frame, (cx_s, cy_s), 2, (0, 0, 255), -1)
            # Label — green text to distinguish from pallets
            cv.putText(frame, f"G{i}", (cx_s + 10, cy_s - 5),
                       cv.FONT_HERSHEY_SIMPLEX, 0.4, (0, 200, 0), 1, cv.LINE_AA)

    def _draw_robots(self, frame, cv_obj, scale):
        '''Draw red dot, blue orientation arrow, and dashed movement trail for each robot.'''
        robot_positions, robot_ids = cv_obj.cv_GetRobotPositions()

        for pose, fid in zip(robot_positions, robot_ids):
            cx, cy, orientation = pose[0], pose[1], pose[2]
            cx_s = int(cx * scale)
            cy_s = int(cy * scale)

            # --- Red dot at ArUco center ---
            cv.circle(frame, (cx_s, cy_s), 2, (0, 0, 255), -1)

            # --- Blue orientation arrow ---
            arrow_len = 60 * scale
            end_x = int(cx_s + arrow_len * np.cos(orientation))
            end_y = int(cy_s - arrow_len * np.sin(orientation))
            cv.arrowedLine(frame, (cx_s, cy_s), (end_x, end_y),
                           (255, 0, 0), 2, tipLength=0.3)  # Blue (BGR)

            # --- Label with fiducial ID ---
            cv.putText(frame, f"R{fid}", (cx_s + 10, cy_s - 10),
                       cv.FONT_HERSHEY_SIMPLEX, 0.45, (255, 0, 0), 1, cv.LINE_AA)

            # --- Update movement trail ---
            self._update_trail(fid, cx, cy)

            # --- Draw dashed trail ---
            trail_color = self._get_trail_color(fid)
            self._draw_dashed_trail(frame, fid, scale, trail_color)

    # =================================================================
    #  Trail management
    # =================================================================

    def _update_trail(self, fiducial_id, x, y):
        '''Append position to a robot's trail if it has moved enough (anti-jitter).'''
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

    def _get_trail_color(self, fiducial_id):
        '''Assign a unique, stable trail color to each robot.'''
        if fiducial_id not in self._trail_color_map:
            idx = self._trail_color_index % len(TRAIL_COLORS)
            self._trail_color_map[fiducial_id] = TRAIL_COLORS[idx]
            self._trail_color_index += 1
        return self._trail_color_map[fiducial_id]

    # =================================================================
    #  Dashed line drawing
    # =================================================================

    def _draw_dashed_trail(self, img, fiducial_id, scale, color, thickness=2):
        '''Draw a continuous dashed polyline along the robot's trail.'''
        trail = self.robot_trails.get(fiducial_id)
        if trail is None or len(trail) < 2:
            return

        # Convert trail to scaled pixel coordinates
        points = [(int(p[0] * scale), int(p[1] * scale)) for p in trail]

        # Walk along the polyline, accumulating distance, alternating dash/gap
        acc = 0.0          # accumulated distance within current phase
        drawing = True     # True = dash phase, False = gap phase
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

                # Switch phase when current one is exhausted
                if acc >= phase_limit:
                    drawing = not drawing
                    phase_limit = DASH_LENGTH if drawing else DASH_GAP
                    acc = 0.0
