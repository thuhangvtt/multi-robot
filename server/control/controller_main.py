from collections import deque
import math
import time
import sys
import os
import traceback

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'config'))
import constants

PWM_NEUTRAL   = 90
PWM_MAX_DELTA = constants.CONTROLS_MAX_PWM_OFFSET
PWM_MAX       = constants.CONTROLS_MAX_PWM
PWM_MIN       = constants.CONTROLS_MIN_PWM

WAYPOINT_XY_THRESH_MM     = 40.0
WAYPOINT_THETA_THRESH     = 0.4
PALLET_XY_THRESH_MM       = 45.0
PALLET_PRE_DOCK_DIST_MM   = 45.0
PALLET_INSERT_DONE_MM     = -30.0
PALLET_ALIGN_THETA_THRESH = 0.065
ALIGN_SETTLE_TIME_S       = 0.20
ALIGN_MAX_TIME_S          = 4.0
INSERT_MAX_TIME_S         = 3.5
PALLET_ENTRY_HEADING_MAX  = 0.50
PALLET_LIFT_ALIGN_MAX     = 0.15
PALLET_LIFT_LATERAL_MAX_MM = 22.0
PALLET_OVERSHOOT_DIST_MM  = 85.0
PALLET_OVERSHOOT_ALPHA    = 1.2
BACKOFF_SPEED_MM_S        = 42.0
BACKOFF_CLEAR_DIST_MM     = 105.0
BACKOFF_MAX_TIME_S        = 2.2
MAX_DOCK_RETRIES          = 2
NEAR_PALLET_DIST_MM       = 120.0
NEAR_PALLET_MAX_SPEED_MM_S = 44.0
IN_PLACE_RECOVERY_ALPHA   = 1.20
LIFT_HOLD_TIME_S          = constants.CONTROLS_PALLET_ACTION_TIME_THRESHOLD

STATE_IDLE              = "IDLE"
STATE_DRIVING_TO_PALLET = "DRIVING_TO_PALLET"
STATE_ALIGNING_PALLET   = "ALIGNING_PALLET"
STATE_INSERTING_PALLET  = "INSERTING_PALLET"
STATE_BACKING_OFF       = "BACKING_OFF"
STATE_FAILED            = "FAILED"
STATE_WAITING_LIFT_UP   = "WAITING_LIFT_UP"
STATE_DRIVING_TO_GOAL   = "DRIVING_TO_GOAL"
STATE_WAITING_LIFT_DOWN = "WAITING_LIFT_DOWN"
STATE_DONE              = "DONE"

TAG_DRIVE  = 0
TAG_PALLET = 1
TAG_GOAL   = 2


class RobotController:
    """Outer-loop Pure Pursuit + curvature speed profiler outputting (v_mm_s, w_deg_s, action)."""

    def __init__(self, hw_id, waypoints, is_pickup_mode=False):
        self.hw_id     = hw_id
        self.waypoints = waypoints
        self.wp_idx    = 0
        self.state     = STATE_IDLE if waypoints else STATE_DONE
        self._lift_time = None
        self._stage_enter_time = None
        self._last_target = None
        self._trajectory_start = None
        self._last_tick_time = None
        self._paused_marker = False
        self._stage_curve = []
        self._stage_curve_key = None
        self._curve_progress_idx = 0
        self._prev_steering_err = 0.0
        self._pickup_gains = is_pickup_mode
        self._dock_attempts = 0
        self.pallet_fid_id = None
        self.last_latency_ms = 0.0
        self.last_wheel_velocities = (0.0, 0.0)
        self.last_velocity_cmd = (0.0, 0.0)
        self._pose_history = deque(maxlen=30)
        self._v_boost = 0.0
        self._last_boost_t = None

        self.pallet_wp_idx = None
        for idx, wp in enumerate(self.waypoints):
            if int(wp[4]) == TAG_PALLET:
                self.pallet_wp_idx = idx

    @property
    def is_done(self):
        return self.state in (STATE_DONE, STATE_FAILED)

    @property
    def current_waypoint(self):
        if self._last_target is not None:
            return self._last_target
        if self.wp_idx < len(self.waypoints):
            return self.waypoints[self.wp_idx]
        return None

    def reset_tracking(self):
        self._prev_steering_err = 0.0
        self._pose_history.clear()
        self._v_boost = 0.0
        self._last_boost_t = None

    def _estimate_real_speed(self, now, rx, ry):
        self._pose_history.append((now, rx, ry))
        for t0, x0, y0 in reversed(self._pose_history):
            dt = now - t0
            if dt >= 0.20:
                return math.hypot(rx - x0, ry - y0) / dt
        return None

    def _apply_speed_feedback(self, v_target, now, rx, ry):
        if abs(v_target) < 15.0:
            self._v_boost = 0.0
            self._last_boost_t = now
            return v_target

        dt = 0.0 if self._last_boost_t is None else _clamp(now - self._last_boost_t, 0.0, 0.15)
        self._last_boost_t = now
        v_real = self._estimate_real_speed(now, rx, ry)
        if v_real is None:
            return v_target

        target_mag = abs(v_target)
        if v_real < 0.45 * target_mag:
            self._v_boost += 38.0 * dt
        elif v_real < 0.85 * target_mag:
            self._v_boost += 15.0 * dt
        elif v_real > 1.05 * target_mag:
            self._v_boost -= 120.0 * dt

        self._v_boost = _clamp(self._v_boost, -15.0, 65.0)
        v_cmd = _clamp(target_mag + self._v_boost, 20.0, target_mag + 65.0)
        return math.copysign(v_cmd, v_target)

    def reset_pids(self):
        self.reset_tracking()

    def _make_cmd(self, linear_mm_s, angular_rad_s, pallet_action=constants.PALLET_ACTION_DONT_SEND):
        angular_rad_s *= constants.CONTROLS_ANGULAR_COMMAND_SIGN
        half_base = constants.CONTROLS_WHEEL_BASE_MM / 2.0
        v_left = linear_mm_s - half_base * angular_rad_s
        v_right = linear_mm_s + half_base * angular_rad_s
        self.last_wheel_velocities = (v_left, v_right)
        w_deg_s = math.degrees(angular_rad_s)
        self.last_velocity_cmd = (float(linear_mm_s), float(w_deg_s))
        return (float(linear_mm_s), float(w_deg_s), pallet_action)

    def _begin_backoff(self, now, reason):
        self._dock_attempts += 1
        self.reset_tracking()
        if self._dock_attempts > MAX_DOCK_RETRIES:
            print(f"[Controller] R{self.hw_id} DỪNG: gắp pallet thất bại sau {MAX_DOCK_RETRIES} lần thử ({reason}).")
            self.state = STATE_FAILED
            return self._make_cmd(0.0, 0.0)
        print(f"[Controller] R{self.hw_id} lùi ra thử lại lần {self._dock_attempts}/{MAX_DOCK_RETRIES}: {reason}")
        self.state = STATE_BACKING_OFF
        self._stage_enter_time = now
        return self._make_cmd(0.0, 0.0)

    def _finish_backoff(self, rx, ry):
        n = len(self._stage_curve)
        start = max(0, n // 2 - 1)
        best_i, best_d = start, float('inf')
        for i in range(start, n):
            d = math.hypot(self._stage_curve[i][0] - rx, self._stage_curve[i][1] - ry)
            if d < best_d:
                best_d, best_i = d, i
        self._curve_progress_idx = best_i
        self.reset_tracking()
        self.state = STATE_DRIVING_TO_PALLET

    def _pallet_aim_error(self, rx, ry, rtheta, pallet_wp):
        offset = getattr(constants, 'CV_PALLET_OFFSET', 95.0)
        px = pallet_wp[0] + offset * math.cos(pallet_wp[2])
        py = pallet_wp[1] - offset * math.sin(pallet_wp[2])
        aim_theta = _interpolate_angle(pallet_wp[2], math.atan2(-(py - ry), px - rx), 0.25)
        align_err = _normalize_angle(aim_theta - rtheta)
        rem_forward = (pallet_wp[0] - rx) * math.cos(pallet_wp[2]) - (pallet_wp[1] - ry) * math.sin(pallet_wp[2])
        pallet_dist = math.hypot(pallet_wp[0] - rx, pallet_wp[1] - ry)
        return align_err, rem_forward, pallet_dist

    def _tick_trajectory(self, robot_pose, now):
        if self._trajectory_start is None:
            self._trajectory_start = now

        if self.state == STATE_DRIVING_TO_PALLET and self.pallet_wp_idx is not None:
            stage_start_idx = 0
            stage_end_idx = self.pallet_wp_idx
        else:
            stage_start_idx = self.pallet_wp_idx if self.pallet_wp_idx is not None else 0
            stage_end_idx = len(self.waypoints) - 1

        stage_key = (stage_start_idx, stage_end_idx)
        if self._stage_curve_key != stage_key:
            self._stage_curve = _build_stage_curve(self.waypoints, stage_start_idx, stage_end_idx)
            self._stage_curve_key = stage_key
            self._curve_progress_idx = 0

        rx, ry, rtheta = robot_pose
        win_end = min(len(self._stage_curve), self._curve_progress_idx + 40)
        closest_idx = self._curve_progress_idx
        best_dist = float('inf')
        for i in range(self._curve_progress_idx, win_end):
            sx, sy = self._stage_curve[i][0], self._stage_curve[i][1]
            d = math.hypot(sx - rx, sy - ry)
            if d < best_dist:
                best_dist = d
                closest_idx = i
        self._curve_progress_idx = closest_idx
        self.wp_idx = self._stage_curve[closest_idx][3]

        half_curve_idx = max(1, len(self._stage_curve) // 2) if len(self._stage_curve) >= 2 else 0
        if self.state == STATE_DRIVING_TO_PALLET and self.pallet_wp_idx is not None:
            pallet_wp = self.waypoints[self.pallet_wp_idx]
            pallet_dist = math.hypot(pallet_wp[0] - rx, pallet_wp[1] - ry)
            if pallet_dist <= PALLET_PRE_DOCK_DIST_MM and self._curve_progress_idx >= half_curve_idx:
                heading_err = _normalize_angle(pallet_wp[2] - rtheta)
                if abs(heading_err) <= PALLET_ENTRY_HEADING_MAX:
                    self.state = STATE_INSERTING_PALLET
                    self._stage_enter_time = now
                    self.reset_tracking()
                    self._last_target = [
                        pallet_wp[0] + abs(PALLET_INSERT_DONE_MM) * math.cos(pallet_wp[2]),
                        pallet_wp[1] - abs(PALLET_INSERT_DONE_MM) * math.sin(pallet_wp[2]),
                        pallet_wp[2],
                        pallet_wp[3],
                        pallet_wp[4],
                    ]
                    return self._make_cmd(44.0, 0.0)
                return self._begin_backoff(now, f"vào pallet lệch hướng {math.degrees(heading_err):.0f}°")
        else:
            goal_wp = self.waypoints[stage_end_idx]
            goal_dist = math.hypot(goal_wp[0] - rx, goal_wp[1] - ry)
            if goal_dist <= WAYPOINT_XY_THRESH_MM and self._curve_progress_idx >= half_curve_idx:
                self.state = STATE_WAITING_LIFT_DOWN
                self._lift_time = now
                self.reset_tracking()
                return self._make_cmd(0.0, 0.0, constants.PALLET_ACTION_LIFT_DOWN)

        base_lookahead = getattr(constants, 'CONTROLS_LOOKAHEAD_DIST_MM', 55.0)
        if self.state == STATE_DRIVING_TO_PALLET and self.pallet_wp_idx is not None:
            lookahead_dist = _clamp(0.5 * pallet_dist, 34.0, base_lookahead) if pallet_dist < 140.0 else base_lookahead
        else:
            lookahead_dist = base_lookahead

        s_now = self._stage_curve[closest_idx][4]
        look_idx = len(self._stage_curve) - 1
        brake_idx = len(self._stage_curve) - 1
        curv_idx = len(self._stage_curve) - 1
        found_look = False
        found_brake = False
        found_curv = False
        for i in range(closest_idx, len(self._stage_curve)):
            s_i = self._stage_curve[i][4]
            if not found_look and s_i >= s_now + lookahead_dist:
                look_idx = i
                found_look = True
            if not found_brake and s_i >= s_now + 50.0:
                brake_idx = i
                found_brake = True
            if not found_curv and s_i >= s_now + 85.0:
                curv_idx = i
                found_curv = True
            if found_look and found_brake and found_curv:
                break

        lx, ly, ltheta, _, _, _ = self._stage_curve[look_idx]
        v_prof_cap = min(self._stage_curve[i][5] for i in range(closest_idx, brake_idx + 1))
        end_wp = self.waypoints[stage_end_idx]
        self._last_target = [lx, ly, ltheta, end_wp[3], end_wp[4]]

        dx = lx - rx
        dy_cart = -(ly - ry)
        dist_to_look = max(math.hypot(dx, dy_cart), 1.0)
        alpha = _normalize_angle(math.atan2(dy_cart, dx) - rtheta)
        tangent_err = _normalize_angle(ltheta - rtheta)
        if self.state == STATE_DRIVING_TO_PALLET and dist_to_look < 50.0:
            w_pursuit = _clamp((dist_to_look - 25.0) / 35.0, 0.1, 0.85)
        else:
            w_pursuit = _clamp((dist_to_look - 15.0) / 30.0, 0.2, 0.85)
        steering_err = _interpolate_angle(tangent_err, alpha, w_pursuit)
        d_steering = _normalize_angle(steering_err - self._prev_steering_err)
        self._prev_steering_err = steering_err

        near_pallet = (
            self.state == STATE_DRIVING_TO_PALLET
            and self.pallet_wp_idx is not None
            and pallet_dist < NEAR_PALLET_DIST_MM
        )
        if near_pallet and pallet_dist < PALLET_OVERSHOOT_DIST_MM and abs(alpha) > PALLET_OVERSHOOT_ALPHA:
            return self._begin_backoff(now, f"lố điểm pallet (alpha={math.degrees(alpha):.0f}°)")

        upcoming_turn = max(
            abs(_normalize_angle(self._stage_curve[curv_idx][2] - self._stage_curve[closest_idx][2])),
            abs(_normalize_angle(ltheta - self._stage_curve[closest_idx][2])),
        )
        is_curve_segment = upcoming_turn > 0.15 or abs(steering_err) > 0.15 or abs(alpha) > 0.18
        slowdown = (
            1.0 + getattr(constants, 'CONTROLS_CURVATURE_SLOWDOWN_K', 0.85)
            * (abs(steering_err) + 0.85 * upcoming_turn + 0.5 * abs(alpha))
            if is_curve_segment else 1.0
        )

        speed_ceiling = (
            constants.CONTROLS_MAX_LINEAR_SPEED_MM_S
            if is_curve_segment else constants.CONTROLS_MAX_STRAIGHT_SPEED_MM_S
        )
        min_moving_speed = getattr(constants, 'CONTROLS_MIN_MOVING_SPEED_MM_S', 38.0)
        speed_ceiling = min(speed_ceiling, max(min_moving_speed, v_prof_cap))

        if abs(alpha) > IN_PLACE_RECOVERY_ALPHA:
            linear_velocity = 0.0
        elif abs(alpha) > 0.55 or abs(steering_err) > 0.45:
            linear_velocity = min_moving_speed * max(0.35, math.cos(alpha))
        else:
            linear_velocity = _clamp(speed_ceiling / slowdown, min_moving_speed, speed_ceiling)

        if near_pallet:
            linear_velocity = min(linear_velocity, NEAR_PALLET_MAX_SPEED_MM_S)

        omega_pp = 1.55 * max(linear_velocity, min_moving_speed) * math.sin(alpha) / max(dist_to_look, 42.0)
        kp_th = getattr(constants, 'CONTROLS_PURE_PURSUIT_KP_THETA', 0.90)
        kd_th = getattr(constants, 'CONTROLS_PURE_PURSUIT_KD_THETA', 0.18)
        omega_pd = kp_th * steering_err + kd_th * d_steering
        max_omega = constants.CONTROLS_MAX_ANGULAR_SPEED_RAD_S
        angular_velocity = _clamp(omega_pp + omega_pd, -max_omega, max_omega)
        if linear_velocity > 0.0:
            linear_velocity = self._apply_speed_feedback(linear_velocity, now, rx, ry)
        return self._make_cmd(linear_velocity, angular_velocity)

    def tick(self, robot_pose, now, latency_s=0.0, live_pallet_pose=None):
        try:
            if self.is_done or not self.waypoints:
                return None

            self.last_latency_ms = round(latency_s * 1000.0, 1)

            if self.state == STATE_IDLE:
                self.state = STATE_DRIVING_TO_PALLET
                self.wp_idx = 0
                self.reset_tracking()

            if self.state == STATE_INSERTING_PALLET:
                pallet_wp = self.waypoints[self.pallet_wp_idx]
                insert_depth = abs(PALLET_INSERT_DONE_MM)
                self._last_target = [
                    pallet_wp[0] + insert_depth * math.cos(pallet_wp[2]),
                    pallet_wp[1] - insert_depth * math.sin(pallet_wp[2]),
                    pallet_wp[2],
                    pallet_wp[3],
                    pallet_wp[4],
                ]
                rx, ry, rtheta = robot_pose
                align_err, rem_forward, _ = self._pallet_aim_error(rx, ry, rtheta, pallet_wp)
                reached = rem_forward <= PALLET_INSERT_DONE_MM
                timed_out = now - self._stage_enter_time >= INSERT_MAX_TIME_S
                if reached or timed_out:
                    self.state = STATE_WAITING_LIFT_UP
                    self._lift_time = now
                    self.reset_tracking()
                    return self._make_cmd(0.0, 0.0, constants.PALLET_ACTION_LIFT_UP)
                v_ins = self._apply_speed_feedback(45.0, now, rx, ry)
                omega = _clamp(0.45 * align_err, -0.30, 0.30)
                return self._make_cmd(v_ins, omega)

            if self.state == STATE_BACKING_OFF:
                pallet_wp = self.waypoints[self.pallet_wp_idx]
                rx, ry, _ = robot_pose
                pallet_dist = math.hypot(pallet_wp[0] - rx, pallet_wp[1] - ry)
                if pallet_dist >= BACKOFF_CLEAR_DIST_MM or now - self._stage_enter_time >= BACKOFF_MAX_TIME_S:
                    self._finish_backoff(rx, ry)
                    return self._make_cmd(0.0, 0.0)
                v_back = self._apply_speed_feedback(-BACKOFF_SPEED_MM_S, now, rx, ry)
                return self._make_cmd(v_back, 0.0)

            if self.state == STATE_WAITING_LIFT_UP:
                if now - self._lift_time >= LIFT_HOLD_TIME_S:
                    self.state = STATE_DRIVING_TO_GOAL
                    if self.pallet_wp_idx is not None:
                        self._trajectory_start = now - (self.waypoints[self.pallet_wp_idx][3] + LIFT_HOLD_TIME_S)
                    self.reset_tracking()
                return self._make_cmd(0.0, 0.0, constants.PALLET_ACTION_DONT_SEND)

            if self.state == STATE_WAITING_LIFT_DOWN:
                if now - self._lift_time >= LIFT_HOLD_TIME_S:
                    self.state = STATE_DONE
                return self._make_cmd(0.0, 0.0, constants.PALLET_ACTION_DONT_SEND)

            return self._tick_trajectory(robot_pose, now)
        except Exception as e:
            print(f"[RobotController][ERROR] Lỗi trong tick R{self.hw_id}: {e}\n{traceback.format_exc()}")
            raise


class ControllerMain:

    def __init__(self, network):
        self._network     = network
        self._controllers = {}
        self._hw_id_map   = {}
        self._loaded      = False
        self._last_activity = None

    def load_paths(self, paths, robot_hw_ids, start_time=None):
        try:
            if len(paths) != len(robot_hw_ids) or len(set(robot_hw_ids)) != len(robot_hw_ids):
                raise ValueError('Path/robot IDs must match and be unique')
            start_time = time.monotonic() if start_time is None else start_time
            self._controllers.clear()
            self._hw_id_map.clear()

            for idx, (path, hw_id) in enumerate(zip(paths, robot_hw_ids)):
                if path:
                    has_pallet_tag = any(wp[4] == TAG_PALLET for wp in path)
                    self._controllers[idx] = RobotController(hw_id, path, is_pickup_mode=has_pallet_tag)
                    self._controllers[idx]._trajectory_start = start_time
                    self._hw_id_map[idx] = hw_id
                    print(f"[Controller] Robot idx={idx} hw_id={hw_id}: {len(path)} waypoints nạp thành công.")
                else:
                    print(f"[Controller] Robot idx={idx} hw_id={hw_id}: không có path, bỏ qua.")

            self._loaded = True
            print(f"[Controller] Đã nạp {len(self._controllers)} robot(s).")
        except Exception as e:
            print(f"[ControllerMain][ERROR] Lỗi load_paths: {e}\n{traceback.format_exc()}")
            raise

    @staticmethod
    def _freeze_controller_clocks(ctrl, dt):
        if dt <= 0.0:
            return
        if ctrl._trajectory_start is not None:
            ctrl._trajectory_start += dt
        if ctrl._lift_time is not None:
            ctrl._lift_time += dt
        if ctrl._stage_enter_time is not None:
            ctrl._stage_enter_time += dt

    def hold_stale(self):
        """Camera frame stale: stop robots and freeze the trajectory clock."""
        if not self._loaded:
            return
        now = time.monotonic()
        dt = 0.0 if self._last_activity is None else max(0.0, now - self._last_activity)
        self._last_activity = now
        for idx, ctrl in self._controllers.items():
            if ctrl.is_done:
                continue
            self._freeze_controller_clocks(ctrl, dt)
            ctrl._last_tick_time = now
            ctrl.reset_tracking()
            ctrl.last_wheel_velocities = (0.0, 0.0)
            ctrl.last_velocity_cmd = (0.0, 0.0)
            if not self._network.stop_robot(self._hw_id_map[idx]):
                raise RuntimeError('Stale-frame stop rejected')

    def _dispatch_command(self, hw_id, linear_mm_s, angular_deg_s, pallet_action):
        if hasattr(self._network, 'send_velocity'):
            return self._network.send_velocity(hw_id, linear_mm_s, angular_deg_s, pallet_action)
        left_pwm, right_pwm = _velocity_to_legacy_pwms(linear_mm_s, math.radians(angular_deg_s))
        return self._network.send_command(hw_id, left_pwm, right_pwm, pallet_action)

    def run_one_tick(self, cv_system, live_pallet_poses=None):
        try:
            if not self._loaded:
                return

            self._last_activity = time.monotonic()
            self._network.check_health()
            fid_ids = [constants.ROBOT_FIDUCIALS[constants.ROBOT_HARDWARE_NUMBERS.index(hw)] for hw in self._hw_id_map.values()]
            cv_system.assert_fresh(fid_ids)
            now = time.monotonic()
            captured_at = cv_system.last_captured_at() if hasattr(cv_system, 'last_captured_at') else None
            vision_lat = max(0.0, now - captured_at) if captured_at is not None else getattr(constants, 'CONTROLS_DEFAULT_LATENCY_S', 0.04)
            robot_poses_mm, robot_fid_ids = cv_system.cv_GetRobotPositionsMM()

            max_missed = getattr(constants, 'CV_MARKER_MAX_MISSED_FRAMES', 2)
            max_marker_age = getattr(constants, 'CV_MARKER_STALE_PAUSE_S', 0.18)

            for idx, ctrl in self._controllers.items():
                hw_id = self._hw_id_map[idx]
                if ctrl.is_done:
                    if not self._network.stop_robot(hw_id):
                        raise RuntimeError('Completed robot heartbeat rejected')
                    continue

                try:
                    fid_id = constants.ROBOT_FIDUCIALS[
                        constants.ROBOT_HARDWARE_NUMBERS.index(hw_id)
                    ]
                except (ValueError, IndexError) as err:
                    raise RuntimeError(f"Missing fiducial mapping for R{hw_id}: {err}")

                marker_fresh = (
                    cv_system.is_marker_fresh(fid_id, max_missed_frames=max_missed, max_age_s=max_marker_age, now=now)
                    if hasattr(cv_system, 'is_marker_fresh') else True
                )
                if not marker_fresh:
                    dt = 0.0 if ctrl._last_tick_time is None else max(0.0, now - ctrl._last_tick_time)
                    ctrl._last_tick_time = now
                    self._freeze_controller_clocks(ctrl, dt)
                    ctrl.reset_tracking()
                    ctrl.last_wheel_velocities = (0.0, 0.0)
                    ctrl.last_velocity_cmd = (0.0, 0.0)
                    if not self._network.stop_robot(hw_id):
                        raise RuntimeError(f"Marker-loss stop rejected for R{hw_id}")
                    if not ctrl._paused_marker:
                        print(f"[Controller] R{hw_id} PAUSED (marker {fid_id} lost)")
                        ctrl._paused_marker = True
                    continue

                if ctrl._paused_marker:
                    print(f"[Controller] R{hw_id} RESUMED (marker {fid_id} re-acquired)")
                    ctrl._paused_marker = False
                ctrl._last_tick_time = now

                pose = _find_pose_by_fid(robot_poses_mm, robot_fid_ids, fid_id)
                if pose is None:
                    raise RuntimeError(f"Missing robot pose: {hw_id}")

                net_rtt = self._network.get_rtt(hw_id) if hasattr(self._network, 'get_rtt') else 0.04
                total_latency_s = vision_lat + 0.5 * net_rtt
                live_pallet = live_pallet_poses.get(hw_id) if live_pallet_poses else None

                result = ctrl.tick(pose, now, latency_s=total_latency_s, live_pallet_pose=live_pallet)
                if result is None:
                    continue

                linear_mm_s, angular_deg_s, pallet_action = result
                if not self._dispatch_command(hw_id, linear_mm_s, angular_deg_s, pallet_action):
                    raise RuntimeError(f"Command rejected for R{hw_id}")

                if constants.CONTROLS_DEBUG:
                    wp = ctrl.current_waypoint
                    print(f"[Controller] R{hw_id} state={ctrl.state} "
                          f"wp={ctrl.wp_idx}/{len(ctrl.waypoints)} "
                          f"pose=({pose[0]:.0f},{pose[1]:.0f},{math.degrees(pose[2]):.1f}deg) "
                          f"target=({wp[0]:.0f},{wp[1]:.0f}) "
                          f"VW=({linear_mm_s:.0f},{angular_deg_s:.0f}) lat={ctrl.last_latency_ms:.0f}ms action={pallet_action}")
        except Exception as e:
            print(f"[ControllerMain][ERROR] Lỗi trong run_one_tick: {e}\n{traceback.format_exc()}")
            raise

    def get_wheel_speeds(self):
        speeds = {}
        try:
            for idx, ctrl in self._controllers.items():
                hw_id = self._hw_id_map.get(idx)
                if hw_id is not None:
                    try:
                        fid_id = constants.ROBOT_FIDUCIALS[
                            constants.ROBOT_HARDWARE_NUMBERS.index(hw_id)
                        ]
                        speeds[fid_id] = getattr(ctrl, 'last_wheel_velocities', (0.0, 0.0))
                    except (ValueError, IndexError):
                        pass
        except Exception as e:
            print(f"[ControllerMain][ERROR] Lỗi get_wheel_speeds: {e}")
        return speeds

    def all_done(self):
        try:
            return self._loaded and all(ctrl.is_done for ctrl in self._controllers.values())
        except Exception as e:
            print(f"[ControllerMain][ERROR] Lỗi all_done: {e}\n{traceback.format_exc()}")
            return False

    def get_status(self):
        status = {}
        try:
            for idx, ctrl in self._controllers.items():
                status[idx] = {
                    "hw_id":    self._hw_id_map[idx],
                    "state":    ctrl.state,
                    "wp_idx":   ctrl.wp_idx,
                    "total_wp": len(ctrl.waypoints),
                    "done":     ctrl.is_done,
                    "latency_ms": ctrl.last_latency_ms,
                    "paused_marker": ctrl._paused_marker,
                    "cmd_vw":   ctrl.last_velocity_cmd,
                }
        except Exception as e:
            print(f"[ControllerMain][ERROR] Lỗi get_status: {e}\n{traceback.format_exc()}")
        return status

    def emergency_stop(self):
        try:
            print("[Controller] !!! EMERGENCY STOP !!!")
            self._network.stop_all(blocking=True)
            for ctrl in self._controllers.values():
                ctrl.state = STATE_DONE
        except Exception as e:
            print(f"[ControllerMain][ERROR] Lỗi emergency_stop: {e}\n{traceback.format_exc()}")


def _normalize_angle(angle):
    try:
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    except Exception as e:
        print(f"[_normalize_angle][ERROR]: {e}")
        return 0.0


def _interpolate_angle(start, end, amount):
    return _normalize_angle(start + _normalize_angle(end - start) * amount)


def _clamp(value, lo, hi):
    try:
        return max(lo, min(hi, value))
    except Exception as e:
        print(f"[_clamp][ERROR]: {e}")
        return lo


def _build_stage_curve(waypoints, stage_start_idx, stage_end_idx, samples_per_seg=12):
    v_straight = getattr(constants, 'CONTROLS_MAX_STRAIGHT_SPEED_MM_S', 75.0)
    if stage_start_idx >= stage_end_idx:
        wp = waypoints[stage_start_idx]
        return [(wp[0], wp[1], wp[2], stage_start_idx, 0.0, v_straight)]

    pts = []
    for seg in range(stage_start_idx, stage_end_idx):
        t0 = waypoints[seg][3]
        t1 = waypoints[seg + 1][3]
        for k in range(samples_per_seg):
            t = t0 + (k / float(samples_per_seg)) * (t1 - t0)
            ref = trajectory_reference(waypoints, t, seg, seg + 1)[1]
            pts.append((ref[0], ref[1], ref[2], seg))
    end_wp = waypoints[stage_end_idx]
    pts.append((end_wp[0], end_wp[1], end_wp[2], stage_end_idx))

    n = len(pts)
    s_list = [0.0] * n
    for i in range(1, n):
        s_list[i] = s_list[i - 1] + math.hypot(pts[i][0] - pts[i - 1][0], pts[i][1] - pts[i - 1][1])

    a_lat = getattr(constants, 'CONTROLS_PROFILE_LAT_ACCEL_MM_S2', 50.0)
    decel = getattr(constants, 'CONTROLS_PROFILE_DECEL_MM_S2', 75.0)
    v_min = getattr(constants, 'CONTROLS_MIN_MOVING_SPEED_MM_S', 28.0)
    v_prof = [v_straight] * n
    for i in range(1, n - 1):
        ds = max(s_list[i + 1] - s_list[i - 1], 1e-3)
        dth = abs(_normalize_angle(pts[i + 1][2] - pts[i - 1][2]))
        kappa = dth / ds
        if kappa > 1e-4:
            v_prof[i] = min(v_straight, max(v_min, math.sqrt(a_lat / kappa)))
    v_prof[-1] = min(v_prof[-1], getattr(constants, 'CONTROLS_MAX_LINEAR_SPEED_MM_S', 48.0))

    # Backward pass: brake early before curves and dock points
    for i in range(n - 2, -1, -1):
        ds = max(s_list[i + 1] - s_list[i], 0.0)
        v_prof[i] = min(v_prof[i], math.sqrt(v_prof[i + 1] ** 2 + 2.0 * decel * ds))

    return [(pts[i][0], pts[i][1], pts[i][2], pts[i][3], s_list[i], v_prof[i]) for i in range(n)]


def trajectory_reference(waypoints, elapsed, stage_start_idx=0, stage_end_idx=None):
    if stage_end_idx is None:
        stage_end_idx = len(waypoints) - 1
    is_curve_segment = False
    stage_end_time = waypoints[stage_end_idx][3]
    curr_elapsed   = min(elapsed, stage_end_time)
    rotation_rate = 0.0

    if len(waypoints) <= 1:
        index = 0
        target = waypoints[0]
        dxdt = dydt = ddxdt = ddydt = 0.0
    else:
        max_cand = max(stage_start_idx, stage_end_idx - 1)
        index = max_cand
        for candidate in range(stage_start_idx, stage_end_idx):
            if curr_elapsed < waypoints[candidate + 1][3]:
                index = candidate
                break

        first    = waypoints[index]
        second   = waypoints[index + 1]
        duration = max(second[3] - first[3], 1e-3)
        u        = _clamp((curr_elapsed - first[3]) / duration, 0.0, 1.0)
        dx       = second[0] - first[0]
        dy       = second[1] - first[1]
        same_heading  = abs(_normalize_angle(second[2] - first[2])) < 1e-3
        same_position = math.hypot(dx, dy) < 1e-3
        is_curve_segment = not same_heading and not same_position

        if same_heading:
            target = [first[0] + u * dx, first[1] + u * dy, first[2], second[3], second[4]]
            dxdt = dx / duration
            dydt = dy / duration
            ddxdt = ddydt = 0.0
        elif same_position:
            target = [first[0], first[1],
                      first[2] + (second[2] - first[2]) * u,
                      second[3], second[4]]
            dxdt = dydt = ddxdt = ddydt = 0.0
            rotation_rate = (second[2] - first[2]) / duration
        else:
            seg_length    = math.hypot(dx, dy)
            tangent_scale = constants.tangent_curviness * seg_length
            m0 = (math.cos(first[2]) * tangent_scale,
                  -math.sin(first[2]) * tangent_scale)
            m1 = (math.cos(second[2]) * tangent_scale,
                  -math.sin(second[2]) * tangent_scale)
            h00 = 2 * u**3 - 3 * u**2 + 1
            h10 = u**3 - 2 * u**2 + u
            h01 = -2 * u**3 + 3 * u**2
            h11 = u**3 - u**2
            target = [
                h00 * first[0] + h10 * m0[0] + h01 * second[0] + h11 * m1[0],
                h00 * first[1] + h10 * m0[1] + h01 * second[1] + h11 * m1[1],
                0.0, second[3], second[4],
            ]
            dh00 = 6 * u**2 - 6 * u
            dh10 = 3 * u**2 - 4 * u + 1
            dh01 = -dh00
            dh11 = 3 * u**2 - 2 * u
            dxdt = (dh00 * first[0] + dh10 * m0[0] + dh01 * second[0] + dh11 * m1[0]) / duration
            dydt = (dh00 * first[1] + dh10 * m0[1] + dh01 * second[1] + dh11 * m1[1]) / duration
            ddh00 = 12 * u - 6
            ddh10 = 6 * u - 4
            ddh01 = -ddh00
            ddh11 = 6 * u - 2
            ddxdt = (ddh00 * first[0] + ddh10 * m0[0]
                     + ddh01 * second[0] + ddh11 * m1[0]) / (duration * duration)
            ddydt = (ddh00 * first[1] + ddh10 * m0[1]
                     + ddh01 * second[1] + ddh11 * m1[1]) / (duration * duration)
            target[2] = math.atan2(-dydt, dxdt)

    if len(waypoints) > 1 and (elapsed < first[3] or elapsed >= second[3]):
        endpoint = first if elapsed < first[3] else second
        target = list(endpoint)
        dxdt = dydt = ddxdt = ddydt = rotation_rate = 0.0

    return index, target, dxdt, dydt, ddxdt, ddydt, rotation_rate, is_curve_segment


def _velocity_to_legacy_pwms(linear_mm_s, angular_rad_s):
    """Fallback PWM conversion only when paired with a legacy network mock lacking send_velocity."""
    half_base = constants.CONTROLS_WHEEL_BASE_MM / 2.0
    v_left = linear_mm_s - half_base * angular_rad_s
    v_right = linear_mm_s + half_base * angular_rad_s
    scale = min(1.0, constants.CONTROLS_CALIBRATED_SPEED_MM_S / max(abs(v_left), abs(v_right), 1e-9))
    v_left *= scale
    v_right *= scale

    def map_wheel(vel, sign):
        if abs(vel) < 2.0:
            return PWM_NEUTRAL
        frac = min(abs(vel) / max(constants.CONTROLS_CALIBRATED_SPEED_MM_S, 1.0), 1.0)
        delta = constants.CONTROLS_PWM_START_OFFSET + frac * (PWM_MAX_DELTA - constants.CONTROLS_PWM_START_OFFSET)
        return int(round(_clamp(PWM_NEUTRAL + math.copysign(delta, vel * sign), PWM_MIN, PWM_MAX)))

    return map_wheel(v_left, constants.CONTROLS_WHEEL_LEFT_SIGN), map_wheel(v_right, constants.CONTROLS_WHEEL_RIGHT_SIGN)


def _find_pose_by_fid(poses, fid_ids, target_fid):
    try:
        for pose, fid in zip(poses, fid_ids):
            if fid == target_fid:
                return pose
    except Exception as e:
        print(f"[_find_pose_by_fid][ERROR]: {e}\n{traceback.format_exc()}")
    return None
