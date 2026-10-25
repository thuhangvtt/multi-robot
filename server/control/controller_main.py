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

WAYPOINT_XY_THRESH_MM = 30.0
WAYPOINT_THETA_THRESH = 0.25
PALLET_XY_THRESH_MM   = 20.0
LIFT_HOLD_TIME_S      = constants.CONTROLS_PALLET_ACTION_TIME_THRESHOLD

STATE_IDLE              = "IDLE"
STATE_DRIVING_TO_PALLET = "DRIVING_TO_PALLET"
STATE_WAITING_LIFT_UP   = "WAITING_LIFT_UP"
STATE_DRIVING_TO_GOAL   = "DRIVING_TO_GOAL"
STATE_WAITING_LIFT_DOWN = "WAITING_LIFT_DOWN"
STATE_DONE              = "DONE"

TAG_DRIVE  = 0
TAG_PALLET = 1
TAG_GOAL   = 2


class PID:
    """PID controller với giới hạn chống bão hòa tích phân (anti-windup)."""

    def __init__(self, kp, ki, kd, output_limit=None):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self._limit = output_limit
        self._integral  = 0.0
        self._prev_err  = 0.0
        self._prev_time = None

    def reset(self):
        self._integral  = 0.0
        self._prev_err  = 0.0
        self._prev_time = None

    def compute(self, error, now=None):
        try:
            if now is None:
                now = time.time()

            if self._prev_time is None:
                dt = 0.05
            else:
                dt = max(now - self._prev_time, 1e-4)

            self._integral += error * dt
            derivative = (error - self._prev_err) / dt
            output = self.kp * error + self.ki * self._integral + self.kd * derivative

            if self._limit is not None:
                if output > self._limit:
                    self._integral -= error * dt
                    output = self._limit
                elif output < -self._limit:
                    self._integral -= error * dt
                    output = -self._limit

            self._prev_err  = error
            self._prev_time = now
            return output
        except Exception as e:
            print(f"[PID][ERROR] Lỗi compute PID: {e}\n{traceback.format_exc()}")
            return 0.0


class RobotController:
    """Điều khiển một robot bám theo danh sách waypoints."""

    def __init__(self, hw_id, waypoints, is_pickup_mode=False):
        self.hw_id     = hw_id
        self.waypoints = waypoints
        self.wp_idx    = 0
        self.state     = STATE_IDLE if waypoints else STATE_DONE
        self._lift_time = None

        if is_pickup_mode:
            self.pid_x     = PID(constants.CONTROLS_PICKUP_ROBOT_PID_KPx,
                                 constants.CONTROLS_PICKUP_ROBOT_PID_KIx,
                                 constants.CONTROLS_PICKUP_ROBOT_PID_KDx,
                                 output_limit=PWM_MAX_DELTA)
            self.pid_y     = PID(constants.CONTROLS_PICKUP_ROBOT_PID_KPy,
                                 constants.CONTROLS_PICKUP_ROBOT_PID_KIy,
                                 constants.CONTROLS_PICKUP_ROBOT_PID_KDy,
                                 output_limit=PWM_MAX_DELTA)
            self.pid_theta = PID(constants.CONTROLS_PICKUP_ROBOT_PID_KPtheta,
                                 constants.CONTROLS_PICKUP_ROBOT_PID_KItheta,
                                 constants.CONTROLS_PICKUP_ROBOT_PID_KDtheta,
                                 output_limit=PWM_MAX_DELTA)
        else:
            self.pid_x     = PID(constants.CONTROLS_ROBOT_PID_KPx,
                                 constants.CONTROLS_ROBOT_PID_KIx,
                                 constants.CONTROLS_ROBOT_PID_KDx,
                                 output_limit=PWM_MAX_DELTA)
            self.pid_y     = PID(constants.CONTROLS_ROBOT_PID_KPy,
                                 constants.CONTROLS_ROBOT_PID_KIy,
                                 constants.CONTROLS_ROBOT_PID_KDy,
                                 output_limit=PWM_MAX_DELTA)
            self.pid_theta = PID(constants.CONTROLS_ROBOT_PID_KPtheta,
                                 constants.CONTROLS_ROBOT_PID_KItheta,
                                 constants.CONTROLS_ROBOT_PID_KDtheta,
                                 output_limit=PWM_MAX_DELTA)

    @property
    def is_done(self):
        return self.state == STATE_DONE

    @property
    def current_waypoint(self):
        if self.wp_idx < len(self.waypoints):
            return self.waypoints[self.wp_idx]
        return None

    def reset_pids(self):
        self.pid_x.reset()
        self.pid_y.reset()
        self.pid_theta.reset()

    def tick(self, robot_pose, now):
        """Tính toán lệnh PWM và hành động servo từ vị trí thực tế [x, y, theta]."""
        try:
            if self.is_done or not self.waypoints:
                return None

            if self.state == STATE_IDLE:
                self.state = STATE_DRIVING_TO_PALLET
                self.wp_idx = 0
                self.reset_pids()

            if self.state == STATE_WAITING_LIFT_UP:
                if now - self._lift_time >= LIFT_HOLD_TIME_S:
                    self.state = STATE_DRIVING_TO_GOAL
                    self.wp_idx += 1
                    self.reset_pids()
                return (PWM_NEUTRAL, PWM_NEUTRAL, constants.PALLET_ACTION_DONT_SEND)

            if self.state == STATE_WAITING_LIFT_DOWN:
                if now - self._lift_time >= LIFT_HOLD_TIME_S:
                    self.state = STATE_DONE
                return (PWM_NEUTRAL, PWM_NEUTRAL, constants.PALLET_ACTION_DONT_SEND)

            wp = self.current_waypoint
            if wp is None:
                self.state = STATE_DONE
                return (PWM_NEUTRAL, PWM_NEUTRAL, constants.PALLET_ACTION_DONT_SEND)

            target_x, target_y, target_theta, _, tag = wp
            rx, ry, rtheta = robot_pose

            dx = target_x - rx
            dy = target_y - ry
            dist = math.sqrt(dx * dx + dy * dy)

            angle_to_target = math.atan2(dy, dx)
            heading_err = _normalize_angle(angle_to_target - rtheta)
            theta_err   = _normalize_angle(target_theta - rtheta)

            err_x = dist * math.cos(heading_err)
            err_y = dist * math.sin(heading_err)

            xy_thresh = PALLET_XY_THRESH_MM if tag == TAG_PALLET else WAYPOINT_XY_THRESH_MM

            if dist < xy_thresh and abs(theta_err) < WAYPOINT_THETA_THRESH:
                if tag == TAG_PALLET:
                    self.state = STATE_WAITING_LIFT_UP
                    self._lift_time = now
                    return (PWM_NEUTRAL, PWM_NEUTRAL, constants.PALLET_ACTION_LIFT_UP)

                elif tag == TAG_GOAL:
                    self.state = STATE_WAITING_LIFT_DOWN
                    self._lift_time = now
                    return (PWM_NEUTRAL, PWM_NEUTRAL, constants.PALLET_ACTION_LIFT_DOWN)

                else:
                    self.wp_idx += 1
                    self.reset_pids()
                    if self.wp_idx >= len(self.waypoints):
                        self.state = STATE_DONE
                    return (PWM_NEUTRAL, PWM_NEUTRAL, constants.PALLET_ACTION_DONT_SEND)

            out_x     = self.pid_x.compute(err_x, now)
            out_y     = self.pid_y.compute(err_y, now)
            out_theta = self.pid_theta.compute(theta_err, now)

            forward = out_x
            turn    = out_y + out_theta

            left_delta  = forward - turn
            right_delta = forward + turn

            left_pwm  = int(_clamp(PWM_NEUTRAL + left_delta,  PWM_MIN, PWM_MAX))
            right_pwm = int(_clamp(PWM_NEUTRAL + right_delta, PWM_MIN, PWM_MAX))

            return (left_pwm, right_pwm, constants.PALLET_ACTION_DONT_SEND)
        except Exception as e:
            print(f"[RobotController][ERROR] Lỗi trong tick R{self.hw_id}: {e}\n{traceback.format_exc()}")
            return (PWM_NEUTRAL, PWM_NEUTRAL, constants.PALLET_ACTION_DONT_SEND)


class ControllerMain:
    """Điều phối điều khiển cho tất cả robot trong hệ thống."""

    def __init__(self, network):
        self._network     = network
        self._controllers = {}
        self._hw_id_map   = {}
        self._loaded      = False

    def load_paths(self, paths, robot_hw_ids):
        """Nạp danh sách quỹ đạo cho từng robot theo hardware ID."""
        try:
            self._controllers.clear()
            self._hw_id_map.clear()

            for idx, (path, hw_id) in enumerate(zip(paths, robot_hw_ids)):
                if path:
                    has_pallet_tag = any(wp[4] == TAG_PALLET for wp in path)
                    self._controllers[idx] = RobotController(hw_id, path, is_pickup_mode=has_pallet_tag)
                    self._hw_id_map[idx]   = hw_id
                    print(f"[Controller] Robot idx={idx} hw_id={hw_id}: {len(path)} waypoints nạp thành công.")
                else:
                    print(f"[Controller] Robot idx={idx} hw_id={hw_id}: không có path, bỏ qua.")

            self._loaded = True
            print(f"[Controller] Đã nạp {len(self._controllers)} robot(s).")
        except Exception as e:
            print(f"[ControllerMain][ERROR] Lỗi load_paths: {e}\n{traceback.format_exc()}")

    def run_one_tick(self, cv_system):
        """Chạy một chu kỳ điều khiển: đọc vị trí từ camera, tính PID và gửi lệnh Wi-Fi."""
        try:
            if not self._loaded:
                return

            now = time.time()
            robot_poses_mm, robot_fid_ids = cv_system.cv_GetRobotPositionsMM()

            for idx, ctrl in self._controllers.items():
                if ctrl.is_done:
                    continue

                hw_id = self._hw_id_map[idx]

                try:
                    fid_id = constants.ROBOT_FIDUCIALS[
                        constants.ROBOT_HARDWARE_NUMBERS.index(hw_id)
                    ]
                    pose = _find_pose_by_fid(robot_poses_mm, robot_fid_ids, fid_id)
                except (ValueError, IndexError) as err:
                    print(f"[ControllerMain][WARN] Không tìm thấy fiducial ID cho R{hw_id}: {err}")
                    pose = None

                if pose is None:
                    self._network.stop_robot(hw_id)
                    print(f"[Controller] R{hw_id}: Không thấy trên camera, dừng tạm.")
                    continue

                result = ctrl.tick(pose, now)
                if result is None:
                    continue

                left_pwm, right_pwm, pallet_action = result
                self._network.send_command(hw_id, left_pwm, right_pwm, pallet_action)

                if constants.CONTROLS_DEBUG:
                    wp = ctrl.current_waypoint
                    print(f"[Controller] R{hw_id} state={ctrl.state} "
                          f"wp={ctrl.wp_idx}/{len(ctrl.waypoints)} "
                          f"pose=({pose[0]:.0f},{pose[1]:.0f},{math.degrees(pose[2]):.1f}deg) "
                          f"target=({wp[0]:.0f},{wp[1]:.0f}) "
                          f"PWM=({left_pwm},{right_pwm}) action={pallet_action}")
        except Exception as e:
            print(f"[ControllerMain][ERROR] Lỗi trong run_one_tick: {e}\n{traceback.format_exc()}")

    def all_done(self):
        """Kiểm tra tất cả robot đã hoàn thành nhiệm vụ."""
        try:
            if not self._controllers:
                return True
            return all(ctrl.is_done for ctrl in self._controllers.values())
        except Exception as e:
            print(f"[ControllerMain][ERROR] Lỗi all_done: {e}\n{traceback.format_exc()}")
            return False

    def get_status(self):
        """Trả về trạng thái hiện tại của từng robot."""
        status = {}
        try:
            for idx, ctrl in self._controllers.items():
                status[idx] = {
                    "hw_id":    self._hw_id_map[idx],
                    "state":    ctrl.state,
                    "wp_idx":   ctrl.wp_idx,
                    "total_wp": len(ctrl.waypoints),
                    "done":     ctrl.is_done,
                }
        except Exception as e:
            print(f"[ControllerMain][ERROR] Lỗi get_status: {e}\n{traceback.format_exc()}")
        return status

    def emergency_stop(self):
        """Dừng khẩn cấp toàn bộ robot."""
        try:
            print("[Controller] !!! EMERGENCY STOP !!!")
            self._network.stop_all(blocking=True)
            for ctrl in self._controllers.values():
                ctrl.state = STATE_DONE
        except Exception as e:
            print(f"[ControllerMain][ERROR] Lỗi emergency_stop: {e}\n{traceback.format_exc()}")


def _normalize_angle(angle):
    """Chuẩn hóa góc về khoảng [-pi, pi]."""
    try:
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    except Exception as e:
        print(f"[_normalize_angle][ERROR]: {e}")
        return 0.0


def _clamp(value, lo, hi):
    """Giới hạn giá trị trong khoảng [lo, hi]."""
    try:
        return max(lo, min(hi, value))
    except Exception as e:
        print(f"[_clamp][ERROR]: {e}")
        return lo


def _find_pose_by_fid(poses, fid_ids, target_fid):
    """Tìm pose [x, y, theta] của marker theo fiducial ID."""
    try:
        for pose, fid in zip(poses, fid_ids):
            if fid == target_fid:
                return pose
    except Exception as e:
        print(f"[_find_pose_by_fid][ERROR]: {e}\n{traceback.format_exc()}")
    return None
