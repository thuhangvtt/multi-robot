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
        self._trajectory_start = None

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

    def _tick_trajectory(self, robot_pose, now):
        if self._trajectory_start is None:
            self._trajectory_start = now

        elapsed = now - self._trajectory_start
        if len(self.waypoints) == 1:
            index = 0
            target = self.waypoints[0]
            dxdt = dydt = 0.0
        else:
            index = len(self.waypoints) - 2
            for candidate in range(len(self.waypoints) - 1):
                if elapsed <= self.waypoints[candidate + 1][3]:
                    index = candidate
                    break

            first = self.waypoints[index]
            second = self.waypoints[index + 1]
            duration = max(second[3] - first[3], 1e-3)
            u = _clamp((elapsed - first[3]) / duration, 0.0, 1.0)
            dx = second[0] - first[0]
            dy = second[1] - first[1]
            distance = math.hypot(dx, dy)
            tangent_scale = max(distance, 1.0)
            m0 = (
                math.cos(first[2]) * tangent_scale,
                math.sin(first[2]) * tangent_scale,
            )
            m1 = (
                math.cos(second[2]) * tangent_scale,
                math.sin(second[2]) * tangent_scale,
            )
            h00 = 2 * u ** 3 - 3 * u ** 2 + 1
            h10 = u ** 3 - 2 * u ** 2 + u
            h01 = -2 * u ** 3 + 3 * u ** 2
            h11 = u ** 3 - u ** 2
            target = [
                h00 * first[0] + h10 * m0[0] + h01 * second[0] + h11 * m1[0],
                h00 * first[1] + h10 * m0[1] + h01 * second[1] + h11 * m1[1],
                _interpolate_angle(first[2], second[2], u),
                second[3],
                second[4],
            ]
            dh00 = 6 * u ** 2 - 6 * u
            dh10 = 3 * u ** 2 - 4 * u + 1
            dh01 = -dh00
            dh11 = 3 * u ** 2 - 2 * u
            dxdt = (dh00 * first[0] + dh10 * m0[0] + dh01 * second[0] + dh11 * m1[0]) / duration
            dydt = (dh00 * first[1] + dh10 * m0[1] + dh01 * second[1] + dh11 * m1[1]) / duration

        self.wp_idx = index
        rx, ry, rtheta = robot_pose
        target_x, target_y, target_theta, target_time, tag = target
        error_x = target_x - rx
        error_y = target_y - ry
        distance = math.hypot(error_x, error_y)
        heading = math.atan2(error_y, error_x)
        heading_err = _normalize_angle(heading - rtheta)
        theta_err = _normalize_angle(target_theta - rtheta)
        cos_theta = math.cos(rtheta)
        sin_theta = math.sin(rtheta)
        body_x = cos_theta * error_x + sin_theta * error_y
        body_y = -sin_theta * error_x + cos_theta * error_y

        if tag in (TAG_PALLET, TAG_GOAL) and elapsed >= target_time:
            threshold = PALLET_XY_THRESH_MM if tag == TAG_PALLET else WAYPOINT_XY_THRESH_MM
            if distance <= threshold and abs(theta_err) <= WAYPOINT_THETA_THRESH:
                self.state = STATE_WAITING_LIFT_UP if tag == TAG_PALLET else STATE_WAITING_LIFT_DOWN
                self._lift_time = now
                return (PWM_NEUTRAL, PWM_NEUTRAL, (
                    constants.PALLET_ACTION_LIFT_UP if tag == TAG_PALLET
                    else constants.PALLET_ACTION_LIFT_DOWN
                ))

        if elapsed >= self.waypoints[-1][3] and distance < WAYPOINT_XY_THRESH_MM:
            self.state = STATE_DONE
            return (PWM_NEUTRAL, PWM_NEUTRAL, constants.PALLET_ACTION_DONT_SEND)

        mm_s_per_pwm = 1.0 / max(constants.CONTROLS_PWM_PER_MM_S, 1e-6)
        feedforward = math.hypot(dxdt, dydt)
        linear_velocity = max(
            0.0,
            self.pid_x.compute(body_x, now) * mm_s_per_pwm + feedforward,
        )
        angular_velocity = (
            self.pid_y.compute(body_y, now)
            + self.pid_theta.compute(theta_err, now)
        ) * mm_s_per_pwm / max(constants.CONTROLS_WHEEL_BASE_MM / 2, 1)
        linear_velocity = _clamp(
            linear_velocity,
            -constants.CONTROLS_MAX_LINEAR_SPEED_MM_S,
            constants.CONTROLS_MAX_LINEAR_SPEED_MM_S,
        )
        angular_velocity = _clamp(
            angular_velocity,
            -constants.CONTROLS_MAX_ANGULAR_SPEED_RAD_S,
            constants.CONTROLS_MAX_ANGULAR_SPEED_RAD_S,
        )
        half_base = constants.CONTROLS_WHEEL_BASE_MM / 2
        left_velocity = linear_velocity - half_base * angular_velocity
        right_velocity = linear_velocity + half_base * angular_velocity
        left_pwm = _velocity_to_pwm(left_velocity, constants.CONTROLS_WHEEL_LEFT_SIGN)
        right_pwm = _velocity_to_pwm(right_velocity, constants.CONTROLS_WHEEL_RIGHT_SIGN)
        left_pwm, right_pwm = _apply_pwm_safety_filter(left_pwm, right_pwm)
        return (left_pwm, right_pwm, constants.PALLET_ACTION_DONT_SEND)

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

            return self._tick_trajectory(robot_pose, now)
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


def _interpolate_angle(start, end, amount):
    return _normalize_angle(start + _normalize_angle(end - start) * amount)


def _clamp(value, lo, hi):
    """Giới hạn giá trị trong khoảng [lo, hi]."""
    try:
        return max(lo, min(hi, value))
    except Exception as e:
        print(f"[_clamp][ERROR]: {e}")
        return lo


def _velocity_to_pwm(velocity, wheel_sign):
    if abs(velocity) < 1e-6:
        return PWM_NEUTRAL
    pwm_delta = abs(velocity) * constants.CONTROLS_PWM_PER_MM_S
    pwm_delta += constants.CONTROLS_PWM_START_OFFSET
    pwm_delta = min(pwm_delta, PWM_MAX_DELTA)
    return int(round(PWM_NEUTRAL + math.copysign(pwm_delta, velocity * wheel_sign)))


def _apply_pwm_safety_filter(left_pwm, right_pwm):
    left_delta = _clamp(left_pwm - PWM_NEUTRAL, -PWM_MAX_DELTA, PWM_MAX_DELTA)
    right_delta = _clamp(right_pwm - PWM_NEUTRAL, -PWM_MAX_DELTA, PWM_MAX_DELTA)
    largest_delta = max(abs(left_delta), abs(right_delta))
    if largest_delta > PWM_MAX_DELTA:
        scale = PWM_MAX_DELTA / largest_delta
        left_delta *= scale
        right_delta *= scale
    return (
        int(round(_clamp(PWM_NEUTRAL + left_delta, PWM_MIN, PWM_MAX))),
        int(round(_clamp(PWM_NEUTRAL + right_delta, PWM_MIN, PWM_MAX)),
        ),
    )


def _find_pose_by_fid(poses, fid_ids, target_fid):
    """Tìm pose [x, y, theta] của marker theo fiducial ID."""
    try:
        for pose, fid in zip(poses, fid_ids):
            if fid == target_fid:
                return pose
    except Exception as e:
        print(f"[_find_pose_by_fid][ERROR]: {e}\n{traceback.format_exc()}")
    return None
