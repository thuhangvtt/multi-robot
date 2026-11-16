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

WAYPOINT_XY_THRESH_MM = 40.0
WAYPOINT_THETA_THRESH = 0.4
PALLET_XY_THRESH_MM   = 45.0
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
                now = time.monotonic()

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

    def __init__(self, hw_id, waypoints, is_pickup_mode=False):
        self.hw_id     = hw_id
        self.waypoints = waypoints
        self.wp_idx    = 0
        self.state     = STATE_IDLE if waypoints else STATE_DONE
        self._lift_time = None
        self._trajectory_start = None
        self._previous_tracking_error = (0.0, 0.0, 0.0)
        self._pickup_gains = is_pickup_mode

        self.pallet_wp_idx = None
        for idx, wp in enumerate(self.waypoints):
            if int(wp[4]) == TAG_PALLET:
                self.pallet_wp_idx = idx

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
        self._previous_tracking_error = (0.0, 0.0, 0.0)
        self.pid_x.reset()
        self.pid_y.reset()
        self.pid_theta.reset()

    def _find_closest_waypoint_idx(self, rx, ry, start_idx, end_idx):

        best_idx = start_idx
        best_dist = float('inf')
        for i in range(start_idx, end_idx + 1):
            wp = self.waypoints[i]
            d = math.hypot(wp[0] - rx, wp[1] - ry)
            if d < best_dist:
                best_dist = d
                best_idx = i
        return best_idx, best_dist

    def _tick_trajectory(self, robot_pose, now):

        if self._trajectory_start is None:
            self._trajectory_start = now

        rx, ry, rtheta = robot_pose
        is_curve_segment = False

        if self.state == STATE_DRIVING_TO_PALLET and self.pallet_wp_idx is not None:
            stage_start_idx = 0
            stage_end_idx   = self.pallet_wp_idx
            pallet_wp       = self.waypoints[self.pallet_wp_idx]
            pallet_dist     = math.hypot(pallet_wp[0] - rx, pallet_wp[1] - ry)
            pallet_theta_err = _normalize_angle(pallet_wp[2] - rtheta)
            if (pallet_dist <= PALLET_XY_THRESH_MM
                    and abs(pallet_theta_err) <= WAYPOINT_THETA_THRESH
                    and now - self._trajectory_start >= pallet_wp[3]):
                self.state    = STATE_WAITING_LIFT_UP
                self._lift_time = now
                self.reset_pids()
                return (PWM_NEUTRAL, PWM_NEUTRAL, constants.PALLET_ACTION_LIFT_UP)
        else:
            stage_start_idx = self.pallet_wp_idx if self.pallet_wp_idx is not None else 0
            stage_end_idx   = len(self.waypoints) - 1
            goal_wp         = self.waypoints[stage_end_idx]
            goal_dist       = math.hypot(goal_wp[0] - rx, goal_wp[1] - ry)
            goal_theta_err  = _normalize_angle(goal_wp[2] - rtheta)
            if (goal_dist <= WAYPOINT_XY_THRESH_MM
                    and abs(goal_theta_err) <= WAYPOINT_THETA_THRESH
                    and now - self._trajectory_start >= goal_wp[3]):
                self.state    = STATE_WAITING_LIFT_DOWN
                self._lift_time = now
                self.reset_pids()
                return (PWM_NEUTRAL, PWM_NEUTRAL, constants.PALLET_ACTION_LIFT_DOWN)

        elapsed = now - self._trajectory_start

        (index, target, dxdt, dydt, ddxdt, ddydt,
         rotation_rate, is_curve_segment) = trajectory_reference(
            self.waypoints, elapsed, stage_start_idx, stage_end_idx)

        self.wp_idx = index
        target_x, target_y, target_theta, target_time, tag = target
        error_x = target_x - rx
        error_y = target_y - ry
        theta_err = _normalize_angle(target_theta - rtheta)

        error_y_cart = -error_y
        cos_theta = math.cos(rtheta)
        sin_theta = math.sin(rtheta)

        body_x = cos_theta * error_x + sin_theta * error_y_cart
        body_y = -sin_theta * error_x + cos_theta * error_y_cart

        feedforward = math.hypot(dxdt, dydt)
        feedforward_angular = rotation_rate
        if feedforward > 1e-6:
            feedforward_angular = (dydt * ddxdt - dxdt * ddydt) / (feedforward ** 2)

        tracking_correction, angular_feedback = self._tracking_feedback(
            body_x, body_y, theta_err, tag)
        tracking_correction = _clamp(
            tracking_correction,
            -constants.CONTROLS_MAX_TRACKING_CORRECTION_MM_S,
            constants.CONTROLS_MAX_TRACKING_CORRECTION_MM_S,
        )
        linear_velocity = max(0.0, feedforward + tracking_correction)
        half_base = constants.CONTROLS_WHEEL_BASE_MM / 2.0

        angular_velocity = feedforward_angular + angular_feedback

        speed_ceiling = (
            constants.CONTROLS_MAX_LINEAR_SPEED_MM_S
            if is_curve_segment
            else constants.CONTROLS_MAX_STRAIGHT_SPEED_MM_S
        )
        slowdown = 1.0 + getattr(constants, 'CONTROLS_CURVATURE_SLOWDOWN_K', 0.85) * abs(angular_velocity)
        if abs(body_y) > 25.0:
            slowdown += getattr(constants, 'CONTROLS_DEVIATION_SLOWDOWN_K', 0.015) * (abs(body_y) - 25.0)

        dynamic_max_linear = speed_ceiling / slowdown

        if linear_velocity > 1e-3 and (is_curve_segment or abs(body_y) > 10.0):
            min_moving_speed = getattr(constants, 'CONTROLS_MIN_MOVING_SPEED_MM_S', 35.0)
            dynamic_max_linear = max(min_moving_speed, dynamic_max_linear)

        linear_velocity = _clamp(
            linear_velocity,
            -dynamic_max_linear,
            dynamic_max_linear,
        )
        angular_velocity = _clamp(
            angular_velocity,
            -constants.CONTROLS_MAX_ANGULAR_SPEED_RAD_S,
            constants.CONTROLS_MAX_ANGULAR_SPEED_RAD_S,
        )
        angular_velocity *= constants.CONTROLS_ANGULAR_COMMAND_SIGN

        left_velocity = linear_velocity - half_base * angular_velocity
        right_velocity = linear_velocity + half_base * angular_velocity
        speed_limit = constants.CONTROLS_CALIBRATED_SPEED_MM_S
        scale = min(1.0, speed_limit / max(abs(left_velocity), abs(right_velocity), 1e-9))
        left_velocity *= scale
        right_velocity *= scale
        self.last_wheel_velocities = (left_velocity, right_velocity)
        left_pwm = _velocity_to_pwm(left_velocity, constants.CONTROLS_WHEEL_LEFT_SIGN)
        right_pwm = _velocity_to_pwm(right_velocity, constants.CONTROLS_WHEEL_RIGHT_SIGN)
        left_pwm, right_pwm = _apply_pwm_safety_filter(left_pwm, right_pwm)
        return (left_pwm, right_pwm, constants.PALLET_ACTION_DONT_SEND)

    def _tracking_feedback(self, body_x, body_y, theta_err, tag):
        prefix = ('CONTROLS_PICKUP_ROBOT_PID_' if self._pickup_gains or int(tag) == TAG_PALLET
                  else 'CONTROLS_ROBOT_PID_')
        gain = lambda name: getattr(constants, prefix + name)
        previous_x, previous_y, _ = self._previous_tracking_error
        self._previous_tracking_error = (body_x, body_y, theta_err)
        return (
            gain('KPx') * body_x + gain('KDx') * (body_x - previous_x),
            gain('KPy') * body_y + gain('KDy') * (body_y - previous_y)
            + gain('KPtheta') * theta_err,
        )

    def tick(self, robot_pose, now):

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
                    self.reset_pids()

                return (PWM_NEUTRAL, PWM_NEUTRAL, constants.PALLET_ACTION_LIFT_UP)

            if self.state == STATE_WAITING_LIFT_DOWN:
                if now - self._lift_time >= LIFT_HOLD_TIME_S:
                    self.state = STATE_DONE
                return (PWM_NEUTRAL, PWM_NEUTRAL, constants.PALLET_ACTION_LIFT_DOWN)

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
                    self._hw_id_map[idx]   = hw_id
                    print(f"[Controller] Robot idx={idx} hw_id={hw_id}: {len(path)} waypoints nạp thành công.")
                else:
                    print(f"[Controller] Robot idx={idx} hw_id={hw_id}: không có path, bỏ qua.")

            self._loaded = True
            print(f"[Controller] Đã nạp {len(self._controllers)} robot(s).")
        except Exception as e:
            print(f"[ControllerMain][ERROR] Lỗi load_paths: {e}\n{traceback.format_exc()}")
            raise

    def run_one_tick(self, cv_system):

        try:
            if not self._loaded:
                return

            self._network.check_health()
            fid_ids = [constants.ROBOT_FIDUCIALS[constants.ROBOT_HARDWARE_NUMBERS.index(hw)] for hw in self._hw_id_map.values()]
            cv_system.assert_fresh(fid_ids)
            now = time.monotonic()
            robot_poses_mm, robot_fid_ids = cv_system.cv_GetRobotPositionsMM()

            for idx, ctrl in self._controllers.items():
                if ctrl.is_done:
                    if not self._network.stop_robot(self._hw_id_map[idx]):
                        raise RuntimeError('Completed robot heartbeat rejected')
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
                    raise RuntimeError(f"Missing robot pose: {hw_id}")

                result = ctrl.tick(pose, now)
                if result is None:
                    continue

                left_pwm, right_pwm, pallet_action = result
                if not self._network.send_command(hw_id, left_pwm, right_pwm, pallet_action):
                    raise RuntimeError(f"Command rejected for R{hw_id}")

                if constants.CONTROLS_DEBUG:
                    wp = ctrl.current_waypoint
                    print(f"[Controller] R{hw_id} state={ctrl.state} "
                          f"wp={ctrl.wp_idx}/{len(ctrl.waypoints)} "
                          f"pose=({pose[0]:.0f},{pose[1]:.0f},{math.degrees(pose[2]):.1f}deg) "
                          f"target=({wp[0]:.0f},{wp[1]:.0f}) "
                          f"CMD90=({left_pwm},{right_pwm}) action={pallet_action}")
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
            if not self._controllers:
                return False
            return all(ctrl.is_done for ctrl in self._controllers.values())
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

            target = [first[0] + u*dx, first[1] + u*dy, first[2], second[3], second[4]]
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
            m0 = ( math.cos(first[2])  * tangent_scale,
                  -math.sin(first[2])  * tangent_scale)
            m1 = ( math.cos(second[2]) * tangent_scale,
                  -math.sin(second[2]) * tangent_scale)
            h00 = 2*u**3 - 3*u**2 + 1
            h10 = u**3 - 2*u**2 + u
            h01 = -2*u**3 + 3*u**2
            h11 = u**3 - u**2
            target = [
                h00*first[0] + h10*m0[0] + h01*second[0] + h11*m1[0],
                h00*first[1] + h10*m0[1] + h01*second[1] + h11*m1[1],
                0.0, second[3], second[4],
            ]
            dh00 = 6*u**2 - 6*u
            dh10 = 3*u**2 - 4*u + 1
            dh01 = -dh00
            dh11 = 3*u**2 - 2*u
            dxdt = (dh00*first[0] + dh10*m0[0] + dh01*second[0] + dh11*m1[0]) / duration
            dydt = (dh00*first[1] + dh10*m0[1] + dh01*second[1] + dh11*m1[1]) / duration
            ddh00 = 12*u - 6
            ddh10 = 6*u - 4
            ddh01 = -ddh00
            ddh11 = 6*u - 2
            ddxdt = (ddh00*first[0] + ddh10*m0[0]
                     + ddh01*second[0] + ddh11*m1[0]) / (duration*duration)
            ddydt = (ddh00*first[1] + ddh10*m0[1]
                     + ddh01*second[1] + ddh11*m1[1]) / (duration*duration)
            target[2] = math.atan2(-dydt, dxdt)

    if len(waypoints) > 1 and (elapsed < first[3] or elapsed >= second[3]):
        endpoint = first if elapsed < first[3] else second
        target = list(endpoint)
        dxdt = dydt = ddxdt = ddydt = rotation_rate = 0.0

    return index, target, dxdt, dydt, ddxdt, ddydt, rotation_rate, is_curve_segment

def _velocity_to_pwm(velocity, wheel_sign):
    if abs(velocity) < 1e-6:
        return PWM_NEUTRAL

    fraction = min(abs(velocity) / constants.CONTROLS_CALIBRATED_SPEED_MM_S, 1.0)
    pwm_delta = 2.0 + fraction * (PWM_MAX_DELTA - 2.0)
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

    try:
        for pose, fid in zip(poses, fid_ids):
            if fid == target_fid:
                return pose
    except Exception as e:
        print(f"[_find_pose_by_fid][ERROR]: {e}\n{traceback.format_exc()}")
    return None
