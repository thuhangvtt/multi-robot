
import sys
import os
import math
import time

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


BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(BASE_DIR, 'config'))
sys.path.insert(0, os.path.join(BASE_DIR, 'control'))
sys.path.insert(0, os.path.join(BASE_DIR, 'comms'))

import constants
from controller_main import ControllerMain, _normalize_angle



class MockRobotNetwork:
    def __init__(self, verbose=False):
        self.verbose = verbose
        self.last_cmd = {}
        self.cmd_history = []
        self.cmd_count = 0

    def check_health(self):
        pass

    def get_rtt(self, hw):
        return 0.02

    def send_velocity(self, robot_hw_id, linear_mm_s, angular_deg_s, pallet_action=0, blocking=False):
        cmd = ("vw", float(linear_mm_s), float(angular_deg_s), int(pallet_action))
        self.last_cmd[robot_hw_id] = cmd
        self.cmd_history.append((robot_hw_id, cmd))
        self.cmd_count += 1
        if self.verbose:
            print(f"  [Net] R{robot_hw_id} VW=({linear_mm_s:.1f},{angular_deg_s:.1f}) action={pallet_action}")
        return True

    def send_command(self, robot_hw_id, left_pwm, right_pwm,
                     electromagnet=0, blocking=False):
        cmd = ("pwm", int(left_pwm), int(right_pwm), int(electromagnet))
        self.last_cmd[robot_hw_id] = cmd
        self.cmd_history.append((robot_hw_id, cmd))
        self.cmd_count += 1
        if self.verbose:
            print(f"  [Net] R{robot_hw_id} PWM=({left_pwm},{right_pwm}) action={electromagnet}")
        return True

    def stop_robot(self, robot_hw_id, blocking=False):
        return self.send_command(robot_hw_id, 90, 90)

    def stop_all(self, blocking=True):
        pass



class MockCV:
   
    STEP_MM        = 15.0   # mm mỗi tick
    STEP_THETA_RAD = 0.08   # rad mỗi tick

    def __init__(self, initial_poses, fid_map):
       
        self._poses   = {fid: list(p) for fid, p in initial_poses.items()}
        self._fid_map = fid_map
        self._targets = {}
        self._fresh_map = {fid: True for fid in initial_poses}

    def set_targets(self, targets):
        self._targets = targets

    def set_marker_fresh(self, fid, fresh):
        self._fresh_map[fid] = bool(fresh)

    def is_marker_fresh(self, fid, max_missed_frames=2, max_age_s=None, now=None):
        return self._fresh_map.get(fid, True)

    def last_captured_at(self):
        return time.monotonic()

    def assert_fresh(self, fid_ids=()):
        pass

    def cv_GetRobotPositionsMM(self):

        for fid, pose in self._poses.items():
            if fid in self._targets:
                tx, ty, ttheta = self._targets[fid]
                dx = tx - pose[0]
                dy = ty - pose[1]
                dist = math.sqrt(dx*dx + dy*dy)
                if dist > self.STEP_MM:
                    pose[0] += dx / dist * self.STEP_MM
                    pose[1] += dy / dist * self.STEP_MM
                else:
                    pose[0] = tx
                    pose[1] = ty
       
                dtheta = _normalize_angle(ttheta - pose[2])
                if abs(dtheta) > self.STEP_THETA_RAD:
                    pose[2] += math.copysign(self.STEP_THETA_RAD, dtheta)
                else:
                    pose[2] = ttheta

        poses   = list(self._poses.values())
        fid_ids = list(self._poses.keys())
        return poses, fid_ids



ROBOT_HW_IDS = [1, 2]

PATH_ROBOT_0 = [
    [300.0, 300.0, 0.0,    0.0,  0],
    [500.0, 350.0, 0.0,    1.0,  0],
    [700.0, 450.0, 0.0,    2.0,  0],
    [800.0, 500.0, 0.0,    3.0,  1],  
    [1000.0, 500.0, 0.0,   4.0,  0],
    [1200.0, 500.0, 0.0,   5.0,  0],
    [1500.0, 500.0, 0.0,   6.0,  2],   
]

PATH_ROBOT_1 = [
    [300.0,  1200.0, 0.0,  0.0,  0],
    [500.0,  1100.0, 0.0,  1.0,  0],
    [700.0,  1050.0, 0.0,  2.0,  0],
    [800.0,  1000.0, 0.0,  3.0,  1],   
    [1000.0, 1000.0, 0.0,  4.0,  0],
    [1300.0, 1000.0, 0.0,  5.0,  0],
    [1500.0, 1000.0, 0.0,  6.0,  2],   
]


def test_full_mission():
    print("\n[Test 1] Mô phỏng toàn bộ nhiệm vụ...")

    net  = MockRobotNetwork(verbose=False)
    ctrl = ControllerMain(net)

    paths = [PATH_ROBOT_0, PATH_ROBOT_1]
    ctrl.load_paths(paths, ROBOT_HW_IDS)

    initial_poses = {
        0: [300.0, 300.0, 0.0],
        3: [300.0, 1200.0, 0.0],
    }
    cv_mock = MockCV(initial_poses, fid_map={0: 0, 3: 3})

    MAX_TICKS = 2000
    tick = 0
    while not ctrl.all_done() and tick < MAX_TICKS:
        targets = {}
        for idx, robot_ctrl in ctrl._controllers.items():
            wp = robot_ctrl.current_waypoint
            if wp:
                hw_id = ctrl._hw_id_map[idx]
                fid = constants.ROBOT_FIDUCIALS[constants.ROBOT_HARDWARE_NUMBERS.index(hw_id)]
                targets[fid] = [wp[0], wp[1], wp[2]]
        cv_mock.set_targets(targets)

        ctrl.run_one_tick(cv_mock)
        time.sleep(0.005)
        tick += 1

    status = ctrl.get_status()
    all_done = ctrl.all_done()

    print(f"  Ticks: {tick}/{MAX_TICKS}")
    print(f"  Lệnh mạng đã gửi: {net.cmd_count}")
    for idx, s in status.items():
        ok = "DONE" if s["done"] else "NOT DONE"
        print(f"  Robot idx={idx} hw={s['hw_id']}: {ok}  "
              f"state={s['state']}  wp={s['wp_idx']}/{s['total_wp']}")

    if all_done:
        print("  OK: Tất cả robot hoàn thành nhiệm vụ!")
    else:
        print("  FAIL: Một số robot chưa hoàn thành trong giới hạn tick!")

    return all_done



def test_emergency_stop():
    print("\n[Test 2] Kiểm tra emergency stop...")

    net  = MockRobotNetwork(verbose=False)
    ctrl = ControllerMain(net)
    ctrl.load_paths([PATH_ROBOT_0], [1])

    # Chạy 5 tick rồi dừng khẩn cấp
    cv_mock = MockCV({0: [300.0, 300.0, 0.0]}, {})
    for _ in range(5):
        ctrl.run_one_tick(cv_mock)

    ctrl.emergency_stop()
    passed = ctrl.all_done()

    if passed:
        print("  OK: Emergency stop hoạt động đúng!")
    else:
        print("  FAIL: Emergency stop không đặt trạng thái DONE!")

    return passed


def test_empty_path():
    print("\n[Test 3] Kiểm tra path rỗng...")

    net  = MockRobotNetwork()
    ctrl = ControllerMain(net)
    ctrl.load_paths([[], []], [3, 1])

    done = ctrl.all_done()
    if done:
        print("  OK: Path rỗng -> all_done() = True ngay lập tức.")
    else:
        print("  FAIL: Path rỗng nhưng all_done() = False!")

    return done


def test_stale_marker_pauses_robot():
    print("\n[Test 4] Kiểm tra dừng ngay khi mất marker...")

    net = MockRobotNetwork()
    ctrl = ControllerMain(net)
    ctrl.load_paths([PATH_ROBOT_0], [1])
    cv_mock = MockCV({0: [300.0, 300.0, 0.0]}, {0: 0})

    # Normal tick sends velocity command
    ctrl.run_one_tick(cv_mock)
    first_cmd = net.last_cmd.get(1)
    assert first_cmd is not None and first_cmd[0] == "vw" and first_cmd[1] > 0.0

    # Lose marker -> must immediately send stop and mark paused_marker
    cv_mock.set_marker_fresh(0, False)
    ctrl.run_one_tick(cv_mock)
    paused_cmd = net.last_cmd.get(1)
    status = ctrl.get_status()[0]
    passed_pause = (paused_cmd == ("pwm", 90, 90, 0) and status["paused_marker"] is True)

    # Reacquire marker -> must resume velocity commands
    cv_mock.set_marker_fresh(0, True)
    ctrl.run_one_tick(cv_mock)
    resumed_cmd = net.last_cmd.get(1)
    status_resumed = ctrl.get_status()[0]
    passed_resume = (resumed_cmd[0] == "vw" and resumed_cmd[1] > 0.0 and status_resumed["paused_marker"] is False)

    ok = passed_pause and passed_resume
    print(f"  {'OK' if ok else 'FAIL'}: paused_cmd={paused_cmd}, resumed_cmd={resumed_cmd}")
    return ok


def test_curve_braking_profile():
    print("\n[Test 5] Kiểm tra giảm tốc trước khúc cua gấp...")

    sharp_bend_path = [
        [200.0, 300.0, 0.0, 0.0, 0],
        [450.0, 300.0, 0.0, 3.0, 0],
        [520.0, 260.0, math.pi / 2.0, 4.5, 0],
        [520.0, 100.0, math.pi / 2.0, 7.0, 2],
    ]
    net = MockRobotNetwork()
    ctrl = ControllerMain(net)
    ctrl.load_paths([sharp_bend_path], [1])
    robot_ctrl = ctrl._controllers[0]

    # Far on straight segment: should cruise faster than near the bend
    v_straight, _, _ = robot_ctrl.tick([220.0, 300.0, 0.0], now=1.0)
    # 35 mm before the apex of the 90-deg turn: must slow down significantly
    v_near_bend, w_near_bend, _ = robot_ctrl.tick([425.0, 300.0, 0.0], now=1.1)

    ok = (v_straight >= 60.0) and (v_near_bend < v_straight) and (v_near_bend <= 48.0) and (w_near_bend > 0.0)
    print(f"  {'OK' if ok else 'FAIL'}: v_straight={v_straight:.1f}mm/s, v_near_bend={v_near_bend:.1f}mm/s, w={w_near_bend:.1f}deg/s")
    return ok


def test_anti_stall_boost_and_release():
    print("\n[Test 6] Kiểm tra vòng kín chống kẹt v_real -> v_boost...")

    straight_path = [
        [200.0, 300.0, 0.0, 0.0, 0],
        [700.0, 300.0, 0.0, 8.0, 2],
    ]
    net = MockRobotNetwork()
    ctrl = ControllerMain(net)
    ctrl.load_paths([straight_path], [1])
    robot_ctrl = ctrl._controllers[0]

    # Simulate robot stalled at (220, 300) for 0.8s -> boost must ramp up safely (<= target + 65)
    t = 0.0
    v_stalled_cmds = []
    for _ in range(16):
        t += 0.05
        v_cmd, _, _ = robot_ctrl.tick([220.0, 300.0, 0.0], now=t)
        v_stalled_cmds.append(v_cmd)

    ramped_up = v_stalled_cmds[-1] > v_stalled_cmds[0] + 10.0 and v_stalled_cmds[-1] <= 65.0 + 65.0 + 1e-6

    # Simulate robot now rolling fast above target (e.g. 120 mm/s) -> boost must discharge rapidly
    x = 220.0
    v_after_move = []
    for _ in range(12):
        t += 0.05
        x += 6.0  # 120 mm/s
        v_cmd, _, _ = robot_ctrl.tick([x, 300.0, 0.0], now=t)
        v_after_move.append(v_cmd)

    discharged = v_after_move[-1] < v_stalled_cmds[-1] - 10.0
    ok = ramped_up and discharged
    print(f"  {'OK' if ok else 'FAIL'}: initial={v_stalled_cmds[0]:.1f}, stalled_peak={v_stalled_cmds[-1]:.1f}, after_rolling={v_after_move[-1]:.1f}")
    return ok


def test_direct_deeper_pallet_pickup():
    print("\n[Test 7] Kiểm tra vào thẳng pallet sâu 2cm, bỏ quay chỉnh & không lùi khi chạm pallet...")

    from controller_main import STATE_DRIVING_TO_PALLET, STATE_INSERTING_PALLET, STATE_WAITING_LIFT_UP
    path = [
        [200.0, 500.0, 0.0, 0.0, 0],
        [600.0, 500.0, 0.0, 6.0, 0],
        [750.0, 500.0, 0.0, 8.5, 0],
        [800.0, 500.0, 0.0, 9.5, 1],
        [1100.0, 500.0, 0.0, 14.0, 2],
    ]
    net = MockRobotNetwork()
    ctrl = ControllerMain(net)
    ctrl.load_paths([path], [1])
    r = ctrl._controllers[0]

    # Advance near pre-dock zone along runway
    r.tick([730.0, 500.0, 0.05], now=1.0)
    assert r.state == STATE_DRIVING_TO_PALLET
    # Within 45 mm pre-dock with moderate 15-deg angle: must jump directly to INSERTING_PALLET (no ALIGNING)
    r.tick([765.0, 500.0, 0.25], now=1.1)
    direct_insert = (r.state == STATE_INSERTING_PALLET)

    # Shifted live pallet (e.g. bumped 25mm / 25deg during insertion) must NOT abort/back off
    shifted_pallet = [825.0, 505.0, 0.45]
    v_ins, _, action_mid = r.tick([815.0, 500.0, 0.05], now=1.2, live_pallet_pose=shifted_pallet)
    still_inserting = (r.state == STATE_INSERTING_PALLET and v_ins > 0.0 and action_mid == 0)

    # At rem_forward == -20mm (earlier threshold -10mm): must STILL keep inserting deeper!
    r.tick([820.0, 500.0, 0.02], now=1.3, live_pallet_pose=shifted_pallet)
    not_yet_lifted = (r.state == STATE_INSERTING_PALLET)

    # At rem_forward == -31mm (past -30mm deeper threshold): must lift immediately!
    v_lift, w_lift, action_lift = r.tick([831.0, 500.0, 0.02], now=1.4, live_pallet_pose=shifted_pallet)
    lifted = (r.state == STATE_WAITING_LIFT_UP and v_lift == 0.0 and w_lift == 0.0 and action_lift == constants.PALLET_ACTION_LIFT_UP)

    ok = direct_insert and still_inserting and not_yet_lifted and lifted
    print(f"  {'OK' if ok else 'FAIL'}: direct={direct_insert}, no_abort={still_inserting}, deeper={not_yet_lifted}, lifted={lifted}")
    return ok


def test_lift_action_not_spammed_or_rejected():
    print("\n[Test 8] Kiểm tra không spam action=1/2 và gộp trùng trong RobotNetwork...")

    from collections import deque
    import threading
    from controller_main import STATE_INSERTING_PALLET, STATE_WAITING_LIFT_UP, STATE_WAITING_LIFT_DOWN, STATE_DONE
    from robot_network import RobotNetwork

    path = [
        [200.0, 500.0, 0.0, 0.0, 0],
        [750.0, 500.0, 0.0, 8.5, 0],
        [800.0, 500.0, 0.0, 9.5, 1],
        [1100.0, 500.0, 0.0, 14.0, 2],
    ]
    net = MockRobotNetwork()
    ctrl = ControllerMain(net)
    ctrl.load_paths([path], [1])
    r = ctrl._controllers[0]
    r.state = STATE_INSERTING_PALLET
    r._stage_enter_time = 1.0

    # Transition tick sends action=1 once
    _, _, act_enter = r.tick([831.0, 500.0, 0.0], now=1.2)
    # Subsequent hold ticks during 2.5s must send action=0
    hold_actions = [r.tick([831.0, 500.0, 0.0], now=1.2 + 0.05 * k)[2] for k in range(1, 20)]
    no_lift_spam = (r.state == STATE_WAITING_LIFT_UP and act_enter == 1 and all(a == 0 for a in hold_actions))

    # Same check for WAITING_LIFT_DOWN
    r.state = STATE_WAITING_LIFT_DOWN
    r._lift_time = 5.0
    lower_hold = [r.tick([1100.0, 500.0, 0.0], now=5.0 + 0.1 * k)[2] for k in range(1, 10)]
    no_lower_spam = all(a == 0 for a in lower_hold)

    # RobotNetwork must coalesce repeated identical actions without hitting len >= 8 overflow
    real_net = RobotNetwork(robot_ip_map={}, debug=False)
    real_net._states[1] = dict(
        motion=None, actions=deque(), failed=False,
        reconnecting=False, down_since=None,
        consecutive_failures=0, rtt_s=0.04,
        imu_yaw=None, imu_gz=None, last_error=None,
        engaged=False, io_lock=threading.Lock(),
        stats=dict(sent=0, failed=0, dropped=0, reconnects=0),
    )
    results = [real_net.send_velocity(1, 0.0, 0.0, pallet_action=1) for _ in range(20)]
    coalesced_ok = all(results) and len(real_net._states[1]['actions']) == 1 and real_net._states[1]['stats']['dropped'] == 19
    real_net.close()

    ok = no_lift_spam and no_lower_spam and coalesced_ok
    print(f"  {'OK' if ok else 'FAIL'}: no_lift_spam={no_lift_spam}, no_lower_spam={no_lower_spam}, coalesced={coalesced_ok}")
    return ok


def main():
    print("=" * 60)
    print("  Controller Module — Standalone Simulation Test")
    print("=" * 60)

    results = {
        "Mô phỏng toàn bộ nhiệm vụ": test_full_mission(),
        "Emergency stop":            test_emergency_stop(),
        "Path rỗng":                 test_empty_path(),
        "Dừng khi mất marker":       test_stale_marker_pauses_robot(),
        "Giảm tốc trước khúc cua":   test_curve_braking_profile(),
        "Chống kẹt vận tốc v_real":  test_anti_stall_boost_and_release(),
        "Gắp pallet sâu 2cm thẳng":  test_direct_deeper_pallet_pickup(),
        "Không tràn hàng đợi action": test_lift_action_not_spammed_or_rejected(),
    }

    print("\n" + "=" * 60)
    print("  KẾT QUẢ")
    print("=" * 60)
    all_passed = True
    for name, passed in results.items():
        icon = "PASS" if passed else "FAIL"
        print(f"  [{icon}]  {name}")
        if not passed:
            all_passed = False

    print("=" * 60)
    if all_passed:
        print("  Tất cả kiểm thử DAT — Controller san sang!")
    else:
        print("  Mot so kiem thu THAT BAI — xem chi tiet.")
    print("=" * 60)

    sys.exit(0 if all_passed else 1)


if __name__ == "__main__":
    main()
