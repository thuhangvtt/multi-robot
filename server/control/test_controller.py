
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

import constants
from controller_main import ControllerMain, _normalize_angle



class MockRobotNetwork:
    def __init__(self, verbose=False):
        self.verbose = verbose
        self.last_cmd = {}  
        self.cmd_count = 0

    def check_health(self):
        pass

    def send_command(self, robot_hw_id, left_pwm, right_pwm,
                     electromagnet=0, blocking=False):
        self.last_cmd[robot_hw_id] = (left_pwm, right_pwm, electromagnet)
        self.cmd_count += 1
        if self.verbose:
            print(f"  [Net] R{robot_hw_id} PWM=({left_pwm},{right_pwm}) magnet={electromagnet}")
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

    def set_targets(self, targets):
        self._targets = targets

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


def main():
    print("=" * 60)
    print("  Controller Module — Standalone Simulation Test")
    print("=" * 60)

    results = {
        "Mô phỏng toàn bộ nhiệm vụ": test_full_mission(),
        "Emergency stop":            test_emergency_stop(),
        "Path rỗng":                 test_empty_path(),
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
