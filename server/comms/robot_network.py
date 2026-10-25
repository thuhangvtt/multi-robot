import threading
import requests
import time
import os
import sys
import traceback

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'config'))
import constants

ROBOT_IP_MAP = {
    1: "192.168.100.101",
    2: "192.168.100.102",
    3: "192.168.100.103",
}

_CMD_ENDPOINT = "/cmd"
_HTTP_PORT = 80
_TIMEOUT_S = 0.25
_RETRY_COUNT = 1
PWM_NEUTRAL = 90


class RobotNetwork:
    """Quản lý giao tiếp HTTP đến tất cả robot ESP32."""

    def __init__(self, robot_ip_map=None, debug=True):
        """Khởi tạo mạng kết nối robot."""
        self._ip_map = robot_ip_map if robot_ip_map is not None else ROBOT_IP_MAP
        self._debug = debug
        self._lock = threading.Lock()
        self._stats = {hw_id: {"sent": 0, "failed": 0} for hw_id in self._ip_map}

    def send_command(
        self,
        robot_hw_id,
        left_pwm,
        right_pwm,
        pallet_action=constants.PALLET_ACTION_DONT_SEND,
        blocking=False,
    ):
        """Gửi lệnh điều khiển (PWM bánh xe và hành động servo) đến một robot."""
        try:
            if robot_hw_id not in self._ip_map:
                print(f"[RobotNetwork][ERROR] Robot hw_id={robot_hw_id} không tồn tại trong bảng ánh xạ IP.")
                return False

            left_pwm = max(50, min(130, int(left_pwm)))
            right_pwm = max(50, min(130, int(right_pwm)))
            payload = {"l": left_pwm, "r": right_pwm, "action": pallet_action}

            if blocking:
                return self._send(robot_hw_id, payload)
            else:
                t = threading.Thread(
                    target=self._send,
                    args=(robot_hw_id, payload),
                    daemon=True,
                )
                t.start()
                return True
        except Exception as e:
            print(f"[RobotNetwork][ERROR] Lỗi send_command cho Robot {robot_hw_id}: {e}\n{traceback.format_exc()}")
            return False

    def stop_robot(self, robot_hw_id, blocking=False):
        """Gửi lệnh dừng động cơ đến một robot."""
        try:
            return self.send_command(
                robot_hw_id,
                left_pwm=PWM_NEUTRAL,
                right_pwm=PWM_NEUTRAL,
                pallet_action=constants.PALLET_ACTION_DONT_SEND,
                blocking=blocking,
            )
        except Exception as e:
            print(f"[RobotNetwork][ERROR] Lỗi stop_robot R{robot_hw_id}: {e}\n{traceback.format_exc()}")
            return False

    def stop_all(self, blocking=True):
        """Dừng tất cả robot."""
        threads = []
        try:
            for hw_id in self._ip_map:
                t = threading.Thread(
                    target=self.stop_robot,
                    args=(hw_id, True),
                    daemon=True,
                )
                threads.append(t)
                t.start()
            if blocking:
                for t in threads:
                    t.join(timeout=1.0)
        except Exception as e:
            print(f"[RobotNetwork][ERROR] Lỗi stop_all: {e}\n{traceback.format_exc()}")

    def lift_pallet(self, robot_hw_id):
        """Nâng hoặc gắp pallet bằng servo trên xe."""
        try:
            return self.send_command(
                robot_hw_id,
                left_pwm=PWM_NEUTRAL,
                right_pwm=PWM_NEUTRAL,
                pallet_action=constants.PALLET_ACTION_LIFT_UP,
            )
        except Exception as e:
            print(f"[RobotNetwork][ERROR] Lỗi lift_pallet R{robot_hw_id}: {e}\n{traceback.format_exc()}")
            return False

    def lower_pallet(self, robot_hw_id):
        """Hạ hoặc nhả pallet bằng servo trên xe."""
        try:
            return self.send_command(
                robot_hw_id,
                left_pwm=PWM_NEUTRAL,
                right_pwm=PWM_NEUTRAL,
                pallet_action=constants.PALLET_ACTION_LIFT_DOWN,
            )
        except Exception as e:
            print(f"[RobotNetwork][ERROR] Lỗi lower_pallet R{robot_hw_id}: {e}\n{traceback.format_exc()}")
            return False

    def ping(self, robot_hw_id):
        """Kiểm tra trạng thái kết nối của robot."""
        if robot_hw_id not in self._ip_map:
            return False
        url = f"http://{self._ip_map[robot_hw_id]}:{_HTTP_PORT}/"
        try:
            r = requests.get(url, timeout=_TIMEOUT_S * 4)
            return r.status_code == 200
        except requests.exceptions.Timeout as e:
            if self._debug:
                print(f"[RobotNetwork][TimeoutException] Ping R{robot_hw_id} timed out: {e}")
            return False
        except requests.exceptions.ConnectionError as e:
            if self._debug:
                print(f"[RobotNetwork][SocketException] Ping R{robot_hw_id} connection refused: {e}")
            return False
        except Exception as e:
            print(f"[RobotNetwork][ERROR] Ping R{robot_hw_id} lỗi: {e}\n{traceback.format_exc()}")
            return False

    def ping_all(self):
        """Kiểm tra kết nối của toàn bộ robot."""
        results = {}
        threads = []

        def _ping_one(hw_id):
            try:
                results[hw_id] = self.ping(hw_id)
            except Exception as e:
                print(f"[RobotNetwork][ERROR] Lỗi trong ping_one R{hw_id}: {e}\n{traceback.format_exc()}")
                results[hw_id] = False

        try:
            for hw_id in self._ip_map:
                t = threading.Thread(target=_ping_one, args=(hw_id,), daemon=True)
                threads.append(t)
                t.start()
            for t in threads:
                t.join(timeout=2.0)
        except Exception as e:
            print(f"[RobotNetwork][ERROR] Lỗi ping_all: {e}\n{traceback.format_exc()}")
        return results

    def get_stats(self):
        """Lấy số lượng lệnh đã gửi và số lần lỗi theo từng robot."""
        try:
            with self._lock:
                return {k: dict(v) for k, v in self._stats.items()}
        except Exception as e:
            print(f"[RobotNetwork][ERROR] Lỗi get_stats: {e}\n{traceback.format_exc()}")
            return {}

    def _send(self, robot_hw_id, payload):
        """Thực hiện HTTP POST tới robot."""
        ip = self._ip_map[robot_hw_id]
        url = f"http://{ip}:{_HTTP_PORT}{_CMD_ENDPOINT}"

        for attempt in range(1 + _RETRY_COUNT):
            try:
                response = requests.post(url, json=payload, timeout=_TIMEOUT_S)
                if response.status_code == 200:
                    with self._lock:
                        self._stats[robot_hw_id]["sent"] += 1
                    if self._debug:
                        print(f"[RobotNetwork] R{robot_hw_id} <- {payload}  OK")
                    return True
            except requests.exceptions.Timeout as e:
                if self._debug:
                    print(f"[RobotNetwork][TimeoutException] R{robot_hw_id} timeout (attempt {attempt + 1}): {e}\n{traceback.format_exc()}")
            except requests.exceptions.ConnectionError as e:
                if self._debug:
                    print(f"[RobotNetwork][SocketException] R{robot_hw_id} connection error (attempt {attempt + 1}): {e}\n{traceback.format_exc()}")
            except Exception as exc:
                print(f"[RobotNetwork][ERROR] R{robot_hw_id} lỗi không xác định: {exc}\n{traceback.format_exc()}")
            finally:
                pass

        with self._lock:
            self._stats[robot_hw_id]["failed"] += 1
        return False
