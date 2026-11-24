import socket
import threading
import time
from collections import deque

import requests

ROBOT_IP_MAP = {1: "192.168.100.101", 2: "192.168.100.102", 3: "192.168.100.103"}
PWM_NEUTRAL = 90

class RobotNetwork:
    def __init__(self, robot_ip_map=None, debug=True, session_factory=requests.Session, reconnect_grace_s=6.0):
        self._ip_map = dict(ROBOT_IP_MAP if robot_ip_map is None else robot_ip_map)
        self._debug = debug
        self._factory = session_factory
        self._reconnect_grace_s = float(reconnect_grace_s)
        self._cv = threading.Condition()
        self._closed = False
        self._halted = False
        self._states = {}
        self._threads = []
        for hw in self._ip_map:
            self._states[hw] = dict(motion=None, actions=deque(), failed=False,
                                    reconnecting=False, down_since=None,
                                    consecutive_failures=0, rtt_s=0.04,
                                    imu_yaw=None, imu_gz=None,
                                    last_error=None, engaged=False, io_lock=threading.Lock(),
                                    stats=dict(sent=0, failed=0, dropped=0, reconnects=0))
            worker = threading.Thread(target=self._worker, args=(hw,), daemon=True)
            self._threads.append(worker)
            worker.start()

    def _url(self, hw, endpoint):
        return f"http://{self._ip_map[hw]}{endpoint}"

    def _candidate_ips(self, hw):
        current = self._ip_map[hw]
        candidates = [current]
        host_octet = 100 + hw
        hotspot_cand = f"192.168.137.{host_octet}"
        if hotspot_cand not in candidates:
            candidates.append(hotspot_cand)
        try:
            for addr in socket.gethostbyname_ex(socket.gethostname())[2]:
                parts = addr.split('.')
                if len(parts) == 4 and not addr.startswith('127.'):
                    cand = f"{parts[0]}.{parts[1]}.{parts[2]}.{host_octet}"
                    if cand not in candidates:
                        candidates.append(cand)
        except OSError:
            pass
        return candidates

    @staticmethod
    def _check_response(response, endpoint):
        try:
            response.raise_for_status()
        except requests.HTTPError as exc:
            raise requests.HTTPError(
                f'{endpoint} HTTP {response.status_code}: {response.text[:300]}', response=response) from exc
        if response.status_code != 200:
            raise RuntimeError(f'{endpoint} expected HTTP 200, got {response.status_code}')

    def _enqueue_job(self, robot_hw_id, payload, pallet_action, blocking):
        job = dict(payload=payload, event=threading.Event(), ok=False)
        with self._cv:
            if self._closed or self._halted or robot_hw_id not in self._states:
                return False
            state = self._states[robot_hw_id]
            if state['failed']:
                return False
            state['engaged'] = True
            if pallet_action:
                if state['actions'] and state['actions'][-1]['payload'].get('action') == pallet_action:
                    old = state['actions'].pop()
                    old['event'].set()
                    state['stats']['dropped'] += 1
                if len(state['actions']) >= 8:
                    return False
                if state['motion']:
                    state['motion']['event'].set()
                    state['motion'] = None
                state['actions'].append(job)
            else:
                if state['motion']:
                    state['motion']['event'].set()
                    state['stats']['dropped'] += 1
                state['motion'] = job
            self._cv.notify_all()

        return job['event'].wait(2.5) and job['ok'] if blocking else True

    def send_command(self, robot_hw_id, left_pwm, right_pwm, pallet_action=0, blocking=False):
        if pallet_action not in (0, 1, 2):
            raise ValueError('Invalid pallet action')
        payload = dict(
            l=max(50, min(130, int(left_pwm))),
            r=max(50, min(130, int(right_pwm))),
            action=pallet_action,
        )
        return self._enqueue_job(robot_hw_id, payload, pallet_action, blocking)

    def send_velocity(self, robot_hw_id, linear_mm_s, angular_deg_s, pallet_action=0, blocking=False):
        if pallet_action not in (0, 1, 2):
            raise ValueError('Invalid pallet action')
        payload = dict(
            v=int(round(max(-280.0, min(280.0, float(linear_mm_s))))),
            w=int(round(max(-180.0, min(180.0, float(angular_deg_s))))),
            action=pallet_action,
        )
        return self._enqueue_job(robot_hw_id, payload, pallet_action, blocking)

    def _worker(self, hw):
        state = self._states[hw]
        session = self._factory()
        if hasattr(session, 'headers') and hasattr(session.headers, 'update'):
            session.headers.update({'Connection': 'Keep-Alive', 'Keep-Alive': 'timeout=5, max=10000000000'})
        try:
            while True:
                with self._cv:
                    self._cv.wait_for(lambda: self._closed or state['actions'] or state['motion'])
                    if self._closed:
                        return
                    job = state['actions'].popleft() if state['actions'] else state['motion']
                    if job is state['motion']:
                        state['motion'] = None
                try:
                    with state['io_lock']:
                        if self._halted or state['failed']:
                            continue
                        last_exc = None
                        for attempt in range(3):
                            if self._halted:
                                break
                            t_req = time.monotonic()
                            try:
                                response = session.post(self._url(hw, '/cmd'),
                                                        data=job['payload'], timeout=0.55)
                                self._check_response(response, '/cmd')
                                rtt = max(0.0, time.monotonic() - t_req)
                                state['rtt_s'] = 0.7 * state['rtt_s'] + 0.3 * rtt
                                self._parse_imu_reply(state, response)
                                job['ok'] = True
                                break
                            except requests.RequestException as exc:
                                last_exc = exc
                                if attempt == 0:
                                    for ip in self._candidate_ips(hw)[1:]:
                                        if self._halted:
                                            break
                                        t_cand = time.monotonic()
                                        try:
                                            response = session.post(f"http://{ip}/cmd",
                                                                    data=job['payload'], timeout=0.55)
                                            self._check_response(response, '/cmd')
                                            rtt = max(0.0, time.monotonic() - t_cand)
                                            state['rtt_s'] = 0.7 * state['rtt_s'] + 0.3 * rtt
                                            self._parse_imu_reply(state, response)
                                            self._ip_map[hw] = ip
                                            job['ok'] = True
                                            print(f'[NET] R{hw} resolved to {ip}')
                                            break
                                        except requests.RequestException as cand_exc:
                                            last_exc = cand_exc
                                    if job['ok']:
                                        break
                                if attempt >= 2 and last_exc is not None:
                                    raise last_exc
                        if job['ok']:
                            if state['reconnecting']:
                                state['stats']['reconnects'] += 1
                                print(f"[NET] R{hw} đã kết nối lại ({self._ip_map[hw]}, rtt={state['rtt_s']*1000:.0f}ms)")
                            state['stats']['sent'] += 1
                            state['reconnecting'] = False
                            state['down_since'] = None
                            state['consecutive_failures'] = 0
                except Exception as exc:
                    now = time.monotonic()
                    state['stats']['failed'] += 1
                    state['consecutive_failures'] += 1
                    state['last_error'] = str(exc)
                    if state['down_since'] is None:
                        state['down_since'] = now
                    outage = now - state['down_since']
                    if outage >= self._reconnect_grace_s:
                        state['failed'] = True
                        print(f'[HTTP ERROR] R{hw} ({self._ip_map[hw]}) mất kết nối {outage:.1f}s: {exc}')
                    else:
                        if not state['reconnecting']:
                            print(f'[NET WARN] R{hw} ({self._ip_map[hw]}) tạm mất kết nối, đang thử nối lại...: {exc}')
                        state['reconnecting'] = True
                        time.sleep(0.08)
                finally:
                    job['event'].set()
        finally:
            session.close()

    def check_health(self):
        failed = [hw for hw, state in self._states.items() if state['failed']]
        if failed:
            raise RuntimeError('Robot HTTP request failed: ' + '; '.join(
                f"R{hw}: {self._states[hw]['last_error']}" for hw in failed))

    def last_error(self, hw):
        return self._states[hw]['last_error'] or 'HTTP request did not complete'

    def stop_robot(self, robot_hw_id, blocking=False):
        return self.send_command(robot_hw_id, 90, 90, blocking=blocking)

    def lift_pallet(self, robot_hw_id):
        return self.send_command(robot_hw_id, 90, 90, 1)

    def lower_pallet(self, robot_hw_id):
        return self.send_command(robot_hw_id, 90, 90, 2)

    def stop_all(self, blocking=True):
        with self._cv:
            self._halted = True
            for state in self._states.values():
                jobs = list(state['actions']) + ([state['motion']] if state['motion'] else [])
                state['actions'].clear()
                state['motion'] = None
                for job in jobs:
                    job['event'].set()
        def stop(hw):
            try:
                with self._states[hw]['io_lock']:
                    for ip in self._candidate_ips(hw):
                        try:
                            response = requests.get(f"http://{ip}/stop", timeout=0.5)
                            self._check_response(response, '/stop')
                            self._ip_map[hw] = ip
                            return
                        except Exception as exc:
                            last_exc = exc
                    raise last_exc
            except Exception as exc:
                print(f'[STOP] R{hw}: HTTP stop failed ({exc}); watchdog remains fallback')
        threads = [threading.Thread(target=stop, args=(hw,), daemon=True)
                   for hw, state in self._states.items() if state['engaged']]
        for worker in threads:
            worker.start()
        if blocking:
            for worker in threads:
                worker.join(timeout=2.0)

    def ping(self, hw):

        try:
            response = requests.get(self._url(hw, '/'), timeout=0.5)
            self._check_response(response, '/')
            return True
        except Exception as exc:
            print(f'[HTTP] R{hw}: {exc}')
            return False

    def ping_all(self):
        return {hw: self.ping(hw) for hw in self._ip_map}

    def get_rtt(self, hw):
        state = self._states.get(hw)
        return float(state['rtt_s']) if state is not None else 0.04

    @staticmethod
    def _parse_imu_reply(state, response):
        text = getattr(response, 'text', '')
        if not text or not text.lstrip().startswith('{'):
            return
        try:
            data = response.json()
            if 'yaw' in data:
                state['imu_yaw'] = float(data['yaw'])
            if 'gz' in data:
                state['imu_gz'] = float(data['gz'])
        except (ValueError, TypeError):
            pass

    def get_stats(self):
        with self._cv:
            out = {}
            for hw, state in self._states.items():
                s = dict(state['stats'])
                s['rtt_ms'] = round(state['rtt_s'] * 1000.0, 1)
                s['reconnecting'] = bool(state['reconnecting'])
                if state['imu_yaw'] is not None:
                    s['imu_yaw'] = state['imu_yaw']
                if state['imu_gz'] is not None:
                    s['imu_gz'] = state['imu_gz']
                out[hw] = s
            return out

    def close(self):
        with self._cv:
            self._closed = True
            self._cv.notify_all()
        for worker in self._threads:
            worker.join(timeout=1.5)
