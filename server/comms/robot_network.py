import threading
from collections import deque

import requests

ROBOT_IP_MAP = {1: "192.168.100.101", 2: "192.168.100.102", 3: "192.168.100.103"}
PWM_NEUTRAL = 90

class RobotNetwork:
    def __init__(self, robot_ip_map=None, debug=True, session_factory=requests.Session):
        self._ip_map = dict(ROBOT_IP_MAP if robot_ip_map is None else robot_ip_map)
        self._debug = debug
        self._factory = session_factory
        self._cv = threading.Condition()
        self._closed = False
        self._halted = False
        self._states = {}
        self._threads = []
        for hw in self._ip_map:
            self._states[hw] = dict(motion=None, actions=deque(), failed=False,
                                    last_error=None, engaged=False, io_lock=threading.Lock(),
                                    stats=dict(sent=0, failed=0, dropped=0))
            worker = threading.Thread(target=self._worker, args=(hw,), daemon=True)
            self._threads.append(worker)
            worker.start()

    def _url(self, hw, endpoint):
        return f"http://{self._ip_map[hw]}{endpoint}"

    @staticmethod
    def _check_response(response, endpoint):
        try:
            response.raise_for_status()
        except requests.HTTPError as exc:
            raise requests.HTTPError(
                f'{endpoint} HTTP {response.status_code}: {response.text[:300]}', response=response) from exc
        if response.status_code != 200:
            raise RuntimeError(f'{endpoint} expected HTTP 200, got {response.status_code}')

    def send_command(self, robot_hw_id, left_pwm, right_pwm, pallet_action=0, blocking=False):
        if pallet_action not in (0, 1, 2):
            raise ValueError('Invalid pallet action')
        job = dict(payload=dict(l=max(50, min(130, int(left_pwm))),
                                r=max(50, min(130, int(right_pwm))), action=pallet_action),
                   event=threading.Event(), ok=False)
        with self._cv:
            if self._closed or self._halted or robot_hw_id not in self._states:
                return False
            state = self._states[robot_hw_id]
            if state['failed']:
                return False
            state['engaged'] = True
            if pallet_action:
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
                        for attempt in range(2):
                            if self._halted:
                                break
                            try:
                                response = session.post(self._url(hw, '/cmd'),
                                                        data=job['payload'], timeout=0.35)
                                self._check_response(response, '/cmd')
                                job['ok'] = True
                                break
                            except requests.RequestException:
                                if attempt:
                                    raise
                        if job['ok']:
                            state['stats']['sent'] += 1
                except Exception as exc:
                    state['stats']['failed'] += 1
                    state['failed'] = True
                    state['last_error'] = str(exc)
                    print(f'[HTTP ERROR] R{hw} ({self._ip_map[hw]}): {exc}')
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
                    response = requests.get(self._url(hw, '/stop'), timeout=0.5)
                    self._check_response(response, '/stop')
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

    def get_stats(self):
        with self._cv:
            return {hw: dict(state['stats']) for hw, state in self._states.items()}

    def close(self):
        with self._cv:
            self._closed = True
            self._cv.notify_all()
        for worker in self._threads:
            worker.join(timeout=1.5)
