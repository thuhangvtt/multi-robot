import os
import sys
import traceback
import math

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

_PLANNER_DIR = os.path.dirname(os.path.abspath(__file__))
_BUILD_BIN_DIR = os.path.join(_PLANNER_DIR, "build", "bin")

planner_core = None
_CORE_AVAILABLE = False

_candidates = [
    os.path.join(_BUILD_BIN_DIR, "planner_core.cp39-win_amd64.pyd"),
    os.path.join(_BUILD_BIN_DIR, "planner_core.pyd"),
    os.path.join(_PLANNER_DIR, "planner_core.cp39-win_amd64.pyd"),
    os.path.join(_PLANNER_DIR, "planner_core.pyd"),
]

for _cand in _candidates:
    if os.path.isfile(_cand):
        try:
            import importlib.util
            _spec = importlib.util.spec_from_file_location("planner_core", _cand)
            if _spec and _spec.loader:
                _mod = importlib.util.module_from_spec(_spec)
                sys.modules["planner_core"] = _mod
                _spec.loader.exec_module(_mod)
                planner_core = _mod
                _CORE_AVAILABLE = True
                break
        except Exception as _e:
            pass

if not _CORE_AVAILABLE:
    try:
        if _BUILD_BIN_DIR not in sys.path and os.path.isdir(_BUILD_BIN_DIR):
            sys.path.insert(0, _BUILD_BIN_DIR)
        import planner_core
        _CORE_AVAILABLE = True
    except ImportError as e:
        planner_core = None
        _CORE_AVAILABLE = False
        print(f"[PlannerService][WARN] planner_core chưa được biên dịch: {e}")

from path_postprocessor import precondition_path

class PlannerService:

    def __init__(self, num_robots: int):
        self.num_robots = num_robots
        self._coordinator = None

        try:
            if _CORE_AVAILABLE:
                self._coordinator = planner_core.PlannerCoordinator(num_robots)
                print(f"[PlannerService] Khởi tạo thành công cho {num_robots} robot(s).")
            else:
                print("[PlannerService] CẢNH BÁO: planner_core C++ extension chưa được biên dịch!")
        except Exception as e:
            print(f"[PlannerService][ERROR] Lỗi khởi tạo PlannerCoordinator: {e}\n{traceback.format_exc()}")

    @property
    def is_available(self) -> bool:

        return self._coordinator is not None

    def plan_trajectories(
        self,
        map_size: list,
        robot_poses: list,
        pallet_poses: list,
        goal_poses: list,
        obstacle_poses=None,
    ) -> list:

        try:
            if self._coordinator is None:
                raise RuntimeError(
                    "planner_core extension chưa được biên dịch. "
                    "Xem README.md trong server/planner/ để hướng dẫn build."
                )

            if len(pallet_poses) != len(goal_poses):
                raise ValueError(
                    f"Số pallet ({len(pallet_poses)}) phải bằng số goal ({len(goal_poses)})."
                )

            if len(map_size) != 2 or any(not isinstance(v, int) or v < 10 for v in map_size):
                raise ValueError('Map must contain two positive integer dimensions in mm')
            for poses in (robot_poses, pallet_poses, goal_poses, obstacle_poses or []):
                for pose in poses:
                    if len(pose) != 3 or not all(isinstance(v, (int, float)) and math.isfinite(v) for v in pose):
                        raise ValueError('Pose must be finite [x_mm, y_mm, theta_rad]')
                    if not (0 <= pose[0] < map_size[0] and 0 <= pose[1] < map_size[1]):
                        raise ValueError('Pose outside fixed field bounds')

            if len(robot_poses) == 0:
                print("[PlannerService] Không có robot — trả về danh sách rỗng.")
                return []

            if len(pallet_poses) == 0:
                print("[PlannerService] Không có pallet — không có việc để lập kế hoạch.")
                return [[] for _ in range(self.num_robots)]

            raw_paths = self._coordinator.generate_paths(
                map_size, robot_poses, pallet_poses, goal_poses, obstacle_poses or []
            )
            self.last_assignments = list(self._coordinator.get_task_assignments())
            if not any(raw_paths):
                raise RuntimeError("NO_PATH: tasks remain but no robot has a path")

            processed_paths = []
            for i, path in enumerate(raw_paths):
                if path:
                    path_copy = [list(wp) for wp in path]
                    processed = precondition_path(path_copy)

                    if processed and (abs(processed[0][0] - path[0][0]) > 1e-3 or abs(processed[0][1] - path[0][1]) > 1e-3):
                        processed.insert(0, list(path[0]))
                    processed_paths.append(processed)
                else:
                    processed_paths.append([])

            return processed_paths
        except ValueError as e:
            print(f"[PlannerService][ValueError] Lỗi dữ liệu đầu vào: {e}\n{traceback.format_exc()}")
            raise
        except Exception as e:
            print(f"[PlannerService][ERROR] Lỗi trong quá trình lập kế hoạch: {e}\n{traceback.format_exc()}")
            raise
