import os
import sys
import traceback

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
    """Python Adapter cho C++ PlannerCoordinator."""

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
        """Trả về True nếu C++ extension đã được load thành công."""
        return self._coordinator is not None

    def plan_trajectories(
        self,
        map_size: list,
        robot_poses: list,
        pallet_poses: list,
        goal_poses: list
    ) -> list:
        """Lập kế hoạch đường đi cho tất cả robot."""
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

            if len(robot_poses) == 0:
                print("[PlannerService] Không có robot — trả về danh sách rỗng.")
                return []

            if len(pallet_poses) == 0:
                print("[PlannerService] Không có pallet — không có việc để lập kế hoạch.")
                return [[] for _ in range(self.num_robots)]

            raw_paths = self._coordinator.generate_paths(
                map_size, robot_poses, pallet_poses, goal_poses
            )

            processed_paths = []
            for i, path in enumerate(raw_paths):
                if path:
                    path_copy = [list(wp) for wp in path]
                    processed_paths.append(precondition_path(path_copy))
                else:
                    processed_paths.append([])

            return processed_paths
        except ValueError as e:
            print(f"[PlannerService][ValueError] Lỗi dữ liệu đầu vào: {e}\n{traceback.format_exc()}")
            raise
        except Exception as e:
            print(f"[PlannerService][ERROR] Lỗi trong quá trình lập kế hoạch: {e}\n{traceback.format_exc()}")
            return [[] for _ in range(self.num_robots)]
