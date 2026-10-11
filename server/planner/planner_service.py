"""
planner_service.py
==================
Python Facade / Adapter Layer cho module planner.

Cung cấp class PlannerService — interface cấp cao duy nhất mà server/main_system.py
cần biết. Ẩn đi toàn bộ chi tiết C++ extension, chuyển đổi đơn vị, và hậu xử lý.

SỬA LỖI:
    - Không có os.system("make") hay os.system("cd build && make").
      Việc build C++ phải thực hiện 1 lần trước khi chạy server (xem README.md).
    - Không có biến toàn cục Planner* — vòng đời PlannerCoordinator gắn với
      instance PlannerService (tự động thu hồi bởi Python GC).
    - Import an toàn: nếu planner_core chưa được biên dịch, in cảnh báo rõ ràng
      thay vì crash im lặng.

Cách dùng từ main_system.py:
    from planner.planner_service import PlannerService

    planner = PlannerService(num_robots=3)
    paths = planner.plan_trajectories(
        map_size=[2000, 1500],          # [width_mm, height_mm]
        robot_poses=[[x,y,t], ...],     # mm, radians
        pallet_poses=[[x,y,t], ...],    # mm
        goal_poses=[[x,y,t], ...]       # mm
    )
    # paths[i] = list of [x(mm), y(mm), theta(rad), time(s), tag] for robot i
"""

import os
import sys

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

# Thêm thư mục chứa planner_core.pyd/.so vào sys.path
_PLANNER_DIR = os.path.dirname(os.path.abspath(__file__))
if _PLANNER_DIR not in sys.path:
    sys.path.insert(0, _PLANNER_DIR)

# Import C++ extension — không crash nếu chưa biên dịch
try:
    import planner_core
    _CORE_AVAILABLE = True
except ImportError:
    planner_core = None
    _CORE_AVAILABLE = False

from path_postprocessor import precondition_path


class PlannerService:
    """
    Python Facade cho C++ PlannerCoordinator.

    Trách nhiệm:
        1. Kiểm tra dữ liệu đầu vào.
        2. Gọi C++ generate_paths() — nhận về raw waypoints.
        3. Gọi precondition_path() lọc collinear waypoints.
        4. Trả về danh sách quỹ đạo sạch cho Controller.
    """

    def __init__(self, num_robots: int):
        """
        Parameters
        ----------
        num_robots : int
            Số lượng robot tham gia hệ thống (tối đa 3).
        """
        self.num_robots = num_robots
        self._coordinator = None

        if _CORE_AVAILABLE:
            self._coordinator = planner_core.PlannerCoordinator(num_robots)
            print(f"[PlannerService] Khởi tạo thành công cho {num_robots} robot(s).")
        else:
            print("[PlannerService] ⚠️  CẢNH BÁO: planner_core C++ extension chưa được biên dịch!")
            print("[PlannerService]    Chạy lệnh sau để build:")
            print(f"[PlannerService]    cd {_PLANNER_DIR}")
            print("[PlannerService]    cmake -B build && cmake --build build --config Release")

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
        """
        Lập kế hoạch đường đi cho tất cả robot.

        Parameters
        ----------
        map_size : list
            [width_mm, height_mm] — kích thước sandbox.
        robot_poses : list
            [[x, y, theta], ...] — vị trí robot (mm, radians).
        pallet_poses : list
            [[x, y, theta], ...] — vị trí pallet (mm, radians).
        goal_poses : list
            [[x, y, theta], ...] — vị trí điểm đặt (mm, radians).
            Phải cùng số lượng và thứ tự với pallet_poses.

        Returns
        -------
        list
            [robot_0_path, robot_1_path, ...]
            Mỗi path là [[x(mm), y(mm), theta(rad), time(s), tag], ...]
            tag: 0=đi, 1=gắp pallet, 2=trả pallet.
            Path rỗng nếu robot không được gán việc hoặc lập kế hoạch thất bại.

        Raises
        ------
        RuntimeError
            Nếu planner_core chưa được biên dịch.
        ValueError
            Nếu số lượng pallet và goal không khớp.
        """
        if self._coordinator is None:
            raise RuntimeError(
                "planner_core extension chưa được biên dịch. "
                "Xem README.md trong server/planner/ để hướng dẫn build."
            )

        if len(pallet_poses) != len(goal_poses):
            raise ValueError(
                f"Số pallet ({len(pallet_poses)}) phải bằng số goal ({len(goal_poses)}). "
                "Hệ thống ghép pallet[i] với goal[i] theo chỉ số."
            )

        if len(robot_poses) == 0:
            print("[PlannerService] Không có robot — trả về danh sách rỗng.")
            return []

        if len(pallet_poses) == 0:
            print("[PlannerService] Không có pallet — không có việc để lập kế hoạch.")
            return [[] for _ in range(self.num_robots)]

        # Gọi C++ core: generate_paths = set_map + set_poses + plan + get_paths
        raw_paths = self._coordinator.generate_paths(
            map_size, robot_poses, pallet_poses, goal_poses
        )

        # Hậu xử lý: lọc waypoint thẳng hàng trên bản sao của mỗi path
        processed_paths = []
        for i, path in enumerate(raw_paths):
            if path:
                # precondition_path chỉnh sửa in-place — dùng bản sao để an toàn
                path_copy = [list(wp) for wp in path]
                processed_paths.append(precondition_path(path_copy))
            else:
                processed_paths.append([])

        return processed_paths
