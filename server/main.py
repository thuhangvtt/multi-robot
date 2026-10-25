import sys
import os
import time
import traceback
import cv2 as cv

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(BASE_DIR, 'config'))
sys.path.insert(0, os.path.join(BASE_DIR, 'vision'))
sys.path.insert(0, os.path.join(BASE_DIR, 'planner'))
sys.path.insert(0, os.path.join(BASE_DIR, 'control'))
sys.path.insert(0, os.path.join(BASE_DIR, 'comms'))

if sys.stdout and hasattr(sys.stdout, 'reconfigure'):
    try:
        sys.stdout.reconfigure(encoding='utf-8', errors='replace')
    except Exception:
        pass

import constants
from cv_main import CV
from cv_visualize import CVVisualizer
from planner_service import PlannerService
from controller_main import ControllerMain
from robot_network import RobotNetwork

REPLAN_TIMEOUT_S      = 30.0
REPLAN_COOLDOWN_S     = 10.0
CONTROLLER_TICK_EVERY = 1


class SystemState:
    INIT       = "INIT"
    PLANNING   = "PLANNING"
    RUNNING    = "RUNNING"
    CYCLE_DONE = "CYCLE_DONE"
    ERROR      = "ERROR"


def main():
    """Vòng lặp chính điều phối toàn bộ hệ thống: Vision -> Planner -> Controller -> Comms."""
    print("=" * 58)
    print("  PARROT Multi-Robot System — Main Loop")
    print("=" * 58)

    try:
        network    = RobotNetwork(debug=False)
        cv_system  = CV()
        visualizer = CVVisualizer()
        planner    = PlannerService(num_robots=len(constants.ROBOT_HARDWARE_NUMBERS))
        controller = ControllerMain(network)
    except Exception as e:
        print(f"[Main] FATAL: Khởi tạo hệ thống thất bại: {e}")
        print(traceback.format_exc())
        sys.exit(1)

    if not planner.is_available:
        print("[Main] FATAL: planner_core chưa được biên dịch.")
        print("       Chạy: cd server/planner && cmake -B build && cmake --build build --config Release")
        sys.exit(1)

    try:
        cv_system.start_camera(constants.WEBCAM_ID)
        print("[Main] Dang tim ArUco corner markers de canh chinh sandbox...")
        cv_system.cv_InitComputerVision()
    except Exception as e:
        print(f"[Main] FATAL: Khởi tạo camera/vision thất bại: {e}")
        print(traceback.format_exc())
        sys.exit(1)

    print("\n[Main] Kiem tra ket noi robot...")
    try:
        online = network.ping_all()
        for hw_id, is_online in online.items():
            status = "ONLINE" if is_online else "OFFLINE"
            print(f"  Robot hw_id={hw_id}: {status}")
    except Exception as e:
        print(f"[Main] WARNING: Ping robot thất bại: {e}")
        print(traceback.format_exc())

    state         = SystemState.INIT
    last_replan_t = 0.0
    cycle_count   = 0

    print("\n[Main] Bat dau vong lap chinh. Nhan 'q' hoac ESC de dung.\n")

    try:
        while True:
            loop_start = time.time()

            try:
                cv_system.cv_runLocalizer()
                visualizer.visualize(cv_system)
            except Exception as e:
                print(f"[Main] LỖI: Vision/Visualizer: {e}")
                print(traceback.format_exc())
                time.sleep(0.5)
                continue

            key = cv.waitKey(1) & 0xFF
            if key == ord('q') or key == 27:
                print("\n[Main] Người dùng yêu cầu dừng.")
                break
            if key == ord('e'):
                print("\n[Main] EMERGENCY STOP!")
                controller.emergency_stop()
                state = SystemState.ERROR
                break

            now = time.time()

            if state == SystemState.INIT or state == SystemState.CYCLE_DONE:
                try:
                    pallet_poses_mm = cv_system.cv_GetPalletPositionsMM()
                    goal_poses      = cv_system.cv_GetGoalPositions()
                except Exception as e:
                    print(f"[Main] LỖI: Lấy vị trí pallet/goal thất bại: {e}")
                    print(traceback.format_exc())
                    time.sleep(0.5)
                    continue

                if not pallet_poses_mm:
                    print("[Main] Chưa thấy pallet, chờ thêm...", end='\r')
                    time.sleep(0.5)
                    continue

                print(f"\n[Main] Chu kỳ {cycle_count + 1}: bắt đầu lập kế hoạch ({len(pallet_poses_mm)} pallet(s))...")
                state = SystemState.PLANNING

            if state == SystemState.PLANNING:
                if now - last_replan_t < REPLAN_COOLDOWN_S and last_replan_t > 0:
                    remaining = REPLAN_COOLDOWN_S - (now - last_replan_t)
                    print(f"[Main] Cooldown replan: con {remaining:.1f}s...", end='\r')
                    time.sleep(0.2)
                    continue

                try:
                    robot_poses_mm, robot_fid_ids = cv_system.cv_GetRobotPositionsMM()
                    pallet_poses_mm = cv_system.cv_GetPalletPositionsMM()
                    goal_poses_mm   = _get_goal_poses_mm(cv_system)
                    all_x = [p[0] for p in robot_poses_mm + pallet_poses_mm + goal_poses_mm] + [constants.CV_SANDBOX_WIDTH]
                    all_y = [p[1] for p in robot_poses_mm + pallet_poses_mm + goal_poses_mm] + [constants.CV_SANDBOX_HEIGHT]
                    map_size = [int(max(all_x) + 200), int(max(all_y) + 200)]
                except Exception as e:
                    print(f"[Main] LỖI: Lấy dữ liệu vision thất bại: {e}")
                    print(traceback.format_exc())
                    time.sleep(0.5)
                    continue

                if not robot_poses_mm:
                    print("[Main] Không thấy robot nào trên camera! Chờ...", end='\r')
                    time.sleep(0.5)
                    continue

                if len(pallet_poses_mm) == 0:
                    print("[Main] Hết pallet — chu kỳ hoàn thành!")
                    state = SystemState.CYCLE_DONE
                    cycle_count += 1
                    continue

                n_tasks = min(len(pallet_poses_mm), len(goal_poses_mm))
                if n_tasks == 0:
                    print("[Main] Không có goal position! Kiểm tra lại marker.", end='\r')
                    time.sleep(1.0)
                    continue

                use_pallets = pallet_poses_mm[:n_tasks]
                use_goals   = goal_poses_mm[:n_tasks]

                print(f"[Main] Đang lập kế hoạch cho {len(robot_poses_mm)} robot(s), {n_tasks} task(s)...")
                t0 = time.time()

                try:
                    paths = planner.plan_trajectories(
                        map_size    = [int(map_size[0]), int(map_size[1])],
                        robot_poses = robot_poses_mm,
                        pallet_poses = use_pallets,
                        goal_poses  = use_goals,
                    )
                except Exception as e:
                    print(f"[Main] LỖI: Planner thất bại: {e}")
                    print(traceback.format_exc())
                    state = SystemState.ERROR
                    time.sleep(2.0)
                    continue

                t_plan = time.time() - t0
                n_planned = sum(1 for p in paths if p)
                print(f"[Main] Kế hoạch xong: {n_planned}/{len(paths)} robot có quỹ đạo ({t_plan*1000:.0f} ms)")

                try:
                    hw_ids = _fid_ids_to_hw_ids(robot_fid_ids)
                    controller.load_paths(paths, hw_ids)
                    visualizer.set_paths(paths, robot_fid_ids)  # Vẽ planned path lên màn hình
                except Exception as e:
                    print(f"[Main] LỖI: Tải đường đi thất bại: {e}")
                    print(traceback.format_exc())
                    state = SystemState.ERROR
                    continue

                last_replan_t = now
                state = SystemState.RUNNING

            if state == SystemState.RUNNING:
                try:
                    controller.run_one_tick(cv_system)

                    status = controller.get_status()
                    n_done = sum(1 for s in status.values() if s["done"])
                    n_total = len(status)
                    print(f"[Main] Running: {n_done}/{n_total} robot done | net_cmds={network.get_stats()} ", end='\r')

                    if controller.all_done():
                        print(f"\n[Main] Tất cả robot hoàn thành chu kỳ {cycle_count + 1}!")
                        cycle_count += 1
                        state = SystemState.CYCLE_DONE

                    elif now - last_replan_t > REPLAN_TIMEOUT_S:
                        print(f"\n[Main] Timeout {REPLAN_TIMEOUT_S}s — replan...")
                        network.stop_all()
                        state = SystemState.PLANNING
                except Exception as e:
                    print(f"[Main] LỖI: Controller tick thất bại: {e}")
                    print(traceback.format_exc())
                    time.sleep(0.5)

            elapsed = time.time() - loop_start
            sleep_t = max(0.0, 0.033 - elapsed)
            if sleep_t > 0:
                time.sleep(sleep_t)

    except KeyboardInterrupt:
        print("\n[Main] Dừng bởi Ctrl+C.")

    finally:
        print("[Main] Dừng tất cả robot...")
        try:
            network.stop_all(blocking=True)
        except Exception as e:
            print(f"[Main] WARNING: stop_all thất bại: {e}")
            print(traceback.format_exc())
        try:
            cv_system.stop_camera()
        except Exception as e:
            print(f"[Main] WARNING: stop_camera thất bại: {e}")
            print(traceback.format_exc())
        cv.destroyAllWindows()

        try:
            stats = network.get_stats()
            print(f"[Main] Thống kê mạng: {stats}")
        except Exception:
            pass
        print(f"[Main] Tổng số chu kỳ hoàn thành: {cycle_count}")
        print("[Main] Hệ thống đã tắt.")


def _get_goal_poses_mm(cv_system):
    """Lấy danh sách goal pose theo đơn vị mm."""
    try:
        goal_px = cv_system.cv_GetGoalPositions()
        mm_per_pixel = getattr(cv_system.cv_fiducial, 'mm_per_pixel', None)
        if mm_per_pixel is None or mm_per_pixel == 0:
            return [[g[0], g[1], g[2]] for g in goal_px]
        return [[g[0] * mm_per_pixel, g[1] * mm_per_pixel, g[2]] for g in goal_px]
    except Exception as e:
        print(f"[Main] LỖI: _get_goal_poses_mm: {e}")
        print(traceback.format_exc())
        return []


def _fid_ids_to_hw_ids(fid_ids):
    """Chuyển danh sách fiducial ID thành hardware ID theo cấu hình constants."""
    try:
        fid_to_hw = {
            fid: hw
            for fid, hw in zip(constants.ROBOT_FIDUCIALS, constants.ROBOT_HARDWARE_NUMBERS)
        }
        result = []
        for fid in fid_ids:
            hw = fid_to_hw.get(fid)
            if hw is None:
                print(f"[Main] WARNING: fiducial ID {fid} không có trong bảng ánh xạ!")
                hw = fid
            result.append(hw)
        return result
    except Exception as e:
        print(f"[Main] LỖI: _fid_ids_to_hw_ids: {e}")
        print(traceback.format_exc())
        return list(fid_ids)


if __name__ == "__main__":
    main()
