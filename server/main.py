import json
import math
import os
from pathlib import Path
import sys
import time

BASE_DIR = Path(__file__).resolve().parent
for folder in ('config', 'vision', 'planner', 'control', 'comms'):
    sys.path.insert(0, str(BASE_DIR / folder))

import cv2 as cv
import constants
from cv_main import CV
from cv_visualize import CVVisualizer
from planner_service import PlannerService
from controller_main import ControllerMain
from robot_network import RobotNetwork
from task_registry import TaskRegistry

def _fid_ids_to_hw_ids(fid_ids):
    mapping = dict(zip(constants.ROBOT_FIDUCIALS, constants.ROBOT_HARDWARE_NUMBERS))
    if len(mapping) != len(constants.ROBOT_HARDWARE_NUMBERS):
        raise ValueError('Invalid hardware/fiducial mapping')
    return [mapping[fid] for fid in fid_ids]

def _get_pallet_dock_pose(pallet_pose):

    px, py, p_theta = pallet_pose
    offset = getattr(constants, 'CV_PALLET_OFFSET', 0.0)
    angle_offset = getattr(constants, 'CV_PALLET_APPROACH_ANGLE_OFFSET', 0.0)
    approach_theta = (p_theta + angle_offset) % (2 * math.pi)
    dock_x = px - offset * math.cos(approach_theta)
    dock_y = py + offset * math.sin(approach_theta)
    return [dock_x, dock_y, approach_theta]

def main():

    registry = TaskRegistry(None, constants.PALLET_FIDUCIALS, constants.GOAL_FIDUCIALS)
    network = RobotNetwork(debug=False)
    vision = CV()
    controller = ControllerMain(network)
    visualizer = CVVisualizer()
    log_file = None
    active = []
    state = 'PLANNING'
    plan_id = 0
    known_robots = set()
    last_wait_message = 0.0
    try:
        planner = PlannerService(num_robots=len(constants.ROBOT_HARDWARE_NUMBERS))
        if not planner.is_available:
            raise RuntimeError('Build planner_core before starting')
        vision.start_camera(constants.WEBCAM_ID)
        vision.cv_InitComputerVision()
        runtime = BASE_DIR / 'runtime'
        runtime.mkdir(exist_ok=True)
        log_file = (runtime / f'run-{time.time_ns()}.jsonl').open('w', encoding='utf-8')
        print('q/ESC: stop; e: stop.')
        while True:
            fresh_frame = vision.cv_runLocalizer()
            vision.assert_fresh()
            visualizer.visualize(vision)
            key = cv.waitKey(1) & 0xff
            if key in (ord('q'), ord('e'), 27):
                break
            if not fresh_frame:
                time.sleep(0.005)
                continue
            network.check_health()
            markers = vision.cv_GetMarkerPositionsMM()
            robots = [fid for fid in constants.ROBOT_FIDUCIALS if fid in markers]
            if known_robots:
                vision.assert_fresh(known_robots)
                if state == 'RUNNING' and set(robots) - known_robots:
                    raise RuntimeError('New robot entered an executing plan; stop before replanning')
            if state == 'PLANNING':
                tasks = registry.pending(markers)
                if registry.all_delivered():
                    print('All tasks observed in this run completed by controller; finished.')
                    break
                if not tasks or not robots:
                    if time.monotonic() - last_wait_message >= 2.0:
                        visible_pallets = [fid for fid in constants.PALLET_FIDUCIALS if fid in markers]
                        visible_goals = [fid for fid in constants.GOAL_FIDUCIALS if fid in markers]
                        used_goals = [t['goal_id'] for t in registry.tasks.values() if t['state'] != 'PENDING']
                        print(f"[WAIT PLANNING] visible robot IDs={robots}; "
                              f"pallet IDs={visible_pallets}; goal IDs={visible_goals}; "
                              f"occupied/reserved goals={used_goals}; "
                              f"available pairs={[(t['pallet_id'], t['goal_id']) for t in tasks]}")
                        last_wait_message = time.monotonic()
                    time.sleep(0.03)
                    continue
                known_robots.update(robots)
                vision.assert_fresh(robots)
                hw_ids = _fid_ids_to_hw_ids(robots)
                print(f"[PLANNING] PARROT visible-order pairs: "
                      f"{[(t['pallet_id'], t['goal_id']) for t in tasks]}; robot IDs={robots}")
                poses = [markers[fid] for fid in robots]
                pallets = [_get_pallet_dock_pose(markers[t['pallet_id']]) for t in tasks]
                goals = [markers[t['goal_id']] for t in tasks]
                width, height = vision.cv_GetSandboxSize()
                for pose in poses + pallets + goals:
                    if not (0 <= pose[0] < width and 0 <= pose[1] < height):
                        raise ValueError(f'Pose outside calibrated field: {pose}')
                task_pallet_ids = {t['pallet_id'] for t in tasks}
                obstacles = [markers[fid] for fid in constants.PALLET_FIDUCIALS
                             if fid in markers and fid not in task_pallet_ids]
                for delivered in registry.tasks.values():
                    if delivered['state'] == 'DELIVERED' and delivered['pallet_id'] not in markers:
                        if 'delivery_pose' not in delivered:
                            raise RuntimeError('Missing completed pallet position in current run')
                        obstacles.append(delivered['delivery_pose'])

                paths = planner.plan_trajectories([int(width), int(height)], poses, pallets, goals,
                                                  obstacle_poses=obstacles)

                vision.cv_runLocalizer()
                vision.assert_fresh(robots)
                after = vision.cv_GetMarkerPositionsMM()
                if any(math.hypot(after[fid][0]-pose[0], after[fid][1]-pose[1]) > 35
                       or abs(math.atan2(math.sin(after[fid][2]-pose[2]), math.cos(after[fid][2]-pose[2]))) > 0.3
                       for fid, pose in zip(robots, poses)):
                    raise RuntimeError('Robot moved during planning; discard plan')
                for task in tasks:
                    task['delivery_pose'] = markers[task['goal_id']]
                active = registry.assign(tasks, planner.last_assignments, hw_ids)
                if not active or not any(paths):
                    raise RuntimeError('NO_PATH: no executable task; not a completed round')
                for task in active:
                    print(f"[ASSIGN] robot hw={task['robot_id']}: "
                          f"pallet {task['pallet_id']} -> goal {task['goal_id']}")
                plan_id += 1
                start = time.monotonic()
                controller.load_paths(paths, hw_ids, start_time=start)
                visualizer.reset_trails()
                visualizer.set_paths(paths, robots)
                visualizer.visualize(vision)
                cv.waitKey(1)
                state = 'RUNNING'
            if state == 'RUNNING':
                controller.run_one_tick(vision)
                visualizer.set_wheel_speeds(controller.get_wheel_speeds())
                for hw, path in zip(hw_ids, paths):
                    if not path and not network.stop_robot(hw):
                        raise RuntimeError('Idle robot heartbeat rejected')
                if controller.all_done():
                    registry.complete_round(active)
                    state = 'PLANNING'
                    print('Controller round completed; checking camera for remaining tasks.')
            log_file.write(json.dumps(dict(at=time.monotonic(), frame_id=vision._localized_frame_id,
                                           plan_id=plan_id, state=state, markers=markers,
                                           controllers=controller.get_status(), network=network.get_stats()))+'\n')
            log_file.flush()
    except KeyboardInterrupt:
        print('Stopped by operator')
    except Exception as exc:
        print(f'[FAULT] {exc}')
        raise
    finally:
        network.stop_all(blocking=True)
        network.close()
        vision.stop_camera()
        cv.destroyAllWindows()
        if log_file:
            log_file.close()

if __name__ == '__main__':
    main()
