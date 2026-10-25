/**
 * @file planner_coordinator.cpp
 * @brief Điều phối lập lịch quỹ đạo đa robot.
 */

#include "planner_coordinator.hpp"
#include <chrono>

std::vector<std::vector<int>> generate_map(
    int map_length_mm,
    int map_width_mm,
    std::vector<std::vector<double>> pallet_poses)
{
    std::vector<std::vector<int>> map(
        map_length_mm / 10,
        std::vector<int>(map_width_mm / 10, 0));

    for (auto &p : pallet_poses)
    {
        int x = (int)(p[0] / 10);
        int y = (int)(p[1] / 10);

        for (int i = -1; i <= 1; i++)
        {
            for (int j = -1; j <= 1; j++)
            {
                int xi = x + i;
                int yj = y + j;
                if (xi >= 0 && xi < (int)map.size() &&
                    yj >= 0 && yj < (int)map[0].size())
                {
                    map[xi][yj] = 1;
                }
            }
        }
    }

    return map;
}

PlannerCoordinator::PlannerCoordinator(int num_robots)
{
    this->num_robots = num_robots;

    for (int i = 0; i < num_robots; i++)
    {
        this->motion_planners.push_back(new MotionPlanner4D(i));
    }

    this->task_assigner = new TaskAssigner(num_robots);
}

PlannerCoordinator::~PlannerCoordinator()
{
    for (auto *mp : this->motion_planners)
    {
        delete mp;
    }
    this->motion_planners.clear();

    delete this->task_assigner;
    this->task_assigner = nullptr;
}

void PlannerCoordinator::set_map(std::vector<std::vector<int>> map)
{
    this->map = map;
    for (int i = 0; i < (int)this->motion_planners.size(); i++)
    {
        this->motion_planners[i]->set_map(map);
    }
}

void PlannerCoordinator::set_robot_poses(std::vector<std::vector<double>> robot_poses)
{
    this->robot_poses.clear();

    for (int i = 0; i < (int)robot_poses.size(); i++)
    {
        std::vector<double> pose;
        for (int j = 0; j < (int)robot_poses[i].size() - 1; j++)
        {
            pose.push_back(std::round(robot_poses[i][j] / 10.0));
        }
        pose.push_back(robot_poses[i].back());
        this->robot_poses.push_back(pose);
    }
}

void PlannerCoordinator::set_pallet_and_goal_poses(
    std::vector<std::vector<double>> pallet_and_goal_poses)
{
    this->pallet_and_goal_poses.clear();

    for (int i = 0; i < (int)pallet_and_goal_poses.size(); i++)
    {
        std::vector<double> pose;
        for (int j = 0; j < (int)pallet_and_goal_poses[i].size(); j++)
        {
            if (j == 2 || j == 5)
            {
                pose.push_back(pallet_and_goal_poses[i][j]);
            }
            else
            {
                pose.push_back(std::round(pallet_and_goal_poses[i][j] / 10.0));
            }
        }
        this->pallet_and_goal_poses.push_back(pose);
    }

    std::cout << "[PlannerCoordinator] Vị trí pallet và đích (cm):" << std::endl;
    for (auto &pose : this->pallet_and_goal_poses)
    {
        for (auto &v : pose) std::cout << v << " ";
        std::cout << std::endl;
    }
}

int PlannerCoordinator::plan()
{
    int n_robots = (int)this->robot_poses.size();
    if (n_robots == 0)
    {
        this->paths.clear();
        return 0;
    }

    while ((int)this->motion_planners.size() < n_robots)
    {
        int id = (int)this->motion_planners.size();
        MotionPlanner4D *mp = new MotionPlanner4D(id);
        if (!this->map.empty())
        {
            mp->set_map(this->map);
        }
        this->motion_planners.push_back(mp);
    }

    this->task_assigner->assign_pallets_to_robots(
        this->robot_poses, this->pallet_and_goal_poses);
    std::vector<int> task_assignments = this->task_assigner->get_task_assignments();

    this->paths = std::vector<std::vector<std::vector<double>>>(n_robots);
    for (int i = 0; i < n_robots; i++)
    {
        this->paths[i] = {{
            this->robot_poses[i][0],
            this->robot_poses[i][1],
            this->robot_poses[i][2],
            0.0, 2.0
        }};
    }

    for (int i = 0; i < n_robots; i++)
    {
        std::cout << "[PlannerCoordinator] Lập kế hoạch cho robot " << i << std::endl;
        std::cout << "[PlannerCoordinator] Vị trí robot hiện tại: ";
        for (auto v : this->robot_poses[i]) std::cout << v << " ";
        std::cout << std::endl;

        if (i >= (int)task_assignments.size() || task_assignments[i] == -1)
        {
            std::cout << "[PlannerCoordinator] Không có nhiệm vụ nào được gán cho robot " << i << std::endl;
            continue;
        }

        std::cout << "[PlannerCoordinator] Vị trí pallet và đích hiện tại: ";
        for (auto v : this->pallet_and_goal_poses[task_assignments[i]])
            std::cout << v << " ";
        std::cout << std::endl;

        this->motion_planners[i]->set_start(this->robot_poses[i]);
        this->motion_planners[i]->set_goals(this->pallet_and_goal_poses[task_assignments[i]]);
        this->motion_planners[i]->set_robot_paths(this->paths);

        if (this->motion_planners[i]->plan() == -1)
        {
            std::cout << "[PlannerCoordinator] Lập kế hoạch thất bại cho robot " << i << std::endl;
            this->task_assigner->unassign_pallet_from_robot(i);
            task_assignments = this->task_assigner->get_task_assignments();
        }
        else
        {
            std::cout << "[PlannerCoordinator] Lập kế hoạch thành công cho robot " << i << std::endl;
            this->paths[i] = this->motion_planners[i]->get_path();
        }
    }

    for (int i = 0; i < n_robots; i++)
    {
        if (i >= (int)task_assignments.size() || task_assignments[i] == -1)
        {
            this->paths[i].clear();
        }
    }

    return 0;
}

std::vector<std::vector<std::vector<double>>> PlannerCoordinator::get_paths()
{
    std::vector<std::vector<std::vector<double>>> result;

    for (int i = 0; i < (int)this->paths.size(); i++)
    {
        std::vector<std::vector<double>> robot_path;
        for (int j = 0; j < (int)this->paths[i].size(); j++)
        {
            std::vector<double> waypoint;
            for (int k = 0; k < (int)this->paths[i][j].size(); k++)
            {
                if (k == 0 || k == 1)
                {
                    waypoint.push_back(this->paths[i][j][k] * 10.0);
                }
                else if (k == 2)
                {
                    waypoint.push_back(std::atan2(
                        std::sin(this->paths[i][j][k]),
                        std::cos(this->paths[i][j][k])));
                }
                else
                {
                    waypoint.push_back(this->paths[i][j][k]);
                }
            }
            robot_path.push_back(waypoint);
        }
        result.push_back(robot_path);
    }

    return result;
}

std::vector<std::vector<std::vector<double>>> PlannerCoordinator::generate_paths(
    std::vector<int> map_size,
    std::vector<std::vector<double>> robot_poses,
    std::vector<std::vector<double>> pallet_poses,
    std::vector<std::vector<double>> goal_poses)
{
    std::vector<std::vector<int>> map = generate_map(map_size[0], map_size[1], pallet_poses);
    this->set_map(map);

    std::cout << "[PlannerCoordinator] Vị trí các robot:" << std::endl;
    for (auto &pose : robot_poses)
        std::cout << pose[0] << ", " << pose[1] << ", " << pose[2] << std::endl;

    this->set_robot_poses(robot_poses);

    std::vector<std::vector<double>> pallet_and_goal_poses;
    for (int i = 0; i < (int)pallet_poses.size(); i++)
    {
        pallet_and_goal_poses.push_back({
            pallet_poses[i][0], pallet_poses[i][1], pallet_poses[i][2],
            goal_poses[i][0],   goal_poses[i][1],   goal_poses[i][2]
        });
    }
    this->set_pallet_and_goal_poses(pallet_and_goal_poses);

    this->plan();
    return this->get_paths();
}

void PlannerCoordinator::print_paths()
{
    for (int i = 0; i < (int)this->paths.size(); i++)
    {
        std::cout << "[PlannerCoordinator] Quỹ đạo cho robot " << i << ":" << std::endl;
        if (i < (int)this->motion_planners.size())
        {
            this->motion_planners[i]->print_path();
        }
    }
}
