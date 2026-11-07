/**
 * @file task_assigner.cpp
 * @brief Gán nhiệm vụ Pallet cho Robot.
 */

#include "task_assigner.hpp"
#include "planner_constants.hpp"

TaskAssigner::TaskAssigner(int num_robots)
{
    this->num_robots = num_robots;
}

TaskAssigner::~TaskAssigner()
{
}

std::vector<int> TaskAssigner::assign_pallets_to_robots_simple(
    std::vector<std::vector<double>> robot_poses,
    std::vector<std::vector<double>> pallet_and_goal_poses)
{
    std::vector<int> task_assignments;
    int n_robots = (int)robot_poses.size();

    std::cout << "[TaskAssigner] Danh sach pallet da hoan thanh: ";
    for (int i = 0; i < (int)this->dropped_off_pallets.size(); i++)
    {
        std::cout << this->dropped_off_pallets[i] << " ";
    }
    std::cout << std::endl;

    // Track pallets assigned in THIS planning round (separate from completed pallets)
    std::vector<int> currently_assigned;

    for (int i = 0; i < n_robots; i++)
    {
        bool task_assigned = false;
        for (int j = 0; j < (int)pallet_and_goal_poses.size(); j++)
        {
            // Skip pallets already completed (delivered)
            bool already_done = std::find(this->dropped_off_pallets.begin(),
                                          this->dropped_off_pallets.end(), j)
                                != this->dropped_off_pallets.end();
            // Skip pallets already assigned to another robot this round
            bool already_assigned = std::find(currently_assigned.begin(),
                                              currently_assigned.end(), j)
                                   != currently_assigned.end();

            if (!already_done && !already_assigned)
            {
                task_assignments.push_back(j);
                currently_assigned.push_back(j);
                task_assigned = true;
                break;
            }
        }
        if (!task_assigned)
        {
            task_assignments.push_back(-1);
        }
    }

    return task_assignments;
}

std::vector<int> TaskAssigner::assign_pallets_to_robots_heuristic(
    std::vector<std::vector<double>> robot_poses,
    std::vector<std::vector<double>> pallet_and_goal_poses)
{
    int n_robots = (int)robot_poses.size();
    std::vector<int>  task_assignments(n_robots, -1);
    std::vector<bool> robot_assigned(n_robots, false);

    std::vector<std::vector<double>> pallet_to_robot_distances;
    for (int i = 0; i < (int)pallet_and_goal_poses.size(); i++)
    {
        std::vector<double> distances;
        for (int j = 0; j < n_robots; j++)
        {
            double dx = pallet_and_goal_poses[i][0] - robot_poses[j][0];
            double dy = pallet_and_goal_poses[i][1] - robot_poses[j][1];
            distances.push_back(std::sqrt(dx * dx + dy * dy));
        }
        pallet_to_robot_distances.push_back(distances);
    }

    for (int i = 0; i < (int)pallet_and_goal_poses.size(); i++)
    {
        if (std::find(this->dropped_off_pallets.begin(),
                      this->dropped_off_pallets.end(), i) != this->dropped_off_pallets.end())
        {
            continue;
        }

        int    closest_robot          = -1;
        double closest_robot_distance = 1000000.0;

        for (int j = 0; j < n_robots; j++)
        {
            if (pallet_to_robot_distances[i][j] < closest_robot_distance && !robot_assigned[j])
            {
                closest_robot          = j;
                closest_robot_distance = pallet_to_robot_distances[i][j];
            }
        }

        if (closest_robot != -1)
        {
            task_assignments[closest_robot] = i;
            robot_assigned[closest_robot]   = true;
        }
    }

    return task_assignments;
}

void TaskAssigner::reset()
{
    this->dropped_off_pallets.clear();
    this->task_assignments.clear();
}

void TaskAssigner::assign_pallets_to_robots(
    std::vector<std::vector<double>> robot_poses,
    std::vector<std::vector<double>> pallet_and_goal_poses)
{
    if (USE_HEURISTIC)
    {
        this->task_assignments = this->assign_pallets_to_robots_heuristic(robot_poses, pallet_and_goal_poses);
    }
    else
    {
        this->task_assignments = this->assign_pallets_to_robots_simple(robot_poses, pallet_and_goal_poses);
    }
}

void TaskAssigner::unassign_pallet_from_robot(int robot_id)
{
    if (robot_id < 0 || robot_id >= (int)this->task_assignments.size())
        return;
    int pallet_id = this->task_assignments[robot_id];
    this->task_assignments[robot_id] = -1;

    int index = -1;
    for (int i = 0; i < (int)this->dropped_off_pallets.size(); i++)
    {
        if (this->dropped_off_pallets[i] == pallet_id)
        {
            index = i;
            break;
        }
    }

    if (index != -1)
    {
        this->dropped_off_pallets.erase(this->dropped_off_pallets.begin() + index);
    }
}

std::vector<int> TaskAssigner::get_task_assignments()
{
    return this->task_assignments;
}
