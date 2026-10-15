/**
 * @file task_assigner.cpp
 * @brief Triển khai thuật toán gán Pallet -> Robot.
 *        Kế thừa 100% logic từ task_planner.cc gốc.
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

// =============================================================================
// Thuật toán đơn giản: gán FIFO — robot 0 lấy pallet đầu tiên chưa giao, v.v.
// Giữ nguyên 100% so với assign_pallets_to_robots_simple() gốc.
// =============================================================================
std::vector<int> TaskAssigner::assign_pallets_to_robots_simple(
    std::vector<std::vector<double>> robot_poses,
    std::vector<std::vector<double>> pallet_and_goal_poses)
{
    std::vector<int> task_assignments;

    std::cout << "Dropped off pallets: ";
    for (int i = 0; i < (int)this->dropped_off_pallets.size(); i++)
    {
        std::cout << this->dropped_off_pallets[i] << " ";
    }
    std::cout << std::endl;

    for (int i = 0; i < this->num_robots; i++)
    {
        bool task_assigned = false;
        for (int j = 0; j < (int)pallet_and_goal_poses.size(); j++)
        {
            // Bỏ qua pallet đã giao
            if (std::find(this->dropped_off_pallets.begin(),
                          this->dropped_off_pallets.end(), j) == this->dropped_off_pallets.end())
            {
                task_assignments.push_back(j);
                this->dropped_off_pallets.push_back(j);
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

// =============================================================================
// Thuật toán heuristic Euclidean: với mỗi pallet, tìm robot gần nhất chưa có việc.
// Giữ nguyên 100% so với assign_pallets_to_robots_heuristic() gốc.
// =============================================================================
std::vector<int> TaskAssigner::assign_pallets_to_robots_heuristic(
    std::vector<std::vector<double>> robot_poses,
    std::vector<std::vector<double>> pallet_and_goal_poses)
{
    std::vector<int>  task_assignments(this->num_robots, -1);
    std::vector<bool> robot_assigned(robot_poses.size(), false);

    // Tính ma trận khoảng cách pallet -> robot
    std::vector<std::vector<double>> pallet_to_robot_distances;
    for (int i = 0; i < (int)pallet_and_goal_poses.size(); i++)
    {
        std::vector<double> distances;
        for (int j = 0; j < (int)robot_poses.size(); j++)
        {
            double dx = pallet_and_goal_poses[i][0] - robot_poses[j][0];
            double dy = pallet_and_goal_poses[i][1] - robot_poses[j][1];
            distances.push_back(std::sqrt(dx * dx + dy * dy));
        }
        pallet_to_robot_distances.push_back(distances);
    }

    // Với mỗi pallet (theo thứ tự index), gán cho robot gần nhất chưa có việc
    for (int i = 0; i < (int)pallet_and_goal_poses.size(); i++)
    {
        // Bỏ qua pallet đã giao
        if (std::find(this->dropped_off_pallets.begin(),
                      this->dropped_off_pallets.end(), i) != this->dropped_off_pallets.end())
        {
            continue;
        }

        int    closest_robot          = -1;
        double closest_robot_distance = 1000000.0;

        for (int j = 0; j < (int)robot_poses.size(); j++)
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
            this->dropped_off_pallets.push_back(i);
        }
    }

    return task_assignments;
}

// =============================================================================
// Dispatcher: chọn thuật toán dựa trên cờ USE_HEURISTIC.
// Giữ nguyên 100% so với assign_pallets_to_robots() gốc.
// =============================================================================
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

// =============================================================================
// Hủy gán pallet của robot (dùng khi lập kế hoạch thất bại).
// Xóa pallet khỏi dropped_off_pallets để robot khác có thể được gán.
// Giữ nguyên 100% so với unassign_pallet_from_robot() gốc.
// =============================================================================
void TaskAssigner::unassign_pallet_from_robot(int robot_id)
{
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
