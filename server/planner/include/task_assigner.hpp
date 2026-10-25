/**
 * @file task_assigner.hpp
 * @brief Thuật toán gán Pallet cho Robot.
 */

#ifndef TASK_ASSIGNER_HPP
#define TASK_ASSIGNER_HPP

#include <iostream>
#include <vector>
#include <algorithm>
#include <cmath>

class TaskAssigner
{
public:
    TaskAssigner(int num_robots);
    ~TaskAssigner();

    /** Gán Pallet cho Robot theo cấu hình thuật toán. */
    void assign_pallets_to_robots(
        std::vector<std::vector<double>> robot_poses,
        std::vector<std::vector<double>> pallet_and_goal_poses);

    /** Gán tuần tự FIFO. */
    std::vector<int> assign_pallets_to_robots_simple(
        std::vector<std::vector<double>> robot_poses,
        std::vector<std::vector<double>> pallet_and_goal_poses);

    /** Gán theo khoảng cách Euclidean gần nhất. */
    std::vector<int> assign_pallets_to_robots_heuristic(
        std::vector<std::vector<double>> robot_poses,
        std::vector<std::vector<double>> pallet_and_goal_poses);

    /** Hủy gán pallet khi lập kế hoạch đường đi thất bại. */
    void unassign_pallet_from_robot(int robot_id);

    /** Xóa dữ liệu gán việc và pallet đã hoàn thành để lập kế hoạch mới. */
    void reset();

    /** Lấy danh sách gán việc hiện tại. */
    std::vector<int> get_task_assignments();

private:
    int num_robots;
    std::vector<int> dropped_off_pallets;
    std::vector<int> task_assignments;
};

#endif // TASK_ASSIGNER_HPP
