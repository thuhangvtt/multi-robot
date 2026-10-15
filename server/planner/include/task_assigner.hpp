/**
 * @file task_assigner.hpp
 * @brief Thuật toán gán Pallet -> Robot cho hệ thống đa robot PARROT.
 *
 * Kế thừa 100% logic từ task_planner.hh / task_planner.cc gốc.
 * Đổi tên class TaskPlanner -> TaskAssigner để phân biệt với package planner.
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

    /**
     * @brief Gán Pallet cho Robot. Gọi heuristic hoặc simple dựa trên cờ USE_HEURISTIC.
     *
     * @param robot_poses        Danh sách [x, y, theta] của từng robot (đơn vị: cm)
     * @param pallet_and_goal_poses Danh sách [px, py, ptheta, gx, gy, gtheta] (đơn vị: cm)
     */
    void assign_pallets_to_robots(
        std::vector<std::vector<double>> robot_poses,
        std::vector<std::vector<double>> pallet_and_goal_poses);

    /**
     * @brief Thuật toán gán đơn giản (tuần tự FIFO, không xét khoảng cách).
     *
     * @return std::vector<int> Chỉ số pallet cho từng robot. -1 nếu không có việc.
     */
    std::vector<int> assign_pallets_to_robots_simple(
        std::vector<std::vector<double>> robot_poses,
        std::vector<std::vector<double>> pallet_and_goal_poses);

    /**
     * @brief Thuật toán gán heuristic Euclidean: gán pallet cho robot gần nhất chưa có việc.
     *
     * @return std::vector<int> Chỉ số pallet cho từng robot. -1 nếu không có việc.
     */
    std::vector<int> assign_pallets_to_robots_heuristic(
        std::vector<std::vector<double>> robot_poses,
        std::vector<std::vector<double>> pallet_and_goal_poses);

    /**
     * @brief Hủy gán pallet của robot (khi lập kế hoạch thất bại).
     *        Xóa pallet khỏi danh sách dropped_off_pallets để cho phép gán lại.
     *
     * @param robot_id Chỉ số robot cần hủy gán.
     */
    void unassign_pallet_from_robot(int robot_id);

    /**
     * @brief Lấy danh sách gán việc hiện tại.
     *
     * @return std::vector<int> Chỉ số pallet cho từng robot. -1 nếu không có việc.
     */
    std::vector<int> get_task_assignments();

private:
    int num_robots;

    // Danh sách pallet đã giao thành công — tránh gán lại khi replan
    std::vector<int> dropped_off_pallets;

    // Kết quả gán việc hiện tại: task_assignments[robot_id] = pallet_index
    std::vector<int> task_assignments;
};

#endif // TASK_ASSIGNER_HPP
