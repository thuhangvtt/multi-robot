/**
 * @file planner_coordinator.hpp
 * @brief Điều phối lập lịch quỹ đạo đa robot.
 */

#ifndef PLANNER_COORDINATOR_HPP
#define PLANNER_COORDINATOR_HPP

#include <iostream>
#include <vector>
#include <cmath>
#include "motion_planner_4d.hpp"
#include "task_assigner.hpp"

/** Sinh bản đồ lưới 2D từ kích thước sandbox và vị trí các pallet. */
std::vector<std::vector<int>> generate_map(
    int map_length_mm,
    int map_width_mm,
    std::vector<std::vector<double>> pallet_poses);

class PlannerCoordinator
{
public:
    PlannerCoordinator(int num_robots);
    ~PlannerCoordinator();

    /** Đặt bản đồ lưới nhị phân. */
    void set_map(std::vector<std::vector<int>> map);

    /** Đặt vị trí hiện tại của các robot theo mm. */
    void set_robot_poses(std::vector<std::vector<double>> robot_poses);

    /** Đặt vị trí pallet và điểm đích theo mm. */
    void set_pallet_and_goal_poses(std::vector<std::vector<double>> pallet_and_goal_poses);

    /** Lập kế hoạch tuần tự tránh va chạm động cho toàn bộ robot. */
    int plan();

    /** Lấy quỹ đạo sau khi lập kế hoạch theo đơn vị mm. */
    std::vector<std::vector<std::vector<double>>> get_paths();

    /** Pipeline đầy đủ: sinh map, gán việc, lập quỹ đạo và trả về kết quả. */
    std::vector<std::vector<std::vector<double>>> generate_paths(
        std::vector<int> map_size,
        std::vector<std::vector<double>> robot_poses,
        std::vector<std::vector<double>> pallet_poses,
        std::vector<std::vector<double>> goal_poses);

    /** In quỹ đạo các robot ra màn hình kiểm tra. */
    void print_paths();

private:
    int num_robots;
    std::vector<std::vector<int>>                 map;
    std::vector<std::vector<double>>              robot_poses;
    std::vector<std::vector<double>>              pallet_and_goal_poses;
    std::vector<std::vector<std::vector<double>>> paths;

    std::vector<MotionPlanner4D *> motion_planners;
    TaskAssigner                  *task_assigner;
};

#endif 
