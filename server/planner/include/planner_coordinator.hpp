/**
 * @file planner_coordinator.hpp
 * @brief Điều phối lập lịch đường đi cho hệ thống đa robot PARROT.
 *
 * Kế thừa từ planner.hh / planner.cc và main.cc gốc.
 *
 * Sửa lỗi so với bản gốc:
 *   - SỬA LỖI 3 (Global State): Không có biến toàn cục Planner* planner.
 *     PlannerCoordinator là một class bình thường, vòng đời quản lý bởi Python.
 *   - Destructor đầy đủ: giải phóng tất cả MotionPlanner4D* và TaskAssigner*.
 *   - Tích hợp hàm generate_map() từ main.cc gốc.
 *   - Hàm generate_paths() tiện ích gộp toàn bộ pipeline.
 *
 * Giữ nguyên:
 *   - Logic chuyển đổi đơn vị: mm / 10 => cm (input), cm * 10 => mm (output)
 *   - Logic chuyển theta sang [-pi, pi] bằng atan2(sin, cos)
 *   - Lập lịch tuần tự: robot 0 trước, robot 1 dùng path của robot 0 làm vật cản động
 *   - Khởi tạo path mặc định (pose hiện tại, tag=2) trước khi plan
 */

#ifndef PLANNER_COORDINATOR_HPP
#define PLANNER_COORDINATOR_HPP

#include <iostream>
#include <vector>
#include <cmath>
#include "motion_planner_4d.hpp"
#include "task_assigner.hpp"

// =============================================================================
// Hàm tiện ích sinh bản đồ lưới 2D (1cm/cell)
// Kế thừa từ generate_map() trong main.cc gốc.
//
// @param map_length_mm  Chiều dài sandbox (mm)
// @param map_width_mm   Chiều rộng sandbox (mm)
// @param pallet_poses   Danh sách [x, y, theta] pallet (mm)
// @return Lưới 2D: 0 = tự do, 1 = vật cản
// =============================================================================
std::vector<std::vector<int>> generate_map(
    int map_length_mm,
    int map_width_mm,
    std::vector<std::vector<double>> pallet_poses);

class PlannerCoordinator
{
public:
    /**
     * @param num_robots Số lượng robot tham gia hệ thống.
     */
    PlannerCoordinator(int num_robots);

    /**
     * @brief Destructor: Giải phóng tất cả MotionPlanner4D* và TaskAssigner*.
     *        Sửa lỗi Global State — đảm bảo không rò rỉ bộ nhớ khi Python thu hồi object.
     */
    ~PlannerCoordinator();

    /**
     * @brief Đặt bản đồ lưới. Mỗi ô 1cm x 1cm. 0=tự do, 1=vật cản.
     */
    void set_map(std::vector<std::vector<int>> map);

    /**
     * @brief Đặt vị trí robot [x, y, theta] (đơn vị: mm).
     *        Tự động chuyển sang cm bên trong.
     */
    void set_robot_poses(std::vector<std::vector<double>> robot_poses);

    /**
     * @brief Đặt vị trí pallet+goal [px, py, ptheta, gx, gy, gtheta] (đơn vị: mm).
     *        Tự động chuyển tọa độ sang cm, giữ nguyên theta.
     */
    void set_pallet_and_goal_poses(std::vector<std::vector<double>> pallet_and_goal_poses);

    /**
     * @brief Lập kế hoạch tuần tự cho tất cả robot.
     * Robot trước làm vật cản động cho robot sau.
     * @return 0 nếu thành công.
     */
    int plan();

    /**
     * @brief Lấy quỹ đạo đã lập kế hoạch (đơn vị: mm, theta: [-pi, pi]).
     * @return Vector 3D: [robot][waypoint][x, y, theta, time, tag]
     */
    std::vector<std::vector<std::vector<double>>> get_paths();

    /**
     * @brief Hàm tiện ích gộp: Sinh map, gán việc, lập kế hoạch, trả về quỹ đạo.
     *
     * @param map_size    [width_mm, height_mm]
     * @param robot_poses  [[x, y, theta], ...] (mm)
     * @param pallet_poses [[x, y, theta], ...] (mm)
     * @param goal_poses   [[x, y, theta], ...] (mm)
     * @return Vector 3D: [robot][waypoint][x, y, theta, time, tag]
     */
    std::vector<std::vector<std::vector<double>>> generate_paths(
        std::vector<int> map_size,
        std::vector<std::vector<double>> robot_poses,
        std::vector<std::vector<double>> pallet_poses,
        std::vector<std::vector<double>> goal_poses);

    /**
     * @brief In tất cả quỹ đạo ra stdout (debug).
     */
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

#endif // PLANNER_COORDINATOR_HPP
