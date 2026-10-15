/**
 * @file motion_planner_4d.hpp
 * @brief Core A* 4D search engine cho từng robot.
 *
 * Kế thừa 100% logic từ motion_planner.hh / motion_planner.cc gốc.
 *
 * Sửa lỗi so với bản gốc:
 *   - SỬA LỖI 1 (Memory Leak): Hàm a_star() giờ dùng allocated_nodes vector
 *     để theo dõi và giải phóng toàn bộ Node* trước khi return.
 *
 * Giữ nguyên:
 *   - Bảng động học 8 góc angle_to_dxdythetadtimecosts
 *   - Kiểm tra va chạm tĩnh (footprint 6cm / 7cm)
 *   - Kiểm tra va chạm động với robot khác (cửa sổ 5s)
 *   - Heuristic Euclidean 3D (x, y, theta)
 *   - Chạy a_star(true) đến pallet, rồi a_star(false) đến goal
 */

#ifndef MOTION_PLANNER_4D_HPP
#define MOTION_PLANNER_4D_HPP

#include <iostream>
#include <vector>
#include <map>
#include <cmath>
#include "search_node.hpp"

class MotionPlanner4D
{
public:
    MotionPlanner4D(int robot_index);
    ~MotionPlanner4D();

    /**
     * @brief Lập kế hoạch đường đi cho robot: start -> pallet -> goal.
     * @return 0 nếu thành công, -1 nếu không tìm được đường.
     */
    int plan();

    /**
     * @brief Đặt vị trí bắt đầu (x, y, theta) — đơn vị cm.
     */
    void set_start(std::vector<double> start);

    /**
     * @brief Đặt hai điểm đích: (px, py, ptheta, gx, gy, gtheta) — đơn vị cm.
     */
    void set_goals(std::vector<double> goals);

    /**
     * @brief Đặt bản đồ lưới. 0 = tự do, 1 = vật cản. Mỗi ô 1 cm x 1 cm.
     */
    void set_map(std::vector<std::vector<int>> map);

    /**
     * @brief Đặt danh sách quỹ đạo của tất cả robot (dùng để kiểm tra va chạm động).
     * Định dạng mỗi waypoint: [x, y, theta, time, tag].
     */
    void set_robot_paths(std::vector<std::vector<std::vector<double>>> robot_paths);

    /**
     * @brief Lấy quỹ đạo đã lập kế hoạch.
     * Định dạng mỗi waypoint: [x, y, theta, time, tag]
     * tag: 0=đang đi, 1=tại pallet, 2=tại goal.
     */
    std::vector<std::vector<double>> get_path();

    /**
     * @brief In quỹ đạo ra stdout (debug).
     */
    void print_path();

    int robot_index;

private:
    std::vector<std::vector<int>>             map;
    std::vector<double>                       start;
    std::vector<double>                       pallet_goal;
    std::vector<double>                       dropoff_goal;
    std::vector<std::vector<double>>          path;
    std::vector<std::vector<std::vector<double>>> paths; // quỹ đạo các robot khác

    /**
     * @brief A* 4D search từ start đến pallet hoặc goal.
     * Tích hợp cơ chế thu hồi bộ nhớ: toàn bộ Node* được xóa trước khi return.
     *
     * @param is_pallet_goal true = tìm đến pallet, false = tìm đến drop-off goal.
     * @return 0 nếu thành công, -1 nếu thất bại / timeout.
     */
    int a_star(bool is_pallet_goal);

    /**
     * @brief Sinh danh sách neighbor hợp lệ từ node hiện tại.
     * Mỗi neighbor là Node* cấp phát bằng new — PHẢI được đưa vào allocated_nodes.
     */
    std::vector<Node *> get_neighbors(Node *curr_node, bool is_pallet_goal);

    /**
     * @brief Kiểm tra va chạm tĩnh và động.
     * @return true nếu đang va chạm.
     */
    bool is_in_collision(Node *node, bool is_pallet_goal);

    double straight_time_step = STRAIGHT_TIME_STEP;
    double turn_time_step     = TURN_TIME_STEP;

    // Bảng chuyển động vi sai: angle_index -> [{dx, dy, new_theta, dt, cost}, ...]
    // 0 = phải (East), 1 = Đông-Bắc, 2 = Bắc (North), ..., 7 = Đông-Nam
    std::map<int, std::vector<std::vector<double>>> angle_to_dxdythetadtimecosts = {
        {0, {{1, 0, 0.0, straight_time_step, STRAIGHT_COST}, {5, -5, M_PI / 4, turn_time_step, TURN_COST}, {5, 5, 7 * M_PI / 4, turn_time_step, TURN_COST}}},
        {1, {{1, -1, M_PI / 4, straight_time_step, STRAIGHT_COST}, {0, -5, M_PI / 2, turn_time_step, TURN_COST}, {5, 0, 0, turn_time_step, TURN_COST}}},
        {2, {{0, -1, M_PI / 2, straight_time_step, STRAIGHT_COST}, {-5, -5, 3 * M_PI / 4, turn_time_step, TURN_COST}, {5, -5, M_PI / 4, turn_time_step, TURN_COST}}},
        {3, {{-1, -1, 3 * M_PI / 4, straight_time_step, STRAIGHT_COST}, {-5, 0, M_PI, turn_time_step, TURN_COST}, {0, -5, M_PI / 2, turn_time_step, TURN_COST}}},
        {4, {{-1, 0, M_PI, straight_time_step, STRAIGHT_COST}, {-5, 5, 5 * M_PI / 4, turn_time_step, TURN_COST}, {-5, -5, 3 * M_PI / 4, turn_time_step, TURN_COST}}},
        {5, {{-1, 1, 5 * M_PI / 4, straight_time_step, STRAIGHT_COST}, {0, 5, 3 * M_PI / 2, turn_time_step, TURN_COST}, {-5, 0, M_PI, turn_time_step, TURN_COST}}},
        {6, {{0, 1, 3 * M_PI / 2, straight_time_step, STRAIGHT_COST}, {5, 5, 7 * M_PI / 4, turn_time_step, TURN_COST}, {-5, 5, 5 * M_PI / 4, turn_time_step, TURN_COST}}},
        {7, {{1, 1, 7 * M_PI / 4, straight_time_step, STRAIGHT_COST}, {5, 0, 0.0, turn_time_step, TURN_COST}, {0, 5, 3 * M_PI / 2, turn_time_step, TURN_COST}}}
    };
};

#endif // MOTION_PLANNER_4D_HPP
