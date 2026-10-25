/**
 * @file motion_planner_4d.hpp
 * @brief Định nghĩa lớp MotionPlanner4D cho thuật toán A* 4D (x, y, theta, time).
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

    /** Lập kế hoạch đường đi cho robot từ start đến pallet và drop-off. */
    int plan();

    /** Đặt vị trí bắt đầu [x, y, theta] theo cm. */
    void set_start(std::vector<double> start);

    /** Đặt vị trí pallet và drop-off theo cm. */
    void set_goals(std::vector<double> goals);

    /** Đặt bản đồ lưới 2D. */
    void set_map(std::vector<std::vector<int>> map);

    /** Đặt danh sách quỹ đạo của các robot để tránh va chạm động. */
    void set_robot_paths(std::vector<std::vector<std::vector<double>>> robot_paths);

    /** Lấy quỹ đạo đã tìm được [x, y, theta, time, tag]. */
    std::vector<std::vector<double>> get_path();

    /** In quỹ đạo ra màn hình để kiểm tra. */
    void print_path();

    int robot_index;

private:
    std::vector<std::vector<int>> map;
    std::vector<double> start;
    std::vector<double> pallet_goal;
    std::vector<double> dropoff_goal;
    std::vector<std::vector<double>> path;
    std::vector<std::vector<std::vector<double>>> paths;

    /** Thuật toán tìm kiếm A* 4D tới đích. */
    int a_star(bool is_pallet_goal);

    /** Sinh tập nút lân cận hợp lệ theo bảng động học 8 hướng. */
    std::vector<Node *> get_neighbors(Node *curr_node, bool is_pallet_goal);

    /** Kiểm tra va chạm tĩnh với bản đồ và động với các robot khác. */
    bool is_in_collision(Node *node, bool is_pallet_goal);

    double straight_time_step = STRAIGHT_TIME_STEP;
    double turn_time_step     = TURN_TIME_STEP;

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
