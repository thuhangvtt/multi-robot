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

    int plan();

    void set_start(std::vector<double> start);

    void set_goals(std::vector<double> goals);

    void set_map(std::vector<std::vector<int>> map);

    void set_robot_paths(std::vector<std::vector<std::vector<double>>> robot_paths);

    std::vector<std::vector<double>> get_path();

    void print_path();

    int robot_index;

private:
    std::vector<std::vector<int>> map;
    std::vector<double> start;
    std::vector<double> pallet_goal;
    std::vector<double> dropoff_goal;
    std::vector<std::vector<double>> path;
    std::vector<std::vector<std::vector<double>>> paths;

    int a_star(bool is_pallet_goal);

    std::vector<Node *> get_neighbors(Node *curr_node, bool is_pallet_goal);

    bool is_in_collision(Node *node, bool is_pallet_goal);

    double straight_time_step = STRAIGHT_TIME_STEP;
    double turn_time_step     = TURN_TIME_STEP;
    double in_place_time_step = IN_PLACE_TURN_TIME_STEP;

    std::map<int, std::vector<std::vector<double>>> angle_to_dxdythetadtimecosts = {
        {0, {{1, 0, 0.0, straight_time_step, STRAIGHT_COST},
             {7, -3, M_PI / 4, turn_time_step, TURN_COST},
             {7, 3, 7 * M_PI / 4, turn_time_step, TURN_COST}}},
        {1, {{1, -1, M_PI / 4, straight_time_step, STRAIGHT_COST},
             {3, -7, M_PI / 2, turn_time_step, TURN_COST},
             {7, -3, 0.0, turn_time_step, TURN_COST}}},
        {2, {{0, -1, M_PI / 2, straight_time_step, STRAIGHT_COST},
             {-3, -7, 3 * M_PI / 4, turn_time_step, TURN_COST},
             {3, -7, M_PI / 4, turn_time_step, TURN_COST}}},
        {3, {{-1, -1, 3 * M_PI / 4, straight_time_step, STRAIGHT_COST},
             {-7, -3, M_PI, turn_time_step, TURN_COST},
             {-3, -7, M_PI / 2, turn_time_step, TURN_COST}}},
        {4, {{-1, 0, M_PI, straight_time_step, STRAIGHT_COST},
             {-7, 3, 5 * M_PI / 4, turn_time_step, TURN_COST},
             {-7, -3, 3 * M_PI / 4, turn_time_step, TURN_COST}}},
        {5, {{-1, 1, 5 * M_PI / 4, straight_time_step, STRAIGHT_COST},
             {-3, 7, 3 * M_PI / 2, turn_time_step, TURN_COST},
             {-7, 3, M_PI, turn_time_step, TURN_COST}}},
        {6, {{0, 1, 3 * M_PI / 2, straight_time_step, STRAIGHT_COST},
             {3, 7, 7 * M_PI / 4, turn_time_step, TURN_COST},
             {-3, 7, 5 * M_PI / 4, turn_time_step, TURN_COST}}},
        {7, {{1, 1, 7 * M_PI / 4, straight_time_step, STRAIGHT_COST},
             {7, 3, 0.0, turn_time_step, TURN_COST},
             {3, 7, 3 * M_PI / 2, turn_time_step, TURN_COST}}}
    };
};

#endif
