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

    void assign_pallets_to_robots(
        std::vector<std::vector<double>> robot_poses,
        std::vector<std::vector<double>> pallet_and_goal_poses);

    std::vector<int> assign_pallets_to_robots_simple(
        std::vector<std::vector<double>> robot_poses,
        std::vector<std::vector<double>> pallet_and_goal_poses);

    std::vector<int> assign_pallets_to_robots_heuristic(
        std::vector<std::vector<double>> robot_poses,
        std::vector<std::vector<double>> pallet_and_goal_poses);

    void unassign_pallet_from_robot(int robot_id);

    void reset();

    std::vector<int> get_task_assignments();

private:
    int num_robots;
    std::vector<int> dropped_off_pallets;
    std::vector<int> task_assignments;
};

#endif
