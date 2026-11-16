#ifndef PLANNER_COORDINATOR_HPP
#define PLANNER_COORDINATOR_HPP

#include <iostream>
#include <vector>
#include <cmath>
#include "motion_planner_4d.hpp"
#include "task_assigner.hpp"

std::vector<std::vector<int>> generate_map(
    int map_length_mm,
    int map_width_mm,
    std::vector<std::vector<double>> pallet_poses);

class PlannerCoordinator
{
public:
    PlannerCoordinator(int num_robots);
    ~PlannerCoordinator();

    void set_map(std::vector<std::vector<int>> map);

    void set_robot_poses(std::vector<std::vector<double>> robot_poses);

    void set_pallet_and_goal_poses(std::vector<std::vector<double>> pallet_and_goal_poses);

    int plan();
    std::vector<int> get_task_assignments() { return task_assigner->get_task_assignments(); }

    std::vector<std::vector<std::vector<double>>> get_paths();

    std::vector<std::vector<std::vector<double>>> generate_paths(
        std::vector<int> map_size,
        std::vector<std::vector<double>> robot_poses,
        std::vector<std::vector<double>> pallet_poses,
        std::vector<std::vector<double>> goal_poses,
        std::vector<std::vector<double>> obstacle_poses = {});

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
