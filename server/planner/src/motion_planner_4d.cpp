/**
 * @file motion_planner_4d.cpp
 * @brief Triển khai thuật toán lập quỹ đạo A* 4D (x, y, theta, time).
 */

#include "motion_planner_4d.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <queue>
#include <unordered_set>

MotionPlanner4D::MotionPlanner4D(int robot_index) {
  this->robot_index = robot_index;
}

MotionPlanner4D::~MotionPlanner4D() {}

void MotionPlanner4D::set_start(std::vector<double> start) {
  this->start = start;
  if (this->start[2] < 0) {
    this->start[2] += 2 * M_PI;
  } else if (this->start[2] > 2 * M_PI) {
    this->start[2] -= 2 * M_PI;
  }
}

void MotionPlanner4D::set_goals(std::vector<double> goals) {
  this->pallet_goal = {goals[0], goals[1], goals[2]};
  this->dropoff_goal = {goals[3], goals[4], goals[5]};

  auto clamp_theta = [](double &theta) {
    if (theta < 0)
      theta += 2 * M_PI;
    else if (theta > 2 * M_PI)
      theta -= 2 * M_PI;
  };
  clamp_theta(this->pallet_goal[2]);
  clamp_theta(this->dropoff_goal[2]);
}

void MotionPlanner4D::set_robot_paths(
    std::vector<std::vector<std::vector<double>>> robot_paths) {
  this->paths = robot_paths;
}

void MotionPlanner4D::set_map(std::vector<std::vector<int>> map) {
  this->map = map;
}

std::vector<std::vector<double>> MotionPlanner4D::get_path() {
  return this->path;
}

void MotionPlanner4D::print_path() {
  for (auto &p : this->path) {
    std::cout << "x: " << p[0] << " y: " << p[1] << " theta: " << p[2]
              << " time: " << p[3] << " tag: " << p[4] << std::endl;
  }
  std::cout << std::endl;
}

std::vector<Node *> MotionPlanner4D::get_neighbors(Node *curr_node,
                                                   bool is_pallet_goal) {
  std::vector<Node *> neighbors;

  double theta = curr_node->theta;
  while (theta < 0)
    theta += 2 * M_PI;
  while (theta >= 2 * M_PI)
    theta -= 2 * M_PI;

  int angle_index = (int)std::round(theta / (M_PI / 4)) % 8;

  std::vector<double> goal =
      is_pallet_goal ? this->pallet_goal : this->dropoff_goal;

  for (auto &move : this->angle_to_dxdythetadtimecosts[angle_index]) {
    double dx = move[0];
    double dy = move[1];
    double new_theta = move[2];
    double time = move[3];
    double cost = move[4];

    double h = std::sqrt(std::pow(goal[0] - (curr_node->x + dx), 2) +
                         std::pow(goal[1] - (curr_node->y + dy), 2) +
                         std::pow(goal[2] - new_theta, 2));

    double g = curr_node->g_value + cost;

    if (is_pallet_goal && cost == TURN_COST && h <= NO_TURN_THRESH) {
      continue;
    } else if (h <= NO_TURN_THRESH && cost == STRAIGHT_COST) {
      time = time * PALLET_RUNWAY_SLOWDOWN_FACTOR;
    }

    Node *neighbor =
        new Node(curr_node->x + (int)dx, curr_node->y + (int)dy, new_theta,
                 curr_node->time + time, g, h, curr_node);

    neighbors.push_back(neighbor);
  }

  return neighbors;
}

bool MotionPlanner4D::is_in_collision(Node *node, bool is_pallet_goal) {
  int robot_inflation_radius = is_pallet_goal
                                   ? ROBOT_FOOTPRINT_RADIUS
                                   : ROBOT_FOOTPRINT_RADIUS_WITH_PALLET;

  int x_min = node->x - robot_inflation_radius;
  int x_max = node->x + robot_inflation_radius;
  int y_min = node->y - robot_inflation_radius;
  int y_max = node->y + robot_inflation_radius;

  int pallet_x_min = (int)this->pallet_goal[0] - PALLET_FOOTPRINT_RADIUS;
  int pallet_x_max = (int)this->pallet_goal[0] + PALLET_FOOTPRINT_RADIUS;
  int pallet_y_min = (int)this->pallet_goal[1] - PALLET_FOOTPRINT_RADIUS;
  int pallet_y_max = (int)this->pallet_goal[1] + PALLET_FOOTPRINT_RADIUS;

  if (x_min < 0 || x_max >= (int)this->map.size() || y_min < 0 ||
      y_max >= (int)this->map[0].size()) {
    if (DEBUG)
      std::cout << "[MotionPlanner4D] Robot ra ngoài biên bản đồ" << std::endl;
    return true;
  }

  for (int x = x_min; x <= x_max; x++) {
    for (int y = y_min; y <= y_max; y++) {
      if (this->map[x][y] == 1) {
        if (x < pallet_x_min || x > pallet_x_max || y < pallet_y_min ||
            y > pallet_y_max) {
          return true;
        }

        if (is_pallet_goal) {
          double diff_th = std::abs(node->theta - this->pallet_goal[2]);
          while (diff_th > 2 * M_PI) diff_th -= 2 * M_PI;
          if (diff_th > M_PI) diff_th = 2 * M_PI - diff_th;

          if (diff_th > GOAL_THETA_TOLERANCE) {
            return true;
          }
        }
      }
    }
  }

  for (int i = 0; i < (int)this->paths.size(); i++) {
    if (i == this->robot_index)
      continue;

    for (int j = 0; j < (int)this->paths[i].size(); j++) {
      double other_x = this->paths[i][j][0];
      double other_y = this->paths[i][j][1];
      double other_time = this->paths[i][j][3];
      double other_tag = this->paths[i][j][4];

      bool is_static = (other_tag == 2);
      bool time_overlap =
          is_static || (std::abs(node->time - other_time) <= 5.0);

      if (!time_overlap)
        continue;

      int other_x_min = (int)other_x - robot_inflation_radius;
      int other_x_max = (int)other_x + robot_inflation_radius;
      int other_y_min = (int)other_y - robot_inflation_radius;
      int other_y_max = (int)other_y + robot_inflation_radius;

      bool x_overlap = (x_min <= other_x_max) && (x_max >= other_x_min);
      bool y_overlap = (y_min <= other_y_max) && (y_max >= other_y_min);

      if (x_overlap && y_overlap) {
        return true;
      }
    }
  }

  return false;
}

int MotionPlanner4D::a_star(bool is_pallet_goal) {
  std::vector<Node *> allocated_nodes;

  auto cleanup = [&allocated_nodes]() {
    for (Node *n : allocated_nodes) {
      delete n;
    }
    allocated_nodes.clear();
  };

  std::vector<double> goal =
      is_pallet_goal ? this->pallet_goal : this->dropoff_goal;
  std::vector<double> start_pos =
      is_pallet_goal ? this->start : this->path.back();
  double start_time =
      is_pallet_goal ? 0.0 : (this->path.back()[3] + PALLET_WAIT_TIME);

  std::priority_queue<Node *, std::vector<Node *>, OpenListNodeComparator>
      open_list;
  std::unordered_set<Node *, NodeHasher, ClosedListNodeComparator> closed_list;

  double start_h = std::sqrt(std::pow(goal[0] - start_pos[0], 2) +
                             std::pow(goal[1] - start_pos[1], 2) +
                             std::pow(goal[2] - start_pos[2], 2));

  Node *start_node = new Node((int)start_pos[0], (int)start_pos[1],
                              start_pos[2], start_time, 0.0, start_h, nullptr);
  allocated_nodes.push_back(start_node);

  Node *goal_node =
      new Node((int)goal[0], (int)goal[1], goal[2], 0.0, 0.0, 0.0, nullptr);
  allocated_nodes.push_back(goal_node);

  std::cout << "[MotionPlanner4D] Điểm bắt đầu: " << start_node->x << ", " << start_node->y << ", "
            << start_node->theta << ", " << start_node->time << ", "
            << start_node->g_value << ", " << start_node->h_value << std::endl;
  std::cout << "[MotionPlanner4D] Điểm đích: " << goal_node->x << ", " << goal_node->y << ", "
            << goal_node->theta << std::endl;

  if (this->is_in_collision(goal_node, is_pallet_goal) && !is_pallet_goal) {
    std::cout << "[MotionPlanner4D] Điểm đích bị va chạm" << std::endl;
    cleanup();
    return -1;
  }

  open_list.push(start_node);

  std::chrono::steady_clock::time_point begin =
      std::chrono::steady_clock::now();

  while (!open_list.empty()) {
    Node *current_node = open_list.top();
    open_list.pop();

    while (closed_list.find(current_node) != closed_list.end()) {
      if (open_list.empty())
        goto search_done;
      current_node = open_list.top();
      open_list.pop();
    }

    closed_list.insert(current_node);

    if (std::abs(current_node->x - goal_node->x) <= 1 &&
        std::abs(current_node->y - goal_node->y) <= 1) {
      double diff_th = std::abs(current_node->theta - goal_node->theta);
      while (diff_th > 2 * M_PI) diff_th -= 2 * M_PI;
      if (diff_th > M_PI) diff_th = 2 * M_PI - diff_th;

      if (diff_th <= GOAL_THETA_TOLERANCE) {
        goal_node = current_node;
        break;
      }
    }

    std::vector<Node *> neighbors = get_neighbors(current_node, is_pallet_goal);

    for (Node *neighbor : neighbors) {
      allocated_nodes.push_back(neighbor);

      if (this->is_in_collision(neighbor, is_pallet_goal))
        continue;

      if (closed_list.find(neighbor) == closed_list.end()) {
        open_list.push(neighbor);
      }
    }

    std::chrono::steady_clock::time_point end =
      std::chrono::steady_clock::now();
    if (std::chrono::duration_cast<std::chrono::seconds>(end - begin).count() >
        MAX_SEARCH_TIME) {
      std::cout << "[MotionPlanner4D] Quá thời gian tìm kiếm tối đa" << std::endl;
      break;
    }
  }

search_done:
  if (goal_node->parent == nullptr) {
    std::cout << "[MotionPlanner4D] Không tìm thấy quỹ đạo hợp lệ" << std::endl;
    cleanup();
    return -1;
  }

  std::vector<std::vector<double>> sub_path;
  Node *current_node = goal_node;
  while (current_node != nullptr) {
    sub_path.push_back({
        (double)(current_node->x), (double)(current_node->y),
        current_node->theta, current_node->time,
        0.0
    });
    current_node = current_node->parent;
  }

  std::reverse(sub_path.begin(), sub_path.end());

  sub_path.back()[4] = is_pallet_goal ? 1.0 : 2.0;

  for (auto &wp : sub_path) {
    this->path.push_back(wp);
  }

  cleanup();
  return 0;
}

int MotionPlanner4D::plan() {
  this->path = {};

  if (this->a_star(true) == -1) {
    std::cout << "[MotionPlanner4D] Không tìm thấy quỹ đạo tới pallet" << std::endl;
    return -1;
  }
  std::cout << "[MotionPlanner4D] Đã tìm thấy quỹ đạo tới pallet" << std::endl;

  if (this->a_star(false) == -1) {
    std::cout << "[MotionPlanner4D] Không tìm thấy quỹ đạo tới điểm trả" << std::endl;
    return -1;
  }
  std::cout << "[MotionPlanner4D] Đã tìm thấy quỹ đạo tới điểm trả" << std::endl;

  return 0;
}
