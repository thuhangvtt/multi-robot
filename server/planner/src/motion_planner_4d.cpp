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

  double nearest_grid_theta = std::round(this->start[2] / (M_PI / 4.0)) * (M_PI / 4.0);
  if (std::abs(this->start[2] - nearest_grid_theta) <= (M_PI / 12.0)) {
    this->start[2] = nearest_grid_theta;
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

    if (cost == TURN_COST && curr_node->parent != nullptr) {
      double prev_dth = std::atan2(
          std::sin(curr_node->theta - curr_node->parent->theta),
          std::cos(curr_node->theta - curr_node->parent->theta));
      if (std::abs(prev_dth) > 1e-3) {
        double back_dth = std::atan2(
            std::sin(new_theta - curr_node->parent->theta),
            std::cos(new_theta - curr_node->parent->theta));
        if (std::abs(back_dth) < 1e-3) {
          g += 10.0;
        }
      }
    }

    if (is_pallet_goal && curr_node->parent == nullptr && cost == TURN_COST) {
      continue;
    }
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
      std::cout << "[MotionPlanner4D] Robot ra ngoai bien ban do" << std::endl;
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

  int search_goal_x = (int)goal[0];
  int search_goal_y = (int)goal[1];
  double chosen_runway_cm = 0.0;
  if (is_pallet_goal) {
    for (int runway_cm : {20, 16, 12, 8, 5}) {
      int rx = (int)std::lround(goal[0] - runway_cm * std::cos(goal[2]));
      int ry = (int)std::lround(goal[1] + runway_cm * std::sin(goal[2]));
      Node probe_start(rx, ry, goal[2], 0.0, 0.0, 0.0, nullptr);
      bool runway_clear = !this->is_in_collision(&probe_start, true);
      int entry_margin = (runway_cm >= 12) ? 8 : 4;
      for (int d = 4; runway_clear && d <= runway_cm + entry_margin; d += 4) {
        int px = (int)std::lround(goal[0] - d * std::cos(goal[2]));
        int py = (int)std::lround(goal[1] + d * std::sin(goal[2]));
        Node probe_pt(px, py, goal[2], 0.0, 0.0, 0.0, nullptr);
        if (this->is_in_collision(&probe_pt, true)) {
          runway_clear = false;
        }
      }
      if (runway_clear) {
        search_goal_x = rx;
        search_goal_y = ry;
        chosen_runway_cm = (double)runway_cm;
        break;
      }
    }
  }

  double start_h = std::sqrt(std::pow(search_goal_x - start_pos[0], 2) +
                             std::pow(search_goal_y - start_pos[1], 2) +
                             std::pow(goal[2] - start_pos[2], 2));

  Node *start_node = new Node((int)start_pos[0], (int)start_pos[1],
                              start_pos[2], start_time, 0.0, start_h, nullptr);
  allocated_nodes.push_back(start_node);

  Node *goal_node =
      new Node(search_goal_x, search_goal_y, goal[2], 0.0, 0.0, 0.0, nullptr);
  allocated_nodes.push_back(goal_node);

  std::cout << "[MotionPlanner4D] Diem bat dau: " << start_node->x << ", " << start_node->y << ", "
            << start_node->theta << ", " << start_node->time << ", "
            << start_node->g_value << ", " << start_node->h_value << std::endl;
  std::cout << "[MotionPlanner4D] Diem dich: " << goal_node->x << ", " << goal_node->y << ", "
            << goal_node->theta << std::endl;

  if (this->is_in_collision(goal_node, is_pallet_goal) && !is_pallet_goal) {
    std::cout << "[MotionPlanner4D] Diem dich bi va cham" << std::endl;
    cleanup();
    return -1;
  }

  Node *search_start = start_node;
  if (is_pallet_goal) {
    // Lead-in: drive straight along the real robot heading before A* may turn.
    double lead_dx = std::cos(start_pos[2]);
    double lead_dy = -std::sin(start_pos[2]);
    for (int k = 1; k <= LEAD_IN_LENGTH_CM; k++) {
      int lx = (int)std::lround(start_pos[0] + lead_dx * k);
      int ly = (int)std::lround(start_pos[1] + lead_dy * k);
      if (lx == search_start->x && ly == search_start->y) continue;
      double lh = std::sqrt(std::pow(search_goal_x - lx, 2) +
                            std::pow(search_goal_y - ly, 2) +
                            std::pow(goal[2] - start_pos[2], 2));
      Node *lead = new Node(lx, ly, start_pos[2],
                            start_time + k * straight_time_step, 0.0, lh,
                            search_start);
      allocated_nodes.push_back(lead);
      if (this->is_in_collision(lead, is_pallet_goal)) break;
      search_start = lead;
    }
  }
  open_list.push(search_start);
  if (search_start != start_node) {
    Node *fallback_start = new Node((int)start_pos[0], (int)start_pos[1],
                                    start_pos[2], start_time, 8.0, start_h, nullptr);
    allocated_nodes.push_back(fallback_start);
    open_list.push(fallback_start);
  }

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

    if (std::abs(current_node->x - search_goal_x) <= 1 &&
        std::abs(current_node->y - search_goal_y) <= 1) {
      double diff_th = std::abs(current_node->theta - goal[2]);
      while (diff_th > 2 * M_PI) diff_th -= 2 * M_PI;
      if (diff_th > M_PI) diff_th = 2 * M_PI - diff_th;

      if (!is_pallet_goal || diff_th <= GOAL_THETA_TOLERANCE) {
        if (is_pallet_goal) {
          current_node->theta = goal[2];
          Node *runway_prev = current_node;
          int final_x = (int)std::lround(goal[0]);
          int final_y = (int)std::lround(goal[1]);
          if (final_x != runway_prev->x || final_y != runway_prev->y) {
            int steps = std::max(1, (int)std::lround(chosen_runway_cm / 5.0));
            double dt_step =
                (std::max(chosen_runway_cm, 1.0) / steps) * straight_time_step * PALLET_RUNWAY_SLOWDOWN_FACTOR;
            int sx = runway_prev->x;
            int sy = runway_prev->y;
            for (int s = 1; s <= steps; s++) {
              double frac = (double)s / (double)steps;
              int nx = (int)std::lround(sx + frac * (final_x - sx));
              int ny = (int)std::lround(sy + frac * (final_y - sy));
              if (s < steps && (nx == runway_prev->x && ny == runway_prev->y)) {
                continue;
              }
              if (s < steps && (nx == final_x && ny == final_y)) {
                continue;
              }
              Node *rnode = new Node(nx, ny, goal[2],
                                     runway_prev->time + dt_step,
                                     runway_prev->g_value, 0.0, runway_prev);
              allocated_nodes.push_back(rnode);
              runway_prev = rnode;
            }
            goal_node = runway_prev;
          } else {
            Node *dock_node = new Node(final_x, final_y, goal[2],
                                       runway_prev->time,
                                       runway_prev->g_value, 0.0, runway_prev);
            allocated_nodes.push_back(dock_node);
            goal_node = dock_node;
          }
        } else {
          goal_node = current_node;
        }
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
      std::cout << "[MotionPlanner4D] Qua thoi gian tim kiem toi da" << std::endl;
      break;
    }
  }

search_done:
  if (goal_node->parent == nullptr) {
    std::cout << "[MotionPlanner4D] Khong tim thay quy dao hop le" << std::endl;
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

  if (is_pallet_goal && sub_path.size() >= 3) {
    sub_path[sub_path.size() - 3][2] = goal[2];
    sub_path[sub_path.size() - 2][2] = goal[2];
    sub_path.back()[2] = goal[2];
  } else if (is_pallet_goal && sub_path.size() >= 2) {
    sub_path[sub_path.size() - 2][2] = goal[2];
    sub_path.back()[2] = goal[2];
  }

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
    std::cout << "[MotionPlanner4D] Khong tim thay quy dao toi pallet" << std::endl;
    return -1;
  }
  std::cout << "[MotionPlanner4D] Da tim thay quy dao toi pallet" << std::endl;

  if (this->a_star(false) == -1) {
    std::cout << "[MotionPlanner4D] Khong tim thay quy dao toi diem tra" << std::endl;
    return -1;
  }
  std::cout << "[MotionPlanner4D] Da tim thay quy dao toi diem tra" << std::endl;

  return 0;
}
