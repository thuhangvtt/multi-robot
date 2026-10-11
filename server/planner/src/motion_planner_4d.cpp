/**
 * @file motion_planner_4d.cpp
 * @brief Triển khai A* 4D search engine cho từng robot.
 *
 * Kế thừa 100% thuật toán từ motion_planner.cc gốc.
 *
 * SỬA LỖI 1 — Memory Leak:
 *   Bản gốc: new Node() được gọi hàng ngàn lần trong a_star() và get_neighbors()
 *             nhưng chưa bao giờ bị delete -> RAM tăng vô hạn khi chạy liên tục.
 *   Bản sửa: std::vector<Node*> allocated_nodes được khai báo tại đầu a_star().
 *             MỌI Node* tạo ra đều được push vào danh sách này.
 *             Trước BẤT KỲ return nào (thành công, thất bại, timeout),
 *             toàn bộ Node* trong allocated_nodes đều bị delete và danh sách được xóa.
 *
 *   Lưu ý: Sau khi giải phóng, các con trỏ trong open_list và closed_list
 *           trở thành dangling pointer — đây là điều bình thường vì các cấu trúc
 *           đó đều là stack variable và sẽ bị hủy cùng lúc khi a_star() return.
 */

#include "motion_planner_4d.hpp"
#include <queue>
#include <unordered_set>
#include <algorithm>
#include <cmath>
#include <chrono>

MotionPlanner4D::MotionPlanner4D(int robot_index)
{
    this->robot_index = robot_index;
}

MotionPlanner4D::~MotionPlanner4D()
{
}

void MotionPlanner4D::set_start(std::vector<double> start)
{
    this->start = start;
    // Chuẩn hóa theta về [0, 2pi] — giữ nguyên so với bản gốc
    if (this->start[2] < 0)
    {
        this->start[2] += 2 * M_PI;
    }
    else if (this->start[2] > 2 * M_PI)
    {
        this->start[2] -= 2 * M_PI;
    }
}

void MotionPlanner4D::set_goals(std::vector<double> goals)
{
    this->pallet_goal  = {goals[0], goals[1], goals[2]};
    this->dropoff_goal = {goals[3], goals[4], goals[5]};

    // Chuẩn hóa theta về [0, 2pi] cho cả hai goal — giữ nguyên so với bản gốc
    auto clamp_theta = [](double &theta) {
        if (theta < 0)        theta += 2 * M_PI;
        else if (theta > 2 * M_PI) theta -= 2 * M_PI;
    };
    clamp_theta(this->pallet_goal[2]);
    clamp_theta(this->dropoff_goal[2]);
}

void MotionPlanner4D::set_robot_paths(std::vector<std::vector<std::vector<double>>> robot_paths)
{
    this->paths = robot_paths;
}

void MotionPlanner4D::set_map(std::vector<std::vector<int>> map)
{
    this->map = map;
}

std::vector<std::vector<double>> MotionPlanner4D::get_path()
{
    return this->path;
}

void MotionPlanner4D::print_path()
{
    for (auto &p : this->path)
    {
        std::cout << "x: " << p[0] << " y: " << p[1]
                  << " theta: " << p[2] << " time: " << p[3]
                  << " tag: " << p[4] << std::endl;
    }
    std::cout << std::endl;
}

// =============================================================================
// get_neighbors(): Sinh danh sách neighbor theo bảng động học 8 góc.
// Giữ nguyên 100% so với bản gốc.
// CHÚ Ý: Mọi Node* tạo ra tại đây PHẢI được push vào allocated_nodes trong a_star().
// =============================================================================
std::vector<Node *> MotionPlanner4D::get_neighbors(Node *curr_node, bool is_pallet_goal)
{
    std::vector<Node *> neighbors;

    double theta = curr_node->theta;

    // Chuẩn hóa theta về [0, 2pi] rồi lượng tử hóa thành 8 hướng rời rạc
    while (theta < 0)         theta += 2 * M_PI;
    while (theta >= 2 * M_PI) theta -= 2 * M_PI;

    int angle_index = (int)std::round(theta / (M_PI / 4)) % 8;

    std::vector<double> goal = is_pallet_goal ? this->pallet_goal : this->dropoff_goal;

    for (auto &move : this->angle_to_dxdythetadtimecosts[angle_index])
    {
        double dx    = move[0];
        double dy    = move[1];
        double new_theta = move[2];
        double time  = move[3];
        double cost  = move[4];

        // Heuristic Euclidean 3D: sqrt((gx-x)^2 + (gy-y)^2 + (gtheta-theta)^2)
        double h = std::sqrt(
            std::pow(goal[0] - (curr_node->x + dx), 2) +
            std::pow(goal[1] - (curr_node->y + dy), 2) +
            std::pow(goal[2] - new_theta, 2));

        double g = curr_node->g_value + cost;

        // Giảm tốc khi lại gần pallet goal — giữ nguyên logic gốc
        if (is_pallet_goal && cost == TURN_COST && h <= NO_TURN_THRESH)
        {
            continue; // Không rẽ khi đã gần
        }
        else if (h <= NO_TURN_THRESH && cost == STRAIGHT_COST)
        {
            time = time * PALLET_RUNWAY_SLOWDOWN_FACTOR;
        }

        Node *neighbor = new Node(
            curr_node->x + (int)dx,
            curr_node->y + (int)dy,
            new_theta,
            curr_node->time + time,
            g, h,
            curr_node);

        neighbors.push_back(neighbor);
    }

    return neighbors;
}

// =============================================================================
// is_in_collision(): Kiểm tra va chạm tĩnh và động.
// Giữ nguyên 100% so với bản gốc.
// =============================================================================
bool MotionPlanner4D::is_in_collision(Node *node, bool is_pallet_goal)
{
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

    int robot_offset_x = (int)(ROBOT_OFFSET * std::cos(node->theta)) + node->x;
    int robot_offset_y = (int)(ROBOT_OFFSET * std::sin(node->theta)) + node->y;

    int robot_front_x_min = robot_offset_x - PALLET_FOOTPRINT_RADIUS;
    int robot_front_x_max = robot_offset_x + PALLET_FOOTPRINT_RADIUS;
    int robot_front_y_min = robot_offset_y - PALLET_FOOTPRINT_RADIUS;
    int robot_front_y_max = robot_offset_y + PALLET_FOOTPRINT_RADIUS;

    // Kiểm tra giới hạn map
    if (x_min < 0 || x_max >= (int)this->map.size() ||
        y_min < 0 || y_max >= (int)this->map[0].size())
    {
        if (DEBUG) std::cout << "Robot is out of bounds" << std::endl;
        return true;
    }

    // Kiểm tra va chạm tĩnh với vật cản
    for (int x = x_min; x <= x_max; x++)
    {
        for (int y = y_min; y <= y_max; y++)
        {
            if (this->map[x][y] == 1)
            {
                // Nếu vật cản nằm ngoài vùng pallet được gán -> va chạm
                if (x < pallet_x_min || x > pallet_x_max ||
                    y < pallet_y_min || y > pallet_y_max)
                {
                    return true;
                }

                // Đây là vật cản pallet được gán của robot này.
                // Chỉ cho phép khi đang đến lấy pallet (is_pallet_goal=true)
                // và góc hướng vào đúng trong dung sai.
                if (is_pallet_goal)
                {
                    if (std::abs(node->theta - this->pallet_goal[2]) > GOAL_THETA_TOLERANCE)
                    {
                        return true;
                    }
                }
            }
        }
    }

    // Kiểm tra va chạm động với các robot khác (cửa sổ thời gian 5 giây)
    for (int i = 0; i < (int)this->paths.size(); i++)
    {
        // Bỏ qua path của chính robot này
        if (i == this->robot_index) continue;

        for (int j = 0; j < (int)this->paths[i].size(); j++)
        {
            double other_x    = this->paths[i][j][0];
            double other_y    = this->paths[i][j][1];
            double other_time = this->paths[i][j][3];
            double other_tag  = this->paths[i][j][4];

            // Robot đang dừng tại goal (tag=2) -> vật cản tĩnh vĩnh viễn
            bool is_static = (other_tag == 2);

            // Kiểm tra cửa sổ thời gian 5 giây
            bool time_overlap = is_static ||
                (std::abs(node->time - other_time) <= 5.0);

            if (!time_overlap) continue;

            // Kiểm tra overlap bounding box footprint
            int other_x_min = (int)other_x - robot_inflation_radius;
            int other_x_max = (int)other_x + robot_inflation_radius;
            int other_y_min = (int)other_y - robot_inflation_radius;
            int other_y_max = (int)other_y + robot_inflation_radius;

            bool x_overlap = (x_min <= other_x_max) && (x_max >= other_x_min);
            bool y_overlap = (y_min <= other_y_max) && (y_max >= other_y_min);

            if (x_overlap && y_overlap)
            {
                return true;
            }
        }
    }

    return false;
}

// =============================================================================
// a_star(): A* 4D tìm đường từ start đến pallet hoặc goal.
//
// SỬA LỖI 1 — Memory Leak:
//   Tất cả Node* tạo ra trong quá trình tìm kiếm được theo dõi trong allocated_nodes.
//   Trước khi return (dù thành công, thất bại hay timeout), toàn bộ được delete.
//
// Giữ nguyên 100%:
//   - Open list (priority_queue), closed list (unordered_set)
//   - Điều kiện thoát: đến đúng (x,y) và theta trong dung sai pi/8
//   - Timeout MAX_SEARCH_TIME seconds
//   - Truy vết đường đi bằng chuỗi parent
// =============================================================================
int MotionPlanner4D::a_star(bool is_pallet_goal)
{
    // -------------------------------------------------------------------------
    // SỬA LỖI 1: Danh sách quản lý bộ nhớ cho toàn bộ lần tìm kiếm này
    // -------------------------------------------------------------------------
    std::vector<Node *> allocated_nodes;

    // Helper giải phóng bộ nhớ — được gọi trước MỌI điểm return
    auto cleanup = [&allocated_nodes]() {
        for (Node *n : allocated_nodes) {
            delete n;
        }
        allocated_nodes.clear();
    };

    // Set goal và start theo pha tìm kiếm
    std::vector<double> goal = is_pallet_goal ? this->pallet_goal : this->dropoff_goal;
    std::vector<double> start_pos = is_pallet_goal ? this->start : this->path.back();
    double start_time = is_pallet_goal ? 0.0 : (this->path.back()[3] + PALLET_WAIT_TIME);

    // Tạo start node và goal node
    std::priority_queue<Node *, std::vector<Node *>, OpenListNodeComparator> open_list;
    std::unordered_set<Node *, NodeHasher, ClosedListNodeComparator> closed_list;

    double start_h = std::sqrt(
        std::pow(goal[0] - start_pos[0], 2) +
        std::pow(goal[1] - start_pos[1], 2) +
        std::pow(goal[2] - start_pos[2], 2));

    Node *start_node = new Node(
        (int)start_pos[0], (int)start_pos[1], start_pos[2],
        start_time, 0.0, start_h, nullptr);
    allocated_nodes.push_back(start_node);

    Node *goal_node = new Node(
        (int)goal[0], (int)goal[1], goal[2],
        0.0, 0.0, 0.0, nullptr);
    allocated_nodes.push_back(goal_node);

    std::cout << "Start Node: " << start_node->x << ", " << start_node->y
              << ", " << start_node->theta << ", " << start_node->time
              << ", " << start_node->g_value << ", " << start_node->h_value << std::endl;
    std::cout << "Goal Node:  " << goal_node->x << ", " << goal_node->y
              << ", " << goal_node->theta << std::endl;

    // Kiểm tra goal có hợp lệ không (chỉ cho drop-off goal)
    if (this->is_in_collision(goal_node, is_pallet_goal) && !is_pallet_goal)
    {
        std::cout << "Goal is in collision" << std::endl;
        cleanup();  // SỬA LỖI 1: Giải phóng trước khi return
        return -1;
    }

    open_list.push(start_node);

    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

    while (!open_list.empty())
    {
        Node *current_node = open_list.top();
        open_list.pop();

        // Bỏ qua node đã nằm trong closed list
        while (closed_list.find(current_node) != closed_list.end())
        {
            if (open_list.empty()) goto search_done;
            current_node = open_list.top();
            open_list.pop();
        }

        closed_list.insert(current_node);

        // Kiểm tra điều kiện đạt goal: trùng (x, y) và theta trong dung sai pi/8
        if (current_node->x == goal_node->x && current_node->y == goal_node->y)
        {
            if (std::abs(current_node->theta - goal_node->theta) <= GOAL_THETA_TOLERANCE)
            {
                goal_node = current_node;
                break;
            }
        }

        // Sinh neighbors và đưa vào open list
        std::vector<Node *> neighbors = get_neighbors(current_node, is_pallet_goal);

        for (Node *neighbor : neighbors)
        {
            // SỬA LỖI 1: Theo dõi toàn bộ Node* tạo ra từ get_neighbors
            allocated_nodes.push_back(neighbor);

            if (this->is_in_collision(neighbor, is_pallet_goal)) continue;

            if (closed_list.find(neighbor) == closed_list.end())
            {
                open_list.push(neighbor);
            }
        }

        // Kiểm tra timeout
        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::seconds>(end - begin).count() > MAX_SEARCH_TIME)
        {
            std::cout << "Search time exceeded" << std::endl;
            break;
        }
    }

search_done:
    // Kiểm tra có tìm được đường không
    if (goal_node->parent == nullptr)
    {
        std::cout << "Could not find the path" << std::endl;
        cleanup();  // SỬA LỖI 1: Giải phóng trước khi return -1
        return -1;
    }

    // Truy vết đường đi bằng chuỗi parent
    std::vector<std::vector<double>> sub_path;
    Node *current_node = goal_node;
    while (current_node != nullptr)
    {
        sub_path.push_back({
            (double)(current_node->x),
            (double)(current_node->y),
            current_node->theta,
            current_node->time,
            0.0  // tag tạm thời
        });
        current_node = current_node->parent;
    }

    std::reverse(sub_path.begin(), sub_path.end());

    // Gán tag cho waypoint cuối: 1=tại pallet, 2=tại goal
    sub_path.back()[4] = is_pallet_goal ? 1.0 : 2.0;

    // Nối sub_path vào quỹ đạo chính
    for (auto &wp : sub_path)
    {
        this->path.push_back(wp);
    }

    cleanup();  // SỬA LỖI 1: Giải phóng sau khi đã trích xuất đường đi
    return 0;
}

// =============================================================================
// plan(): Chạy A* 2 pha: start -> pallet, rồi pallet -> drop-off goal.
// Giữ nguyên 100% so với bản gốc.
// =============================================================================
int MotionPlanner4D::plan()
{
    // Reset đường đi trước mỗi lần plan
    this->path = {};

    // Pha 1: Đến gắp pallet
    if (this->a_star(true) == -1)
    {
        std::cout << "No path found for pallet goal" << std::endl;
        return -1;
    }
    std::cout << "Pallet goal path found" << std::endl;

    // Pha 2: Giao pallet đến drop-off goal (bắt đầu từ vị trí cuối pha 1 + PALLET_WAIT_TIME)
    if (this->a_star(false) == -1)
    {
        std::cout << "No path found for dropoff goal" << std::endl;
        return -1;
    }
    std::cout << "Dropoff goal path found" << std::endl;

    return 0;
}
