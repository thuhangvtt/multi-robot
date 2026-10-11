/**
 * @file planner_coordinator.cpp
 * @brief Triển khai điều phối lập lịch đường đi đa robot.
 *
 * Kế thừa từ planner.cc và main.cc gốc.
 *
 * SỬA LỖI 3 (Global State):
 *   - PlannerCoordinator là class thông thường, không có biến toàn cục.
 *   - Destructor đầy đủ: giải phóng tất cả MotionPlanner4D* và TaskAssigner*.
 *
 * Giữ nguyên:
 *   - Chuyển đổi đơn vị: mm/10 -> cm (input), cm*10 -> mm (output)
 *   - Chuẩn hóa theta -> [-pi, pi] bằng atan2(sin, cos) ở output
 *   - Lập lịch tuần tự: robot 0 -> robot 1 -> ...
 *   - Khởi tạo default paths (pose hiện tại, tag=2) trước khi plan
 *   - Generate map: lưới (length/10) x (width/10), gán ô cản 3x3 quanh pallet
 */

#include "planner_coordinator.hpp"
#include <chrono>

// =============================================================================
// generate_map(): Tạo bản đồ lưới 2D (1cm/cell).
// Kế thừa từ generate_map() trong main.cc gốc.
// =============================================================================
std::vector<std::vector<int>> generate_map(
    int map_length_mm,
    int map_width_mm,
    std::vector<std::vector<double>> pallet_poses)
{
    std::vector<std::vector<int>> map(
        map_length_mm / 10,
        std::vector<int>(map_width_mm / 10, 0));

    // Gán vùng cản 3x3 (±1 ô) quanh mỗi pallet — giữ nguyên so với bản gốc
    for (auto &p : pallet_poses)
    {
        int x = (int)(p[0] / 10);
        int y = (int)(p[1] / 10);

        for (int i = -1; i <= 1; i++)
        {
            for (int j = -1; j <= 1; j++)
            {
                int xi = x + i;
                int yj = y + j;
                if (xi >= 0 && xi < (int)map.size() &&
                    yj >= 0 && yj < (int)map[0].size())
                {
                    map[xi][yj] = 1;
                }
            }
        }
    }

    return map;
}

// =============================================================================
// Constructor
// =============================================================================
PlannerCoordinator::PlannerCoordinator(int num_robots)
{
    this->num_robots = num_robots;

    for (int i = 0; i < num_robots; i++)
    {
        this->motion_planners.push_back(new MotionPlanner4D(i));
    }

    this->task_assigner = new TaskAssigner(num_robots);
}

// =============================================================================
// Destructor — SỬA LỖI: Giải phóng tất cả đối tượng con
// =============================================================================
PlannerCoordinator::~PlannerCoordinator()
{
    for (auto *mp : this->motion_planners)
    {
        delete mp;
    }
    this->motion_planners.clear();

    delete this->task_assigner;
    this->task_assigner = nullptr;
}

void PlannerCoordinator::set_map(std::vector<std::vector<int>> map)
{
    this->map = map;
    for (int i = 0; i < this->num_robots; i++)
    {
        this->motion_planners[i]->set_map(map);
    }
}

// =============================================================================
// set_robot_poses(): Chuyển mm -> cm (chia 10, làm tròn), giữ nguyên theta.
// Giữ nguyên 100% logic chuyển đổi so với bản gốc.
// =============================================================================
void PlannerCoordinator::set_robot_poses(std::vector<std::vector<double>> robot_poses)
{
    this->robot_poses.clear();

    for (int i = 0; i < (int)robot_poses.size(); i++)
    {
        std::vector<double> pose;
        // Chuyển x, y từ mm sang cm (bỏ phần tử cuối = theta)
        for (int j = 0; j < (int)robot_poses[i].size() - 1; j++)
        {
            pose.push_back(std::round(robot_poses[i][j] / 10.0));
        }
        // Theta giữ nguyên
        pose.push_back(robot_poses[i].back());
        this->robot_poses.push_back(pose);
    }
}

// =============================================================================
// set_pallet_and_goal_poses(): Chuyển x,y từ mm->cm, giữ nguyên theta (index 2, 5).
// Giữ nguyên 100% logic chuyển đổi so với bản gốc.
// =============================================================================
void PlannerCoordinator::set_pallet_and_goal_poses(
    std::vector<std::vector<double>> pallet_and_goal_poses)
{
    this->pallet_and_goal_poses.clear();

    for (int i = 0; i < (int)pallet_and_goal_poses.size(); i++)
    {
        std::vector<double> pose;
        for (int j = 0; j < (int)pallet_and_goal_poses[i].size(); j++)
        {
            if (j == 2 || j == 5)
            {
                // Giữ nguyên theta của pallet và goal
                pose.push_back(pallet_and_goal_poses[i][j]);
            }
            else
            {
                // Chuyển tọa độ x, y từ mm sang cm
                pose.push_back(std::round(pallet_and_goal_poses[i][j] / 10.0));
            }
        }
        this->pallet_and_goal_poses.push_back(pose);
    }

    // Debug print
    std::cout << "pallet_and_goal_poses (cm): " << std::endl;
    for (auto &pose : this->pallet_and_goal_poses)
    {
        for (auto &v : pose) std::cout << v << " ";
        std::cout << std::endl;
    }
}

// =============================================================================
// plan(): Lập lịch tuần tự cho tất cả robot.
// Giữ nguyên 100% logic lập lịch so với bản gốc.
// =============================================================================
int PlannerCoordinator::plan()
{
    // Gán việc cho robot
    this->task_assigner->assign_pallets_to_robots(
        this->robot_poses, this->pallet_and_goal_poses);
    std::vector<int> task_assignments = this->task_assigner->get_task_assignments();

    // Khởi tạo paths mặc định (pose hiện tại, tag=2) để làm vật cản tĩnh ban đầu
    this->paths = std::vector<std::vector<std::vector<double>>>(this->num_robots);
    for (int i = 0; i < this->num_robots; i++)
    {
        this->paths[i] = {{
            this->robot_poses[i][0],
            this->robot_poses[i][1],
            this->robot_poses[i][2],
            0.0, 2.0
        }};
    }

    // Lập lịch tuần tự: robot 0 trước, robot sau xem path trước là vật cản động
    for (int i = 0; i < this->num_robots; i++)
    {
        std::cout << "Planning for robot " << i << std::endl;
        std::cout << "Current robot pose: ";
        for (auto v : this->robot_poses[i]) std::cout << v << " ";
        std::cout << std::endl;

        if (task_assignments[i] == -1)
        {
            std::cout << "No task assigned to robot " << i << std::endl;
            continue;
        }

        std::cout << "Current pallet and goal pose: ";
        for (auto v : this->pallet_and_goal_poses[task_assignments[i]])
            std::cout << v << " ";
        std::cout << std::endl;

        this->motion_planners[i]->set_start(this->robot_poses[i]);
        this->motion_planners[i]->set_goals(this->pallet_and_goal_poses[task_assignments[i]]);
        this->motion_planners[i]->set_robot_paths(this->paths);

        if (this->motion_planners[i]->plan() == -1)
        {
            std::cout << "Planning failed for robot " << i << std::endl;
            this->task_assigner->unassign_pallet_from_robot(i);
            task_assignments = this->task_assigner->get_task_assignments();
        }
        else
        {
            std::cout << "Planning successful for robot " << i << std::endl;
            this->paths[i] = this->motion_planners[i]->get_path();
        }
    }

    // Xóa paths của robot không có việc
    for (int i = 0; i < this->num_robots; i++)
    {
        if (task_assignments[i] == -1)
        {
            this->paths[i].clear();
        }
    }

    return 0;
}

// =============================================================================
// get_paths(): Trả về quỹ đạo theo đơn vị mm và theta trong [-pi, pi].
// Giữ nguyên 100% logic chuyển đổi đầu ra so với bản gốc.
// =============================================================================
std::vector<std::vector<std::vector<double>>> PlannerCoordinator::get_paths()
{
    std::vector<std::vector<std::vector<double>>> result;

    for (int i = 0; i < (int)this->paths.size(); i++)
    {
        std::vector<std::vector<double>> robot_path;
        for (int j = 0; j < (int)this->paths[i].size(); j++)
        {
            std::vector<double> waypoint;
            for (int k = 0; k < (int)this->paths[i][j].size(); k++)
            {
                if (k == 0 || k == 1)
                {
                    // Nhân 10 để đổi từ cm về mm
                    waypoint.push_back(this->paths[i][j][k] * 10.0);
                }
                else if (k == 2)
                {
                    // Chuẩn hóa theta từ [0, 2pi] về [-pi, pi]
                    waypoint.push_back(std::atan2(
                        std::sin(this->paths[i][j][k]),
                        std::cos(this->paths[i][j][k])));
                }
                else
                {
                    // time và tag giữ nguyên
                    waypoint.push_back(this->paths[i][j][k]);
                }
            }
            robot_path.push_back(waypoint);
        }
        result.push_back(robot_path);
    }

    return result;
}

// =============================================================================
// generate_paths(): Hàm tiện ích gộp pipeline đầy đủ.
// =============================================================================
std::vector<std::vector<std::vector<double>>> PlannerCoordinator::generate_paths(
    std::vector<int> map_size,
    std::vector<std::vector<double>> robot_poses,
    std::vector<std::vector<double>> pallet_poses,
    std::vector<std::vector<double>> goal_poses)
{
    // 1. Sinh bản đồ lưới từ kích thước sandbox và vị trí pallet
    std::vector<std::vector<int>> map = generate_map(map_size[0], map_size[1], pallet_poses);
    this->set_map(map);

    // 2. Đặt vị trí robot (mm -> cm tự động trong set_robot_poses)
    std::cout << "Robot poses from generate_paths:" << std::endl;
    for (auto &pose : robot_poses)
        std::cout << pose[0] << ", " << pose[1] << ", " << pose[2] << std::endl;

    this->set_robot_poses(robot_poses);

    // 3. Ghép pallet và goal poses thành vector kết hợp [px, py, ptheta, gx, gy, gtheta]
    std::vector<std::vector<double>> pallet_and_goal_poses;
    for (int i = 0; i < (int)pallet_poses.size(); i++)
    {
        pallet_and_goal_poses.push_back({
            pallet_poses[i][0], pallet_poses[i][1], pallet_poses[i][2],
            goal_poses[i][0],   goal_poses[i][1],   goal_poses[i][2]
        });
    }
    this->set_pallet_and_goal_poses(pallet_and_goal_poses);

    // 4. Lập kế hoạch
    this->plan();

    // 5. Trả về quỹ đạo (đơn vị mm, theta [-pi, pi])
    return this->get_paths();
}

void PlannerCoordinator::print_paths()
{
    for (int i = 0; i < this->num_robots; i++)
    {
        std::cout << "Path for robot " << i << ":" << std::endl;
        this->motion_planners[i]->print_path();
    }
}
