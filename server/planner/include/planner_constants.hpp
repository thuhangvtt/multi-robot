/**
 * @file planner_constants.hpp
 * @brief Toàn bộ hằng số cấu hình cho thuật toán A* 4D và thông số robot/pallet.
 *        Kế thừa từ include/constants.hh (thư mục gốc). Không thay đổi giá trị.
 */

#ifndef PLANNER_CONSTANTS_HPP
#define PLANNER_CONSTANTS_HPP

#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// =========================================================================
// Trọng số hàm đánh giá A* — f(n) = G_VALUE_WEIGHT * g + H_VALUE_WEIGHT * h
// =========================================================================
const double H_VALUE_WEIGHT = 1.0;
const double G_VALUE_WEIGHT = 1.0;

// =========================================================================
// Kích thước footprint robot (đơn vị: cm — lưới A* nội bộ)
// =========================================================================
const int ROBOT_FOOTPRINT_RADIUS             = 6; // cm (chưa mang pallet)
const int ROBOT_FOOTPRINT_RADIUS_WITH_PALLET = 7; // cm (đang mang pallet)
const int PALLET_FOOTPRINT_RADIUS            = 3; // cm (vùng cản tĩnh pallet)
const int ROBOT_OFFSET                       = 1; // cm (offset tâm robot về phía trước)

// =========================================================================
// Bước thời gian và chi phí cho các hành động di chuyển
// =========================================================================
const double STRAIGHT_TIME_STEP = 0.45; // seconds (≈ 1.11 cm/s khi tiến 1 ô)
const double TURN_TIME_STEP     = 5.5;  // seconds (≈ 1.57 cm/s khi rẽ vòng cung 5 ô)

const bool ALLOW_IN_PLACE_TURN = false; // Bật/tắt xoay tại chỗ (mặc định tắt)

const double TURN_COST         = 5.0;  // g-cost cho hành động rẽ
const double STRAIGHT_COST     = 0.1;  // g-cost cho hành động tiến thẳng
const double IN_PLACE_TURN_COST = 0.5; // g-cost cho xoay tại chỗ

// =========================================================================
// Ngưỡng và hệ số điều chỉnh hành vi lân cận pallet
// =========================================================================
const double NO_TURN_THRESH              = 4.0; // cm — dưới ngưỡng này không rẽ nữa
const double PALLET_WAIT_TIME            = 3.0; // seconds — dừng gắp pallet
const double PALLET_RUNWAY_SLOWDOWN_FACTOR = 3.0; // Hệ số giảm tốc khi vào vùng pallet

// =========================================================================
// Dung sai góc khi kiểm tra va chạm tĩnh với pallet
// =========================================================================
const double GOAL_THETA_TOLERANCE = M_PI / 8.0; // radians (= 22.5 độ)

// =========================================================================
// Cờ chọn thuật toán gán việc
// =========================================================================
const bool USE_HEURISTIC = true; // true: Euclidean heuristic, false: simple greedy

// =========================================================================
// Cờ debug và giới hạn tìm kiếm
// =========================================================================
const bool DEBUG = false;

const int    NUM_SAMPLED_STARTS = 10;
const double SEARCH_RADIUS_CM   = 10.0;
const double MAX_SEARCH_TIME    = 10.0; // seconds — timeout cho mỗi lần gọi a_star()

#endif // PLANNER_CONSTANTS_HPP
