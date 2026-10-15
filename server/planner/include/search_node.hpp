/**
 * @file search_node.hpp
 * @brief Node A* 4D (x, y, theta, time) cho thuật toán tìm kiếm đường đi.
 *
 * Sửa lỗi so với node.hh gốc:
 *   - NodeHasher: Thay thế string concatenation (dễ gây hash collision)
 *     bằng bitwise hash combine O(1) — loại bỏ hoàn toàn khả năng đụng độ hash
 *     giữa các cặp tọa độ như (x=1, y=23) và (x=12, y=3).
 *
 * Giữ nguyên:
 *   - Cấu trúc Node (x, y, theta, time, g_value, h_value, parent)
 *   - OpenListNodeComparator (ưu tiên hàng đợi theo f = G*g + H*h)
 *   - ClosedListNodeComparator (kiểm tra trùng theo x, y, theta, time)
 */

#ifndef SEARCH_NODE_HPP
#define SEARCH_NODE_HPP

#include <stdlib.h>
#include <string>
#include <algorithm>
#include <cmath>
#include "planner_constants.hpp"

class Node
{
public:
    int    x;
    int    y;
    double theta;
    double time;
    double g_value;
    double h_value;
    Node  *parent;

    Node(int x, int y, double theta, double time, double g_value, double h_value, Node *parent);
    ~Node();
};

// =============================================================================
// OpenListNodeComparator: Dùng cho priority_queue (min-heap theo f-value)
// Giữ nguyên so với bản gốc.
// =============================================================================
struct OpenListNodeComparator
{
    bool operator()(const Node *n1, const Node *n2) const
    {
        return (G_VALUE_WEIGHT * n1->g_value + H_VALUE_WEIGHT * n1->h_value)
             > (G_VALUE_WEIGHT * n2->g_value + H_VALUE_WEIGHT * n2->h_value);
    }
};

// =============================================================================
// ClosedListNodeComparator: Dùng cho unordered_set (kiểm tra bằng nhau).
// Giữ nguyên so với bản gốc.
// =============================================================================
struct ClosedListNodeComparator
{
    bool operator()(const Node *n1, const Node *n2) const
    {
        return n1->x == n2->x
            && n1->y == n2->y
            && n1->theta == n2->theta
            && n1->time  == n2->time;
    }
};

// =============================================================================
// NodeHasher: SỬA LỖI HASH COLLISION
//
// Bản gốc dùng:
//   std::to_string(x) + std::to_string(y) + std::to_string(theta) + std::to_string(time)
// → Lỗi: (x=1, y=23) và (x=12, y=3) cho ra cùng chuỗi "123..."
//
// Bản mới: Bitwise hash combine trên từng trường integer (chuẩn boost::hash_combine).
// Không cấp phát chuỗi — O(1) thực sự, không có collision cơ học.
// =============================================================================
struct NodeHasher
{
    size_t operator()(const Node *n) const
    {
        size_t seed = 0;

        // Lambda nội tuyến: XOR kết hợp với phép dịch và hằng số nguyên tố
        auto hash_combine = [&seed](size_t val) {
            seed ^= val + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        };

        hash_combine(std::hash<int>{}(n->x));
        hash_combine(std::hash<int>{}(n->y));
        // Nhân 1000 trước khi round để giữ 3 chữ số thập phân của theta
        hash_combine(std::hash<int>{}(static_cast<int>(std::round(n->theta * 1000.0))));
        // Nhân 100 trước khi round để giữ 2 chữ số thập phân của time
        hash_combine(std::hash<int>{}(static_cast<int>(std::round(n->time * 100.0))));

        return seed;
    }
};

#endif // SEARCH_NODE_HPP
