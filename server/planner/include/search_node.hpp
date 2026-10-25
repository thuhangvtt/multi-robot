/**
 * @file search_node.hpp
 * @brief Định nghĩa cấu trúc Node A* 4D và các bộ so sánh / băm.
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

struct OpenListNodeComparator
{
    /** So sánh mức ưu tiên f-value giữa 2 nút trong priority queue. */
    bool operator()(const Node *n1, const Node *n2) const
    {
        return (G_VALUE_WEIGHT * n1->g_value + H_VALUE_WEIGHT * n1->h_value)
             > (G_VALUE_WEIGHT * n2->g_value + H_VALUE_WEIGHT * n2->h_value);
    }
};

struct ClosedListNodeComparator
{
    /** So sánh bằng giữa 2 nút trong tập hợp closed list. */
    bool operator()(const Node *n1, const Node *n2) const
    {
        return n1->x == n2->x
            && n1->y == n2->y
            && n1->theta == n2->theta
            && n1->time  == n2->time;
    }
};

struct NodeHasher
{
    /** Hàm băm tọa độ 4D không va chạm. */
    size_t operator()(const Node *n) const
    {
        size_t seed = 0;
        auto hash_combine = [&seed](size_t val) {
            seed ^= val + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        };

        hash_combine(std::hash<int>{}(n->x));
        hash_combine(std::hash<int>{}(n->y));
        hash_combine(std::hash<int>{}(static_cast<int>(std::round(n->theta * 1000.0))));
        hash_combine(std::hash<int>{}(static_cast<int>(std::round(n->time * 100.0))));

        return seed;
    }
};

#endif // SEARCH_NODE_HPP
