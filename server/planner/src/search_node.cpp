/**
 * @file search_node.cpp
 * @brief Triển khai constructor và destructor của Node A* 4D.
 *        Giữ nguyên 100% so với node.cc gốc.
 */

#include "search_node.hpp"

Node::Node(int x, int y, double theta, double time,
           double g_value, double h_value, Node *parent)
{
    this->x       = x;
    this->y       = y;
    this->theta   = theta;
    this->time    = time;
    this->g_value = g_value;
    this->h_value = h_value;
    this->parent  = parent;
}

Node::~Node()
{
    // Con trỏ parent không bị xóa tại đây —
    // việc giải phóng toàn bộ chuỗi node do allocated_nodes trong a_star() đảm nhiệm.
}
