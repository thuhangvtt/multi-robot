/**
 * @file planner_bindings.cpp
 * @brief Pybind11 module binding cho planner_core C++ extension.
 *
 * SỬA LỖI 4 — Global State:
 *   Bản gốc (main.cc): Planner *planner; — biến toàn cục, không quản lý vòng đời.
 *   Bản mới: Xuất trực tiếp class PlannerCoordinator như một Python Type.
 *   Vòng đời gắn với Python object -> garbage collected tự động khi Python không dùng nữa.
 *
 * Module name: planner_core
 *   - Khác với thư mục package "planner" -> tránh hoàn toàn Module Shadowing
 *   - Python import: from planner import planner_service (package)
 *                    planner_core.PlannerCoordinator(...)  (C++ extension bên trong)
 */

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "planner_coordinator.hpp"

namespace py = pybind11;

PYBIND11_MODULE(planner_core, m)
{
    m.doc() = "PARROT Multi-Robot Path Planning C++ Core Engine\n"
              "Provides PlannerCoordinator class and generate_map utility.\n"
              "Module: planner_core (avoid shadowing the planner/ Python package)";

    // -------------------------------------------------------------------------
    // Hàm tiện ích: sinh bản đồ lưới 2D từ kích thước sandbox và vị trí pallet
    // -------------------------------------------------------------------------
    m.def("generate_map", &generate_map,
          py::arg("map_length_mm"),
          py::arg("map_width_mm"),
          py::arg("pallet_poses"),
          "Tạo bản đồ lưới 2D (1cm/cell). 0=tự do, 1=vật cản (3x3 quanh pallet).");

    // -------------------------------------------------------------------------
    // Class PlannerCoordinator: Điều phối lập lịch đường đi đa robot
    // -------------------------------------------------------------------------
    py::class_<PlannerCoordinator>(m, "PlannerCoordinator")
        .def(py::init<int>(),
             py::arg("num_robots"),
             "Khởi tạo hệ thống lập kế hoạch cho num_robots robot.")

        .def("set_map", &PlannerCoordinator::set_map,
             py::arg("map"),
             "Đặt bản đồ lưới. Mỗi ô 1cm x 1cm. 0=tự do, 1=vật cản.")

        .def("set_robot_poses", &PlannerCoordinator::set_robot_poses,
             py::arg("robot_poses"),
             "Đặt vị trí robot [[x, y, theta], ...] (đơn vị: mm). Tự động chuyển sang cm.")

        .def("set_pallet_and_goal_poses", &PlannerCoordinator::set_pallet_and_goal_poses,
             py::arg("pallet_and_goal_poses"),
             "Đặt vị trí pallet+goal [[px,py,ptheta,gx,gy,gtheta],...] (mm). Tự động cm.")

        .def("plan", &PlannerCoordinator::plan,
             "Lập kế hoạch tuần tự cho tất cả robot. Trả về 0 nếu thành công.")

        .def("get_paths", &PlannerCoordinator::get_paths,
             "Lấy quỹ đạo: [robot][waypoint][x(mm), y(mm), theta, time, tag].\n"
             "tag: 0=đi, 1=tại pallet, 2=tại goal. theta trong [-pi, pi].")

        .def("generate_paths", &PlannerCoordinator::generate_paths,
             py::arg("map_size"),
             py::arg("robot_poses"),
             py::arg("pallet_poses"),
             py::arg("goal_poses"),
             "Hàm gộp toàn bộ pipeline: sinh map, gán việc, lập kế hoạch, trả quỹ đạo.\n"
             "map_size: [width_mm, height_mm]\n"
             "robot_poses: [[x,y,theta],...] (mm)\n"
             "pallet_poses: [[x,y,theta],...] (mm)\n"
             "goal_poses: [[x,y,theta],...] (mm)\n"
             "Trả về: [robot][waypoint][x(mm), y(mm), theta, time, tag]");
}
