/**
 * @file planner_bindings.cpp
 * @brief Python binding qua pybind11 cho module planner_core.
 */

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "planner_coordinator.hpp"

namespace py = pybind11;

PYBIND11_MODULE(planner_core, m)
{
    m.doc() = "Multi-Robot Path Planning C++ Core Engine";

    m.def("generate_map", &generate_map,
          py::arg("map_length_mm"),
          py::arg("map_width_mm"),
          py::arg("pallet_poses"),
          "Tạo bản đồ lưới 2D từ kích thước sandbox và vị trí pallet.");

    py::class_<PlannerCoordinator>(m, "PlannerCoordinator")
        .def(py::init<int>(),
             py::arg("num_robots"),
             "Khởi tạo điều phối viên lập lịch.")

        .def("set_map", &PlannerCoordinator::set_map,
             py::arg("map"),
             "Đặt bản đồ lưới nhị phân.")

        .def("set_robot_poses", &PlannerCoordinator::set_robot_poses,
             py::arg("robot_poses"),
             "Đặt vị trí hiện tại của các robot theo mm.")

        .def("set_pallet_and_goal_poses", &PlannerCoordinator::set_pallet_and_goal_poses,
             py::arg("pallet_and_goal_poses"),
             "Đặt vị trí pallet và điểm đích theo mm.")

        .def("plan", &PlannerCoordinator::plan,
             "Lập kế hoạch tuần tự cho tất cả robot.")

        .def("get_paths", &PlannerCoordinator::get_paths,
             "Lấy quỹ đạo các robot theo mm.")

        .def("generate_paths", &PlannerCoordinator::generate_paths,
             py::arg("map_size"),
             py::arg("robot_poses"),
             py::arg("pallet_poses"),
             py::arg("goal_poses"),
             "Pipeline trọn gói: sinh map, gán việc, lập quỹ đạo và trả về kết quả.");
}
