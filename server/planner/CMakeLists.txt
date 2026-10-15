cmake_minimum_required(VERSION 3.12)
project(planner_core)

# ============================================================
# Tiêu chuẩn C++14 (tương thích pybind11 v2.11)
# ============================================================
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Thêm thư mục include
include_directories(${PROJECT_SOURCE_DIR}/include)

# ============================================================
# SỬA LỖI 3 — Build Đa Nền Tảng:
#   Bản gốc dùng:  set_target_properties(planner PROPERTIES SUFFIX ".so")
#   -> Cứng đuôi .so, Windows không import được (cần .pyd).
#
#   Bản mới: pybind11_add_module tự động xử lý:
#     - Windows:  planner_core.cp310-win_amd64.pyd   (hoặc tương ứng)
#     - Linux:    planner_core.cpython-310-x86_64.so
#     - macOS:    planner_core.cpython-310-darwin.so
#   Không cần ép SUFFIX thủ công.
# ============================================================
include(FetchContent)
FetchContent_Declare(
    pybind11
    GIT_REPOSITORY https://github.com/pybind/pybind11.git
    GIT_TAG        v2.11.1
    GIT_SHALLOW    TRUE
)
FetchContent_MakeAvailable(pybind11)

# Danh sách tất cả file C++ implementation
set(PLANNER_SOURCES
    src/search_node.cpp
    src/task_assigner.cpp
    src/motion_planner_4d.cpp
    src/planner_coordinator.cpp
    src/planner_bindings.cpp
)

# Tạo Python extension module — tên: planner_core
pybind11_add_module(planner_core MODULE ${PLANNER_SOURCES})

# Xuất file .pyd/.so thẳng vào thư mục server/planner/
# để Python có thể import trực tiếp: import planner_core
set_target_properties(planner_core PROPERTIES
    OUTPUT_NAME "planner_core"
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
)

# Multi-config generators (MSVC, Xcode): đặt output cho cả Debug và Release
foreach(CONFIG_TYPE Debug Release RelWithDebInfo MinSizeRel)
    string(TOUPPER ${CONFIG_TYPE} CONFIG_UPPER)
    set_target_properties(planner_core PROPERTIES
        LIBRARY_OUTPUT_DIRECTORY_${CONFIG_UPPER} "${CMAKE_CURRENT_SOURCE_DIR}"
        RUNTIME_OUTPUT_DIRECTORY_${CONFIG_UPPER} "${CMAKE_CURRENT_SOURCE_DIR}"
    )
endforeach()

# ============================================================
# Thông báo sau khi cấu hình
# ============================================================
message(STATUS "planner_core build configured:")
message(STATUS "  Output dir : ${CMAKE_CURRENT_SOURCE_DIR}")
message(STATUS "  C++ std    : C++${CMAKE_CXX_STANDARD}")
message(STATUS "  Python     : ${Python_EXECUTABLE}")
