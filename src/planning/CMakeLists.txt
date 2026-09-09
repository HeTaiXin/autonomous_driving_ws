#[[
  CMakeLists.txt 是C++功能包的“构建蓝图”与“施工指南”。它是一组基于CMake的脚本，指导编译器
如何编译源码、链接库文件，并最终将生成的可执行文件或库安装到指定位置。他负责将源代码转化为可运
行的文件。
  package.xml 申明需要哪些“原材料”，CMakeLists.txt 则使用这些“原材料”来进行构建。
]]

cmake_minimum_required(VERSION 3.8) # 指定 CMake 的最低版本为 3.8。
project(planning) # 定义项目名称，必须与 package.xml 中的 <name> 一致。后续可通过${PROJECT_NAME}引用

# 设置 C 语言标准为 C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
# 设置 C++ 语言标准为 C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# CMAKE_COMPILER_IS_GUNCXX 在新版 cmake 中已经弃用，改用 CMAKE_CXX_COMPILER_ID
# 为 GCC 和 Clang 编译器开启常见警告（-Wall -Wextra）和严格 ISO C++ 警告（-Wpedantic），
# 有助于发现潜在代码问题
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU" OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 设置与编译指令，生成 compile_commands.json
# 然后让 c_cpp_properties.json 读取这个文件，防止预编译找不到 cmake 中定义的宏而报错
# c_cpp_properties.json 中加上 "compileCommands": "${workspaceFolder}/build/compile_commands.json"来读取这个文件
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# 设置包名，让子目录节点安装时使用
set(PLANNING_PACKAGE_NAME ${PROJECT_NAME})

# 包含头文件目录
include_directories(
  /usr/local/include/eigen-5.0.0
  src/common/config_reader
  src/common/math
  src/decision_center
  src/pnc_map_creator
  src/pnc_map_creator/pnc_map_straight
  src/pnc_map_creator/pnc_map_sturn
  src/global_planner
  src/global_planner/global_planner_normal
  src/local_planner
  src/local_planner/local_path
  src/local_planner/local_speeds
  src/reference_line
  src/vehicle_info
  src/vehicle_info/main_car
  src/vehicle_info/obs_car
  src/planning_process
  src/move_cmd
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(base_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OsqpEigen REQUIRED)

# 添加子目录
add_subdirectory(src/common)
add_subdirectory(src/decision_center)
add_subdirectory(src/pnc_map_creator)
add_subdirectory(src/global_planner)
add_subdirectory(src/local_planner)
add_subdirectory(src/reference_line)
add_subdirectory(src/vehicle_info)
add_subdirectory(src/planning_process)
add_subdirectory(src/move_cmd)

# 安装辅助目录：将 config/、launch/、urdf/、rviz/目录安装到 install/share/planning/下，
# 使得 ros2 launch 和 ros2 run 能够访问这些资源。
install(DIRECTORY
  config launch urdf rviz
  DESTINATION share/${PROJECT_NAME})

# 当 BUILD_TESTING 开启时（默认 ON），自动加载 ament_lint_auto 进行代码风格检查
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# 必须位于文件末尾，用于生成功能包所需的配置文件和索引信息，使该包能够被其他包发现和依赖。
ament_package()
