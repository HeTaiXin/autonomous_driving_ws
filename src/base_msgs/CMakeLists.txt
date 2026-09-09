cmake_minimum_required(VERSION 3.8)
project(base_msgs)

# 设置 C 和 C++ 标准
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# CMAKE_COMPILER_IS_GUNCXX 在新版 cmake 中已经弃用，改用 CMAKE_CXX_COMPILER_ID
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU" OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 兼容旧版 cmake，因为 ros2 humble 自动生存的 cmake 是 3.8 版本
if(POLICY CMP0148)
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
# 1. 查找生成接口代码的依赖包（必须）
find_package(rosidl_default_generators REQUIRED)

# 2. 生成接口代码：指定功能包名、要编译的.msg/.srv文件路径
# 添加依赖
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GlobalPathService.srv"
  "srv/PNCMapService.srv"
  "msg/PNCMap.msg"
  "msg/ReferlinePoint.msg"
  "msg/Referline.msg"
  "msg/LocalPathPoint.msg"
  "msg/LocalPath.msg"
  "msg/LocalSpeedsPoint.msg"
  "msg/LocalSpeeds.msg"
  "msg/LocalTrajectoryPoint.msg"
  "msg/LocalTrajectory.msg"
  "msg/ObsInfo.msg"
  "msg/PlotInfo.msg"

  DEPENDENCIES 
  visualization_msgs
  std_msgs
  geometry_msgs
  nav_msgs
)

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

ament_package()
