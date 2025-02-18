cmake_minimum_required(VERSION 3.8)
project(mtc_dual_arm_real)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_visual_tools REQUIRED)


add_executable(mtc_dual_arm_node src/mtc_dual_arm_node.cpp)
target_include_directories(mtc_dual_arm_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(mtc_dual_arm_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  mtc_dual_arm_node
  "moveit_task_constructor_core"
  "rclcpp"
  moveit_visual_tools
)

install(TARGETS mtc_dual_arm_node
  DESTINATION lib/${PROJECT_NAME})


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

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  )
  add_executable(load_scene src/load_scene.cpp)
  ament_target_dependencies(load_scene
    rclcpp
    geometry_msgs
    moveit_core
    moveit_ros_planning_interface
    # moveit_task_constructor_core
    moveit_visual_tools
  )
  install(TARGETS load_scene DESTINATION lib/${PROJECT_NAME})
ament_package()
