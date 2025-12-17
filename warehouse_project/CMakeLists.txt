cmake_minimum_required(VERSION 3.8)
project(warehouse_project)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# --- SEZIONE FIND_PACKAGE AGGIORNATA ---
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ros_gz_interfaces REQUIRED)
# Assicurati che ros2_kdl_package sia nel workspace o installato
find_package(ros2_kdl_package REQUIRED) 
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_kdl REQUIRED)
find_package(nav2_msgs REQUIRED)
# ---------------------------------------

add_executable(iiwa_task_manager src/iiwa_task_manager.cpp)
ament_target_dependencies(iiwa_task_manager
  rclcpp
  rclcpp_action
  geometry_msgs
  std_msgs
  ros2_kdl_package # Per l'action MoveArm
  ros_gz_interfaces # Per il servizio SetEntityPose
  tf2_ros             
  tf2_geometry_msgs
)

add_executable(fra2mo_task_manager src/fra2mo_task_manager.cpp)
ament_target_dependencies(fra2mo_task_manager
  rclcpp
  rclcpp_action
  nav2_msgs
  geometry_msgs
  std_msgs
  tf2_ros
  tf2_geometry_msgs
  ros_gz_interfaces
)

install(TARGETS
  iiwa_task_manager
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  fra2mo_task_manager
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch worlds models maps
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Salta il controllo copyright
  set(ament_cmake_copyright_FOUND TRUE)
  # Salta cpplint
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
