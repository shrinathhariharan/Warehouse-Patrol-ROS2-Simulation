cmake_minimum_required(VERSION 3.20)
project(warehouse_patrol)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(safety_filter src/safety_filter.cpp)
target_link_libraries(safety_filter PUBLIC
  rclcpp::rclcpp
  geometry_msgs::geometry_msgs
  sensor_msgs::sensor_msgs
)

install(TARGETS
  safety_filter
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/patrol_scheduler
  scripts/odom_tf_broadcaster
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch worlds models urdf rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_python_install_package(${PROJECT_NAME}
  SCRIPTS_DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
