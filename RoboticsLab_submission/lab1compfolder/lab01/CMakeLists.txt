cmake_minimum_required(VERSION 3.8)
project(lab01)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(Eigen3 REQUIRED)

# Part 2: shortest transform
add_executable(shortest_transform src/shortest_transform.cpp)
target_include_directories(shortest_transform PRIVATE ${EIGEN3_INCLUDE_DIR})
ament_target_dependencies(shortest_transform rclcpp tf2 tf2_ros geometry_msgs)

# Part 3: forward kinematics with KDL
add_executable(test_fk src/test_fk.cpp)
ament_target_dependencies(test_fk rclcpp tf2_ros geometry_msgs sensor_msgs kdl_parser)

install(TARGETS shortest_transform test_fk
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
