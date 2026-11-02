cmake_minimum_required(VERSION 3.8)
project(lab02)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(urdf REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
	"srv/SolveIK.srv"
	DEPENDENCIES geometry_msgs sensor_msgs)

# Part 3: Jacobians
add_executable(test_jacobians src/test_jacobians.cpp)
target_include_directories(test_jacobians PRIVATE ${EIGEN3_INCLUDE_DIR})
ament_target_dependencies(test_jacobians rclcpp kdl_parser orocos_kdl)

# Part 4: IK solver
add_executable(test_ik src/test_ik.cpp)
target_include_directories(test_ik PRIVATE ${EIGEN3_INCLUDE_DIR})
ament_target_dependencies(test_ik rclcpp kdl_parser orocos_kdl sensor_msgs geometry_msgs urdf)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(test_ik "${cpp_typesupport_target}")


install(TARGETS test_jacobians test_ik
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
