cmake_minimum_required(VERSION 3.8)
project(imu_sensor_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)

# Creating libraries
add_library(imu_hardware_lib include/imu_sensor_cpp/driver_platform/linuxI2C.cpp)
add_library(madgwick_filter_lib src/algorithms/madgwick_filter.cpp)
add_library(complementary_filter_lib src/algorithms/complementary_filter.cpp)

# Pointing where are heder files for specified library
target_include_directories(imu_hardware_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_include_directories(madgwick_filter_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_include_directories(complementary_filter_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(imu_node
  src/node/imu_node.cpp
  src/main.cpp)
# Linking imu_node with other libraries
target_link_libraries(
  imu_node 
  imu_hardware_lib
  madgwick_filter_lib
  complementary_filter_lib
  i2c
  )
# Adding ROS2 dependencies
ament_target_dependencies(imu_node
  rclcpp
  rclcpp_lifecycle
  sensor_msgs)

# Installing package
install( TARGETS
  imu_hardware_lib
  madgwick_filter_lib
  complementary_filter_lib
  imu_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)



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
