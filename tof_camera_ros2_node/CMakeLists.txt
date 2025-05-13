cmake_minimum_required(VERSION 3.8)
project(tof_camera_ros2_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(ArducamDepthCamera REQUIRED)
find_package(rclcpp REQUIRED)

set(CLIBS ArducamDepthCamera::ArducamDepthCamera4C)
set(CXXLIBS ArducamDepthCamera::ArducamDepthCamera)

add_executable(tof_camera_node src/tof_camera_node.cpp)
target_include_directories(tof_camera_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(tof_camera_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

target_link_libraries(tof_camera_node ${CXXLIBS} ${OpenCV_LIBS})

install(TARGETS tof_camera_node
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

ament_package()
