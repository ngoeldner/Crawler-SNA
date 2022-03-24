cmake_minimum_required(VERSION 3.8)
project(jetson_cam_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)


#
find_package(sensor_msgs REQUIRED) 
find_package(jetson-utils REQUIRED)
find_package(jetson-inference REQUIRED)
find_package(CUDA REQUIRED)
find_package(OpenCV REQUIRED)


set(CMAKE_CXX_STANDARD 14)
include_directories(${CUDA_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
# add_definitions(-DROS2 -DROS_DISTRO=${ROS_DISTRO})

add_executable(talker src/segnet_intel02.cpp)
# target_link_libraries(talker ${catkin_LIBRARIES} jetson-inference)
target_link_libraries(talker ${catkin_LIBRARIES} jetson-inference ${OpenCV_LIBS})
ament_target_dependencies(talker rclcpp std_msgs sensor_msgs)

add_executable(listener src/sub.cpp)
target_link_libraries(listener ${catkin_LIBRARIES} jetson-inference)
ament_target_dependencies(listener rclcpp std_msgs sensor_msgs)

install(TARGETS
  talker 
  listener
  DESTINATION lib/${PROJECT_NAME})
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
