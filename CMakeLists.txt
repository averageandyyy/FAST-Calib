cmake_minimum_required(VERSION 3.8)
project(fast_calib)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosidl_typesupport_cpp REQUIRED)
find_package(PCL 1.10 REQUIRED)
find_package(OpenCV REQUIRED)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(DEPENDENCIES
  rclcpp
  sensor_msgs
  pcl_conversions
  rosbag2_cpp
  rosidl_typesupport_cpp
)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

set(CMAKE_BUILD_TYPE Release)

if (CMAKE_BUILD_TYPE STREQUAL "Release")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
endif()

add_executable(fast_calib src/main.cpp)

ament_target_dependencies(fast_calib ${DEPENDENCIES})
target_link_libraries(fast_calib ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(multi_fast_calib src/multi_scene.cpp)
ament_target_dependencies(multi_fast_calib ${DEPENDENCIES})
target_link_libraries(multi_fast_calib ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

install(TARGETS
  fast_calib
  multi_fast_calib
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  rviz_cfg
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/distance_filter_tool.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()