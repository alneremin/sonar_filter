cmake_minimum_required(VERSION 3.0.2)
project(sonar_filter)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

find_package(OpenCV REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES sonar_filter
#  CATKIN_DEPENDS roscppp std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(sonar_filter src/main.cpp src/filters.cpp src/LowPassFilter/LowPassFilter.cpp src/LowPassFilter/LowPassFilter.hpp src/filters.h )

target_link_libraries(sonar_filter
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
