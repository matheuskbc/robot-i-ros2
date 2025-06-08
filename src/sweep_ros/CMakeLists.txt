cmake_minimum_required(VERSION 3.5)
project(sweep_ros)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp)
find_package(pcl_conversions REQUIRED)
find_package(pcl_msgs)
find_package(sensor_msgs)


find_package(PCL REQUIRED)
find_package(Sweep REQUIRED)
find_package(Threads REQUIRED)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c99 -Wall -Wextra")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Wall -Wextra")

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

## Add link directories
link_directories(
  ${PCL_LIBRARY_DIRS}
)

add_executable(sweep_node src/node.cpp)
ament_target_dependencies(sweep_node rclcpp sensor_msgs pcl_conversions)
target_link_libraries(sweep_node ${LIBSWEEP_LIBRARY})

install(TARGETS
  sweep_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
