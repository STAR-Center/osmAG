cmake_minimum_required(VERSION 3.1)
project(osm_ag_parse)

add_compile_options(-Wall -Wextra)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  eigen_conversions
  grid_map_core
  grid_map_msgs
  grid_map_cv
  grid_map_sdf
  sensor_msgs
  nav_msgs
  std_msgs
  geometry_msgs
  cv_bridge
  rosbag
  tf
  visualization_msgs
)

add_library(
  ${PROJECT_NAME}
  src/node.cpp
  src/area.cpp
  src/astarcal.cpp
  src/path_graph.cpp
  ../tinyxml2/tinyxml2.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC include ${catkin_INCLUDE_DIRS}  ${EIGEN3_INCLUDE_DIR})

add_executable(data_process src/data_load_save.cpp)
target_link_libraries(data_process ${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBS})


