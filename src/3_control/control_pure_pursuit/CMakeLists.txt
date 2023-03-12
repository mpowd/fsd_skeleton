cmake_minimum_required(VERSION 2.8.3)
project(control_pure_pursuit)

add_compile_options(-std=c++11)

# For debugging
set(CMAKE_BUILD_TYPE RelWithDebInfo)

set(PROJECT_DEPS
  roscpp
  std_msgs
  fsd_common_msgs
  fs_msgs
  )

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  fsd_common_msgs
  geometry_msgs
  fs_msgs
  tf
  )

catkin_package(
  INCLUDE_DIRS
  LIBRARIES
  CATKIN_DEPENDS
  DEPENDS
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${roscpp_INCLUDE_DIRS}
)

# Each node in the package must be declared like this
add_executable(${PROJECT_NAME}
  src/pure_pursuit_handle.cpp
  src/pure_pursuit.cpp
  src/main.cpp
)
add_executable(track_handle
  src/track_handle.cpp
) 
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
target_link_libraries(track_handle
  ${catkin_LIBRARIES}
)
