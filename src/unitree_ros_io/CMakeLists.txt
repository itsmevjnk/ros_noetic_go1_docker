cmake_minimum_required(VERSION 3.0.2)
project(unitree_ros_io)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS roscpp tf std_msgs sensor_msgs nav_msgs unitree_legged_msgs)
catkin_package(CATKIN_DEPENDS tf roscpp message_runtime)

message("-- CMAKE_SYSTEM_PROCESSOR: ${CMAKE_SYSTEM_PROCESSOR}")
if("${CMAKE_SYSTEM_PROCESSOR}" MATCHES "x86_64.*")
  set(ARCH amd64)
else()
  set(ARCH arm64)
endif()

link_directories(${CMAKE_SOURCE_DIR}/unitree_legged_sdk/lib/cpp/${ARCH})
set(EXTRA_LIBS -pthread libunitree_legged_sdk.so)
set(CMAKE_CXX_FLAGS "-O3 -fPIC")

message("-- CMAKE_SOURCE_DIR: ${CMAKE_SOURCE_DIR}")
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_SOURCE_DIR}/unitree_legged_sdk/include
  ${CMAKE_SOURCE_DIR}/unitree_ros_to_real/unitree_legged_real/include
)

add_executable(driver_node src/driver_node.cpp)
target_link_libraries(driver_node ${EXTRA_LIBS} ${catkin_LIBRARIES})
add_dependencies(driver_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
