cmake_minimum_required(VERSION 2.8.3)
project(object_loader)

add_compile_options(-std=c++11 -DBOOST_MPL_CFG_NO_PREPROCESSED_HEADERS -DBOOST_MPL_LIMIT_VECTOR_SIZE=50 -DFUSION_MAX_VECTOR_SIZE=50)

set(CMAKE_BUILD_TYPE Debug)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  std_msgs
  std_srvs
  rosparam_utilities
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_msgs
  object_loader_msgs
  eigen_conversions
  tf_conversions
)

## System dependencies are found with CMake's conventions
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    roscpp
    geometry_msgs
    std_srvs
    rosparam_utilities
    moveit_core
    moveit_ros_planning_interface
    moveit_msgs
    eigen_conversions
    tf_conversions
    octomap
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# add_library(
#   manage_planning_scene src/manage_planning_scene.cpp
# )

add_executable       (context_manager_node src/context_manager_node.cpp)
add_dependencies     (context_manager_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(context_manager_node ${catkin_LIBRARIES} )


add_executable       (scene_spawner src/scene_spawner.cpp)
add_dependencies     (scene_spawner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(scene_spawner ${catkin_LIBRARIES} )
