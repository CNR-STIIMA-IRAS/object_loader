cmake_minimum_required(VERSION 2.8.3)
project(object_loader_msgs)

add_compile_options(-std=c++11 -DBOOST_MPL_CFG_NO_PREPROCESSED_HEADERS -DBOOST_MPL_LIMIT_VECTOR_SIZE=50 -DFUSION_MAX_VECTOR_SIZE=50)

set(CMAKE_BUILD_TYPE Debug)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  std_msgs
  std_srvs
  roscpp
  message_generation
  rosparam_utilities
)


add_message_files( DIRECTORY msg
                   FILES Object.msg)

add_service_files( DIRECTORY srv
                   FILES
                   AddObjects.srv
                   RemoveObjects.srv
                   AttachObject.srv
                   DetachObject.srv
                   ChangeColor.srv
                   ListObjects.srv
                   AddObjectsGroup.srv
                   RemoveObjectsGroup.srv
                 )

generate_messages( DEPENDENCIES
                    geometry_msgs
                    std_msgs
                    std_srvs
)




## System dependencies are found with CMake's conventions
catkin_package( CATKIN_DEPENDS
    geometry_msgs
    std_srvs
    message_generation
)
