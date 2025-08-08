cmake_minimum_required(VERSION 3.0.2)
project(mission_pkg)

add_compile_options(-std=c++17 -g -rdynamic)

find_package(catkin REQUIRED
    roscpp
    std_msgs
    mavros_msgs
    behaviortree_cpp
    geometry_msgs
    common_msgs
    message_filters
    tf2
    tf2_geometry_msgs
    quadrotor_msgs
    sensor_msgs
    nav_msgs
    visualization_msgs
    )
#find_package(livox_ros_driver2 REQUIRED)
#find_package(behaviortree_cpp REQUIRED)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ros1_node
#  CATKIN_DEPENDS other_catkin_pkg
  CATKIN_DEPENDS behaviortree_cpp
#  DEPENDS system_lib
)
include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)
#add_library(common SHARED src/plugins/common.cpp)
#list(APPEND plugin_libs common)

add_library(mavros_cnt SHARED src/mavros_cnt.cpp)
list(APPEND plugin_libs mavros_cnt )

set(LIBSOURCE src/plugins)

file(GLOB_RECURSE SOURCE_FILES "${LIBSOURCE}/*.cpp")

foreach(SOURCE_FILE ${SOURCE_FILES})
    get_filename_component(NODE_LIBS ${SOURCE_FILE} NAME_WE)
    # 创建一个共享库
    add_library(${NODE_LIBS} SHARED ${SOURCE_FILE})
    target_compile_definitions( ${NODE_LIBS} PRIVATE  BT_PLUGIN_EXPORT )
    set_target_properties(${NODE_LIBS}  PROPERTIES PREFIX "")
    target_link_libraries(${NODE_LIBS}  PRIVATE ${BTCPP_LIBRARY} mavros_cnt ${catkin_LIBRARIES})
endforeach()

add_executable(bt_ros src/bt_ros.cpp)
add_dependencies(bt_ros ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(bt_ros
   ${catkin_LIBRARIES}
    ${NODE_LIBS}
    mavros_cnt
    dl
 )

