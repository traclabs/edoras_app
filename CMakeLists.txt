project(GATEWAY_APP C)

include_directories(fsw/mission_inc)
include_directories(fsw/platform_inc)
#include_directories(${gateway_app_MISSION_DIR}/fsw/platform_inc) # ros_app?

set(SCRATCH_DIR /home/ana/ros2/edoras_ws/edoras/src/scratch)
include_directories(${SCRATCH_DIR}/c_serialize_udp)
link_directories(${SCRATCH_DIR}/c_serialize_udp/build)


set( include_deps
 rosidl_typesupport_introspection_c
 rosidl_runtime_c
 rcutils
 rosidl_typesupport_interface
)

set( library_deps
 rosidl_typesupport_introspection_c
 rosidl_runtime_c
 rcutils
)

set(ROS_ROOT /opt/ros/jazzy)
foreach(dep ${include_deps})
  list(APPEND DEP_INCLUDE_DIRS "${ROS_ROOT}/include/${dep}")
endforeach()

set(EDORAS_DIR /home/ana/ros2/edoras_ws/edoras_parsing)
include_directories(${EDORAS_DIR}/include
 ${DEP_INCLUDE_DIRS})
find_library(EDORAS_LIBRARY edoras  HINTS ${EDORAS_DIR}/build ) 




# Create library for talking to robot using sockets
add_library(robot_comm_udp_test fsw/src/robot_comm_udp_test.c)

# Create the app module
add_cfe_app(gateway_app fsw/src/gateway_app.c)
target_link_libraries(gateway_app 
                      m 
                      serialize_library 
                      robot_comm_udp_test
                      ${EDORAS_LIBRARY})

target_include_directories(gateway_app PUBLIC
  fsw/mission_inc
  fsw/platform_inc
)

get_target_property(target_options gateway_app COMPILE_OPTIONS)
list(REMOVE_ITEM target_options "-pedantic")
set_property(TARGET gateway_app PROPERTY COMPILE_OPTIONS ${target_options})

