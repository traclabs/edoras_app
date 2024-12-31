project(GATEWAY_APP C)

include_directories(fsw/mission_inc)
include_directories(fsw/platform_inc)
#include_directories(${gateway_app_MISSION_DIR}/fsw/platform_inc) # ros_app?

set(edoras_core_DIR $ENV{HOME}/local/lib/cmake/edoras_core)
find_package(edoras_core REQUIRED)

# TODO: Get rid of this, just use another edoras bridge to talk to robot
set(EDORAS_WS /home/ana/ros2/edoras_ws)
set(SCRATCH_DIR ${EDORAS_WS}/edoras/src/scratch)
#include_directories(${SCRATCH_DIR}/c_serialize_udp)
link_directories(${SCRATCH_DIR}/c_serialize_udp/build)

 
# Create library for talking to robot using sockets
add_library(robot_comm_udp_test fsw/src/robot_comm_udp_test.c)

# Create the app module
add_cfe_app(gateway_app fsw/src/gateway_app.c)
target_link_libraries(gateway_app 
                      m 
                      serialize_library 
                      robot_comm_udp_test
                      ${edoras_core_LIBRARIES})
target_include_directories(gateway_app PUBLIC
  fsw/mission_inc
  fsw/platform_inc
  ${SCRATCH_DIR}/c_serialize_udp
  ${edoras_core_INCLUDE_DIRS}
# ${DEP_INCLUDE_DIRS}

)

get_target_property(target_options gateway_app COMPILE_OPTIONS)
list(REMOVE_ITEM target_options "-pedantic")
set_property(TARGET gateway_app PROPERTY COMPILE_OPTIONS ${target_options})

