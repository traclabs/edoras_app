project(EDORAS_APP C)

include_directories(fsw/mission_inc)
include_directories(fsw/platform_inc)
#include_directories(${edoras_app_MISSION_DIR}/fsw/platform_inc) # ros_app?

set(edoras_core_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../../edoras_core/install/lib/cmake/edoras_core)
find_package(edoras_core REQUIRED)
 
# Create library for talking to robot using sockets
add_library(robot_comm_udp_gateway_big_arm fsw/src/robot_comm_udp_gateway_big_arm.c)

# Create the app module
add_cfe_app(edoras_app fsw/src/edoras_app.c)
target_link_libraries(edoras_app 
                      m 
                      robot_comm_udp_gateway_big_arm
                      ${edoras_core_LIBRARIES})

target_include_directories(edoras_app PUBLIC
  fsw/mission_inc
  fsw/platform_inc
  ${edoras_core_INCLUDE_DIRS}
)

get_target_property(target_options edoras_app COMPILE_OPTIONS)
list(REMOVE_ITEM target_options "-pedantic")
set_property(TARGET edoras_app PROPERTY COMPILE_OPTIONS ${target_options})

