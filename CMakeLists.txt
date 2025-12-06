project(EDORAS_APP C)

include_directories(fsw/mission_inc)
include_directories(fsw/platform_inc)
#include_directories(${edoras_app_MISSION_DIR}/fsw/platform_inc) # ros_app?
 
# Create library for talking to robot using sockets
add_library(robot_comm_udp_imetro fsw/src/robot_comm_udp_imetro.c)

# Create the app module
add_cfe_app(edoras_app fsw/src/edoras_app.c)
target_link_libraries(edoras_app
                      m 
                      robot_comm_udp_imetro)

target_include_directories(edoras_app PUBLIC
  fsw/mission_inc
  fsw/platform_inc
)

get_target_property(target_options edoras_app COMPILE_OPTIONS)
list(REMOVE_ITEM target_options "-pedantic")
set_property(TARGET edoras_app PROPERTY COMPILE_OPTIONS ${target_options})

