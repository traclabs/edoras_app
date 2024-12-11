
set( library_deps
 rosidl_typesupport_introspection_c
 rosidl_runtime_c
 rosidl_typesupport_macros
 rosidl_typesupport_interface
 rcutils
)

foreach(lib_i ${library_deps})
  include_directories(/opt/ros/jazzy/include/${lib_i})
  find_library(${lib_i}_LIB ${lib_i} HINTS /opt/ros/jazzy/lib/${lib_i})
endforeach() 
  link_directories(/opt/ros/jazzy/lib/rosidl_typesupport_introspection_c)
set(rosidl_libraries rosidl_typesupport_introspection_c)

add_library(edoras_conversion debug_utils_c_version.c)

foreach(lib_i ${library_deps})
  target_link_libraries(edoras_conversion ${lib_i}_LIB)
endforeach() 


