
set( include_deps
 rosidl_typesupport_introspection_c
 rosidl_runtime_c
 rosidl_typesupport_interface
 rcutils
)

set( library_deps
 rosidl_typesupport_introspection_c
 rosidl_runtime_c
 rcutils
)

# **********************************
# Add include / link dependencies
# **********************************
foreach(include_i ${include_deps})
  include_directories(/opt/ros/jazzy/include/${include_i})
endforeach() 

foreach(lib_i ${library_deps})
  find_library( ${lib_i}_LIBRARY ${lib_i} HINT /opt/ros/jazzy/lib)
endforeach() 

# ****************************************************
# Add library and link accordingly
add_library(edoras_conversion debug_utils_c_version.c)

foreach(lib_i ${library_deps})
  target_link_libraries(edoras_conversion PUBLIC ${${lib_i}_LIBRARY} )
endforeach() 

foreach(include_i ${include_deps})
  target_include_directories(edoras_conversion PUBLIC /opt/ros/jazzy/include/${include_i})
endforeach() 


# dlsym is throwing an error if using -pedantic
get_target_property(target_options edoras_conversion COMPILE_OPTIONS)
list(REMOVE_ITEM target_options "-pedantic")
set_property(TARGET edoras_conversion PROPERTY COMPILE_OPTIONS ) #${target_options})

