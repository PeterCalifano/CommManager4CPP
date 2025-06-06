set(lib_name ${project_name}) # Default name for the library is project_name

# Add config.h file to build
configure_file(config.h.in config.h)

# Add src files to library
set(SRC_PATTERN "*.cpp; *.c")
file(GLOB srcLibFiles RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ${SRC_PATTERN})

message("Library src files found: ${srcLibFiles_local}")

# Define INCLUDE_DIRECTORIES (subdirectories append to this)
set(${${lib_name}_INCLUDE_DIRS} ${${lib_name}_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR})

# Add subdirectories with src files
message(STATUS "Adding module: tcp-server")
add_subdirectory( tcp-server )

# Exclude EXCLUDED_LIST from the list of src files
set(EXCLUDED_LIST "")
list(REMOVE_ITEM srcLibFiles ${EXCLUDED_LIST})

# Add custom library with src files
list(APPEND srcLibFiles "${CMAKE_CURRENT_SOURCE_DIR}/config.h")

add_library(${lib_name} SHARED
            ${srcLibFiles})

# If need to add other files after add_library, use:
#target_sources(lib_name SHARED ${srcLibFiles})

# Link targets
target_include_directories(${lib_name} PUBLIC 
                          ${${lib_name}_INCLUDE_DIRS} 
                          ${OpenCV_INCLUDE_DIRS} 
                          ${PROJECT_SOURCE_DIR} 
                          ${JSONCPP_INCLUDE_DIRS} 
                          )

#target_include_directories(lib_name INTERFACE ${interface_INCLUDE_DIRS} ) 
# TIP: INTERFACE ensures that the consumer target does not need to include manually the include directories the library target depends on, because they are automagically propagated to the target, even if the target does not need thme. The same happens with PUBLIC, but this time the includes are used by the target library itself.

target_link_libraries(${lib_name} PUBLIC 
                      ${OpenCV_LIBS} 
                      Eigen3::Eigen 
                      jsoncpp
                      ${OPENGL_LIBRARIES}
                      )

# Define INCLUDE_DIRECTORIES
set(${${lib_name}_INCLUDE_DIRS} ${${lib_name}_INCLUDE_DIRS} PARENT_SCOPE)

# Add target to installable list 
set(installable_targets ${installable_targets} ${lib_name} PARENT_SCOPE)

# INSTALL CONFIGURATION
install(
	TARGETS ${installable_targets}
	LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
	ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
	RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  ) # Install targets

# Install headers from this directory
file(GLOB installable_headers "*.h") # Discover and install all header files
install(FILES ${installable_headers} 
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${project_name}
      ) # Install headers

install(FILES "${CMAKE_CURRENT_SOURCE_DIR}/config.h"  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${project_name}) # Install config.h 
