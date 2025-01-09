# Discover and install all header files
file(GLOB installable_headers "*.h")

# Get the relative path from the root
file(RELATIVE_PATH relative_path_from_root ${CMAKE_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR})

# Remove the 'src/' prefix from the relative path if it exists
string(REPLACE "src/" "" relative_path_without_src ${relative_path_from_root})

# Install the header files to the relative path
install(FILES ${installable_headers} DESTINATION include/${project_name}/${relative_path_from_root})

# Add src files to library
set(SRC_PATTERN "*.cpp; *.c")
file(GLOB srcLibFiles_local ${CMAKE_CURRENT_SOURCE_DIR} ${SRC_PATTERN})
message("Library src files found: ${srcLibFiles_local}")

# Set global # Export list of src files to parent scope
set(srcLibFiles ${srcLibFiles} ${srcLibFiles_local} PARENT_SCOPE)
set(${project_name}_INCLUDE_DIRS ${GTSAM_SPACENAV_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR} PARENT_SCOPE)