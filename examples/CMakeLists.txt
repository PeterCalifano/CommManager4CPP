# Add examples to build
set(EXAMPLES_PATTERN "example_*.cpp")
file(GLOB srcExampleFiles RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ${EXAMPLES_PATTERN})
message("Examples files found: ${srcExampleFiles}")

# Exclude EXCLUDED_LIST from the list of examples
set(EXCLUDED_LIST "")
list(REMOVE_ITEM srcExampleFiles ${EXCLUDED_LIST})

# Add to build and link
foreach(exampleFile ${srcExampleFiles})

    get_filename_component(exampleName ${exampleFile} NAME_WE)
    add_executable(${exampleName} ${exampleFile})

    target_link_libraries(${exampleName} PUBLIC lib_name)
    target_include_directories(${exampleName} PRIVATE ${lib_name_INCLUDE_DIRS})

endforeach()