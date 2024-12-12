cmake_minimum_required(VERSION 3.10)
project(openc3-cosmos-nos3)

if(NOT DEFINED BASE_DIR)
    message(FATAL_ERROR "BASE_DIR not set")
endif()
if(NOT DEFINED GSW_DIR)
    message(FATAL_ERROR "GSW_DIR not set")
endif()

# Set plugin directory
set(PLUGIN_DIR ${CMAKE_CURRENT_BINARY_DIR}/openc3-cosmos-nos3)

add_custom_command(
    OUTPUT ${PLUGIN_DIR}/structure.stamp
    COMMAND ${CMAKE_COMMAND} -E make_directory ${PLUGIN_DIR}
    COMMAND ${CMAKE_COMMAND} -E make_directory ${PLUGIN_DIR}/targets
    COMMAND ${CMAKE_COMMAND} -E touch ${PLUGIN_DIR}/structure.stamp
)

set(PLUGIN_FILES
    Rakefile
    openc3-cosmos-nos3.gemspec
    LICENSE.txt
    README.md
)

foreach(file ${PLUGIN_FILES})
    add_custom_command(
        OUTPUT ${PLUGIN_DIR}/${file}
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
                ${CMAKE_CURRENT_SOURCE_DIR}/${file}
                ${PLUGIN_DIR}/${file}
        DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/${file}
    )
endforeach()

# Copy and process targets
add_custom_command(
    OUTPUT ${PLUGIN_DIR}/targets.stamp
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/setup_plugin.py
            --base-dir ${BASE_DIR}
            --gsw-dir ${GSW_DIR}
            --plugin-dir ${PLUGIN_DIR}
    COMMAND ${CMAKE_COMMAND} -E touch ${PLUGIN_DIR}/targets.stamp
    DEPENDS 
        ${PLUGIN_DIR}/structure.stamp
        ${CMAKE_CURRENT_SOURCE_DIR}/setup_plugin.py
)

# Main target
add_custom_target(openc3_plugin ALL
    DEPENDS 
        ${PLUGIN_DIR}/structure.stamp
        ${PLUGIN_DIR}/targets.stamp
        ${PLUGIN_FILES}
)
