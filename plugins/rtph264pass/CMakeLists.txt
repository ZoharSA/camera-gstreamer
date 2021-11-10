set(TARGET rtph264pass)
set(VERSION 1.0.0)

file(GLOB "${TARGET}_SRC_FILES" "*.c")
add_library("${TARGET}" SHARED ${${TARGET}_SRC_FILES})
target_compile_definitions("${TARGET}" PRIVATE VERSION="${VERSION}")
target_include_directories("${TARGET}" PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
target_link_libraries("${TARGET}" PRIVATE PkgConfig::GST)
install(TARGETS "${TARGET}" LIBRARY DESTINATION "/usr/lib/x86_64-linux-gnu/gstreamer-1.0/")