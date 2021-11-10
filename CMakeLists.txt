cmake_minimum_required(VERSION 3.10)

set(PROJECT App)
set(VERSION 0.0.1)

project("${PROJECT}" VERSION "${VERSION}")


find_package(PkgConfig REQUIRED)

pkg_check_modules(GST REQUIRED IMPORTED_TARGET
    "gstreamer-1.0"
    "gstreamer-base-1.0"
    "gstreamer-app-1.0"
    "gstreamer-rtp-1.0"
    "gstreamer-audio-1.0"
    "gstreamer-video-1.0"
    "gstreamer-pbutils-1.0"
    "gstreamer-tag-1.0"
    "gstreamer-webrtc-1.0"
    "gstreamer-sdp-1.0"
)

set(TARGET "${PROJECT}")
set(SRC_PATH "${CMAKE_CURRENT_SOURCE_DIR}/src")
file(GLOB SRC_FILES "${SRC_PATH}/**/*.cpp" "${SRC_PATH}/*.cpp")
set(INCLUDE_DIRS 
    "${SRC_PATH}" 
)

add_executable("${TARGET}" ${SRC_FILES})
target_link_libraries("${TARGET}" PUBLIC PkgConfig::GST)
target_include_directories("${TARGET}" PRIVATE ${INCLUDE_DIRS})
target_include_directories("${TARGET}" PUBLIC ${GST_INCLUDE_DIRS})

add_subdirectory("plugins")