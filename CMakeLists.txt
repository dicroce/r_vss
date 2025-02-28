cmake_minimum_required(VERSION 3.14)
set(PROJECT_VERSION_MAJOR 0)
set(PROJECT_VERSION_MINOR 0)
set(PROJECT_VERSION_PATCH 1)
project(r_vss VERSION ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}.${PROJECT_VERSION_PATCH})

include(r_build.txt)
include(FetchContent)

FetchContent_Declare(
    r_utils
    GIT_REPOSITORY https://github.com/dicroce/r_utils.git
    GIT_TAG        main
)
FetchContent_MakeAvailable(r_utils)

FetchContent_Declare(
    r_storage
    GIT_REPOSITORY https://github.com/dicroce/r_storage.git
    GIT_TAG        main
)
FetchContent_MakeAvailable(r_storage)

FetchContent_Declare(
    r_disco
    GIT_REPOSITORY https://github.com/dicroce/r_disco.git
    GIT_TAG        main
)
FetchContent_MakeAvailable(r_disco)

FetchContent_Declare(
    r_av
    GIT_REPOSITORY https://github.com/dicroce/r_av.git
    GIT_TAG        main
)
FetchContent_MakeAvailable(r_av)

FetchContent_Declare(
    r_pipeline
    GIT_REPOSITORY https://github.com/dicroce/r_pipeline.git
    GIT_TAG        main
)
FetchContent_MakeAvailable(r_pipeline)

FetchContent_Declare(
    r_motion
    GIT_REPOSITORY https://github.com/dicroce/r_motion.git
    GIT_TAG        main
)
FetchContent_MakeAvailable(r_motion)

find_package(PkgConfig)
pkg_search_module(gstreamer REQUIRED IMPORTED_TARGET gstreamer-1.0)
pkg_search_module(gstreamer-sdp REQUIRED IMPORTED_TARGET gstreamer-sdp-1.0)
pkg_search_module(gstreamer-app REQUIRED IMPORTED_TARGET gstreamer-app-1.0)
pkg_search_module(gstreamer-video REQUIRED IMPORTED_TARGET gstreamer-video-1.0)
pkg_search_module(gstreamer-rtsp-server REQUIRED IMPORTED_TARGET gstreamer-rtsp-server-1.0)
pkg_search_module(gstreamer-codecparsers REQUIRED IMPORTED_TARGET gstreamer-codecparsers-1.0)
pkg_search_module(gstreamer-plugins-base REQUIRED IMPORTED_TARGET gstreamer-plugins-base-1.0)
pkg_search_module(gstreamer-plugins-good REQUIRED IMPORTED_TARGET gstreamer-plugins-good-1.0)
pkg_search_module(gstreamer-plugins-bad REQUIRED IMPORTED_TARGET gstreamer-plugins-bad-1.0)

add_library(
    r_vss
    include/r_vss/r_motion_engine.h
    source/r_motion_engine.cpp
    include/r_vss/r_prune.h
    source/r_prune.cpp
    include/r_vss/r_recording_context.h
    source/r_recording_context.cpp
    include/r_vss/r_stream_keeper.h
    source/r_stream_keeper.cpp
    include/r_vss/r_vss_utils.h
    source/r_vss_utils.cpp
    include/r_vss/r_ws.h
    source/r_ws.cpp
)

target_include_directories(
    r_vss PUBLIC
    include
    ${r_utils_SOURCE_DIR}/include
    ${r_storage_SOURCE_DIR}/include
    ${r_disco_SOURCE_DIR}/include
    ${r_av_SOURCE_DIR}/include
    ${r_pipeline_SOURCE_DIR}/include
    ${r_motion_SOURCE_DIR}/include
    PkgConfig::gstreamer
    PkgConfig::gstreamer-sdp
    PkgConfig::gstreamer-app
    PkgConfig::gstreamer-video
)

target_link_libraries(
    r_vss
    r_motion
    r_pipeline
    r_av
    r_disco
    r_storage
    r_utils
    PkgConfig::gstreamer
    PkgConfig::gstreamer-sdp
    PkgConfig::gstreamer-app
    PkgConfig::gstreamer-video
)

#add_subdirectory(ut)