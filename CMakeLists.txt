cmake_minimum_required(VERSION 3.14)
set(PROJECT_VERSION_MAJOR 0)
set(PROJECT_VERSION_MINOR 0)
set(PROJECT_VERSION_PATCH 1)
project(r_vss VERSION ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}.${PROJECT_VERSION_PATCH})

include(r_build.txt)
include(GSTREAMERConfig.cmake)
include(FetchContent)

FetchContent_Declare(
    r_utils
    GIT_REPOSITORY https://github.com/dicroce/r_utils.git
    GIT_TAG        main
    FIND_PACKAGE_ARGS NAMES r_utils
)
FetchContent_MakeAvailable(r_utils)

FetchContent_Declare(
    r_storage
    GIT_REPOSITORY https://github.com/dicroce/r_storage.git
    GIT_TAG        main
    FIND_PACKAGE_ARGS NAMES r_storage
)
FetchContent_MakeAvailable(r_storage)

FetchContent_Declare(
    r_disco
    GIT_REPOSITORY https://github.com/dicroce/r_disco.git
    GIT_TAG        main
    FIND_PACKAGE_ARGS NAMES r_disco
)
FetchContent_MakeAvailable(r_disco)

FetchContent_Declare(
    r_av
    GIT_REPOSITORY https://github.com/dicroce/r_av.git
    GIT_TAG        main
    FIND_PACKAGE_ARGS NAMES r_av
)
FetchContent_MakeAvailable(r_av)

FetchContent_Declare(
    r_pipeline
    GIT_REPOSITORY https://github.com/dicroce/r_pipeline.git
    GIT_TAG        main
    FIND_PACKAGE_ARGS NAMES r_pipeline
)
FetchContent_MakeAvailable(r_pipeline)

FetchContent_Declare(
    r_motion
    GIT_REPOSITORY https://github.com/dicroce/r_motion.git
    GIT_TAG        main
    FIND_PACKAGE_ARGS NAMES r_motion
)
FetchContent_MakeAvailable(r_motion)

FetchContent_Declare(
    r_http
    GIT_REPOSITORY https://github.com/dicroce/r_http.git
    GIT_TAG        main
    FIND_PACKAGE_ARGS NAMES r_http
)
FetchContent_MakeAvailable(r_http)

#find_package(PkgConfig)
#pkg_search_module(gtk+3 REQUIRED IMPORTED_TARGET gtk+-3.0)
#pkg_search_module(gtk+3 REQUIRED IMPORTED_TARGET glib-2.0)

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
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(
    r_vss
    r_motion
    r_pipeline
    r_av
    r_disco
    r_storage
    r_http
    r_utils
    ${GST_LIBS}
    gtk-3
    glib-2.0
)

#add_subdirectory(ut)

configure_dev_package("r_vss" "A C++ library that hosts a video recoder and motion engine.")
