cmake_minimum_required(VERSION 3.17)
project(test_opencv)

set(CMAKE_CXX_STANDARD 17)

set(BUILD_opencv_world ON)
set(BUILD_PERF_TESTS OFF)
set(BUILD_TESTS OFF)
set(BUILD_WITH_STATIC_CRT OFF)
set(WITH_CUDA OFF)
set(WITH_CUFFT OFF)
set(BUILD_EXAMPLES OFF)
set(BUILD_ANDROID_EXAMPLES OFF)
set(BUILD_ANDROID_PROJECTS OFF)
set(INSTALL_ANDROID_EXAMPLES OFF)
set(BUILD_opencv_apps OFF)
set(BUILD_JAVA OFF)
set(BUILD_opencv_python2 OFF)
set(BUILD_opencv_python3 OFF)
set(WITH_PROTOBUF OFF)
set(BUILD_PROTOBUF OFF)
set(OPENCV_DNN_OPENCL OFF)
include(FetchContent)
Set(FETCHCONTENT_QUIET FALSE)

FetchContent_Declare(
    opencv
    GIT_REPOSITORY https://github.com/opencv/opencv.git
    GIT_TAG 4.6.0
    GIT_PROGRESS TRUE
    GIT_SHALLOW TRUE
)

# FetchContent_GetProperties(opencv)
# if (NOT opencv_POPULATED)
# FetchContent_Populate(opencv)
# endif ()
FetchContent_MakeAvailable(opencv)

macro(collect_include_dirs result curdir)
    file(GLOB children ${curdir}/*)
    set(dirlist "")

    foreach(child ${children})
        if(IS_DIRECTORY ${child})
            list(APPEND dirlist ${child}/include)
        endif()
    endforeach()

    set(${result} ${dirlist})
endmacro()

collect_include_dirs(OPENCV_INCLUDE_DIRS ${OpenCV_SOURCE_DIR}/modules)
list(APPEND OPENCV_INCLUDE_DIRS ${CMAKE_CURRENT_BINARY_DIR} ${OpenCV_SOURCE_DIR}/include)

set(BOOST_ENABLE_CMAKE ON)
FetchContent_Declare(
    boostorg
    GIT_REPOSITORY https://github.com/boostorg/boost.git
    GIT_TAG boost-1.80.0
    GIT_PROGRESS TRUE
    GIT_SHALLOW TRUE
)

# FetchContent_MakeAvailable(boostorg)
# find_package(Boost 1.80.0)
# # FetchContent_GetProperties(boostorg)
# # if (NOT boostorg_POPULATED)
# #    FetchContent_Populate(boostorg)
# #    add_subdirectory(${boostorg_SOURCE_DIR} ${boostorg_BINARY_DIR} EXCLUDE_FROM_ALL)
# # endif ()
FetchContent_GetProperties(boostorg)

if(NOT boostorg_POPULATED)
    FetchContent_Populate(boostorg)
    add_subdirectory(${boostorg_SOURCE_DIR} ${boostorg_BINARY_DIR})
endif()

collect_include_dirs(BOOST_INCLUDE_DIRS ${boostorg_SOURCE_DIR}/libs)

add_executable(image_to_cad main.cpp preprocess.cpp voronoi.cpp process_graph.cpp)
target_link_libraries(image_to_cad PRIVATE opencv_world)
target_include_directories(image_to_cad PRIVATE ${OPENCV_INCLUDE_DIRS} ${BOOST_INCLUDE_DIRS})

add_executable(test_opencv test.cpp)
target_link_libraries(test_opencv PRIVATE opencv_world)
target_include_directories(test_opencv PRIVATE ${OPENCV_INCLUDE_DIRS} ${BOOST_INCLUDE_DIRS})
