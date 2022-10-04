include(cmake/utils.cmake)

if(NOT DEFINED OpenCV_DIR AND NOT $ENV{OpenCV_DIR} STREQUAL "")
	set(OpenCV_DIR $ENV{OpenCV_DIR})
endif()

if(OpenCV_DIR)
	cmake_path(IS_PREFIX CMAKE_BINARY_DIR ${OpenCV_DIR} is_fetched)

	if(NOT is_fetched)
		find_package(OpenCV 4.6.0 EXACT)
	endif()
endif()

if(NOT OpenCV_FOUND OR is_fetched)
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
	FetchContent_Declare(
		OpenCV
		GIT_REPOSITORY https://github.com/opencv/opencv.git
		GIT_TAG 4.6.0
		GIT_PROGRESS TRUE
		GIT_SHALLOW TRUE
	)

	FetchContent_MakeAvailableExclude(OpenCV)

	collect_include_dirs(OpenCV_INCLUDE_DIRS ${OpenCV_SOURCE_DIR}/modules)
	list(APPEND OpenCV_INCLUDE_DIRS ${CMAKE_CURRENT_BINARY_DIR} ${OpenCV_SOURCE_DIR}/include)
	set(opencv_world_path $<TARGET_FILE:opencv_world>)
else()
	get_property(opencv_world_path TARGET opencv_world PROPERTY LOCATION_${CMAKE_BUILD_TYPE})
endif()
