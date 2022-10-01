include(cmake/utils.cmake)

if(DEFINED Boost_DIR OR DEFINED $ENV{BOOST_ROOT} OR DEFINED $ENV{Boost_DIR})
    if(DEFINED Boost_DIR AND NOT ${Boost_DIR} STREQUAL "Boost_DIR-NOTFOUND")
        cmake_path(IS_PREFIX CMAKE_BINARY_DIR ${OpenCV_DIR} is_fetched)
    else()
        set(is_fetched FALSE)
    endif()

    if(NOT is_fetched)
        find_package(Boost 1.80.0 EXACT)
    endif()
endif()

if(NOT Boost_FOUND)
    set(BOOST_ENABLE_CMAKE ON)
    FetchContent_Declare(
        Boost
        GIT_REPOSITORY https://github.com/NadavT/shallow_boost.git
        GIT_TAG boost-1.80.0
        GIT_PROGRESS TRUE
        GIT_SHALLOW TRUE
    )
    FetchContent_GetProperties(Boost)

    if(NOT Boost_POPULATED)
        FetchContent_Populate(Boost)
    endif()

    collect_include_dirs(Boost_INCLUDE_DIRS ${boostorg_SOURCE_DIR}/libs)
endif()
