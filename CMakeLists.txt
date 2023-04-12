cmake_minimum_required(VERSION 3.17)
project(image_to_cad)

set(CMAKE_CXX_STANDARD 17)

include(cmake/FetchOpenCV.cmake)
include(cmake/FetchBoost.cmake)
include(cmake/FetchArgParse.cmake)
set(Irit_FIND_REQUIRED TRUE)
include(cmake/FindIrit.cmake)

set(image_to_cad_CUSTOM_COMPILE_DEFINITIONS "" CACHE STRING "Custom compile definitions for image_to_cad")

add_executable(image_to_cad main.cpp preprocess.cpp voronoi.cpp process_graph.cpp irit_exporter.cpp surfaces_generator.cpp)
target_link_libraries(image_to_cad PUBLIC opencv_world argparse ${Irit_LIBRARIES})
target_include_directories(image_to_cad PRIVATE ${OpenCV_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${Irit_INCLUDE_DIRS})
target_compile_definitions(image_to_cad PRIVATE ${Irit_COMPILE_DEFINITIONS} ${image_to_cad_CUSTOM_COMPILE_DEFINITIONS})

add_executable(image_to_cad_preprocess preprocess.cpp preprocess_only_main.cpp)
target_link_libraries(image_to_cad_preprocess PUBLIC opencv_world argparse)
target_include_directories(image_to_cad_preprocess PRIVATE ${OpenCV_INCLUDE_DIRS})
target_compile_definitions(image_to_cad_preprocess PRIVATE ${image_to_cad_CUSTOM_COMPILE_DEFINITIONS})

install(TARGETS image_to_cad COMPONENT image_to_cad RUNTIME DESTINATION .)
install(FILES ${opencv_world_path} ${irit_dynamic_lib_path} COMPONENT image_to_cad DESTINATION .)
install(TARGETS image_to_cad_preprocess COMPONENT image_to_cad_preprocess RUNTIME DESTINATION .)
install(FILES ${opencv_world_path} COMPONENT image_to_cad_preprocess DESTINATION .)
