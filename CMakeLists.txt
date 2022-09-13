cmake_minimum_required(VERSION 3.10)
project(corner_detect)
find_package( OpenCV REQUIRED )
if(MSVC)
    add_definitions(-D_CRT_SECURE_NO_WARNINGS)
endif()

include_directories( ${OpenCV_INCLUDE_DIRS} )
include_directories(${PROJECT_SOURCE_DIR}/src)
include_directories(${PROJECT_SOURCE_DIR}/inc)


set(CMAKE_RUNTIME_OUTPUT_DIRECTORY $<1:${CMAKE_SOURCE_DIR}/prebuilt>)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY  $<1:${CMAKE_SOURCE_DIR}/prebuilt>)
set(CMAKE_CXX_FLAGS "-Wextra -O3  -ffast-math")
add_executable(corner_detect
src/corner_detect.cpp
main.cpp)
target_link_libraries(corner_detect ${OpenCV_LIBS})