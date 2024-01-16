cmake_minimum_required(VERSION 2.8.12)
project(tp3)

set(CMAKE_CXX_STANDARD 17)
# Set the default build type if not specified
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS "-Wall -Wextra -O3")
set(CMAKE_CXX_FLAGS_DEBUG "-Wall -Wextra -g")

include_directories(eigen-3.4.0 src)
add_executable(unique_plan
    src/versions/part1.cpp
    src/ransac_part1.cpp
    src/ransac.cpp
    src/color.cpp
    src/obj.cpp)

add_executable(multiple_plan
    src/versions/part2.cpp
    src/ransac.cpp
    src/color.cpp
    src/obj.cpp)