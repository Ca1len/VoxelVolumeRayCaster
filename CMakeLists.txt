# cmake_minimum_required(VERSION 3.23)
project(RayCaster)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# file(GLOB_RECURSE headers "headers/*")
file(GLOB_RECURSE sources "src/*")

find_package(OpenCV REQUIRED)

add_executable(RayCaster main.cpp ${sources})

target_include_directories(RayCaster PUBLIC ${OpenCV_INCLUDE_DIRS} headers)
target_link_libraries(RayCaster PUBLIC ${OpenCV_LIBS})
