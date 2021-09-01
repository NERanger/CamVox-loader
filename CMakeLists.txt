cmake_minimum_required(VERSION 3.1)
project(camvox_loader)

set(CMAKE_BUILD_TYPE Release)

set(CMAKE_CXX_FLAGS "-std=c++17 -Wall")
set(CMAKE_CXX_FLAGS_RELEASE  "-std=c++17 -O3")

find_package(OpenCV 3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem system)
find_package(Eigen3 REQUIRED)
find_package(Threads REQUIRED)

include_directories(
    ${OpenCV_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
)

add_library(
    camvox_loader STATIC
        camvox_loader.cc
)
target_link_libraries(
    camvox_loader
        ${OpenCV_LIBS}
        ${Boost_FILESYSTEM_LIBRARY_RELEASE}
        ${Boost_SYSTEM_LIBRARY_RELEASE}
        Eigen3::Eigen
        Threads::Threads
)

add_executable(load_camvox example.cc)
target_link_libraries(
    load_camvox 
        camvox_loader
)