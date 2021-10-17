cmake_minimum_required(VERSION 3.1)
project(camvox_loader)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_STANDARD 17)

if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
  # using GCC
  set(CMAKE_CXX_FLAGS "-Wall -Wextra")
  set(CMAKE_CXX_FLAGS_RELEASE  "-O3")
elseif (CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
  # using Visual Studio C++
  set(CMAKE_CXX_FLAGS_RELEASE  "/O2")
endif()


find_package(OpenCV 4 REQUIRED)
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