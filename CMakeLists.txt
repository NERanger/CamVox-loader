cmake_minimum_required(VERSION 3.1)
project(camvox_loader)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
  add_definitions(-DNDEBUG)
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
find_package(PCL REQUIRED COMPONENTS common io visualization)
find_package(Boost REQUIRED COMPONENTS filesystem system)
find_package(Eigen3 REQUIRED)
find_package(Threads REQUIRED)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(
    ${PROJECT_SOURCE_DIR}
    ${OpenCV_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
)

add_library(
    dataset_loader STATIC
        camvox_loader/camvox_loader.cc
        stream_loader/StreamLoader.cc
        utils/utils.cc)

target_link_libraries(
    dataset_loader
        ${OpenCV_LIBS}
        ${Boost_FILESYSTEM_LIBRARY_RELEASE}
        ${Boost_SYSTEM_LIBRARY_RELEASE}
        ${PCL_LIBRARIES}
        Eigen3::Eigen
        Threads::Threads
)

add_subdirectory(example)
add_subdirectory(tools)