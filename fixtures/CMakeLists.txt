cmake_minimum_required(VERSION 3.16)
project(fast_lio_fixtures CXX)

# FAST-LIO itself builds as C++14; match it so the golden data comes from the same code paths.
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(FAST_LIO_DIR "" CACHE PATH "Checkout of hku-mars/FAST_LIO at the pinned commit")
if(NOT EXISTS "${FAST_LIO_DIR}/include/use-ikfom.hpp")
  message(FATAL_ERROR "FAST_LIO_DIR must point at a FAST_LIO checkout")
endif()

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)
find_package(OpenMP REQUIRED)
find_package(Threads REQUIRED)

add_library(ikd_tree STATIC "${FAST_LIO_DIR}/include/ikd-Tree/ikd_Tree.cpp")
target_include_directories(ikd_tree PUBLIC "${FAST_LIO_DIR}/include" ${PCL_INCLUDE_DIRS})
target_link_libraries(ikd_tree PUBLIC Eigen3::Eigen Threads::Threads)

add_executable(smoke src/smoke.cpp)
target_link_libraries(smoke PRIVATE ikd_tree OpenMP::OpenMP_CXX)
