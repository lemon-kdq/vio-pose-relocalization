cmake_minimum_required(VERSION 2.8)
project(VioPoseRelocalization)

find_package(Eigen3)

include_directories(include ${Eigen_INCLUDE_DIR})

add_executable(test_eigen app/test_eigen.cpp)
target_link_libraries(test_eigen ${Eigen3_LIBRARY})


add_executable(test_relocalizer app/test-relocalizer.cpp)
target_link_libraries(test_relocalizer ${Eigen3_LIBRARY})