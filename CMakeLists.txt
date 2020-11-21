cmake_minimum_required(VERSION 3.5)
project(myproject)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
find_package(CGAL REQUIRED)
include(${CGAL_USE_FILE})

set(SOURCE_FILES main.cpp Resovle_NeedleTriangle_face.hpp)
add_executable(myproject ${SOURCE_FILES})
target_link_libraries(myproject ${CGAL_LIBS})