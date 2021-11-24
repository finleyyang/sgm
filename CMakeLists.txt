cmake_minimum_required(VERSION 3.20)
project(sgm)

set(CMAKE_CXX_STANDARD 14)
find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(CGAL)
include(${CGAL_USE_FILE})


include_directories("/usr/include/eigen3/")

add_executable(sgm main.cpp Camera.h Camera.cpp Image.h Image.cpp sgm.h sgm.cpp  Plane.cpp Plane.h Ray.cpp Ray.h common.h common.cpp)
target_link_libraries(sgm ${OpenCV_LIBS})