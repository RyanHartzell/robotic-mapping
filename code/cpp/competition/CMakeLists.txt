# Can get the latest CMake from https://apt.kitware.com/.
cmake_minimum_required(VERSION 3.8)

project(slam LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17) # WORKS WITH C++17!!! :) Needed by our epipolar code
set(CMAKE_CXX_STANDARD_REQUIRED True)

SET(CMAKE_CXX_FLAGS "-D DEBUG -Wall -Wfatal-errors -Wextra -Wno-unused-variable")

# if(NOT CMAKE_BUILD_TYPE)
#     message(STATUS "No CMAKE_BUILD_TYPE specified, default to Release.")
#     set(CMAKE_BUILD_TYPE "Release")
# endif()

##################### Eigen ######################
find_package(Eigen3 REQUIRED)
message(STATUS "Eigen Version: ${EIGEN3_VERSION_STRING} ${EIGEN3_VERSION}")


##################### OpenCV ######################
# Find installed OpenCV
set(OpenCV_DIR "/usr/local/include/opencv4")
find_package(OpenCV 4 REQUIRED)
include_directories( ${OpenCV_INCLUDE_DIRS} )

message(STATUS "OpenCV Version: " ${OpenCV_VERSION})
# message(STATUS "OpenCV Version: " ${OpenCV_VERSION})
# message(STATUS "OpenCV_INSTALL_PATH = ${OpenCV_INSTALL_PATH}")
# message(STATUS "OpenCV_INCLUDE_DIRS = ${OpenCV_INCLUDE_DIRS}")
# message(STATUS "OpenCV_LIBS = ${OpenCV_LIBS}")


##################### Open3D ######################
# Find Open3D package
find_package(Open3D REQUIRED)


##########################################
include_directories(include)

set(SOURCES
    src/keyframe_selection.cpp
    src/odometry.cpp
    src/competition.cpp
    )
add_executable(competition ${SOURCES})

target_link_libraries(competition 
                      ${OpenCV_LIBS} 
                      Eigen3::Eigen
                      Open3D::Open3D
                      )

##########################################
# # VO_EPIPOLAR
# include_directories(include)

# FILE(GLOB_RECURSE SOURCES2 "src/keyframe_selection.cpp" "src/odometry.cpp" "src/vo_epipolar.cpp" "src/*.h")

# add_executable(vo_epipolar ${SOURCES2})

# target_link_libraries(vo_epipolar
#                       ${OpenCV_LIBS}
#                       Eigen3::Eigen
#                       )