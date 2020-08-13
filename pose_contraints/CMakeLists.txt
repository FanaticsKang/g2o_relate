#
# The interface of pose graph by Ceres And G2O  
# Author: Kang Yifei
# Email: ccyclonel@126.com
#

cmake_minimum_required( VERSION 2.8 )
project( My_New_Project )

# CMake source could not be same with cmake binary.
if(" ${CMAKE_SOURCE_DIR}" STREQUAL " ${CMAKE_BINARY_DIR}")
  message(FATAL_ERROR "
FATAL: In-source builds are not allowed.
       You should create a separate directory for build files.
")
endif()

#set( CMAKE_BUILD_TYPE "Debug" )
set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )

# # OpenMP
# FIND_PACKAGE( OpenMP QUIET)
# if(OPENMP_FOUND)
#   message("OPENMP FOUND")
#   message("OPENMP C FLAGS: ${OpenMP_C_FLAGS}")
#   message("OPENMP CXX FLAGS: ${OpenMP_CXX_FLAGS}")
#   set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
#   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
#   set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
# endif()
find_package( Sophus REQUIRED )
find_package( g2o REQUIRED )
include_directories(${Sophus_INCLUDE_DIRS}
                    "/usr/include/eigen3" )

add_executable(pose_constraints pose_constraints.cc)
target_link_libraries(pose_constraints ${Sophus_LIBRARIES} )

add_executable(g2o_pose_constraints pose_constraints_g2o.cc)
target_link_libraries(g2o_pose_constraints g2o_core g2o_stuff g2o_types_slam3d )
