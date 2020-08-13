#
# The interface of pose graph by Ceres And G2O  
# Author: Kang Yifei
# Email: ccyclonel@126.com
#

cmake_minimum_required( VERSION 2.8 )
project(GNC)

# CMake source could not be same with cmake binary.
if(" ${CMAKE_SOURCE_DIR}" STREQUAL " ${CMAKE_BINARY_DIR}")
  message(FATAL_ERROR "
FATAL: In-source builds are not allowed.
       You should create a separate directory for build files.
")
endif()

#set( CMAKE_BUILD_TYPE "Debug" )
set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )
list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )

# G2O
find_package( G2O REQUIRED )

find_package(PythonLibs 2.7)

include_directories( ${G2O_INCLUDE_DIR} 
                     ${PYTHON_INCLUDE_DIRS}
                     "/usr/include/eigen3"
                     "ThirdParty/matplotlib-cpp")

add_executable(${PROJECT_NAME} gnc_curve.cc)
target_link_libraries(${PROJECT_NAME}
g2o_core
g2o_stuff
${PYTHON_LIBRARIES}
)

add_executable(Test_Plot test_python_plot.cc)
target_link_libraries(Test_Plot ${PYTHON_LIBRARIES})

