cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)
#set(ROS_BUILD_TYPE Release)
set(ROS_BUILD_TYPE Debug)

rosbuild_init()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-ignored-qualifiers")

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

file(GLOB_RECURSE MOVE_ITOMP_HEADER_FILES RELATIVE ${PROJECT_SOURCE_DIR} *.h)

rosbuild_add_executable(move_itomp src/move_itomp.cpp)
rosbuild_add_executable(cio_test src/cio_test.cpp)
rosbuild_add_executable(apartment 
src/apartment.cpp
${MOVE_ITOMP_HEADER_FILES}
)
rosbuild_add_executable(app_rbprm
src/app_rbprm.cpp
src/move_itomp_util.cpp
src/rbprm_reader.cpp
src/bvh_writer.cpp
${MOVE_ITOMP_HEADER_FILES}
)


# mixamo walking animation visualize
rosbuild_add_executable(walking_rbprm
src/walking_rbprm.cpp
src/move_itomp_util.cpp
src/rbprm_reader.cpp
${MOVE_ITOMP_HEADER_FILES}
)
