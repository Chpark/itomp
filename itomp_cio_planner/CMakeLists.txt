cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)
set(ROS_BUILD_TYPE Release)
#set(ROS_BUILD_TYPE Debug)

rosbuild_init()

FIND_PACKAGE( OpenMP REQUIRED)
if(OPENMP_FOUND)
message("OPENMP FOUND")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

# supress some warnings
SET(CXX_ADDITIONAL_FLAGS "-Wno-ignored-qualifiers")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${CXX_ADDITIONAL_FLAGS}")

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

include(${PROJECT_SOURCE_DIR}/dlib/dlib/cmake)

file(GLOB_RECURSE ITOMP_HEADER_FILES RELATIVE ${PROJECT_SOURCE_DIR} *.h)

rosbuild_add_library(itomp
src/planner/itomp_planner_node.cpp
src/planner/planning_info_manager.cpp
src/model/itomp_robot_model.cpp
src/model/rbdl_model_util.cpp
src/model/rbdl_urdf_reader.cpp
src/model/treefksolverjointposaxis.cpp
src/model/treefksolverjointposaxis_partial.cpp
src/trajectory/trajectory_factory.cpp
src/trajectory/new_trajectory.cpp
src/trajectory/element_trajectory.cpp
src/trajectory/composite_trajectory.cpp
src/trajectory/itomp_trajectory.cpp
src/cost/trajectory_cost.cpp
src/cost/trajectory_cost_manager.cpp
src/contact/contact_point.cpp
src/contact/contact_util.cpp
src/contact/ground_manager.cpp
src/visualization/new_viz_manager.cpp
src/util/min_jerk_trajectory.cpp
src/util/planning_parameters.cpp
src/util/point_to_triangle_projection.cpp
src/util/joint_state_util.cpp
src/util/exponential_map.cpp
src/util/jacobian.cpp
src/optimization/itomp_optimizer.cpp
src/optimization/new_eval_manager.cpp
src/optimization/improvement_manager.cpp
src/optimization/improvement_manager_nlp.cpp
src/optimization/phase_manager.cpp
src/rom/ROM.cpp
src/collision/collision_world_fcl_derivatives.cpp
src/collision/collision_robot_fcl_derivatives.cpp
${ITOMP_HEADER_FILES}
)
target_link_libraries(itomp dlib)
set(LIBRARY_INPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
target_link_libraries(itomp ${LIBRARY_INPUT_PATH}/librbdl.a)

set(LIBRARY_NAME itomp_planner_plugin)
rosbuild_add_library(${LIBRARY_NAME} src/itomp_plugin.cpp src/itomp_planning_interface.cpp)
rosbuild_link_boost(${LIBRARY_NAME} thread)

target_link_libraries(${LIBRARY_NAME} itomp)
