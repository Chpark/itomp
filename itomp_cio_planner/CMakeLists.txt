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

FIND_PACKAGE( OpenMP REQUIRED)
if(OPENMP_FOUND)
message("OPENMP FOUND")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

include(${PROJECT_SOURCE_DIR}/dlib/dlib/cmake)

rosbuild_add_library(itomp
src/planner/itomp_planner_node.cpp
src/planner/planning_info_manager.cpp
src/model/itomp_robot_model.cpp
src/model/treefksolverjointposaxis.cpp
src/model/treefksolverjointposaxis_partial.cpp
src/trajectory/trajectory.cpp
src/trajectory/full_trajectory.cpp
src/trajectory/parameter_trajectory.cpp
src/trajectory/trajectory_factory.cpp
#src/cost/smoothness_cost.cpp
#src/cost/trajectory_cost_accumulator.cpp
src/cost/trajectory_cost.cpp
src/cost/trajectory_cost_manager.cpp
src/contact/contact_point.cpp
src/contact/ground_manager.cpp
src/contact/contact_force_solver.cpp
src/visualization/visualization_manager.cpp
src/util/min_jerk_trajectory.cpp
src/util/planning_parameters.cpp
src/util/point_to_triangle_projection.cpp
src/util/joint_state_util.cpp
src/optimization/itomp_optimizer.cpp
src/optimization/new_eval_manager.cpp
src/optimization/improvement_manager.cpp
src/optimization/improvement_manager_nlp.cpp
#src/optimization/improvement_manager_chomp.cpp
src/optimization/rollout.cpp
)
target_link_libraries(itomp dlib)
set(LIBRARY_INPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
target_link_libraries(itomp ${LIBRARY_INPUT_PATH}/liblevmar.a)
target_link_libraries(itomp ${LIBRARY_INPUT_PATH}/libf2c.a)
target_link_libraries(itomp ${LIBRARY_INPUT_PATH}/liblapack.a)
target_link_libraries(itomp ${LIBRARY_INPUT_PATH}/libblas.a)
target_link_libraries(itomp ${LIBRARY_INPUT_PATH}/librbdl.a)
target_link_libraries(itomp ${LIBRARY_INPUT_PATH}/librbdl_urdfreader.a)

#rosbuild_add_executable(itomp_planner_node src/main.cpp)
#rosbuild_link_boost(itomp_planner_node thread)
#target_link_libraries(itomp_planner_node itomp levmar f2c lapack blas)

set(LIBRARY_NAME itomp_planner_plugin)
rosbuild_add_library(${LIBRARY_NAME} src/itomp_plugin.cpp src/itomp_planning_interface.cpp)
rosbuild_link_boost(${LIBRARY_NAME} thread)

#target_link_libraries(${LIBRARY_NAME} itomp levmar f2c lapack blas)
target_link_libraries(${LIBRARY_NAME} itomp)
