#!/bin/sh
rosparam load `rospack find itomp_cio_planner`/config/params.yaml move_itomp
rosparam load `rospack find human_moveit_generated`/config/kinematics.yaml itomp_planner
rosparam set /move_itomp/planning_plugin "itomp_cio_planner/ItompPlanner"
