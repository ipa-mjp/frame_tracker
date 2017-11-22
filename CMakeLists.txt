cmake_minimum_required(VERSION 2.8.3)
project(frame_tracker)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS actionlib_msgs actionlib cob_srvs control_toolbox dynamic_reconfigure geometry_msgs interactive_markers kdl_conversions kdl_parser message_generation roscpp roslint sensor_msgs std_msgs std_srvs tf visualization_msgs)

find_package(Boost REQUIRED COMPONENTS thread)

#find_package(Eigen REQUIRED)
#add_definitions(${EIGEN_DEFINITIONS})

find_package(orocos_kdl REQUIRED)

find_package(Ceres REQUIRED)

find_package(ACADO REQUIRED)

SET(ACADO_DIR /home/bfb-ws/ACADOtoolkit/)
SET(ACADO_INCLUDE_PACKAGES ${ACADO_DIR} ${ACADO_DIR}/acado ${ACADO_DIR}/external_packages)

### Message Generation ###
generate_dynamic_reconfigure_options(cfg/FrameTracker.cfg)

add_action_files(
  DIRECTORY action
  FILES FrameTracking.action
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  actionlib_msgs
)

catkin_package(
  CATKIN_DEPENDS actionlib_msgs actionlib cob_srvs control_toolbox dynamic_reconfigure geometry_msgs interactive_markers kdl_parser message_runtime roscpp sensor_msgs std_msgs std_srvs tf visualization_msgs
  DEPENDS Boost CERES ACADO
  INCLUDE_DIRS include ${ACADO_INCLUDE_PACKAGES}
  LIBRARIES kinematic_computations cob_frame_tracker interactive_frame_target
)

### BUILD ###
FIND_LIBRARY(libacado libacado_toolkit_s.so ${ACADO_DIR}/build/lib)
include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN_INCLUDE_DIRS} ${orocos_kdl_INCLUDE_DIRS} ${CERES_INCLUDE_DIRS} ${ACADO_INCLUDE_PACKAGES})

add_library(kinematic_computations src/kinematic_calculations.cpp)
add_dependencies(kinematic_computations ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(kinematic_computations ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES} ${CERES_LIBRARIES} ${libacado})

add_library(cob_frame_tracker src/cob_frame_tracker.cpp)
add_dependencies(cob_frame_tracker ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(cob_frame_tracker kinematic_computations ${catkin_LIBRARIES} ${CERES_LIBRARIES} ${orocos_kdl_LIBRARIES} ${libacado})

add_library(interactive_frame_target src/interactive_frame_target.cpp)
add_dependencies(interactive_frame_target ${catkin_EXPORTED_TARGETS})
target_link_libraries(interactive_frame_target ${catkin_LIBRARIES} ${libacado})


add_executable(frame_tracker_node src/cob_frame_tracker_node.cpp)
target_link_libraries(frame_tracker_node cob_frame_tracker ${catkin_LIBRARIES} ${CERES_LIBRARIES} ${libacado})

add_executable(interactive_frame_target_node src/interactive_frame_target_node.cpp)
target_link_libraries(interactive_frame_target_node interactive_frame_target ${catkin_LIBRARIES} ${libacado})

add_executable(spacenav_commander src/spacenav_commander.cpp)
add_dependencies(spacenav_commander ${catkin_EXPORTED_TARGETS})
target_link_libraries(spacenav_commander ${catkin_LIBRARIES} ${libacado})

roslint_cpp()

### INSTALL ###
install(TARGETS kinematic_computations cob_frame_tracker frame_tracker_node interactive_frame_target interactive_frame_target_node spacenav_commander
 ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
 LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
 RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
 DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(PROGRAMS scripts/interactive_frame_target.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/scripts
)
