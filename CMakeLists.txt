cmake_minimum_required(VERSION 2.8.3)
project(frame_tracker)

add_compile_options(-std=c++11)

set(catkin_RUN_PACKAGES  
	eigen_conversions 
	geometry_msgs 
	kdl_conversions 
	kdl_parser 
	nav_msgs 
	roscpp 
	#roslint 
	sensor_msgs 
	std_msgs 
	tf 
	tf_conversions 
	trajectory_msgs 
	urdf 
	visualization_msgs
	interactive_markers
)

set(catkin_BUILD_PACKAGES ${catkin_RUN_PACKAGES} cmake_modules)

find_package(catkin REQUIRED COMPONENTS ${catkin_BUILD_PACKAGES})

find_package(Boost REQUIRED COMPONENTS thread)
find_package(Eigen REQUIRED)

find_package(Ceres REQUIRED)
find_package(ACADO REQUIRED)
message("-- Found Ceres version ${CERES_VERSION}: ${CERES_INCLUDE_DIRS}")

add_definitions(${EIGEN_DEFINITIONS})

find_package(orocos_kdl REQUIRED)

SET(ACADO_DIR /home/bfb-ws/ACADOtoolkit/)
SET(ACADO_INCLUDE_PACKAGES ${ACADO_DIR} ${ACADO_DIR}/acado ${ACADO_DIR}/external_packages)

## Generate services in the 'srv' folder
add_service_files(
	DIRECTORY 
		srv
	FILES
		GetFrameTrackingInfo.srv
)

# Generate added messages and services with any dependencies listed here
generate_messages(
	DEPENDENCIES
		actionlib_msgs
		geometry_msgs
		sensor_msgs
		std_msgs
)

catkin_package(
  CATKIN_DEPENDS ${catkin_RUN_PACKAGES}
  DEPENDS Boost CERES ACADO
  INCLUDE_DIRS include ${ACADO_INCLUDE_PACKAGES}	
  LIBRARIES frame_tracker interactive_marker
)

### BUILD ###
include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN_INCLUDE_DIRS} ${orocos_kdl_INCLUDE_DIRS} ${CERES_INCLUDE_DIRS} ${ACADO_INCLUDE_PACKAGES})

FIND_LIBRARY(libacado libacado_toolkit_s.so ${ACADO_DIR}/build/lib)

add_library(frame_tracker src/frame_tracker.cpp)
add_dependencies(frame_tracker ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(frame_tracker ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES} ${CERES_LIBRARIES} ${libacado}) 

add_library(interactive_marker src/interactive_marker.cpp)
add_dependencies(interactive_marker ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(interactive_marker ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES} ${CERES_LIBRARIES} ${libacado})

add_executable(frame_tracker_node src/frame_tracker_node.cpp)
add_dependencies(frame_tracker_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(frame_tracker_node frame_tracker ${catkin_LIBRARIES} )

add_executable(interactive_marker_node src/interactive_marker_node.cpp)
add_dependencies(interactive_marker_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(interactive_marker_node interactive_marker ${catkin_LIBRARIES} )

### INSTALL ###
install(TARGETS frame_tracker interactive_marker
 ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
 LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
 RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
