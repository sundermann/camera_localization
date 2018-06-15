cmake_minimum_required(VERSION 2.8.3)
add_definitions(-std=c++11)
project(fub_visual_odometry)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        geometry_msgs
        cv_bridge
	    image_transport
        image_geometry
		dynamic_reconfigure
        tf2
        tf2_ros
		tf2_geometry_msgs
)

find_package(OpenCV REQUIRED)


generate_dynamic_reconfigure_options(
        cfg/VisualOdometry.cfg
)

catkin_package(
#	LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS std_msgs geometry_msgs
)

include_directories(
        ${catkin_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/VisualOdometry.cpp src/VisualOdometryNode.cpp src/VisualOdometry.h)
add_dependencies(${PROJECT_NAME}_node ${PROJECT_NAME}_gencfg)
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})


install(TARGETS ${PROJECT_NAME}_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch cfg params
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)
