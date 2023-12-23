cmake_minimum_required(VERSION 3.0.2)
project(map_render_engine)
set (CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -DDEBUG_MODE")

find_package(OpenCV)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    sensor_msgs
    geometry_msgs
    tf2_sensor_msgs
    tf2_geometry_msgs
    nav_msgs
    cv_bridge
)

catkin_package(
    CATKIN_DEPENDS   
    roscpp
    std_msgs
    sensor_msgs
    geometry_msgs
    tf2_sensor_msgs
    tf2_geometry_msgs
    nav_msgs
    cv_bridge
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

add_executable(render_mapping_screen
    src/render_mapping_screen.cpp
)

target_link_libraries(render_mapping_screen
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)

install(TARGETS render_mapping_screen
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch 
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch 
)

install(DIRECTORY assets/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/assets 
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
