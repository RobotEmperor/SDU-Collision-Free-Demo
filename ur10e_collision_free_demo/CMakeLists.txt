cmake_minimum_required(VERSION 3.0.2)
project(ur10e_collision_free_demo)

set(CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  tesseract_monitoring
  tesseract_rosutils
  tesseract_planning_server
  octomap_ros
  pcl_conversions
)

set(RW_ROOT "$ENV{RW_ROOT}")
find_package(RobWork REQUIRED PATHS "${RW_ROOT}")

find_package(tesseract_environment REQUIRED)
find_package(tesseract_command_language REQUIRED)
find_package(tesseract_motion_planners REQUIRED)
find_package(tesseract_process_managers REQUIRED)
find_package(tesseract_common REQUIRED)
find_package(trajopt_sqp REQUIRED)
find_package(trajopt_ifopt REQUIRED)
find_package(cmake_common_scripts REQUIRED)
find_package(PCL REQUIRED COMPONENTS core features filters io segmentation surface)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS std_msgs
  DEPENDS system_lib
)
    
add_library(ur10e_collision_free_demo_lib src/ur10e_collision_free_demo.cpp)# src/force_constraint.cpp)
target_link_libraries(ur10e_collision_free_demo_lib ${ROBWORK_LIBRARIES} tesseract::tesseract_environment_core tesseract::tesseract_environment_ofkt tesseract::tesseract_process_managers ${catkin_LIBRARIES})
target_include_directories(ur10e_collision_free_demo_lib PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(ur10e_collision_free_demo_lib SYSTEM PUBLIC
    ${catkin_INCLUDE_DIRS}   ${ROBWORK_INCLUDE_DIR})

add_executable(ur10e_collision_free_demo_node src/ur10e_collision_free_demo_node.cpp)
target_include_directories(ur10e_collision_free_demo_node PRIVATE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(ur10e_collision_free_demo_node SYSTEM PRIVATE
    ${catkin_INCLUDE_DIRS} ${ROBWORK_INCLUDE_DIR})
target_link_libraries(ur10e_collision_free_demo_node ur10e_collision_free_demo_lib ${catkin_LIBRARIES} ${ROBWORK_LIBRARIES})

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
 )

# Mark executables and/or libraries for installation
install(
  TARGETS
  ur10e_collision_free_demo_lib
  ur10e_collision_free_demo_node

  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## OMPL Constraint planning is not available in version less than 1.4
#if(NOT OMPL_VERSION VERSION_LESS "1.4.0")
#  install(
#    TARGETS
#      ${PROJECT_NAME}_glass_upright_ompl_example
#      ${PROJECT_NAME}_glass_upright_ompl_example_node
#    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#  )
#endif()

install(
  DIRECTORY config launch meshes urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(
  DIRECTORY test
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  FILES_MATCHING PATTERN "*.test"
)
