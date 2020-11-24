cmake_minimum_required(VERSION 2.8.3)
project(ur10e_collision_free_demo)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  tesseract_monitoring
  tesseract_rosutils
  octomap_ros
  pcl_conversions
)

find_package(trajopt REQUIRED)
find_package(trajopt_ifopt REQUIRED)
find_package(trajopt_sqp REQUIRED)
find_package(tesseract REQUIRED)
find_package(tesseract_motion_planners REQUIRED)
find_package(tesseract_common REQUIRED)
find_package(PCL REQUIRED COMPONENTS core features filters io segmentation surface)

catkin_package()



add_library(${PROJECT_NAME}_ur10e_collision_free_demo src/ur10e_collision_free_demo.cpp)
target_link_libraries(${PROJECT_NAME}_ur10e_collision_free_demo tesseract::tesseract tesseract::tesseract_motion_planners_trajopt ${catkin_LIBRARIES}
    trajopt::trajopt_ifopt trajopt::trajopt_sqp)
tesseract_target_compile_options(${PROJECT_NAME}_ur10e_collision_free_demo PUBLIC)
tesseract_clang_tidy(${PROJECT_NAME}_ur10e_collision_free_demo)
target_include_directories(${PROJECT_NAME}_ur10e_collision_free_demo PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME}_ur10e_collision_free_demo SYSTEM PUBLIC
    ${catkin_INCLUDE_DIRS})

add_executable(${PROJECT_NAME}_ur10e_collision_free_demo_node src/ur10e_collision_free_demo_node.cpp)
target_link_libraries(${PROJECT_NAME}_ur10e_collision_free_demo_node ${PROJECT_NAME}_ur10e_collision_free_demo ${catkin_LIBRARIES})
tesseract_target_compile_options(${PROJECT_NAME}_ur10e_collision_free_demo_node )
tesseract_clang_tidy(${PROJECT_NAME}_ur10e_collision_free_demo_node)
target_include_directories(${PROJECT_NAME}_ur10e_collision_free_demo_node PRIVATE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME}_ur10e_collision_free_demo_node SYSTEM PRIVATE
    ${catkin_INCLUDE_DIRS})


