cmake_minimum_required(VERSION 3.5)
cmake_policy(SET CMP0077 NEW)
project(realsense_post_processing)

set(CMAKE_BUILD_TYPE Debug) #set(CMAKE_BUILD_TYPE Release)

set(PROJECT_LIB ${PROJECT_NAME}_lib)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
# find_package(Eigen3 REQUIRED)
# find_package(yaml-cpp REQUIRED)

# Include directories
include_directories(
	include
	${CMAKE_CURRENT_SOURCE_DIR}
	${PCL_INCLUDE_DIRS}
	${YAML_CPP_INCLUDE_DIRS}
	# ${EIGEN3_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS}) # ${YAML_CPP_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Source files
set(src_files 
	src/realsense_processor_node.cpp
)

set(dependencies
	rclcpp
	std_msgs
	sensor_msgs
    PCL
	pcl_conversions
    pcl_ros
	# tf2_eigen
    # yaml-cpp
)

# Add library
add_library(${PROJECT_LIB} STATIC ${src_files})
target_link_libraries(${PROJECT_LIB} ${PCL_LIBRARIES}) # ${YAML_CPP_LIBRARIES})
ament_target_dependencies(${PROJECT_LIB} ${dependencies})

install(TARGETS
  ${PROJECT_LIB}
  DESTINATION lib/${PROJECT_NAME})

# Add executable
add_executable(${PROJECT_NAME} src/main.cpp)
target_link_libraries(${PROJECT_NAME} ${PROJECT_LIB} ${PCL_LIBRARIES}) # ${YAML_CPP_LIBRARIES})
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

# Installation
install(TARGETS
  ${PROJECT_NAME}
  DESTINATION 
  lib/${PROJECT_NAME})

install(DIRECTORY
	launch params
	DESTINATION
	share/${PROJECT_NAME}
)

ament_package()
