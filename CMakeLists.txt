project(mocap4ros2_technaid)

cmake_minimum_required(VERSION 3.5)

find_package(Threads REQUIRED)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(mocap_msgs REQUIRED)
find_package(mcs_technaid REQUIRED)

set(CMAKE_CXX_STANDARD 17)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

set(dependencies
    rclcpp
    rclcpp_action
    rclcpp_lifecycle
    sensor_msgs
    mocap_msgs
)

include_directories(include ${mcs_technaid_INCLUDE_DIRS})

add_library(${PROJECT_NAME} STATIC 
  src/mocap4ros2_technaid/MCSNode.cpp
)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})
target_link_libraries(${PROJECT_NAME} mcs_technaid)
add_executable(mcs_technaid_main
  src/mcs_technaid_main.cpp
)
ament_target_dependencies(mcs_technaid_main ${dependencies})
target_link_libraries(mcs_technaid_main ${PROJECT_NAME})



install(TARGETS
  ${PROJECT_NAME}
  mcs_technaid_main
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  # add_subdirectory(test)
endif()

ament_export_dependencies(${dependencies})

ament_package()
