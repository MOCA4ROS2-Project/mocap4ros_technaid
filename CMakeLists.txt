project(mocap4ros2_technaid)

cmake_minimum_required(VERSION 3.5)

set(CMAKE_CXX_STANDARD 17)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")


find_package(Threads REQUIRED)
find_package(mcs_technaid REQUIRED)

set(dependencies 
  roscpp
  mocap_control
  mocap_msgs
  sensor_msgs
)

find_package(catkin REQUIRED COMPONENTS ${dependencies})
catkin_package(CATKIN_DEPENDS ${dependencies})




include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${mcs_technaid_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} STATIC
  src/mocap4ros2_technaid/MCSNode.cpp
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} mcs_technaid)

add_executable(mcs_technaid_main
  src/mcs_technaid_main.cpp
)
target_link_libraries(mcs_technaid_main ${catkin_LIBRARIES} ${PROJECT_NAME})



install(TARGETS
  ${PROJECT_NAME}
  mcs_technaid_main
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
