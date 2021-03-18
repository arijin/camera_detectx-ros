cmake_minimum_required(VERSION 2.8.3)
project(darknet_d)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}  -std=c++0x")

find_package(catkin REQUIRED COMPONENTS
        roscpp
        sensor_msgs
        std_msgs
        cv_bridge
        image_transport
        )

find_package(OpenCV REQUIRED)
catkin_package(
        CATKIN_DEPENDS roscpp sensor_msgs std_msgs cv_bridge image_transport
)

set(CMAKE_CXX_FLAGS "-O2 -Wall ${CMAKE_CXX_FLAGS}")

include_directories(include)
include_directories(${catkin_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})

link_directories(${OpenCV_LIBRARY_DIRS})

###########
## Build ##
###########
add_executable(darknet_d
  src/darknet_d.cpp
  src/image_rectify.cpp
)
target_link_libraries(darknet_d
          ${catkin_LIBRARIES} 
          ${OpenCV_LIBS} 
          ${PROJECT_SOURCE_DIR}/lib/libdarknet.so
          )
