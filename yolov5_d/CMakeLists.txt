cmake_minimum_required(VERSION 2.8.3)
project(yolov5_d)

add_definitions(-std=c++11)

option(CUDA_USE_STATIC_CUDA_RUNTIME OFF)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE Debug)

find_package(CUDA REQUIRED)

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

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Ofast -Wfatal-errors -D_MWAITXINTRIN_H_INCLUDED")

# include and link dirs of cuda and tensorrt, you need adapt them if yours are different
# cuda
include_directories(/usr/local/cuda/include)
link_directories(/usr/local/cuda/lib64)
# tensorrt
include_directories(/usr/include/x86_64-linux-gnu/)
link_directories(/usr/lib/x86_64-linux-gnu/)

include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${catkin_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})

link_directories(${OpenCV_LIBRARY_DIRS})

cuda_add_library(myplugins SHARED ${PROJECT_SOURCE_DIR}/src/yololayer.cu)
target_link_libraries(myplugins nvinfer cudart)
###########
## Build ##
###########
add_executable(yolov5_d
  src/yolov5_d.cpp
  src/image_rectify.cpp
)
target_link_libraries(yolov5_d
          ${catkin_LIBRARIES} 
          ${OpenCV_LIBS} 
          )
target_link_libraries(yolov5_d nvinfer)
target_link_libraries(yolov5_d cudart)
target_link_libraries(yolov5_d myplugins)
