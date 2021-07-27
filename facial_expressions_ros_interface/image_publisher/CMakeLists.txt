cmake_minimum_required(VERSION 3.5)
project(image_publisher)

set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wall -g -O3")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -g -O3")

find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        cv_bridge
        image_transport
        sensor_msgs
        )

find_package(OpenCV REQUIRED)
find_package( Boost )

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

add_executable(image_publisher
        src/main.cpp
        src/image_publisher_node.cpp
        )

add_dependencies(image_publisher
        ${catkin_EXPORTED_TARGETS}
        )

target_include_directories(image_publisher PUBLIC ${OpenCV_INCLUDE_DIRS})

target_link_libraries(image_publisher
        ${Boost_LIBRARIES}
        ${catkin_LIBRARIES}
        ${OpenCV_LIBS}
        )




