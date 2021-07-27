cmake_minimum_required(VERSION 3.5)
project(model_interface CUDA CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CUDA_STANDARD_REQUIRED ON)

# Optimize the CPU:
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


# Thrust for parallel programing:
set(THRUST_COMPONENTS CPP OMP TBB)
set(THRUST_OPTIONAL_COMPONENTS CUDA)
add_subdirectory(thrust)




include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${CMAKE_CUDA_TOOLKIT_INCLUDE_DIRECTORIES}
)

add_executable(model_interface
        src/main.cpp
        src/model_interface_node.cpp
        src/face_detector.cpp
        src/consumer.cpp
        src/face_classifier.cpp )

thrust_create_target(
        ThrustTBB
        HOST
        TBB
        DEVICE
        CUDA
        IGNORE_CUB_VERSION_CHECK
)        

add_dependencies(model_interface
        ${catkin_EXPORTED_TARGETS}
        )

target_include_directories(model_interface PUBLIC ${OpenCV_INCLUDE_DIRS})

target_link_libraries(model_interface
        ${Boost_LIBRARIES}
        ${catkin_LIBRARIES}
        ${OpenCV_LIBS}
        ThrustTBB # TBB backend
        )

thrust_update_system_found_flags()


