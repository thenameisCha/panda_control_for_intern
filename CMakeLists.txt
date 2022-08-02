cmake_minimum_required(VERSION 3.0)

project(panda_control_real CXX)

LIST( APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake" )
MESSAGE( STATUS "CMAKE_MODULE_PATH: " ${CMAKE_MODULE_PATH} )

set(Franka_DIR /home/dyros/libfranka/build)
add_definitions(-DDOF=7)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Franka REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(RBDL REQUIRED)

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

include_directories(include
    ${EIGEN3_INCLUDE_DIRS}
    /home/dyros/libfranka/include
)

add_library(${PROJECT_NAME}_shared_libs
    src/controller.cpp
    src/motion_generator.cpp
    src/franka_panda_model.cpp
)

add_executable(panda_position_controller 
    src/main_position_control.cpp
)

add_executable(panda_torque_controller 
    src/main_torque_control.cpp
)

target_link_libraries(${PROJECT_NAME}_shared_libs
    ${RBDL_LIBRARY} 
    Franka::Franka pthread
)

target_link_libraries(panda_position_controller
    ${RBDL_LIBRARY} 
    Franka::Franka pthread
    ${PROJECT_NAME}_shared_libs
)
        
target_link_libraries(panda_torque_controller
    ${RBDL_LIBRARY} 
    Franka::Franka pthread
    ${PROJECT_NAME}_shared_libs
)