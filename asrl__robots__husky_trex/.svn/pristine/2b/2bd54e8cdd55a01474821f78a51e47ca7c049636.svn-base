cmake_minimum_required(VERSION 2.8)
project(asrl__robots__husky_trex)

find_package(
    catkin REQUIRED COMPONENTS
    std_msgs
    roscpp
    clearpath_base
    asrl__robots__vehicle
    asrl__common__library
    robot_state_publisher
    digi_realport_serial_ethernet
    )

catkin_package(
    #INCLUDE_DIRS include
    #LIBRARIES
    CATKIN_DEPENDS std_msgs roscpp clearpath_base robot_state_publisher asrl__common__library asrl__robots__vehicle digi_realport_serial_ethernet xboxdrv_linux
    # DEPENDS
)

#message("include path: "${catkin_INCLUDE_DIRS})
include_directories(
    include
    ${asrl__robots__vehicle_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
)

add_executable(asrl_husky_trex_node src/asrl_husky_trex.cpp)

target_link_libraries(asrl_husky_trex_node
    ${catkin_LIBRARIES}
)

add_dependencies(asrl_husky_trex_node ${catkin_EXPORTED_TARGETS})
