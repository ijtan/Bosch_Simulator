cmake_minimum_required(VERSION 3.0.2)
project(simulation_vehicle_interface)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  btc_msgs
)

catkin_package(

 CATKIN_DEPENDS

)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
