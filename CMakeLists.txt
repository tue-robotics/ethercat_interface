cmake_minimum_required(VERSION 2.8.3)

## Use C++ 11
set (CMAKE_CXX_STANDARD 11)

project(ethercat_interface)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  soem
  hardware_interface
  controller_manager)

## Declare a catkin package
catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ## The following work around allows SOEM headers to include other SOEM headers.
  ## SOEM headers assume all headers are installed in a flat directory structure
  ## See https://github.com/smits/soem/issues/4 for more information.
  ## Thanks: https://github.com/ros-industrial/robotiq/blob/a2a77e73370c522fa9865f7a797408ef7831026e/robotiq_ethercat/CMakeLists.txt#L15
  ${soem_INCLUDE_DIRS}/soem
)

# find all headers for qtcreator
file(GLOB HEADERS include/ethercat_interface/*.h src/*.h)

# New and improved
add_library(ethercat_interface
    ${HEADERS}
    src/ethercat_interface.cpp
    src/el4xxx.cpp
    src/io_interfaces.cpp
)
target_link_libraries(ethercat_interface ${catkin_LIBRARIES})

add_executable(ethercat_interface_node src/ethercat_interface_node.cpp)
target_link_libraries(ethercat_interface_node ethercat_interface ${catkin_LIBRARIES})
