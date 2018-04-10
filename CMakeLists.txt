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
catkin_package(CATKIN_DEPENDS roscpp
               INCLUDE_DIRS include ${soem_INCLUDE_DIRS}/soem
               LIBRARIES ethercat_interface)

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
file(GLOB HEADERS include/ethercat_interface/*.h src/*.h src/io_interfaces/*.h src/drivers/*.h)

# IO Interfaces
add_library(io_interfaces
#    src/analog_input.cpp
#    src/analog_output.cpp
    src/io_interfaces/io_interfaces.cpp
    src/io_interfaces/read_interface.cpp
    src/io_interfaces/write_interface.cpp
)

# New and improved
add_library(ethercat_drivers
    ${HEADERS}
    src/drivers/ethercat_driver.cpp
    src/drivers/el4xxx.cpp
    src/drivers/el5101.cpp
    src/drivers/tuees030.cpp
)
target_link_libraries(ethercat_drivers io_interfaces ${catkin_LIBRARIES})

# New and improved
add_library(ethercat_interface
    ${HEADERS}
    src/ethercat_interface.cpp
)
target_link_libraries(ethercat_interface ethercat_drivers io_interfaces ${catkin_LIBRARIES})

# #### Test executables ####
# Ethercat interface node
add_executable(ethercat_interface_node test/ethercat_interface_node.cpp)
target_link_libraries(ethercat_interface_node ethercat_interface ${catkin_LIBRARIES})

# Test AO
add_executable(test_ao test/test_ao.cpp)
target_link_libraries(test_ao ethercat_interface ${catkin_LIBRARIES})

# Test encoder
add_executable(test_encoder test/test_encoder.cpp)
target_link_libraries(test_encoder ethercat_interface ${catkin_LIBRARIES})


