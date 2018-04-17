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
               LIBRARIES ${PROJECT_NAME})

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
file(GLOB HEADERS include/*/*.h src/*.h src/*/*.h)

# IO Interfaces
add_library(${PROJECT_NAME}
    ${HEADERS}
    src/drivers/el4xxx.cpp
    src/drivers/el5101.cpp
    src/drivers/ethercat_driver.cpp
    src/drivers/tuees030.cpp
    src/io_interfaces/io_interfaces.cpp
    src/io_interfaces/read_interface.cpp
    src/io_interfaces/write_interface.cpp
    src/ethercat_interface.cpp
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

# #### Test executables ####
# Ethercat interface node
add_executable(ethercat_interface_node test/ethercat_interface_node.cpp)
target_link_libraries(ethercat_interface_node ${PROJECT_NAME} ${catkin_LIBRARIES})

# Test AO
add_executable(test_ao test/test_ao.cpp)
target_link_libraries(test_ao ${PROJECT_NAME} ${catkin_LIBRARIES})

# Test DO
add_executable(test_do test/test_do.cpp)
target_link_libraries(test_do ${PROJECT_NAME} ${catkin_LIBRARIES})

# Test encoder
add_executable(test_encoder test/test_encoder.cpp)
target_link_libraries(test_encoder ${PROJECT_NAME} ${catkin_LIBRARIES})
