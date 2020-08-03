cmake_minimum_required(VERSION 2.8.3)

## Use C++ 11
add_compile_options(-std=c++11)

project(ethercat_interface)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  soem
  hardware_interface
  controller_manager)

## Declare a catkin package
catkin_package(CATKIN_DEPENDS roscpp
               INCLUDE_DIRS include
               LIBRARIES ${PROJECT_NAME})

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Ethercat interface library
add_library(${PROJECT_NAME}
    ${HEADERS}

    # Driver implementations
    src/drivers/el200x.h
    src/drivers/el4xxx.cpp
    src/drivers/el5101.cpp
    src/drivers/tuees030.cpp

    # IO implementations
    src/io/analog_input.h
    src/io/analog_output.h
    src/io/digital_input.h
    src/io/digital_output.h
    src/io/encoder.h

    src/interface.cpp
    src/driver.cpp
    src/io.cpp
    src/input.cpp
    src/output.cpp
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

# #### Test executables ####
# Ethercat interface node
add_executable(ethercat_interface_node test/ethercat_interface_node.cpp)
target_link_libraries(ethercat_interface_node ${PROJECT_NAME} ${catkin_LIBRARIES})

# List interfaces
add_executable(list_interfaces test/list_interfaces.cpp)
target_link_libraries(list_interfaces ${PROJECT_NAME} ${catkin_LIBRARIES})

# Test AI
add_executable(test_ai test/test_ai.cpp)
target_link_libraries(test_ai ${PROJECT_NAME} ${catkin_LIBRARIES})

# Test AO
add_executable(test_ao test/test_ao.cpp)
target_link_libraries(test_ao ${PROJECT_NAME} ${catkin_LIBRARIES})

# Test DO
add_executable(test_do test/test_do.cpp)
target_link_libraries(test_do ${PROJECT_NAME} ${catkin_LIBRARIES})

# Test encoder
add_executable(test_encoder test/test_encoder.cpp)
target_link_libraries(test_encoder ${PROJECT_NAME} ${catkin_LIBRARIES})
