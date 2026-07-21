cmake_minimum_required(VERSION 3.0.0)
project(driving_controller)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

set(PGV100_HEADER_PATH "/home/jetson/workspace/QRAGV/driving/modules/pgv100/include")
set(ISV2MOTOR_HEADER_PATH "/home/jetson/workspace/QRAGV/driving/modules/isv2motor/include")
set(HARDWARE_INTERFACE_DIR "/home/jetson/workspace/QRAGV/driving/modules/hw_interfece")





# make excutable
add_executable(${PROJECT_NAME}_test 
                    src/driving_controller_test.cpp  
                    # src/driving_controller.cpp  
                          )

target_include_directories(${PROJECT_NAME}_test PUBLIC 
  ${CMAKE_CURRENT_SOURCE_DIR}/include 
  ${HARDWARE_INTERFACE_DIR}/include
  ${PGV100_HEADER_PATH}
  ${ISV2MOTOR_HEADER_PATH}
  )


target_link_libraries(${PROJECT_NAME}_test
  isv2motor
  pgv100
  )

# make library
add_library(${PROJECT_NAME} src/driving_controller.cpp
                          )

target_include_directories(${PROJECT_NAME} PUBLIC 
  ${CMAKE_CURRENT_SOURCE_DIR}/include 
  ${HARDWARE_INTERFACE_DIR}/include
  ${CMAKE_SOURCE_DIR}/include
  # ${PGV100_HEADER_PATH}
  # ${ISV2MOTOR_HEADER_PATH}
)


