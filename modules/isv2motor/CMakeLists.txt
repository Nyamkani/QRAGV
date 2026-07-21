cmake_minimum_required(VERSION 3.0.0)
project(isv2motor)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

# set(GLOBAL_HEADER_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../hw_interface/include")

set(HARDWARE_INTERFACE_DIR "/home/jetson/workspace/QRAGV/driving/modules/hw_interfece")

SET(CMAKE_CXX_FLAGS "-pthread -I/usr/local/include -L/usr/local/lib")


file(GLOB_RECURSE sources CONFIGURE_DEPENDS
      ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp
      ${CMAKE_CURRENT_SOURCE_DIR}/src/*.c
)

# make excutable
add_executable(${PROJECT_NAME}_test 
                    ${sources}
                    )

target_include_directories(${PROJECT_NAME}_test PUBLIC 
  ${CMAKE_CURRENT_SOURCE_DIR}/include 
  ${HARDWARE_INTERFACE_DIR}/include

)

# make library
add_library(${PROJECT_NAME} 
                    ${sources}
                    )


target_include_directories(${PROJECT_NAME} PUBLIC 
  ${CMAKE_CURRENT_SOURCE_DIR}/include 
  ${HARDWARE_INTERFACE_DIR}/include
  
)




